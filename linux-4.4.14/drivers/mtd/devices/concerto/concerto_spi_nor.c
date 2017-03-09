/******************************************************************************/
/* Copyright (c) 2012 Montage Tech - All Rights Reserved                      */
/******************************************************************************/
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/bootinfo.h>

#include "concerto_spi_nor.h"
#include <asm/mach-panther/bootinfo.h>
#include <asm/mach-panther/pdma.h>

#define CONCERTO_SPI_FLASH_NAME     "mt_sf"

#include <linux/mtd/partitions.h>
#include "../../mtdcore.h"

#ifndef CONFIG_MTD_CMDLINE_PARTS

#define SF_MIN_BLOCK_SIZE     64 * 1024

#if !defined(CONFIG_PANTHER_MTD_SIZE)
#define CONFIG_PANTHER_MTD_SIZE 0x1000000
#endif

#if !defined(CONFIG_PANTHER_KERNEL_SIZE)
#define CONFIG_PANTHER_KERNEL_SIZE 2
#endif

#define SNF_BTINT_START    0x00000000                            //0x00000000
#define SNF_BTINT_LENTH    SF_MIN_BLOCK_SIZE                    //0x00020000

#define SNF_CDB_START      (SNF_BTINT_START + SNF_BTINT_LENTH)
#define SNF_CDB_LENTH      SF_MIN_BLOCK_SIZE

#define SNF_LINUX_START    (SNF_CDB_START + SNF_CDB_LENTH)
#define SNF_LINUX_LENTH    (CONFIG_PANTHER_KERNEL_SIZE * 0x100000)

#define SNF_ROOTFS_START   (SNF_LINUX_START + SNF_LINUX_LENTH)
#define SNF_ROOTFS_LENTH   (CONFIG_PANTHER_MTD_SIZE - SNF_ROOTFS_START)

/*
|<-------------- whole firmware partition --------------->|
|<---- linux partition ----->|<---- rootfs partition ---->|
*/
#define SNF_FIRMWARE_START  SNF_LINUX_START
#define SNF_FIRMWARE_LENTH  (SNF_LINUX_LENTH + SNF_ROOTFS_LENTH)


/*=============================================================================+
| Extern Function/Variables                                                    |
+=============================================================================*/
extern int otp_get_boot_type(void);


static struct mtd_partition concerto_spi_nor_partitions[] = 
{
    {
        .name       = "boot_init",
        .size       = SNF_BTINT_LENTH,
        .offset     = SNF_BTINT_START,
    },
    {
        .name       = "cdb",
        .size       = SNF_CDB_LENTH,
        .offset     = SNF_CDB_START,
    },
    {
        .name       = "firmware",
        .size       = SNF_FIRMWARE_LENTH,
        .offset     = SNF_FIRMWARE_START,
    },
    {
        .name       = "linux",
        .size       = SNF_LINUX_LENTH,
        .offset     = SNF_LINUX_START,
    },
    {
        .name       = "rootfs",
        .size       = SNF_ROOTFS_LENTH,
        .offset     = SNF_ROOTFS_START,
    },
};
#endif

#if 0

void flash_printData(char * title, u8* buf, u32 size)
{
    u32 i;

    if(buf == NULL || size == 0)
    {
        return;
    }

    if(title != NULL)
    {
        FLASH_DEBUG("%s:\n",title);
    }
    for(i=0; i<size; i++)
    {
        FLASH_DEBUG("0x%02x ", buf[i]);
        if(i % 16 == 15)
        {
            FLASH_DEBUG("\n");
        }
    }
    FLASH_DEBUG("\n");
}

void printRegister(void)
{
    u32 i;

    for(i = 0; i < 0x50; i = i + 4)
    {
        FLASH_DEBUG("0x%08x: 0x%08x\n", (0xBF010000 + i), REG32((0xBF010000 + i)));
    }
    FLASH_DEBUG("\n");
}
#endif

static int concerto_spi_get_device(struct mtd_info *mtd, int new_state)
{
	struct concerto_sfc_host *host	= mtd->priv;

	spinlock_t *lock = &host->controller->lock;
	wait_queue_head_t *wq = &host->controller->wq;
	DECLARE_WAITQUEUE(wait, current);
retry:
	spin_lock(lock);

	/* Hardware controller shared among independent devices */
	if (!host->controller->active)
		host->controller->active = host;

	if (host->controller->active == host && host->state == FL_READY) {
		host->state = new_state;
		spin_unlock(lock);
		return 0;
	}
	if (new_state == FL_PM_SUSPENDED) {
		if (host->controller->active->state == FL_PM_SUSPENDED) {
			host->state = FL_PM_SUSPENDED;
			spin_unlock(lock);
			return 0;
		}
	}
	set_current_state(TASK_UNINTERRUPTIBLE);
	add_wait_queue(wq, &wait);
	spin_unlock(lock);
	schedule();
	remove_wait_queue(wq, &wait);
	goto retry;
}

static void concerto_spi_release_device(struct mtd_info *mtd)
{
	struct concerto_sfc_host *host	= mtd->priv;

	/* Release the controller and the chip */
	spin_lock(&host->controller->lock);
	host->controller->active = NULL;
	host->state = FL_READY;
	wake_up(&host->controller->wq);
	spin_unlock(&host->controller->lock);
}



static int concerto_spi_nor_getJedecID(struct mtd_info *mtd, u8 *idcode, u32 idlen)
{
	struct concerto_sfc_host *host	= mtd->priv;

    u8 cmd[4];

    cmd[0] = SPI_FLASH_CMD_RD_MAN_ID;
    
    return concerto_spi_read_common(host->spi, cmd, 1, idcode, idlen);
}

int concerto_spi_nor_cmd_erase(struct mtd_info *mtd, u32 offset)
{
    struct concerto_sfc_host *host = mtd->priv;

    int ret;
    u8 cmd[5];

    cmd[0] = host->erase_cmd;

    if(host->byte_mode == 4)
    {
        cmd[1] = offset >> 24;
        cmd[2] = offset >> 16;
        cmd[3] = offset >> 8;
        cmd[4] = offset >> 0;
    }
    else
    {
        cmd[1] = offset >> 16;
        cmd[2] = offset >> 8;
        cmd[3] = offset >> 0;
    }
    
    ret = concerto_spi_cmd_write_enable(host->spi);
    if (ret)
    {
        goto out;
    }

    if(host->byte_mode == 4)
    {
        ret = concerto_spi_cmd_write(host->spi, cmd, 5, NULL, 0, 0);
    }
    else
    {
        ret = concerto_spi_cmd_write(host->spi, cmd, 4, NULL, 0, 0);
    }
    if (ret)
    {
        goto out;
    }

    ret = concerto_spi_cmd_wait_ready(host->spi, SPI_FLASH_PAGE_ERASE_TIMEOUT);
    if (ret)
    {
        goto out;
    }
        
    //FLASH_DEBUG("SF DRV: Successfully erased %zu bytes @ %#x\n", mtd->erasesize, offset);
    
out:
    return ret;
}

int panther_spi_nor_read_enable_secure(struct concerto_sfc_host *host, loff_t from,
                            size_t len, size_t *retlen, u_char *buf, u32 dma_sel)
{
    u8 cmd[6];
    u32 data_len;
    int ret;
    u32 offset = 0;
    u32 start_offset = (from % MAX_NOR_BUFFER_SIZE);
    loff_t start_addr;

    cmd[0] = host->fastr_cmd;

    if (start_offset != 0)
    {
        // Ex: start_offset=0x700 => data_len=0x100;
        data_len = (MAX_NOR_BUFFER_SIZE - start_offset);
        // Ex: start_offset=0x700, from=0x18700 => start_addr=0x18000;
        start_addr = (from - start_offset);

        // Ex: data_len = 0x600, len=0x200 => but only need 0x200, and no need to enter following while loop
        if (len < data_len)
        {
            data_len = len;
        }

        if (host->byte_mode == 4)
        {
            cmd[1] = start_addr >> 24;
            cmd[2] = start_addr >> 16;
            cmd[3] = start_addr >> 8;
            cmd[4] = start_addr >> 0;
            cmd[5] = 0x00;
            ret = concerto_spi_cmd_read(host->spi, cmd, 6, host->buffer, MAX_NOR_BUFFER_SIZE, NULL, dma_sel);
        }
        else
        {
            cmd[1] = start_addr >> 16;
            cmd[2] = start_addr >> 8;
            cmd[3] = start_addr >> 0;
            cmd[4] = 0x00;
            ret = concerto_spi_cmd_read(host->spi, cmd, 5, host->buffer, MAX_NOR_BUFFER_SIZE, NULL, dma_sel);
        }
        
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
        memcpy(buf + offset, (void*)UNCACHED_ADDR(host->buffer + start_offset), data_len);
#else
        memcpy(buf + offset, (void*)(host->buffer + start_offset), data_len);
#endif
        
        len -= data_len;
        offset += data_len;
        from += data_len;
    }
    
    while (len > 0)
    {
        if (len >= MAX_NOR_BUFFER_SIZE)
        {
            data_len = MAX_NOR_BUFFER_SIZE;
        }
        else
        {
            data_len = len;
        }
    
        if (host->byte_mode == 4)
        {
            cmd[1] = from >> 24;
            cmd[2] = from >> 16;
            cmd[3] = from >> 8;
            cmd[4] = from >> 0;
            cmd[5] = 0x00;
            ret = concerto_spi_cmd_read(host->spi, cmd, 6, host->buffer, MAX_NOR_BUFFER_SIZE, NULL, dma_sel);
        }
        else
        {
            cmd[1] = from >> 16;
            cmd[2] = from >> 8;
            cmd[3] = from >> 0;
            cmd[4] = 0x00;
            ret = concerto_spi_cmd_read(host->spi, cmd, 5, host->buffer, MAX_NOR_BUFFER_SIZE, NULL, dma_sel);
        }
        
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
        memcpy(buf + offset, (void*)UNCACHED_ADDR(host->buffer), data_len);
#else
        memcpy(buf + offset, (void*)(host->buffer), data_len);
#endif

        len -= data_len;
        offset += data_len;
        from += data_len;
    }

    return ret;
}

int panther_spi_nor_read_disable_secure(struct concerto_sfc_host *host, loff_t from,
                            size_t len, size_t *retlen, u_char *buf, u32 dma_sel)
{
    u8 cmd[6];
    u32 data_len;
    int ret;
    u32 offset = 0;

    cmd[0] = host->fastr_cmd;    
    while (len > 0)
    {
        if (len >= MAX_NOR_BUFFER_SIZE)
        {
            data_len = MAX_NOR_BUFFER_SIZE;
        }
        else
        {
            data_len = len;
        }
    
        if (host->byte_mode == 4)
        {
            cmd[1] = from >> 24;
            cmd[2] = from >> 16;
            cmd[3] = from >> 8;
            cmd[4] = from >> 0;
            cmd[5] = 0x00;
            ret = concerto_spi_cmd_read(host->spi, cmd, 6, host->buffer, data_len, NULL, dma_sel);
        }
        else
        {
            cmd[1] = from >> 16;
            cmd[2] = from >> 8;
            cmd[3] = from >> 0;
            cmd[4] = 0x00;
            ret = concerto_spi_cmd_read(host->spi, cmd, 5, host->buffer, data_len, NULL, dma_sel);
        }
        
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
        memcpy(buf + offset, (void*)UNCACHED_ADDR(host->buffer), data_len);
#else
        memcpy(buf + offset, (void*)host->buffer, data_len);
#endif

        len -= data_len;
        offset += data_len;
        from += data_len;
    }

    return ret;
}

extern u32 is_enable_secure_boot;
int concerto_spi_nor_read(struct mtd_info *mtd, loff_t from, size_t len,
		                    size_t *retlen, u_char *buf)
{
    struct concerto_sfc_host *host	= mtd->priv;
    int ret = -EIO;
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    u32 dma_sel = 1;
#else
    u32 dma_sel = 0;
#endif

    concerto_spi_get_device(mtd, FL_READING);

    *retlen = len;

	//FLASH_DEBUG("read byte_mode:%d cmd: %d %d %d %d %d\n",host->byte_mode,cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);

    concerto_spi_set_trans_ionum(host->spi, host->read_ionum);

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    if (is_enable_secure_boot)
    {
        panther_spi_nor_read_enable_secure(host, from, len, retlen, buf, dma_sel);
    }
    else
#endif
    {
        panther_spi_nor_read_disable_secure(host, from, len, retlen, buf, dma_sel);
    }

    concerto_spi_set_trans_ionum(host->spi, 1);
    concerto_spi_release_device(mtd);

    return ret;
}

#if 0
int concerto_spi_nor_dual_read(struct mtd_info *mtd, loff_t from, size_t len,
		      size_t *retlen, u_char *buf)
{
    struct concerto_sfc_host *host	= mtd->priv;

	u8 cmd[6];
    int ret;

    concerto_spi_get_device(mtd, FL_READING);

	cmd[0] = host->fastr_cmd;

    if(host->byte_mode == 4)
    {
        cmd[1] = from >> 24;
        cmd[2] = from >> 16;
        cmd[3] = from >> 8;
        cmd[4] = from >> 0;
        cmd[5] = 0x00;
    }
    else
    {
        cmd[1] = from >> 16;
        cmd[2] = from >> 8;
        cmd[3] = from >> 0;
        cmd[4] = 0x00;
    }

    concerto_spi_set_trans_ionum(host->spi, 2);

    *retlen = len;

	//FLASH_DEBUG("read byte_mode:%d cmd: %d %d %d %d %d\n",host->byte_mode,cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);
    if(host->byte_mode == 4)
    {
        ret = concerto_spi_read_common(host->spi, cmd, 6, buf, len);
    }
    else
    {
        ret = concerto_spi_read_common(host->spi, cmd, 5, buf, len);
    }

    concerto_spi_set_trans_ionum(host->spi, 1);

    
    concerto_spi_release_device(mtd);

    return ret;
}
#endif

int concerto_spi_nor_write(struct mtd_info *mtd, loff_t offset,
                                        size_t len, size_t *retlen, const u_char *buf)
{
    struct concerto_sfc_host *host	= mtd->priv;
    int page_addr, byte_addr;
	size_t chunk_len, actual;
	int ret = 0;
	u8 cmd[5];
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    u32 dma_sel = 1;
#else
    u32 dma_sel = 0;
#endif

    concerto_spi_get_device(mtd, FL_WRITING);

	page_addr = (int)(offset >> host->page_shift);
	byte_addr = (int)(offset & ((1 << host->page_shift) - 1));

    *retlen = 0;

	cmd[0] = host->write_cmd;
	for (actual = 0; actual < len; actual += chunk_len) 
    {
		chunk_len = MIN(len - actual, host->pagesize - byte_addr);

        if(host->byte_mode == 4)
        {
            cmd[1] = page_addr >> 16;
            cmd[2] = page_addr >> 8;
            cmd[3] = page_addr;
            cmd[4] = byte_addr;
        }
        else
        {
            cmd[1] = page_addr >> 8;
            cmd[2] = page_addr;
            cmd[3] = byte_addr;
        }
		//FLASH_DEBUG("write byte_mode:%d cmd: %d %d %d %d %d\n",host->byte_mode,cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);

		ret = concerto_spi_cmd_write_enable(host->spi);
		if (ret < 0) 
        {
			FLASH_DEBUG("SF DRV ERROR: enabling write failed!!!\n");
			break;
		}

        concerto_spi_set_trans_ionum(host->spi, host->write_ionum);

        memset(host->buffer, 0xff, MAX_NOR_BUFFER_SIZE);
        memcpy(host->buffer, buf + actual, chunk_len);

        if(host->byte_mode == 4)
        {
            ret = concerto_spi_cmd_write(host->spi, cmd, 5, host->buffer, chunk_len, dma_sel);
        }
        else
        {
            ret = concerto_spi_cmd_write(host->spi, cmd, 4, host->buffer, chunk_len, dma_sel);
        }

        concerto_spi_set_trans_ionum(host->spi, 1);
        
		if (ret < 0) 
        {
			FLASH_DEBUG("SF DRV ERROR: write failed\n");
			break;
		}

		ret = concerto_spi_cmd_wait_ready(host->spi, SPI_FLASH_PROG_TIMEOUT);
		if (ret)
        {
			break;
        }

		page_addr++;
		byte_addr = 0;

        *retlen += chunk_len;
	}

    concerto_spi_release_device(mtd);
    
	//FLASH_DEBUG("SF DRV: program %s %zu bytes @ %#x\n", ret ? "failure" : "success", len, (unsigned int)offset);

	return ret;
}


int concerto_spi_nor_erase(struct mtd_info *mtd, struct erase_info *instr)
{
    struct concerto_sfc_host *host	= mtd->priv;

    int ret;
    loff_t len;

    /* Start address must align on block boundary */
    if (instr->addr & ((1 << host->erase_shift) - 1)) 
    {
        FLASH_DEBUG("concerto_spi_erase: Unaligned address\n");
        return -EINVAL;
    }

    /* Length must align on block boundary */
    if (instr->len & ((1 << host->erase_shift) - 1)) 
    {
        FLASH_DEBUG("concerto_spi_erase: "
            "Length not block aligned\n");
        return -EINVAL;
    }

    /* Do not allow erase past end of device */
    if ((instr->len + instr->addr) > mtd->size) 
    {
        FLASH_DEBUG("concerto_spi_erase: "
            "Erase past end of device\n");
        return -EINVAL;
    }

    concerto_spi_get_device(mtd, FL_ERASING);
    
    len = instr->len;

    instr->state = MTD_ERASING;

    while (len) 
    {

        ret = concerto_spi_nor_cmd_erase(mtd, instr->addr);
        if(ret)
        {
            goto erase_exit;
        }

        len -= mtd->erasesize;

    }
    instr->state = MTD_ERASE_DONE;

erase_exit:

    ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;

    concerto_spi_release_device(mtd);
    
    /* Do call back function */
    if (!ret)
    {
        mtd_erase_callback(instr);
    }

    return ret;
}




const spi_flash_probe_t spi_nor_flashes[] = 
{
#if 0
    /* Keep it sorted by define name */
    { 0, 0x1f, spi_flash_probe_atmel, },
    { 0, 0x20, spi_flash_probe_stmicro, },
#endif
    { 0, 0xbf, spi_flash_probe_sst, },
	{ 0, 0x8c, spi_flash_probe_esmt, },
    { 0, 0xc2, spi_flash_probe_macronix, },
	{ 0, 0xc8, spi_flash_probe_gigadevice, },
	{ 0, 0x9d, spi_flash_probe_issi, },
    { 0, 0x1c, spi_flash_probe_eon, },
    { 0, 0x01, spi_flash_probe_spansion, },
    { 0, 0xef, spi_flash_probe_winbond, },
    { 0, 0xf8, spi_flash_probe_dosilicon, },
};

#if 0
static void concerto_spi_set_pinmux(void)
{
    REG32(0xbf156008) = 0;
    return;
}
#endif

static int concerto_spi_nor_init(struct mtd_info *mtd)
{
    struct concerto_sfc_host *host	= mtd->priv;

    //concerto_spi_set_pinmux();

    host->buffer = kmalloc(MAX_NOR_BUFFER_SIZE, GFP_KERNEL);
	if (!host->buffer)
    {
		return 1;
    }
	memset(host->buffer, 0xff, MAX_NOR_BUFFER_SIZE);
    
    host->controller = &host->hwcontrol;
    spin_lock_init(&host->controller->lock);
    init_waitqueue_head(&host->controller->wq);

    host->read_ionum = 1;
    host->write_ionum = 1;

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    spi_panther_init_secure();
#endif

    return 0;
}

static int concerto_spi_flash_probe(struct mtd_info *mtd, unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode)
{
    struct concerto_sfc_host *host	= mtd->priv;

    struct spi_slave *spi;
	int ret, i;
	u8 idcode[IDCODE_LEN];

	spi = spi_setup_slave(bus, cs, max_hz, spi_mode);

	if (!spi) {
		FLASH_DEBUG("SF DRV: Failed to set up slave\n");
		return 1;
	}

	ret = spi_claim_bus(spi);
	if (ret) 
    {
		FLASH_DEBUG("SF DRV: Failed to claim SPI bus: %d\n", ret);
		goto err_claim_bus;
	}

    host->spi = spi;

	/* Read the ID codes */
    ret = concerto_spi_nor_getJedecID(mtd, idcode, sizeof(idcode));
	if (ret)
    {
		goto err_read_id;
    }

    //flash_printData("Got idcodes", idcode, sizeof(idcode));

    ret = 1;

	/* search the table for matches in shift and id */
	for (i = 0; i < ARRAY_SIZE(spi_nor_flashes); ++i)
    {
		if (spi_nor_flashes[i].idcode == idcode[0]) 
        {
			/* we have a match, call probe */
			ret = spi_nor_flashes[i].probe(mtd, idcode);
			if (!ret)
            {
				break;
            }
		}
    }

	if (ret)
    {
		FLASH_DEBUG("SF DRV ERROR: Unsupported manufacturer %02x\n", idcode[0]);
		goto err_manufacturer_probe;
	}


	spi_release_bus(host->spi);

	return 0;

err_manufacturer_probe:
err_read_id:
	spi_release_bus(host->spi);
err_claim_bus:
	spi_free_slave(host->spi);
	return 1;
}

static int concerto_spi_nor_init_qe_status(struct mtd_info *mtd)
{
    struct concerto_sfc_host *host = mtd->priv;
    int ret = 0;
    u8 value = 0;
    u8 cmd[1];

    ret = concerto_spi_cmd_write_enable(host->spi);
	if (ret < 0) 
    {
		FLASH_DEBUG("SF DRV ERROR: enabling write failed!!!\n");
		goto err_init_qe;
	}
	
    cmd[0] = host->read_reg2_cmd;
	
    concerto_spi_read_common(host->spi, cmd, 1, &value, 1);

    // if use quad speed to IO, need to enable QE bit
    if (host->read_ionum == 4 || host->write_ionum == 4)
    {
        value |= (1 << 1);
    }
    else
    {
        value &= ~(1 << 1);
    }
	
    cmd[0] = host->write_reg2_cmd;

    // for support big & little endian
    concerto_spi_cmd_write(host->spi, cmd, 1, &value, 1, 0);
	
    ret = concerto_spi_cmd_wait_ready(host->spi, SPI_FLASH_PROG_TIMEOUT);
	if (ret)
    {
		goto err_init_qe;
    }

    concerto_spi_cmd_write_disable(host->spi);
    
    return 0;

err_init_qe:
    return 1;
}

#if 0
static void spi_printData(char * title, u8* buf, u32 size)
{
    u32 i;

    if(buf == NULL || size == 0)
    {
        return;
    }

    if(title != NULL)
    {
        FLASH_DEBUG("%s:\n",title);
    }
    for(i=0; i<size; i++)
    {
        FLASH_DEBUG("0x%02x ", buf[i]);
        if(i % 16 == 15)
        {
            FLASH_DEBUG("\n");
        }
    }
    FLASH_DEBUG("\n");
}

void concerto_nand_test(struct mtd_info *mtd)
{
	struct concerto_sfc_host *host	= mtd->priv;

    u32 size = 2048;
    int i;
    
    u8 *wr_buf;
    u8 *rd_buf;
    struct erase_info instr;

    loff_t offset = 0x1c0000;
    size_t retlen = 0;


    instr.mtd = mtd;
    instr.addr = offset;
    instr.len = host->blocksize;
    instr.callback = NULL;


    wr_buf = kmalloc(size, GFP_KERNEL);
    rd_buf = kmalloc(size, GFP_KERNEL);

    memset(wr_buf, 0, size);
    memset(rd_buf, 0, size);


    for(i = 0; i < size; i ++)
    {
        wr_buf[i] = i;
    }

    mtd->_read(mtd, offset, size, &retlen, rd_buf);
    spi_printData("read org", rd_buf, size);

    mtd->_erase(mtd, &instr);
    mtd->_read(mtd, offset, size, &retlen, rd_buf);
    spi_printData("read erase", rd_buf, size);

    mtd->_write(mtd, offset, size, &retlen, wr_buf);
    mtd->_read(mtd, offset, size, &retlen, rd_buf);
    spi_printData("read erase", rd_buf, size);

    kfree(wr_buf);
    kfree(rd_buf);

    return;

}
#endif

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
extern u8 empty_page_data[0x800];
static void panther_spi_read_encrypt_empty_page(struct mtd_info *mtd)
{
    //struct nand_chip *chip = mtd->priv;
    //struct concerto_snfc_host *host	= chip->priv;
    size_t retlen;
    
    // read last page of first block
    change_aes_enable_status(DISABLE_SECURE);
    concerto_spi_nor_read(mtd, 0x1f800, 0x800, &retlen, empty_page_data);
    change_aes_enable_status(ENABLE_SECURE_OTP);
}
#endif

static int concerto_spi_nor_probe(struct platform_device * pltdev)
{
	int	result = 0;
    u32 boot_type;

	struct concerto_sfc_host *host;
	struct mtd_info	  *mtd;

	int	size = sizeof(struct concerto_sfc_host)	+ sizeof(struct	mtd_info);

#ifdef CONFIG_MTD_CMDLINE_PARTS
    int nr_parts = 0;
    struct mtd_partition *parts = NULL;

    static const char *part_probes[] = {"cmdlinepart", NULL,};
#endif

    // check boot from NAND or NOR
    boot_type = otp_get_boot_type();
    if (boot_type != BOOT_FROM_NOR)
    {
        return -1;
    }

    FLASH_DEBUG("===============concerto_spi_init start........===============\n");
    
	host = kmalloc(size, GFP_KERNEL);
	if (!host)
	{
		dev_err(&pltdev->dev, "failed to allocate device structure.\n");
		return -ENOMEM;
	}

	memset((char *)host, 0,	size);
	platform_set_drvdata(pltdev, host);

	host->dev  = &pltdev->dev;
	host->mtd  = mtd  =	(struct	mtd_info *)&host[1];


	mtd->priv  = host;
	mtd->owner = THIS_MODULE;
	mtd->name  = (char*)(pltdev->name);

    if (concerto_spi_nor_init(mtd))
    {
        dev_err(&pltdev->dev, "failed to spi nor init.\n");
        result = -EIO;
        goto err;
    }

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    pdma_init();
    init_spi_data();
#endif

    if (concerto_spi_flash_probe(mtd, CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS, 
                                CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE))
    {
		dev_err(&pltdev->dev, "failed to spi probe.\n");
		result = -ENXIO;
		goto err;
    }

    if (concerto_spi_nor_init_qe_status(mtd))
    {
        dev_err(&pltdev->dev, "failed to initial qe status.\n");
        result = -ENXIO;
		goto err;
    }

    host->page_shift = ffs(host->pagesize) - 1;
    host->erase_shift = ffs(host->blocksize) - 1;
    if (host->chipsize & 0xffffffff)
    {
        host->chip_shift = ffs((unsigned)host->chipsize) - 1;
    }
    else
    {
        host->chip_shift = ffs((unsigned)(host->chipsize >> 32)) + 32 - 1;
    }

    mtd->erasesize = host->blocksize;
    mtd->size = host->chipsize;
    mtd->writesize = 1;
    mtd->writebufsize = 2048;  // added for UBI/UBIFS back-port, not tested yet!

    FLASH_DEBUG("SF DRV: Detected %s with page size: %d, total %d M bytes\n", 
        host->name, host->pagesize, (u32)(host->chipsize >> 20));

	host->state = FL_READY;

	mtd->type = MTD_NORFLASH;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->owner = THIS_MODULE;
 	mtd->_point = NULL;
 	mtd->_unpoint = NULL;
	mtd->_sync = NULL;
	mtd->_lock = NULL;
	mtd->_unlock = NULL;
	mtd->_suspend = NULL;
	mtd->_resume = NULL;

    mtd->name = CONCERTO_SPI_FLASH_NAME;

#ifdef CONFIG_MTD_CMDLINE_PARTS
    nr_parts = parse_mtd_partitions(mtd, part_probes, &parts, 0);

    if (!nr_parts)
    {
        FLASH_DEBUG(("cmdlinepart has no any partitions!!!\n"));
        return -1;
    }

    if (nr_parts > 0) 
    {
        result = add_mtd_partitions(mtd, parts, nr_parts);
    }
#else
    result = add_mtd_partitions (mtd, concerto_spi_nor_partitions, ARRAY_SIZE(concerto_spi_nor_partitions));
#endif

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    panther_spi_read_encrypt_empty_page(mtd);
#endif

    FLASH_DEBUG("===============concerto_spi_init end..........===============\n");

    //concerto_nand_test(mtd);

    return result;

err:
    if (host->buffer)
	{
		kfree(host->buffer);
		host->buffer = NULL;
	}
	kfree(host);
	platform_set_drvdata(pltdev, NULL);

	return result;
}


void concerto_spi_nor_release(struct mtd_info *mtd)
{
	del_mtd_partitions(mtd);

	del_mtd_device(mtd);
}


/*****************************************************************************/
int	concerto_spi_nor_remove(struct platform_device *pltdev)
{
	struct concerto_sfc_host *host	= platform_get_drvdata(pltdev);

	concerto_spi_nor_release(host->mtd);
	kfree(host);
	platform_set_drvdata(pltdev, NULL);

	return 0;
}
/*****************************************************************************/
static void	concerto_spi_nor_pltdev_release(struct device *dev)
{
}
/*****************************************************************************/
static struct platform_driver concerto_spi_nor_pltdrv =
{
	.driver.name   = "concerto spi nor",
	.probe	= concerto_spi_nor_probe,
	.remove	= concerto_spi_nor_remove,
};
/*****************************************************************************/
static struct platform_device concerto_spi_nor_pltdev =
{
	.name			= "concerto spi nor",
	.id				= -1,

	.dev.platform_data	   = NULL,
	.dev.release		   = concerto_spi_nor_pltdev_release,

	.num_resources	= 0,
	.resource		= NULL,
};

/*****************************************************************************/

static int __init concerto_spi_nor_module_init(void)
{
	int	result = 0;
	
	FLASH_DEBUG("\nMT concerto Spi Nor Flash Controller Device Driver, Version 1.00\n");

	result = platform_driver_register(&concerto_spi_nor_pltdrv);
	if (result < 0)
	{
		return result;
	}

	result = platform_device_register(&concerto_spi_nor_pltdev);
	if (result < 0)
	{
		platform_driver_unregister(&concerto_spi_nor_pltdrv);
		return result;
	}

	return result;
}

/*****************************************************************************/

static void	__exit concerto_spi_nor_module_exit	(void)
{
	platform_driver_unregister(&concerto_spi_nor_pltdrv);
	platform_device_unregister(&concerto_spi_nor_pltdev);
}
/*****************************************************************************/

module_init(concerto_spi_nor_module_init);
module_exit(concerto_spi_nor_module_exit);


/*****************************************************************************/







