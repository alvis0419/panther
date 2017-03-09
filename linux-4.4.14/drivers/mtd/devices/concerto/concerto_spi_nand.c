
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>

#include <linux/mtd/nand.h>
#include <linux/platform_device.h>

#include <asm/bootinfo.h>

#include "concerto_spi_nand.h"
#include <asm/mach-panther/bootinfo.h>
#include <asm/mach-panther/pdma.h>

#define CONCERTO_SPI_NAND_FLASH_NAME     "mt_snf"

#include <linux/mtd/partitions.h>
#include "../../mtdcore.h"

#ifndef CONFIG_MTD_CMDLINE_PARTS

#define SNF_ADJUST_PART_NUM    2
#define SNF_MIN_BLOCK_SIZE     128 * 1024

#if !defined(CONFIG_PANTHER_MTD_SIZE)
#define CONFIG_PANTHER_MTD_SIZE 0x1000000
#endif

#if !defined(CONFIG_PANTHER_KERNEL_SIZE)
#define CONFIG_PANTHER_KERNEL_SIZE 2
#endif

#define SNF_BTINT_START    0x00000000                            //0x00000000
#define SNF_BTINT_LENTH    SNF_MIN_BLOCK_SIZE                    //0x00020000

#define SNF_CDB_START      (SNF_BTINT_START + SNF_BTINT_LENTH)
#define SNF_CDB_LENTH      SNF_MIN_BLOCK_SIZE

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

static struct mtd_partition concerto_spi_nand_partitions[] = 
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
static void concerto_printData(char * title, u8* buf, u32 size)
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
#endif

/*****************************************************************************/


#if 1
// panther support now
static struct nand_ecclayout nand_oob_GD5F1GQ4UCYIG = 
{
	.oobfree = {{0x10, 16}, {0x20, 16}, {0x30, 16}}
};

static struct nand_ecclayout nand_oob_GD5F2GQ4UCYIG = 
{
	.oobfree = {{0x10, 16}, {0x20, 16}, {0x30, 16}}
};


static struct nand_ecclayout nand_oob_GD5F1GQ4UAYIG = 
{
	.oobfree = {{0x4, 8}, {0x14, 8}, {0x24, 8}, {0x34, 8}}
};

static struct nand_ecclayout nand_oob_GD5F2GQ4UAYIG = 
{
	.oobfree = {{0x4, 8}, {0x14, 8}, {0x24, 8}, {0x34, 8}}
};

static struct nand_ecclayout nand_oob_F50L512M41A = 
{
	.oobfree = {{0x8, 8}, {0x18, 8}, {0x28, 8}, {0x38, 8}}
};

static struct nand_ecclayout nand_oob_F50L1G41A = 
{
	.oobfree = {{0x8, 8}, {0x18, 8}, {0x28, 8}, {0x38, 8}}
};

static struct nand_ecclayout nand_oob_W25N01GV = 
{
	.oobfree = {{0x2, 6}, {0x10, 8}, {0x20, 8}, {0x30, 8}}
};

static struct nand_ecclayout nand_oob_GD5F1GQ4UB = 
{
	.oobfree = {{0x10, 48}}
};

static struct nand_ecclayout nand_oob_GD5F2GQ4UB = 
{
	.oobfree = {{0x10, 48}}
};

static struct nand_ecclayout nand_oob_GD5F4GQ4U = 
{
	.oobfree = {{0x10, 112}}
};

static struct nand_ecclayout nand_oob_HYF1GQ4UAACAE = 
{
	.oobfree = {{0x2, 6}, {0x22, 6}, {0x42, 6}, {0x62, 6}}
};

static struct nand_ecclayout nand_oob_HYF2GQ4UAACAE = 
{
	.oobfree = {{0x2, 6}, {0x22, 6}, {0x42, 6}, {0x62, 6}}
};

static const struct concerto_spi_nand_flash_params concerto_spi_nand_flash_table[] = 
{
    {
        .id                 = SPI_NAND_FLASH_GD5F1GQ4UCYIG,
        .page_size          = 2048,
        .oob_size           = 128,
        .pages_per_block    = 64,
        .nr_blocks          = 1024,
        .name               = "GD5F1GQ4UCYIG",
        .layout             = &nand_oob_GD5F1GQ4UCYIG,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_START,
        .read_from_cache_dummy = 8,
    },
    {
        .id                 = SPI_NAND_FLASH_GD5F2GQ4UCYIG,
        .page_size          = 2048,
        .oob_size           = 128,
        .pages_per_block    = 64,
        .nr_blocks          = 2048,
        .name               = "GD5F2GQ4UCYIG",
        .layout             = &nand_oob_GD5F2GQ4UCYIG,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_START,
        .read_from_cache_dummy = 8,
    },
	{
		.id                 = SPI_NAND_FLASH_F50L512M41A,
		.page_size          = 2048,
        .oob_size           = 64,
		.pages_per_block    = 64,
		.nr_blocks          = 512,
		.name               = "F50L512M41A",
		.layout             = &nand_oob_F50L512M41A,
		.read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
	},
	{
		.id                 = SPI_NAND_FLASH_F50L1G41A,
		.page_size          = 2048,
        .oob_size           = 64,
		.pages_per_block    = 64,
		.nr_blocks          = 1024,
		.name               = "F50L1G41A",
		.layout             = &nand_oob_F50L1G41A,
		.read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
	},
	{
		.id                 = SPI_NAND_FLASH_GD5F1GQ4UAYIG,
		.page_size          = 2048,
        .oob_size           = 64,
		.pages_per_block    = 64,
		.nr_blocks          = 1024,
		.name               = "GD5F1GQ4UAYIG",
        .layout             = &nand_oob_GD5F1GQ4UAYIG,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
	},
    {
        .id                 = SPI_NAND_FLASH_GD5F2GQ4UAYIG,
        .page_size          = 2048,
        .oob_size           = 64,
        .pages_per_block    = 64,
        .nr_blocks          = 2048,
        .name               = "GD5F2GQ4UAYIG",
        .layout             = &nand_oob_GD5F2GQ4UAYIG,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
    {
        .id                 = SPI_NAND_FLASH_W25N01GV,
        .page_size          = 2048,
        .oob_size           = 64,
        .pages_per_block    = 64,
        .nr_blocks          = 1024,
        .name               = "W25N01GV",
        .layout             = &nand_oob_W25N01GV,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
    {
        .id                 = SPI_NAND_FLASH_GD5F1GQ4UB,
        .page_size          = 2048,
        .oob_size           = 128,
        .pages_per_block    = 64,
        .nr_blocks          = 1024,
        .name               = "GD5F1GQ4UB",
        .layout             = &nand_oob_GD5F1GQ4UB,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
    {
        .id                 = SPI_NAND_FLASH_GD5F2GQ4UB,
        .page_size          = 2048,
        .oob_size           = 128,
        .pages_per_block    = 64,
        .nr_blocks          = 2048,
        .name               = "GD5F2GQ4UB",
        .layout             = &nand_oob_GD5F2GQ4UB,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
    {
        .id                 = SPI_NAND_FLASH_GD5F4GQ4U_A,
        .page_size          = 4096,
        .oob_size           = 256,
        .pages_per_block    = 64,
        .nr_blocks          = 2048,
        .name               = "GD5F4GQ4U",
        .layout             = &nand_oob_GD5F4GQ4U,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
    {
        .id                 = SPI_NAND_FLASH_GD5F4GQ4U_B,
        .page_size          = 4096,
        .oob_size           = 256,
        .pages_per_block    = 64,
        .nr_blocks          = 2048,
        .name               = "GD5F4GQ4U",
        .layout             = &nand_oob_GD5F4GQ4U,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
    {
        .id                 = SPI_NAND_FLASH_HYF1GQ4UAACAE,
        .page_size          = 2048,
        .oob_size           = 128,
        .pages_per_block    = 64,
        .nr_blocks          = 1024,
        .name               = "HYF1GQ4UAACAE",
        .layout             = &nand_oob_HYF1GQ4UAACAE,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
    {
        .id                 = SPI_NAND_FLASH_HYF2GQ4UAACAE,
        .page_size          = 2048,
        .oob_size           = 128,
        .pages_per_block    = 64,
        .nr_blocks          = 2048,
        .name               = "HYF2GQ4UAACAE",
        .layout             = &nand_oob_HYF2GQ4UAACAE,
        .read_cmd_dummy_type   = READ_CMD_DUMMY_END,
        .read_from_cache_dummy = 0,
    },
};


static int concerto_spi_nand_get_device(struct mtd_info *mtd, int new_state)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

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

static void concerto_spi_nand_release_device(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

	/* Release the controller and the chip */
	spin_lock(&host->controller->lock);
	host->controller->active = NULL;
	host->state = FL_READY;
	wake_up(&host->controller->wq);
	spin_unlock(&host->controller->lock);
}


static u32 concerto_spi_nand_getJedecID(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 cmd[4];
    u8 codeID[4];
    u32 id = 0xffffff;
    int ret;

    cmd[0] = SPI_NAND_FLASH_CMD_RD_MAN_ID;
    cmd[1] = 0x00;
    
    ret = concerto_spi_read_common(host->spi, cmd, 2, codeID, 4);

    if (ret)
    {
        id = 0xffffff;
    }
    else
    {
        id = codeID[0] | codeID[1] << 8 | (codeID[2] << 16);
    }
    
    return id;
}


static int concerto_spi_nand_wait_write_complete(struct mtd_info *mtd, u32 timeout)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 resp = 0;
    int ret = 0;
    u32 status = 0;

    u8 cmd[2];

    status = 1;

    while (status && timeout)
    {
        cmd[0] = SPI_NAND_FLASH_CMD_RD_SR;
        cmd[1] = SPI_NAND_FLASH_ADDR_ST;
        
        ret = concerto_spi_read_common(host->spi, cmd, 2, &resp, 1);
        if (ret < 0)
        {
            timeout--;
            continue;
        }
        if ((resp & 0x01) == 1)
        {
            status = 1;
            timeout--;
        }
        else
        {
            status = 0;
        }
    }
    if (timeout == 0)
    {
        FLASH_DEBUG("concerto_spi_nand_wait_write_complete: time out! %s %d %s\n",
                __FUNCTION__, __LINE__, __FILE__);
    }
    return 0;
}


static void concerto_spi_nand_internel_ecc_on(struct mtd_info *mtd, u8 on)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;
	
    u8 value = 0;
    u8 cmd[2];
	
    cmd[0] = SPI_NAND_FLASH_CMD_RD_SR;
    cmd[1] = SPI_NAND_FLASH_ADDR_FT;
	
    concerto_spi_read_common(host->spi, cmd, 2, &value, 1);
	
    if (on)
    {
        value |= (1 << 4);
    }
    else
    {
        value &= ~(1 << 4);
    }
    
    cmd[0] = SPI_NAND_FLASH_CMD_WR_SR;
    cmd[1] = SPI_NAND_FLASH_ADDR_FT;
	
    concerto_spi_cmd_write(host->spi, cmd, 2, &value, 1, 0);
	
    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    return;

}


static u8 concerto_spi_nand_get_ecc_result(struct mtd_info *mtd)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

    u8 resp = 0;
    u8 result = 0;
    u8 cmd[2];

    cmd[0] = SPI_NAND_FLASH_CMD_RD_SR;
    cmd[1] = SPI_NAND_FLASH_ADDR_ST;
    
    concerto_spi_read_common(host->spi, cmd, 2, &resp, 1);
    result = (resp >> 4) & 0x03;

    if (result == 0x03)
    {
        result = 0x01;
    }

    return result;
}


static void concerto_spi_nand_quad_feature_on(struct mtd_info *mtd, u8 on)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 value = 0;
    u8 cmd[2];

    cmd[0] = SPI_NAND_FLASH_CMD_RD_SR;
    cmd[1] = SPI_NAND_FLASH_ADDR_FT;

    concerto_spi_read_common(host->spi, cmd, 2, &value, 1);

    if (on)
    {
        value |= (1 << 0);
    }
    else
    {
        value &= ~(1 << 0);
    }
    
    cmd[0] = SPI_NAND_FLASH_CMD_WR_SR;
    cmd[1] = SPI_NAND_FLASH_ADDR_FT;

    concerto_spi_cmd_write(host->spi, cmd, 2, &value, 1, 0);

    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    return;

}


static u8 concerto_spi_nand_get_quad_feature(struct mtd_info *mtd)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

    u8 resp = 0;
    u8 result = 0;
    u8 cmd[2];

    cmd[0] = SPI_NAND_FLASH_CMD_RD_SR;
    cmd[1] = SPI_NAND_FLASH_ADDR_FT;
        
    concerto_spi_read_common(host->spi, cmd, 2, &resp, 1);

    result = resp & 0x01;

    return result;
}

static int concerto_spi_nand_set_feature(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 value = 0;
    u8 cmd[2];
    
    cmd[0] = SPI_NAND_FLASH_CMD_WR_SR;
    cmd[1] = SPI_NAND_FLASH_ADDR_PT;
    concerto_spi_cmd_write(host->spi, cmd, 2, &value, 1, 0);

    return 0;
}

static int concerto_spi_nand_reset(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 cmd[5];
    
    cmd[0] = SPI_NAND_FLASH_CMD_WR_RESET;
	
    concerto_spi_read_common(host->spi, cmd, 1, NULL, 0);
    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    concerto_spi_nand_set_feature(mtd);
    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    return 0;
}

static int concerto_spi_nand_write_enable(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 cmd[5];
	
    cmd[0] = SPI_NAND_FLASH_CMD_WR_EN;
	
    concerto_spi_read_common(host->spi, cmd, 1, NULL, 0);

    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    return 0;
}

static int concerto_spi_nand_block_erase(struct mtd_info *mtd, u32 block_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 cmd[5];
    u32 page_addr = block_addr << (host->erase_shift - host->page_shift);

    concerto_spi_nand_write_enable(mtd);

    cmd[0] = SPI_NAND_FLASH_CMD_BLK_ERASE;
    cmd[1] = page_addr >> 16;
    cmd[2] = page_addr >> 8;
    cmd[3] = page_addr >> 0;

    concerto_spi_cmd_write(host->spi, cmd, 4, NULL, 0, 0);

    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    return 0;

}

static int concerto_spi_nand_program_execute(struct mtd_info *mtd, u32 page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 cmd[5];

    cmd[0] = SPI_NAND_FLASH_CMD_PG_EXC;
    cmd[1] = page_addr >> 16;
    cmd[2] = page_addr >> 8;
    cmd[3] = page_addr >> 0;

    concerto_spi_cmd_write(host->spi, cmd, 4, NULL, 0, 0);

    return 0;

}

static int concerto_spi_nand_program_single_load(struct mtd_info *mtd, u32 col_addr, u8* buf, u32 size)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 cmd[5];

    cmd[0] = SPI_NAND_FLASH_CMD_LOAD;
    cmd[1] = col_addr >> 8;
    cmd[2] = col_addr;

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    concerto_spi_cmd_write(host->spi, cmd, 3, buf, size, 1);
#else
    concerto_spi_cmd_write(host->spi, cmd, 3, buf, size, 0);
#endif


    return 0;
}

static int concerto_spi_nand_program_quad_load(struct mtd_info *mtd, u32 col_addr, u8 *buf, u32 size)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host = chip->priv;

    u8 cmd[5];

    cmd[0] = SPI_NAND_FLASH_CMD_QUAD_LOAD;
    cmd[1] = col_addr >> 8;
    cmd[2] = col_addr;

    concerto_spi_set_trans_ionum(host->spi, 4);
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    concerto_spi_cmd_write(host->spi, cmd, 3, buf, size, 1);
#else
    concerto_spi_cmd_write(host->spi, cmd, 3, buf, size, 0);
#endif
    concerto_spi_set_trans_ionum(host->spi, 1);

    return 0;
}

static int concerto_spi_nand_single_page_read_from_cache(struct mtd_info *mtd, u32 col_addr, u8* buf, u32 size)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;    
    struct spi_read_cfg read_cfg;
    u8 cmd[4];

    memset(&read_cfg, 0, sizeof(read_cfg));

    
    cmd[0] = SPI_NAND_FLASH_CMD_PAGE_RD_FRM_CACHE;
    if (host->read_cmd_dummy_type == READ_CMD_DUMMY_END)
    {
        cmd[1] = (col_addr >> 8);
        cmd[2] = (col_addr >> 0);
        cmd[3] = 0;
    }
    else
    {
        cmd[1] = 0;
        cmd[2] = (col_addr >> 8);
        cmd[3] = (col_addr >> 0);
    }

    // see explain about read_from_cache_dummy of concerto_spi_nand.h
    if (host->read_from_cache_dummy != 0)
    {
        read_cfg.read_from_cache_dummy = host->read_from_cache_dummy;
    }

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    concerto_spi_cmd_read(host->spi, cmd, 4, buf, size, &read_cfg, 1);
#else
    concerto_spi_cmd_read(host->spi, cmd, 4, buf, size, &read_cfg, 0);
#endif

    return 0;
}

static int concerto_spi_nand_dual_page_read_from_cache(struct mtd_info *mtd, u32 col_addr, u8* buf, u32 size)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;
    struct spi_read_cfg read_cfg;
    u8 cmd[4];

    memset(&read_cfg, 0, sizeof(read_cfg));
    
    cmd[0] = SPI_NAND_FLASH_CMD_PAGE_RD_DUAL_FRM_CACHE;
    if (host->read_cmd_dummy_type == READ_CMD_DUMMY_END)
    {
        cmd[1] = (col_addr >> 8);
        cmd[2] = (col_addr >> 0);
        cmd[3] = 0;
    }
    else
    {
        cmd[1] = 0;
        cmd[2] = (col_addr >> 8);
        cmd[3] = (col_addr >> 0);
    }

    // see explain about read_from_cache_dummy of concerto_spi_nand.h
    if (host->read_from_cache_dummy != 0)
    {
        read_cfg.read_from_cache_dummy = host->read_from_cache_dummy;
    }

    concerto_spi_set_trans_ionum(host->spi, 2);
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    concerto_spi_cmd_read(host->spi, cmd, 4, buf, size, &read_cfg, 1);
#else
    concerto_spi_cmd_read(host->spi, cmd, 4, buf, size, &read_cfg, 0);
#endif
    concerto_spi_set_trans_ionum(host->spi, 1);

    return 0;
}

static int concerto_spi_nand_quad_page_read_from_cache(struct mtd_info *mtd, u32 col_addr, u8* buf, u32 size)
{
    struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;
    struct spi_read_cfg read_cfg;
    u8 cmd[4];

    memset(&read_cfg, 0, sizeof(read_cfg));
    
    cmd[0] = SPI_NAND_FLASH_CMD_PAGE_RD_QUAD_FRM_CACHE;
    if (host->read_cmd_dummy_type == READ_CMD_DUMMY_END)
    {
        cmd[1] = (col_addr >> 8);
        cmd[2] = (col_addr >> 0);
        cmd[3] = 0;
    }
    else
    {
        cmd[1] = 0;
        cmd[2] = (col_addr >> 8);
        cmd[3] = (col_addr >> 0);
    }

    // see explain about read_from_cache_dummy of concerto_spi_nand.h
    if (host->read_from_cache_dummy != 0)
    {
        read_cfg.read_from_cache_dummy = host->read_from_cache_dummy;
    }

    concerto_spi_set_trans_ionum(host->spi, 4);
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    concerto_spi_cmd_read(host->spi, cmd, 4, buf, size, &read_cfg, 1);
#else
    concerto_spi_cmd_read(host->spi, cmd, 4, buf, size, &read_cfg, 0);
#endif
    concerto_spi_set_trans_ionum(host->spi, 1);

    return 0;
}

static int concerto_spi_nand_page_read_to_cache(struct mtd_info *mtd, u32 page_addr)
{    
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 cmd[4];
    
    cmd[0] = SPI_NAND_FLASH_CMD_PAGE_RD_TO_CACHE;
    cmd[1] = page_addr >> 16;
    cmd[2] = page_addr >> 8;
    cmd[3] = page_addr >> 0;
    
    concerto_spi_read_common(host->spi, cmd, 4, NULL, 0);

    return 0;
}


static int concerto_spi_nand_page_write(struct mtd_info *mtd, u32 page_addr, u32 col_addr, u8* buf, u32 size)
{    
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    memset(host->buffer, 0xff, mtd->writesize);
    memcpy(host->buffer + col_addr, buf, size);

    concerto_spi_nand_write_enable(mtd);
    if (host->write_ionum == 4)
    {
        concerto_spi_nand_program_quad_load(mtd, 0, host->buffer, mtd->writesize);
    }
    else
    {
        concerto_spi_nand_program_single_load(mtd, 0, host->buffer, mtd->writesize);
    }
    concerto_spi_nand_program_execute(mtd, page_addr);

    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    return 0;
}

static int concerto_spi_nand_oob_write(struct mtd_info *mtd, u32 page_addr, u8* buf)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    concerto_spi_nand_write_enable(mtd);
    
    change_aes_enable_status(DISABLE_SECURE);
    if (host->write_ionum == 4)
    {
        concerto_spi_nand_program_quad_load(mtd, mtd->writesize, host->oob_poi, mtd->oobsize);
    }
    else
    {
        concerto_spi_nand_program_single_load(mtd, mtd->writesize, host->oob_poi, mtd->oobsize);
    }
    change_aes_enable_status(ENABLE_SECURE_OTP);

    concerto_spi_nand_program_execute(mtd, page_addr);

    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    return 0;
}

static int concerto_spi_nand_page_read(struct mtd_info *mtd, u32 page_addr, u32 col_addr, u8* buf, u32 size)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

    u8 ecc_result;
    
    concerto_spi_nand_page_read_to_cache(mtd, page_addr);
    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    if (host->read_ionum == 4)
    {
        concerto_spi_nand_quad_page_read_from_cache(mtd, 0, host->buffer, mtd->writesize);
    }
    else if (host->read_ionum == 2)
    {
        concerto_spi_nand_dual_page_read_from_cache(mtd, 0, host->buffer, mtd->writesize);
    }
    else
    {
        concerto_spi_nand_single_page_read_from_cache(mtd, 0, host->buffer, mtd->writesize);
    }

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    memcpy(buf, (void*)UNCACHED_ADDR(host->buffer + col_addr), size);
#else
    memcpy(buf, (void*)(host->buffer + col_addr), size);
#endif
    ecc_result = concerto_spi_nand_get_ecc_result(mtd);

    if (ecc_result == 0x02)
    {
        mtd->ecc_stats.failed++;
    }

    return 0;
}

static int concerto_spi_nand_oob_read(struct mtd_info *mtd, u32 page_addr, u8* buf)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u8 ecc_result;
    
    concerto_spi_nand_page_read_to_cache(mtd, page_addr);
    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    change_aes_enable_status(DISABLE_SECURE);
    if (host->read_ionum == 4)
    {
        concerto_spi_nand_quad_page_read_from_cache(mtd, mtd->writesize, host->oob_poi, mtd->oobsize);
    }
    else if (host->read_ionum == 2)
    {
        concerto_spi_nand_dual_page_read_from_cache(mtd, mtd->writesize, host->oob_poi, mtd->oobsize);
    }
    else
    {
        concerto_spi_nand_single_page_read_from_cache(mtd, mtd->writesize, host->oob_poi, mtd->oobsize);
    }
    change_aes_enable_status(ENABLE_SECURE_OTP);

    ecc_result = concerto_spi_nand_get_ecc_result(mtd);

    if (ecc_result == 0x02)
    {
        mtd->ecc_stats.failed++;
    }

    return 0;
}

static int concerto_spi_nand_oob_badblock_flag_read(struct mtd_info *mtd, u32 page_addr, u8* buf)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    concerto_spi_nand_page_read_to_cache(mtd, page_addr);
    concerto_spi_nand_wait_write_complete(mtd, SPI_FLASH_PROG_TIMEOUT);

    concerto_spi_nand_single_page_read_from_cache(mtd, mtd->writesize + host->badblockpos, buf, 1);

    return 0;
}

static int concerto_spi_nand_block_is_bad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    u32 block_addr = (u32)(offs >> host->erase_shift);
    
    u8 flag = 0;
    concerto_spi_nand_internel_ecc_on(mtd, 0);
    change_aes_enable_status(DISABLE_SECURE);
    
    concerto_spi_nand_oob_badblock_flag_read(mtd, (block_addr << (host->erase_shift - host->page_shift)), &flag);
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    flag = *((u8 *)UNCACHED_ADDR(&flag));
#endif
    if (flag != 0xff)
    {
        return 1;
    }
    
    concerto_spi_nand_oob_badblock_flag_read(mtd, (block_addr << (host->erase_shift - host->page_shift)) + 1, &flag);
#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    flag = *((u8 *)UNCACHED_ADDR(&flag));
#endif
    if (flag != 0xff)
    {
        return 1;
    }

    change_aes_enable_status(ENABLE_SECURE_OTP);
    concerto_spi_nand_internel_ecc_on(mtd, 1);
    return 0;
}


static int concerto_spi_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

    u32 blockno;
    int ret;
    loff_t len;

    /* Start address must align on block boundary */
    if (instr->addr & ((1 << host->erase_shift) - 1)) 
    {
        FLASH_DEBUG("concerto_spi_nand_erase: Unaligned address\n");
        return -EINVAL;
    }

    /* Length must align on block boundary */
    if (instr->len & ((1 << host->erase_shift) - 1)) 
    {
        FLASH_DEBUG("concerto_spi_nand_erase: "
            "Length not block aligned\n");
        return -EINVAL;
    }

    /* Do not allow erase past end of device */
    if ((instr->len + instr->addr) > mtd->size) 
    {
        FLASH_DEBUG("concerto_spi_nand_erase: "
            "Erase past end of device\n");
        return -EINVAL;
    }

    concerto_spi_nand_get_device(mtd, FL_ERASING);

    blockno = (u32)(instr->addr >> host->erase_shift);

    len = instr->len;

    instr->state = MTD_ERASING;

    while (len) 
    {
        if(concerto_spi_nand_block_is_bad(mtd, (loff_t)(blockno << host->erase_shift)))
        {
            FLASH_DEBUG("concerto_spi_nand_erase: attempt to erase a "
                "bad block %d\n", blockno);
            instr->state = MTD_ERASE_FAILED;
            goto erase_exit;
        }

        concerto_spi_nand_block_erase(mtd, blockno);

        /* Increment page address and decrement length */
        len -= mtd->erasesize;
        blockno += 1;

    }
    instr->state = MTD_ERASE_DONE;

erase_exit:

    ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;

    concerto_spi_nand_release_device(mtd);
    
    /* Do call back function */
    if (!ret)
    {
        mtd_erase_callback(instr);
    }

    return ret;
}



static u8 *concerto_spi_nand_transfer_oob(struct mtd_info *mtd, u8 *oob,
				  struct mtd_oob_ops *ops, size_t len)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

	switch(ops->mode) 
    {
	case MTD_OPS_PLACE_OOB:
	case MTD_OPS_RAW:
        memcpy(oob, host->oob_poi + ops->ooboffs, len);
		return oob + len;

	case MTD_OPS_AUTO_OOB: 
    {
		struct nand_oobfree *free = host->ecclayout->oobfree;
		u32 boffs = 0, roffs = ops->ooboffs;
		size_t bytes = 0;

		for(; free->length && len; free++, len -= bytes) 
        {
			/* Read request not from offset 0 ? */
			if (unlikely(roffs)) 
            {
				if (roffs >= free->length) 
                {
					roffs -= free->length;
					continue;
				}
				boffs = free->offset + roffs;
				bytes = min_t(size_t, len, (free->length - roffs));
				roffs = 0;
			} 
            else 
            {
				bytes = min_t(size_t, len, free->length);
				boffs = free->offset;
			}
            memcpy(oob, host->oob_poi + boffs, bytes);
			oob += bytes;
		}
		return oob;
	}
	default:
		BUG();
	}
	return NULL;
}

static u8 *concerto_spi_nand_fill_oob(struct mtd_info *mtd, uint8_t *oob, size_t len,
				  struct mtd_oob_ops *ops)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

	memset(host->oob_poi, 0xff, mtd->oobsize);

    switch(ops->mode) 
    {
        case MTD_OPS_PLACE_OOB:
        case MTD_OPS_RAW:
            memcpy(host->oob_poi + ops->ooboffs, oob, len);
            return oob + len;

        case MTD_OPS_AUTO_OOB: 
            {
                struct nand_oobfree *free = host->ecclayout->oobfree;
                uint32_t boffs = 0, woffs = ops->ooboffs;
                size_t bytes = 0;

                for(; free->length && len; free++, len -= bytes) 
                {
                    /* Write request not from offset 0 ? */
                    if (unlikely(woffs)) 
                    {
                        if (woffs >= free->length) 
                        {
                            woffs -= free->length;
                            continue;
                        }
                        boffs = free->offset + woffs;
                        bytes = min_t(size_t, len, (free->length - woffs));
                        woffs = 0;
                    } 
                    else 
                    {
                        bytes = min_t(size_t, len, free->length);
                        boffs = free->offset;
                    }
                    memcpy(host->oob_poi + boffs, oob, bytes);
                    oob += bytes;
                }
                return oob;
            }
        default:
            BUG();
    }
    return NULL;
}


static int concerto_spi_nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

	int page, col, bytes;
	struct mtd_ecc_stats stats;

	int ret = 0;
	uint32_t readlen = ops->len;
	uint32_t oobreadlen = ops->ooblen;
	uint32_t max_oobsize = ops->mode == MTD_OPS_AUTO_OOB ?
		mtd->oobavail : mtd->oobsize;

	uint8_t *bufpoi, *oob, *buf;
	unsigned int max_bitflips = 0;

	stats = mtd->ecc_stats;

	page = (int)(from >> host->page_shift);
	col = (int)(from & (mtd->writesize - 1));

	buf = ops->datbuf;
	oob = ops->oobbuf;
    
    while(1) 
    {
        bytes = min(mtd->writesize - col, readlen);

        bufpoi = buf;
        
        ret = concerto_spi_nand_page_read(mtd, page, col, bufpoi, bytes);

        max_bitflips = max_t(unsigned int, max_bitflips, ret);

        buf += bytes;

        // use unlikely macro for optimization
        if (unlikely(oob)) 
        {
            int toread = min(oobreadlen, max_oobsize);

            if (toread) 
            {
                oob = concerto_spi_nand_transfer_oob(mtd, oob, ops, toread);
                oobreadlen -= toread;
            }
        }

        readlen -= bytes;

        if (!readlen)
        {
            break;
        }

        /* For subsequent reads align to page boundary. */
        col = 0;
        /* Increment page address */
        page++;
    }

    ops->retlen = ops->len - (size_t)readlen;
    if (oob)
    {
        ops->oobretlen = ops->ooblen - oobreadlen;
    }

    if (ret)
    {
        return ret;
    }

    if (mtd->ecc_stats.failed - stats.failed)
    {
        return -EBADMSG;
    }

	return max_bitflips;
}


static int concerto_spi_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
		     size_t *retlen, uint8_t *buf)
{
	struct mtd_oob_ops ops;

	int ret;

    concerto_spi_nand_get_device(mtd, FL_READING);
	ops.len = len;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ops.mode = 0;

	ret = concerto_spi_nand_do_read_ops(mtd, from, &ops);

	*retlen = ops.retlen;

    concerto_spi_nand_release_device(mtd);
	return ret;
}


static int concerto_spi_nand_do_read_oob(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

    int page;
    
	struct mtd_ecc_stats stats;

    int readlen = ops->ooblen;
    int len;
    u8 *buf = ops->oobbuf;

    // FLASH_DEBUG("%s: from = 0x%08Lx, len = %i\n",
	//		__func__, (unsigned long long)from, readlen);

    stats = mtd->ecc_stats;

    if (ops->mode == MTD_OPS_AUTO_OOB)
    {
        len = host->ecclayout->oobavail;
    }
    else
    {
        len = mtd->oobsize;
    }

    if (unlikely(ops->ooboffs >= len)) 
    {
        FLASH_DEBUG("%s: attempt to start read outside oob\n",
				__func__);
        return -EINVAL;
    }

    /* Do not allow reads past end of device */
    if (unlikely(from >= mtd->size ||
        ops->ooboffs + readlen > ((mtd->size >> host->page_shift) -
            (from >> host->page_shift)) * len)) 
    {
        FLASH_DEBUG("%s: attempt to read beyond end of device\n",
				__func__);
        return -EINVAL;
    }

    /* Shift to get page */
    page = (int)(from >> host->page_shift);
    while(1) 
    {
        concerto_spi_nand_oob_read(mtd, page, buf);

        len = min(len, readlen);
        buf = concerto_spi_nand_transfer_oob(mtd, buf, ops, len);

        readlen -= len;
        if (!readlen)
            break;

        /* Increment page address */
        page++;
    }

	ops->oobretlen = ops->ooblen - readlen;
	if (mtd->ecc_stats.failed - stats.failed)
    {
        return -EBADMSG;
    }

	return  mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
}


static int concerto_spi_nand_read_oob(struct mtd_info *mtd, loff_t from,
			 struct mtd_oob_ops *ops)
{
    int ret = -ENOTSUPP;

    ops->retlen = 0;

    /* Do not allow reads past end of device */
    if (ops->datbuf && (from + ops->len) > mtd->size) 
    {
        FLASH_DEBUG("%s: attempt to read beyond end of device\n",
            __func__);
        return -EINVAL;
    }

    concerto_spi_nand_get_device(mtd, FL_READING);
    switch(ops->mode) 
    {
        case MTD_OPS_PLACE_OOB:
        case MTD_OPS_AUTO_OOB:
        case MTD_OPS_RAW:
            break;

        default:
            goto out;
    }

    if (!ops->datbuf)
    {
        ret = concerto_spi_nand_do_read_oob(mtd, from, ops);
    }
    else
    {
        ret = concerto_spi_nand_do_read_ops(mtd, from, ops);
    }
 out:
     concerto_spi_nand_release_device(mtd);
    return ret;
}


static int concerto_spi_nand_do_write_ops(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;

    int page;
    uint32_t writelen = ops->len;

	uint32_t oobwritelen = ops->ooblen;
	uint32_t oobmaxlen = ops->mode == MTD_OPS_AUTO_OOB ? mtd->oobavail : mtd->oobsize;

    uint8_t *oob = ops->oobbuf;
    uint8_t *buf = ops->datbuf;

    ops->retlen = 0;
    if (!writelen)
    {
        return 0;
    }

    if((to & (mtd->writesize - 1)) || 
        (ops->len & (mtd->writesize - 1)))
    {
        FLASH_DEBUG("concerto_spi_nand_do_write_ops: "
            "Attempt to write not page aligned data\n");
        return -EINVAL;
    }

    page = (int)(to >> host->page_shift);


	/* Don't allow multipage oob writes with offset */
	if (oob && ops->ooboffs && (ops->ooboffs + ops->ooblen > oobmaxlen))
		return -EINVAL;

    while(1) 
    {
        uint8_t *wbuf = buf;

        if (unlikely(oob))
        {
			size_t len = min(oobwritelen, oobmaxlen);
            oob = concerto_spi_nand_fill_oob(mtd, oob, len, ops);
			oobwritelen -= len;
        }
        else
        {
            memset(host->oob_poi, 0xff, mtd->oobsize);
        }

        concerto_spi_nand_page_write(mtd, page, 0, wbuf, mtd->writesize);

        writelen -= mtd->writesize;
        if (!writelen)
        {
            break;
        }

        buf += mtd->writesize;
        page++;

    }

    ops->retlen = ops->len - writelen;
    if (unlikely(oob))
    {
        ops->oobretlen = ops->ooblen;
    }
    return 0;
}


static int concerto_spi_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t *retlen, const uint8_t *buf)
{
	int ret;
    struct mtd_oob_ops ops;

	/* Do not allow reads past end of device */
	if ((to + len) > mtd->size)
    {   
		return -EINVAL;
    }
	if (!len)
    {   
		return 0;
    }

    concerto_spi_nand_get_device(mtd, FL_WRITING);
	ops.len = len;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ops.mode = 0;

	ret = concerto_spi_nand_do_write_ops(mtd, to, &ops);

	*retlen = ops.retlen;

    concerto_spi_nand_release_device(mtd);
	return ret;
}

static int concerto_spi_nand_do_write_oob(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;
	int  page, len;

	if (ops->mode == MTD_OPS_AUTO_OOB)
    {   
		len = host->ecclayout->oobavail;
    }
	else
    {   
		len = mtd->oobsize;
    }

	/* Do not allow write past end of page */
	if ((ops->ooboffs + ops->ooblen) > len)
    {
		FLASH_DEBUG("concerto_spi_nand_do_write_oob: "
		      "Attempt to write past end of page\n");
		return -EINVAL;
	}

	if (unlikely(ops->ooboffs >= len))
    {
		FLASH_DEBUG("concerto_spi_nand_do_write_oob: "
			"Attempt to start write outside oob\n");
		return -EINVAL;
	}

	/* Do not allow reads past end of device */
	if (unlikely(to >= mtd->size ||
		     ops->ooboffs + ops->ooblen >
			((mtd->size >> host->page_shift) -
			 (to >> host->page_shift)) * len))
	{
		FLASH_DEBUG("concerto_spi_nand_do_write_oob: "
			"Attempt write beyond end of device\n");
		return -EINVAL;
	}

	/* Shift to get page */
	page = (int)(to >> host->page_shift);


	memset(host->oob_poi, 0xff, mtd->oobsize);
	concerto_spi_nand_fill_oob(mtd, ops->oobbuf, ops->ooblen, ops);

    concerto_spi_nand_oob_write(mtd, page, NULL);
	memset(host->oob_poi, 0xff, mtd->oobsize);

	ops->oobretlen = ops->ooblen;

	return 0;
}

static int concerto_spi_nand_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow writes past end of device */
	if (ops->datbuf && (to + ops->len) > mtd->size) 
    {
		FLASH_DEBUG("nand_read_oob: "
		      "Attempt read beyond end of device\n");
		return -EINVAL;
	}

    concerto_spi_nand_get_device(mtd, FL_WRITING);

	switch(ops->mode) 
    {
    	case MTD_OPS_PLACE_OOB:
    	case MTD_OPS_AUTO_OOB:
    	case MTD_OPS_RAW:
    		break;
    	default:
    		goto out;
	}

	if (!ops->datbuf)
    {   
		ret = concerto_spi_nand_do_write_oob(mtd, to, ops);
    }
	else
    {   
		ret = concerto_spi_nand_do_write_ops(mtd, to, ops);
    }
 out:
    concerto_spi_nand_release_device(mtd);
	return ret;
}

static int concerto_spi_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

	struct mtd_oob_ops ops;
	struct erase_info einfo;
    
	uint8_t buf[2] = { 0, 0 };
	int block, ret = 0;

	memset(&einfo, 0, sizeof(einfo));
	einfo.mtd = mtd;
	einfo.addr = ofs;
	einfo.len = 1 << host->erase_shift;
	concerto_spi_nand_erase(mtd, &einfo);

	/* Get block number */
	block = (int)(ofs >> host->erase_shift);

    concerto_spi_nand_get_device(mtd, FL_WRITING);

	ops.datbuf = NULL;
	ops.oobbuf = buf;
	ops.ooboffs = host->badblockpos;
	ops.len = ops.ooblen = 1;
	ops.mode = MTD_OPS_PLACE_OOB;

	ret = concerto_spi_nand_do_write_oob(mtd, ofs, &ops);

    concerto_spi_nand_release_device(mtd);

	if (!ret)
    {
        mtd->ecc_stats.badblocks++;
    }

	return ret;
}

static u32 panther_convert_to_dummy_id(u32 id)
{
    u32 dummy_id = 0xffffff;
    u8 codeID[3];

    codeID[0] = (id >> 8) & 0xFF;
    codeID[1] = id & 0xFF;
    codeID[2] = (id >> 16) & 0xFF;
    dummy_id = (codeID[0] << 16) | (codeID[1] << 8) | codeID[2];  // special ID

    return dummy_id;
}

static u32 panther_convert_to_inverse_id(u32 id)
{
    u32 inverse_id = 0xffffff;
    u8 codeID[3];

    codeID[0] = (id >> 16) & 0xFF;
    codeID[1] = id & 0xFF;
    codeID[2] = (id >> 8) & 0xFF;
    inverse_id = (codeID[0] << 16) | (codeID[1] << 8) | codeID[2];  // special ID

    return inverse_id;
}

static int concerto_spi_nand_flash_probe_common(struct mtd_info *mtd, u32 id)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;
    const struct concerto_spi_nand_flash_params *params;
    u32 dummy_id;   // special case with some chip, need to check
    unsigned int i;

    dummy_id = panther_convert_to_dummy_id(id);
    
    for (i = 0; i < ARRAY_SIZE(concerto_spi_nand_flash_table); i++) 
    {
        params = &concerto_spi_nand_flash_table[i];
        if (params->id == id) 
        {
            break;
        }

        if (params->id == dummy_id) 
        {
            id = dummy_id;
            break;
        }
    }

    // trying compare id table with inverse_id,
    // because big & little endian
    if (i == ARRAY_SIZE(concerto_spi_nand_flash_table)) 
    {
        FLASH_DEBUG("SNF: Unsupported nand flash ID = 0x%x\n", id);
        FLASH_DEBUG("SNF: Unsupported nand flash dummy = ID 0x%x\n", dummy_id);

        id = panther_convert_to_inverse_id(id);
        dummy_id = panther_convert_to_inverse_id(dummy_id);
        FLASH_DEBUG("SNF: inverse ID = 0x%x\n", id);
        FLASH_DEBUG("SNF: inverse dummy ID = 0x%x\n", dummy_id);
    }

    for (i = 0; i < ARRAY_SIZE(concerto_spi_nand_flash_table); i++) 
    {
        params = &concerto_spi_nand_flash_table[i];
        if (params->id == id) 
        {
            break;
        }
        
        if (params->id == dummy_id) 
        {
            id = dummy_id;
            break;
        }
    }
	
    if (i == ARRAY_SIZE(concerto_spi_nand_flash_table)) 
    {
        FLASH_DEBUG("SNF: Unsupported inverse nand flash ID 0x%x\n", id);
        FLASH_DEBUG("SNF: Unsupported inverse nand flash dummy ID 0x%x\n", dummy_id);
        return 1;
    }

    host->name = params->name;
    host->pagesize = mtd->writesize = params->page_size;
    host->oobsize = mtd->oobsize = params->oob_size;
    host->blocksize = mtd->erasesize = params->page_size * params->pages_per_block;
    host->chipsize = mtd->size = host->blocksize * params->nr_blocks;
    mtd->writebufsize = mtd->writesize;   // added for UBI/UBIFS back-port

    host->page_shift = ffs(mtd->writesize) - 1;
    host->erase_shift = ffs(mtd->erasesize) - 1;
    if (host->chipsize & 0xffffffff)
    {
        host->chip_shift = ffs((unsigned)host->chipsize) - 1;
    }
    else
    {
        host->chip_shift = ffs((unsigned)(host->chipsize >> 32)) + 32 - 1;
    }

    host->badblockpos = 0;
    host->oob_poi = host->buffer + host->pagesize;

    host->ecclayout = params->layout;

    host->ecclayout->oobavail = 0;
    for (i = 0; host->ecclayout->oobfree[i].length; i++)
    {
        host->ecclayout->oobavail += host->ecclayout->oobfree[i].length;
    }
    mtd->oobavail = host->ecclayout->oobavail;

    host->read_cmd_dummy_type = params->read_cmd_dummy_type;
    host->read_from_cache_dummy = params->read_from_cache_dummy;

    return 0;
}




/*****************************************************************************/


static int concerto_spi_nand_flash_probe(struct mtd_info *mtd, unsigned int bus, 
        unsigned int cs, unsigned int max_hz, unsigned int spi_mode)
{
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;
    struct spi_slave *spi;
    int ret;
    u32 id = 0xffffff;

    spi = spi_setup_slave(bus, cs, max_hz, spi_mode);
    if(!spi) 
    {
        FLASH_DEBUG("SNF: Failed to set up slave\n");
        return 1;
    }

    ret = spi_claim_bus(spi);
    if(ret) 
    {
        FLASH_DEBUG("SNF: Failed to claim SPI bus: %d\n", ret);
        goto err_claim_bus;
    }

    host->spi = spi;

    ret = concerto_spi_nand_reset(mtd);
    if (ret) 
    {
        FLASH_DEBUG("SNF: cannot reset spi nand flash!!!\n");
        goto err_manufacturer_probe;
    }

    /* Read the ID codes */
    id = concerto_spi_nand_getJedecID(mtd);

    if (id == 0xffffff)
    {
        FLASH_DEBUG("SNF: get JedecID error!!!\n");
        goto err_read_id;
    }

    ret = concerto_spi_nand_flash_probe_common(mtd, id);

    if (ret) 
    {
        FLASH_DEBUG("SNF: Unsupported manufacturer 0x%x\n", id);
        goto err_manufacturer_probe;
    }

    // initial with ecc on
    concerto_spi_nand_internel_ecc_on(mtd, 1);
    // initial config about read/write speed
    if (host->read_ionum == 4 || host->write_ionum == 4)
    {
        concerto_spi_nand_quad_feature_on(mtd, 1);
    }
    else
    {
        concerto_spi_nand_quad_feature_on(mtd, 0);
    }
    FLASH_DEBUG("QE feature status = %d\n", concerto_spi_nand_get_quad_feature(mtd));

    FLASH_DEBUG("SNF: Got idcodes: 0x%06x\n", id);

    FLASH_DEBUG("SNF DRV: Detected %s with page size: %d, oob size: %d, total %d M bytes\n", 
        host->name, host->pagesize, host->oobsize, (u32)(host->chipsize >> 20));

    spi_release_bus(spi);

    return 0;

err_manufacturer_probe:
err_read_id:
    spi_release_bus(spi);
err_claim_bus:
    spi_free_slave(spi);
    return 1;
}



#endif





/*****************************************************************************/

static int concerto_spi_nand_init(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct concerto_snfc_host *host	= chip->priv;

    host->buffer = kmalloc((NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE), GFP_KERNEL);
	if (!host->buffer)
    {
		return 1;
    }
	memset(host->buffer, 0xff, (NAND_MAX_PAGESIZE +	NAND_MAX_OOBSIZE));

    host->read_ionum = 4;
    host->write_ionum = 4;

    host->controller = &host->hwcontrol;
    spin_lock_init(&host->controller->lock);
    init_waitqueue_head(&host->controller->wq);

    host->read_cmd_dummy_type = READ_CMD_DUMMY_END;
    host->read_from_cache_dummy = 0;

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    spi_panther_init_secure();
#endif

	return 0;
}
/*****************************************************************************/

#if 0
static void concerto_spi_nand_set_pinmux(void)
{
    REG32(0xbf156008) = 0;
}
#endif

static void concerto_spi_nand_inithw(struct mtd_info *mtd)
{
    //concerto_spi_nand_set_pinmux();
}

static char* sp = NULL; /* the start position of the string */
static char* panther_spi_nand_strtok(char *str, char *delim)
{
    int i = 0;
    char *p_start = sp;
    int len = strlen(delim);
 
    /* check in the delimiters */
    if (len == 0)
    {
        FLASH_DEBUG("Delimiters are empty...\n");
    }
 
    /* if the original string has nothing left */
    if (!str && !sp)
    {
        return NULL;
    }
 
    /* initialize the sp during the first call */
    if (str && !sp)
    {
        sp = str;
    }
 
    /* find the start of the substring, skip delimiters */
    while (true)
    {
        for (i = 0; i < len;i ++)
        {
            if (*p_start == delim[i])
            {
                p_start++;
                break;
            }
        }
 
        if (i == len)
        {
            sp = p_start;
            break;
        }
    }
 
    /* return NULL if nothing left */
    if (*sp == '\0')
    {
        sp = NULL;
        return sp;
    }
 
    /* find the end of the substring, and
        replace the delimiter with null */
    while (*sp != '\0')
    {
        for(i = 0; i < len; i ++)
        {
            if (*sp == delim[i])
            {
                *sp = '\0';
                break;
            }
        }
        sp++;
        
        if (i < len)
        {
            break;
        }
    }
 
    return p_start;
}

static int panther_spi_is_value_in_array(int val, int *arr, int size)
{
    int i;

    for (i = 0; i < size; i++)
    {
        if (arr[i] == val)
        {
            return 1;
        }
    }

    return 0;
}

static void panther_spi_nand_fit_page_size(struct mtd_info *mtd)
{
    //struct nand_chip *chip = mtd->priv;
    //struct concerto_snfc_host *host	= chip->priv;
    struct mtd_partition *parts = NULL;
    //uint64_t block_size = SNF_MIN_BLOCK_SIZE;
    uint64_t part_shift = 0;
    //uint64_t size = 0;
    //uint64_t offset = 0;
    int ratio;
    int i;
	
    // put boot and cdb partition into correct place,
    // because different NAND flash have different page size
    ratio = 1;//(host->blocksize / block_size);

    if (ratio == 1)
    {
        return;
    }

    for (i = 0; i < ARRAY_SIZE(concerto_spi_nand_partitions); i++)
    {
        parts = &concerto_spi_nand_partitions[i];

        if (i < 2)
        {
            part_shift += (ratio - 1) * parts->size;        
            parts->offset *= ratio;
            parts->size *= ratio;
        }
        else
        {
            parts->offset += part_shift;
        }
    }
    
    parts = &concerto_spi_nand_partitions[4];
    parts->size = (CONFIG_PANTHER_MTD_SIZE - parts->offset);
    parts = &concerto_spi_nand_partitions[2];
    parts->size = (CONFIG_PANTHER_MTD_SIZE - parts->offset);
}

static void panther_spi_nand_adjust_partitions(struct mtd_info *mtd)
{
    int i, j;
    struct nand_chip *chip = mtd->priv;
    struct concerto_snfc_host *host	= chip->priv;
    struct mtd_partition *parts = NULL;
    int count;
    char *token;
    char *delim = ",";
    struct cmdline_param cmd_param;
    u32 bad_block_table[MAX_BAD_BLOCK_NUM];
    int b_start;
    int b_part_len;
    int b_offset;
    char *init_info;
    uint64_t chip_size = CONFIG_PANTHER_MTD_SIZE;
    uint64_t block_size = SNF_MIN_BLOCK_SIZE;
    uint64_t total_shift;
    uint64_t bad_shift;
    int ratio;

    init_info = strstr(arcs_cmdline, "bad_list=");
    if (init_info == NULL)
    {
        FLASH_DEBUG("panther_spi_nand_adjust_partitions() return!!\n");
        return;
    }
    
    sscanf(&init_info[9], "%s", cmd_param.list_bad_block_str);
    init_info = strstr(arcs_cmdline, "img_size=");
    sscanf(&init_info[9], "%x", &cmd_param.img_size);
    count = 0;
    FLASH_DEBUG("cmd_param.img_size = %x\n", cmd_param.img_size);
    FLASH_DEBUG("cmd_param.list_bad_block_str = %s\n", cmd_param.list_bad_block_str);

    token = panther_spi_nand_strtok(cmd_param.list_bad_block_str, delim);
    while (token != NULL)
    {
        sscanf(token, "%d", &bad_block_table[count]);
        FLASH_DEBUG("bad_block_table[%d] = %d\n", count, bad_block_table[count]);
        count++;
        // check whether the bad block list stringhave next token
        token = panther_spi_nand_strtok(NULL, delim);
    }

    ratio = (host->blocksize / block_size);
    
    /* add some bad block information into table
    |*      |<- boot ->|<- kernel/linux ->|<- other partitions...... ->|
    |* =>   |<- boot ->|<- offset_1 ->|<- kernel/linux ->|<- other partitions...... ->|
    |* but in boot code, table only check until kernel size
    |* =>   |<-------    kernel/linux     ------->|
    |* =>   |<- kernel size ->|<- need to check ->|
    */
    b_offset = 0;
    parts = &concerto_spi_nand_partitions[1];
    b_start = count + (parts->offset + cmd_param.img_size) / block_size;
    FLASH_DEBUG("b_start = %d\n", b_start);
    b_part_len = (parts->size - cmd_param.img_size) / block_size;
    FLASH_DEBUG("b_part_len = %d\n", b_part_len);
    while (b_part_len > 0)
    {
        if (concerto_spi_nand_block_is_bad(mtd, (loff_t)((b_start + b_offset) << host->erase_shift)))
        {
            bad_block_table[count] = (b_start + b_offset);
            FLASH_DEBUG("bad_block_table[%d] = %d\n", count, bad_block_table[count]);
            count++;
        }
        else
        {
            b_part_len--;
        }
        
        b_offset++;
    }

    // adjust default partitions array with bad block table
    total_shift = 0;
    for (i = 0; i < ARRAY_SIZE(concerto_spi_nand_partitions); i++)
    {
        // because firmware partition: linux + rootfs, no need to do adjust
        if (i == 2)
        {
            continue;
        }
        
        bad_shift = 0;
        parts = &concerto_spi_nand_partitions[i];
        parts->offset += total_shift;
        FLASH_DEBUG("===============================\n");
        FLASH_DEBUG("parts->name = %s\n", parts->name);
        FLASH_DEBUG("Before Shift.................\n");
        FLASH_DEBUG("parts->offset = 0x%08llx\n", parts->offset);
        FLASH_DEBUG("parts->size = 0x%08llx\n", parts->size);

        // only previous partitions need to shift with bad block information
        b_start = parts->offset / block_size;
        b_part_len = (parts->size / block_size);

        b_offset = 0;
        for (j = 0; j < b_part_len; j++)
        {
            while (panther_spi_is_value_in_array(b_start + b_offset + j, bad_block_table, count))
            {
                bad_shift += SNF_MIN_BLOCK_SIZE * ratio;
                b_offset++;
            }
        }

        if (i != SNF_ADJUST_PART_NUM)
        {
            parts->size += bad_shift;
            total_shift += bad_shift;
        }

        if ((parts->size + parts->offset) > chip_size)
        {
            parts->size = (chip_size - parts->offset);
        }
        
        FLASH_DEBUG("After Shift..................\n");
        FLASH_DEBUG("parts->offset = 0x%08llx\n", parts->offset);
        FLASH_DEBUG("parts->size = 0x%08llx\n", parts->size);
    }

    // #define SNF_FIRMWARE_START  SNF_LINUX_START
    // #define SNF_FIRMWARE_LENTH  (SNF_LINUX_LENTH + SNF_ROOTFS_LENTH)
    parts = &concerto_spi_nand_partitions[2];
    parts->offset = concerto_spi_nand_partitions[3].offset;
    parts->size = (concerto_spi_nand_partitions[3].size + concerto_spi_nand_partitions[4].size);
}

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
extern u8 empty_page_data[0x800];
static void panther_spi_read_encrypt_empty_page(struct mtd_info *mtd)
{
    //struct nand_chip *chip = mtd->priv;
    //struct concerto_snfc_host *host	= chip->priv;

    // read last page of first block
    change_aes_enable_status(DISABLE_SECURE);
    concerto_spi_nand_page_read(mtd, 63, 0, empty_page_data, 0x800);
    change_aes_enable_status(ENABLE_SECURE_OTP);
}
#endif

static int concerto_spi_nand_probe(struct platform_device * pltdev)
{
	int	result = 0;
    u32 boot_type;

	struct concerto_snfc_host *host;
	struct nand_chip  *chip;
	struct mtd_info	  *mtd;

#ifdef CONFIG_MTD_CMDLINE_PARTS
    struct mtd_partition *parts = NULL;
    int nr_parts = 0;
    static const char *part_probes[] = {"cmdlinepart", NULL,};
#endif

	int	size = sizeof(struct concerto_snfc_host) + sizeof(struct nand_chip)
		+ sizeof(struct	mtd_info);

    // check boot from NAND or NOR
    boot_type = otp_get_boot_type();
    if ((boot_type != BOOT_FROM_NAND) && (boot_type != BOOT_FROM_NAND_WITH_OTP))
    {
        return -1;
    }

    FLASH_DEBUG("===============concerto_spi_nand_init start........===============\n");
    
	host = kmalloc(size, GFP_KERNEL);
	if (!host)
	{
		dev_err(&pltdev->dev, "failed to allocate device structure.\n");
		return -ENOMEM;
	}

	memset((char *)host, 0,	size);
	platform_set_drvdata(pltdev, host);

	host->dev  = &pltdev->dev;
	host->chip = chip =	(struct	nand_chip *)&host[1];
	host->mtd  = mtd  =	(struct	mtd_info *)&chip[1];


	mtd->priv  = chip;
	mtd->owner = THIS_MODULE;
	mtd->name  = (char*)(pltdev->name);

    chip->priv = host;

    concerto_spi_nand_inithw(mtd);

	if (concerto_spi_nand_init(mtd))
	{
		dev_err(&pltdev->dev, "failed to allocate device buffer.\n");
		result = -ENOMEM;
		goto err;
	}

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    pdma_init();
    init_spi_data();
#endif
    
    if (concerto_spi_nand_flash_probe(mtd, CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS, 
                                CONFIG_SNF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE))
    {
		dev_err(&pltdev->dev, "failed to spi nand probe.\n");
		result = -ENXIO;
		goto err;
    }

	host->state = FL_READY;

	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->_erase = concerto_spi_nand_erase;
	mtd->_point = NULL;
	mtd->_unpoint = NULL;
	mtd->_read = concerto_spi_nand_read;
	mtd->_write = concerto_spi_nand_write;
	mtd->_read_oob = concerto_spi_nand_read_oob;
	mtd->_write_oob = concerto_spi_nand_write_oob;
	mtd->_sync = NULL;
	mtd->_lock = NULL;
	mtd->_unlock = NULL;
	mtd->_suspend = NULL;
	mtd->_resume = NULL;
	mtd->_block_isbad = concerto_spi_nand_block_is_bad;
	mtd->_block_markbad = concerto_spi_nand_block_markbad;
    mtd->ecclayout = host->ecclayout;
    mtd->name = CONCERTO_SPI_NAND_FLASH_NAME;

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
    panther_spi_nand_fit_page_size(mtd);
    panther_spi_nand_adjust_partitions(mtd);
    result = add_mtd_partitions(mtd, concerto_spi_nand_partitions, ARRAY_SIZE(concerto_spi_nand_partitions));
#endif

#ifdef CONFIG_MTD_PDMA_SPI_FLASH
    panther_spi_read_encrypt_empty_page(mtd);
#endif

    FLASH_DEBUG("===============concerto_spi_nand_init end..........===============\n");

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


void concerto_spi_nand_release(struct mtd_info *mtd)
{
	del_mtd_partitions(mtd);

	del_mtd_device(mtd);
}


/*****************************************************************************/
int	concerto_spi_nand_remove(struct platform_device *pltdev)
{
	struct concerto_snfc_host *host	= platform_get_drvdata(pltdev);

	concerto_spi_nand_release(host->mtd);
	kfree(host);
	platform_set_drvdata(pltdev, NULL);

	return 0;
}
/*****************************************************************************/
static void	concerto_spi_nand_pltdev_release(struct device *dev)
{
}
/*****************************************************************************/
static struct platform_driver concerto_spi_nand_pltdrv =
{
	.driver.name   = "concerto spi nand",
	.probe	= concerto_spi_nand_probe,
	.remove	= concerto_spi_nand_remove,
};
/*****************************************************************************/
static struct platform_device concerto_spi_nand_pltdev =
{
	.name			= "concerto spi nand",
	.id				= -1,

	.dev.platform_data	   = NULL,
	.dev.release		   = concerto_spi_nand_pltdev_release,

	.num_resources	= 0,
	.resource		= NULL,
};

/*****************************************************************************/

static int __init concerto_spi_nand_module_init(void)
{
	int	result = 0;
	
	FLASH_DEBUG("\nMT concerto Spi Nand Flash Controller Device Driver, Version 1.00\n");

	result = platform_driver_register(&concerto_spi_nand_pltdrv);
	if (result < 0)
	{
		return result;
	}

	result = platform_device_register(&concerto_spi_nand_pltdev);
	if (result < 0)
	{
		platform_driver_unregister(&concerto_spi_nand_pltdrv);
		return result;
	}

	return result;
}

/*****************************************************************************/

static void	__exit concerto_spi_nand_module_exit	(void)
{
	platform_driver_unregister(&concerto_spi_nand_pltdrv);
	platform_device_unregister(&concerto_spi_nand_pltdev);
}
/*****************************************************************************/

module_init(concerto_spi_nand_module_init);
module_exit(concerto_spi_nand_module_exit);


/*****************************************************************************/






















