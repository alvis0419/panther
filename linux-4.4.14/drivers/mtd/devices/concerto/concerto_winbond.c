/*
 * Copyright 2008, Network Appliance Inc.
 * Author: Jason McMullan <mcmullan <at> netapp.com>
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>

#include "concerto_spi_nor.h"

#define MAX_READ_IO_NUM     2   /* three speed: 1, 2, 4  */
#define MAX_WRITE_IO_NUM    1   /* TWO speed: 1, 4  */

/* M25Pxx-specific commands */
#define CMD_W25_WREN		0x06	/* Write Enable */
#define CMD_W25_WRDI		0x04	/* Write Disable */
#define CMD_W25_RDSR		0x05	/* Read Status Register */
#define CMD_W25_WRSR		0x01	/* Write Status Register */
#define CMD_W25_READ		0x03	/* Read Data Bytes */
#define CMD_W25_FAST_READ	0x0b	/* Read Data Bytes at Higher Speed */
#define CMD_W25_PP		0x02	/* Page Program */
#define CMD_W25_SE		0x20	/* Sector (4K) Erase */
#define CMD_W25_BE		0xd8	/* Block (64K) Erase */
#define CMD_W25_CE		0xc7	/* Chip Erase */
#define CMD_W25_DP		0xb9	/* Deep Power-down */
#define CMD_W25_RES		0xab	/* Release from DP, and Read Signature */

struct winbond_spi_flash_params 
{
	u16	id;
	/* Log2 of page size in power-of-two mode */
	u8 l2_page_size;
	u16	pages_per_sector;
	u16 sectors_per_block;
	u16 nr_blocks;
	const char	*name;
};

static const struct winbond_spi_flash_params winbond_spi_flash_table[] = 
{
	{
		.id			= 0x3013,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 8,
		.name			= "W25X40",
	},
	{
		.id			= 0x3015,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 32,
		.name			= "W25X16",
	},
	{
		.id			= 0x3016,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 64,
		.name			= "W25X32",
	},
	{
		.id			= 0x3017,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 128,
		.name			= "W25X64",
	},
	{
		.id			= 0x4015,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 32,
		.name			= "W25Q16",
	},
	{
		.id			= 0x4016,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 64,
		.name			= "W25Q32",
	},
	{
		.id			= 0x4017,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 128,
		.name			= "W25Q64",
	},
	{
		.id			= 0x4018,
		.l2_page_size		= 8,
		.pages_per_sector	= 16,
		.sectors_per_block	= 16,
		.nr_blocks		= 256,
		.name			= "W25Q128",
	},
};

int spi_flash_probe_winbond(struct mtd_info *mtd, u8 *idcode)
{
    struct concerto_sfc_host *host	= mtd->priv;

	const struct winbond_spi_flash_params *params = NULL;
	unsigned int i;
    unsigned int jedec;
    unsigned int jedec_inv; // inverse of jecdec for endian(big/little)

	jedec = (idcode[1] << 8) | idcode[2];
    jedec_inv = idcode[1] | (idcode[2] << 8);

	for (i = 0; i < ARRAY_SIZE(winbond_spi_flash_table); i++) 
    {
		params = &winbond_spi_flash_table[i];
		if (params->id == jedec || params->id == jedec_inv) 
        {
			break;
		}
	}

	if (i == ARRAY_SIZE(winbond_spi_flash_table)) 
    {
		FLASH_DEBUG("SF DRV ERROR: Unsupported Winbond ID %02x%02x\n",
				idcode[1], idcode[2]);
		return 1;
	}

    host->name = params->name;
    host->pagesize = 1 << params->l2_page_size;
    host->blocksize = host->pagesize * params->pages_per_sector * params->sectors_per_block;
    host->chipsize = host->blocksize * params->nr_blocks;

    host->byte_mode = 3;
    if (host->read_ionum > MAX_READ_IO_NUM)
    {
        host->read_ionum = MAX_READ_IO_NUM;
    }
    if (host->write_ionum > MAX_WRITE_IO_NUM)
    {
        host->write_ionum = MAX_WRITE_IO_NUM;
    }
    
    host->erase_cmd = CMD_W25_BE;
    if (host->read_ionum == 2)
    {
        host->fastr_cmd = CMD_W25_FAST_READ;
    }
    else
    {
        host->fastr_cmd = CMD_W25_READ;
    }
    host->write_cmd = CMD_W25_PP;

	mtd->_erase = concerto_spi_nor_erase;
	mtd->_read = concerto_spi_nor_read;
	mtd->_write = concerto_spi_nor_write;

	return 0;
}
