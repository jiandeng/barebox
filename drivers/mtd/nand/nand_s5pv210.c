/* linux/drivers/mtd/nand/s5pv210.c
 *
 * Copyright (C) 2009 Jian Deng, CITTI
 *
 * Copyright © 2004-2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Samsung S5PV210 NAND driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

//#define CONFIG_S3C_NAND_USE_8BIT_ECC

#include <config.h>
#include <common.h>
#include <driver.h>
#include <malloc.h>
#include <init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <mach/s3c-generic.h>
#include <mach/s3c-iomap.h>
#include <io.h>
#include <asm-generic/errno.h>

/* NAND controller's register */
#define NFCONF     0x00
#define NFCONT     0x04
#define NFCMMD     0x08
#define NFADDR     0x0C
#define NFDATA     0x10
#define NFMECCD0   0x14
#define NFMECCD1   0x18
#define NFSECCD    0x1C
#define NFSBLK     0x20
#define NFEBLK     0x24
#define NFSTAT     0x28
#define NFECCERR0  0x2C
#define NFECCERR1  0x30
#define NFMECC0    0x34
#define NFMECC1    0x38
#define NFSECC     0x3C
#define NFMLCBITPT 0x40
#define NFECCCONF    0x20000
#define NFECCCONT    0x20020
#define NFECCSTAT    0x20030
#define NFECCSECSTAT 0x20040
#define NFECCPRGECC0 0x20090
#define NFECCPRGECC1 0x20094
#define NFECCPRGECC2 0x20098
#define NFECCPRGECC3 0x2009C
#define NFECCPRGECC4 0x200A0
#define NFECCPRGECC5 0x200A4
#define NFECCPRGECC6 0x200A8
#define NFECCERL0    0x200C0
#define NFECCERL1    0x200C4
#define NFECCERL2    0x200C8
#define NFECCERL3    0x200CC
#define NFECCERL4    0x200D0
#define NFECCERL5    0x200D4
#define NFECCERL6    0x200D8
#define NFECCERL7    0x200DC
#define NFECCERP0    0x200F0
#define NFECCERP1    0x200F4
#define NFECCERP2    0x200F8
#define NFECCERP3    0x200FC
#define NFECCCONECC0 0x20110
#define NFECCCONECC1 0x20114
#define NFECCCONECC2 0x20118
#define NFECCCONECC3 0x2011C
#define NFECCCONECC4 0x20120
#define NFECCCONECC5 0x20124
#define NFECCCONECC6 0x20128

/* S3C specific bits */
#define NFSTAT_READY            (1 << 0)
#define NFCONT_EN               (1 << 0)
#define NFCONT_LOCK             (1 << 16)
#define NFCONT_nFCE0            (1 << 1)
#define NFCONT_INITSECC         (1 << 4)
#define NFCONT_INITMECC         (1 << 5)
#define NFCONT_SECCLOCK         (1 << 6)
#define NFCONT_MECCLOCK         (1 << 7)
#define NFCONF_ADDRCYCLE        (1 << 1)
#define NFCONF_PAGESIZE         (1 << 2)
#define NFCONF_MLCFLASH         (1 << 3)
#define NFCONF_MSGLEN           (1 << 25)
#define NFCONF_ECCTYPE_MASK     (3 << 23)
#define NFCONF_ECCTYPE_1BIT     (0 << 23)
#define NFCONF_ECCTYPE_4BIT     (1 << 23)
#define NFCONF_ECCTYPE_NONE     (3 << 23)
#define NFECCCONF_MSGLEN_SHIFT  (16)
#define NFECCCONF_MSGLEN_MASK   (0x3ff << 16)
#define NFECCCONF_ECCTYPE_MASK  (0x0f)
#define NFECCCONF_ECCTYPE_NONE  (0x00)
#define NFECCCONF_ECCTYPE_8BIT  (0x03)
#define NFECCCONF_ECCTYPE_12BIT (0x04)
#define NFECCCONF_ECCTYPE_16BIT (0x05)
#define NFECCCONT_ECCDIR_ENC    (1 << 16)
#define NFECCCONT_INITMECC      (1 << 2)
#define NFECCCONT_RESETECC      (1 << 0)
#define NFECCSTAT_ECCBUSY       (1 << 31)
#define NFECCSTAT_ENCDONE       (1 << 25)
#define NFECCSTAT_DECDONE       (1 << 24)
#define NFECCSTAT_FREEPAGE      (1 << 8)

/* S3C specific const */
#define S3C_NAND_ECC_WAIT_MS    (80)


struct s3c_nand_host {
	struct mtd_info		mtd;
	struct nand_chip	nand;
	struct mtd_partition	*parts;
	struct device_d		*dev;
	void __iomem		*base;
};

/* Nand flash oob definition for SLC 2k page using 1-bit ecc */
static struct nand_ecclayout s3c_nand_oob_64 = {
	.eccbytes = 16,
	.eccpos = {
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55
	},
	.oobfree = {
		{
		.offset = 2,
		.length = 38
		}
	}
};

/* Nand flash oob definition for SLC 2k page using 8-bit ecc */
static struct nand_ecclayout s3c_nand_oob_64_8bit = {
	.eccbytes = 52,
	.eccpos = {
		12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 
		25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
		38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 
		51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
	},
	.oobfree = {
		{
		.offset = 2,
		.length = 10
		}
	}
};

/**
 * Issue the specified command to the NAND device
 * @param[in] host Base address of the NAND controller
 * @param[in] cmd Command for NAND flash
 */
static void send_cmd(void __iomem *host, uint8_t cmd)
{
	writeb(cmd, host + NFCMMD);
}

/**
 * Issue the specified address to the NAND device
 * @param[in] host Base address of the NAND controller
 * @param[in] addr Address for the NAND flash
 */
static void send_addr(void __iomem *host, uint8_t addr)
{
	writeb(addr, host + NFADDR);
}

/**
 * Enable the NAND flash access
 * @param[in] host Base address of the NAND controller
 */
static void enable_cs(void __iomem *host)
{
	writel(readl(host + NFCONT) & ~NFCONT_nFCE0, host + NFCONT);
}

/**
 * Disable the NAND flash access
 * @param[in] host Base address of the NAND controller
 */
static void disable_cs(void __iomem *host)
{
	writel(readl(host + NFCONT) | NFCONT_nFCE0, host + NFCONT);
}

/**
 * Enable the NAND flash controller
 * @param[in] host Base address of the NAND controller
 * @param[in] timing Timing to access the NAND memory
 */
static void enable_nand_controller(void __iomem *host, uint32_t timing)
{
	writel(NFCONT_EN | NFCONT_nFCE0, host + NFCONT);
	writel(timing | NFCONF_ADDRCYCLE, host + NFCONF);
}

/**
 * Diable the NAND flash controller
 * @param[in] host Base address of the NAND controller
 */
static void disable_nand_controller(void __iomem *host)
{
	writel(NFCONT_nFCE0, host + NFCONT);
}

/**
 * Init the NAND flash controller
 * @param[in] host Base address of the NAND controller
 */
static int s3c_nand_inithw(struct s3c_nand_host *host)
{
	/* reset the NAND controller */
	disable_nand_controller(host->base);

	/* reenable the NAND controller */
	enable_nand_controller(host->base, 0x141 << 4);

	return 0;
}

/**
 * Control the chip enable
 * @param[in] mtd info structure
 * @param[in] chipnumber to select, -1 for deselect
 */
static void s3c_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct s3c_nand_host *host = nand_chip->priv;

	if (chip == -1)
		disable_cs(host->base);
	else
		enable_cs(host->base);
}

/*
 * Check the nand ready pin
 * @param[in] mtd info structure
 */
static int s3c_nand_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct s3c_nand_host *host = nand_chip->priv;

	return readl(host->base + NFSTAT) & NFSTAT_READY;
}

/**
 * Access to control-lines function
 * @param[in] mtd info structure
 * @param[in] command
 * @param[in] control line
 */
static void s3c_nand_hwcontrol(struct mtd_info *mtd, int cmd,
					unsigned int ctrl)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct s3c_nand_host *host = nand_chip->priv;

	if (cmd == NAND_CMD_NONE)
		return;
	/*
	* If the CLE should be active, this call is a NAND command
	*/
	if (ctrl & NAND_CLE)
		send_cmd(host->base, cmd);
	/*
	* If the ALE should be active, this call is a NAND address
	*/
	if (ctrl & NAND_ALE)
		send_addr(host->base, cmd);
}

/**
 * Wait till 8-bit ecc done
 * @param[in] host Base address of the NAND controller
 */
static void s3c_nand_wait_eccbusy(void __iomem *host)
{
	int timeout = S3C_NAND_ECC_WAIT_MS;
	
	while(readl(host + NFECCSTAT) & NFECCSTAT_ECCBUSY){
		if(--timeout <= 0){
			printk(KERN_INFO "s3c_nand_wait_eccbusy: timeout\n");
			break;
		}
		udelay(1000);
	}
}

/**
 * Wait till 8-bit ecc encode done
 * @param[in] host Base address of the NAND controller
 */
static void s3c_nand_wait_encdone(void __iomem *host)
{
	int timeout = S3C_NAND_ECC_WAIT_MS;
	
	while(!(readl(host + NFECCSTAT) & NFECCSTAT_ENCDONE)){
		if(--timeout <= 0){
			printk(KERN_INFO "s3c_nand_wait_encdone: timeout\n");
			break;
		}
		udelay(1000);
	}
}

/**
 * Wait till 8-bit ecc decode done
 * @param[in] host Base address of the NAND controller
 */
static void s3c_nand_wait_decdone(void __iomem *host)
{
	int timeout = S3C_NAND_ECC_WAIT_MS;
	
	while(!(readl(host + NFECCSTAT) & NFECCSTAT_DECDONE)){
		if(--timeout <= 0){
			printk(KERN_INFO "s3c_nand_wait_decdone: timeout\n");
			break;
		}
		udelay(1000);
	}
}

/**
 * Enable 8-bit hardware ecc 
 * @param[in] mtd info structure 
 * @param[in] ecc mode, select encode or decode
 */
static void s3c_nand_enable_hwecc_8bit(struct mtd_info *mtd, int mode)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct s3c_nand_host *host = nand_chip->priv;
	unsigned long nfconf = readl(host->base + NFCONF);
	unsigned long nfcont = readl(host->base + NFCONT);
	unsigned long nfecccont = NFECCCONT_INITMECC;

	nfconf &= ~NFCONF_ECCTYPE_MASK;
	nfconf |= NFCONF_ECCTYPE_NONE;
	writel(nfconf, host->base + NFCONF);
	
	nfcont &= ~NFCONT_MECCLOCK;
	writel(nfcont, host->base + NFCONT);	
	
	writel(NFECCSTAT_ENCDONE | NFECCSTAT_DECDONE, host->base + NFECCSTAT);
	writel(NFECCCONF_ECCTYPE_8BIT | (0x1ff << NFECCCONF_MSGLEN_SHIFT), host->base + NFECCCONF);
	
	if(mode == NAND_ECC_WRITE)
		nfecccont |= NFECCCONT_ECCDIR_ENC;
	writel(nfecccont, host->base + NFECCCONT);
}

/**
 * Calculate 8-bit hardware ecc 
 * @param[in] mtd info structure 
 * @param[in] raw data 
 * @param[out] ecc code
 */
static int s3c_nand_calculate_hwecc_8bit(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	struct nand_chip *chip = mtd->priv;
	struct s3c_nand_host *host = chip->priv;
	unsigned long nfecccont = readl(host->base + NFECCCONT);
	unsigned long nfcont = 0; 
	unsigned long nfeccprg = 0;

	if(nfecccont & NFECCCONT_ECCDIR_ENC){
		s3c_nand_wait_encdone(host->base);
		
		nfcont = readl(host->base + NFCONT);
		nfcont |= NFCONT_MECCLOCK;
		writel(nfcont, host->base + NFCONT);	
		
		nfeccprg = readl(host->base + NFECCPRGECC0);
		ecc_code[0] = (nfeccprg) & 0xff;
		ecc_code[1] = (nfeccprg >> 8) & 0xff;
		ecc_code[2] = (nfeccprg >> 16) & 0xff;
		ecc_code[3] = (nfeccprg >> 24) & 0xff;
		
		nfeccprg = readl(host->base + NFECCPRGECC1);
		ecc_code[4] = (nfeccprg) & 0xff;
		ecc_code[5] = (nfeccprg >> 8) & 0xff;
		ecc_code[6] = (nfeccprg >> 16) & 0xff;
		ecc_code[7] = (nfeccprg >> 24) & 0xff;	

		nfeccprg = readl(host->base + NFECCPRGECC2);
		ecc_code[8] = (nfeccprg) & 0xff;
		ecc_code[9] = (nfeccprg >> 8) & 0xff;
		ecc_code[10] = (nfeccprg >> 16) & 0xff;
		ecc_code[11] = (nfeccprg >> 24) & 0xff;	

		nfeccprg = readl(host->base + NFECCPRGECC3);
		ecc_code[12] = (nfeccprg) & 0xff;
	}

	return 0;
}

/**
 * Check the ECC and try to repair the data if possible
 * @param[in] mtd info structure 
 * @param[inout] dat Pointer to the data buffer that might contain a bit error
 * @param[in] read_ecc ECC data from the OOB space
 * @param[in] calc_ecc ECC data calculated from the data
 *
 * @note: This routine works always on a 8-bit ECC
 */
static int s3c_nand_correct_data_8bit(struct mtd_info *mtd, uint8_t *dat,
				uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct nand_chip *chip = mtd->priv;
	struct s3c_nand_host *host = chip->priv;
	unsigned long nfeccstat, nfeccsecstat;
	unsigned long nfeccerl0, nfeccerl1, nfeccerl2, nfeccerl3;
	unsigned long nfeccerp0, nfeccerp1;
	unsigned long nfcont = 0;
	int ret = -1;
	
	chip->write_buf(mtd, read_ecc, chip->ecc.bytes);	
	
	nfcont = readl(host->base + NFCONT);
	nfcont |= NFCONT_MECCLOCK;
	writel(nfcont, host->base + NFCONT);
	
	s3c_nand_wait_decdone(host->base);
	s3c_nand_wait_eccbusy(host->base);
	
	nfeccstat = readl(host->base + NFECCSTAT);
	if(nfeccstat & NFECCSTAT_FREEPAGE){
		return 0;
	}
	
	nfeccsecstat = readl(host->base + NFECCSECSTAT);
	if(!(nfeccsecstat & 0x0f)){
		return 0;
	}
	
	nfeccerl0 = readl(host->base + NFECCERL0);
	nfeccerl1 = readl(host->base + NFECCERL1);
	nfeccerl2 = readl(host->base + NFECCERL2);
	nfeccerl3 = readl(host->base + NFECCERL3);
	nfeccerp0 = readl(host->base + NFECCERP0);
	nfeccerp1 = readl(host->base + NFECCERP1);	
	switch(nfeccsecstat & 0x0f)
	{
		case 8:
			dat[(nfeccerl3 >> 16) & 0x3ff] ^= ((nfeccerp1 >> 24) & 0xff);
		case 7:
			dat[nfeccerl3 & 0x3ff] ^= ((nfeccerp1 >> 16) & 0xff);
		case 6:
			dat[(nfeccerl2 >> 16) & 0x3ff] ^= ((nfeccerp1 >> 8) & 0xff);
		case 5:
			dat[nfeccerl2 & 0x3ff] ^= (nfeccerp1 & 0xff);
		case 4:
			dat[(nfeccerl1 >> 16) & 0x3ff] ^= ((nfeccerp0 >> 24) & 0xff);
		case 3:
			dat[nfeccerl1 & 0x3ff] ^= ((nfeccerp0 >> 16) & 0xff);
		case 2:
			dat[(nfeccerl0 >> 16) & 0x3ff] ^= ((nfeccerp0 >> 8) & 0xff);
		case 1:
			dat[nfeccerl0 & 0x3ff] ^= (nfeccerp0 & 0xff);
			printk(KERN_INFO "s3c-nand: ECC correctable error detected\n");
			ret = nfeccsecstat & 0x0f;
			break;
		default:
			printk(KERN_INFO "s3c-nand: ECC uncorrectable error detected\n");
			ret = -1;
			break;
	}
	
	return ret;
}

/**
 * Enable 1-bit hardware ecc 
 * @param[in] mtd info structure 
 * @param[in] ecc mode, select encode or decode
 */
static void s3c_nand_enable_hwecc_1bit(struct mtd_info *mtd, int mode)
{
	struct nand_chip *chip = mtd->priv;
	struct s3c_nand_host *host = chip->priv;
	unsigned long nfconf = readl(host->base + NFCONF);
	unsigned long nfcont = readl(host->base + NFCONT);

	nfconf &= ~NFCONF_ECCTYPE_MASK;
	nfconf |= NFCONF_ECCTYPE_1BIT;
	nfconf &= ~NFCONF_MSGLEN;
	writel(nfconf, host->base + NFCONF);

	nfcont &= ~NFCONT_MECCLOCK;
	writel(nfcont, host->base + NFCONT);	

	nfcont |= NFCONT_INITMECC;
	writel(nfcont, host->base + NFCONT);	
}

/**
 * Calculate 1-bit hardware ecc 
 * @param[in] mtd info structure 
 * @param[in] raw data 
 * @param[out] ecc code
 */
static int s3c_nand_calculate_hwecc_1bit(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	struct nand_chip *chip = mtd->priv;
	struct s3c_nand_host *host = chip->priv;
	unsigned long nfcont = readl(host->base + NFCONT);
	unsigned long nfmecc0 = readl(host->base + NFMECC0);

	nfcont |= NFCONT_MECCLOCK;
	writel(nfcont, host->base + NFCONT);	

	ecc_code[0] = (nfmecc0) & 0xff;
	ecc_code[1] = (nfmecc0 >> 8) & 0xff;
	ecc_code[2] = (nfmecc0 >> 16) & 0xff;
	ecc_code[3] = (nfmecc0 >> 24) & 0xff;

	return 0;
}

/**
 * Check the ECC and try to repair the data if possible
 * @param[in] mtd info structure 
 * @param[inout] dat Pointer to the data buffer that might contain a bit error
 * @param[in] read_ecc ECC data from the OOB space
 * @param[in] calc_ecc ECC data calculated from the data
 * @return 0 no error, 1 repaired error, -1 no way...
 *
 * @note: This routine works always on a 1-bit ECC
 */
static int s3c_nand_correct_data_1bit(struct mtd_info *mtd, uint8_t *dat,
				uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct s3c_nand_host *host = nand_chip->priv;

	int ret = -1;
	unsigned long eccerr = 0;

	writel((read_ecc[1] << 16) | read_ecc[0], host->base + NFMECCD0);
	writel((read_ecc[3] << 16) | read_ecc[2], host->base + NFMECCD1);
	
	eccerr = readl(host->base + NFECCERR0);
	switch(eccerr & 0x03)
	{
		case 0:
			ret = 0;
			break;
            	case 1: 
                	printk("s3c-nand: 1 bit error detected at byte %ld, correcting from "  
                        	"0x%02x ", (eccerr >> 7) & 0x7ff, dat[(eccerr >> 7) & 0x7ff]);  
                	dat[(eccerr >> 7) & 0x7ff] ^= (1 << ((eccerr >> 4) & 0x7));  
                	printk("to 0x%02x...OK\n", dat[(eccerr >> 7) & 0x7ff]);  
			ret = 0;
			break;
		case 2:
		case 3:
                	printk("s3c-nand: ECC uncorrectable error detected\n");  
			ret = -1;	
			break;
	}

	return ret;
}

/**
 * Hardware ecc based page read function
 * @param[in] mtd info structure
 * @param[in] nand chip info structure
 * @param[out] buffer to store read data
 */
static int s3c_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
//	int col = 0;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize, -1);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
//	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize, col += eccsize) {
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

//		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, col, -1);
		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		chip->read_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
//		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize + eccpos[i], -1); 
//		chip->read_buf(mtd, &ecc_code[i], eccbytes); // Reading ecc code from oob is replaced by dummy writing
		stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);

		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}

	return 0;
}


static int s3c_nand_probe(struct device_d *dev)
{
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct s3c_nand_host *host;
	int ret;

	/* Allocate memory for MTD device structure and private data */
	host = kzalloc(sizeof(struct s3c_nand_host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->dev = dev;
	host->base = dev_request_mem_region(dev, 0);

	/* structures must be linked */
	chip = &host->nand;
	mtd = &host->mtd;
	mtd->priv = chip;
	mtd->parent = dev;

	/* init the default settings */

	/* 50 us command delay time */
	chip->chip_delay = 50;
	chip->priv = host;

	chip->IO_ADDR_R = host->base + NFDATA;
	chip->IO_ADDR_W = host->base + NFDATA;
	chip->cmd_ctrl = s3c_nand_hwcontrol;
	chip->dev_ready = s3c_nand_device_ready;
	chip->select_chip = s3c_nand_select_chip;
	chip->ecc.read_page = s3c_nand_read_page;
	
	/* we are using the hardware ECC feature of this device */
	chip->ecc.mode = NAND_ECC_HW;
#ifdef CONFIG_S3C_NAND_USE_8BIT_ECC
	chip->ecc.bytes = 13;	
	chip->ecc.size = 512;
	chip->ecc.layout = &s3c_nand_oob_64_8bit;
	chip->ecc.hwctl = s3c_nand_enable_hwecc_8bit;
	chip->ecc.calculate = s3c_nand_calculate_hwecc_8bit;
	chip->ecc.correct = s3c_nand_correct_data_8bit;
#else
	chip->ecc.bytes = 4;	
	chip->ecc.size = 512;
	chip->ecc.layout = &s3c_nand_oob_64;
	chip->ecc.hwctl = s3c_nand_enable_hwecc_1bit;
	chip->ecc.calculate = s3c_nand_calculate_hwecc_1bit;
	chip->ecc.correct = s3c_nand_correct_data_1bit;
#endif	

	ret = s3c_nand_inithw(host);
	if (ret != 0)
		goto on_error;

	/* Scan to find existence of the device */
	ret = nand_scan(mtd, 1);
	if (ret != 0) {
		ret = -ENXIO;
		goto on_error;
	}

	return add_mtd_device(mtd, "nand");

on_error:
	free(host);
	return ret;
}

static struct driver_d s3c_nand_driver = {
	.name  = "s3c_nand",
	.probe = s3c_nand_probe,
};

/*
 * Main initialization routine
 * @return 0 if successful; non-zero otherwise
 */
static int __init s3c_nand_init(void)
{
	return platform_driver_register(&s3c_nand_driver);
}

device_initcall(s3c_nand_init);

