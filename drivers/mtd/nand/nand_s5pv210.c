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

#ifdef CONFIG_S5P_NAND_BOOT
#define __nand_boot_init __bare_init
#else
#define __nand_boot_init
#endif

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

#ifdef CONFIG_S5P_NAND_BOOT
/* Nand flash oob definition for MLC 8k page using 16-bit ecc */
static struct nand_ecclayout s3c_nand_oob_mlc_512 = {
	.eccbytes = 448,
	.eccpos = {
		36, 37, 38, 39, 
		40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 
		50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 
		60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 
		70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 
		80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 
		90, 91, 92, 93, 94, 95,	96, 97, 98, 99, 
		100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
		110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
		130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
		150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
		170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
		190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209,
		210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
		220, 221, 222, 223, 224, 225, 226, 227, 228, 229,
		230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249,
		250, 251, 252, 253, 254, 255, 256, 257, 258, 259,
		260, 261, 262, 263, 264, 265, 266, 267, 268, 269,
		270, 271, 272, 273, 274, 275, 276, 277, 278, 279,
		280, 281, 282, 283, 284, 285, 286, 287, 288, 289,
		290, 291, 292, 293, 294, 295, 296, 297, 298, 299,
		300, 301, 302, 303, 304, 305, 306, 307, 308, 309,
		310, 311, 312, 313, 314, 315, 316, 317, 318, 319,
		320, 321, 322, 323, 324, 325, 326, 327, 328, 329,
		330, 331, 332, 333, 334, 335, 336, 337, 338, 339,
		340, 341, 342, 343, 344, 345, 346, 347, 348, 349,
		350, 351, 352, 353, 354, 355, 356, 357, 358, 359,
		360, 361, 362, 363, 364, 365, 366, 367, 368, 369,
		370, 371, 372, 373, 374, 375, 376, 377, 378, 379,
		380, 381, 382, 383, 384, 385, 386, 387, 388, 389,
		390, 391, 392, 393, 394, 395, 396, 397, 398, 399,
		400, 401, 402, 403, 404, 405, 406, 407, 408, 409,
		410, 411, 412, 413, 414, 415, 416, 417, 418, 419,
		420, 421, 422, 423, 424, 425, 426, 427, 428, 429,
		430, 431, 432, 433, 434, 435, 436, 437, 438, 439,
		440, 441, 442, 443, 444, 445, 446, 447, 448, 449,
		450, 451, 452, 453, 454, 455, 456, 457, 458, 459,
		460, 461, 462, 463, 464, 465, 466, 467, 468, 469,
		470, 471, 472, 473, 474, 475, 476, 477, 478, 479,
		480, 481, 482, 483,
	},
	.oobfree = {
		{
		.offset = 4,
		.length = 32
		}
	}
};
#else
/* Nand flash oob definition for MLC 8k page using 16-bit ecc */
static struct nand_ecclayout s3c_nand_oob_mlc_512 = {
	.eccbytes = 208,
	.eccpos = {
		12, 13, 14, 15, 16, 17, 18, 19, 
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
		30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 
		40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 
		50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 
		60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 
		70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 
		80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 
		90, 91, 92, 93, 94, 95,	96, 97, 98, 99, 
		100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
		110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
		130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
		150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
		170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
		190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209,
		210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
	},
	.oobfree = {
		{
		.offset = 2,
		.length = 10
		}
	}
};
#endif

/**
 * Issue the specified command to the NAND device
 * @param[in] host Base address of the NAND controller
 * @param[in] cmd Command for NAND flash
 */
static void __nand_boot_init send_cmd(void __iomem *host, uint8_t cmd)
{
	writeb(cmd, host + NFCMMD);
}

/**
 * Issue the specified address to the NAND device
 * @param[in] host Base address of the NAND controller
 * @param[in] addr Address for the NAND flash
 */
static void __nand_boot_init send_addr(void __iomem *host, uint8_t addr)
{
	writeb(addr, host + NFADDR);
}

/**
 * Enable the NAND flash access
 * @param[in] host Base address of the NAND controller
 */
static void __nand_boot_init enable_cs(void __iomem *host)
{
	writel(readl(host + NFCONT) & ~NFCONT_nFCE0, host + NFCONT);
}

/**
 * Disable the NAND flash access
 * @param[in] host Base address of the NAND controller
 */
static void __nand_boot_init disable_cs(void __iomem *host)
{
	writel(readl(host + NFCONT) | NFCONT_nFCE0, host + NFCONT);
}

/**
 * Enable the NAND flash controller
 * @param[in] host Base address of the NAND controller
 * @param[in] timing Timing to access the NAND memory
 */
static void __nand_boot_init enable_nand_controller(void __iomem *host, uint32_t timing)
{
	writel(NFCONT_EN | NFCONT_nFCE0, host + NFCONT);
	writel(timing | NFCONF_ADDRCYCLE, host + NFCONF);
}

/**
 * Diable the NAND flash controller
 * @param[in] host Base address of the NAND controller
 */
static void __nand_boot_init disable_nand_controller(void __iomem *host)
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
 * Wait till 8/16-bit ecc done
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
 * Wait till 8/16-bit ecc encode done
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
 * Wait till 8/16-bit ecc decode done
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
 * Enable 16-bit hardware ecc 
 * @param[in] mtd info structure 
 * @param[in] ecc mode, select encode or decode
 */
static unsigned long ecc_msg_len = 511;
static unsigned long ecc_max_err = 16;
static void s3c_nand_enable_hwecc_16bit(struct mtd_info *mtd, int mode)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct s3c_nand_host *host = nand_chip->priv;
	unsigned long nfconf = readl(host->base + NFCONF);
	unsigned long nfcont = readl(host->base + NFCONT);
	unsigned long nfecccont = NFECCCONT_INITMECC;

	ecc_max_err = 16;
	nfconf &= ~NFCONF_ECCTYPE_MASK;
	nfconf |= NFCONF_ECCTYPE_NONE;
	writel(nfconf, host->base + NFCONF);
	
	nfcont &= ~NFCONT_MECCLOCK;
	writel(nfcont, host->base + NFCONT);	
	
	writel(NFECCSTAT_ENCDONE | NFECCSTAT_DECDONE, host->base + NFECCSTAT);
	writel(NFECCCONF_ECCTYPE_16BIT | (ecc_msg_len << NFECCCONF_MSGLEN_SHIFT), host->base + NFECCCONF);
	
	if(mode == NAND_ECC_WRITE)
		nfecccont |= NFECCCONT_ECCDIR_ENC;
	writel(nfecccont, host->base + NFECCCONT);
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

	ecc_max_err = 8;
	nfconf &= ~NFCONF_ECCTYPE_MASK;
	nfconf |= NFCONF_ECCTYPE_NONE;
	writel(nfconf, host->base + NFCONF);
	
	nfcont &= ~NFCONT_MECCLOCK;
	writel(nfcont, host->base + NFCONT);	
	
	writel(NFECCSTAT_ENCDONE | NFECCSTAT_DECDONE, host->base + NFECCSTAT);
	writel(NFECCCONF_ECCTYPE_8BIT | (ecc_msg_len << NFECCCONF_MSGLEN_SHIFT), host->base + NFECCCONF);
	
	if(mode == NAND_ECC_WRITE)
		nfecccont |= NFECCCONT_ECCDIR_ENC;
	writel(nfecccont, host->base + NFECCCONT);
}

/**
 * Calculate 8/16-bit hardware ecc 
 * @param[in] mtd info structure 
 * @param[in] raw data 
 * @param[out] ecc code
 */
static int s3c_nand_calculate_hwecc(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	struct nand_chip *chip = mtd->priv;
	struct s3c_nand_host *host = chip->priv;
	unsigned long nfecccont = readl(host->base + NFECCCONT);
	unsigned long nfcont = 0; 
	unsigned long nfeccprg[8];

	if(nfecccont & NFECCCONT_ECCDIR_ENC){
		s3c_nand_wait_encdone(host->base);
		
		nfcont = readl(host->base + NFCONT);
		nfcont |= NFCONT_MECCLOCK;
		writel(nfcont, host->base + NFCONT);	
		
		nfeccprg[0] = readl(host->base + NFECCPRGECC0);
		nfeccprg[1] = readl(host->base + NFECCPRGECC1);
		nfeccprg[2] = readl(host->base + NFECCPRGECC2);
		nfeccprg[3] = readl(host->base + NFECCPRGECC3);
		nfeccprg[4] = readl(host->base + NFECCPRGECC4);
		nfeccprg[5] = readl(host->base + NFECCPRGECC5);
		nfeccprg[6] = readl(host->base + NFECCPRGECC6) | 0xFFFF0000;
		memcpy(ecc_code, nfeccprg, chip->ecc.bytes);
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
 * @note: This routine works always on 8/16-bit ECC
 */
static int s3c_nand_correct_data(struct mtd_info *mtd, uint8_t *dat,
				uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct nand_chip *chip = mtd->priv;
	struct s3c_nand_host *host = chip->priv;
	unsigned long nfeccerl[8];
	unsigned long nfeccerp[4];
	unsigned short *loc = (unsigned short *)nfeccerl;
	unsigned char *pat = (unsigned char *)nfeccerp;
	unsigned long nfcont = 0;
	int i, ret = -1;
	int err_cnt;
	
	chip->write_buf(mtd, read_ecc, chip->ecc.bytes);	
	
	nfcont = readl(host->base + NFCONT);
	nfcont |= NFCONT_MECCLOCK;
	writel(nfcont, host->base + NFCONT);
	
	s3c_nand_wait_decdone(host->base);
	s3c_nand_wait_eccbusy(host->base);
	
	if(readl(host->base + NFECCSTAT) & NFECCSTAT_FREEPAGE){
		return 0;
	}
	
	err_cnt = readl(host->base + NFECCSECSTAT) & 0x1f;
	if(! err_cnt){
		return 0;
	}

	if(err_cnt > ecc_max_err){
		printk(KERN_INFO "s3c-nand: ECC uncorrectable error detected\n");
		return -1;
	}
	
	nfeccerl[0] = readl(host->base + NFECCERL0);
	nfeccerl[1] = readl(host->base + NFECCERL1);
	nfeccerl[2] = readl(host->base + NFECCERL2);
	nfeccerl[3] = readl(host->base + NFECCERL3);
	nfeccerl[4] = readl(host->base + NFECCERL4);
	nfeccerl[5] = readl(host->base + NFECCERL5);
	nfeccerl[6] = readl(host->base + NFECCERL6);
	nfeccerl[7] = readl(host->base + NFECCERL7);

	nfeccerp[0] = readl(host->base + NFECCERP0);
	nfeccerp[1] = readl(host->base + NFECCERP1);	
	nfeccerp[2] = readl(host->base + NFECCERP2);
	nfeccerp[3] = readl(host->base + NFECCERP3);	

	for(i = 0; i < err_cnt; i++)
	{
		dat[loc[i]] ^= pat[i];
	}
	//printk(KERN_INFO "s3c-nand: ECC correctable error detected\n");
	ret = err_cnt; 
	
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

#ifdef CONFIG_S5P_NAND_BOOT
/**
 * 16bit hardware ecc based page write function
 * @param[in] mtd info structure
 * @param[in] nand chip info structure
 * @param[out] buffer to store read data
 */
static void s3c_nand_write_page_16bit(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	const uint8_t *p = buf;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
		chip->write_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
	}

	for (i = 0; i < chip->ecc.total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];
	ecc_msg_len = 31;
	chip->write_buf(mtd, chip->oob_poi, 4);
	chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
	chip->write_buf(mtd, chip->oob_poi + 4, 32);
	chip->ecc.calculate(mtd, p, ecc_calc);
	for (i = 0; i < eccbytes; i++)
		chip->oob_poi[eccpos[chip->ecc.total - 1] + i + 1] = ecc_calc[i];
	chip->write_buf(mtd, chip->oob_poi + 4 + 32, mtd->oobsize - 4 - 32);
	ecc_msg_len = 511;
}
#else
/**
 * 16bit hardware ecc based page write function
 * @param[in] mtd info structure
 * @param[in] nand chip info structure
 * @param[out] buffer to store read data
 */
static void s3c_nand_write_page_16bit(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	const uint8_t *p = buf;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
		chip->write_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
	}

	for (i = 0; i < chip->ecc.total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];
	chip->write_buf(mtd, chip->oob_poi, eccpos[chip->ecc.total - 1] + 1);
}
#endif

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
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize, -1);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		chip->read_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
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
	chip->ecc.layout = &s3c_nand_oob_mlc_512;
	chip->ecc.hwctl = s3c_nand_enable_hwecc_16bit;
	chip->ecc.calculate = s3c_nand_calculate_hwecc;
	chip->ecc.correct = s3c_nand_correct_data;
	chip->ecc.write_page = s3c_nand_write_page_16bit;
	chip->ecc.size = 512;
	chip->ecc.bytes = 28;	
	ret = s3c_nand_inithw(host);
	if (ret != 0)
		goto on_error;

	/* Scan to find existence of the device */
	ret = nand_scan(mtd, 1);
	if (ret != 0) {
		ret = -ENXIO;
		goto on_error;
	}

#ifdef CONFIG_S5P_NAND_BOOT
	if(mtd->writesize == 2048){
		chip->ecc.bytes = 4;	
		chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;
		chip->ecc.layout = &s3c_nand_oob_64;
		chip->ecc.hwctl = s3c_nand_enable_hwecc_1bit;
		chip->ecc.calculate = s3c_nand_calculate_hwecc_1bit;
		chip->ecc.correct = s3c_nand_correct_data_1bit;
	}
	else if(mtd->writesize == 8192){
		chip->ecc.bytes = 28;	
		chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;
		chip->ecc.layout = &s3c_nand_oob_mlc_512;
		chip->ecc.hwctl = s3c_nand_enable_hwecc_16bit;
		chip->ecc.calculate = s3c_nand_calculate_hwecc;
		chip->ecc.correct = s3c_nand_correct_data;
		chip->ecc.write_page = s3c_nand_write_page_16bit;
	}
#else
	if(mtd->writesize == 2048){
		chip->ecc.bytes = 13;	
		chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;
		chip->ecc.layout = &s3c_nand_oob_64_8bit;
		chip->ecc.hwctl = s3c_nand_enable_hwecc_8bit;
		chip->ecc.calculate = s3c_nand_calculate_hwecc;
		chip->ecc.correct = s3c_nand_correct_data;
	}
	else if(mtd->writesize == 8192){
		mtd->writesize >>= 1;
		chip->ecc.steps >>= 1;
		chip->page_shift -= 1;
		chip->phys_erase_shift -= 1;
		chip->ecc.bytes = 26;	
		chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;
		chip->ecc.layout = &s3c_nand_oob_mlc_512;
		chip->ecc.hwctl = s3c_nand_enable_hwecc_16bit;
		chip->ecc.calculate = s3c_nand_calculate_hwecc;
		chip->ecc.correct = s3c_nand_correct_data;
		chip->ecc.write_page = s3c_nand_write_page_16bit;

	}
#endif	

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


#ifdef CONFIG_S5P_NAND_BOOT
static void __nand_boot_init wait_while_busy(void __iomem *host)
{
	while(! (readl(host + NFSTAT) & NFSTAT_READY))
	{
	}
}

void __nand_boot_init s3c_nand_load_image(void *dest, int size)
{
	void __iomem *host = (void __iomem *)S3C_NAND_BASE;
	unsigned long omr = readl((void __iomem *)0xE010E100);
	unsigned int page = 0;
	unsigned int pagesize = ((omr & 0x3E) == 2)?2048:4096;
	int i;

	/* Enable the NAND controller */
	enable_nand_controller(host, 0x141 << 4);
	
	/* Reset the NAND device */
	enable_cs(host);	
	send_cmd(host, NAND_CMD_RESET);
	wait_while_busy(host);
	disable_cs(host);

	/* Read from the NAND device */
	do {
		/* Send the address */
		enable_cs(host);
		send_cmd(host, NAND_CMD_READ0);
		send_addr(host, 0); /* collumn part 1 */
		send_addr(host, 0); /* collumn part 2 */
		send_addr(host, page);
		send_addr(host, page >> 8);
		send_addr(host, page >> 16);
		send_cmd(host, NAND_CMD_READSTART);
		wait_while_busy(host);

		/* Copy one page */
		for (i = 0; i < pagesize; i++)
			writeb(readb(host + NFDATA), (void __iomem *)(dest + i));
		disable_cs(host);

		page++;
		dest += pagesize;
		size -= pagesize;
	} while (size >= 0);

	/* Disable the controller */
	disable_nand_controller(host);	
}

#endif
