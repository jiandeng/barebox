/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <common.h>
#include <bootsource.h>
#include <environment.h>
#include <init.h>
#include <magicvar.h>

#include <io.h>
#include <mach/generic.h>
#include <mach/imx25-regs.h>
#include <mach/imx35-regs.h>

/* [CTRL][TYPE] */
static const enum bootsource locations[4][4] = {
	{ /* CTRL = WEIM */
		BOOTSOURCE_NOR,
		BOOTSOURCE_UNKNOWN,
		BOOTSOURCE_ONENAND,
		BOOTSOURCE_UNKNOWN,
	}, { /* CTRL == NAND */
		BOOTSOURCE_NAND,
		BOOTSOURCE_NAND,
		BOOTSOURCE_NAND,
		BOOTSOURCE_NAND,
	}, { /* CTRL == ATA, (imx35 only) */
		BOOTSOURCE_UNKNOWN,
		BOOTSOURCE_UNKNOWN, /* might be p-ata */
		BOOTSOURCE_UNKNOWN,
		BOOTSOURCE_UNKNOWN,
	}, { /* CTRL == expansion */
		BOOTSOURCE_MMC, /* note imx25 could also be: movinand, ce-ata */
		BOOTSOURCE_UNKNOWN,
		BOOTSOURCE_I2C,
		BOOTSOURCE_SPI,
	}
};

/*
 * Saves the boot source media into the $bootsource environment variable
 *
 * This information is useful for barebox init scripts as we can then easily
 * use a kernel image stored on the same media that we launch barebox with
 * (for example).
 *
 * imx25 and imx35 can boot into barebox from several media such as
 * nand, nor, mmc/sd cards, serial roms. "mmc" is used to represent several
 * sources as its impossible to distinguish between them.
 *
 * Some sources such as serial roms can themselves have 3 different boot
 * possibilities (i2c1, i2c2 etc). It is assumed that any board will
 * only be using one of these at any one time.
 *
 * Note also that I suspect that the boot source pins are only sampled at
 * power up.
 */
static void imx25_35_boot_save_loc(unsigned int ctrl, unsigned int type)
{
	enum bootsource src;

	src = locations[ctrl][type];

	bootsource_set(src);
}

void imx25_boot_save_loc(void __iomem *ccm_base)
{
	uint32_t val;

	val = readl(ccm_base + MX25_CCM_RCSR);
	imx25_35_boot_save_loc((val >> MX25_CCM_RCSR_MEM_CTRL_SHIFT) & 0x3,
			       (val >> MX25_CCM_RCSR_MEM_TYPE_SHIFT) & 0x3);
}

void imx35_boot_save_loc(void __iomem *ccm_base)
{
	uint32_t val;

	val = readl(ccm_base + MX35_CCM_RCSR);
	imx25_35_boot_save_loc((val >> MX35_CCM_RCSR_MEM_CTRL_SHIFT) & 0x3,
			       (val >> MX35_CCM_RCSR_MEM_TYPE_SHIFT) & 0x3);
}

#define IMX27_SYSCTRL_GPCR	0x18
#define IMX27_GPCR_BOOT_SHIFT			16
#define IMX27_GPCR_BOOT_MASK			(0xf << IMX27_GPCR_BOOT_SHIFT)
#define IMX27_GPCR_BOOT_UART_USB		0
#define IMX27_GPCR_BOOT_8BIT_NAND_2k		2
#define IMX27_GPCR_BOOT_16BIT_NAND_2k		3
#define IMX27_GPCR_BOOT_16BIT_NAND_512		4
#define IMX27_GPCR_BOOT_16BIT_CS0		5
#define IMX27_GPCR_BOOT_32BIT_CS0		6
#define IMX27_GPCR_BOOT_8BIT_NAND_512		7

void imx27_boot_save_loc(void __iomem *sysctrl_base)
{
	enum bootsource src;
	uint32_t val;

	val = readl(sysctrl_base + IMX27_SYSCTRL_GPCR);
	val &= IMX27_GPCR_BOOT_MASK;
	val >>= IMX27_GPCR_BOOT_SHIFT;

	switch (val) {
	case IMX27_GPCR_BOOT_UART_USB:
		src = BOOTSOURCE_SERIAL;
		break;
	case IMX27_GPCR_BOOT_8BIT_NAND_2k:
	case IMX27_GPCR_BOOT_16BIT_NAND_2k:
	case IMX27_GPCR_BOOT_16BIT_NAND_512:
	case IMX27_GPCR_BOOT_8BIT_NAND_512:
		src = BOOTSOURCE_NAND;
		break;
	default:
		src = BOOTSOURCE_NOR;
		break;
	}

	bootsource_set(src);
}

#define IMX51_SRC_SBMR			0x4
#define IMX51_SBMR_BT_MEM_TYPE_SHIFT	7
#define IMX51_SBMR_BT_MEM_CTL_SHIFT	0
#define IMX51_SBMR_BMOD_SHIFT		14

void imx51_boot_save_loc(void __iomem *src_base)
{
	enum bootsource src = BOOTSOURCE_UNKNOWN;
	uint32_t reg;
	unsigned int ctrl, type;

	reg = readl(src_base + IMX51_SRC_SBMR);

	switch ((reg >> IMX51_SBMR_BMOD_SHIFT) & 0x3) {
	case 0:
	case 2:
		/* internal boot */
		ctrl = (reg >> IMX51_SBMR_BT_MEM_CTL_SHIFT) & 0x3;
		type = (reg >> IMX51_SBMR_BT_MEM_TYPE_SHIFT) & 0x3;

		src = locations[ctrl][type];
		break;
	case 1:
		/* reserved */
		src = BOOTSOURCE_UNKNOWN;
		break;
	case 3:
		src = BOOTSOURCE_SERIAL;
		break;

	}

	bootsource_set(src);
}

#define IMX53_SRC_SBMR	0x4
void imx53_boot_save_loc(void __iomem *src_base)
{
	enum bootsource src = BOOTSOURCE_UNKNOWN;
	int instance;
	uint32_t cfg1 = readl(src_base + IMX53_SRC_SBMR);

	switch ((cfg1 & 0xff) >> 4) {
	case 2:
		src = BOOTSOURCE_HD;
		break;
	case 3:
		if (cfg1 & (1 << 3))
			src = BOOTSOURCE_SPI;
		else
			src = BOOTSOURCE_I2C;
		break;
	case 4:
	case 5:
	case 6:
	case 7:
		src = BOOTSOURCE_MMC;
		break;
	default:
		break;
	}

	if (cfg1 & (1 << 7))
		src = BOOTSOURCE_NAND;


	switch (src) {
	case BOOTSOURCE_MMC:
	case BOOTSOURCE_SPI:
	case BOOTSOURCE_I2C:
		instance = (cfg1 >> 21) & 0x3;
		break;
	default:
		instance = 0;
		break;
	}

	bootsource_set(src);
	bootsource_set_instance(instance);
}

#define IMX6_SRC_SBMR1	0x04
#define IMX6_SRC_SBMR2	0x1c

void imx6_boot_save_loc(void __iomem *src_base)
{
	enum bootsource src = BOOTSOURCE_UNKNOWN;
	uint32_t sbmr2 = readl(src_base + IMX6_SRC_SBMR2) >> 24;
	uint32_t cfg1 = readl(src_base + IMX6_SRC_SBMR1) & 0xff;
	uint32_t boot_cfg_4_2_0;
	int boot_mode;

	boot_mode = (sbmr2 >> 24) & 0x3;

	switch (boot_mode) {
	case 0: /* Fuses, fall through */
	case 2: /* internal boot */
		goto internal_boot;
	case 1: /* Serial Downloader */
		src = BOOTSOURCE_SERIAL;
		break;
	case 3: /* reserved */
		break;
	};

	bootsource_set(src);

	return;

internal_boot:

	switch (cfg1 >> 4) {
	case 2:
		src = BOOTSOURCE_HD;
		break;
	case 3:
		boot_cfg_4_2_0 = (cfg1 >> 16) & 0x7;

		if (boot_cfg_4_2_0 > 4)
			src = BOOTSOURCE_I2C;
		else
			src = BOOTSOURCE_SPI;
		break;
	case 4:
	case 5:
	case 6:
	case 7:
		src = BOOTSOURCE_MMC;
		break;
	default:
		break;
	}

	if (cfg1 & (1 << 7))
		src = BOOTSOURCE_NAND;

	bootsource_set(src);

	return;
}
