/*
 * Copyright (C) 2011 Sascha Hauer, Pengutronix
 *
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
 *
 */

#include <common.h>
#include <console.h>
#include <init.h>
#include <driver.h>
#include <io.h>
#include <ns16550.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <mach/omap4-silicon.h>
#include <mach/sdrc.h>
#include <mach/sys_info.h>
#include <mach/syslib.h>
#include <mach/control.h>
#include <linux/err.h>
#include <sizes.h>
#include <partition.h>
#include <nand.h>
#include <asm/mmu.h>
#include <mach/gpio.h>
#include <mach/gpmc.h>
#include <mach/gpmc_nand.h>
#include <mach/omap_hsmmc.h>
#include <mach/omap4-devices.h>
#include <i2c/i2c.h>

static int pcaaxl2_console_init(void)
{
	omap44xx_add_uart3();

	return 0;
}
console_initcall(pcaaxl2_console_init);

static int pcaaxl2_mem_init(void)
{
	omap_add_ram0(SZ_512M);

	omap44xx_add_sram0();

	return 0;
}
mem_initcall(pcaaxl2_mem_init);

static struct gpmc_config net_cfg = {
	.cfg = {
		0x00001000, /* CONF1 */
		0x00080800, /* CONF2 */
		0x00000000, /* CONF3 */
		0x08000800, /* CONF4 */
		0x000a0a0a, /* CONF5 */
		0x000003c2, /* CONF6 */
	},
	.base = 0x2C000000,
	.size = GPMC_SIZE_16M,
};

static void pcaaxl2_network_init(void)
{
	gpmc_cs_config(5, &net_cfg);

	add_ks8851_device(DEVICE_ID_DYNAMIC, net_cfg.base, net_cfg.base + 2,
				IORESOURCE_MEM_16BIT, NULL);
}

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO("twlcore", 0x48),
	},
};

static struct omap_hsmmc_platform_data mmc_device = {
	.f_max = 26000000,
};

#define OMAP4_CONTROL_PBIASLITE			0x4A100600
#define OMAP4_MMC1_PBIASLITE_VMODE		(1<<21)
#define OMAP4_MMC1_PBIASLITE_PWRDNZ		(1<<22)
#define OMAP4_MMC1_PWRDNZ			(1<<26)

static struct gpmc_nand_platform_data nand_plat = {
	.device_width = 16,
	.ecc_mode = OMAP_ECC_BCH8_CODE_HW,
	.nand_cfg = &omap4_nand_cfg,
};

static int pcaaxl2_devices_init(void)
{
	u32 value;

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	omap44xx_add_i2c1(NULL);

	value = readl(OMAP4_CONTROL_PBIASLITE);
	value &= ~OMAP4_MMC1_PBIASLITE_VMODE;
	value |= (OMAP4_MMC1_PBIASLITE_PWRDNZ |	OMAP4_MMC1_PWRDNZ);
	writel(value, OMAP4_CONTROL_PBIASLITE);

	omap44xx_add_mmc1(&mmc_device);

	gpmc_generic_init(0x10);

	pcaaxl2_network_init();

	omap_add_gpmc_nand_device(&nand_plat);

#ifdef CONFIG_PARTITION
	devfs_add_partition("nand0", 0x00000, SZ_128K,
			DEVFS_PARTITION_FIXED, "xload_raw");
	dev_add_bb_dev("xload_raw", "xload");
	devfs_add_partition("nand0", SZ_128K, SZ_512K,
			DEVFS_PARTITION_FIXED, "self_raw");
	dev_add_bb_dev("self_raw", "self0");
	devfs_add_partition("nand0", SZ_128K + SZ_512K, SZ_128K,
			DEVFS_PARTITION_FIXED, "env_raw");
	dev_add_bb_dev("env_raw", "env0");
#endif

	armlinux_set_bootparams((void *)0x80000100);
	armlinux_set_architecture(MACH_TYPE_PCAAXL2);

	return 0;
}
device_initcall(pcaaxl2_devices_init);
