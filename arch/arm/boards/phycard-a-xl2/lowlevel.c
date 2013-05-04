/*
 * (C) Copyright 2004-2009
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <common.h>
#include <io.h>
#include <sizes.h>
#include <mach/omap4-mux.h>
#include <mach/omap4-silicon.h>
#include <mach/omap4-clock.h>
#include <mach/syslib.h>
#include <asm/barebox-arm.h>
#include <asm/barebox-arm-head.h>

#define TPS62361_VSEL0_GPIO    7

void set_muxconf_regs(void);

static const struct ddr_regs ddr_regs_mt42L64M64_25_400_mhz = {
	.tim1           = 0x0EEB0662,
	.tim2           = 0x20370DD2,
	.tim3           = 0x00BFC33F,
	.phy_ctrl_1     = 0x849FF408,
	.ref_ctrl       = 0x00000618,
	.config_init    = 0x80001AB1,
	.config_final   = 0x80001AB1,
	.zq_config      = 0xd0093215,
	.mr1            = 0x83,
	.mr2            = 0x4
};

static noinline void pcaaxl2_init_lowlevel(void)
{
	struct dpll_param core = OMAP4_CORE_DPLL_PARAM_19M2_DDR400;
	struct dpll_param mpu44xx = OMAP4_MPU_DPLL_PARAM_19M2_MPU1000;
	struct dpll_param mpu4460 = OMAP4_MPU_DPLL_PARAM_19M2_MPU920;
	struct dpll_param iva = OMAP4_IVA_DPLL_PARAM_19M2;
	struct dpll_param per = OMAP4_PER_DPLL_PARAM_19M2;
	struct dpll_param abe = OMAP4_ABE_DPLL_PARAM_19M2;
	struct dpll_param usb = OMAP4_USB_DPLL_PARAM_19M2;

	set_muxconf_regs();

	omap4_ddr_init(&ddr_regs_mt42L64M64_25_400_mhz, &core);

	/* Set VCORE1 = 1.3 V, VCORE2 = VCORE3 = 1.21V */
	omap4_scale_vcores(TPS62361_VSEL0_GPIO);

	writel(CM_SYS_CLKSEL_19M2, CM_SYS_CLKSEL);

	/* Configure all DPLL's at 100% OPP */
	if (omap4_revision() < OMAP4460_ES1_0)
		omap4_configure_mpu_dpll(&mpu44xx);
	else
		omap4_configure_mpu_dpll(&mpu4460);

	omap4_configure_iva_dpll(&iva);
	omap4_configure_per_dpll(&per);
	omap4_configure_abe_dpll(&abe);
	omap4_configure_usb_dpll(&usb);

	/* Enable all clocks */
	omap4_enable_all_clocks();

	sr32(0x4A30a31C, 8, 1, 0x1);  /* enable software ioreq */
	sr32(0x4A30a31C, 1, 2, 0x0);  /* set for sys_clk (19.2MHz) */
	sr32(0x4A30a31C, 16, 4, 0x0); /* set divisor to 1 */
	sr32(0x4A30a110, 0, 1, 0x1);  /* set the clock source to active */
	sr32(0x4A30a110, 2, 2, 0x3);  /* enable clocks */
}

void barebox_arm_reset_vector(void)
{
	arm_cpu_lowlevel_init();

	if (get_pc() > 0x80000000)
		goto out;

	arm_setup_stack(0x4030d000);

	pcaaxl2_init_lowlevel();
out:
	barebox_arm_entry(0x80000000, SZ_512M, 0);
}
