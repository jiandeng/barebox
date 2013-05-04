/*
 * esdctl.c - i.MX sdram controller functions
 *
 * Copyright (c) 2012 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <io.h>
#include <sizes.h>
#include <init.h>
#include <asm/barebox-arm.h>
#include <asm/memory.h>
#include <mach/esdctl.h>
#include <mach/esdctl-v4.h>
#include <mach/imx1-regs.h>
#include <mach/imx21-regs.h>
#include <mach/imx25-regs.h>
#include <mach/imx27-regs.h>
#include <mach/imx31-regs.h>
#include <mach/imx35-regs.h>
#include <mach/imx51-regs.h>
#include <mach/imx53-regs.h>

struct imx_esdctl_data {
	unsigned long base0;
	unsigned long base1;
	void (*add_mem)(void *esdctlbase, struct imx_esdctl_data *);
};

/*
 * v1 - found on i.MX1
 */
static inline unsigned long imx_v1_sdram_size(void __iomem *esdctlbase, int num)
{
	void __iomem *esdctl = esdctlbase + (num ? 4 : 0);
	u32 ctlval = readl(esdctl);
	unsigned long size;
	int rows, cols, width = 2, banks = 4;

	if (!(ctlval & ESDCTL0_SDE))
		/* SDRAM controller disabled, so no RAM here */
		return 0;

	rows = ((ctlval >> 24) & 0x3) + 11;
	cols = ((ctlval >> 20) & 0x3) + 8;

	if (ctlval & (1 << 17))
		width = 4;

	size = (1 << cols) * (1 << rows) * banks * width;

	if (size > SZ_64M)
		size = SZ_64M;

	return size;
}

/*
 * v2 - found on i.MX25, i.MX27, i.MX31 and i.MX35
 */
static inline unsigned long imx_v2_sdram_size(void __iomem *esdctlbase, int num)
{
	void __iomem *esdctl = esdctlbase + (num ? IMX_ESDCTL1 : IMX_ESDCTL0);
	u32 ctlval = readl(esdctl);
	unsigned long size;
	int rows, cols, width = 2, banks = 4;

	if (!(ctlval & ESDCTL0_SDE))
		/* SDRAM controller disabled, so no RAM here */
		return 0;

	rows = ((ctlval >> 24) & 0x7) + 11;
	cols = ((ctlval >> 20) & 0x3) + 8;

	if ((ctlval & ESDCTL0_DSIZ_MASK) == ESDCTL0_DSIZ_31_0)
		width = 4;

	size = (1 << cols) * (1 << rows) * banks * width;

	if (size > SZ_256M)
		size = SZ_256M;

	return size;
}

/*
 * v3 - found on i.MX51
 */
static inline unsigned long imx_v3_sdram_size(void __iomem *esdctlbase, int num)
{
	unsigned long size;

	size = imx_v2_sdram_size(esdctlbase, num);

	if (readl(esdctlbase + IMX_ESDMISC) & (1 << 6))
		size *= 2;

	if (size > SZ_256M)
		size = SZ_256M;

	return size;
}

/*
 * v4 - found on i.MX53
 */
static inline unsigned long imx_v4_sdram_size(void __iomem *esdctlbase, int cs)
{
	u32 ctlval = readl(esdctlbase + ESDCTL_V4_ESDCTL0);
	u32 esdmisc = readl(esdctlbase + ESDCTL_V4_ESDMISC);
	unsigned long size;
	int rows, cols, width = 2, banks = 8;

	if (cs == 0 && !(ctlval & ESDCTL_V4_ESDCTLx_SDE0))
		return 0;
	if (cs == 1 && !(ctlval & ESDCTL_V4_ESDCTLx_SDE1))
		return 0;
	/* one 2GiB cs, memory is returned for cs0 only */
	if (cs == 1 && (esdmisc & ESDCTL_V4_ESDMISC_ONE_CS))
		return 0;
	rows = ((ctlval >> 24) & 0x7) + 11;

	cols = (ctlval >> 20) & 0x7;
	if (cols == 3)
		cols = 8;
	else if (cols == 4)
		cols = 12;
	else
		cols += 9;

	if (ctlval & ESDCTL_V4_ESDCTLx_DSIZ_32B)
		width = 4;

	if (esdmisc & ESDCTL_V4_ESDMISC_BANKS_4)
		banks = 4;

	size = (1 << cols) * (1 << rows) * banks * width;

	return size;
}

static void add_mem(unsigned long base0, unsigned long size0,
		unsigned long base1, unsigned long size1)
{
	debug("%s: cs0 base: 0x%08lx cs0 size: 0x%08lx\n", __func__, base0, size0);
	debug("%s: cs1 base: 0x%08lx cs1 size: 0x%08lx\n", __func__, base1, size1);

	if (base0 + size0 == base1 && size1 > 0) {
		/*
		 * concatenate both chip selects to a single bank
		 */
		arm_add_mem_device("ram0", base0, size0 + size1);

		return;
	}

	if (size0)
		arm_add_mem_device("ram0", base0, size0);

	if (size1)
		arm_add_mem_device(size0 ? "ram1" : "ram0", base1, size1);
}

/*
 * On i.MX27, i.MX31 and i.MX35 the second chipselect is enabled by reset default.
 * This setting makes it impossible to detect the correct SDRAM size on
 * these SoCs. We disable the chipselect if this reset default setting is
 * found. This of course leads to incorrect SDRAM detection on boards which
 * really have this reset default as a valid setting. If you have such a
 * board drop a mail to search for a solution.
 */
#define ESDCTL1_RESET_DEFAULT 0x81120080

static inline void imx_esdctl_v2_disable_default(void *esdctlbase)
{
	u32 ctlval = readl(esdctlbase + IMX_ESDCTL1);

	if (ctlval == ESDCTL1_RESET_DEFAULT) {
		ctlval &= ~(1 << 31);
		writel(ctlval, esdctlbase + IMX_ESDCTL1);
	}
}

static void imx_esdctl_v1_add_mem(void *esdctlbase, struct imx_esdctl_data *data)
{
	add_mem(data->base0, imx_v1_sdram_size(esdctlbase, 0),
			data->base1, imx_v1_sdram_size(esdctlbase, 1));
}

static void imx_esdctl_v2_add_mem(void *esdctlbase, struct imx_esdctl_data *data)
{
	add_mem(data->base0, imx_v2_sdram_size(esdctlbase, 0),
			data->base1, imx_v2_sdram_size(esdctlbase, 1));
}

static void imx_esdctl_v2_bug_add_mem(void *esdctlbase, struct imx_esdctl_data *data)
{
	imx_esdctl_v2_disable_default(esdctlbase);

	add_mem(data->base0, imx_v2_sdram_size(esdctlbase, 0),
			data->base1, imx_v2_sdram_size(esdctlbase, 1));
}

static void imx_esdctl_v3_add_mem(void *esdctlbase, struct imx_esdctl_data *data)
{
	add_mem(data->base0, imx_v3_sdram_size(esdctlbase, 0),
			data->base1, imx_v3_sdram_size(esdctlbase, 1));
}

static void imx_esdctl_v4_add_mem(void *esdctlbase, struct imx_esdctl_data *data)
{
	add_mem(data->base0, imx_v4_sdram_size(esdctlbase, 0),
			data->base1, imx_v4_sdram_size(esdctlbase, 1));
}

static int imx_esdctl_probe(struct device_d *dev)
{
	struct imx_esdctl_data *data;
	int ret;
	void *base;

	ret = dev_get_drvdata(dev, (unsigned long *)&data);
	if (ret)
		return ret;

	base = dev_request_mem_region(dev, 0);
	if (!base)
		return -ENOMEM;

	data->add_mem(base, data);

	return 0;
}

static __maybe_unused struct imx_esdctl_data imx1_data = {
	.base0 = MX1_CSD0_BASE_ADDR,
	.base1 = MX1_CSD1_BASE_ADDR,
	.add_mem = imx_esdctl_v1_add_mem,
};

static __maybe_unused struct imx_esdctl_data imx25_data = {
	.base0 = MX25_CSD0_BASE_ADDR,
	.base1 = MX25_CSD1_BASE_ADDR,
	.add_mem = imx_esdctl_v2_add_mem,
};

static __maybe_unused struct imx_esdctl_data imx27_data = {
	.base0 = MX27_CSD0_BASE_ADDR,
	.base1 = MX27_CSD1_BASE_ADDR,
	.add_mem = imx_esdctl_v2_bug_add_mem,
};

static __maybe_unused struct imx_esdctl_data imx31_data = {
	.base0 = MX31_CSD0_BASE_ADDR,
	.base1 = MX31_CSD1_BASE_ADDR,
	.add_mem = imx_esdctl_v2_bug_add_mem,
};

static __maybe_unused struct imx_esdctl_data imx35_data = {
	.base0 = MX35_CSD0_BASE_ADDR,
	.base1 = MX35_CSD1_BASE_ADDR,
	.add_mem = imx_esdctl_v2_bug_add_mem,
};

static __maybe_unused struct imx_esdctl_data imx51_data = {
	.base0 = MX51_CSD0_BASE_ADDR,
	.base1 = MX51_CSD1_BASE_ADDR,
	.add_mem = imx_esdctl_v3_add_mem,
};

static __maybe_unused struct imx_esdctl_data imx53_data = {
	.base0 = MX53_CSD0_BASE_ADDR,
	.base1 = MX53_CSD1_BASE_ADDR,
	.add_mem = imx_esdctl_v4_add_mem,
};

static struct platform_device_id imx_esdctl_ids[] = {
#ifdef CONFIG_ARCH_IMX1
	{
		.name = "imx1-sdramc",
		.driver_data = (unsigned long)&imx1_data,
	},
#endif
#ifdef CONFIG_ARCH_IMX25
	{
		.name = "imx25-esdctl",
		.driver_data = (unsigned long)&imx25_data,
	},
#endif
#ifdef CONFIG_ARCH_IMX27
	{
		.name = "imx27-esdctl",
		.driver_data = (unsigned long)&imx27_data,
	},
#endif
#ifdef CONFIG_ARCH_IMX31
	{
		.name = "imx31-esdctl",
		.driver_data = (unsigned long)&imx31_data,
	},
#endif
#ifdef CONFIG_ARCH_IMX35
	{
		.name = "imx35-esdctl",
		.driver_data = (unsigned long)&imx35_data,
	},
#endif
#ifdef CONFIG_ARCH_IMX51
	{
		.name = "imx51-esdctl",
		.driver_data = (unsigned long)&imx51_data,
	},
#endif
#ifdef CONFIG_ARCH_IMX53
	{
		.name = "imx53-esdctl",
		.driver_data = (unsigned long)&imx53_data,
	},
#endif
	{
		/* sentinel */
	},
};

static struct driver_d imx_serial_driver = {
	.name   = "imx-esdctl",
	.probe  = imx_esdctl_probe,
	.id_table = imx_esdctl_ids,
};

static int imx_esdctl_init(void)
{
	return platform_driver_register(&imx_serial_driver);
}

mem_initcall(imx_esdctl_init);

/*
 * The i.MX SoCs usually have two SDRAM chipselects. The following
 * SoC specific functions return:
 *
 * - cs0 disabled, cs1 disabled: 0
 * - cs0 enabled, cs1 disabled: SDRAM size for cs0
 * - cs0 disabled, c1 enabled: 0 (currently assumed that no hardware does this)
 * - cs0 enabled, cs1 enabled: The largest continuous region, that is, cs0 + cs1
 *                             if cs0 is taking the whole address space.
 */
void __naked __noreturn imx1_barebox_entry(uint32_t boarddata)
{
	unsigned long base;
	unsigned long size;

	base = MX1_CSD0_BASE_ADDR;

	size = imx_v1_sdram_size((void *)MX1_SDRAMC_BASE_ADDR, 0);
	if (size == SZ_64M)
		size += imx_v1_sdram_size((void *)MX1_SDRAMC_BASE_ADDR, 1);

	barebox_arm_entry(base, size, boarddata);
}

void __naked __noreturn imx25_barebox_entry(uint32_t boarddata)
{
	unsigned long base;
	unsigned long size;

	base = MX25_CSD0_BASE_ADDR;

	size = imx_v2_sdram_size((void *)MX25_ESDCTL_BASE_ADDR, 0);
	if (size == SZ_256M)
		size += imx_v2_sdram_size((void *)MX25_ESDCTL_BASE_ADDR, 1);

	barebox_arm_entry(base, size, boarddata);
}

void __naked __noreturn imx27_barebox_entry(uint32_t boarddata)
{
	unsigned long base;
	unsigned long size;

	imx_esdctl_v2_disable_default((void *)MX27_ESDCTL_BASE_ADDR);

	base = MX27_CSD0_BASE_ADDR;

	size = imx_v2_sdram_size((void *)MX27_ESDCTL_BASE_ADDR, 0);
	if (size == SZ_256M)
		size += imx_v2_sdram_size((void *)MX27_ESDCTL_BASE_ADDR, 1);

	barebox_arm_entry(base, size, boarddata);
}

void __naked __noreturn imx31_barebox_entry(uint32_t boarddata)
{
	unsigned long base;
	unsigned long size;

	imx_esdctl_v2_disable_default((void *)MX31_ESDCTL_BASE_ADDR);

	base = MX31_CSD0_BASE_ADDR;

	size = imx_v2_sdram_size((void *)MX31_ESDCTL_BASE_ADDR, 0);
	if (size == SZ_256M)
		size += imx_v2_sdram_size((void *)MX31_ESDCTL_BASE_ADDR, 1);

	barebox_arm_entry(base, size, boarddata);
}

void __naked __noreturn imx35_barebox_entry(uint32_t boarddata)
{
	unsigned long base;
	unsigned long size;

	imx_esdctl_v2_disable_default((void *)MX35_ESDCTL_BASE_ADDR);

	base = MX35_CSD0_BASE_ADDR;

	size = imx_v2_sdram_size((void *)MX35_ESDCTL_BASE_ADDR, 0);
	if (size == SZ_256M)
		size += imx_v2_sdram_size((void *)MX35_ESDCTL_BASE_ADDR, 1);

	barebox_arm_entry(base, size, boarddata);
}

void __naked __noreturn imx51_barebox_entry(uint32_t boarddata)
{
	unsigned long base;
	unsigned long size;

	base = MX51_CSD0_BASE_ADDR;

	size = imx_v3_sdram_size((void *)MX51_ESDCTL_BASE_ADDR, 0);
	if (size == SZ_256M)
		size += imx_v3_sdram_size((void *)MX51_ESDCTL_BASE_ADDR, 1);

	barebox_arm_entry(base, size, boarddata);
}

void __naked __noreturn imx53_barebox_entry(uint32_t boarddata)
{
	unsigned long base;
	unsigned long size;

	base = MX53_CSD0_BASE_ADDR;

	size = imx_v4_sdram_size((void *)MX53_ESDCTL_BASE_ADDR, 0);
	if (size == SZ_1G)
		size += imx_v4_sdram_size((void *)MX53_ESDCTL_BASE_ADDR, 1);

	barebox_arm_entry(base, size, boarddata);
}
