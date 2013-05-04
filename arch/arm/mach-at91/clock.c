/*
 * linux/arch/arm/mach-at91/clock.c
 *
 * Copyright (C) 2005 David Brownell
 * Copyright (C) 2005 Ivan Kokshaysky
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <common.h>
#include <command.h>
#include <complete.h>
#include <linux/list.h>
#include <errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <init.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/at91_pmc.h>
#include <mach/cpu.h>

#include "clock.h"
#include "generic.h"

/*
 * There's a lot more which can be done with clocks, including cpufreq
 * integration, slow clock mode support (for system suspend), letting
 * PLLB be used at other rates (on boards that don't need USB), etc.
 */

#define clk_is_primary(x)	((x)->type & CLK_TYPE_PRIMARY)
#define clk_is_programmable(x)	((x)->type & CLK_TYPE_PROGRAMMABLE)
#define clk_is_peripheral(x)	((x)->type & CLK_TYPE_PERIPHERAL)
#define clk_is_sys(x)		((x)->type & CLK_TYPE_SYSTEM)


/*
 * Chips have some kind of clocks : group them by functionality
 */
#define cpu_has_utmi()		(  cpu_is_at91sam9rl() \
				|| cpu_is_at91sam9g45() \
				|| cpu_is_at91sam9x5() \
				|| cpu_is_sama5d3())

#define cpu_has_1056M_plla()	(cpu_is_sama5d3())

#define cpu_has_800M_plla()	(  cpu_is_at91sam9g20() \
				|| cpu_is_at91sam9g45() \
				|| cpu_is_at91sam9x5() \
				|| cpu_is_at91sam9n12())

#define cpu_has_300M_plla()	(cpu_is_at91sam9g10())

#define cpu_has_240M_plla()	(cpu_is_at91sam9261() \
				|| cpu_is_at91sam9263() \
				|| cpu_is_at91sam9rl())

#define cpu_has_210M_plla()	(cpu_is_at91sam9260())

#define cpu_has_pllb()		(!(cpu_is_at91sam9rl() \
				|| cpu_is_at91sam9g45() \
				|| cpu_is_at91sam9x5() \
				|| cpu_is_at91sam9n12()))

#define cpu_has_upll()		(cpu_is_at91sam9g45() \
				|| cpu_is_at91sam9x5() \
				|| cpu_is_sama5d3())

/* USB host HS & FS */
#define cpu_has_uhp()		(!cpu_is_at91sam9rl())

/* USB device FS only */
#define cpu_has_udpfs()		(!(cpu_is_at91sam9rl() \
				|| cpu_is_at91sam9g45() \
				|| cpu_is_at91sam9x5() \
				|| cpu_is_sama5d3()))

#define cpu_has_plladiv2()	(cpu_is_at91sam9g45() \
				|| cpu_is_at91sam9x5() \
				|| cpu_is_at91sam9n12() \
				|| cpu_is_sama5d3())

#define cpu_has_mdiv3()		(cpu_is_at91sam9g45() \
				|| cpu_is_at91sam9x5() \
				|| cpu_is_at91sam9n12() \
				|| cpu_is_sama5d3())

#define cpu_has_alt_prescaler()	(cpu_is_at91sam9x5() \
				|| cpu_is_at91sam9n12() \
				|| cpu_is_sama5d3())

static LIST_HEAD(clocks);

static u32 at91_pllb_usb_init;

/*
 * Four primary clock sources:  two crystal oscillators (32K, main), and
 * two PLLs.  PLLA usually runs the master clock; and PLLB must run at
 * 48 MHz (unless no USB function clocks are needed).  The main clock and
 * both PLLs are turned off to run in "slow clock mode" (system suspend).
 */
static struct clk clk32k = {
	.name		= "clk32k",
	.rate_hz	= AT91_SLOW_CLOCK,
	.users		= 1,		/* always on */
	.id		= 0,
	.type		= CLK_TYPE_PRIMARY,
};
static struct clk main_clk = {
	.name		= "main",
	.pmc_mask	= AT91_PMC_MOSCS,	/* in PMC_SR */
	.id		= 1,
	.type		= CLK_TYPE_PRIMARY,
};
static struct clk plla = {
	.name		= "plla",
	.parent		= &main_clk,
	.pmc_mask	= AT91_PMC_LOCKA,	/* in PMC_SR */
	.id		= 2,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};

static void pllb_mode(struct clk *clk, int is_on)
{
	u32	value;

	if (is_on) {
		is_on = AT91_PMC_LOCKB;
		value = at91_pllb_usb_init;
	} else
		value = 0;

	// REVISIT: Add work-around for AT91RM9200 Errata #26 ?
	at91_pmc_write(AT91_CKGR_PLLBR, value);

	do {
		barrier();
	} while ((at91_pmc_read(AT91_PMC_SR) & AT91_PMC_LOCKB) != is_on);
}

static struct clk pllb = {
	.name		= "pllb",
	.parent		= &main_clk,
	.pmc_mask	= AT91_PMC_LOCKB,	/* in PMC_SR */
	.mode		= pllb_mode,
	.id		= 3,
	.type		= CLK_TYPE_PRIMARY | CLK_TYPE_PLL,
};

static void pmc_sys_mode(struct clk *clk, int is_on)
{
	if (is_on)
		at91_pmc_write(AT91_PMC_SCER, clk->pmc_mask);
	else
		at91_pmc_write(AT91_PMC_SCDR, clk->pmc_mask);
}

static void pmc_uckr_mode(struct clk *clk, int is_on)
{
	unsigned int uckr = at91_pmc_read(AT91_CKGR_UCKR);

	if (cpu_is_at91sam9g45()) {
		if (is_on)
			uckr |= AT91_PMC_BIASEN;
		else
			uckr &= ~AT91_PMC_BIASEN;
	}

	if (is_on) {
		is_on = AT91_PMC_LOCKU;
		at91_pmc_write(AT91_CKGR_UCKR, uckr | clk->pmc_mask);
	} else
		at91_pmc_write(AT91_CKGR_UCKR, uckr & ~(clk->pmc_mask));

	do {
		barrier();
	} while ((at91_pmc_read(AT91_PMC_SR) & AT91_PMC_LOCKU) != is_on);
}

/* USB function clocks (PLLB must be 48 MHz) */
static struct clk udpck = {
	.name		= "udpck",
	.parent		= &pllb,
	.mode		= pmc_sys_mode,
};
struct clk utmi_clk = {
	.name		= "utmi_clk",
	.parent		= &main_clk,
	.pmc_mask	= AT91_PMC_UPLLEN,	/* in CKGR_UCKR */
	.mode		= pmc_uckr_mode,
	.type		= CLK_TYPE_PLL,
};
static struct clk uhpck = {
	.name		= "uhpck",
	/*.parent		= ... we choose parent at runtime */
	.mode		= pmc_sys_mode,
};


/*
 * The master clock is divided from the CPU clock (by 1-4).  It's used for
 * memory, interfaces to on-chip peripherals, the AIC, and sometimes more
 * (e.g baud rate generation).  It's sourced from one of the primary clocks.
 */
struct clk mck = {
	.name		= "mck",
	.pmc_mask	= AT91_PMC_MCKRDY,	/* in PMC_SR */
};

static void pmc_periph_mode(struct clk *clk, int is_on)
{
	u32 regval = 0;

	/*
	 * With sama5d3 chips, you have more than 32 peripherals so only one
	 * register is not enough to manage their clocks. A peripheral
	 * control register has been introduced to solve this issue.
	 */
	if (cpu_is_sama5d3()) {
		regval |= AT91_PMC_PCR_CMD; /* write command */
		regval |= clk->pid & AT91_PMC_PCR_PID; /* peripheral selection */
		regval |= AT91_PMC_PCR_DIV(clk->div);
		if (is_on)
			regval |= AT91_PMC_PCR_EN; /* enable clock */
		at91_pmc_write(AT91_PMC_PCR, regval);
	} else {
		if (is_on)
			at91_pmc_write(AT91_PMC_PCER, clk->pmc_mask);
		else
			at91_pmc_write(AT91_PMC_PCDR, clk->pmc_mask);
	}
}

static struct clk *at91_css_to_clk(unsigned long css)
{
	switch (css) {
		case AT91_PMC_CSS_SLOW:
			return &clk32k;
		case AT91_PMC_CSS_MAIN:
			return &main_clk;
		case AT91_PMC_CSS_PLLA:
			return &plla;
		case AT91_PMC_CSS_PLLB:
			if (cpu_has_upll())
				/* CSS_PLLB == CSS_UPLL */
				return &utmi_clk;
			else if (cpu_has_pllb())
				return &pllb;
			break;
		/* alternate PMC: can use master clock */
		case AT91_PMC_CSS_MASTER:
			return &mck;
	}

	return NULL;
}

static int pmc_prescaler_divider(u32 reg)
{
	if (cpu_has_alt_prescaler()) {
		return 1 << ((reg & AT91_PMC_ALT_PRES) >> PMC_ALT_PRES_OFFSET);
	} else {
		return 1 << ((reg & AT91_PMC_PRES) >> PMC_PRES_OFFSET);
	}
}

static void __clk_enable(struct clk *clk)
{
	if (clk->parent)
		__clk_enable(clk->parent);
	if (clk->users++ == 0 && clk->mode)
		clk->mode(clk, 1);
}

int clk_enable(struct clk *clk)
{
	__clk_enable(clk);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

static void __clk_disable(struct clk *clk)
{
	BUG_ON(clk->users == 0);
	if (--clk->users == 0 && clk->mode)
		clk->mode(clk, 0);
	if (clk->parent)
		__clk_disable(clk->parent);
}

void clk_disable(struct clk *clk)
{
	__clk_disable(clk);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long	rate;

	for (;;) {
		rate = clk->rate_hz;
		if (rate || !clk->parent)
			break;
		clk = clk->parent;
	}
	return rate;
}
EXPORT_SYMBOL(clk_get_rate);

/*------------------------------------------------------------------------*/

#ifdef CONFIG_AT91_PROGRAMMABLE_CLOCKS

/*
 * For now, only the programmable clocks support reparenting (MCK could
 * do this too, with care) or rate changing (the PLLs could do this too,
 * ditto MCK but that's more for cpufreq).  Drivers may reparent to get
 * a better rate match; we don't.
 */

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long	flags;
	unsigned	prescale;
	unsigned long	actual;
	unsigned long	prev = ULONG_MAX;

	if (!clk_is_programmable(clk))
		return -EINVAL;

	actual = clk->parent->rate_hz;
	for (prescale = 0; prescale < 7; prescale++) {
		if (actual > rate)
			prev = actual;

		if (actual && actual <= rate) {
			if ((prev - rate) < (rate - actual)) {
				actual = prev;
				prescale--;
			}
			break;
		}
		actual >>= 1;
	}

	return (prescale < 7) ? actual : -ENOENT;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long	flags;
	unsigned	prescale;
	unsigned long	prescale_offset, css_mask;
	unsigned long	actual;

	if (!clk_is_programmable(clk))
		return -EINVAL;
	if (clk->users)
		return -EBUSY;

	if (cpu_has_alt_prescaler()) {
		prescale_offset = PMC_ALT_PRES_OFFSET;
		css_mask = AT91_PMC_ALT_PCKR_CSS;
	} else {
		prescale_offset = PMC_PRES_OFFSET;
		css_mask = AT91_PMC_CSS;
	}

	actual = clk->parent->rate_hz;
	for (prescale = 0; prescale < 7; prescale++) {
		if (actual && actual <= rate) {
			u32	pckr;

			pckr = at91_pmc_read(AT91_PMC_PCKR(clk->id));
			pckr &= css_mask;	/* keep clock selection */
			pckr |= prescale << prescale_offset;
			at91_pmc_write(AT91_PMC_PCKR(clk->id), pckr);
			clk->rate_hz = actual;
			break;
		}
		actual >>= 1;
	}

	return (prescale < 7) ? actual : -ENOENT;
}
EXPORT_SYMBOL(clk_set_rate);

struct clk *clk_get_parent(struct clk *clk)
{
	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long	flags;

	if (clk->users)
		return -EBUSY;
	if (!clk_is_primary(parent) || !clk_is_programmable(clk))
		return -EINVAL;

	if (cpu_is_at91sam9rl() && parent->id == AT91_PMC_CSS_PLLB)
		return -EINVAL;

	clk->rate_hz = parent->rate_hz;
	clk->parent = parent;
	at91_pmc_write(AT91_PMC_PCKR(clk->id), parent->id);

	return 0;
}
EXPORT_SYMBOL(clk_set_parent);

/* establish PCK0..PCKN parentage and rate */
static void init_programmable_clock(struct clk *clk)
{
	struct clk	*parent;
	u32		pckr;
	unsigned int	css_mask;

	if (cpu_has_alt_prescaler())
		css_mask = AT91_PMC_ALT_PCKR_CSS;
	else
		css_mask = AT91_PMC_CSS;

	pckr = at91_pmc_read(AT91_PMC_PCKR(clk->id));
	parent = at91_css_to_clk(pckr & css_mask);
	clk->parent = parent;
	clk->rate_hz = parent->rate_hz / pmc_prescaler_divider(pckr);
}

#endif	/* CONFIG_AT91_PROGRAMMABLE_CLOCKS */

/*------------------------------------------------------------------------*/

/* Register a new clock */
static void __init at91_clk_add(struct clk *clk)
{
	list_add_tail(&clk->node, &clocks);

	clk->cl.con_id = clk->name;
	clk->cl.clk = clk;
	clkdev_add(&clk->cl);
}

int clk_register(struct clk *clk)
{
	if (clk_is_peripheral(clk)) {
		if (!clk->parent)
			clk->parent = &mck;
		if (cpu_is_sama5d3())
			clk->rate_hz = DIV_ROUND_UP(clk->parent->rate_hz, 1 << clk->div);
		clk->mode = pmc_periph_mode;
	}
	else if (clk_is_sys(clk)) {
		clk->parent = &mck;
		clk->mode = pmc_sys_mode;
	}
#ifdef CONFIG_AT91_PROGRAMMABLE_CLOCKS
	else if (clk_is_programmable(clk)) {
		clk->mode = pmc_sys_mode;
		init_programmable_clock(clk);
	}
#endif

	at91_clk_add(clk);

	return 0;
}

/*------------------------------------------------------------------------*/

static u32 at91_pll_rate(struct clk *pll, u32 freq, u32 reg)
{
	unsigned mul, div;

	div = reg & 0xff;
	if (cpu_is_sama5d3())
		mul = (reg >> 18) & 0x7ff;
	else
		mul = (reg >> 16) & 0x7ff;
	if (div && mul) {
		freq /= div;
		freq *= mul + 1;
	} else
		freq = 0;

	return freq;
}

static u32 at91_usb_rate(struct clk *pll, u32 freq, u32 reg)
{
	if (pll == &pllb && (reg & AT91_PMC_USB96M))
		return freq / 2;
	else
		return freq;
}

static unsigned at91_pll_calc(unsigned main_freq, unsigned out_freq)
{
	unsigned i, div = 0, mul = 0, diff = 1 << 30;
	unsigned ret = (out_freq > 155000000) ? 0xbe00 : 0x3e00;

	/* PLL output max 240 MHz (or 180 MHz per errata) */
	if (out_freq > 240000000)
		goto fail;

	for (i = 1; i < 256; i++) {
		int diff1;
		unsigned input, mul1;

		/*
		 * PLL input between 1MHz and 32MHz per spec, but lower
		 * frequences seem necessary in some cases so allow 100K.
		 * Warning: some newer products need 2MHz min.
		 */
		input = main_freq / i;
		if (cpu_is_at91sam9g20() && input < 2000000)
			continue;
		if (input < 100000)
			continue;
		if (input > 32000000)
			continue;

		mul1 = out_freq / input;
		if (cpu_is_at91sam9g20() && mul > 63)
			continue;
		if (mul1 > 2048)
			continue;
		if (mul1 < 2)
			goto fail;

		diff1 = out_freq - input * mul1;
		if (diff1 < 0)
			diff1 = -diff1;
		if (diff > diff1) {
			diff = diff1;
			div = i;
			mul = mul1;
			if (diff == 0)
				break;
		}
	}
	if (i == 256 && diff > (out_freq >> 5))
		goto fail;
	return ret | ((mul - 1) << 16) | div;
fail:
	return 0;
}

static struct clk *const standard_pmc_clocks[] = {
	/* four primary clocks */
	&clk32k,
	&main_clk,
	&plla,

	/* MCK */
	&mck
};

/* PLLB generated USB full speed clock init */
static void at91_pllb_usbfs_clock_init(unsigned long main_clock)
{
	/*
	 * USB clock init:  choose 48 MHz PLLB value,
	 * disable 48MHz clock during usb peripheral suspend.
	 *
	 * REVISIT:  assumes MCK doesn't derive from PLLB!
	 */
	uhpck.parent = &pllb;

	at91_pllb_usb_init = at91_pll_calc(main_clock, 48000000 * 2) | AT91_PMC_USB96M;
	pllb.rate_hz = at91_pll_rate(&pllb, main_clock, at91_pllb_usb_init);
	if (cpu_is_at91rm9200()) {
		uhpck.pmc_mask = AT91RM9200_PMC_UHP;
		udpck.pmc_mask = AT91RM9200_PMC_UDP;
		at91_pmc_write(AT91_PMC_SCER, AT91RM9200_PMC_MCKUDP);
	} else if (cpu_is_at91sam9260() || cpu_is_at91sam9261() ||
		   cpu_is_at91sam9263() || cpu_is_at91sam9g20() ||
		   cpu_is_at91sam9g10()) {
		uhpck.pmc_mask = AT91SAM926x_PMC_UHP;
		udpck.pmc_mask = AT91SAM926x_PMC_UDP;
	}
	at91_pmc_write(AT91_CKGR_PLLBR, 0);

	udpck.rate_hz = at91_usb_rate(&pllb, pllb.rate_hz, at91_pllb_usb_init);
	uhpck.rate_hz = at91_usb_rate(&pllb, pllb.rate_hz, at91_pllb_usb_init);
}

/* UPLL generated USB full speed clock init */
static void at91_upll_usbfs_clock_init(unsigned long main_clock)
{
	/*
	 * USB clock init: choose 480 MHz from UPLL,
	 */
	unsigned int usbr = AT91_PMC_USBS_UPLL;

	/* Setup divider by 10 to reach 48 MHz */
	usbr |= ((10 - 1) << 8) & AT91_PMC_OHCIUSBDIV;

	at91_pmc_write(AT91_PMC_USB, usbr);

	/* Now set uhpck values */
	uhpck.parent = &utmi_clk;
	uhpck.pmc_mask = AT91SAM926x_PMC_UHP;
	uhpck.rate_hz = utmi_clk.rate_hz;
	uhpck.rate_hz /= 1 + ((at91_pmc_read(AT91_PMC_USB) & AT91_PMC_OHCIUSBDIV) >> 8);
}

static int pll_overclock = 0;
static u32 cpu_freq = 0;

int at91_clock_init(unsigned long main_clock)
{
	unsigned tmp, freq, mckr;
	int i;



	/*
	 * When the bootloader initialized the main oscillator correctly,
	 * there's no problem using the cycle counter.  But if it didn't,
	 * or when using oscillator bypass mode, we must be told the speed
	 * of the main clock.
	 */
	if (!main_clock) {
		do {
			tmp = at91_pmc_read(AT91_CKGR_MCFR);
		} while (!(tmp & AT91_PMC_MAINRDY));
		main_clock = (tmp & AT91_PMC_MAINF) * (AT91_SLOW_CLOCK / 16);
	}
	main_clk.rate_hz = main_clock;

	/* report if PLLA is more than mildly overclocked */
	plla.rate_hz = at91_pll_rate(&plla, main_clock, at91_pmc_read(AT91_CKGR_PLLAR));
	if (cpu_has_1056M_plla()) {
		if (plla.rate_hz > 1056000000)
			pll_overclock = 1;
	} else if (cpu_has_300M_plla()) {
		if (plla.rate_hz > 300000000)
			pll_overclock = 1;
	} else if (cpu_has_800M_plla()) {
		if (plla.rate_hz > 800000000)
			pll_overclock = 1;
	} else if (cpu_has_300M_plla()) {
		if (plla.rate_hz > 300000000)
			pll_overclock = 1;
	} else if (cpu_has_240M_plla()) {
		if (plla.rate_hz > 240000000)
			pll_overclock = 1;
	} else if (cpu_has_210M_plla()) {
		if (plla.rate_hz > 210000000)
			pll_overclock = 1;
	} else {
		if (plla.rate_hz > 209000000)
			pll_overclock = 1;
	}

	if (cpu_has_plladiv2()) {
		mckr = at91_pmc_read(AT91_PMC_MCKR);
		plla.rate_hz /= (1 << ((mckr & AT91_PMC_PLLADIV2) >> 12));	/* plla divisor by 2 */
	}

	if (!cpu_has_pllb() && cpu_has_upll()) {
		/* setup UTMI clock as the fourth primary clock
		 * (instead of pllb) */
		utmi_clk.type |= CLK_TYPE_PRIMARY;
		utmi_clk.id = 3;
	}

	/*
	 * USB HS clock init
	 */
	if (cpu_has_utmi()) {
		/*
		 * multiplier is hard-wired to 40
		 * (obtain the USB High Speed 480 MHz when input is 12 MHz)
		 */
		utmi_clk.rate_hz = 40 * utmi_clk.parent->rate_hz;

		/* UTMI bias and PLL are managed at the same time */
		if (cpu_has_upll())
			utmi_clk.pmc_mask |= AT91_PMC_BIASEN;
	}

	/*
	 * USB FS clock init
	 */
	if (cpu_has_pllb())
		at91_pllb_usbfs_clock_init(main_clock);
	if (cpu_has_upll())
		/* assumes that we choose UPLL for USB and not PLLA */
		at91_upll_usbfs_clock_init(main_clock);

	/*
	 * MCK and CPU derive from one of those primary clocks.
	 * For now, assume this parentage won't change.
	 */
	mckr = at91_pmc_read(AT91_PMC_MCKR);
	mck.parent = at91_css_to_clk(mckr & AT91_PMC_CSS);
	freq = mck.parent->rate_hz;
	freq /= pmc_prescaler_divider(mckr);					/* prescale */
	if (cpu_is_at91rm9200()) {
		mck.rate_hz = freq / (1 + ((mckr & AT91_PMC_MDIV) >> 8));	/* mdiv */
	} else if (cpu_is_at91sam9g20()) {
		mck.rate_hz = (mckr & AT91_PMC_MDIV) ?
			freq / ((mckr & AT91_PMC_MDIV) >> 7) : freq;	/* mdiv ; (x >> 7) = ((x >> 8) * 2) */
		if (mckr & AT91_PMC_PDIV)
			freq /= 2;		/* processor clock division */
	} else if (cpu_has_mdiv3()) {
		mck.rate_hz = (mckr & AT91_PMC_MDIV) == AT91SAM9_PMC_MDIV_3 ?
			freq / 3 : freq / (1 << ((mckr & AT91_PMC_MDIV) >> 8));	/* mdiv */
	} else {
		mck.rate_hz = freq / (1 << ((mckr & AT91_PMC_MDIV) >> 8));      /* mdiv */
	}

	if (cpu_has_alt_prescaler()) {
		/* Programmable clocks can use MCK */
		mck.type |= CLK_TYPE_PRIMARY;
		mck.id = 4;
	}

	cpu_freq = freq;

	/* Register the PMC's standard clocks */
	for (i = 0; i < ARRAY_SIZE(standard_pmc_clocks); i++)
		at91_clk_add(standard_pmc_clocks[i]);

	if (cpu_has_pllb())
		at91_clk_add(&pllb);

	if (cpu_has_uhp())
		at91_clk_add(&uhpck);

	if (cpu_has_udpfs())
		at91_clk_add(&udpck);

	if (cpu_has_utmi())
		at91_clk_add(&utmi_clk);

	/* MCK and CPU clock are "always on" */
	clk_enable(&mck);

	return 0;
}

static int at91_clock_display(void)
{
	if (pll_overclock)
		pr_info("Clocks: PLLA overclocked, %ld MHz\n", plla.rate_hz / 1000000);

	pr_info("Clocks: CPU %u MHz, master %u MHz, main %u.%03u MHz\n",
		cpu_freq / 1000000, (unsigned) mck.rate_hz / 1000000,
		(unsigned) main_clk.rate_hz / 1000000,
		((unsigned) main_clk.rate_hz % 1000000) / 1000);

	return 0;
}
postconsole_initcall(at91_clock_display);

/*
 * Several unused clocks may be active.  Turn them off.
 */
static int at91_clock_reset(void)
{
	unsigned long pcdr = 0;
	unsigned long pcdr1 = 0;
	unsigned long scdr = 0;
	struct clk *clk;

	list_for_each_entry(clk, &clocks, node) {
		if (clk->users > 0)
			continue;

		if (clk->mode == pmc_periph_mode) {
			if (cpu_is_sama5d3()) {
				u32 pmc_mask = 1 << (clk->pid % 32);

				if (clk->pid > 31)
					pcdr1 |= pmc_mask;
				else
					pcdr |= pmc_mask;
			} else
				pcdr |= clk->pmc_mask;
		}

		if (clk->mode == pmc_sys_mode)
			scdr |= clk->pmc_mask;

		pr_debug("Clocks: disable unused %s\n", clk->name);
	}

	at91_pmc_write(AT91_PMC_PCDR, pcdr);
	if (cpu_is_sama5d3())
		at91_pmc_write(AT91_PMC_PCDR1, pcdr1);
	at91_pmc_write(AT91_PMC_SCDR, scdr);

	return 0;
}
late_initcall(at91_clock_reset);

#ifdef CONFIG_CMD_AT91CLK
static int do_at91clk(int argc, char *argv[])
{
	u32		scsr, pcsr, pcsr1 = 0, uckr = 0, sr;
	struct clk	*clk;

	scsr = at91_pmc_read(AT91_PMC_SCSR);
	pcsr = at91_pmc_read(AT91_PMC_PCSR);
	if (cpu_is_sama5d3())
		pcsr1 = at91_pmc_read(AT91_PMC_PCSR1);
	sr = at91_pmc_read(AT91_PMC_SR);
	printf("SCSR = %8x\n", scsr);
	printf("PCSR = %8x\n", pcsr);
	if (cpu_is_sama5d3())
		printf("PCSR1 = %8x\n", pcsr1);
	printf("MOR  = %8x\n", at91_pmc_read(AT91_CKGR_MOR));
	printf("MCFR = %8x\n", at91_pmc_read(AT91_CKGR_MCFR));
	printf("PLLA = %8x\n", at91_pmc_read(AT91_CKGR_PLLAR));
	if (cpu_has_pllb())
		printf("PLLB = %8x\n", at91_pmc_read(AT91_CKGR_PLLBR));
	if (cpu_has_utmi()) {
		uckr = at91_pmc_read(AT91_CKGR_UCKR);
		printf("UCKR = %8x\n", uckr);
	}
	printf("MCKR = %8x\n", at91_pmc_read(AT91_PMC_MCKR));
	if (cpu_has_upll())
		printf("USB  = %8x\n", at91_pmc_read(AT91_PMC_USB));
	printf("SR   = %8x\n", sr);

	printf("\n");

	list_for_each_entry(clk, &clocks, node) {
		char	*state;
		char	*mode = "";

		if (clk->mode == pmc_sys_mode) {
			state = (scsr & clk->pmc_mask) ? "on" : "off";
			mode = "sys";
		} else if (clk->mode == pmc_periph_mode) {
			if (cpu_is_sama5d3()) {
				u32 pmc_mask = 1 << (clk->pid % 32);

				if (clk->pid > 31)
					state = (pcsr1 & pmc_mask) ? "on" : "off";
				else
					state = (pcsr & pmc_mask) ? "on" : "off";
			} else {
				state = (pcsr & clk->pmc_mask) ? "on" : "off";
			}
			mode = "periph";
		} else if (clk->mode == pmc_uckr_mode) {
			state = (uckr & clk->pmc_mask) ? "on" : "off";
			mode = "uckr";
		} else if (clk->pmc_mask) {
			state = (sr & clk->pmc_mask) ? "on" : "off";
		} else if (clk == &clk32k || clk == &main_clk) {
			state = "on";
		} else {
			state = "";
		}

		printf("%-10s %-7s users=%2d %-3s %10lu Hz %s\n",
			clk->name, mode, clk->users, state, clk_get_rate(clk),
			clk->parent ? clk->parent->name : "");
	}
	return 0;
}

BAREBOX_CMD_HELP_START(at91clk)
BAREBOX_CMD_HELP_USAGE("at91clk\n")
BAREBOX_CMD_HELP_SHORT("dump current clock configuration\n");
BAREBOX_CMD_HELP_END

BAREBOX_CMD_START(at91clk)
	.cmd		= do_at91clk,
	.usage		= "dump current clock configuration",
	BAREBOX_CMD_HELP(cmd_at91clk_help)
	BAREBOX_CMD_COMPLETE(empty_complete)
BAREBOX_CMD_END
#endif
