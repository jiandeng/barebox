/*
 * (C) Copyright 2010 Juergen Beisert - Pengutronix
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
 */
#include <common.h>
#include <init.h>
#include <gpio.h>
#include <environment.h>
#include <envfs.h>
#include <errno.h>
#include <mci.h>
#include <sizes.h>
#include <usb/ehci.h>
#include <asm/armlinux.h>
#include <io.h>
#include <asm/mmu.h>
#include <generated/mach-types.h>
#include <mach/imx-regs.h>
#include <mach/clock.h>
#include <mach/mci.h>
#include <mach/fb.h>
#include <mach/usb.h>

static struct mxs_mci_platform_data mci_pdata = {
	.caps = MMC_MODE_4BIT | MMC_MODE_HS | MMC_MODE_HS_52MHz,
	.voltages = MMC_VDD_32_33 | MMC_VDD_33_34,	/* fixed to 3.3 V */
};

#define GPIO_LCD_RESET		50
#define GPIO_LCD_BACKLIGHT	60

static void chumby_fb_enable(int enable)
{
	gpio_direction_output(GPIO_LCD_RESET, enable);

	/* Give the display a chance to sync before we enable
	 * the backlight to avoid flickering
	 */
	if (enable)
		mdelay(100);

	gpio_direction_output(GPIO_LCD_BACKLIGHT, enable);
}

static struct fb_videomode falconwing_vmode = {
	/*
	 * Nanovision NMA35QV65-B2-K01 (directly connected)
	 * Clock: 6.25 MHz
	 * Syncs: high active, DE low active
	 * Display area: 70.08 mm x 52.56 mm
	 */
	.name = "NMA35",
	.refresh = 60,
	.xres = 320,
	.yres = 240,
	.pixclock = KHZ2PICOS(6250),    /* max. 10 MHz */
	/* line lenght should be 64 �s */
	.left_margin = 28,
	.hsync_len = 24,
	.right_margin = 28,
	/* frame rate should be 60 Hz */
	.upper_margin = 8,
	.vsync_len = 4,
	.lower_margin = 8,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

#define MAX_FB_SIZE SZ_1M

static struct imx_fb_platformdata fb_mode = {
	.mode_list = &falconwing_vmode,
	.mode_cnt = 1,
	/* the NMA35 is a 24 bit display, but only 18 bits are connected */
	.ld_intf_width = STMLCDIF_18BIT,
	.enable = chumby_fb_enable,
	.fixed_screen = (void *)(0x40000000 + SZ_64M - MAX_FB_SIZE),
	.fixed_screen_size = MAX_FB_SIZE,
};

static const uint32_t pad_setup[] = {
	/* may be not required as already done by the bootlet code */
#if 0
	/* SDRAM data signals */
	EMI_D15 | STRENGTH(0) | VE_2_5V,
	EMI_D14 | STRENGTH(0) | VE_2_5V,
	EMI_D13 | STRENGTH(0) | VE_2_5V,
	EMI_D12 | STRENGTH(0) | VE_2_5V,
	EMI_D11 | STRENGTH(0) | VE_2_5V,
	EMI_D10 | STRENGTH(0) | VE_2_5V,
	EMI_D9 | STRENGTH(0) | VE_2_5V,
	EMI_D8 | STRENGTH(0) | VE_2_5V,
	EMI_D7 | STRENGTH(0) | VE_2_5V,
	EMI_D6 | STRENGTH(0) | VE_2_5V,
	EMI_D5 | STRENGTH(0) | VE_2_5V,
	EMI_D4 | STRENGTH(0) | VE_2_5V,
	EMI_D3 | STRENGTH(0) | VE_2_5V,
	EMI_D2 | STRENGTH(0) | VE_2_5V,
	EMI_D1 | STRENGTH(0) | VE_2_5V,
	EMI_D0 | STRENGTH(0) | VE_2_5V,

	/* SDRAM data control signals */
	EMI_DQM0 | STRENGTH(0) | VE_2_5V,	/* LDM */
	EMI_DQM1 | STRENGTH(0) | VE_2_5V,	/* UDM */

	/* SDRAM address signals */
	EMI_A0 | STRENGTH(0) | VE_2_5V,
	EMI_A1 | STRENGTH(0) | VE_2_5V,
	EMI_A2 | STRENGTH(0) | VE_2_5V,
	EMI_A3 | STRENGTH(0) | VE_2_5V,
	EMI_A4 | STRENGTH(0) | VE_2_5V,
	EMI_A5 | STRENGTH(0) | VE_2_5V,
	EMI_A6 | STRENGTH(0) | VE_2_5V,
	EMI_A7 | STRENGTH(0) | VE_2_5V,
	EMI_A8 | STRENGTH(0) | VE_2_5V,
	EMI_A9 | STRENGTH(0) | VE_2_5V,
	EMI_A10 | STRENGTH(0) | VE_2_5V,
	EMI_A11 | STRENGTH(0) | VE_2_5V,
	EMI_A12 | STRENGTH(0) | VE_2_5V,

	/* SDRAM address control signals */
	EMI_RASN | STRENGTH(0) | VE_2_5V,
	EMI_CASN | STRENGTH(0) | VE_2_5V,

	/* SDRAM control signals */
	EMI_CE0N | STRENGTH(0) | VE_2_5V,
	EMI_CLK | STRENGTH(0) | VE_2_5V,
	EMI_CLKN | STRENGTH(0) | VE_2_5V,
	EMI_CKE | STRENGTH(0) | VE_2_5V,
	EMI_WEN | STRENGTH(0) | VE_2_5V,
	EMI_BA0 | STRENGTH(0) | VE_2_5V,
	EMI_BA1 | STRENGTH(0) | VE_2_5V,
	EMI_DQS0 | STRENGTH(0) | VE_2_5V,
	EMI_DQS1 | STRENGTH(0) | VE_2_5V,
#endif
	/* debug port */
	PWM1_DUART_TX | STRENGTH(S4MA),	/*  strength is TBD */
	PWM0_DUART_RX | STRENGTH(S4MA),	/*  strength is TBD */

	/* lcd */
	LCD_VSYNC | STRENGTH(S12MA),
	LCD_HSYNC | STRENGTH(S12MA),
	LCD_ENABE | STRENGTH(S12MA),
	LCD_DOTCLOCK | STRENGTH(S12MA),
	LCD_D17 | STRENGTH(S12MA),
	LCD_D16 | STRENGTH(S12MA),
	LCD_D15 | STRENGTH(S12MA),
	LCD_D14 | STRENGTH(S12MA),
	LCD_D13 | STRENGTH(S12MA),
	LCD_D12 | STRENGTH(S12MA),
	LCD_D11 | STRENGTH(S12MA),
	LCD_D10 | STRENGTH(S12MA),
	LCD_D9 | STRENGTH(S12MA),
	LCD_D8 | STRENGTH(S12MA),
	LCD_D7 | STRENGTH(S12MA),
	LCD_D6 | STRENGTH(S12MA),
	LCD_D5 | STRENGTH(S12MA),
	LCD_D4 | STRENGTH(S12MA),
	LCD_D3 | STRENGTH(S12MA),
	LCD_D2 | STRENGTH(S12MA),
	LCD_D1 | STRENGTH(S12MA),
	LCD_D0 | STRENGTH(S12MA),

	/* LCD usage currently unknown */
	LCD_CS,	/* used as SPI SS */
	LCD_RS,	/* used as SPI CLK */
	/* keep the display in reset state */
	LCD_RESET_GPIO | STRENGTH(S4MA) | GPIO_OUT | GPIO_VALUE(0),
	LCD_WR,	/* used as SPI MOSI */

	/* I2C to the MMA7455L, KXTE9, AT24C08 (DCID), AT24C128B (ID EEPROM) and QN8005B */
	I2C_SDA,
	I2C_CLK,

	/* Rotary decoder (external pull ups) */
	ROTARYA,
	ROTARYB,

	/* the chumby bend (external pull up) */
	PWM4_GPIO | GPIO_IN,

	/* backlight control, to be controled by PWM, here we only want to disable it */
	PWM2_GPIO | GPIO_OUT | GPIO_VALUE(0),	/* 1 enables, 0 disables the backlight */

	/* USB hub reset (active low) */
	AUART1_TX_GPIO | GPIO_OUT | GPIO_VALUE(0),

	/* USB power (active high) */
	AUART1_CTS_GPIO | GPIO_OUT | GPIO_VALUE(0),

	/* Detecting if a display is connected (0 = display attached) (external pull up) */
	AUART1_RTS_GPIO | GPIO_IN,

	/* disable the audio amplifier */
	GPMI_D08_GPIO | GPIO_OUT | GPIO_VALUE(0),

	/* Head Phone detection (FIXME what level when plugged in) (external pull up) */
	GPMI_D11_GPIO | GPIO_IN,

#if 0
	/* Enable the local 5V (FIXME what to do when the bootloader runs) */
	GPMI_D12_GPIO | GPIO_OUT | GPIO_VALUE(1),
#endif

	/* not used pins */
	GPMI_D09_GPIO | GPIO_IN | PULLUP(1),
	GPMI_D10_GPIO | GPIO_IN | PULLUP(1),
	GPMI_D13_GPIO | GPIO_IN | PULLUP(1),

	/* unknown. Not connected to anything than test pin J113 */
	GPMI_D14_GPIO | GPIO_IN | PULLUP(1),

	/* unknown. Not connected to anything than test pin J114 */
	GPMI_D15_GPIO | GPIO_IN | PULLUP(1),

	/* NAND controller (Note: There is no NAND device on the board) */
	GPMI_D00 | PULLUP(1),
	GPMI_D01 | PULLUP(1),
	GPMI_D02 | PULLUP(1),
	GPMI_D03 | PULLUP(1),
	GPMI_D04 | PULLUP(1),
	GPMI_D05 | PULLUP(1),
	GPMI_D06 | PULLUP(1),
	GPMI_D07 | PULLUP(1),
	GPMI_CE0N,
	GPMI_RDY0 | PULLUP(1),
	GPMI_WRN,	/* kernel tries here with 12 mA */
	GPMI_RDN,	/* kernel tries here with 12 mA */
	GPMI_WPM,	/* kernel tries here with 12 mA */
	GPMI_CLE,
	GPMI_ALE,

	/* SD card interface */
	SSP1_DATA0 | PULLUP(1),	/* available at J201 */
	SSP1_DATA1 | PULLUP(1),	/* available at J200 */
	SSP1_DATA2 | PULLUP(1),	/* available at J205 */
	SSP1_DATA3 | PULLUP(1),	/* available at J204 */
	SSP1_SCK,		/* available at J202 */
	SSP1_CMD | PULLUP(1),	/* available at J203 */
	SSP1_DETECT | PULLUP(1),	/* only connected to test pin J115 */

	/* other not used pins */
	GPMI_CE1N_GPIO | GPIO_IN | PULLUP(1),
	GPMI_CE2N_GPIO | GPIO_IN | PULLUP(1),
	GPMI_RDY1_GPIO | GPIO_IN | PULLUP(1),
	GPMI_RDY2_GPIO | GPIO_IN | PULLUP(1),
	GPMI_RDY3_GPIO | GPIO_IN | PULLUP(1),
};

static int falconwing_mem_init(void)
{
	arm_add_mem_device("ram0", IMX_MEMORY_BASE, 64 * 1024 * 1024);

	return 0;
}
mem_initcall(falconwing_mem_init);

#define GPIO_USB_HUB_RESET	29
#define GPIO_USB_HUB_POWER	26

static void falconwing_init_usb(void)
{
	/* power USB hub */
	gpio_direction_output(GPIO_USB_HUB_POWER, 1);
	mdelay(1);
	/* bring USB hub out of reset */
	gpio_direction_output(GPIO_USB_HUB_RESET, 1);

	imx23_usb_phy_enable();

	add_generic_usb_ehci_device(DEVICE_ID_DYNAMIC, IMX_USB_BASE, NULL);
}

static int falconwing_devices_init(void)
{
	int i, rc;

	/* initizalize gpios */
	for (i = 0; i < ARRAY_SIZE(pad_setup); i++)
		imx_gpio_mode(pad_setup[i]);

	imx_set_ioclk(480000000); /* enable IOCLK to run at the PLL frequency */
	/* run the SSP unit clock at 100,000 kHz */
	imx_set_sspclk(0, 100000000, 1);
	add_generic_device("mxs_mci", 0, NULL, IMX_SSP1_BASE, 0x2000,
			   IORESOURCE_MEM, &mci_pdata);
	add_generic_device("stmfb", 0, NULL, IMX_FB_BASE, 4096,
			   IORESOURCE_MEM, &fb_mode);

	falconwing_init_usb();

	armlinux_set_bootparams((void *)IMX_MEMORY_BASE + 0x100);
	armlinux_set_architecture(MACH_TYPE_CHUMBY);

	rc = envfs_register_partition("disk0", 1);
	if (rc != 0)
		printf("Cannot create the 'env0' persistent environment storage (%d)\n", rc);

	return 0;
}

device_initcall(falconwing_devices_init);

static int falconwing_console_init(void)
{
	add_generic_device("stm_serial", 0, NULL, IMX_DBGUART_BASE, 8192,
			   IORESOURCE_MEM, NULL);

	return 0;
}

console_initcall(falconwing_console_init);

/** @page chumbyone Chumby Industrie's Falconwing

This device is also known as "chumby one" (http://www.chumby.com/)

This CPU card is based on a Freescale i.MX23 CPU. The card is shipped with:

- 64 MiB synchronous dynamic RAM (DDR type)

Memory layout when @b barebox is running:

- 0x40000000 start of SDRAM
- 0x40000100 start of kernel's boot parameters
  - below malloc area: stack area
  - below barebox: malloc area
- 0x42000000 start of @b barebox

@section get_falconwing_binary How to get the bootloader binary image:

Using the default configuration:

@verbatim
make ARCH=arm chumbyone_defconfig
@endverbatim

Build the bootloader binary image:

@verbatim
make ARCH=arm CROSS_COMPILE=armv5compiler
@endverbatim

@note replace the armv5compiler with your ARM v5 cross compiler.

@section setup_falconwing How to prepare an MCI card to boot the "chumby one" with barebox

- Create four primary partitions on the MCI card
 - the first one for the bootlets (about 256 kiB)
 - the second one for the persistant environment (size is up to you, at least 256k)
 - the third one for the kernel (2 MiB ... 4 MiB in size)
 - the 4th one for the root filesystem which can fill the rest of the available space

- Mark the first partition with the partition ID "53" and copy the bootlets
  into this partition (currently not part of @b barebox!).

- Copy the default @b barebox environment into the second partition (no filesystem required).

- Copy the kernel into the third partition (no filesystem required).

- Create the root filesystem in the 4th partition. You may copy an image into this
  partition or you can do it in the classic way: mkfs on it, mount it and copy
  all required data and programs into it.

@section gpio_falconwing Available GPIOs

The Falconwing uses some GPIOs to control various features. With the regular
GPIO commands these features can be controlled at @a barebox's runtime.

<table width="100%" border="1" cellspacing="1" cellpadding="3">
	<tr>
		<td>No</td>
		<td>Direction</td>
		<td>Function</td>
		<td>Reset</td>
		<td>Set</td>
	</tr>
	<tr>
		<td>8</td>
		<td>Output</td>
		<td>Switch Audio Amplifier</td>
		<td>Off</td>
		<td>On</td>
	</tr>
	<tr>
		<td>11</td>
		<td>Input</td>
		<td>Head Phone Detection</td>
		<td>TBD</td>
		<td>TBD</td>
	</tr>
	<tr>
		<td>14</td>
		<td>Input</td>
		<td>Unused (J113)</td>
		<td>User</td>
		<td>User</td>
	</tr>
	<tr>
		<td>15</td>
		<td>Input</td>
		<td>Unused (J114)</td>
		<td>User</td>
		<td>User</td>
	</tr>
	<tr>
		<td>26</td>
		<td>Output</td>
		<td>USB Power</td>
		<td>TBD</td>
		<td>TBD</td>
	</tr>
	<tr>
		<td>27</td>
		<td>Input</td>
		<td>Display Connected</td>
		<td>Display<br>Attached</td>
		<td>Display<br>Disconnected</td>
	</tr>
	<tr>
		<td>29</td>
		<td>Output</td>
		<td>USB HUB Reset</td>
		<td>TBD</td>
		<td>TBD</td>
	</tr>
	<tr>
		<td>50</td>
		<td>Output</td>
		<td>Display Reset</td>
		<td>Display<br>Reset</td>
		<td>Display<br>Running</td>
	</tr>
	<tr>
		<td>60</td>
		<td>Output</td>
		<td>Display Backlight</td>
		<td>Backlight<br>Off</td>
		<td>Backlight<br>On (100 %)</td>
	</tr>
	<tr>
		<td>62</td>
		<td>Input</td>
		<td>Bend</td>
		<td>Not pressed</td>
		<td>Pressed</td>
	</tr>
</table>

*/
