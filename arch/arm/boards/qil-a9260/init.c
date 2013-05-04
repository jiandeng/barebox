/*
 * Copyright (C) 2011-2012 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * GPLv2
 *
 */

#include <common.h>
#include <net.h>
#include <init.h>
#include <environment.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <partition.h>
#include <fs.h>
#include <fcntl.h>
#include <io.h>
#include <asm/hardware.h>
#include <nand.h>
#include <sizes.h>
#include <linux/mtd/nand.h>
#include <linux/clk.h>
#include <mach/board.h>
#include <mach/at91sam9_smc.h>
#include <gpio.h>
#include <led.h>
#include <mach/io.h>
#include <mach/at91_pmc.h>
#include <mach/at91_rstc.h>

static struct atmel_nand_data nand_pdata = {
	.ale		= 21,
	.cle		= 22,
	.det_pin	= -EINVAL,
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.on_flash_bbt	= 1,
};

static struct sam9_smc_config nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 1,
	.ncs_write_setup	= 0,
	.nwe_setup		= 1,

	.ncs_read_pulse		= 3,
	.nrd_pulse		= 3,
	.ncs_write_pulse	= 3,
	.nwe_pulse		= 3,

	.read_cycle		= 5,
	.write_cycle		= 5,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_DBW_8,
	.tdf_cycles		= 2,
};

static void qil_a9260_add_device_nand(void)
{
	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(0, 3, &nand_smc_config);

	at91_add_device_nand(&nand_pdata);
}

#if defined(CONFIG_MCI_ATMEL)
static struct atmel_mci_platform_data __initdata qil_a9260_mci_data = {
	.bus_width	= 4,
	.detect_pin     = -EINVAL,
	.wp_pin         = -EINVAL,
};

static void qil_a9260_add_device_mci(void)
{
	at91_add_device_mci(0, &qil_a9260_mci_data);
}
#else
static void qil_a9260_add_device_mci(void) {}
#endif

#ifdef CONFIG_CALAO_MB_QIL_A9260
static struct macb_platform_data macb_pdata = {
	.phy_interface	= PHY_INTERFACE_MODE_RMII,
	.phy_addr	= -1,
};

static void qil_a9260_phy_reset(void)
{
	unsigned long rstc;
	struct clk *clk = clk_get(NULL, "macb_clk");

	clk_enable(clk);

	at91_set_gpio_input(AT91_PIN_PA14, 0);
	at91_set_gpio_input(AT91_PIN_PA15, 0);
	at91_set_gpio_input(AT91_PIN_PA17, 0);
	at91_set_gpio_input(AT91_PIN_PA25, 0);
	at91_set_gpio_input(AT91_PIN_PA26, 0);
	at91_set_gpio_input(AT91_PIN_PA28, 0);

	rstc = at91_sys_read(AT91_RSTC_MR) & AT91_RSTC_ERSTL;

	/* Need to reset PHY -> 500ms reset */
	at91_sys_write(AT91_RSTC_MR, AT91_RSTC_KEY |
				     (AT91_RSTC_ERSTL & (0x0d << 8)) |
				     AT91_RSTC_URSTEN);

	at91_sys_write(AT91_RSTC_CR, AT91_RSTC_KEY | AT91_RSTC_EXTRST);

	/* Wait for end hardware reset */
	while (!(at91_sys_read(AT91_RSTC_SR) & AT91_RSTC_NRSTL));

	/* Restore NRST value */
	at91_sys_write(AT91_RSTC_MR, AT91_RSTC_KEY |
				     (rstc) |
				     AT91_RSTC_URSTEN);
}

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= -EINVAL,		/* pull-up driven by UDC */
};

static void __init qil_a9260_add_device_mb(void)
{
	qil_a9260_phy_reset();
	at91_add_device_eth(0, &macb_pdata);
	at91_add_device_udc(&ek_udc_data);
}
#else
static void __init qil_a9260_add_device_mb(void)
{
}
#endif

struct gpio_led led = {
	.gpio = AT91_PIN_PB21,
	.led = {
		.name = "user_led",
	},
};

static void __init ek_add_led(void)
{
	at91_set_gpio_output(led.gpio, led.active_low);
	led_gpio_register(&led);
}

static int qil_a9260_mem_init(void)
{
	at91_add_device_sdram(0);

	return 0;
}
mem_initcall(qil_a9260_mem_init);

static void __init ek_add_device_button(void)
{
	at91_set_GPIO_periph(AT91_PIN_PB10, 1);	/* user push button, pull up enabled */
	at91_set_deglitch(AT91_PIN_PB10, 1);

	export_env_ull("dfu_button", AT91_PIN_PB10);
}

static int qil_a9260_devices_init(void)
{
	qil_a9260_add_device_nand();
	qil_a9260_add_device_mci();
	ek_add_led();
	ek_add_device_button();
	qil_a9260_add_device_mb();

	armlinux_set_bootparams((void *)(AT91_CHIPSELECT_1 + 0x100));
	armlinux_set_architecture(MACH_TYPE_QIL_A9260);

	devfs_add_partition("nand0", 0x00000, SZ_128K, DEVFS_PARTITION_FIXED, "at91bootstrap_raw");
	dev_add_bb_dev("at91bootstrap_raw", "at91bootstrap");
	devfs_add_partition("nand0", SZ_128K, SZ_256K, DEVFS_PARTITION_FIXED, "self_raw");
	dev_add_bb_dev("self_raw", "self0");
	devfs_add_partition("nand0", SZ_256K + SZ_128K, SZ_128K, DEVFS_PARTITION_FIXED, "env_raw");
	dev_add_bb_dev("env_raw", "env0");
	devfs_add_partition("nand0", SZ_512K, SZ_128K, DEVFS_PARTITION_FIXED, "env_raw1");
	dev_add_bb_dev("env_raw1", "env1");

	return 0;
}
device_initcall(qil_a9260_devices_init);

#ifdef CONFIG_CALAO_MB_QIL_A9260
static int qil_a9260_console_init(void)
{
	at91_register_uart(0, 0);
	at91_set_A_periph(AT91_PIN_PB14, 1);    /* Enable pull-up on DRXD */

	at91_register_uart(1, ATMEL_UART_CTS | ATMEL_UART_RTS
			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
			   | ATMEL_UART_RI);

	at91_register_uart(2, ATMEL_UART_CTS | ATMEL_UART_RTS);
	at91_set_A_periph(AT91_PIN_PB7, 1);	/* Enable pull-up on RXD1 */

	at91_register_uart(3, ATMEL_UART_CTS | ATMEL_UART_RTS);
	at91_set_A_periph(AT91_PIN_PB9, 1);	/* Enable pull-up on RXD2 */

	return 0;
}
console_initcall(qil_a9260_console_init);
#endif
