/*
 * Copyright (C) 2013 Jean-Christophe PLAGNIOL-VILLARD <plagnio@jcrosoft.com>
 *
 * GPLv2 only
 */

#include <common.h>
#include <init.h>
#include <asm/armlinux.h>
#include <asm/system_info.h>
#include <generated/mach-types.h>
#include <mach/devices.h>
#include <environment.h>
#include <partition.h>
#include <sizes.h>
#include <io.h>
#include <globalvar.h>
#include <linux/amba/sp804.h>

struct vexpress_init {
	void (*core_init)(void);
	void (*mem_init)(void);
	void (*console_init)(void);
	void (*devices_init)(void);
	char *hostname;
};

struct vexpress_init *v2m_init;

static void vexpress_ax_mem_init(void)
{
	vexpress_add_ddram(SZ_512M);
}

#define V2M_SYS_FLASH	0x03c

static void vexpress_ax_devices_init(void)
{
	add_cfi_flash_device(0, 0x08000000, SZ_64M, 0);
	add_cfi_flash_device(1, 0x0c000000, SZ_64M, 0);
	add_generic_device("smc911x", DEVICE_ID_DYNAMIC, NULL, 0x1a000000,
			64 * 1024, IORESOURCE_MEM, NULL);
	armlinux_set_bootparams((void *)(0x80000100));
}

static void vexpress_ax_console_init(void)
{
	vexpress_register_uart(0);
	vexpress_register_uart(1);
	vexpress_register_uart(2);
	vexpress_register_uart(3);
}

struct vexpress_init vexpress_init_ax = {
	.core_init = vexpress_init,
	.mem_init = vexpress_ax_mem_init,
	.console_init = vexpress_ax_console_init,
	.devices_init = vexpress_ax_devices_init,
};

static void vexpress_a9_legacy_mem_init(void)
{
	vexpress_a9_legacy_add_ddram(SZ_512M, SZ_512M);
}

static void vexpress_a9_legacy_devices_init(void)
{
	add_cfi_flash_device(0, 0x40000000, SZ_64M, 0);
	add_cfi_flash_device(1, 0x44000000, SZ_64M, 0);
	add_generic_device("smc911x", DEVICE_ID_DYNAMIC, NULL, 0x4e000000,
			64 * 1024, IORESOURCE_MEM, NULL);

	armlinux_set_architecture(MACH_TYPE_VEXPRESS);
	armlinux_set_bootparams((void *)(0x60000100));
}

static void vexpress_a9_legacy_console_init(void)
{
	vexpress_a9_legacy_register_uart(0);
	vexpress_a9_legacy_register_uart(1);
	vexpress_a9_legacy_register_uart(2);
	vexpress_a9_legacy_register_uart(3);
}

struct vexpress_init vexpress_init_a9_legacy = {
	.core_init = vexpress_a9_legacy_init,
	.mem_init = vexpress_a9_legacy_mem_init,
	.console_init = vexpress_a9_legacy_console_init,
	.devices_init = vexpress_a9_legacy_devices_init,
	.hostname = "vexpress-a9-legacy",
};

static int vexpress_mem_init(void)
{
	v2m_init->mem_init();

	return 0;
}
mem_initcall(vexpress_mem_init);

static int vexpress_devices_init(void)
{
	writel(1, v2m_sysreg_base + V2M_SYS_FLASH);
	v2m_init->devices_init();

	devfs_add_partition("nor0", 0x00000, 0x40000, DEVFS_PARTITION_FIXED, "self");
	devfs_add_partition("nor0", 0x40000, 0x20000, DEVFS_PARTITION_FIXED, "env0");


	globalvar_add_simple("hostname");
	setenv("global.hostname", v2m_init->hostname);

	return 0;
}
device_initcall(vexpress_devices_init);

static int vexpress_console_init(void)
{
	v2m_init->console_init();

	return 0;
}
console_initcall(vexpress_console_init);

static int vexpress_core_init(void)
{
	if (amba_is_arm_sp804(IOMEM(0x10011000))) {
		v2m_init = &vexpress_init_a9_legacy;
	} else {
		v2m_init = &vexpress_init_ax;
		if (cpu_is_cortex_a5())
			v2m_init->hostname = "vexpress-a5";
		else if (cpu_is_cortex_a7())
			v2m_init->hostname = "vexpress-a7";
		else if (cpu_is_cortex_a9())
			v2m_init->hostname = "vexpress-a9";
		else if (cpu_is_cortex_a15())
			v2m_init->hostname = "vexpress-a15";
	}

	v2m_init->core_init();

	return 0;
}
postcore_initcall(vexpress_core_init);
