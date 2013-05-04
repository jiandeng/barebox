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

#include <init.h>
#include <common.h>
#include <io.h>
#include <sizes.h>
#include <mach/generic.h>
#include <mach/imx6-regs.h>

void imx6_init_lowlevel(void)
{
	void __iomem *aips1 = (void *)MX6_AIPS1_ON_BASE_ADDR;
	void __iomem *aips2 = (void *)MX6_AIPS2_ON_BASE_ADDR;

	/*
	 * Set all MPROTx to be non-bufferable, trusted for R/W,
	 * not forced to user-mode.
	 */
	writel(0x77777777, aips1);
	writel(0x77777777, aips1 + 0x4);
	writel(0, aips1 + 0x40);
	writel(0, aips1 + 0x44);
	writel(0, aips1 + 0x48);
	writel(0, aips1 + 0x4c);
	writel(0, aips1 + 0x50);

	writel(0x77777777, aips2);
	writel(0x77777777, aips2 + 0x4);
	writel(0, aips2 + 0x40);
	writel(0, aips2 + 0x44);
	writel(0, aips2 + 0x48);
	writel(0, aips2 + 0x4c);
	writel(0, aips2 + 0x50);

	/* enable all clocks */
	writel(0xffffffff, 0x020c4068);
	writel(0xffffffff, 0x020c406c);
	writel(0xffffffff, 0x020c4070);
	writel(0xffffffff, 0x020c4074);
	writel(0xffffffff, 0x020c4078);
	writel(0xffffffff, 0x020c407c);
	writel(0xffffffff, 0x020c4080);
}

static int imx6_init(void)
{
	imx6_boot_save_loc((void *)MX6_SRC_BASE_ADDR);

	add_generic_device("imx-iomuxv3", 0, NULL, MX6_IOMUXC_BASE_ADDR, 0x1000, IORESOURCE_MEM, NULL);
	add_generic_device("imx6-ccm", 0, NULL, MX6_CCM_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpt", 0, NULL, MX6_GPT_BASE_ADDR, 0x1000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpio", 0, NULL, MX6_GPIO1_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpio", 1, NULL, MX6_GPIO2_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpio", 2, NULL, MX6_GPIO3_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpio", 3, NULL, MX6_GPIO4_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpio", 4, NULL, MX6_GPIO5_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpio", 5, NULL, MX6_GPIO6_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx31-gpio", 6, NULL, MX6_GPIO7_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx21-wdt", 0, NULL, MX6_WDOG1_BASE_ADDR, 0x4000, IORESOURCE_MEM, NULL);
	add_generic_device("imx6-usb-misc", 0, NULL, MX6_USBOH3_USB_BASE_ADDR + 0x800, 0x100, IORESOURCE_MEM, NULL);

	return 0;
}
postcore_initcall(imx6_init);
