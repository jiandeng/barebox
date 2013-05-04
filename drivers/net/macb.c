/*
 * Copyright (C) 2005-2006 Atmel Corporation
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
#include <common.h>

/*
 * The barebox networking stack is a little weird.  It seems like the
 * networking core allocates receive buffers up front without any
 * regard to the hardware that's supposed to actually receive those
 * packets.
 *
 * The MACB receives packets into 128-byte receive buffers, so the
 * buffers allocated by the core isn't very practical to use.  We'll
 * allocate our own, but we need one such buffer in case a packet
 * wraps around the DMA ring so that we have to copy it.
 *
 * Therefore, define CFG_RX_ETH_BUFFER to 1 in the board-specific
 * configuration header.  This way, the core allocates one RX buffer
 * and one TX buffer, each of which can hold a ethernet packet of
 * maximum size.
 *
 * For some reason, the networking core unconditionally specifies a
 * 32-byte packet "alignment" (which really should be called
 * "padding").  MACB shouldn't need that, but we'll refrain from any
 * core modifications here...
 */

#include <net.h>
#include <clock.h>
#include <malloc.h>
#include <xfuncs.h>
#include <init.h>
#include <errno.h>
#include <io.h>
#include <platform_data/macb.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/mmu.h>
#include <linux/phy.h>

#include "macb.h"

#define MACB_RX_BUFFER_SIZE	128
#define RX_BUFFER_MULTIPLE	64  /* bytes */
#define RX_NB_PACKET		10
#define TX_RING_SIZE		2 /* must be power of 2 */

#define RX_RING_BYTES(bp)	(sizeof(struct macb_dma_desc) * bp->rx_ring_size)
#define TX_RING_BYTES		(sizeof(struct macb_dma_desc) * TX_RING_SIZE)

struct macb_device {
	void			__iomem *regs;

	unsigned int		rx_tail;
	unsigned int		tx_head;
	unsigned int		tx_tail;

	void			*rx_buffer;
	void			*tx_buffer;
	struct macb_dma_desc	*rx_ring;
	struct macb_dma_desc	*tx_ring;

	int			rx_buffer_size;
	int			rx_ring_size;

	int			phy_addr;

	struct clk		*pclk;
	const struct device_d	*dev;
	struct eth_device	netdev;

	phy_interface_t		interface;

	struct mii_bus	miibus;

	unsigned int		phy_flags;

	bool			is_gem;
};

static inline bool macb_is_gem(struct macb_device *macb)
{
	return macb->is_gem;
}

static inline bool read_is_gem(struct macb_device *macb)
{
	return MACB_BFEXT(IDNUM, macb_readl(macb, MID)) == 0x2;
}

static int macb_send(struct eth_device *edev, void *packet,
		     int length)
{
	struct macb_device *macb = edev->priv;
	unsigned long ctrl;
	int ret = 0;
	uint64_t start;
	unsigned int tx_head = macb->tx_head;

	ctrl = MACB_BF(TX_FRMLEN, length);
	ctrl |= MACB_BIT(TX_LAST);

	if (tx_head == (TX_RING_SIZE - 1)) {
		ctrl |= MACB_BIT(TX_WRAP);
		macb->tx_head = 0;
	} else {
		macb->tx_head++;
	}

	macb->tx_ring[tx_head].ctrl = ctrl;
	macb->tx_ring[tx_head].addr = (ulong)packet;
	barrier();
	dma_flush_range((ulong) packet, (ulong)packet + length);
	macb_writel(macb, NCR, MACB_BIT(TE) | MACB_BIT(RE) | MACB_BIT(TSTART));

	start = get_time_ns();
	ret = -ETIMEDOUT;
	do {
		barrier();
		ctrl = macb->tx_ring[0].ctrl;
		if (ctrl & MACB_BIT(TX_USED)) {
			ret = 0;
			break;
		}
	} while (!is_timeout(start, 100 * MSECOND));

	if (ctrl & MACB_BIT(TX_UNDERRUN))
		dev_err(macb->dev, "TX underrun\n");
	if (ctrl & MACB_BIT(TX_BUF_EXHAUSTED))
		dev_err(macb->dev, "TX buffers exhausted in mid frame\n");
	if (ret)
		dev_err(macb->dev,"TX timeout\n");

	return ret;
}

static void reclaim_rx_buffers(struct macb_device *macb,
			       unsigned int new_tail)
{
	unsigned int i;

	dev_dbg(macb->dev, "%s\n", __func__);

	i = macb->rx_tail;
	while (i > new_tail) {
		macb->rx_ring[i].addr &= ~MACB_BIT(RX_USED);
		i++;
		if (i > macb->rx_ring_size)
			i = 0;
	}

	while (i < new_tail) {
		macb->rx_ring[i].addr &= ~MACB_BIT(RX_USED);
		i++;
	}

	barrier();
	macb->rx_tail = new_tail;
}

static int gem_recv(struct eth_device *edev)
{
	struct macb_device *macb = edev->priv;
	void *buffer;
	int length;
	u32 status;

	dev_dbg(macb->dev, "%s\n", __func__);

	for (;;) {
		barrier();
		if (!(macb->rx_ring[macb->rx_tail].addr & MACB_BIT(RX_USED)))
			return -1;

		barrier();
		status = macb->rx_ring[macb->rx_tail].ctrl;
		length = MACB_BFEXT(RX_FRMLEN, status);
		buffer = macb->rx_buffer + macb->rx_buffer_size * macb->rx_tail;
		net_receive(buffer, length);
		macb->rx_ring[macb->rx_tail].addr &= ~MACB_BIT(RX_USED);
		barrier();

		macb->rx_tail++;
		if (macb->rx_tail >= macb->rx_ring_size)
			macb->rx_tail = 0;
	}

	return 0;
}

static int macb_recv(struct eth_device *edev)
{
	struct macb_device *macb = edev->priv;
	unsigned int rx_tail = macb->rx_tail;
	void *buffer;
	int length;
	int wrapped = 0;
	u32 status;

	dev_dbg(macb->dev, "%s\n", __func__);

	for (;;) {
		if (!(macb->rx_ring[rx_tail].addr & MACB_BIT(RX_USED)))
			return -1;

		status = macb->rx_ring[rx_tail].ctrl;
		if (status & MACB_BIT(RX_SOF)) {
			if (rx_tail != macb->rx_tail)
				reclaim_rx_buffers(macb, rx_tail);
			wrapped = 0;
		}

		if (status & MACB_BIT(RX_EOF)) {
			buffer = macb->rx_buffer + macb->rx_buffer_size * macb->rx_tail;
			length = MACB_BFEXT(RX_FRMLEN, status);
			if (wrapped) {
				unsigned int headlen, taillen;

				headlen = macb->rx_buffer_size * (macb->rx_ring_size
						 - macb->rx_tail);
				taillen = length - headlen;
				memcpy((void *)NetRxPackets[0],
				       buffer, headlen);
				memcpy((void *)NetRxPackets[0] + headlen,
				       macb->rx_buffer, taillen);
				buffer = (void *)NetRxPackets[0];
			}

			net_receive(buffer, length);
			if (++rx_tail >= macb->rx_ring_size)
				rx_tail = 0;
			reclaim_rx_buffers(macb, rx_tail);
		} else {
			if (++rx_tail >= macb->rx_ring_size) {
				wrapped = 1;
				rx_tail = 0;
			}
		}
		barrier();
	}

	return 0;
}

static void macb_adjust_link(struct eth_device *edev)
{
	struct macb_device *macb = edev->priv;
	u32 reg;

	reg = macb_readl(macb, NCFGR);
	reg &= ~(MACB_BIT(SPD) | MACB_BIT(FD));
	if (macb_is_gem(macb))
		reg &= ~GEM_BIT(GBE);

	if (edev->phydev->duplex)
		reg |= MACB_BIT(FD);
	if (edev->phydev->speed == SPEED_100)
		reg |= MACB_BIT(SPD);
	if (edev->phydev->speed == SPEED_1000)
		reg |= GEM_BIT(GBE);

	macb_or_gem_writel(macb, NCFGR, reg);
}

static int macb_open(struct eth_device *edev)
{
	struct macb_device *macb = edev->priv;

	dev_dbg(macb->dev, "%s\n", __func__);

	/* Enable TX and RX */
	macb_writel(macb, NCR, MACB_BIT(TE) | MACB_BIT(RE));

	/* Obtain the PHY's address/id */
	return phy_device_connect(edev, &macb->miibus, macb->phy_addr,
			       macb_adjust_link, macb->phy_flags,
			       macb->interface);
}

/*
 * Configure the receive DMA engine
 * - use the correct receive buffer size
 * - set the possibility to use INCR16 bursts
 *   (if not supported by FIFO, it will fallback to default)
 * - set both rx/tx packet buffers to full memory size
 * - set discard rx packets if no DMA resource
 * These are configurable parameters for GEM.
 */
static void macb_configure_dma(struct macb_device *bp)
{
	u32 dmacfg;

	if (macb_is_gem(bp)) {
		dmacfg = gem_readl(bp, DMACFG) & ~GEM_BF(RXBS, -1L);
		dmacfg |= GEM_BF(RXBS, bp->rx_buffer_size / RX_BUFFER_MULTIPLE);
		dmacfg |= GEM_BF(FBLDO, 16);
		dmacfg |= GEM_BIT(TXPBMS) | GEM_BF(RXBMS, -1L);
		dmacfg |= GEM_BIT(DDRP);
		dmacfg &= ~GEM_BIT(ENDIA);
		gem_writel(bp, DMACFG, dmacfg);
	}
}

static void macb_init(struct macb_device *macb)
{
	unsigned long paddr, val = 0;
	int i;

	dev_dbg(macb->dev, "%s\n", __func__);

	/*
	 * macb_halt should have been called at some point before now,
	 * so we'll assume the controller is idle.
	 */

	/* initialize DMA descriptors */
	paddr = (ulong)macb->rx_buffer;
	for (i = 0; i < macb->rx_ring_size; i++) {
		macb->rx_ring[i].addr = paddr;
		macb->rx_ring[i].ctrl = 0;
		paddr += macb->rx_buffer_size;
	}
	macb->rx_ring[macb->rx_ring_size - 1].addr |= MACB_BIT(RX_WRAP);

	for (i = 0; i < TX_RING_SIZE; i++) {
		macb->tx_ring[i].addr = 0;
		macb->tx_ring[i].ctrl = MACB_BIT(TX_USED);
	}
	macb->tx_ring[TX_RING_SIZE - 1].addr |= MACB_BIT(TX_WRAP);

	macb->rx_tail = macb->tx_head = macb->tx_tail = 0;

	macb_configure_dma(macb);

	macb_writel(macb, RBQP, (ulong)macb->rx_ring);
	macb_writel(macb, TBQP, (ulong)macb->tx_ring);

	switch(macb->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		val = GEM_BIT(RGMII);
		break;
	case PHY_INTERFACE_MODE_RMII:
		if (IS_ENABLED(CONFIG_ARCH_AT91))
			val = MACB_BIT(RMII) | MACB_BIT(CLKEN);
		else
			val = 0;
		break;
	default:
		if (IS_ENABLED(CONFIG_ARCH_AT91))
			val = MACB_BIT(CLKEN);
		else
			val = MACB_BIT(MII);
	}

	macb_or_gem_writel(macb, USRIO, val);

}

static void macb_halt(struct eth_device *edev)
{
	struct macb_device *macb = edev->priv;
	u32 ncr, tsr;

	/* Halt the controller and wait for any ongoing transmission to end. */
	ncr = macb_readl(macb, NCR);
	ncr |= MACB_BIT(THALT);
	macb_writel(macb, NCR, ncr);

	do {
		tsr = macb_readl(macb, TSR);
	} while (tsr & MACB_BIT(TGO));

	/* Disable TX and RX, and clear statistics */
	macb_writel(macb, NCR, MACB_BIT(CLRSTAT));
}

static int macb_phy_read(struct mii_bus *bus, int addr, int reg)
{
	struct macb_device *macb = bus->priv;

	unsigned long netctl;
	unsigned long frame;
	int value;
	uint64_t start;

	dev_dbg(macb->dev, "%s\n", __func__);

	netctl = macb_readl(macb, NCR);
	netctl |= MACB_BIT(MPE);
	macb_writel(macb, NCR, netctl);

	frame = (MACB_BF(SOF, MACB_MAN_SOF)
		 | MACB_BF(RW, MACB_MAN_READ)
		 | MACB_BF(PHYA, addr)
		 | MACB_BF(REGA, reg)
		 | MACB_BF(CODE, MACB_MAN_CODE));
	macb_writel(macb, MAN, frame);

	start = get_time_ns();
	do {
		if (is_timeout(start, SECOND)) {
			dev_err(macb->dev, "phy read timed out\n");
			return -1;
		}
	} while (!MACB_BFEXT(IDLE, macb_readl(macb, NSR)));

	frame = macb_readl(macb, MAN);
	value = MACB_BFEXT(DATA, frame);

	netctl = macb_readl(macb, NCR);
	netctl &= ~MACB_BIT(MPE);
	macb_writel(macb, NCR, netctl);

	return value;
}

static int macb_phy_write(struct mii_bus *bus, int addr, int reg, u16 value)
{
	struct macb_device *macb = bus->priv;
	unsigned long netctl;
	unsigned long frame;

	dev_dbg(macb->dev, "%s\n", __func__);

	netctl = macb_readl(macb, NCR);
	netctl |= MACB_BIT(MPE);
	macb_writel(macb, NCR, netctl);

	frame = (MACB_BF(SOF, MACB_MAN_SOF)
		 | MACB_BF(RW, MACB_MAN_WRITE)
		 | MACB_BF(PHYA, addr)
		 | MACB_BF(REGA, reg)
		 | MACB_BF(CODE, MACB_MAN_CODE)
		 | MACB_BF(DATA, value));
	macb_writel(macb, MAN, frame);

	while (!MACB_BFEXT(IDLE, macb_readl(macb, NSR)))
		;

	netctl = macb_readl(macb, NCR);
	netctl &= ~MACB_BIT(MPE);
	macb_writel(macb, NCR, netctl);

	return 0;
}

static int macb_get_ethaddr(struct eth_device *edev, unsigned char *adr)
{
	struct macb_device *macb = edev->priv;

	dev_dbg(macb->dev, "%s\n", __func__);

	return -1;
}

static int macb_set_ethaddr(struct eth_device *edev, unsigned char *adr)
{
	struct macb_device *macb = edev->priv;

	dev_dbg(macb->dev, "%s\n", __func__);

	/* set hardware address */
	macb_or_gem_writel(macb, SA1B, adr[0] | adr[1] << 8 | adr[2] << 16 | adr[3] << 24);
	macb_or_gem_writel(macb, SA1T, adr[4] | adr[5] << 8);

	return 0;
}

static u32 gem_mdc_clk_div(struct macb_device *bp)
{
	u32 config;
	unsigned long pclk_hz = clk_get_rate(bp->pclk);

	if (pclk_hz <= 20000000)
		config = GEM_BF(CLK, GEM_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = GEM_BF(CLK, GEM_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = GEM_BF(CLK, GEM_CLK_DIV32);
	else if (pclk_hz <= 120000000)
		config = GEM_BF(CLK, GEM_CLK_DIV48);
	else if (pclk_hz <= 160000000)
		config = GEM_BF(CLK, GEM_CLK_DIV64);
	else
		config = GEM_BF(CLK, GEM_CLK_DIV96);

	return config;
}

static u32 macb_mdc_clk_div(struct macb_device *bp)
{
	u32 config;
	unsigned long pclk_hz;

	if (macb_is_gem(bp))
		return gem_mdc_clk_div(bp);

	pclk_hz = clk_get_rate(bp->pclk);
	if (pclk_hz <= 20000000)
		config = MACB_BF(CLK, MACB_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = MACB_BF(CLK, MACB_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = MACB_BF(CLK, MACB_CLK_DIV32);
	else
		config = MACB_BF(CLK, MACB_CLK_DIV64);

	return config;
}

/*
 * Get the DMA bus width field of the network configuration register that we
 * should program.  We find the width from decoding the design configuration
 * register to find the maximum supported data bus width.
 */
static u32 macb_dbw(struct macb_device *bp)
{
	if (!macb_is_gem(bp))
		return 0;

	switch (GEM_BFEXT(DBWDEF, gem_readl(bp, DCFG1))) {
	case 4:
		return GEM_BF(DBW, GEM_DBW128);
	case 2:
		return GEM_BF(DBW, GEM_DBW64);
	case 1:
	default:
		return GEM_BF(DBW, GEM_DBW32);
	}
}

static void macb_reset_hw(struct macb_device *bp)
{
	/* Disable RX and TX forcefully */
	macb_writel(bp, NCR, 0);

	/* Clear the stats registers (XXX: Update stats first?) */
	macb_writel(bp, NCR, MACB_BIT(CLRSTAT));

	/* Clear all status flags */
	macb_writel(bp, TSR, -1);
	macb_writel(bp, RSR, -1);

	/* Disable all interrupts */
	macb_writel(bp, IDR, -1);
	macb_readl(bp, ISR);
}

static void macb_init_rx_buffer_size(struct macb_device *bp, size_t size)
{
	if (!macb_is_gem(bp)) {
		bp->rx_buffer_size = MACB_RX_BUFFER_SIZE;
		bp->rx_ring_size = roundup(RX_NB_PACKET * PKTSIZE / MACB_RX_BUFFER_SIZE, 2);
	} else {
		bp->rx_buffer_size = size;
		bp->rx_ring_size = RX_NB_PACKET;

		if (bp->rx_buffer_size % RX_BUFFER_MULTIPLE) {
			dev_dbg(bp->dev,
				    "RX buffer must be multiple of %d bytes, expanding\n",
				    RX_BUFFER_MULTIPLE);
			bp->rx_buffer_size =
				roundup(bp->rx_buffer_size, RX_BUFFER_MULTIPLE);
		}
		bp->rx_buffer = dma_alloc_coherent(bp->rx_buffer_size * bp->rx_ring_size);
	}

	dev_dbg(bp->dev, "[%d] rx_buffer_size [%d]\n",
		   size, bp->rx_buffer_size);
}

static int macb_probe(struct device_d *dev)
{
	struct eth_device *edev;
	struct macb_device *macb;
	u32 ncfgr;
	struct macb_platform_data *pdata;

	if (!dev->platform_data) {
		dev_err(dev, "macb: no platform_data\n");
		return -ENODEV;
	}
	pdata = dev->platform_data;

	edev = xzalloc(sizeof(struct eth_device) + sizeof(struct macb_device));
	edev->priv = (struct macb_device *)(edev + 1);
	macb = edev->priv;

	macb->dev = dev;

	edev->open = macb_open;
	edev->send = macb_send;
	edev->halt = macb_halt;
	edev->get_ethaddr = pdata->get_ethaddr ? pdata->get_ethaddr : macb_get_ethaddr;
	edev->set_ethaddr = macb_set_ethaddr;
	edev->parent = dev;

	macb->miibus.read = macb_phy_read;
	macb->miibus.write = macb_phy_write;
	macb->phy_addr = pdata->phy_addr;
	macb->miibus.priv = macb;
	macb->miibus.parent = dev;

	if (pdata->phy_interface == PHY_INTERFACE_MODE_NA)
		macb->interface = PHY_INTERFACE_MODE_MII;
	else
		macb->interface = pdata->phy_interface;

	macb->phy_flags = pdata->phy_flags;

	macb->regs = dev_request_mem_region(dev, 0);

	/*
	 * Do some basic initialization so that we at least can talk
	 * to the PHY
	 */
	macb->pclk = clk_get(dev, "macb_clk");
	if (IS_ERR(macb->pclk)) {
		dev_err(dev, "no macb_clk\n");
		return PTR_ERR(macb->pclk);
	}

	clk_enable(macb->pclk);

	macb->is_gem = read_is_gem(macb);

	if (macb_is_gem(macb))
		edev->recv = gem_recv;
	else
		edev->recv = macb_recv;

	macb_init_rx_buffer_size(macb, PKTSIZE);
	macb->rx_buffer = dma_alloc_coherent(macb->rx_buffer_size * macb->rx_ring_size);
	macb->rx_ring = dma_alloc_coherent(RX_RING_BYTES(macb));
	macb->tx_ring = dma_alloc_coherent(TX_RING_BYTES);

	macb_reset_hw(macb);
	ncfgr = macb_mdc_clk_div(macb);
	ncfgr |= MACB_BIT(PAE);		/* PAuse Enable */
	ncfgr |= MACB_BIT(DRFCS);	/* Discard Rx FCS */
	ncfgr |= macb_dbw(macb);
	macb_writel(macb, NCFGR, ncfgr);

	macb_init(macb);

	mdiobus_register(&macb->miibus);
	eth_register(edev);

	dev_info(dev, "Cadence %s at 0x%p\n",
		macb_is_gem(macb) ? "GEM" : "MACB", macb->regs);

	return 0;
}

static struct driver_d macb_driver = {
	.name  = "macb",
	.probe = macb_probe,
};
device_platform_driver(macb_driver);
