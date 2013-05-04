/*
 * Freescale i.MX28 NAND flash driver
 *
 * Copyright (C) 2011 Wolfram Sang <w.sang@pengutronix.de>
 *
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * Based on code from LTIB:
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Embedded Alley Solutions, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/types.h>
#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <driver.h>
#include <init.h>
#include <asm/mmu.h>
#include <asm/io.h>
#include <mach/clock.h>
#include <mach/imx-regs.h>
#include <mach/dma.h>
#include <mach/mxs.h>

#define	MX28_BLOCK_SFTRST				(1 << 31)
#define	MX28_BLOCK_CLKGATE				(1 << 30)

#define GPMI_CTRL0					0x00000000
#define	GPMI_CTRL0_RUN					(1 << 29)
#define	GPMI_CTRL0_DEV_IRQ_EN				(1 << 28)
/* Disable for now since we don't need it and it is different on MX23.
#define	GPMI_CTRL0_LOCK_CS				(1 << 27)
*/
#define	GPMI_CTRL0_UDMA					(1 << 26)
#define	GPMI_CTRL0_COMMAND_MODE_MASK			(0x3 << 24)
#define	GPMI_CTRL0_COMMAND_MODE_OFFSET			24
#define	GPMI_CTRL0_COMMAND_MODE_WRITE			(0x0 << 24)
#define	GPMI_CTRL0_COMMAND_MODE_READ			(0x1 << 24)
#define	GPMI_CTRL0_COMMAND_MODE_READ_AND_COMPARE	(0x2 << 24)
#define	GPMI_CTRL0_COMMAND_MODE_WAIT_FOR_READY		(0x3 << 24)
#define	GPMI_CTRL0_WORD_LENGTH				(1 << 23)
/* Careful: Is 0x3 on MX23
#define	GPMI_CTRL0_CS_MASK				(0x7 << 20)
*/
#define	GPMI_CTRL0_CS_OFFSET				20
#define	GPMI_CTRL0_ADDRESS_MASK				(0x7 << 17)
#define	GPMI_CTRL0_ADDRESS_OFFSET			17
#define	GPMI_CTRL0_ADDRESS_NAND_DATA			(0x0 << 17)
#define	GPMI_CTRL0_ADDRESS_NAND_CLE			(0x1 << 17)
#define	GPMI_CTRL0_ADDRESS_NAND_ALE			(0x2 << 17)
#define	GPMI_CTRL0_ADDRESS_INCREMENT			(1 << 16)
#define	GPMI_CTRL0_XFER_COUNT_MASK			0xffff
#define	GPMI_CTRL0_XFER_COUNT_OFFSET			0

#define GPMI_CTRL1					0x00000060
#define	GPMI_CTRL1_DECOUPLE_CS				(1 << 24)
#define	GPMI_CTRL1_WRN_DLY_SEL_MASK			(0x3 << 22)
#define	GPMI_CTRL1_WRN_DLY_SEL_OFFSET			22
#define	GPMI_CTRL1_TIMEOUT_IRQ_EN			(1 << 20)
#define	GPMI_CTRL1_GANGED_RDYBUSY			(1 << 19)
#define	GPMI_CTRL1_BCH_MODE				(1 << 18)
#define	GPMI_CTRL1_DLL_ENABLE				(1 << 17)
#define	GPMI_CTRL1_HALF_PERIOD				(1 << 16)
#define	GPMI_CTRL1_RDN_DELAY_MASK			(0xf << 12)
#define	GPMI_CTRL1_RDN_DELAY_OFFSET			12
#define	GPMI_CTRL1_DMA2ECC_MODE				(1 << 11)
#define	GPMI_CTRL1_DEV_IRQ				(1 << 10)
#define	GPMI_CTRL1_TIMEOUT_IRQ				(1 << 9)
#define	GPMI_CTRL1_BURST_EN				(1 << 8)
#define	GPMI_CTRL1_ABORT_WAIT_REQUEST			(1 << 7)
#define	GPMI_CTRL1_ABORT_WAIT_FOR_READY_CHANNEL_MASK	(0x7 << 4)
#define	GPMI_CTRL1_ABORT_WAIT_FOR_READY_CHANNEL_OFFSET	4
#define	GPMI_CTRL1_DEV_RESET				(1 << 3)
#define	GPMI_CTRL1_ATA_IRQRDY_POLARITY			(1 << 2)
#define	GPMI_CTRL1_CAMERA_MODE				(1 << 1)
#define	GPMI_CTRL1_GPMI_MODE				(1 << 0)

#define	GPMI_ECCCTRL_HANDLE_MASK			(0xffff << 16)
#define	GPMI_ECCCTRL_HANDLE_OFFSET			16
#define	GPMI_ECCCTRL_ECC_CMD_MASK			(0x3 << 13)
#define	GPMI_ECCCTRL_ECC_CMD_OFFSET			13
#define	GPMI_ECCCTRL_ECC_CMD_DECODE			(0x0 << 13)
#define	GPMI_ECCCTRL_ECC_CMD_ENCODE			(0x1 << 13)
#define	GPMI_ECCCTRL_ENABLE_ECC				(1 << 12)
#define	GPMI_ECCCTRL_BUFFER_MASK_MASK			0x1ff
#define	GPMI_ECCCTRL_BUFFER_MASK_OFFSET			0
#define	GPMI_ECCCTRL_BUFFER_MASK_BCH_AUXONLY		0x100
#define	GPMI_ECCCTRL_BUFFER_MASK_BCH_PAGE		0x1ff

#define GPMI_STAT				0x000000b0
#define	GPMI_STAT_READY_BUSY_OFFSET			24

#define GPMI_DEBUG				0x000000c0
#define GPMI_DEBUG_READY0_OFFSET			28

#define GPMI_VERSION				0x000000d0
#define GPMI_VERSION_MINOR_OFFSET			16
#define GPMI_VERSION_TYPE_MX23			0x0300

#define BCH_CTRL				0x00000000
#define	BCH_CTRL_COMPLETE_IRQ			(1 << 0)
#define	BCH_CTRL_COMPLETE_IRQ_EN		(1 << 8)

#define BCH_LAYOUTSELECT			0x00000070

#define BCH_FLASH0LAYOUT0			0x00000080
#define	BCH_FLASHLAYOUT0_NBLOCKS_MASK			(0xff << 24)
#define	BCH_FLASHLAYOUT0_NBLOCKS_OFFSET			24
#define	BCH_FLASHLAYOUT0_META_SIZE_MASK			(0xff << 16)
#define	BCH_FLASHLAYOUT0_META_SIZE_OFFSET		16
#define	BCH_FLASHLAYOUT0_ECC0_MASK			(0xf << 12)
#define	BCH_FLASHLAYOUT0_ECC0_OFFSET			12

#define BCH_FLASH0LAYOUT1			0x00000090
#define	BCH_FLASHLAYOUT1_PAGE_SIZE_MASK			(0xffff << 16)
#define	BCH_FLASHLAYOUT1_PAGE_SIZE_OFFSET		16
#define	BCH_FLASHLAYOUT1_ECCN_MASK			(0xf << 12)
#define	BCH_FLASHLAYOUT1_ECCN_OFFSET			12

#define	MXS_NAND_DMA_DESCRIPTOR_COUNT		4

#define	MXS_NAND_CHUNK_DATA_CHUNK_SIZE		512
#define	MXS_NAND_METADATA_SIZE			10

#define	MXS_NAND_COMMAND_BUFFER_SIZE		32

#define	MXS_NAND_BCH_TIMEOUT			10000

struct mxs_nand_info {
	struct nand_chip	nand_chip;
	void __iomem		*io_base;
	struct mtd_info		mtd;
	u32		version;

	int		cur_chip;

	uint32_t	cmd_queue_len;

	uint8_t		*cmd_buf;
	uint8_t		*data_buf;
	uint8_t		*oob_buf;

	uint8_t		marking_block_bad;
	uint8_t		raw_oob_mode;

	/* Functions with altered behaviour */
	int		(*hooked_read_oob)(struct mtd_info *mtd,
				loff_t from, struct mtd_oob_ops *ops);
	int		(*hooked_write_oob)(struct mtd_info *mtd,
				loff_t to, struct mtd_oob_ops *ops);
	int		(*hooked_block_markbad)(struct mtd_info *mtd,
				loff_t ofs);

	/* DMA descriptors */
	struct mxs_dma_desc	**desc;
	uint32_t		desc_index;
};

struct nand_ecclayout fake_ecc_layout;

static struct mxs_dma_desc *mxs_nand_get_dma_desc(struct mxs_nand_info *info)
{
	struct mxs_dma_desc *desc;

	if (info->desc_index >= MXS_NAND_DMA_DESCRIPTOR_COUNT) {
		printf("MXS NAND: Too many DMA descriptors requested\n");
		return NULL;
	}

	desc = info->desc[info->desc_index];
	info->desc_index++;

	return desc;
}

static void mxs_nand_return_dma_descs(struct mxs_nand_info *info)
{
	int i;
	struct mxs_dma_desc *desc;

	for (i = 0; i < info->desc_index; i++) {
		desc = info->desc[i];
		memset(desc, 0, sizeof(struct mxs_dma_desc));
		desc->address = (dma_addr_t)desc;
	}

	info->desc_index = 0;
}

static uint32_t mxs_nand_ecc_chunk_cnt(uint32_t page_data_size)
{
	return page_data_size / MXS_NAND_CHUNK_DATA_CHUNK_SIZE;
}

static uint32_t mxs_nand_ecc_size_in_bits(uint32_t ecc_strength)
{
	return ecc_strength * 13;
}

static uint32_t mxs_nand_aux_status_offset(void)
{
	return (MXS_NAND_METADATA_SIZE + 0x3) & ~0x3;
}

static inline uint32_t mxs_nand_get_ecc_strength(uint32_t page_data_size,
						uint32_t page_oob_size)
{
	if (page_data_size == 2048)
		return 8;

	if (page_data_size == 4096) {
		if (page_oob_size == 128)
			return 8;

		if (page_oob_size == 218)
			return 16;
	}

	return 0;
}

static inline uint32_t mxs_nand_get_mark_offset(uint32_t page_data_size,
						uint32_t ecc_strength)
{
	uint32_t chunk_data_size_in_bits;
	uint32_t chunk_ecc_size_in_bits;
	uint32_t chunk_total_size_in_bits;
	uint32_t block_mark_chunk_number;
	uint32_t block_mark_chunk_bit_offset;
	uint32_t block_mark_bit_offset;

	chunk_data_size_in_bits = MXS_NAND_CHUNK_DATA_CHUNK_SIZE * 8;
	chunk_ecc_size_in_bits  = mxs_nand_ecc_size_in_bits(ecc_strength);

	chunk_total_size_in_bits =
			chunk_data_size_in_bits + chunk_ecc_size_in_bits;

	/* Compute the bit offset of the block mark within the physical page. */
	block_mark_bit_offset = page_data_size * 8;

	/* Subtract the metadata bits. */
	block_mark_bit_offset -= MXS_NAND_METADATA_SIZE * 8;

	/*
	 * Compute the chunk number (starting at zero) in which the block mark
	 * appears.
	 */
	block_mark_chunk_number =
			block_mark_bit_offset / chunk_total_size_in_bits;

	/*
	 * Compute the bit offset of the block mark within its chunk, and
	 * validate it.
	 */
	block_mark_chunk_bit_offset = block_mark_bit_offset -
			(block_mark_chunk_number * chunk_total_size_in_bits);

	if (block_mark_chunk_bit_offset > chunk_data_size_in_bits)
		return 1;

	/*
	 * Now that we know the chunk number in which the block mark appears,
	 * we can subtract all the ECC bits that appear before it.
	 */
	block_mark_bit_offset -=
		block_mark_chunk_number * chunk_ecc_size_in_bits;

	return block_mark_bit_offset;
}

static uint32_t mxs_nand_mark_byte_offset(struct mtd_info *mtd)
{
	uint32_t ecc_strength;
	ecc_strength = mxs_nand_get_ecc_strength(mtd->writesize, mtd->oobsize);
	return mxs_nand_get_mark_offset(mtd->writesize, ecc_strength) >> 3;
}

static uint32_t mxs_nand_mark_bit_offset(struct mtd_info *mtd)
{
	uint32_t ecc_strength;
	ecc_strength = mxs_nand_get_ecc_strength(mtd->writesize, mtd->oobsize);
	return mxs_nand_get_mark_offset(mtd->writesize, ecc_strength) & 0x7;
}

/*
 * Wait for BCH complete IRQ and clear the IRQ
 */
static int mxs_nand_wait_for_bch_complete(void)
{
	void __iomem *bch_regs = (void __iomem *)MXS_BCH_BASE;
	int timeout = MXS_NAND_BCH_TIMEOUT;
	int ret;

	while (--timeout) {
		if (readl(bch_regs + BCH_CTRL) & BCH_CTRL_COMPLETE_IRQ)
			break;
		udelay(1);
	}

	ret = (timeout == 0) ? -ETIMEDOUT : 0;

	writel(BCH_CTRL_COMPLETE_IRQ, bch_regs + BCH_CTRL + BIT_CLR);

	return ret;
}

/*
 * This is the function that we install in the cmd_ctrl function pointer of the
 * owning struct nand_chip. The only functions in the reference implementation
 * that use these functions pointers are cmdfunc and select_chip.
 *
 * In this driver, we implement our own select_chip, so this function will only
 * be called by the reference implementation's cmdfunc. For this reason, we can
 * ignore the chip enable bit and concentrate only on sending bytes to the NAND
 * Flash.
 */
static void mxs_nand_cmd_ctrl(struct mtd_info *mtd, int data, unsigned int ctrl)
{
	struct nand_chip *nand = mtd->priv;
	struct mxs_nand_info *nand_info = nand->priv;
	struct mxs_dma_desc *d;
	uint32_t channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + nand_info->cur_chip;
	int ret;

	/*
	 * If this condition is true, something is _VERY_ wrong in MTD
	 * subsystem!
	 */
	if (nand_info->cmd_queue_len == MXS_NAND_COMMAND_BUFFER_SIZE) {
		printf("MXS NAND: Command queue too long\n");
		return;
	}

	/*
	 * Every operation begins with a command byte and a series of zero or
	 * more address bytes. These are distinguished by either the Address
	 * Latch Enable (ALE) or Command Latch Enable (CLE) signals being
	 * asserted. When MTD is ready to execute the command, it will
	 * deasert both latch enables.
	 *
	 * Rather than run a separate DMA operation for every single byte, we
	 * queue them up and run a single DMA operation for the entire series
	 * of command and data bytes.
	 */
	if (ctrl & (NAND_ALE | NAND_CLE)) {
		if (data != NAND_CMD_NONE)
			nand_info->cmd_buf[nand_info->cmd_queue_len++] = data;
		return;
	}

	/*
	 * If control arrives here, MTD has deasserted both the ALE and CLE,
	 * which means it's ready to run an operation. Check if we have any
	 * bytes to send.
	 */
	if (nand_info->cmd_queue_len == 0)
		return;

	/* Compile the DMA descriptor -- a descriptor that sends command. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_DMA_READ | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_CHAIN | MXS_DMA_DESC_DEC_SEM |
		MXS_DMA_DESC_WAIT4END | (3 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
		(nand_info->cmd_queue_len << MXS_DMA_DESC_BYTES_OFFSET);

	d->cmd.address = (dma_addr_t)nand_info->cmd_buf;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WRITE |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_CLE |
		GPMI_CTRL0_ADDRESS_INCREMENT |
		nand_info->cmd_queue_len;

	mxs_dma_desc_append(channel, d);

	/* Execute the DMA chain. */
	ret = mxs_dma_go(channel);
	if (ret)
		printf("MXS NAND: Error sending command\n");

	mxs_nand_return_dma_descs(nand_info);

	/* Reset the command queue. */
	nand_info->cmd_queue_len = 0;
}

/*
 * Test if the NAND flash is ready.
 */
static int mxs_nand_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct mxs_nand_info *nand_info = chip->priv;
	void __iomem *gpmi_regs = (void *)MXS_GPMI_BASE;
	uint32_t tmp;

	if (nand_info->version > GPMI_VERSION_TYPE_MX23) {
		tmp = readl(gpmi_regs + GPMI_STAT);
		tmp >>= (GPMI_STAT_READY_BUSY_OFFSET + nand_info->cur_chip);
	} else {
		tmp = readl(gpmi_regs + GPMI_DEBUG);
		tmp >>= (GPMI_DEBUG_READY0_OFFSET + nand_info->cur_chip);
	}
	return tmp & 1;
}

/*
 * Select the NAND chip.
 */
static void mxs_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd->priv;
	struct mxs_nand_info *nand_info = nand->priv;

	nand_info->cur_chip = chip;
}

/*
 * Handle block mark swapping.
 *
 * Note that, when this function is called, it doesn't know whether it's
 * swapping the block mark, or swapping it *back* -- but it doesn't matter
 * because the the operation is the same.
 */
static void mxs_nand_swap_block_mark(struct mtd_info *mtd,
					uint8_t *data_buf, uint8_t *oob_buf)
{
	struct nand_chip *chip = mtd->priv;
	struct mxs_nand_info *nand_info = chip->priv;

	uint32_t bit_offset;
	uint32_t buf_offset;

	uint32_t src;
	uint32_t dst;

	/* Don't do swapping on MX23 */
	if (nand_info->version == GPMI_VERSION_TYPE_MX23)
		return;

	bit_offset = mxs_nand_mark_bit_offset(mtd);
	buf_offset = mxs_nand_mark_byte_offset(mtd);

	/*
	 * Get the byte from the data area that overlays the block mark. Since
	 * the ECC engine applies its own view to the bits in the page, the
	 * physical block mark won't (in general) appear on a byte boundary in
	 * the data.
	 */
	src = data_buf[buf_offset] >> bit_offset;
	src |= data_buf[buf_offset + 1] << (8 - bit_offset);

	dst = oob_buf[0];

	oob_buf[0] = src;

	data_buf[buf_offset] &= ~(0xff << bit_offset);
	data_buf[buf_offset + 1] &= 0xff << bit_offset;

	data_buf[buf_offset] |= dst << bit_offset;
	data_buf[buf_offset + 1] |= dst >> (8 - bit_offset);
}

/*
 * Read data from NAND.
 */
static void mxs_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int length)
{
	struct nand_chip *nand = mtd->priv;
	struct mxs_nand_info *nand_info = nand->priv;
	struct mxs_dma_desc *d;
	uint32_t channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + nand_info->cur_chip;
	int ret;

	if (length > NAND_MAX_PAGESIZE) {
		printf("MXS NAND: DMA buffer too big\n");
		return;
	}

	if (!buf) {
		printf("MXS NAND: DMA buffer is NULL\n");
		return;
	}

	/* Compile the DMA descriptor - a descriptor that reads data. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_DMA_WRITE | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM | MXS_DMA_DESC_WAIT4END |
		(1 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
		(length << MXS_DMA_DESC_BYTES_OFFSET);

	d->cmd.address = (dma_addr_t)nand_info->data_buf;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_READ |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		length;

	mxs_dma_desc_append(channel, d);

	/*
	 * A DMA descriptor that waits for the command to end and the chip to
	 * become ready.
	 *
	 * I think we actually should *not* be waiting for the chip to become
	 * ready because, after all, we don't care. I think the original code
	 * did that and no one has re-thought it yet.
	 */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_NAND_WAIT_4_READY | MXS_DMA_DESC_DEC_SEM |
		MXS_DMA_DESC_WAIT4END | (4 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WAIT_FOR_READY |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA;

	mxs_dma_desc_append(channel, d);

	/* Execute the DMA chain. */
	ret = mxs_dma_go(channel);
	if (ret) {
		printf("MXS NAND: DMA read error\n");
		goto rtn;
	}

	memcpy(buf, nand_info->data_buf, length);

rtn:
	mxs_nand_return_dma_descs(nand_info);
}

/*
 * Write data to NAND.
 */
static void mxs_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				int length)
{
	struct nand_chip *nand = mtd->priv;
	struct mxs_nand_info *nand_info = nand->priv;
	struct mxs_dma_desc *d;
	uint32_t channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + nand_info->cur_chip;
	int ret;

	if (length > NAND_MAX_PAGESIZE) {
		printf("MXS NAND: DMA buffer too big\n");
		return;
	}

	if (!buf) {
		printf("MXS NAND: DMA buffer is NULL\n");
		return;
	}

	memcpy(nand_info->data_buf, buf, length);

	/* Compile the DMA descriptor - a descriptor that writes data. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_DMA_READ | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM | MXS_DMA_DESC_WAIT4END |
		(4 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
		(length << MXS_DMA_DESC_BYTES_OFFSET);

	d->cmd.address = (dma_addr_t)nand_info->data_buf;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WRITE |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		length;

	mxs_dma_desc_append(channel, d);

	/* Execute the DMA chain. */
	ret = mxs_dma_go(channel);
	if (ret)
		printf("MXS NAND: DMA write error\n");

	mxs_nand_return_dma_descs(nand_info);
}

/*
 * Read a single byte from NAND.
 */
static uint8_t mxs_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t buf;
	mxs_nand_read_buf(mtd, &buf, 1);
	return buf;
}

/*
 * Read a page from NAND.
 */
static int mxs_nand_ecc_read_page(struct mtd_info *mtd, struct nand_chip *nand,
					uint8_t *buf)
{
	struct mxs_nand_info *nand_info = nand->priv;
	struct mxs_dma_desc *d;
	uint32_t channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + nand_info->cur_chip;
	uint32_t corrected = 0, failed = 0;
	uint8_t	*status;
	int i, ret;

	/* Compile the DMA descriptor - wait for ready. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_CHAIN |
		MXS_DMA_DESC_NAND_WAIT_4_READY | MXS_DMA_DESC_WAIT4END |
		(1 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WAIT_FOR_READY |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA;

	mxs_dma_desc_append(channel, d);

	/* Compile the DMA descriptor - enable the BCH block and read. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_CHAIN |
		MXS_DMA_DESC_WAIT4END |	(6 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_READ |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		(mtd->writesize + mtd->oobsize);
	d->cmd.pio_words[1] = 0;
	d->cmd.pio_words[2] =
		GPMI_ECCCTRL_ENABLE_ECC |
		GPMI_ECCCTRL_ECC_CMD_DECODE |
		GPMI_ECCCTRL_BUFFER_MASK_BCH_PAGE;
	d->cmd.pio_words[3] = mtd->writesize + mtd->oobsize;
	d->cmd.pio_words[4] = (dma_addr_t)nand_info->data_buf;
	d->cmd.pio_words[5] = (dma_addr_t)nand_info->oob_buf;

	mxs_dma_desc_append(channel, d);

	/* Compile the DMA descriptor - disable the BCH block. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_CHAIN |
		MXS_DMA_DESC_NAND_WAIT_4_READY | MXS_DMA_DESC_WAIT4END |
		(3 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WAIT_FOR_READY |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		(mtd->writesize + mtd->oobsize);
	d->cmd.pio_words[1] = 0;
	d->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(channel, d);

	/* Compile the DMA descriptor - deassert the NAND lock and interrupt. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM;

	d->cmd.address = 0;

	mxs_dma_desc_append(channel, d);

	/* Execute the DMA chain. */
	ret = mxs_dma_go(channel);
	if (ret) {
		printf("MXS NAND: DMA read error\n");
		goto rtn;
	}

	ret = mxs_nand_wait_for_bch_complete();
	if (ret) {
		printf("MXS NAND: BCH read timeout\n");
		goto rtn;
	}

	/* Read DMA completed, now do the mark swapping. */
	mxs_nand_swap_block_mark(mtd, nand_info->data_buf, nand_info->oob_buf);

	/* Loop over status bytes, accumulating ECC status. */
	status = nand_info->oob_buf + mxs_nand_aux_status_offset();
	for (i = 0; i < mxs_nand_ecc_chunk_cnt(mtd->writesize); i++) {
		if (status[i] == 0x00)
			continue;

		if (status[i] == 0xff)
			continue;

		if (status[i] == 0xfe) {
			failed++;
			continue;
		}

		corrected += status[i];
	}

	/* Propagate ECC status to the owning MTD. */
	mtd->ecc_stats.failed += failed;
	mtd->ecc_stats.corrected += corrected;

	/*
	 * It's time to deliver the OOB bytes. See mxs_nand_ecc_read_oob() for
	 * details about our policy for delivering the OOB.
	 *
	 * We fill the caller's buffer with set bits, and then copy the block
	 * mark to the caller's buffer. Note that, if block mark swapping was
	 * necessary, it has already been done, so we can rely on the first
	 * byte of the auxiliary buffer to contain the block mark.
	 */
	memset(nand->oob_poi, 0xff, mtd->oobsize);

	nand->oob_poi[0] = nand_info->oob_buf[0];

	memcpy(buf, nand_info->data_buf, mtd->writesize);

rtn:
	mxs_nand_return_dma_descs(nand_info);

	return ret;
}

/*
 * Write a page to NAND.
 */
static void mxs_nand_ecc_write_page(struct mtd_info *mtd,
				struct nand_chip *nand, const uint8_t *buf)
{
	struct mxs_nand_info *nand_info = nand->priv;
	struct mxs_dma_desc *d;
	uint32_t channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + nand_info->cur_chip;
	int ret;

	memcpy(nand_info->data_buf, buf, mtd->writesize);
	memcpy(nand_info->oob_buf, nand->oob_poi, mtd->oobsize);

	/* Handle block mark swapping. */
	mxs_nand_swap_block_mark(mtd, nand_info->data_buf, nand_info->oob_buf);

	/* Compile the DMA descriptor - write data. */
	d = mxs_nand_get_dma_desc(nand_info);
	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM | MXS_DMA_DESC_WAIT4END |
		(6 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WRITE |
		GPMI_CTRL0_WORD_LENGTH |
		(nand_info->cur_chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA;
	d->cmd.pio_words[1] = 0;
	d->cmd.pio_words[2] =
		GPMI_ECCCTRL_ENABLE_ECC |
		GPMI_ECCCTRL_ECC_CMD_ENCODE |
		GPMI_ECCCTRL_BUFFER_MASK_BCH_PAGE;
	d->cmd.pio_words[3] = (mtd->writesize + mtd->oobsize);
	d->cmd.pio_words[4] = (dma_addr_t)nand_info->data_buf;
	d->cmd.pio_words[5] = (dma_addr_t)nand_info->oob_buf;

	mxs_dma_desc_append(channel, d);

	/* Execute the DMA chain. */
	ret = mxs_dma_go(channel);
	if (ret) {
		printf("MXS NAND: DMA write error\n");
		goto rtn;
	}

	ret = mxs_nand_wait_for_bch_complete();
	if (ret) {
		printf("MXS NAND: BCH write timeout\n");
		goto rtn;
	}

rtn:
	mxs_nand_return_dma_descs(nand_info);
}

/*
 * Read OOB from NAND.
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code.
 */
static int mxs_nand_hook_read_oob(struct mtd_info *mtd, loff_t from,
					struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	struct mxs_nand_info *nand_info = chip->priv;
	int ret;

	if (ops->mode == MTD_OOB_RAW)
		nand_info->raw_oob_mode = 1;
	else
		nand_info->raw_oob_mode = 0;

	ret = nand_info->hooked_read_oob(mtd, from, ops);

	nand_info->raw_oob_mode = 0;

	return ret;
}

/*
 * Write OOB to NAND.
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code.
 */
static int mxs_nand_hook_write_oob(struct mtd_info *mtd, loff_t to,
					struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	struct mxs_nand_info *nand_info = chip->priv;
	int ret;

	if (ops->mode == MTD_OOB_RAW)
		nand_info->raw_oob_mode = 1;
	else
		nand_info->raw_oob_mode = 0;

	ret = nand_info->hooked_write_oob(mtd, to, ops);

	nand_info->raw_oob_mode = 0;

	return ret;
}

/*
 * Mark a block bad in NAND.
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code.
 */
static int mxs_nand_hook_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	struct mxs_nand_info *nand_info = chip->priv;
	int ret;

	nand_info->marking_block_bad = 1;

	ret = nand_info->hooked_block_markbad(mtd, ofs);

	nand_info->marking_block_bad = 0;

	return ret;
}

/*
 * There are several places in this driver where we have to handle the OOB and
 * block marks. This is the function where things are the most complicated, so
 * this is where we try to explain it all. All the other places refer back to
 * here.
 *
 * These are the rules, in order of decreasing importance:
 *
 * 1) Nothing the caller does can be allowed to imperil the block mark, so all
 *    write operations take measures to protect it.
 *
 * 2) In read operations, the first byte of the OOB we return must reflect the
 *    true state of the block mark, no matter where that block mark appears in
 *    the physical page.
 *
 * 3) ECC-based read operations return an OOB full of set bits (since we never
 *    allow ECC-based writes to the OOB, it doesn't matter what ECC-based reads
 *    return).
 *
 * 4) "Raw" read operations return a direct view of the physical bytes in the
 *    page, using the conventional definition of which bytes are data and which
 *    are OOB. This gives the caller a way to see the actual, physical bytes
 *    in the page, without the distortions applied by our ECC engine.
 *
 * What we do for this specific read operation depends on whether we're doing
 * "raw" read, or an ECC-based read.
 *
 * It turns out that knowing whether we want an "ECC-based" or "raw" read is not
 * easy. When reading a page, for example, the NAND Flash MTD code calls our
 * ecc.read_page or ecc.read_page_raw function. Thus, the fact that MTD wants an
 * ECC-based or raw view of the page is implicit in which function it calls
 * (there is a similar pair of ECC-based/raw functions for writing).
 *
 * Since MTD assumes the OOB is not covered by ECC, there is no pair of
 * ECC-based/raw functions for reading or or writing the OOB. The fact that the
 * caller wants an ECC-based or raw view of the page is not propagated down to
 * this driver.
 *
 * Since our OOB *is* covered by ECC, we need this information. So, we hook the
 * ecc.read_oob and ecc.write_oob function pointers in the owning
 * struct mtd_info with our own functions. These hook functions set the
 * raw_oob_mode field so that, when control finally arrives here, we'll know
 * what to do.
 */
static int mxs_nand_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *nand,
				int page, int cmd)
{
	struct mxs_nand_info *nand_info = nand->priv;
	int column;

	/*
	 * First, fill in the OOB buffer. If we're doing a raw read, we need to
	 * get the bytes from the physical page. If we're not doing a raw read,
	 * we need to fill the buffer with set bits.
	 */
	if (nand_info->raw_oob_mode && nand_info->version > GPMI_VERSION_TYPE_MX23) {
		/*
		 * If control arrives here, we're doing a "raw" read. Send the
		 * command to read the conventional OOB and read it.
		 */
		nand->cmdfunc(mtd, NAND_CMD_READ0, mtd->writesize, page);
		nand->read_buf(mtd, nand->oob_poi, mtd->oobsize);
	} else {
		/*
		 * If control arrives here, we're not doing a "raw" read. Fill
		 * the OOB buffer with set bits and correct the block mark.
		 */
		memset(nand->oob_poi, 0xff, mtd->oobsize);

		column = nand_info->version == GPMI_VERSION_TYPE_MX23 ? 0 : mtd->writesize;
		nand->cmdfunc(mtd, NAND_CMD_READ0, column, page);
		mxs_nand_read_buf(mtd, nand->oob_poi, 1);
	}

	return 0;

}

/*
 * Write OOB data to NAND.
 */
static int mxs_nand_ecc_write_oob(struct mtd_info *mtd, struct nand_chip *nand,
					int page)
{
	struct mxs_nand_info *nand_info = nand->priv;
	int column;
	uint8_t block_mark = 0;

	/*
	 * There are fundamental incompatibilities between the i.MX GPMI NFC and
	 * the NAND Flash MTD model that make it essentially impossible to write
	 * the out-of-band bytes.
	 *
	 * We permit *ONE* exception. If the *intent* of writing the OOB is to
	 * mark a block bad, we can do that.
	 */

	if (!nand_info->marking_block_bad) {
		printf("NXS NAND: Writing OOB isn't supported\n");
		return -EIO;
	}

	column = nand_info->version == GPMI_VERSION_TYPE_MX23 ? 0 : mtd->writesize;
	/* Write the block mark. */
	nand->cmdfunc(mtd, NAND_CMD_SEQIN, column, page);
	nand->write_buf(mtd, &block_mark, 1);
	nand->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	/* Check if it worked. */
	if (nand->waitfunc(mtd, nand) & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

/*
 * Claims all blocks are good.
 *
 * In principle, this function is *only* called when the NAND Flash MTD system
 * isn't allowed to keep an in-memory bad block table, so it is forced to ask
 * the driver for bad block information.
 *
 * In fact, we permit the NAND Flash MTD system to have an in-memory BBT, so
 * this function is *only* called when we take it away.
 *
 * Thus, this function is only called when we want *all* blocks to look good,
 * so it *always* return success.
 */
static int mxs_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	return 0;
}

/*
 * Nominally, the purpose of this function is to look for or create the bad
 * block table. In fact, since the we call this function at the very end of
 * the initialization process started by nand_scan(), and we doesn't have a
 * more formal mechanism, we "hook" this function to continue init process.
 *
 * At this point, the physical NAND Flash chips have been identified and
 * counted, so we know the physical geometry. This enables us to make some
 * important configuration decisions.
 *
 * The return value of this function propogates directly back to this driver's
 * call to nand_scan(). Anything other than zero will cause this driver to
 * tear everything down and declare failure.
 */
static int mxs_nand_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	struct mxs_nand_info *nand_info = nand->priv;
	void __iomem *bch_regs = (void __iomem *)MXS_BCH_BASE;
	uint32_t tmp;
	int ret;

	/* Reset BCH. Don't use SFTRST on MX23 due to Errata #2847 */
	ret = mxs_reset_block(bch_regs + BCH_CTRL,
				nand_info->version == GPMI_VERSION_TYPE_MX23);
	if (ret)
		return ret;

	/* Configure layout 0 */
	tmp = (mxs_nand_ecc_chunk_cnt(mtd->writesize) - 1)
		<< BCH_FLASHLAYOUT0_NBLOCKS_OFFSET;
	tmp |= MXS_NAND_METADATA_SIZE << BCH_FLASHLAYOUT0_META_SIZE_OFFSET;
	tmp |= (mxs_nand_get_ecc_strength(mtd->writesize, mtd->oobsize) >> 1)
		<< BCH_FLASHLAYOUT0_ECC0_OFFSET;
	tmp |= MXS_NAND_CHUNK_DATA_CHUNK_SIZE;
	writel(tmp, bch_regs + BCH_FLASH0LAYOUT0);

	tmp = (mtd->writesize + mtd->oobsize)
		<< BCH_FLASHLAYOUT1_PAGE_SIZE_OFFSET;
	tmp |= (mxs_nand_get_ecc_strength(mtd->writesize, mtd->oobsize) >> 1)
		<< BCH_FLASHLAYOUT1_ECCN_OFFSET;
	tmp |= MXS_NAND_CHUNK_DATA_CHUNK_SIZE;
	writel(tmp, bch_regs + BCH_FLASH0LAYOUT1);

	/* Set *all* chip selects to use layout 0 */
	writel(0, bch_regs + BCH_LAYOUTSELECT);

	/* Enable BCH complete interrupt */
	writel(BCH_CTRL_COMPLETE_IRQ_EN, bch_regs + BCH_CTRL + BIT_SET);

	/* Hook some operations at the MTD level. */
	if (mtd->read_oob != mxs_nand_hook_read_oob) {
		nand_info->hooked_read_oob = mtd->read_oob;
		mtd->read_oob = mxs_nand_hook_read_oob;
	}

	if (mtd->write_oob != mxs_nand_hook_write_oob) {
		nand_info->hooked_write_oob = mtd->write_oob;
		mtd->write_oob = mxs_nand_hook_write_oob;
	}

	if (mtd->block_markbad != mxs_nand_hook_block_markbad) {
		nand_info->hooked_block_markbad = mtd->block_markbad;
		mtd->block_markbad = mxs_nand_hook_block_markbad;
	}

	/* We use the reference implementation for bad block management. */
	return nand_default_bbt(mtd);
}

/*
 * Allocate DMA buffers
 */
int mxs_nand_alloc_buffers(struct mxs_nand_info *nand_info)
{
	uint8_t *buf;
	const int size = NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE;

	/* DMA buffers */
	buf = dma_alloc_coherent(size);
	if (!buf) {
		printf("MXS NAND: Error allocating DMA buffers\n");
		return -ENOMEM;
	}

	memset(buf, 0, size);

	nand_info->data_buf = buf;
	nand_info->oob_buf = buf + NAND_MAX_PAGESIZE;

	/* Command buffers */
	nand_info->cmd_buf = dma_alloc_coherent(MXS_NAND_COMMAND_BUFFER_SIZE);
	if (!nand_info->cmd_buf) {
		free(buf);
		printf("MXS NAND: Error allocating command buffers\n");
		return -ENOMEM;
	}
	memset(nand_info->cmd_buf, 0, MXS_NAND_COMMAND_BUFFER_SIZE);
	nand_info->cmd_queue_len = 0;

	return 0;
}

/*
 * Initializes the NFC hardware.
 */
int mxs_nand_hw_init(struct mxs_nand_info *info)
{
	void __iomem *gpmi_regs = (void *)MXS_GPMI_BASE;
	void __iomem *bch_regs = (void __iomem *)MXS_BCH_BASE;
	int i = 0, ret;
	u32 val;

	info->desc = malloc(sizeof(struct mxs_dma_desc *) *
				MXS_NAND_DMA_DESCRIPTOR_COUNT);
	if (!info->desc)
		goto err1;

	/* Allocate the DMA descriptors. */
	for (i = 0; i < MXS_NAND_DMA_DESCRIPTOR_COUNT; i++) {
		info->desc[i] = mxs_dma_desc_alloc();
		if (!info->desc[i])
			goto err2;
	}

	/* Init the DMA controller. */
	mxs_dma_init();

	imx_enable_nandclk();

	/* Reset the GPMI block. */
	ret = mxs_reset_block(gpmi_regs + GPMI_CTRL0, 0);
	if (ret)
		return ret;

	val = readl(gpmi_regs + GPMI_VERSION);
	info->version = val >> GPMI_VERSION_MINOR_OFFSET;

	/* Reset BCH. Don't use SFTRST on MX23 due to Errata #2847 */
	ret = mxs_reset_block(bch_regs + BCH_CTRL,
				info->version == GPMI_VERSION_TYPE_MX23);
	if (ret)
		return ret;

	/*
	 * Choose NAND mode, set IRQ polarity, disable write protection and
	 * select BCH ECC.
	 */
	val = readl(gpmi_regs + GPMI_CTRL1);
	val &= ~GPMI_CTRL1_GPMI_MODE;
	val |= GPMI_CTRL1_ATA_IRQRDY_POLARITY | GPMI_CTRL1_DEV_RESET |
		GPMI_CTRL1_BCH_MODE;
	writel(val, gpmi_regs + GPMI_CTRL1);

	return 0;

err2:
	free(info->desc);
err1:
	for (--i; i >= 0; i--)
		mxs_dma_desc_free(info->desc[i]);
	printf("MXS NAND: Unable to allocate DMA descriptors\n");
	return -ENOMEM;
}

static int mxs_nand_probe(struct device_d *dev)
{
	struct mxs_nand_info *nand_info;
	struct nand_chip *nand;
	struct mtd_info *mtd;
	int err;

	nand_info = kzalloc(sizeof(struct mxs_nand_info), GFP_KERNEL);
	if (!nand_info) {
		printf("MXS NAND: Failed to allocate private data\n");
		return -ENOMEM;
	}

	/* XXX: Remove u-boot specific access pointers and use io_base instead? */
	nand_info->io_base = dev_request_mem_region(dev, 0);

	err = mxs_nand_alloc_buffers(nand_info);
	if (err)
		goto err1;

	err = mxs_nand_hw_init(nand_info);
	if (err)
		goto err2;

	memset(&fake_ecc_layout, 0, sizeof(fake_ecc_layout));

	/* structures must be linked */
	nand = &nand_info->nand_chip;
	mtd = &nand_info->mtd;
	mtd->priv = nand;
	mtd->parent = dev;

	nand->priv = nand_info;

	nand->cmd_ctrl		= mxs_nand_cmd_ctrl;

	nand->dev_ready		= mxs_nand_device_ready;
	nand->select_chip	= mxs_nand_select_chip;
	nand->block_bad		= mxs_nand_block_bad;
	nand->scan_bbt		= mxs_nand_scan_bbt;

	nand->read_byte		= mxs_nand_read_byte;

	nand->read_buf		= mxs_nand_read_buf;
	nand->write_buf		= mxs_nand_write_buf;

	nand->ecc.read_page	= mxs_nand_ecc_read_page;
	nand->ecc.write_page	= mxs_nand_ecc_write_page;
	nand->ecc.read_oob	= mxs_nand_ecc_read_oob;
	nand->ecc.write_oob	= mxs_nand_ecc_write_oob;

	nand->ecc.layout	= &fake_ecc_layout;
	nand->ecc.mode		= NAND_ECC_HW;
	nand->ecc.bytes		= 9;
	nand->ecc.size		= 512;

	/* first scan to find the device and get the page size */
	err = nand_scan_ident(mtd, 1);
	if (err)
		goto err2;

	nand->options |= NAND_NO_SUBPAGE_WRITE;

	/* second phase scan */
	err = nand_scan_tail(mtd);
	if (err)
		goto err2;

	return add_mtd_nand_device(mtd, "nand");
err2:
	free(nand_info->data_buf);
	free(nand_info->cmd_buf);
err1:
	free(nand_info);
	return err;
}

static struct driver_d mxs_nand_driver = {
	.name  = "mxs_nand",
	.probe = mxs_nand_probe,
};
device_platform_driver(mxs_nand_driver);

MODULE_AUTHOR("Denx Software Engeneering and Wolfram Sang");
MODULE_DESCRIPTION("MXS NAND MTD driver");
MODULE_LICENSE("GPL");
