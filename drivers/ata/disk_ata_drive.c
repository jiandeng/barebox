/*
 * Copyright (C) 2011 Juergen Beisert, Pengutronix
 *
 * Inspired from various soures like http://wiki.osdev.org/ATA_PIO_Mode,
 * u-boot and the linux kernel
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
 */

#include <common.h>
#include <xfuncs.h>
#include <io.h>
#include <malloc.h>
#include <errno.h>
#include <clock.h>
#include <block.h>
#include <ata_drive.h>
#include <disks.h>
#include <dma.h>

#define ata_id_u32(id,n)        \
        (((uint32_t) (id)[(n) + 1] << 16) | ((uint32_t) (id)[(n)]))
#define ata_id_u64(id,n)        \
        ( ((uint64_t) (id)[(n) + 3] << 48) | \
          ((uint64_t) (id)[(n) + 2] << 32) | \
          ((uint64_t) (id)[(n) + 1] << 16) | \
          ((uint64_t) (id)[(n) + 0]) )

#define ata_id_has_lba(id)               ((id)[49] & (1 << 9))

enum {
	ATA_ID_SERNO		= 10,
#define ATA_ID_SERNO_LEN 20
	ATA_ID_FW_REV		= 23,
#define ATA_ID_FW_REV_LEN 8
	ATA_ID_PROD		= 27,
#define ATA_ID_PROD_LEN 40
	ATA_ID_CAPABILITY	= 49,
	ATA_ID_FIELD_VALID	= 53,
	ATA_ID_LBA_CAPACITY	= 60,
	ATA_ID_MWDMA_MODES	= 63,
	ATA_ID_PIO_MODES	= 64,
	ATA_ID_QUEUE_DEPTH	= 75,
	ATA_ID_MAJOR_VER	= 80,
	ATA_ID_COMMAND_SET_1	= 82,
	ATA_ID_COMMAND_SET_2	= 83,
	ATA_ID_CFSSE		= 84,
	ATA_ID_CFS_ENABLE_1	= 85,
	ATA_ID_CFS_ENABLE_2	= 86,
	ATA_ID_CSF_DEFAULT	= 87,
	ATA_ID_UDMA_MODES	= 88,
	ATA_ID_HW_CONFIG	= 93,
	ATA_ID_LBA_CAPACITY_2	= 100,
};

static int ata_id_is_valid(const uint16_t *id)
{
	if ((id[ATA_ID_FIELD_VALID] & 1) == 0) {
		pr_debug("Drive's ID seems invalid\n");
		return -EINVAL;
	}

	return 0;
}

static inline int ata_id_has_lba48(const uint16_t *id)
{
	if ((id[ATA_ID_COMMAND_SET_2] & 0xC000) != 0x4000)
		return 0;
	if (!ata_id_u64(id, ATA_ID_LBA_CAPACITY_2))
		return 0;
	return id[ATA_ID_COMMAND_SET_2] & (1 << 10);
}

static uint64_t ata_id_n_sectors(uint16_t *id)
{
	if (ata_id_has_lba(id)) {
		if (ata_id_has_lba48(id))
			return ata_id_u64(id, ATA_ID_LBA_CAPACITY_2);
		else
			return ata_id_u32(id, ATA_ID_LBA_CAPACITY);
	}

	return 0;
}

static void ata_id_string(const uint16_t *id, unsigned char *s,
				unsigned ofs, unsigned len)
{
	unsigned c;

	while (len > 0) {
		c = id[ofs] >> 8;
		*s = c;
		s++;

		c = id[ofs] & 0xff;
		*s = c;
		s++;

		ofs++;
		len -= 2;
	}
}

static void ata_id_c_string(const uint16_t *id, unsigned char *s,
				unsigned ofs, unsigned len)
{
	unsigned char *p;

	ata_id_string(id, s, ofs, len - 1);

	p = s + strnlen((char *)s, len - 1);
	while (p > s && p[-1] == ' ')
		p--;
	*p = '\0';
}

static void __maybe_unused ata_dump_id(uint16_t *id)
{
	unsigned char serial[ATA_ID_SERNO_LEN + 1];
	unsigned char firmware[ATA_ID_FW_REV_LEN + 1];
	unsigned char product[ATA_ID_PROD_LEN + 1];
	uint64_t n_sectors;

	/* Serial number */
	ata_id_c_string(id, serial, ATA_ID_SERNO, sizeof(serial));
	printf("S/N: %s\n\r", serial);

	/* Firmware version */
	ata_id_c_string(id, firmware, ATA_ID_FW_REV, sizeof(firmware));
	printf("Firmware version: %s\n\r", firmware);

	/* Product model */
	ata_id_c_string(id, product, ATA_ID_PROD, sizeof(product));
	printf("Product model number: %s\n\r", product);

	/* Total sectors of device  */
	n_sectors = ata_id_n_sectors(id);
	printf("Capablity: %lld sectors\n\r", n_sectors);

	printf ("id[49]: capabilities = 0x%04x\n"
		"id[53]: field valid = 0x%04x\n"
		"id[63]: mwdma = 0x%04x\n"
		"id[64]: pio = 0x%04x\n"
		"id[75]: queue depth = 0x%04x\n",
		id[ATA_ID_CAPABILITY],
		id[ATA_ID_FIELD_VALID],
		id[ATA_ID_MWDMA_MODES],
		id[ATA_ID_PIO_MODES],
		id[ATA_ID_QUEUE_DEPTH]);

	printf ("id[76]: sata capablity = 0x%04x\n"
		"id[78]: sata features supported = 0x%04x\n"
		"id[79]: sata features enable = 0x%04x\n",
		id[76], /* FIXME */
		id[78], /* FIXME */
		id[79]); /* FIXME */

	printf ("id[80]: major version = 0x%04x\n"
		"id[81]: minor version = 0x%04x\n"
		"id[82]: command set supported 1 = 0x%04x\n"
		"id[83]: command set supported 2 = 0x%04x\n"
		"id[84]: command set extension = 0x%04x\n",
		id[ATA_ID_MAJOR_VER],
		id[81], /* FIXME */
		id[ATA_ID_COMMAND_SET_1],
		id[ATA_ID_COMMAND_SET_2],
		id[ATA_ID_CFSSE]);
	printf ("id[85]: command set enable 1 = 0x%04x\n"
		"id[86]: command set enable 2 = 0x%04x\n"
		"id[87]: command set default = 0x%04x\n"
		"id[88]: udma = 0x%04x\n"
		"id[93]: hardware reset result = 0x%04x\n",
		id[ATA_ID_CFS_ENABLE_1],
		id[ATA_ID_CFS_ENABLE_2],
		id[ATA_ID_CSF_DEFAULT],
		id[ATA_ID_UDMA_MODES],
		id[ATA_ID_HW_CONFIG]);
}

/**
 * Swap little endian data on demand
 * @param buf Buffer with little endian word data
 * @param wds 16 bit word count
 *
 * ATA disks report their ID data in little endian notation on a 16 bit word
 * base. So swap the buffer content if the running CPU differs in their
 * endiaeness.
 */
static void ata_fix_endianess(uint16_t *buf, unsigned wds)
{
#ifdef __BIG_ENDIAN
	unsigned u;

	for (u = 0; u < wds; u++)
		buf[u] = le16_to_cpu(buf[u]);
#endif
}

/**
 * Read a chunk of sectors from the drive
 * @param blk All info about the block device we need
 * @param buffer Buffer to read into
 * @param block Sector's LBA number to start read from
 * @param num_blocks Sector count to read
 * @return 0 on success, anything else on failure
 *
 * This routine expects the buffer has the correct size to store all data!
 *
 * @note Due to 'block' is of type 'int' only small disks can be handled!
 * @todo Optimize the read loop
 */
static int ata_read(struct block_device *blk, void *buffer, int block,
				int num_blocks)
{
	struct ata_port *port = container_of(blk, struct ata_port, blk);

	return port->ops->read(port, buffer, block, num_blocks);
}

/**
 * Write a chunk of sectors into the drive
 * @param blk All info about the block device we need
 * @param buffer Buffer to write from
 * @param block Sector's number to start write to
 * @param num_blocks Sector count to write
 * @return 0 on success, anything else on failure
 *
 * This routine expects the buffer has the correct size to read all data!
 *
 * @note Due to 'block' is of type 'int' only small disks can be handled!
 * @todo Optimize the write loop
 */
static int __maybe_unused ata_write(struct block_device *blk,
				const void *buffer, int block, int num_blocks)
{
	struct ata_port *port = container_of(blk, struct ata_port, blk);

	return port->ops->write(port, buffer, block, num_blocks);
}

static struct block_device_ops ata_ops = {
	.read = ata_read,
#ifdef CONFIG_BLOCK_WRITE
	.write = ata_write,
#endif
};

static int ata_port_init(struct ata_port *port)
{
	int rc;
	struct ata_port_operations *ops = port->ops;
	struct device_d *dev = &port->class_dev;

	if (ops->init) {
		rc = ops->init(port);
		if (rc)
			return rc;
	}

	port->id = dma_alloc(SECTOR_SIZE);

	port->blk.dev = dev;
	port->blk.ops = &ata_ops;

	if (ops->reset) {
		rc = ops->reset(port);
		if (rc) {
			dev_dbg(dev, "Resetting failed\n");
			goto on_error;
		}
	}

	rc = ops->read_id(port, port->id);
	if (rc != 0) {
		dev_dbg(dev, "Reading ID failed\n");
		goto on_error;
	}

	ata_fix_endianess(port->id, SECTOR_SIZE / sizeof(uint16_t));

	rc = ata_id_is_valid(port->id);
	if (rc) {
		dev_err(dev, "ata id invalid\n");
		free(port->id);
		return rc;
	}

#ifdef DEBUG
	ata_dump_id(port->id);
#endif
	rc = cdev_find_free_index("ata");
	if (rc == -1)
		pr_err("Cannot find a free index for the disk node\n");

	port->blk.num_blocks = ata_id_n_sectors(port->id);
	port->blk.cdev.name = asprintf("ata%d", rc);
	port->blk.blockbits = SECTOR_SHIFT;

	rc = blockdevice_register(&port->blk);
	if (rc != 0) {
		dev_err(dev, "Failed to register blockdevice\n");
		goto on_error;
	}

	dev_info(dev, "registered /dev/%s\n", port->blk.cdev.name);

	/* create partitions on demand */
	rc = parse_partition_table(&port->blk);
	if (rc != 0)
		dev_warn(dev, "No partition table found\n");

	return 0;

on_error:
	return rc;
}

static int ata_set_probe(struct device_d *class_dev, struct param_d *param,
				const char *val)
{
	struct ata_port *port = container_of(class_dev, struct ata_port, class_dev);
	int ret, probe;

	if (port->initialized) {
		dev_info(class_dev, "already initialized\n");
		return 0;
	}

	probe = !!simple_strtoul(val, NULL, 0);
	if (!probe)
		return 0;

	ret = ata_port_init(port);
	if (ret)
		return ret;

	port->initialized = 1;

	return dev_param_set_generic(class_dev, param, "1");
}

/**
 * Register an ATA drive behind an IDE like interface
 * @param dev The interface device
 * @param io ATA register file description
 * @return 0 on success
 */
int ata_port_register(struct ata_port *port)
{
	int ret;

	port->class_dev.id = DEVICE_ID_DYNAMIC;
	strcpy(port->class_dev.name, "ata");
	port->class_dev.parent = port->dev;

	ret = register_device(&port->class_dev);
	if (ret)
		return ret;

	dev_add_param(&port->class_dev, "probe", ata_set_probe, NULL, 0);

	return ret;
}

/**
 * @file
 * @brief Generic ATA disk drive support
 *
 * Please be aware: This driver covers only a subset of the available ATA drives
 *
 * @todo Support for disks larger than 4 GiB
 * @todo LBA48
 * @todo CHS
 */
