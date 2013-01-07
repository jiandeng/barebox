/*
 * Based on nandtest.c source in mtd-utils package.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <common.h>
#include <command.h>
#include <fs.h>
#include <errno.h>
#include <malloc.h>
#include <getopt.h>
#include <ioctl.h>
#include <linux/mtd/mtd-abi.h>
#include <fcntl.h>
#include <stdlib.h>

/*
 * Structures for flash memory information.
 */
static struct mtd_info_user nandinfo;
static struct mtd_ecc_stats oldstats, newstats;

/* Read nandraw device */
static int dump_nandraw(char *dev, int block, int page)
{
	unsigned char *buf;
	int i = 0;
	int fd = -1;
	int ret = -1;
	int length = -1;
	int offset = -1;

	/* Open device */
	printf("Open device %s\n", dev);	
	fd = open(dev, O_RDONLY);
	if (fd < 0) {
		perror("open");
		return COMMAND_ERROR_USAGE;
	}

	/* Getting flash information. */
	ret = ioctl(fd, MEMGETINFO, &nandinfo);
	if (ret < 0) {
		perror("MEMGETINFO");
		goto err;
	}
	
	/* Calculate the offset and length to read */
	length = nandinfo.writesize + nandinfo.oobsize;
	offset = block * nandinfo.erasesize + page * nandinfo.writesize;
	offset /= nandinfo.writesize;
	offset *= (nandinfo.writesize + nandinfo.oobsize);

	/* Allocate buffer for the page data */
	buf = malloc(length);
	if (!buf) {
		printf("Could not allocate %d bytes for buffer\n",
			length);
		goto err;
	}

	/* Seek to offset */
	ret = lseek(fd, offset, SEEK_SET);
	if (ret < 0) {
		perror("lseek");
		goto err2;
	}

	/* Read from flash and put it into buf  */
	ret = read(fd, buf, length);
	if (ret < 0) {
		perror("read");
		goto err2;
	}
	
	/* Dump the buffer */
	for(i = 0;i < length; i++)
	{
		if(0 == (i % 32)) {
			printf("0x%08x:", (unsigned int)(offset + i));
		}
		printf(" %02x", buf[i]);
		if(31 == (i % 32)) {
			printf("\n");
		}
		else if(3 == (i % 4)) {
			printf(" ");
		}
	}

	/* Close device */
	ret = close(fd);
	if (ret < 0) {
		perror("close");
		goto err2;
	}

	free(buf);
	return 0;
	
err2:
	free(buf);
err:
	printf("Error occurred.\n");
	close(fd);
	return 1;
}

/* Read nandoob device */
static int dump_nandoob(char *dev, int block, int page)
{
	unsigned char *buf;
	int i = 0;
	int fd = -1;
	int ret = -1;
	int length = -1;
	int offset = -1;

	/* Open device */
	printf("Open device %s\n", dev);	
	fd = open(dev, O_RDONLY);
	if (fd < 0) {
		perror("open");
		return COMMAND_ERROR_USAGE;
	}

	/* Getting flash information. */
	ret = ioctl(fd, MEMGETINFO, &nandinfo);
	if (ret < 0) {
		perror("MEMGETINFO");
		goto err;
	}
	
	/* Calculate the offset and length to read */
	length = nandinfo.oobsize;
	offset = block * nandinfo.erasesize + page * nandinfo.writesize;
	offset /= nandinfo.writesize;
	offset *= nandinfo.oobsize;

	/* Allocate buffer for the page data */
	buf = malloc(length);
	if (!buf) {
		printf("Could not allocate %d bytes for buffer\n",
			length);
		goto err;
	}

	/* Seek to offset */
	ret = lseek(fd, offset, SEEK_SET);
	if (ret < 0) {
		perror("lseek");
		goto err2;
	}

	/* Read from flash and put it into buf  */
	ret = read(fd, buf, length);
	if (ret < 0) {
		perror("read");
		goto err2;
	}
	
	/* Dump the buffer */
	for(i = 0;i < length; i++)
	{
		if(0 == (i % 32)) {
			printf("0x%08x:", (unsigned int)(offset + i));
		}
		printf(" %02x", buf[i]);
		if(31 == (i % 32)) {
			printf("\n");
		}
		else if(3 == (i % 4)) {
			printf(" ");
		}
	}

	/* Close device */
	ret = close(fd);
	if (ret < 0) {
		perror("close");
		goto err2;
	}

	free(buf);
	return 0;
	
err2:
	free(buf);
err:
	printf("Error occurred.\n");
	close(fd);
	return 1;
}

/* Read normal nand device */
static int dump_nand(char *dev, int block, int page)
{
	unsigned char *buf;
	int i = 0;
	int fd = -1;
	int ret = -1;
	int length = -1;
	int offset = -1;

	/* Open device */
	printf("Open device %s\n", dev);	
	fd = open(dev, O_RDONLY);
	if (fd < 0) {
		perror("open");
		return COMMAND_ERROR_USAGE;
	}

	/* Getting flash information. */
	ret = ioctl(fd, MEMGETINFO, &nandinfo);
	if (ret < 0) {
		perror("MEMGETINFO");
		goto err;
	}
	
	ret = ioctl(fd, ECCGETSTATS, &oldstats);
	if (ret < 0) {
		perror("ECCGETSTATS");
		goto err;
	}

	/* Calculate the offset and length to read */
	length = nandinfo.writesize;
	offset = block * nandinfo.erasesize + page * nandinfo.writesize;

	/* Allocate buffer for the page data */
	buf = malloc(length);
	if (!buf) {
		printf("Could not allocate %d bytes for buffer\n",
			length);
		goto err;
	}

	/* Seek to offset */
	ret = lseek(fd, offset, SEEK_SET);
	if (ret < 0) {
		perror("lseek");
		goto err2;
	}

	/* Read from flash and put it into buf  */
	ret = read(fd, buf, length);
	if (ret < 0) {
		perror("read");
		goto err2;
	}
	
	ret = ioctl(fd, ECCGETSTATS, &newstats);
	if (ret < 0) {
		perror("\nECCGETSTATS");
		return ret;
	}

	if (newstats.failed > oldstats.failed) {
		printf("\nECC failed\n");
		goto err2;
	}

	/* Dump the buffer */
	for(i = 0;i < length; i++)
	{
		if(0 == (i % 32)) {
			printf("0x%08x:", (unsigned int)(offset + i));
		}
		printf(" %02x", buf[i]);
		if(31 == (i % 32)) {
			printf("\n");
		}
		else if(3 == (i % 4)) {
			printf(" ");
		}
	}

	/* Close device */
	ret = close(fd);
	if (ret < 0) {
		perror("close");
		goto err2;
	}

	free(buf);
	return 0;
	
err2:
	free(buf);
err:
	printf("Error occurred.\n");
	close(fd);
	return 1;

}

/* Main program. */
static int do_nanddump(int argc, char *argv[])
{
	int ret = -1;
	int opt = -1;
	int mode = 'n';	// default to dump normal device
	off_t block = 0; // default to dump block 0
	off_t page = 0; // default to dump page 0
	char *dev = NULL;

	while ((opt = getopt(argc, argv, "d:b:p:ro")) > 0) {
		switch (opt) {
		case 'd':
			dev = optarg;
			break;
		case 'b':
			block = simple_strtoul(optarg, NULL, 0);
			break;
		case 'p':
			page = simple_strtoul(optarg, NULL, 0);
			break;
		case 'r':
		case 'n':
		case 'o':
			mode = opt;
			break;
		default:
			return COMMAND_ERROR_USAGE;
		}
	}

	switch(mode)
	{
		case 'n':
			if(dev == NULL)
				dev = "/dev/nand0";
			ret = dump_nand(dev, block, page);
			break;
		case 'r':
			if(dev == NULL)
				dev = "/dev/nandraw0";
			ret = dump_nandraw(dev, block, page);
			break;
		case 'o':
			if(dev == NULL)
				dev = "/dev/nand_oob0";
			ret = dump_nandoob(dev, block, page);
			break;
		default :
			break;
	}

	return ret;
}

/* String for usage of nanddump */
static const __maybe_unused char cmd_nanddump_help[] =
"Usage: nanddump [OPTION] <device>\n"
		"  -b	<block>, Block number to dump.\n"
		"  -p	<page>, Page number to dump.\n"
		"  -r	dump nandraw device.\n"
		"  -o	dump nandoob device.\n";

BAREBOX_CMD_START(nanddump)
	.cmd		= do_nanddump,
	.usage		= "NAND Dump",
	BAREBOX_CMD_HELP(cmd_nanddump_help)
BAREBOX_CMD_END
