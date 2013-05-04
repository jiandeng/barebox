/*
 * tftp.c - (up)load tftp files
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
#include <command.h>
#include <linux/stat.h>
#include <getopt.h>
#include <fcntl.h>
#include <libgen.h>
#include <fs.h>
#include <net.h>
#include <libbb.h>

#define TFTP_MOUNT_PATH	"/.tftp_tmp_path"

static int do_tftpb(int argc, char *argv[])
{
	char *source, *dest, *freep;
	int opt;
	unsigned long flags;
	int tftp_push = 0;
	int ret;
	IPaddr_t ip;

	while ((opt = getopt(argc, argv, "p")) > 0) {
		switch(opt) {
		case 'p':
			tftp_push = 1;
			break;
		default:
			return COMMAND_ERROR_USAGE;
		}
	}

	if (argc <= optind)
		return COMMAND_ERROR_USAGE;

	source = argv[optind++];

	if (argc == optind)
		dest = basename(source);
	else
		dest = argv[optind];

	if (tftp_push) {
		dest = freep = asprintf("%s/%s", TFTP_MOUNT_PATH, dest);
		flags = O_RDONLY;
	} else {
		source = freep = asprintf("%s/%s", TFTP_MOUNT_PATH, source);
		flags = O_WRONLY | O_CREAT;
	}

	if (!freep)
		return -ENOMEM;

	ret = make_directory(TFTP_MOUNT_PATH);
	if (ret)
		goto err_free;

	ip = net_get_serverip();
	ret = mount(ip_to_string(ip), "tftp", TFTP_MOUNT_PATH);
	if (ret)
		goto err_rmdir;

	debug("%s: %s -> %s\n", __func__, source, dest);

	ret = copy_file(source, dest, 1);

	umount(TFTP_MOUNT_PATH);

err_rmdir:
	rmdir(TFTP_MOUNT_PATH);

err_free:
	free(freep);

	return ret;
}

BAREBOX_CMD_HELP_START(tftp)
BAREBOX_CMD_HELP_USAGE("tftp [-p] <source> [dest]\n")
BAREBOX_CMD_HELP_SHORT("Load a file from or upload to TFTP server.\n")
BAREBOX_CMD_HELP_END

BAREBOX_CMD_START(tftp)
	.cmd		= do_tftpb,
	.usage		= "(up-)Load file using tftp protocol",
	BAREBOX_CMD_HELP(cmd_tftp_help)
BAREBOX_CMD_END
