/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2017 Kalray Inc.
 */

#include <common.h>
#include <command.h>

static int do_cpuinfo(int argc, char *argv[])
{
	printk("Kalray Coolidge\n");

	return 0;
}

BAREBOX_CMD_START(cpuinfo)
	.cmd            = do_cpuinfo,
	BAREBOX_CMD_DESC("show CPU information")
	BAREBOX_CMD_GROUP(CMD_GRP_INFO)
BAREBOX_CMD_END
