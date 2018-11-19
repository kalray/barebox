/*
 * xip - execute some code in flash
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
#include <command.h>
#include <complete.h>
#include <fs.h>
#include <fcntl.h>
#include <linux/ctype.h>
#include <errno.h>

void enter_xip(void);
void exit_xip(void);

static int do_xip(int argc, char *argv[])
{
	void *addr;
	int	(*func)(int argc, char *argv[]);

	if (argc < 2)
		return COMMAND_ERROR_USAGE;

	if (!isdigit(*argv[1])) {
		perror("Not an address");
		return(-EINVAL);
	}

	addr = (void *)simple_strtoul(argv[1], NULL, 16);
	printf("## Starting application at 0x%p ...\n", addr);

	enter_xip();
	func = addr;
	func(argc-1, &argv[1]);
	exit_xip();

	return 0;
}

BAREBOX_CMD_HELP_START(xip)
BAREBOX_CMD_HELP_TEXT("Start application at ADDR (reset spi flash and reconfigure ssi).")
BAREBOX_CMD_HELP_TEXT("")
BAREBOX_CMD_HELP_END

BAREBOX_CMD_START(xip)
	.cmd		= do_xip,
	BAREBOX_CMD_DESC("start application at address")
	BAREBOX_CMD_OPTS("ADDR [ARG...]")
	BAREBOX_CMD_GROUP(CMD_GRP_BOOT)
	BAREBOX_CMD_HELP(cmd_xip_help)
	BAREBOX_CMD_COMPLETE(command_var_complete)
BAREBOX_CMD_END
