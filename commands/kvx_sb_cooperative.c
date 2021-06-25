// SPDX-License-Identifier: GPL-2.0-or-later

#include <kvx_sb_cooperative.h>
#include <environment.h>
#include <globalvar.h>
#include <malloc.h>
#include <libfile.h>
#include <command.h>
#include <fs.h>
#include <stdio.h>

/*
 * Takes the ToC Image Header flag, and returns:
 * - An error, if the given ToC Image Header flag is not valid.
 * - The boot priority of the image.
 *
 * Between several images, the one with the highest priority should be the
 * one to be booted.
 */
static int sb_get_boot_prio(uint32_t toc_image_flag)
{
	switch (toc_image_flag) {
	case LEGACY_IMG:
		return 1;
	case CORRUPTED_IMG:
		return 0;
	case CURRENT_IMG:
		return 3;
	case RECOVERY_IMG:
		return 2;
	case VALID_IMG:
		return 4;
	case UNDER_EVAL_IMG:
		return 5;
	case NEW_IMG:
		return 1;
	case NO_IMG:
		return 0;
	default:
		return -1;
	}
}

static int sb_get_prio(const char *var_name, const char *toc_image_part)
{
	uint32_t sb_flag;
	int ret = 0, fd, *prio;

	fd = open_and_lseek(toc_image_part, O_RDONLY, TOC_IMAGE_FLAG_OFFSET);
	if (fd < 0) {
		perror("toc image open failed");
		return -EACCES;
	}
	if (read(fd, &sb_flag, sizeof(sb_flag)) != sizeof(sb_flag)) {
		perror("flag read failed");
		ret = -EIO;
		goto out;
	}

	prio = malloc(sizeof(int));
	if (prio == NULL) {
		perror("no memory to create priority");
		ret = -ENOMEM;
		goto out;
	}
	*prio = sb_get_boot_prio(sb_flag);
	globalvar_add_simple_int(var_name, prio, "%i");

out:
	close(fd);
	return ret;
}

BAREBOX_CMD_HELP_START(sb_inval_path)
BAREBOX_CMD_HELP_TEXT("Invalidates a boot path for cooperative boot")
BAREBOX_CMD_HELP_TEXT("")
BAREBOX_CMD_HELP_TEXT("Usage:")
BAREBOX_CMD_HELP_TEXT("sb_inval_path <file>")
BAREBOX_CMD_HELP_TEXT("<file> should be the mtd partition associated to the target path")
BAREBOX_CMD_HELP_TEXT("\texample: /dev/mtd0.fsbl_a")
BAREBOX_CMD_HELP_END

static int do_sb_inval_path(int argc, char *argv[])
{
	uint32_t inval_flag = CORRUPTED_IMG;
	int ret = 0, fd;

	if (argc != 2) {
		puts(cmd_sb_inval_path_help);
		return -1;
	}

	fd = open_and_lseek(argv[1], O_WRONLY, TOC_IMAGE_FLAG_OFFSET);
	if (fd < 0) {
		perror("toc image open failed");
		return -EACCES;
	}

	if (write(fd, &inval_flag, sizeof(inval_flag)) != sizeof(inval_flag)) {
		perror("flag inval failed");
		ret = -EIO;
		goto out;
	}

out:
	close(fd);
	return ret;
}

BAREBOX_CMD_START(sb_inval_path)
	.cmd		= do_sb_inval_path,
	BAREBOX_CMD_DESC("Mark a given path as invalid to the SecureBoot (Cooperative Boot Management)")
	BAREBOX_CMD_GROUP(CMD_GRP_ENV)
	BAREBOX_CMD_HELP(cmd_sb_inval_path_help)
BAREBOX_CMD_END

BAREBOX_CMD_HELP_START(sb_get_prio)
BAREBOX_CMD_HELP_TEXT("Creates a chosen int variable with the value set as the priority")
BAREBOX_CMD_HELP_TEXT("of the boot path according to the Secure Boot ToC Image flag.")
BAREBOX_CMD_HELP_TEXT("")
BAREBOX_CMD_HELP_TEXT("Usage:")
BAREBOX_CMD_HELP_TEXT("sb_get_prio <globalvar> <file>")
BAREBOX_CMD_HELP_TEXT("<globalvar> should be in the bootchooser priority format")
BAREBOX_CMD_HELP_TEXT("\texample: bootchooser.<target>.default_priority")
BAREBOX_CMD_HELP_TEXT("<file> should be the mtd partition associated to the target path")
BAREBOX_CMD_HELP_TEXT("\texample: /dev/mtd0.fsbl_a")
BAREBOX_CMD_HELP_TEXT("")
BAREBOX_CMD_HELP_TEXT("The variable will be created as an integer with the boot priority determined")
BAREBOX_CMD_HELP_TEXT("by the ToC Image Header flag of the FSBL partition.")
BAREBOX_CMD_HELP_TEXT("This will enable the bootchooser algorithm to boot the correct path in a")
BAREBOX_CMD_HELP_TEXT("multi-slot environment.")
BAREBOX_CMD_HELP_END

static int do_sb_get_prio(int argc, char *argv[])
{
	if (argc != 3) {
		puts(cmd_sb_get_prio_help);
		return -1;
	}

	return sb_get_prio(argv[1], argv[2]);
}

BAREBOX_CMD_START(sb_get_prio)
	.cmd		= do_sb_get_prio,
	BAREBOX_CMD_DESC("Create the priority variable for a bootchooser target")
	BAREBOX_CMD_OPTS("<bootchooser target priority var> <fsbl slot>")
	BAREBOX_CMD_GROUP(CMD_GRP_MEM)
	BAREBOX_CMD_HELP(cmd_sb_get_prio_help)
BAREBOX_CMD_END
