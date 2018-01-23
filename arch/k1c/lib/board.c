/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <common.h>
#include <malloc.h>
#include <memory.h>
#include <asm-generic/memory_layout.h>


void __noreturn k1c_start_barebox(void)
{
	mem_malloc_init((void *)CONFIG_MALLOC_BASE,
			(void *)(CONFIG_MALLOC_BASE + MALLOC_SIZE - 1));

	start_barebox();

	hang();
}
