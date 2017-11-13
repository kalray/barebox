/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2017 Kalray Inc.
 */

#include <common.h>
#include <malloc.h>
#include <memory.h>
#include <asm-generic/memory_layout.h>

int serial_puts(const char *s)
{
	int len = strlen(s);
	register const char *arg1 asm("r0") = s;
	register unsigned arg2 asm ("r1") = len;

	asm volatile ("scall 0xffe\n\t;;"
			: "+r"(arg1), "+r"(arg2)
			: : "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r11", "memory");
	return len;
}

void __noreturn k1c_start_barebox(void)
{
	mem_malloc_init((void *)CONFIG_MALLOC_BASE,
			(void *)(CONFIG_MALLOC_BASE + MALLOC_SIZE - 1));

	start_barebox();
}
