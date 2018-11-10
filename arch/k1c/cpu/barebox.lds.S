/*
 * barebox - barebox.lds.S
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

#include <config.h>
#include <asm/common.h>
#include <asm-generic/barebox.lds.h>

OUTPUT_FORMAT("elf64-k1")
OUTPUT_ARCH("k1:k1c:64")

#define K1C_ENTRY_ADDRESS	0x0
#define DTB_DEFAULT_SIZE	(24 * 1024)

MEMORY
{
	smem	: ORIGIN = 0, LENGTH = CONFIG_SMEM_SIZE
}

SECTIONS
{
	. = K1C_ENTRY_ADDRESS;
	. = ALIGN(4);

	__start = .;
	.text : AT (__start) {
		*(.startup);
		_stext = .;
		*(.text)
		_etext = .;
	}  > smem

	/* Exception vector must be aligned on a huge frontier */
	.exception ALIGN(EXCEPTION_ALIGNMENT) :
	{
		_exception_start = ABSOLUTE(.);
		/**
		 * First handler is at _exception_start + EXCEPTION_STRIDE
		 * In order to force getting to the next stride, add at
		 * least 1 byte of data. The next ALIGN will then be
		 * forced to get to the next stride.
		 */
		. += 1;
		. = ALIGN(EXCEPTION_STRIDE);

		/* Entry for traps */
		KEEP(*(.exception.trap));
		. += 1;

		/* Entry for interrupts */
		. = ALIGN(EXCEPTION_STRIDE);
		KEEP(*(.exception.interrupt));
		. += 1;

		/* Entry for syscall */
		. = ALIGN(EXCEPTION_STRIDE);
		KEEP(*(.exception.syscall));
	} > smem

	.rodata ALIGN(8) : {
		*(.rodata);
		*(.rodata.*)
	} > smem

	. = ALIGN(4);
	. = .;

	.barebox_cmd ALIGN(8) : {
		__barebox_cmd_start = .;
		BAREBOX_CMDS
		__barebox_cmd_end = .;
	 } > smem

	.barebox_ratp_cmd ALIGN(8) : {
		__barebox_ratp_cmd_start = .;
		BAREBOX_RATP_CMDS
		__barebox_ratp_cmd_end = .;
	 } > smem

	.barebox_magicvar ALIGN(8) : {
		__barebox_magicvar_start = .;
		BAREBOX_MAGICVARS
		__barebox_magicvar_end = .;
	} > smem

	.barebox_initcalls ALIGN(8) : {
		__barebox_initcalls_start = .;
		 INITCALLS
		__barebox_initcalls_end = .;
	} > smem

	.barebox_exitcalls ALIGN(8) : {
		__barebox_exitcalls_start = .;
		EXITCALLS
		__barebox_exitcalls_end = .;
	} > smem

	__usymtab ALIGN(8) : {
		___usymtab_start = .;
		BAREBOX_SYMS
		___usymtab_end = .;
	} > smem

	.oftables : { BAREBOX_CLK_TABLE() } > smem

	.dtb ALIGN(16):
	{
		__dtb_start = .;
		. += DTB_DEFAULT_SIZE;
		__dtb_end = .;
	} > smem

	__etext = .; /* End of text and rodata section */


	. = ALIGN(4);
	.data : {
		sdata = .;
		_sdata = .;
		*(.data)
		edata = .;
		_edata = .;
	} > smem

	.gdb_page ALIGN(4 * 1024) :
	{
		_debug_start = ABSOLUTE(.);
		_debug_phy_start = ABSOLUTE(.);
		. += 4 * 1024;
	}
	__debug_phy_end = ABSOLUTE(.);

	/* We use store quad for bss init so align on 16 bytes */
	.bss ALIGN(16):
	{
		__bss_start = .;
		*(.shbss)
		*(.bss)
		*(COMMON)
	} > smem
	. = ALIGN(16);
	__bss_stop = .;
	__end = .;
}
