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
#include <asm-generic/barebox.lds.h>

OUTPUT_FORMAT("elf64-k1")
OUTPUT_ARCH("k1")

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

	. = ALIGN(8);
	.rodata ALIGN(8) : {
		*(.rodata);
		*(.rodata.*)
	} > smem

	. = ALIGN(4);
	. = .;
	__barebox_cmd_start = .;
	.barebox_cmd : { BAREBOX_CMDS } > smem
	__barebox_cmd_end = .;

	__barebox_magicvar_start = .;
	.barebox_magicvar : { BAREBOX_MAGICVARS } > smem
	__barebox_magicvar_end = .;

	__barebox_initcalls_start = .;
	.barebox_initcalls : { INITCALLS } > smem
	__barebox_initcalls_end = .;

	__barebox_exitcalls_start = .;
	.barebox_exitcalls : { EXITCALLS } > smem
	__barebox_exitcalls_end = .;

	___usymtab_start = .;
	__usymtab : { BAREBOX_SYMS } > smem
	___usymtab_end = .;

	.dtb :
	{
		. = ALIGN(16);
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

	/* We use store quad for bss init so align on 16 bytes */
	. = ALIGN(16);
	__bss_start = .;
	.bss :
	{
		*(.shbss)
		*(.bss)
		*(COMMON)
	} > smem
	. = ALIGN(16);
	__bss_stop = .;
	__end = .;
}
