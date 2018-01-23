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

	.rodata ALIGN(8) : {
		*(.rodata*)
		. = ALIGN(8);
		RO_DATA_SECTION
	} > smem

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
