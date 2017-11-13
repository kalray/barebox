/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2017 Kalray Inc.
 */

#ifndef _ASM_K1C_ELF_H
#define _ASM_K1C_ELF_H

/*
 * ELF register definitions..
 */
#include <linux/types.h>

#define EM_KALRAY 0x1337

#define ELF_ARCH 	EM_KALRAY
#define ELF_CLASS	ELFCLASS32
#define ELF_DATA	ELFDATA2MSB

#endif	/* _ASM_K1C_ELF_H */

