/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2017 Kalray Inc.
 */

#ifndef _ASM_K1C_COMMON_H
#define _ASM_K1C_COMMON_H

#define EXCEPTION_STRIDE	0x40
#define EXCEPTION_ALIGNMENT	0x1000

#ifndef __ASSEMBLY__

extern char _exception_start;

#endif

#endif	/* _ASM_K1C_COMMON_H */

