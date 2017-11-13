/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2017 Kalray Inc.
 */

#ifndef _ASM_K1C_BITOPS_H
#define _ASM_K1C_BITOPS_H

#include <asm-generic/bitops/__ffs.h>
#include <asm-generic/bitops/__fls.h>
#include <asm-generic/bitops/ffs.h>
#include <asm-generic/bitops/fls.h>
#include <asm-generic/bitops/ffz.h>
#include <asm-generic/bitops/hweight.h>
#include <asm-generic/bitops/fls64.h>
#include <asm-generic/bitops/find.h>
#include <asm-generic/bitops/ops.h>

#define set_bit(x, y)			__set_bit(x, y)
#define clear_bit(x, y)			__clear_bit(x, y)
#define change_bit(x, y)		__change_bit(x, y)
#define test_and_set_bit(x, y)		__test_and_set_bit(x, y)
#define test_and_clear_bit(x, y)	__test_and_clear_bit(x, y)
#define test_and_change_bit(x, y)	__test_and_change_bit(x, y)

#endif	/* _ASM_K1C_BITOPS_H */

