/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Kalray Inc.
 */

#include <stdint.h>
#include <asm/cache.h>

#define K1_DCACHE_REFILL         (12)
#define K1_DCACHE_HWLOOP         (1)
#define K1_DCACHE_REFILL_PERCENT (80)
#define K1_DCACHE_LINE_SIZE    (64)
#define K1_DCACHE_SIZE (128*1024)

void k1_dcache_invalidate_mem_area(uintptr_t addr, int size)
{
	/* if hwloop iterations cost < _K1_DCACHE_REFILL_PERCENT cache refill,
	 * use hwloop, otherwise invalid the whole cache
	 */
	if (size <
	(K1_DCACHE_REFILL_PERCENT * (K1_DCACHE_REFILL * K1_DCACHE_SIZE))
			/ (100 * (K1_DCACHE_REFILL + K1_DCACHE_HWLOOP))) {
		/* number of lines that must be invalidated */
		int invalid_lines = ((addr + size) -
					(addr & (~(K1_DCACHE_LINE_SIZE - 1))));

		invalid_lines = invalid_lines / K1_DCACHE_LINE_SIZE
				+ (0 != (invalid_lines % K1_DCACHE_LINE_SIZE));
		if (__builtin_constant_p(invalid_lines) && invalid_lines <= 2) {
			/* when inlining (and doing constant folding),
			 *  gcc is able to unroll small loops
			 */
			int i;

			for (i = 0; i < invalid_lines; i++) {
				__builtin_k1_dinvall((void *)(addr
						+ i * K1_DCACHE_LINE_SIZE));
			}
		} else if (invalid_lines > 0) {
			__asm__ __volatile__ (
				"loopdo %1, 0f\n;;\n"
				"dinvall 0[%0]\n"
				"addd %0 = %0, %2\n;;\n"
				"0:\n"
				: "+r"(addr)
				: "r" (invalid_lines),
				"i" (K1_DCACHE_LINE_SIZE)
				: "ls", "le", "lc", "memory");
		}
	} else {
		__builtin_k1_dinval();
	}
}
