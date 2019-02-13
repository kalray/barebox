/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Kalray Inc.
 */

#ifndef __K1C_CACHE_H
#define __K1C_CACHE_H

void k1_dcache_invalidate_mem_area(uintptr_t addr, int size);

#endif /* __K1C_CACHE_H */
