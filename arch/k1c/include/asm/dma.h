/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Kalray Inc.
 */

#ifndef __ASM_DMA_H
#define __ASM_DMA_H

#include <common.h>

#define K1C_DDR_32BIT_RAM_WINDOW_BA	(0x80000000ULL)
#define K1C_DDR_64BIT_RAM_WINDOW_BA	(0x100000000ULL)
#define MAX_32BIT_ADDR			(0xffffffffULL)

static inline void *dma_alloc_coherent(size_t size, dma_addr_t *dma_handle)
{
	void *ret = xmemalign(PAGE_SIZE, size);

	if (dma_handle)
		*dma_handle = (dma_addr_t)(uintptr_t)ret;

	return ret;
}

#endif /* __ASM_DMA_H */
