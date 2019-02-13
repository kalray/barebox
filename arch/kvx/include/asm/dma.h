/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2019 Kalray Inc.
 */

#ifndef __ASM_DMA_H
#define __ASM_DMA_H

#include <common.h>

#define KVX_DDR_32BIT_RAM_WINDOW_BA	(0x80000000ULL)
#define KVX_DDR_64BIT_RAM_WINDOW_BA	(0x100000000ULL)
#define MAX_32BIT_ADDR			(0xffffffffULL)

#define dma_alloc dma_alloc
static inline void *dma_alloc(size_t size)
{
	return xmemalign(64, ALIGN(size, 64));
}

static inline void *dma_alloc_coherent(size_t size, dma_addr_t *dma_handle)
{
	void *ret = xmemalign(PAGE_SIZE, size);

	if (dma_handle)
		*dma_handle = (dma_addr_t)(uintptr_t)ret;

	return ret;
}

static inline void dma_free_coherent(void *mem, dma_addr_t dma_handle,
				     size_t size)
{
	free(mem);
}

#endif /* __ASM_DMA_H */
