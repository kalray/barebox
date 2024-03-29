/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 by Marc Kleine-Budde <mkl@pengutronix.de>
 */

#ifndef __DMA_H
#define __DMA_H

#include <malloc.h>
#include <xfuncs.h>
#include <linux/kernel.h>

#include <dma-dir.h>
#include <asm/dma.h>
#include <driver.h>

#define DMA_ADDRESS_BROKEN	NULL

#ifndef DMA_ALIGNMENT
#define DMA_ALIGNMENT	32
#endif

#ifndef dma_alloc
static inline void *dma_alloc(size_t size)
{
	return xmemalign(DMA_ALIGNMENT, ALIGN(size, DMA_ALIGNMENT));
}
#endif

#ifndef dma_free
static inline void dma_free(void *mem)
{
	free(mem);
}
#endif

dma_addr_t dma_map_single(struct device *dev, void *ptr, size_t size,
			  enum dma_data_direction dir);
void dma_unmap_single(struct device *dev, dma_addr_t addr, size_t size,
		      enum dma_data_direction dir);

#define DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

#define DMA_MASK_NONE	0x0ULL

static inline void dma_set_mask(struct device *dev, u64 dma_mask)
{
	dev->dma_mask = dma_mask;
}

#define DMA_ERROR_CODE  (~(dma_addr_t)0)

static inline int dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return dma_addr == DMA_ERROR_CODE ||
		(dev->dma_mask && dma_addr > dev->dma_mask);
}

#ifndef __PBL__
/* streaming DMA - implement the below calls to support HAS_DMA */
#ifndef dma_sync_single_for_cpu
void dma_sync_single_for_cpu(dma_addr_t address, size_t size,
			     enum dma_data_direction dir);
#endif

#ifndef dma_sync_single_for_device
void dma_sync_single_for_device(dma_addr_t address, size_t size,
				enum dma_data_direction dir);
#endif
#else
#ifndef dma_sync_single_for_cpu
/*
 * assumes buffers are in coherent/uncached memory, e.g. because
 * MMU is only enabled in barebox_arm_entry which hasn't run yet.
 */
static inline void dma_sync_single_for_cpu(dma_addr_t address, size_t size,
			     enum dma_data_direction dir)
{
	barrier_data((void *)address);
}
#endif

#ifndef dma_sync_single_for_device
static inline void dma_sync_single_for_device(dma_addr_t address, size_t size,
				enum dma_data_direction dir)
{
	barrier_data((void *)address);
}
#endif
#endif

#ifndef dma_alloc_coherent
void *dma_alloc_coherent(size_t size, dma_addr_t *dma_handle);
#endif

#ifndef dma_free_coherent
void dma_free_coherent(void *mem, dma_addr_t dma_handle, size_t size);
#endif

#ifndef dma_alloc_writecombine
void *dma_alloc_writecombine(size_t size, dma_addr_t *dma_handle);
#endif

#endif /* __DMA_H */
