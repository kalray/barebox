/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Kalray Inc.
 */

#include <dma.h>
#include <asm/barrier.h>
#include <asm/io.h>
#include <asm/cache.h>

void dma_sync_single_for_device(dma_addr_t address, size_t size,
					     enum dma_data_direction dir)
{
	/* allow device to read buffer written by CPU */
	wmb();
}

void dma_sync_single_for_cpu(dma_addr_t address, size_t size,
			     enum dma_data_direction dir)
{
	k1_dcache_invalidate_mem_area(address, size);
}

dma_addr_t dma_map_single(struct device_d *dev, void *ptr, size_t size,
			  enum dma_data_direction dir)
{
	dma_addr_t newptr;

	if (dev->dma_mask && (uintptr_t)ptr <= dev->dma_mask)
		return (dma_addr_t)ptr;
	if ((uintptr_t)ptr > K1C_DDR_64BIT_RAM_WINDOW_BA
			+ (MAX_32BIT_ADDR - K1C_DDR_32BIT_RAM_WINDOW_BA))
		return DMA_ERROR_CODE;
	newptr = (dma_addr_t)((uintptr_t)ptr - K1C_DDR_64BIT_RAM_WINDOW_BA
			+ K1C_DDR_32BIT_RAM_WINDOW_BA);
	if (dev->dma_mask && newptr > dev->dma_mask)
		return DMA_ERROR_CODE;
	return newptr;
}

void dma_unmap_single(struct device_d *dev, dma_addr_t addr, size_t size,
		      enum dma_data_direction dir)
{

}
