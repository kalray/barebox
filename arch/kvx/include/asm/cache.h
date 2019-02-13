/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2019 Kalray Inc.
 */

#ifndef __KVX_CACHE_H
#define __KVX_CACHE_H

#include <linux/types.h>

void kvx_dcache_invalidate_mem_area(uint64_t addr, int size);

static inline void sync_dcache_icache(void)
{
	__builtin_kvx_fence();
	__builtin_kvx_iinval();
	__builtin_kvx_barrier();
}

static inline void dcache_inval(void)
{
	__builtin_kvx_fence();
	__builtin_kvx_dinval();
}

#endif /* __KVX_CACHE_H */
