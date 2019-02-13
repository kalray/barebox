/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Kalray Inc.
 */

#ifndef _ASM_K1C_BARRIER_H
#define _ASM_K1C_BARRIER_H

/* fence is sufficient to guarantee write ordering */
#define wmb()	__builtin_k1_fence()

/* no L2 coherency, therefore rmb is D$ invalidation */
#define rmb()   __builtin_k1_dinval()

/* general memory barrier */
#define mb()    do { wmb(); rmb(); } while (0)

#endif /* _ASM_K1C_BARRIER_H */
