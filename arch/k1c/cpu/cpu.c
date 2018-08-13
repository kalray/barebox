/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <common.h>
#include <asm/sfr.h>

void k1c_lowlevel_setup(void)
{
	uint64_t ev_val = (uint64_t) &_exception_start | EXCEPTION_STRIDE;

	/* Install exception handlers */
	k1c_sfr_set(K1C_SFR_EV, ev_val);

	/* Clear exception taken bit now that we setup our handlers */
	k1c_sfr_clear_bit(K1C_SFR_PS, K1C_SFR_PS_ET_SHIFT);

	/* Finally, make sure nobody disabled hardware trap before us */
	k1c_sfr_clear_bit(K1C_SFR_PS, K1C_SFR_PS_HTD_SHIFT);
}
