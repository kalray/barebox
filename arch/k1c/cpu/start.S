/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2017 Kalray Inc
 */

#include <config.h>
#include <linux/linkage.h>
#include <asm/sfr_defs.h>

/* Enable 64bits for both user and kernel */
#define PS_ENABLE_64BIT		(K1C_SFR_PS_P64_MASK | K1C_SFR_PS_U64_MASK)
/* Enable both ICache and DCache */
#define PS_ENABLE_CACHES	(K1C_SFR_PS_DCE_MASK | K1C_SFR_PS_ICE_MASK)
/* Enable also hardware loops and uncached streaming */
#define PS_DEFAULT_VALUE	(K1C_SFR_PS_HLE_MASK | K1C_SFR_PS_USE_MASK | \
				 PS_ENABLE_CACHES | PS_ENABLE_64BIT)
/* Shift the whole value to set the bits to set using wfxl */
#define PS_WFXL_VALUE		(PS_DEFAULT_VALUE << 32)

/* Enable STOP in WS */
#define WS_ENABLE_WU2		(K1C_SFR_WS_WU2_MASK)

#define WS_WFXL_VALUE		(WS_ENABLE_WU2)

.section .startup, "ax"
ENTRY(k1c_start)
	/* (Re)initialize performance counter */
	make $r20 = 0x00000000
	;;
	set $pmc = $r20
	;;
	/* Setup default processor status */
	make $r25 = PS_WFXL_VALUE
	;;
	wfxl $ps = $r25
	;;
	/* Clear BSS */
	make $r22 = __bss_stop
	make $r21 = __bss_start
	;;
	sbfd $r22 = $r21, $r22
	make $r24 = 0
	;;
	/* Divide by 16 for hardware loop */
	srld $r22, $r22, 4
	make $r25 = 0
	;;
	/* Clear bss with hardware loop */
        loopdo $r22, clear_bss_done 
		;;
		sq 0[$r21] = $r24r25
		addd $r21 = $r21, 16
		;;
	clear_bss_done:
	/* Setup stack */
	make $r12, (CONFIG_SMEM_SIZE - 16)
	;;
	/* Here we go ! start the C stuff */
	make $r20 = k1c_start_barebox
	;;
	icall $r20
	;;
	goto k1c_proc_power_off
	;;
ENDPROC(k1c_start)

ENTRY(k1c_proc_power_off):
	dinval
	make $r1 = WS_WFXL_VALUE
	;;
        /* Enable STOP */
	wfxl $ws, $r1
	;;
1:	stop
	;;
	goto 1b
	;; 
ENDPROC(k1c_proc_power_off)
