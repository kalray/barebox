/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2017 Kalray Inc
 */

#include <config.h>
#include <linux/linkage.h>
#include <asm/privilege.h>
#include <asm/sys_arch.h>

#define PS_VAL_WFXL(__field, __val) \
	SFR_SET_VAL_WFXL(PS, __field, __val)

#define PS_WFXL_START_VALUE	PS_VAL_WFXL(HLE, 1) | \
				PS_VAL_WFXL(USE, 1) | \
				PS_VAL_WFXL(DCE, 1) | \
				PS_VAL_WFXL(ICE, 1) | \
				PS_VAL_WFXL(V64, 1) | \
				PS_VAL_WFXL(ET, 0)

#define PCR_VAL_WFXM(__field, __val) \
	SFR_SET_VAL_WFXM(PCR, __field, __val)

#define PCR_WFXM_START_VALUE   PCR_VAL_WFXM(L1CE, 1)

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
	call asm_init_pl
	;;
	/* Setup default processor status */
	make $r25 = PS_WFXL_START_VALUE
	;;
	wfxl $ps = $r25
	;;
	make $r25 = PCR_WFXM_START_VALUE
	;;
	wfxm $pcr = $r25
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
	make $sp, CONFIG_SMEM_SIZE
	;;
	call k1c_lowlevel_setup
	;;
	call k1c_start_barebox
	;;
	goto k1c_proc_power_off
	;;
ENDPROC(k1c_start)

/**
 * Initialize privilege level
 */
ENTRY(asm_init_pl)
	get $r21 = $ps
	;;
	/* Extract privilege level from $ps to check if we need to
	 * lower our privilege level (we might already be in PL1)
	 */
	extfz $r20 = $r21, K1C_SFR_END(PS_PL), K1C_SFR_START(PS_PL)
	;;
	/* If our pviliege level is not 0 (ie we are already in a less
	 * privilege level, then, somebody take care of setting the
	 * ring settings for us and we can return directly
	 */
	cb.deqz $r20? delegate_pl
	;;
	ret
	;;
delegate_pl:
	make $r21 = SYO_WFXL_VALUE
	;;
	wfxl $syow = $r21
	;;
	make $r21 = HTO_WFXL_VALUE
	;;
	wfxl $htow = $r21
	;;
	make $r21 = DO_WFXL_VALUE
	;;
	wfxl $dow = $r21
	;;
	make $r21 = MO_WFXL_VALUE
	make $r22 = MO_WFXM_VALUE
	;;
	wfxl $mow = $r21
	;;
	wfxm $mow = $r22
	;;
	make $r21 = ITO_WFXL_VALUE
	make $r22 = ITO_WFXM_VALUE
	;;
	wfxl $itow = $r21
	;;
	wfxm $itow = $r21
	;;
	make $r21 = PSO_WFXL_VALUE
	;;
	wfxl $psow = $r21
	;;
	make $r21 = PSO_WFXM_VALUE
	;;
	wfxm $psow = $r21
	;;
	/* Copy our $ps into $sps for 1:1 restoration */
	get $r22 = $ps
	;;
	/* We will return to $ra after rfe */
	get $r21 = $ra
	/* Set privilege level to +1 in $sps (relative level from the
	 * current one)
	 */
	addd $r22 = $r22, PL_BAREBOX_REL_LEVEL
	;;
	set $spc = $r21
	;;
	set $sps = $r22
	;;
	/* When using rfe, $spc and $sps will be restored in $ps and $pc,
	 * We will then return to the caller ($ra) in current PL + 1
	 */
	rfe
	;;
ENDPROC(asm_init_pl)

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
