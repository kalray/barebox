#include <linux/linkage.h>

/**
 * We do not aim at handling traps but at least, we want the CPU to
 * stop when taking a trap.
 * Note that we can not expect to be able to issue a print since
 * it might be the cause of trap...
 */
.section .exception.trap, "ax", @progbits
ENTRY(k1c_trap_handler):
	goto k1c_trap_handler
	;;
ENDPROC(k1c_trap_handler)
