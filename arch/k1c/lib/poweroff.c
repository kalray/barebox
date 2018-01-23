/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <init.h>
#include <common.h>
#include <poweroff.h>

static void __noreturn k1c_poweroff(struct poweroff_handler *handler)
{
	register int status asm("r0") = 0;

	shutdown_barebox();

	asm volatile ("scall 0xfff\n\t;;"
			: : "r"(status)
			: "r1", "r2", "r3", "r4", "r5", "r6", "r7",
							"r8", "memory");
	hang();
}

static int k1c_scall_poweroff_probe(struct device_d *dev)
{
	poweroff_handler_register_fn(k1c_poweroff);

	return 0;
}

static __maybe_unused struct of_device_id k1c_scall_poweroff_id[] = {
	{
		.compatible = "kalray,k1c-scall-poweroff",
	}, {
	}
};

static struct driver_d k1c_scall_poweroff = {
	.name  = "k1c_scall_poweroff",
	.probe = k1c_scall_poweroff_probe,
	.of_compatible = DRV_OF_COMPAT(k1c_scall_poweroff_id),
};

device_platform_driver(k1c_scall_poweroff);
