/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <common.h>
#include <clock.h>
#include <init.h>

#include <linux/clk.h>
#include <linux/err.h>

#include <asm/sfr.h>

static uint64_t k1c_pm_read(void)
{
	return k1c_sfr_get(K1C_SFR_PM0);
}

static int k1c_timer_probe(struct device_d *dev)
{
	struct clk *clk;
	struct clocksource *clksrc;
	uint32_t clk_freq;
	struct device_node *np = dev->device_node;

	clksrc = xzalloc(sizeof(struct clocksource));

	/* Get clock frquency */
	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("Failed to get CPU clock: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	clk_freq = clk_get_rate(clk);
	clk_put(clk);

	/* Init clocksource */
	clksrc->read	= k1c_pm_read,
	clksrc->mask	= 0xffffffff,
	clksrc->shift	= 0,

	clksrc->mult = clocksource_hz2mult(clk_freq, clksrc->shift);

	return init_clock(clksrc);
}

static struct of_device_id k1c_timer_dt_ids[] = {
	{ .compatible = "kalray,k1c-core-timer", },
	{ }
};

static struct driver_d k1c_timer_driver = {
	.name = "k1c-timer",
	.probe = k1c_timer_probe,
	.of_compatible = DRV_OF_COMPAT(k1c_timer_dt_ids),
};

device_platform_driver(k1c_timer_driver);
