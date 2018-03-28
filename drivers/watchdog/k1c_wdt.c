/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <common.h>
#include <init.h>
#include <io.h>
#include <of.h>
#include <restart.h>
#include <watchdog.h>

#include <linux/clk.h>
#include <linux/err.h>

#include <asm/sfr.h>

#define to_dw_wdt(wdd)	container_of(wdd, struct dw_wdt, wdd)

struct k1c_wdt {
	uint64_t clk_rate;
	struct watchdog wdd;
	struct restart_handler restart;
	struct reset_control *rst;
};

static void k1c_watchdog_disable(void)
{
	k1c_sfr_clear_bit(K1C_SFR_TC, K1C_SFR_TC_WUI_SHIFT);
	k1c_sfr_clear_bit(K1C_SFR_TC, K1C_SFR_TC_WCE_SHIFT);
}

static int k1c_wdt_set_timeout(struct watchdog *wdd, unsigned int timeout)
{
	struct k1c_wdt *wdt = container_of(wdd, struct k1c_wdt, wdd);
	uint64_t cycle_timeout = wdt->clk_rate * timeout;

	/* Disable watchdog */
	if (timeout == 0) {
		k1c_watchdog_disable();
		return 0;
	}

	k1c_sfr_set(K1C_SFR_WDC, cycle_timeout);
	k1c_sfr_set(K1C_SFR_WDR, 0);

	/* Start watchdog counting */
	k1c_sfr_set_bit(K1C_SFR_TC, K1C_SFR_TC_WUI_SHIFT);
	k1c_sfr_set_bit(K1C_SFR_TC, K1C_SFR_TC_WCE_SHIFT);

	return 0;
}

static void __noreturn k1c_wdt_restart_handle(struct restart_handler *rst)
{
	struct k1c_wdt *k1c_wdt;

	k1c_wdt = container_of(rst, struct k1c_wdt, restart);

	k1c_wdt->wdd.set_timeout(&k1c_wdt->wdd, 1);

	mdelay(1000);

	hang();
}

static int count;

static int k1c_wdt_drv_probe(struct device_d *dev)
{
	struct watchdog *wdd;
	struct clk *clk;
	struct k1c_wdt *k1c_wdt;
	int ret;

	if (count != 0) {
		dev_warn(dev, "Tried to register core watchdog twice\n");
		return -EINVAL;
	}
	count++;

	k1c_wdt = xzalloc(sizeof(*k1c_wdt));
	clk = clk_get(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	k1c_wdt->clk_rate = clk_get_rate(clk);
	clk_put(clk);

	wdd = &k1c_wdt->wdd;
	wdd->name = "k1c_wdt";
	wdd->hwdev = dev;
	wdd->set_timeout = k1c_wdt_set_timeout;

	/* Be sure that interrupt are disable */
	k1c_sfr_clear_bit(K1C_SFR_TC, K1C_SFR_TC_WIE_SHIFT);

	k1c_watchdog_disable();

	ret = watchdog_register(wdd);
	if (ret)
		return -EINVAL;

	k1c_wdt->restart.name = "k1c_wdt";
	k1c_wdt->restart.priority = 50;
	k1c_wdt->restart.restart = k1c_wdt_restart_handle;

	ret = restart_handler_register(&k1c_wdt->restart);
	if (ret)
		dev_warn(dev, "cannot register restart handler\n");

	return 0;

}

static struct of_device_id k1c_wdt_of_match[] = {
	{ .compatible = "kalray,k1c-core-watchdog", },
	{ /* sentinel */ }
};

static struct driver_d k1c_wdt_driver = {
	.name		= "k1c-wdt",
	.probe		= k1c_wdt_drv_probe,
	.of_compatible	= DRV_OF_COMPAT(k1c_wdt_of_match),
};
device_platform_driver(k1c_wdt_driver);
