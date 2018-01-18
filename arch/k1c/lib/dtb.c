/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <common.h>
#include <init.h>
#include <of.h>

extern char __dtb_start[];

static int of_k1c_init(void)
{
	int ret;
	struct device_node *root;

	root = of_unflatten_dtb(__dtb_start);
	if (IS_ERR(root)) {
		ret = PTR_ERR(root);
                panic("Failed to parse DTB: %d\n", ret);
	}

	ret = of_set_root_node(root);
	if (ret)
		panic("Failed to set of root node\n");

	of_probe();

	return 0;
}
core_initcall(of_k1c_init);
