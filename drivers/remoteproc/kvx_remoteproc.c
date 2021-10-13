// SPDX-License-Identifier: GPL-2.0
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Kalray Inc.
 */

#include <init.h>
#include <common.h>
#include <regmap.h>
#include <memory.h>
#include <of_address.h>
#include <mfd/syscon.h>
#include <mfd/kvx-ftu.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/remoteproc.h>
#include <linux/compiler_types.h>

#include "remoteproc_internal.h"

#define KVX_EXTERNAL_MAX_MEM_COUNT	10

/**
 * Internal Memory types
 * @KVX_INTERNAL_MEM_TCM: Tightly Coupled Memory
 * @KVX_INTERNAL_MEM_DSU: Debug System Unit Memory
 * @KVX_INTERNAL_MEM_COUNT: just keep this one at the end
 */
enum {
	KVX_INTERNAL_MEM_TCM,
	KVX_INTERNAL_MEM_DSU,
	KVX_INTERNAL_MEM_COUNT,
};

static const char *mem_names[KVX_INTERNAL_MEM_COUNT] = {
	[KVX_INTERNAL_MEM_TCM]	= "tcm",
	[KVX_INTERNAL_MEM_DSU]	= "dsu",
};

/*
 * All clusters local memory maps are exposed starting from 16M
 * Then, local cluster memories are at address 16M + cluster_id * 16M
 */
#define KVX_RPROC_CLUSTER_LOCAL_ADDR_MASK	(SZ_16M - 1)

/**
 * struct kvx_rproc_mem - internal memory structure
 * @size: Size of the memory region
 * @dev_addr: Remote CPU address used to access the memory region
 * @cpu_addr: CPU virtual address of the memory region
 * @bus_addr: Bus address used to access the memory region
 */
struct kvx_rproc_mem {
	size_t size;
	u64 dev_addr;
	phys_addr_t bus_addr;
	void __iomem *cpu_addr;
};

/**
 * struct kvx_rproc - kvx remote processor driver structure
 * @cluster_id: Cluster ID of the rproc
 * @dev: cached device pointer
 * @rproc: remoteproc device handle
 * @ftu_regmap: regmap struct to the ftu controller
 * @mem: List of memories
 */
struct kvx_rproc {
	int cluster_id;
	struct device_d *dev;
	struct rproc *rproc;
	struct regmap *ftu_regmap;
	unsigned int mem_count;
	struct kvx_rproc_mem mem[KVX_EXTERNAL_MAX_MEM_COUNT];
};

static int kvx_rproc_start(struct rproc *rproc)
{
	int ret;
	u32 boot_addr = rproc->bootaddr;
	struct kvx_rproc *kvx_rproc = rproc->priv;
	unsigned int boot_offset = KVX_FTU_BOOTADDR_OFFSET +
				kvx_rproc->cluster_id * KVX_FTU_CLUSTER_STRIDE;
	unsigned int ctrl_offset = KVX_FTU_CLUSTER_CTRL +
				kvx_rproc->cluster_id * KVX_FTU_CLUSTER_STRIDE;
	/* Reset sequence */
	struct reg_sequence start_cluster[] = {
		/* Set boot address */
		REG_SEQ0(boot_offset, boot_addr),
		/* Wakeup rm */
		REG_SEQ(ctrl_offset, BIT(KVX_FTU_CLUSTER_CTRL_CLKEN_BIT) |
			      BIT(KVX_FTU_CLUSTER_CTRL_WUP_BIT), 1),
		/* Clear wup */
		REG_SEQ(ctrl_offset, BIT(KVX_FTU_CLUSTER_CTRL_CLKEN_BIT), 1),
	};

	if (!IS_ALIGNED(boot_addr, SZ_4K)) {
		dev_err(kvx_rproc->dev, "invalid boot address 0x%x, must be aligned on a 4KB boundary\n",
			boot_addr);
		return -EINVAL;
	}

	/* Apply start sequence */
	ret = regmap_multi_reg_write(kvx_rproc->ftu_regmap, start_cluster,
				     ARRAY_SIZE(start_cluster));
	if (ret) {
		dev_err(kvx_rproc->dev, "regmap_write of ctrl failed, status = %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int kvx_rproc_reset(struct kvx_rproc *kvx_rproc)
{
	int ret;
	unsigned int ctrl_offset = KVX_FTU_CLUSTER_CTRL +
				kvx_rproc->cluster_id * KVX_FTU_CLUSTER_STRIDE;
	struct reg_sequence reset_cluster[] = {
		/* Enable clock and reset */
		REG_SEQ(ctrl_offset, BIT(KVX_FTU_CLUSTER_CTRL_CLKEN_BIT) |
			      BIT(KVX_FTU_CLUSTER_CTRL_RST_BIT), 2),
		/* Release reset */
		REG_SEQ(ctrl_offset, BIT(KVX_FTU_CLUSTER_CTRL_CLKEN_BIT), 1),
	};

	ret = regmap_multi_reg_write(kvx_rproc->ftu_regmap, reset_cluster,
				     ARRAY_SIZE(reset_cluster));
	if (ret) {
		dev_err(kvx_rproc->dev, "regmap_write of ctrl failed, status = %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int kvx_rproc_stop(struct rproc *rproc)
{
	struct kvx_rproc *kvx_rproc = rproc->priv;

	return kvx_rproc_reset(kvx_rproc);
}

static void *kvx_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	int i;
	size_t size;
	u64 dev_addr, offset;
	phys_addr_t bus_addr;
	void __iomem *va = NULL;
	struct kvx_rproc *kvx_rproc = rproc->priv;

	if (len <= 0)
		return NULL;

	for (i = 0; i < kvx_rproc->mem_count; i++) {
		bus_addr = kvx_rproc->mem[i].bus_addr;
		dev_addr = kvx_rproc->mem[i].dev_addr;
		size = kvx_rproc->mem[i].size;

		if (da < KVX_RPROC_CLUSTER_LOCAL_ADDR_MASK) {
			/* handle Cluster-view addresses */
			if ((da >= dev_addr) &&
			    ((da + len) <= (dev_addr + size))) {
				offset = da - dev_addr;
				va = kvx_rproc->mem[i].cpu_addr + offset;
				break;
			}
		} else {
			/* handle SoC-view addresses */
			if ((da >= bus_addr) &&
			    (da + len) <= (bus_addr + size)) {
				offset = da - bus_addr;
				va = kvx_rproc->mem[i].cpu_addr + offset;
				break;
			}
		}
	}

	dev_dbg(&rproc->dev, "da = 0x%llx len = 0x%x va = 0x%p\n",
		da, len, va);

	return (__force void *)va;
}

static const struct rproc_ops kvx_rproc_ops = {
	.start		= kvx_rproc_start,
	.stop		= kvx_rproc_stop,
	.da_to_va	= kvx_rproc_da_to_va,
};


static int kvx_rproc_get_internal_memories(struct device_d *dev,
					   struct kvx_rproc *kvx_rproc)
{
	int i, a, err, nph;
	struct resource *res;
	struct kvx_rproc_mem *mem;
	struct device_node *np = dev->device_node;

	for (i = 0; i < KVX_INTERNAL_MEM_COUNT; i++) {
		mem = &kvx_rproc->mem[i];

		res = dev_request_mem_resource(dev, i);
		if (IS_ERR(res))
			return PTR_ERR(res);

		mem->cpu_addr = IOMEM(res->start);

		mem->bus_addr = res->start;
		mem->dev_addr = res->start & KVX_RPROC_CLUSTER_LOCAL_ADDR_MASK;
		mem->size = resource_size(res);

		dev_dbg(dev, "Adding internal memory %s, ba = 0x%llx, da = 0x%llx, va = 0x%pK, len = 0x%zx\n",
			mem_names[i], mem->bus_addr,
			mem->dev_addr, mem->cpu_addr, mem->size);
		kvx_rproc->mem_count++;
	}

	/* memory-region is optional property */
	nph = of_count_phandle_with_args(np, "memory-region", NULL);
	if (nph <= 0)
		return 0;

	/* remap optional addresses */
	for (a = 0; a < nph; a++) {
		struct device_node *node;
		struct resource iores, *res_cpu;

		if (kvx_rproc->mem_count >= KVX_EXTERNAL_MAX_MEM_COUNT)
			break;

		mem = &kvx_rproc->mem[kvx_rproc->mem_count];

		node = of_parse_phandle(np, "memory-region", a);
		err = of_address_to_resource(node, 0, &iores);
		if (err) {
			dev_err(dev, "unable to resolve memory region\n");
			return err;
		}

		res_cpu = request_sdram_region(dev_name(dev), iores.start,
					       resource_size(&iores));
		if (!res_cpu) {
			dev_err(dev, "remap optional addresses failed\n");
			return -ENOMEM;
		}

		mem->cpu_addr = IOMEM(res_cpu->start);
		mem->bus_addr = iores.start;
		mem->dev_addr = iores.start;
		mem->size = resource_size(&iores);

		dev_dbg(dev, "Adding external memory %s, ba = 0x%llx, da = 0x%llx, va = 0x%pK, len = 0x%zx\n",
			node->name, mem->bus_addr,
			mem->dev_addr, mem->cpu_addr, mem->size);

		kvx_rproc->mem_count++;
	}

	return 0;
}

static int kvx_rproc_of_get_dev_syscon(struct device_d *dev,
				       struct kvx_rproc *kvx_rproc)
{
	int ret;
	struct device_node *np = dev->device_node;

	if (!of_property_read_bool(np, KVX_FTU_NAME)) {
		dev_err(dev, "kalray,ftu-dev property is absent\n");
		return -EINVAL;
	}

	kvx_rproc->ftu_regmap =
		syscon_regmap_lookup_by_phandle(np, KVX_FTU_NAME);
	if (IS_ERR(kvx_rproc->ftu_regmap)) {
		ret = PTR_ERR(kvx_rproc->ftu_regmap);
		return ret;
	}

	if (of_property_read_u32_index(np, KVX_FTU_NAME, 1,
				       &kvx_rproc->cluster_id)) {
		dev_err(dev, "couldn't read the cluster id\n");
		return -EINVAL;
	}

	if (kvx_rproc->cluster_id < 1 || kvx_rproc->cluster_id > 4) {
		dev_err(dev, "Invalid cluster id (must be between in [1..4]\n");
		return -EINVAL;
	}

	return 0;
}


static int kvx_rproc_probe(struct device_d *dev)
{
	struct rproc *rproc;
	int ret = 0;
	struct kvx_rproc *kvx_rproc;
	struct device_node *np = dev->device_node;

	rproc = rproc_alloc(dev, np->name, &kvx_rproc_ops, sizeof(*kvx_rproc));
	if (!rproc)
		return -ENOMEM;

	kvx_rproc = rproc->priv;
	kvx_rproc->rproc = rproc;
	kvx_rproc->dev = dev;

	ret = kvx_rproc_get_internal_memories(dev, kvx_rproc);
	if (ret)
		goto free_rproc;

	ret = kvx_rproc_of_get_dev_syscon(dev, kvx_rproc);
	if (ret)
		goto free_rproc;

	kvx_rproc_reset(kvx_rproc);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "failed to add device with remoteproc core, status = %d\n",
			ret);
		goto free_rproc;
	}

	return 0;

free_rproc:
	rproc_free(rproc);

	return ret;
}

static const struct of_device_id kvx_rproc_of_match[] = {
	{ .compatible = "kalray,kvx-cluster-rproc", },
	{ /* sentinel */ },
};

static struct driver_d kvx_rproc_driver = {
	.name = "kvx-rproc",
	.probe	= kvx_rproc_probe,
	.of_compatible = kvx_rproc_of_match,
};
device_platform_driver(kvx_rproc_driver);
