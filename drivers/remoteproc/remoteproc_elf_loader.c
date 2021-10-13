// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote Processor Framework Elf loader
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 * Mark Grosen <mgrosen@ti.com>
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Suman Anna <s-anna@ti.com>
 * Robert Tivy <rtivy@ti.com>
 * Armando Uribe De Leon <x0095078@ti.com>
 * Sjur Brændeland <sjur.brandeland@stericsson.com>
 */

#include <common.h>
#include <elf.h>
#include <linux/remoteproc.h>

#include "remoteproc_internal.h"

static int load_elf_segment(struct rproc *rproc, struct elf_image *elf,
			    void *phdr)
{
	struct device *dev = &rproc->dev;
	phys_addr_t da = (phys_addr_t) elf_phdr_p_paddr(elf, phdr);
	u64 memsz = elf_phdr_p_memsz(elf, phdr);
	u64 filesz = elf_phdr_p_filesz(elf, phdr);
	u64 offset = elf_phdr_p_offset(elf, phdr);
	void *ptr;

	if (elf_phdr_p_type(elf, phdr) != PT_LOAD)
		return 0;

	dev_dbg(dev, "phdr: type %d da 0x%llx memsz 0x%llx filesz 0x%llx\n",
		elf_phdr_p_type(elf, phdr), da, memsz, filesz);

	if (filesz > memsz) {
		dev_err(dev, "bad phdr filesz 0x%llx memsz 0x%llx\n",
			filesz, memsz);
		return -EINVAL;
	}

	if (offset + filesz > elf->size) {
		dev_err(dev, "truncated fw: need 0x%llx avail 0x%zx\n",
			offset + filesz, elf->size);
		return -EINVAL;
	}

	/* grab the kernel address for this device address */
	ptr = rproc_da_to_va(rproc, da, memsz);
	if (!ptr) {
		dev_err(dev, "bad phdr da 0x%llx mem 0x%llx\n", da, memsz);
		return -EINVAL;
	}

	/* put the segment where the remote processor expects it */
	if (filesz)
		memcpy(ptr, elf->hdr_buf + offset, filesz);

	/*
	 * Zero out remaining memory for this segment.
	 *
	 * This isn't strictly required since dma_alloc_coherent already
	 * did this for us. albeit harmless, we may consider removing
	 * this.
	 */
	if (memsz > filesz)
		memset(ptr + filesz, 0, memsz - filesz);

	return 0;
}

int rproc_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	void *buf, *phdr;
	int i, ret = 0;
	struct elf_image *elf;

	elf = elf_open_binary((void *) fw->data, fw->size);
	if (!elf)
		return -EINVAL;

	buf = elf->hdr_buf;
	phdr = (void *) (buf + elf_hdr_e_phoff(elf, buf));
	rproc->bootaddr = elf_hdr_e_entry(elf, buf);

	/* go through the available ELF segments */
	for (i = 0; i < elf_hdr_e_phnum(elf, buf); i++) {
		ret = load_elf_segment(rproc, elf, phdr);
		if (ret)
			break;

		phdr += elf_size_of_phdr(elf);
	}

	elf_close(elf);

	return ret;
}
EXPORT_SYMBOL(rproc_elf_load_segments);
