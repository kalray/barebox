/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 Kalray Inc.
 */

#include <clock.h>
#include <common.h>
#include <init.h>
#include <io.h>
#include <dma.h>
#include <malloc.h>
#include <mci.h>
#include <linux/err.h>
#include <linux/clk.h>

#include "sdhci.h"

#define CARD_STATUS_MASK (0x1e00)
#define CARD_STATUS_TRAN (4 << 9)
static int dw_mshc_mci_send_cmd(struct mci_host *mci, struct mci_cmd *cmd,
				struct mci_data *data);

struct dw_mshc_mshci {
	struct mci_host mci;
	void __iomem *base;
	unsigned int version;
	u32 cap1;
	u32 cap2;
	struct clk *clk;
	unsigned int in_abort_sequence;
};

#define priv_from_mci_host(h)	\
	container_of(h, struct dw_mshc_mshci, mci)

static inline void dw_mshc_writel(struct dw_mshc_mshci *p, int reg, u32 val)
{
	writel(val, p->base + reg);
}

static inline void dw_mshc_writew(struct dw_mshc_mshci *p, int reg, u16 val)
{
	writew(val, p->base + reg);
}

static inline void dw_mshc_writeb(struct dw_mshc_mshci *p, int reg, u8 val)
{
	writeb(val, p->base + reg);
}

static inline u32 dw_mshc_readl(struct dw_mshc_mshci *p, int reg)
{
	return readl(p->base + reg);
}

static inline u16 dw_mshc_readw(struct dw_mshc_mshci *p, int reg)
{
	return readw(p->base + reg);
}

static inline u8 dw_mshc_readb(struct dw_mshc_mshci *p, int reg)
{
	return readb(p->base + reg);
}

static inline int dw_mshc_sdma_supported(struct dw_mshc_mshci *host)
{
	return host->cap1 & SDHCI_CAN_DO_SDMA;
}

static int dw_mshc_wait_for_done(struct dw_mshc_mshci *host, u16 mask)
{
	u16 status;
	u64 start;
	u64 addr;

	start = get_time_ns();
	while (1) {
		status = dw_mshc_readw(host, SDHCI_INT_NORMAL_STATUS);
		if (status & SDHCI_INT_ERROR) {
			dev_err(host->mci.hw_dev,
				"SDHCI_INT_ERROR, normal int status: %04x\n",
				status);
			return -EPERM;
		}
		/* this special quirk is necessary, as the dma
		 * engine stops on dma boundary and will only
		 * restart after acknowledging it this way.
		 */
		if (status & SDHCI_INT_DMA) {
			dw_mshc_writew(host, SDHCI_INT_NORMAL_STATUS,
				       SDHCI_INT_DMA);
			addr = dw_mshc_readl(host, SDHCI_ADMA_SA_LOW) |
			((u64)dw_mshc_readl(host, SDHCI_ADMA_SA_HIGH) << 32);
			dw_mshc_writel(host,
				       SDHCI_ADMA_SA_LOW, addr & 0xffffffff);
			dw_mshc_writel(host, SDHCI_ADMA_SA_HIGH, addr >> 32);
		}
		if (status & mask)
			break;
		if (is_timeout(start, 10000 * MSECOND)) {
			dev_err(host->mci.hw_dev,
			"SDHCI timeout while waiting for done\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int dw_mshc_wait_for_status_mask(struct dw_mshc_mshci *host,
					struct mci_cmd *cmd, u16 mask)
{
	int ret;

	ret = dw_mshc_wait_for_done(host, mask);
	if (ret) {
		dev_err(host->mci.hw_dev, "error on command %d\n", cmd->cmdidx);
		dev_err(host->mci.hw_dev, "state = %04x %04x interrupt = %04x %04x\n",
			dw_mshc_readw(host, SDHCI_PRESENT_STATE),
			dw_mshc_readw(host, SDHCI_PRESENT_STATE1),
			dw_mshc_readw(host, SDHCI_INT_NORMAL_STATUS),
			dw_mshc_readw(host, SDHCI_INT_ERROR_STATUS));
	}
	dw_mshc_writew(host, SDHCI_INT_NORMAL_STATUS, mask);
	return ret;
}

static int dw_mshc_pio_xfer(struct dw_mshc_mshci *host, struct mci_cmd *cmd,
				struct mci_data *data)
{
	uint32_t *current;
	uint32_t quad;
	unsigned int i;
	int ret;

	if (data->flags & MMC_DATA_READ)
		current = (uint32_t *)data->dest;
	else
		current = (uint32_t *)data->src;

	for (i = 0; i < data->blocks; i++) {
		ret = dw_mshc_wait_for_status_mask(host, cmd,
		data->flags & MMC_DATA_READ ?
		SDHCI_INT_DATA_AVAIL : SDHCI_INT_SPACE_AVAIL);
		if (ret)
			return ret;

		if (data->flags & MMC_DATA_READ) {
			for (quad = 0; quad < data->blocksize / 4;
			quad++, current++)
				*current = dw_mshc_readl(host, SDHCI_BUFFER);
		} else { // WRITE
			for (quad = 0; quad < data->blocksize / 4;
			quad++, current++)
				dw_mshc_writel(host, SDHCI_BUFFER, *current);
		}
	}
	return 0;
}

static void mci_setup_cmd(struct mci_cmd *p, unsigned int cmd, unsigned int arg,
			  unsigned int response)
{
	p->cmdidx = cmd;
	p->cmdarg = arg;
	p->resp_type = response;
}

static int do_abort_sequence(struct mci_host *mci, struct mci_cmd *current_cmd)
{
	int ret = 0;
	struct dw_mshc_mshci *host = priv_from_mci_host(mci);
	struct mci_cmd cmd;
	u64 start;

	host->in_abort_sequence = 1;

	mci_setup_cmd(&cmd, MMC_CMD_STOP_TRANSMISSION, 0, MMC_RSP_R1b);
	ret = dw_mshc_mci_send_cmd(mci, &cmd, NULL);
	if (ret) {
		dev_err(host->mci.hw_dev, "Abort failed at first cmd12!\n");
		goto out;
	}
	mci_setup_cmd(&cmd, MMC_CMD_SEND_STATUS, mci->mci->rca << 16,
		      MMC_RSP_R1);
	ret = dw_mshc_mci_send_cmd(mci, &cmd, NULL);
	if (ret) {
		dev_err(host->mci.hw_dev, "Abort failed at first cmd13!\n");
		goto out;
	}

	if ((cmd.response[0] & CARD_STATUS_MASK) == CARD_STATUS_TRAN)
		goto out; /* All is OK! */

	mci_setup_cmd(&cmd, MMC_CMD_STOP_TRANSMISSION, 0, MMC_RSP_R1b);
	ret = dw_mshc_mci_send_cmd(mci, &cmd, NULL);
	if (ret) {
		dev_err(host->mci.hw_dev, "Abort failed at second cmd12!\n");
		goto out;
	}

	mci_setup_cmd(&cmd, MMC_CMD_SEND_STATUS, mci->mci->rca << 16,
		      MMC_RSP_R1);
	ret = dw_mshc_mci_send_cmd(mci, &cmd, NULL);
	if (ret) {
		dev_err(host->mci.hw_dev, "Abort failed at second cmd13!\n");
		goto out;
	}

	if ((cmd.response[0] & CARD_STATUS_MASK) == CARD_STATUS_TRAN) {
		goto out; /* All is OK! */
	} else {
		dev_err(host->mci.hw_dev,
			"Abort sequence failed to put card in TRAN state!\n");
		ret = 1;
		goto out;
	}

out:
	/* Perform SW reset if in abort sequence */
	dw_mshc_writeb(host, SDHCI_SOFTWARE_RESET,
		       SDHCI_RESET_DATA | SDHCI_RESET_CMD);
	start = get_time_ns();
	while (dw_mshc_readb(host, SDHCI_SOFTWARE_RESET) != 0) {
		if (is_timeout(start, 50 * MSECOND)) {
			dev_err(host->mci.hw_dev,
				"SDHCI data reset timeout\n");
			break;
		}
	}
	host->in_abort_sequence = 0;
	return ret;
}

static int dw_mshc_mci_send_cmd(struct mci_host *mci, struct mci_cmd *cmd,
				struct mci_data *data)
{
	u16 val;
	u64 start;
	int ret;
	unsigned int num_bytes = 0;
	struct dw_mshc_mshci *host = priv_from_mci_host(mci);
	dma_addr_t src = 0, dst = 0;

	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION
	    && host->in_abort_sequence == 0)
		return do_abort_sequence(mci, cmd);

	dw_mshc_writew(host, SDHCI_INT_ERROR_ENABLE, ~0);
	dw_mshc_writew(host, SDHCI_INT_ENABLE, ~0);
	dw_mshc_writel(host, SDHCI_INT_STATUS, ~0);

	/* Do not wait for CMD_INHIBIT_DAT on stop commands */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		val = SDHCI_CMD_INHIBIT_CMD;
	else
		val = SDHCI_CMD_INHIBIT_CMD | SDHCI_CMD_INHIBIT_DATA;

	/* Wait for bus idle */
	start = get_time_ns();
	while (1) {
		if (!(dw_mshc_readw(host, SDHCI_PRESENT_STATE) & val))
			break;
		if (is_timeout(start, 50 * MSECOND)) {
			u16 present = dw_mshc_readw(host, SDHCI_PRESENT_STATE);

			dev_err(host->mci.hw_dev,
				"SDHCI timeout while waiting for idle on cmd %d. PRESENT=0x%04x\n", cmd->cmdidx, present);
			return -ETIMEDOUT;
		}
	}

	/* setup transfer data */
	if (data) {
		unsigned char hostctrl1;

		hostctrl1 = dw_mshc_readb(host, SDHCI_HOST_CONTROL);
		hostctrl1 &= ~SDHCI_CTRL_DMA_MASK; // DMA_SEL = SDMA
		dw_mshc_writeb(host, SDHCI_HOST_CONTROL, hostctrl1);
		num_bytes = data->blocks * data->blocksize;
		dev_dbg(host->mci.hw_dev, "cmd %d arg %d bcnt %d bsize %d\n",
			cmd->cmdidx, cmd->cmdarg, data->blocks, data->blocksize);
		if (dw_mshc_sdma_supported(host)) {
			if (data->flags & MMC_DATA_READ) {
				dst = dma_map_single(mci->hw_dev, data->dest,
						num_bytes, DMA_FROM_DEVICE);
				if (dma_mapping_error(mci->hw_dev, dst)) {
					dev_err(host->mci.hw_dev,
						"Cannot map DMA buffer\n");
					ret = -ENOBUFS;
					goto dma_mapping_error;
				}
				dw_mshc_writel(host, SDHCI_ADMA_SA_LOW,
					(uintptr_t)dst);
				dw_mshc_writel(host, SDHCI_ADMA_SA_HIGH,
					(u64)dst >> 32);
			} else {
				src = dma_map_single(mci->hw_dev,
						(void *)data->src,
						num_bytes, DMA_TO_DEVICE);
				if (dma_mapping_error(mci->hw_dev, src)) {
					dev_err(host->mci.hw_dev,
						"Cannot map DMA buffer\n");
					ret = -ENOBUFS;
					goto dma_mapping_error;
				}
				dw_mshc_writel(host, SDHCI_ADMA_SA_LOW,
					(uintptr_t)src);
				dw_mshc_writel(host, SDHCI_ADMA_SA_HIGH,
					(u64)src >> 32);
			}
		}

		dw_mshc_writew(host, SDHCI_BLOCK_SIZE, SDHCI_DMA_BOUNDARY_512K |
				SDHCI_TRANSFER_BLOCK_SIZE(data->blocksize));
		dw_mshc_writew(host, SDHCI_BLOCK_COUNT, data->blocks);
		dw_mshc_writeb(host, SDHCI_TIMEOUT_CONTROL, 0xe);
		dw_mshc_writew(host, SDHCI_INT_ERROR_STATUS,
			       SDHCI_DATA_TOUT_ERR | SDHCI_CMD_TOUT_ERR);
	} else {
		dev_dbg(host->mci.hw_dev, "sending cmd %d arg %d\n",
			cmd->cmdidx, cmd->cmdarg);
	}

	dw_mshc_writel(host, SDHCI_ARGUMENT, cmd->cmdarg);
	/* setup transfer mode */
	val = 0;
	if (data) {
		if (dw_mshc_sdma_supported(host))
			val |= SDHCI_DMA_EN | SDHCI_BLOCK_COUNT_EN;
		else
			val |= SDHCI_BLOCK_COUNT_EN;
		if (data->blocks > 1)
			val |= SDHCI_MULTIPLE_BLOCKS;
		if (data->flags & MMC_DATA_READ)
			val |= SDHCI_DATA_TO_HOST;
		dw_mshc_writew(host, SDHCI_TRANSFER_MODE, val);
	}

	if (!(cmd->resp_type & MMC_RSP_PRESENT))
		val = SDHCI_RESP_NONE;
	else if (cmd->resp_type & MMC_RSP_136)
		val = SDHCI_RESP_TYPE_136;
	else if (cmd->resp_type & MMC_RSP_BUSY)
		val = SDHCI_RESP_TYPE_48_BUSY;
	else
		val = SDHCI_RESP_TYPE_48;

	if (cmd->resp_type & MMC_RSP_CRC)
		val |= SDHCI_CMD_CRC_CHECK_EN;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		val |= SDHCI_CMD_INDEX_CHECK_EN;
	if (data)
		val |= SDHCI_DATA_PRESENT;
	val |= SDHCI_CMD_INDEX(cmd->cmdidx);

	dw_mshc_writew(host, SDHCI_COMMAND, val);

	ret = dw_mshc_wait_for_status_mask(host, cmd, SDHCI_INT_CMD_COMPLETE);
	if (ret)
		goto cmd_error;

	/* CRC is stripped so we need to do some shifting. */
	if (cmd->resp_type & MMC_RSP_136) {
		int i;

		for (i = 0; i < 4; i++) {
			cmd->response[i] = dw_mshc_readl(host,
					SDHCI_RESPONSE_0 + 4*(3-i)) << 8;
			if (i != 3)
				cmd->response[i] |= dw_mshc_readb(host,
					SDHCI_RESPONSE_0 + 4*(3-i) - 1);
		}
	} else {
		cmd->response[0] = dw_mshc_readl(host, SDHCI_RESPONSE_0);
		dev_dbg(host->mci.hw_dev, "R1 card status: %08X\n",
			cmd->response[0]);
	}

	if (data) {
		if (!dw_mshc_sdma_supported(host)) {
			ret = dw_mshc_pio_xfer(host, cmd, data);
			if (ret) {
				dev_err(host->mci.hw_dev, "error during PIO xfer\n");
				goto cmd_error;
			}
		}
		ret = dw_mshc_wait_for_status_mask(host, cmd,
						SDHCI_INT_XFER_COMPLETE);
		if (ret)
			goto cmd_error;
	}

cmd_error:
	if (dw_mshc_sdma_supported(host)) {
		if (data && (data->flags & MMC_DATA_READ)) {
			dma_unmap_single(mci->hw_dev, dst, num_bytes,
					DMA_FROM_DEVICE);
		} else if (data && (data->flags & MMC_DATA_WRITE)) {
			dma_unmap_single(mci->hw_dev, src, num_bytes,
					DMA_TO_DEVICE);
		}
	}

dma_mapping_error:
	dw_mshc_writel(host, SDHCI_INT_STATUS, ~0);
	return ret;
}

static u16 dw_mshc_get_clock_divider(struct dw_mshc_mshci *host, u32 reqclk)
{
	u16 div;
	u32 clock_freq = clk_get_rate(host->clk);

	for (div = 1; div < SDHCI_MAX_DIV_SPEC_300; div += 2)
		if ((clock_freq / div) <= reqclk)
			break;
	div /= 2;

	return div;
}

static void dw_mshc_mci_set_ios(struct mci_host *mci, struct mci_ios *ios)
{
	u16 val;
	u64 start;

	struct dw_mshc_mshci *host = priv_from_mci_host(mci);

	debug("%s: clock = %u, bus-width = %d, timing = %02x\n", __func__,
	ios->clock, ios->bus_width, ios->timing);

	/* disable on zero clock */
	if (!ios->clock)
		return;

	/* enable bus power */
	val = SDHCI_BUS_VOLTAGE_330;
	dw_mshc_writeb(host, SDHCI_POWER_CONTROL, val | SDHCI_BUS_POWER_EN);
	udelay(400);

	/* set bus width */
	val = dw_mshc_readb(host, SDHCI_HOST_CONTROL) &
		~(SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS);
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_8:
		val |= SDHCI_CTRL_8BITBUS;
		break;
	case MMC_BUS_WIDTH_4:
		val |= SDHCI_CTRL_4BITBUS;
		break;
	case MMC_BUS_WIDTH_1:
		break;
	}

	if (ios->clock > 26000000)
		val |= SDHCI_CTRL_HISPD;
	else
		val &= ~SDHCI_CTRL_HISPD;

	dw_mshc_writeb(host, SDHCI_HOST_CONTROL, val);

	/* set bus clock */
	dw_mshc_writew(host, SDHCI_CLOCK_CONTROL, 0);
	val = dw_mshc_get_clock_divider(host, ios->clock);
	val = SDHCI_CLOCK_INT_EN | SDHCI_FREQ_SEL(val) | ((val & 0x300) >> 2);
	dw_mshc_writew(host, SDHCI_CLOCK_CONTROL, val);

	/* wait for internal clock stable */
	start = get_time_ns();
	while (!(dw_mshc_readw(host, SDHCI_CLOCK_CONTROL) &
			SDHCI_CLOCK_INT_STABLE)) {
		if (is_timeout(start, 20 * MSECOND)) {
			dev_err(host->mci.hw_dev,
			"SDHCI clock stable timeout\n");
			return;
		}
	}

	/* enable bus clock */
	dw_mshc_writew(host, SDHCI_CLOCK_CONTROL, val | SDHCI_CLOCK_CARD_EN);
}

static int dw_mshc_mci_init(struct mci_host *mci, struct device *dev)
{
	u64 start;
	u16 ctrl2;

	struct dw_mshc_mshci *host = priv_from_mci_host(mci);

	/* reset mshci controller */
	dw_mshc_writeb(host, SDHCI_SOFTWARE_RESET, SDHCI_RESET_ALL);

	/* wait for reset completion */
	start = get_time_ns();
	while (1) {
		if ((dw_mshc_readb(host, SDHCI_SOFTWARE_RESET) &
				SDHCI_RESET_ALL) == 0)
			break;
		if (is_timeout(start, 100 * MSECOND)) {
			dev_err(dev, "SDHCI reset timeout\n");
			return -ETIMEDOUT;
		}
	}

	dw_mshc_writel(host, SDHCI_INT_STATUS, ~0);
	dw_mshc_writew(host, SDHCI_INT_ERROR_ENABLE, 0x1fff);
	dw_mshc_writew(host, SDHCI_INT_ENABLE, 0x7fff);
	dw_mshc_writel(host, SDHCI_SIGNAL_ENABLE, ~0);

	// Enable host_version4
	ctrl2 = dw_mshc_readw(host,
	SDHCI_HOST_CONTROL2) | SDHCI_HOST_VER4_ENABLE;
	dw_mshc_writew(host, SDHCI_HOST_CONTROL2, ctrl2);
	ctrl2 = dw_mshc_readw(host, SDHCI_HOST_CONTROL2);

	// Enable 64-bit addressing
	ctrl2 |= SDHCI_ADDRESSING;
	dw_mshc_writew(host, SDHCI_HOST_CONTROL2, ctrl2);

	return 0;
}

static void dw_mshc_set_mci_caps(struct dw_mshc_mshci *host)
{
	u32 caps = dw_mshc_readl(host, SDHCI_CAPABILITIES);

	if (caps & SDHCI_CAN_VDD_180)
		host->mci.voltages |= MMC_VDD_165_195;
	if (caps & SDHCI_CAN_VDD_300)
		host->mci.voltages |= MMC_VDD_29_30 | MMC_VDD_30_31;
	if (caps & SDHCI_CAN_VDD_330)
		host->mci.voltages |= MMC_VDD_32_33 | MMC_VDD_33_34;

	if (caps & SDHCI_CAN_DO_HISPD)
		host->mci.host_caps |= (MMC_CAP_MMC_HIGHSPEED_52MHZ |
					MMC_CAP_MMC_HIGHSPEED |
					MMC_CAP_SD_HIGHSPEED);

	/* parse board supported bus width capabilities */
	mci_of_parse(&host->mci);

	/* limit bus widths to controller capabilities */
	if ((caps & SDHCI_CAN_DO_8BIT) == 0)
		host->mci.host_caps &= ~MMC_CAP_8_BIT_DATA;
}

static int dw_mshc_detect(struct device *dev)
{
	struct dw_mshc_mshci *host = dev->priv;

	return mci_detect_card(&host->mci);
}

static int dw_mshc_mci_card_present(struct mci_host *mci)
{
	u32 pstate;
	struct dw_mshc_mshci *host = priv_from_mci_host(mci);

	pstate = dw_mshc_readl(host, SDHCI_PRESENT_STATE);
	return pstate & SDHCI_CARD_PRESENT;
}

static void dw_mshc_set_dma_mask(struct device *dev)
{
	struct dw_mshc_mshci *host = dev->priv;

	if (host->cap1 & SDHCI_CAN_64BIT_V4)
		dma_set_mask(dev, DMA_BIT_MASK(64));
	else
		dma_set_mask(dev, DMA_BIT_MASK(32));
}

static int dw_mshc_probe(struct device *dev)
{
	struct dw_mshc_mshci *host;
	int ret;
	u16 ctrl2;
	struct clk *clk;

	clk = clk_get(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	clk_enable(clk);

	host = xzalloc(sizeof(*host));
	host->clk = clk;
	host->base = dev_request_mem_region(dev, 0);
	host->in_abort_sequence = 0;
	host->mci.max_req_size = 0x8000;
	host->mci.hw_dev = dev;
	host->mci.send_cmd = dw_mshc_mci_send_cmd;
	host->mci.card_present = dw_mshc_mci_card_present;
	host->mci.set_ios = dw_mshc_mci_set_ios;
	host->mci.init = dw_mshc_mci_init;
	host->mci.f_min = clk_get_rate(clk) / SDHCI_MAX_DIV_SPEC_300;
	host->cap1 = dw_mshc_readl(host, SDHCI_CAPABILITIES);
	host->cap2 = dw_mshc_readl(host, SDHCI_CAPABILITIES_1);
	dev->priv = host;
	dev->detect = dw_mshc_detect;

	dw_mshc_set_dma_mask(dev);

	// Enable host_version4
	ctrl2 = dw_mshc_readw(host,
	SDHCI_HOST_CONTROL2) | SDHCI_HOST_VER4_ENABLE;
	dw_mshc_writew(host, SDHCI_HOST_CONTROL2, ctrl2);
	ctrl2 = dw_mshc_readw(host, SDHCI_HOST_CONTROL2);
	host->version = dw_mshc_readw(host, SDHCI_HOST_VERSION) & 0x7;

	// Enable 64-bit addressing
	ctrl2 |= SDHCI_ADDRESSING;
	dw_mshc_writew(host, SDHCI_HOST_CONTROL2, ctrl2);

	dev_dbg(host->mci.hw_dev, "host controller version : %u\n",
	host->version);
	dev_dbg(host->mci.hw_dev, "host_version4 enable? %s\n",
	(ctrl2 & SDHCI_HOST_VER4_ENABLE) ? "Yes" : "No");

	dw_mshc_set_mci_caps(host);

	ret = mci_register(&host->mci);
	if (ret)
		free(host);
	return ret;
}

static struct of_device_id dw_mshc_dt_ids[] = {
	{ .compatible = "snps,dwcmshc-sdhci", },
	{ }
};

static struct driver dw_mshc_driver = {
	.name = "dwc-mshc",
	.probe = dw_mshc_probe,
	.of_compatible = DRV_OF_COMPAT(dw_mshc_dt_ids),
};
device_platform_driver(dw_mshc_driver);
