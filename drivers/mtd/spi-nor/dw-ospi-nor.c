
#include <clock.h>
#include <common.h>
#include <driver.h>
#include <errno.h>
#include <init.h>
#include <io.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/spi-nor.h>
#include <of.h>
#include <spi/spi.h>

#define DRIVER_NAME "dw_ospi_nor"

/* Register offsets */
#define DW_SPI_CTRL0			0x00
#define DW_SPI_CTRL1			0x04
#define DW_SPI_SSIENR			0x08
#define DW_SPI_MWCR			0x0c
#define DW_SPI_SER			0x10
#define DW_SPI_BAUDR			0x14
#define DW_SPI_TXFLTR			0x18
#define DW_SPI_RXFLTR			0x1c
#define DW_SPI_TXFLR			0x20
#define DW_SPI_RXFLR			0x24
#define DW_SPI_SR			0x28
#define DW_SPI_IMR			0x2c
#define DW_SPI_ISR			0x30
#define DW_SPI_RISR			0x34
#define DW_SPI_TXOICR			0x38
#define DW_SPI_RXOICR			0x3c
#define DW_SPI_RXUICR			0x40
#define DW_SPI_MSTICR			0x44
#define DW_SPI_ICR			0x48
#define DW_SPI_DMACR			0x4c
#define DW_SPI_DMATDLR			0x50
#define DW_SPI_DMARDLR			0x54
#define DW_SPI_IDR			0x58
#define DW_SPI_VERSION			0x5c
#define DW_SPI_DR			0x60
#define DW_SPI_SPI_CTRL0		0xf4

/* Bit fields in CTRLR0 */
#define SPI_DFS_OFFSET			0
#define SPI_DFS_MASK			(0x1f << SPI_DFS_OFFSET)

#define SPI_FRF_OFFSET			6
#define SPI_FRF_SPI			0x0
#define SPI_FRF_SSP			0x1
#define SPI_FRF_MICROWIRE		0x2
#define SPI_FRF_RESV			0x3

#define SPI_MODE_OFFSET			8
#define SPI_SCPH_OFFSET			8
#define SPI_SCOL_OFFSET			9

#define SPI_TMOD_OFFSET			10
#define SPI_TMOD_MASK			(0x3 << SPI_TMOD_OFFSET)
#define	SPI_TMOD_TR			0x0		/* xmit & recv */
#define SPI_TMOD_TO			0x1		/* xmit only */
#define SPI_TMOD_RO			0x2		/* recv only */
#define SPI_TMOD_EPROMREAD		0x3		/* eeprom read mode */

#define SPI_SLVOE_OFFSET		12
#define SPI_SRL_OFFSET			13
#define SPI_SSTE_OFFSET			14

#define SPI_CFS_OFFSET			16
#define SPI_CFS_MASK			(0xf << SPI_CFS_OFFSET)

#define SPI_SPI_FRF_OFFSET		22
#define SPI_SPI_FRF_MASK		(0x3 << SPI_SPI_FRF_OFFSET)
#define SPI_STANDARD_FORMAT		0
#define SPI_DUAL_FORMAT			1
#define SPI_QUAD_FORMAT			2
#define SPI_OCTAL_FORMAT		3


/* Bit fields in SR, 7 bits */
#define SR_MASK				0x7f		/* cover 7 bits */
#define SR_BUSY				BIT(0)
#define SR_TF_NOT_FULL			BIT(1)
#define SR_TF_EMPT			BIT(2)
#define SR_RF_NOT_EMPT			BIT(3)
#define SR_RF_FULL			BIT(4)
#define SR_TX_ERR			BIT(5)
#define SR_DCOL				BIT(6)

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI			BIT(0)
#define SPI_INT_TXOI			BIT(1)
#define SPI_INT_RXUI			BIT(2)
#define SPI_INT_RXOI			BIT(3)
#define SPI_INT_RXFI			BIT(4)
#define SPI_INT_MSTI			BIT(5)

/* Bit fields in DMACR */
#define SPI_DMA_RDMAE			BIT(0)
#define SPI_DMA_TDMAE			BIT(1)

/* Fields in SPI_CTRL0*/
#define SPI_SPI_CTRL0_INST_L8		(0x2 << 8) /*two bit value*/
#define SPI_SPI_CTRL0_ADDR_L32		(0x8 << 2) /*four bit value*/
#define SPI_SPI_CTRL0_WAIT_8_CYCLE	(0x8 << 11)/*five bit value*/
#define SPI_SPI_CTRL0_EN_CLK_STRETCH    BIT(30)

#define SPINOR_OP_RDBR      0x16    /* Read bank address register*/
#define SPINOR_OP_WRBRNV    0x18    /* Write non volatile bank register */
#define SPINOR_OP_RDRP      0x61    /* read read parameters */
#define SPINOR_OP_SRPNV     0x65    /* Write read parameters non volatile*/
#define SPINOR_OP_RSTEN     0x66    /* Enable reset flash */
#define SPINOR_OP_RST       0x99    /* Reset flash*/

/* Read bank register flag */
#define RDBR_EXTADD BIT(7)

/* Read register parameter */
#define RD_PARAM_DUMMYCYCLE_MASK 0x78
#define RD_PARAM_DFLT_DUMMY_CYCLE 0x00

/* TX/RX FIFO maximum size */
#define TX_FIFO_MAX_SIZE 256
#define RX_FIFO_MAX_SIZE 256

/* various timeout */
#define TRANSMIT_FIFO_EMPTY_TIMEOUT_NS 100000000 /* 100 ms*/
#define TRANSMIT_BUSY_TIMEOUT_NS 100000000 /* 100 ms*/

/* TX RX interrupt level threshold, max can be 256 */
#define SPI_INT_THRESHOLD		32

#define DW_SPI_MAX_CHIPSELECT 16

#define DEFAULT_READY_WAIT (1 * SECOND)

struct dw_spi_flash_pdata {
	struct mtd_info	mtd;
	struct spi_nor nor;
	u32 clk_rate;
	unsigned int block_size;
	unsigned int read_delay;
	unsigned int tshsl_ns;
	unsigned int tsd2d_ns;
	unsigned int tchsh_ns;
	unsigned int tslch_ns;
};

struct dw_spi_nor {
	struct device_d *dev;
	struct clk *clk;
	unsigned int sclk;
	void __iomem *regs;
	unsigned int master_ref_clk_hz;
	struct dw_spi_flash_pdata f_pdata[DW_SPI_MAX_CHIPSELECT];

	u32 reg_io_width;	/* DR I/O width in bytes */
	int clk_strech_en;
	int	supported_cs;
	int	current_cs;
	int tx_fifo_len;
	int rx_fifo_len;
};

struct dw_spi_register {
	u32 addr;
	u32 val;
};

static struct spi_nor *dw_nor;
static struct dw_spi_register regs_reset[] =  {
	{ DW_SPI_CTRL0, 0}
	, { DW_SPI_CTRL1, 0}
	, { DW_SPI_SER, 0}
	, { DW_SPI_BAUDR, 0}
	, { DW_SPI_TXFLTR, 0}
	, { DW_SPI_RXFLTR, 0}
	, { DW_SPI_SPI_CTRL0, 0}
};
static struct dw_spi_register regs_backup[ARRAY_SIZE(regs_reset)];

static __maybe_unused u32 dw_readl(struct dw_spi_nor *dws, u32 offset)
{
	return __raw_readl(dws->regs + offset);
}

static __maybe_unused u16 dw_readw(struct dw_spi_nor *dws, u32 offset)
{
	return __raw_readw(dws->regs + offset);
}

static __maybe_unused void dw_writel(
		struct dw_spi_nor *dws
		, u32 offset, u32 val)
{
	__raw_writel(val, dws->regs + offset);
}

static __maybe_unused void dw_writew(
		struct dw_spi_nor *dws
		, u32 offset, u16 val)
{
	__raw_writew(val, dws->regs + offset);
}

static inline u32 dw_read_io_reg(struct dw_spi_nor *dws, u32 offset)
{
	switch (dws->reg_io_width) {
	case 2:
		return __raw_readw(dws->regs + offset);
	case 4:
	default:
		return __raw_readl(dws->regs + offset);
	}
}

static inline void dw_write_io_reg(struct dw_spi_nor *dws, u32 offset, u32 val)
{
	switch (dws->reg_io_width) {
	case 2:
		__raw_writew(val, dws->regs + offset);
		break;
	case 4:
	default:
		__raw_writel(val, dws->regs + offset);
		break;
	}
}

static inline void dw_spi_enable_chip(struct dw_spi_nor *dws, int enable)
{
	dw_writel(dws, DW_SPI_SSIENR, (enable ? 1 : 0));
}

/* Disable IRQ bits */
static inline void dw_spi_mask_intr(struct dw_spi_nor *dws, u32 mask)
{
	u32 new_mask;

	new_mask = dw_readl(dws, DW_SPI_IMR) & ~mask;
	dw_writel(dws, DW_SPI_IMR, new_mask);
}

/*
 * This does disable the SPI controller, interrupts, and re-enable the
 * controller back. Transmit and receive FIFO buffers are cleared when the
 * device is disabled.
 */
static inline void dw_spi_reset_chip(struct dw_spi_nor *dw_spi)
{
	dw_spi_enable_chip(dw_spi, 0);
	dw_spi_mask_intr(dw_spi, 0xff);
	dw_spi_enable_chip(dw_spi, 1);
}

static int dw_spi_set_cs(struct dw_spi_nor *dw_spi, int cs)
{
	if (cs > dw_spi->supported_cs) {
		dev_err(dw_spi->dev, "invalid chip select\n");
		return -EINVAL;
	}

	dw_spi_enable_chip(dw_spi, 0);

	if (cs == -1) {
		/* No slave */
		dw_writel(dw_spi, DW_SPI_SER, 0);
	} else {
		dw_writel(dw_spi, DW_SPI_SER, BIT(cs));
	}
	dw_spi->current_cs = cs;

	dw_spi_enable_chip(dw_spi, 1);

	return 0;
}

static void dw_spi_hw_init(struct dw_spi_nor *dw_spi)
{
	u32 num;
	u32 ctrl0;
	u32 spi_ctrl0;

	dw_spi_reset_chip(dw_spi);

	dw_spi_enable_chip(dw_spi, 0);

	for (num = 0; num < ARRAY_SIZE(regs_reset); num++)
		regs_reset[num].val = dw_readl(dw_spi, regs_reset[num].addr);

	/*
	 * The line will automatically toggle between consecutive data frame
	 */
	ctrl0 = dw_readl(dw_spi, DW_SPI_CTRL0);
	ctrl0 &= ~(BIT(SPI_SSTE_OFFSET));
	dw_writel(dw_spi, DW_SPI_CTRL0, ctrl0);

	/*
	 * SPI_CTRL0 is initializtion
	 */
	spi_ctrl0  = SPI_SPI_CTRL0_INST_L8;
	spi_ctrl0 |= SPI_SPI_CTRL0_ADDR_L32;
	spi_ctrl0 |= SPI_SPI_CTRL0_WAIT_8_CYCLE;
	spi_ctrl0 |= SPI_SPI_CTRL0_EN_CLK_STRETCH;

	dw_writel(dw_spi, DW_SPI_SPI_CTRL0, spi_ctrl0);

	dw_spi_enable_chip(dw_spi, 1);
}

static int dw_spi_of_get_flash_pdata(struct device_d *dev,
				    struct dw_spi_flash_pdata *f_pdata,
				    struct device_node *np)
{
	if (!np)
		return 0;

	if (of_property_read_u32(np, "spi-max-frequency", &f_pdata->clk_rate)) {
		dev_err(dev, "couldn't determine spi-max-frequency\n");
		return -ENXIO;
	}

	dev_dbg(dev, "spi-max-frequency = %u\n", f_pdata->clk_rate);

	return 0;
}

static int dw_spi_find_chipselect(struct spi_nor *nor)
{
	int cs = -1;
	struct dw_spi_nor *dw_spi = nor->priv;

	for (cs = 0; cs < dw_spi->supported_cs; cs++)
		if (nor == &dw_spi->f_pdata[cs].nor)
			break;
	return cs;
}

static int dw_spi_config_baudrate_div(struct dw_spi_nor *dws, unsigned int sclk)
{
	unsigned int div;

	dws->sclk = sclk;
	div = dws->master_ref_clk_hz / sclk;

	dev_dbg(dws->dev, "configure clock divider (%u/%u) -> %u\n",
			dws->master_ref_clk_hz, sclk, div);
	dw_spi_enable_chip(dws, 0);
	dw_writel(dws, DW_SPI_BAUDR, div);
	dw_spi_enable_chip(dws, 1);

	if (dw_readl(dws, DW_SPI_BAUDR) != div) {
		dev_err(dws->dev, "Unable to configure clock divider\n");
		return -EINVAL;
	}
	return 0;
}

const char *flash_read_str[] = {
	"SPI_NOR_NORMAL",
	"SPI_NOR_FAST",
	"SPI_NOR_DUAL",
	"SPI_NOR_QUAD"
};

static int dw_spi_prep(
		struct spi_nor *nor
		, enum spi_nor_ops ops
		, bool enable_quad)
{
	struct dw_spi_nor *dw_spi = nor->priv;
	int cs = dw_spi_find_chipselect(nor);
	struct dw_spi_flash_pdata *f_pdata;
	unsigned int sclk;
	u32 ctrl0;
	int ret = 0;

	/* Switch chip select. */
	if (dw_spi->current_cs != cs) {
		ret = dw_spi_set_cs(dw_spi, cs);
		if (ret)
			return ret;
	}

	/* Setup baudrate divisor */
	f_pdata = &dw_spi->f_pdata[dw_spi->current_cs];
	sclk = f_pdata->clk_rate;
	if (dw_spi->sclk != sclk) {
		ret = dw_spi_config_baudrate_div(dw_spi, sclk);
		if (ret)
			return ret;
	}

	/* Spi mode configuration */
	ctrl0 = dw_readl(dw_spi, DW_SPI_CTRL0);
	ctrl0 &= ~SPI_SPI_FRF_MASK;
	ctrl0 &= ~SPI_TMOD_MASK;

	if (ops == SPI_NOR_OPS_READ)
		ctrl0 |= SPI_TMOD_RO << SPI_TMOD_OFFSET;
	else
		ctrl0 |= SPI_TMOD_TO << SPI_TMOD_OFFSET;

	if ((enable_quad == false) || (ops == SPI_NOR_OPS_WRITE)) {
		ctrl0 |= SPI_STANDARD_FORMAT << SPI_SPI_FRF_OFFSET;
		ctrl0 &= ~SPI_TMOD_MASK;  /* Force TR mode in normal spi */
	} else {
		switch (nor->flash_read) {
		case SPI_NOR_NORMAL:
			dev_dbg(nor->dev, "read single\n");
			ctrl0 |= SPI_STANDARD_FORMAT << SPI_SPI_FRF_OFFSET;
			ctrl0 &= ~SPI_TMOD_MASK;/*Force TR mode, normal spi*/
			break;
		case SPI_NOR_QUAD:
			dev_dbg(nor->dev, "read quad\n");
			ctrl0 |= SPI_QUAD_FORMAT << SPI_SPI_FRF_OFFSET;
			break;
		default:
			return -EINVAL;
		}
	}

	dw_spi_enable_chip(dw_spi, 0);
	dw_writel(dw_spi, DW_SPI_CTRL0, ctrl0);
	dw_spi_enable_chip(dw_spi, 1);
	return ret;
}

static int dw_spi_command_read(struct spi_nor *nor,
		const u8 opcode, int address,
		u8 *rxbuf, unsigned int n_rx)
{
	struct dw_spi_nor *dw_spi = nor->priv;
	int total_tx, total_rx;
	int tx_cnt, rx_cnt, skip_rx;
	u32 ctrl0_tmod;
	int ret = 0;
	int i;
	int fifo_start_level = 0;

	tx_cnt = 0;
	rx_cnt = 0;

	while (dw_readl(dw_spi, DW_SPI_SR) & SR_BUSY)
		;

	/* Clear interrupts */
	dw_readl(dw_spi, DW_SPI_ICR);

	ctrl0_tmod = (dw_readl(dw_spi, DW_SPI_CTRL0) & SPI_TMOD_MASK)
					>> SPI_TMOD_OFFSET;

	/* TX Fifo must not became empty during the frame transfer
	 * Use TXFTHR (Transfert Start FIFO level) to avoid the frame
	 * to start during the first phases computation
	 */

	if (ctrl0_tmod == SPI_TMOD_TR) {
		total_tx = n_rx;
		skip_rx = 1 /* opcode */ + nor->addr_width;
		total_rx = skip_rx + n_rx;
		fifo_start_level = skip_rx + total_tx;
	} else {
		/* We are in enhanced SPI mode => receive only */
		total_tx = 0;
		total_rx = n_rx;
		skip_rx = 0;
		fifo_start_level = 1 /* opcode */ + 1 /* address */;
	}
	if (fifo_start_level > dw_spi->tx_fifo_len)
		fifo_start_level = dw_spi->tx_fifo_len;

	dw_spi_enable_chip(dw_spi, 0);
	dw_writel(dw_spi, DW_SPI_TXFLTR, (fifo_start_level-1) << 16);
	dw_writel(dw_spi, DW_SPI_RXFLTR, dw_spi->rx_fifo_len/2);
	dw_spi_enable_chip(dw_spi, 1);

	/* Opcode Phase */
	dw_write_io_reg(dw_spi, DW_SPI_DR, opcode);

	/* Address phase in TR mode */
	if (ctrl0_tmod == SPI_TMOD_TR) {
		for (i = 0; i < nor->addr_width; i++)
			dw_write_io_reg(dw_spi, DW_SPI_DR
				, (address >> (8*i)) & 0xff);
	} else {
		if (address != -1)
			dw_write_io_reg(dw_spi, DW_SPI_DR, address);
	}

	while (rx_cnt < total_rx) {
		if (tx_cnt < total_tx) {
			if (dw_readl(dw_spi, DW_SPI_SR) & SR_TF_NOT_FULL) {
				dw_write_io_reg(dw_spi, DW_SPI_DR, 0xff);
				tx_cnt++;
			}
		}

		if (rx_cnt < total_rx) {
			while (dw_readl(dw_spi, DW_SPI_SR) & SR_RF_NOT_EMPT) {
				if (rx_cnt < skip_rx) {
					dw_read_io_reg(dw_spi, DW_SPI_DR);
				} else {
					rxbuf[rx_cnt-skip_rx]
						= dw_read_io_reg(
							dw_spi, DW_SPI_DR);
					ret++;
				}
				rx_cnt++;
			}
		}
		/* Check RX overflow */
		if (dw_readl(dw_spi, DW_SPI_RISR) & SPI_INT_RXOI)
			return -EIO;
	}
	return ret;
}

static int dw_spi_command_write(struct spi_nor *nor,
		const u8 *opbuf, unsigned int n_op,
		u8 *txbuf, unsigned int n_tx)
{
	struct	dw_spi_nor *dw_spi = nor->priv;
	int	op_cnt, tx_cnt, txfhr;
	int	res;

	while (dw_readl(dw_spi, DW_SPI_SR) & SR_BUSY)
		;

	op_cnt = 0;
	tx_cnt = 0;

	txfhr = min((unsigned int)dw_spi->tx_fifo_len-1, n_op-1+n_tx);
	dw_spi_enable_chip(dw_spi, 0);
	dw_writel(dw_spi, DW_SPI_TXFLTR, txfhr << 16);
	dw_spi_enable_chip(dw_spi, 1);

	/* Send opcodes */
	while (op_cnt < n_op) {
		if (dw_readl(dw_spi, DW_SPI_SR) & SR_TF_NOT_FULL) {
			dw_write_io_reg(dw_spi, DW_SPI_DR, opbuf[op_cnt]);
			op_cnt++;
		}
	}

	/* Send Data */
	while (tx_cnt < n_tx) {
		if (dw_readl(dw_spi, DW_SPI_SR) & SR_TF_NOT_FULL) {
			dw_write_io_reg(dw_spi, DW_SPI_DR, txbuf[tx_cnt]);
			tx_cnt++;
		}
	}

	/* As specified in ssi_user_guide p63 the BUSY bit cannot be polled
	 * immediatelly.As indicated in ssi_databook p40 the TFE bit shall
	 * be tested before testing busy bit
	 */
	res = wait_on_timeout(TRANSMIT_FIFO_EMPTY_TIMEOUT_NS
			, dw_readl(dw_spi, DW_SPI_SR) & SR_TF_EMPT);
	if (res < 0) {
		dev_err(nor->dev, "SPI write failure, TX FIFO is never empty\n");
		return res;
	}

	wait_on_timeout(TRANSMIT_BUSY_TIMEOUT_NS
			, !(dw_readl(dw_spi, DW_SPI_SR) & SR_BUSY));
	if (res < 0) {
		dev_err(nor->dev, "SPI write failure, transfer never end\n");
		return res;
	}

	return 0;
}

static int dw_spi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret = 0;
	int i;

	ret = dw_spi_prep(nor, SPI_NOR_OPS_READ, false);

	if (ret)
		return ret;

	ret = dw_spi_command_read(nor, opcode, -1, buf, len);

	dev_dbg(nor->dev, "read_reg opcode 0x%02x\n", opcode);
	for (i = 0 ; i < len; i++)
		dev_dbg(nor->dev, "0x%02x ", buf[i]);
	dev_dbg(nor->dev, "\n");

	return ret;
}

static int dw_spi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len,
			   int write_enable)
{
	int ret = 0;
	int i = 0;

	dev_dbg(nor->dev, "write_reg opcode 0x%02x : ", opcode);
	for (i = 0; i < len; i++)
		dev_dbg(nor->dev, "0x%02x ", buf[i]);
	dev_dbg(nor->dev, "\n");

	if (!IS_ENABLED(CONFIG_MTD_WRITE))
		return -ENOTSUPP;

	ret = dw_spi_prep(nor, SPI_NOR_OPS_WRITE, false);
	if (ret)
		return ret;

	if (write_enable) {
		u8 wren_code = SPINOR_OP_WREN;

		dw_spi_command_write(nor, &wren_code, 1, NULL, 0);
	}

	ret = dw_spi_command_write(nor, &opcode, 1, buf, len);
	return ret;
}

static void dw_spi_write(struct spi_nor *nor, loff_t to,
			size_t len, size_t *retlen, const u_char *buf)
{
	u8  opcode[8];
	int i;
	struct dw_spi_nor *dw_spi = nor->priv;

	dev_dbg(dw_spi->dev, "write %u bytes at @0x%x\n"
			, (unsigned int)len
			, (unsigned int)to);

	if (!IS_ENABLED(CONFIG_MTD_WRITE))
		return;

	if (dw_spi_prep(nor, SPI_NOR_OPS_WRITE, false))
		return;

	opcode[0] = nor->program_opcode;
	for (i = 0 ; i < nor->addr_width; i++)
		opcode[1+i] = (to >> (8*(nor->addr_width - 1 - i))) & 0xff;

	dw_spi_command_write(nor, opcode, 1 + nor->addr_width, (u8 *)buf, len);

}

static int dw_spi_read(struct spi_nor *nor, loff_t from,
		      size_t len, size_t *retlen, u_char *buf)
{
	struct dw_spi_nor *dw_spi = nor->priv;
	int to_read = len;
	u8 *ptr = (u8 *)buf;
	loff_t offset = from;
	int ret = 0;
	int chunk;

	dev_dbg(nor->dev, "read %u bytes from @0x%x\n"
			, (unsigned int)len
			, (unsigned int)from);

	ret = dw_spi_prep(nor, SPI_NOR_OPS_READ, true);
	if (ret)
		return ret;

	/* If clock stretching is not supported, we have no way to prevent RX
	 * overflow except reducing the number reveice data to the size of the
	 * RX fifo
	 */

	if (dw_spi->clk_strech_en)
		chunk = len;
	else
		chunk = dw_spi->rx_fifo_len;

	dw_spi_enable_chip(dw_spi, 0);
	dw_writel(dw_spi, DW_SPI_CTRL1, chunk - 1);
	dw_spi_enable_chip(dw_spi, 1);

	*retlen = 0;
	while (to_read) {
		if (to_read < chunk) {
			chunk = to_read;
			dw_spi_enable_chip(dw_spi, 0);
			dw_writel(dw_spi, DW_SPI_CTRL1, chunk - 1);
			dw_spi_enable_chip(dw_spi, 1);
		}

		*retlen += dw_spi_command_read(nor, nor->read_opcode
				, offset, ptr, chunk);
		to_read -= chunk;
		offset += chunk;
		ptr += chunk;
	}

	return ret;
}

static int dw_spi_erase(struct spi_nor *nor, loff_t offs)
{
	int ret = 0;
	int offs_be = cpu_to_be32(offs);

	dev_dbg(nor->dev, "erase(%0x) @0x%x\n"
			, (unsigned int)nor->erase_opcode
			, (unsigned int)offs);

	ret = dw_spi_prep(nor, SPI_NOR_OPS_ERASE, false);
	if (ret)
		return ret;


	/* Caller is responsible for enabling write
	 * , just send erase sector command
	 */
	ret = nor->write_reg(nor, nor->erase_opcode
			, (u8 *)&offs_be, nor->addr_width, 0);

	/* Caller is responsible to wait for operation completion */
	return ret;
}

static int dw_spi_setup_flash(struct device_d *dev,
			     struct dw_spi_flash_pdata *f_pdata,
			     struct device_node *np)
{
	struct dw_spi_nor *dw_spi = dev->priv;
	struct mtd_info *mtd;
	struct spi_nor *nor;
	int ret;

	ret = dw_spi_of_get_flash_pdata(dev, f_pdata, np);
	if (ret)
		goto probe_failed;

	nor = &f_pdata->nor;
	mtd = &f_pdata->mtd;

	nor->mtd = mtd;

	if (np) {
		nor->dev = kzalloc(sizeof(*nor->dev), GFP_KERNEL);
		if (!nor->dev)
			return -ENOMEM;

		dev_set_name(nor->dev, np->name);

		nor->dev->device_node = np;
		nor->dev->id = DEVICE_ID_SINGLE;
		nor->dev->parent = dev;
		ret = register_device(nor->dev);

		if (ret)
			return ret;

		mtd->parent = nor->dev;
	} else {
		nor->dev = dev;
	}

	nor->priv = dw_spi;
	mtd->priv = nor;

	nor->read_reg = dw_spi_read_reg;
	nor->write_reg = dw_spi_write_reg;
	nor->read = dw_spi_read;
	nor->write = dw_spi_write;
	nor->erase = dw_spi_erase;

	ret = spi_nor_scan(nor, NULL, SPI_NOR_QUAD, false);
	if (ret)
		goto probe_failed;

	ret = add_mtd_device(mtd, NULL, DEVICE_ID_DYNAMIC);
	if (ret)
		goto probe_failed;

	return 0;

probe_failed:
	dev_err(dev, "probing for flashchip failed\n");
	return ret;
}

static int dw_spi_probe(struct device_d *dev)
{
	struct dw_spi_nor *dw_spi;
	struct resource *iores;
	struct device_node *np = dev->device_node;
	struct dw_spi_flash_pdata *f_pdata = NULL;
	int fifo;
	int ret;

	dw_spi = kzalloc(sizeof(*dw_spi), GFP_KERNEL);
	if (!dw_spi)
		return -ENOMEM;

	dw_spi->dev = dev;
	dev->priv = dw_spi;

	dw_spi->clk = clk_get(dev, NULL);
	if (IS_ERR(dw_spi->clk)) {
		dev_err(dev, "unable to get spi clk\n");
		ret = PTR_ERR(dw_spi->clk);
		goto probe_failed;
	}

	dw_spi->master_ref_clk_hz = clk_get_rate(dw_spi->clk);
	if (dw_spi->master_ref_clk_hz == 0) {
		dev_err(dev, "unable to get spi clk rate\n");
		ret = PTR_ERR(dw_spi->clk);
		goto probe_failed;
	}

	clk_enable(dw_spi->clk);

	iores = dev_request_mem_resource(dev, 0);
	if (IS_ERR(iores)) {
		dev_err(dev, "dev_request_mem_region failed\n");
		ret = PTR_ERR(iores);
		goto probe_failed;
	}
	dw_spi->regs = IOMEM(iores->start);

	dw_spi_hw_init(dw_spi);

	/* Detect supported slave number */
	dw_spi_enable_chip(dw_spi, 0);
	dw_writel(dw_spi, DW_SPI_SER, 0xffff);
	dw_spi_enable_chip(dw_spi, 1);
	dw_spi->supported_cs = hweight32(dw_readl(dw_spi, DW_SPI_SER));

	dw_spi_set_cs(dw_spi, -1);
	dw_spi->sclk = 0;

	/*
	 * Detect the FIFO depth
	 */
	dw_spi_enable_chip(dw_spi, 0);
	for (fifo = 1; fifo < TX_FIFO_MAX_SIZE; fifo++) {
		dw_writel(dw_spi, DW_SPI_TXFLTR, fifo);
		if (fifo != dw_readl(dw_spi, DW_SPI_TXFLTR))
			break;
	}
	dw_writel(dw_spi, DW_SPI_TXFLTR, 0);
	dw_spi->tx_fifo_len = (fifo == 1) ? 0 : fifo;
	dev_dbg(dw_spi->dev, "Detected TX FIFO size: %u bytes\n"
			, dw_spi->tx_fifo_len);

	for (fifo = 1; fifo < RX_FIFO_MAX_SIZE; fifo++) {
		dw_writel(dw_spi, DW_SPI_RXFLTR, fifo);
		if (fifo != dw_readl(dw_spi, DW_SPI_RXFLTR))
			break;
	}
	dw_writel(dw_spi, DW_SPI_RXFLTR, 0);
	dw_spi->rx_fifo_len = (fifo == 1) ? 0 : fifo;
	dev_dbg(dw_spi->dev, "Detected RX FIFO size: %u bytes\n"
			, dw_spi->tx_fifo_len);
	dw_spi_enable_chip(dw_spi, 1);

	/*
	 * Get clock stretching mode support from device-tree
	 */
	if (of_property_read_u32(dev->device_node
				, "clock-stretching"
				, &dw_spi->clk_strech_en)) {
		dev_err(dev, "couldn't get clock-streching support in device-tree\n");
		return -ENXIO;
	}
	dev_dbg(dev, "clock stretching %s supported\n"
			, dw_spi->clk_strech_en ? "is" : "is not");

	if (!dev->device_node) {
		f_pdata = &dw_spi->f_pdata[0];

		ret = dw_spi_setup_flash(dev, f_pdata, np);
		if (ret)
			goto probe_failed;
	} else {
		/* Get flash device data */
		for_each_available_child_of_node(dev->device_node, np) {
			unsigned int cs;

			if (of_property_read_u32(np, "reg", &cs)) {
				dev_err(dev, "couldn't determine chip select\n");
				ret = -ENXIO;
				goto probe_failed;
			}
			if (cs > dw_spi->supported_cs) {
				dev_err(dev, "chip select %d out of range (%d supported)\n",
						cs, dw_spi->supported_cs);
				ret = -ENXIO;
				goto probe_failed;
			}
			f_pdata = &dw_spi->f_pdata[cs];

			ret = dw_spi_setup_flash(dev, f_pdata, np);
			if (ret)
				goto probe_failed;
		}
	}

	if (f_pdata != NULL)
		dw_nor = &f_pdata->nor;
	dev_info(dev, "Synopsys Octal SPI NOR flash driver\n");
	return 0;

probe_failed:
	dev_err(dev, "Synopsys Octal SPI NOR flash driver probe failed\n");
	return ret;
}

void enter_xip(void)
{
	u32 num;
	u8 data;
	struct dw_spi_nor *dw_spi = NULL;

	if (dw_nor == NULL)
		return;

	dw_spi = dw_nor->priv;

	/* backup registers */
	for (num = 0; num < ARRAY_SIZE(regs_backup); num++)
		regs_backup[num].val = dw_readl(dw_spi, regs_reset[num].addr);

	/* Send reset enable */
	dw_spi_write_reg(dw_nor, SPINOR_OP_RSTEN, NULL, 0, false);
	/* send reset*/
	dw_spi_write_reg(dw_nor, SPINOR_OP_RST, NULL, 0, false);

	//should 4 byte address be made non volatile ?
	dw_spi_read_reg(dw_nor, SPINOR_OP_RDBR, &data, sizeof(data));
	if ((data & RDBR_EXTADD) != RDBR_EXTADD) {
		uint64_t start;

		/* write  --> 4 bytes access */
		data = RDBR_EXTADD;
		dw_spi_write_reg(
			dw_nor, SPINOR_OP_WRBRNV, &data, sizeof(data), true);
		/* wait write completion*/
		start = get_time_ns();
		do {
			dw_spi_read_reg(
				dw_nor, SPINOR_OP_RDSR, &data, sizeof(data));
		} while (!is_timeout(start, DEFAULT_READY_WAIT)
				     && (data & SR_WIP));
	}

	//should 8 wait cycle be made non volatile ?
	dw_spi_read_reg(dw_nor, SPINOR_OP_RDRP, &data, sizeof(data));
	if ((data & RD_PARAM_DUMMYCYCLE_MASK) != 0x00) {
		uint64_t start;

		/* write READ register --> default (0) --> 8 wait cycle */
		data = RD_PARAM_DFLT_DUMMY_CYCLE; /* 8 bits dummy cycle */
		dw_spi_write_reg(dw_nor, SPINOR_OP_SRPNV, &data
				, sizeof(data), true);
		/* wait write completion*/
		start = get_time_ns();
		do {
			dw_spi_read_reg(dw_nor, SPINOR_OP_RDSR
					, &data, sizeof(data));
		} while (!is_timeout(start, DEFAULT_READY_WAIT)
				     && (data & SR_WIP));
	}

	/* Send reset enable */
	dw_spi_write_reg(dw_nor, SPINOR_OP_RSTEN, NULL, 0, false);
	/* send reset*/
	dw_spi_write_reg(dw_nor, SPINOR_OP_RST, NULL, 0, false);

	/*restore boot SSI state*/
	dw_spi_enable_chip(dw_spi, 0);
	for (num = 0; num < ARRAY_SIZE(regs_reset); num++)
		dw_writel(dw_spi, regs_reset[num].addr, regs_reset[num].val);

	dw_spi_enable_chip(dw_spi, 1);
}
EXPORT_SYMBOL(enter_xip)

void exit_xip(void)
{
	u32 num;
	struct dw_spi_nor *dw_spi = dw_nor->priv;

	/* restore initial register state */
	dw_spi_enable_chip(dw_spi, 0);
	for (num = 0; num < ARRAY_SIZE(regs_backup); num++)
		regs_backup[num].val = dw_readl(dw_spi, regs_reset[num].addr);

	dw_spi_enable_chip(dw_spi, 1);

	/* Send reset enable */
	dw_spi_write_reg(dw_nor, SPINOR_OP_RSTEN, NULL, 0, false);
	/* send reset*/
	dw_spi_write_reg(dw_nor, SPINOR_OP_RST, NULL, 0, false);
}
EXPORT_SYMBOL(exit_xip)

static __maybe_unused struct of_device_id dw_spi_dt_ids[] = {
	{.compatible = "snps,ospi-nor",},
	{ /* sentinel */ }
};

static struct driver_d dw_spi_driver = {
	.name           = DRIVER_NAME,
	.probe          = dw_spi_probe,
	.of_compatible  = DRV_OF_COMPAT(dw_spi_dt_ids),
};
device_platform_driver(dw_spi_driver);
