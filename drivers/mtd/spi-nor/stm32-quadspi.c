/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_data/dma-stm32.h>
#include <linux/mutex.h>

#define	DRV_NAME			"stm32-quadspi"

#define QSPI_CR_EN			(1 << 0)
#define QSPI_CR_ABORT			(1 << 1)
#define QSPI_CR_DMAEN			(1 << 2)
#define QSPI_CR_TCEN			(1 << 3)
#define QSPI_CR_SSHIFT			(1 << 4)
#define QSPI_CR_DFM			(1 << 6)
#define QSPI_CR_FSEL			(1 << 7)
#define QSPI_CR_TEIE			(1 << 16)
#define QSPI_CR_TCIE			(1 << 17)
#define QSPI_CR_FTIE			(1 << 18)
#define QSPI_CR_SMIE			(1 << 19)
#define QSPI_CR_TOIE			(1 << 20)
#define QSPI_CR_APMS			(1 << 22)
#define QSPI_CR_PMM			(1 << 23)
#define QSPI_CR_FTHRES(x)		(((x) & 0x1F) << 8)
#define QSPI_CR_PRESCALER(x)		(((x) & 0xFF) << 24)

#define QSPI_DCR_CK_MODE		(1 << 0)
#define QSPI_DCR_CSHT(x)		(((x) & 0x7) << 8)
#define QSPI_DCR_FSIZE(x)		(((x) & 0x1F) << 16)

#define QSPI_SR_TEF			(1 << 0)
#define QSPI_SR_TCF			(1 << 1)
#define QSPI_SR_FTF			(1 << 2)
#define QSPI_SR_SMF			(1 << 3)
#define QSPI_SR_TOF			(1 << 4)
#define QSPI_SR_BUSY			(1 << 5)
#define QSPI_SR_FLEVEL_MASK		(0x3F)
#define QSPI_SR_FLEVEL(x)		(((x) & QSPI_SR_FLEVEL_MASK) << 8)
#define QSPI_SR_FTF_MASK		((QSPI_SR_FTF) | (QSPI_SR_FLEVEL(QSPI_SR_FLEVEL_MASK)))

#define QSPI_FCR_CTEF			(1 << 0)
#define QSPI_FCR_CTCF			(1 << 1)
#define QSPI_FCR_CSMF			(1 << 3)
#define QSPI_FCR_CTOF			(1 << 4)

#define QSPI_CCR_INSTRUCTION(x)		(((x) & 0xFF) << 0)
#define QSPI_CCR_IMODE(x)		(((x) & 0x3) << 8)
#define QSPI_CCR_IMODE_NONE		(QSPI_CCR_IMODE(0))
#define QSPI_CCR_IMODE_SINGLE_LINE	(QSPI_CCR_IMODE(1))
#define QSPI_CCR_IMODE_TWO_LINES	(QSPI_CCR_IMODE(2))
#define QSPI_CCR_IMODE_FOUR_LINES	(QSPI_CCR_IMODE(3))
#define QSPI_CCR_ADMODE(x)		(((x) & 0x3) << 10)
#define QSPI_CCR_ADMODE_NONE		(QSPI_CCR_ADMODE(0))
#define QSPI_CCR_ADMODE_SINGLE_LINE	(QSPI_CCR_ADMODE(1))
#define QSPI_CCR_ADMODE_TWO_LINES	(QSPI_CCR_ADMODE(2))
#define QSPI_CCR_ADMODE_FOUR_LINES	(QSPI_CCR_ADMODE(3))
#define QSPI_CCR_ADSIZE(x)		(((x) & 0x3) << 12)
#define QSPI_CCR_ADSIZE_ONE_BYTE	(QSPI_CCR_ADSIZE(0))
#define QSPI_CCR_ADSIZE_TWO_BYTES	(QSPI_CCR_ADSIZE(1))
#define QSPI_CCR_ADSIZE_THREE_BYTES	(QSPI_CCR_ADSIZE(2))
#define QSPI_CCR_ADSIZE_FOUR_BYTES	(QSPI_CCR_ADSIZE(3))
#define QSPI_CCR_ABMODE(x)		(((x) & 0x3) << 14)
#define QSPI_CCR_ABSIZE(x)		(((x) & 0x3) << 16)
#define QSPI_CCR_DCYC(x)		(((x) & 0x1F) << 18)
#define QSPI_CCR_DMODE(x)		(((x) & 0x3) << 24)
#define QSPI_CCR_DMODE_NONE		(QSPI_CCR_DMODE(0))
#define QSPI_CCR_DMODE_SINGLE_LINE	(QSPI_CCR_DMODE(1))
#define QSPI_CCR_DMODE_TWO_LINES	(QSPI_CCR_DMODE(2))
#define QSPI_CCR_DMODE_FOUR_LINES	(QSPI_CCR_DMODE(3))
#define QSPI_CCR_FMODE(x)		(((x) & 0x3) << 26)
#define QSPI_CCR_FMODE_INDIRECT_WRITE	(QSPI_CCR_FMODE(0))
#define QSPI_CCR_FMODE_INDIRECT_READ	(QSPI_CCR_FMODE(1))
#define QSPI_CCR_FMODE_AUTO_POLL	(QSPI_CCR_FMODE(2))
#define QSPI_CCR_FMODE_MEMORY_MAP	(QSPI_CCR_FMODE(3))
#define QSPI_CCR_SIOO			(1 << 28)
#define QSPI_CCR_DHHC			(1 << 30)
#define QSPI_CCR_DDRM			(1 << 31)

#define STM32_QSPI_BASE			(STM32_AHB3PERIPH_BASE + 0x1000)

#define QSPI_TIMEOUT_MS			1000

#define STM32_DUMMY_TABLE_MAX_SIZE	16

struct stm32_qspi_regs {
	u32	cr;
	u32	dcr;
	u32	sr;
	u32	fcr;
	u32	dlr;
	u32	ccr;
	u32	ar;
	u32	abr;
	u32	dr;
	u32	psmkr;
	u32	psmar;
	u32	pir;
	u32	lptr;
};

struct dummy_cycles_table {
	unsigned	freq;
	unsigned	dummy_cycles;
};

struct flash_info {
	const char			*name;
	size_t				device_size;
	size_t				write_size;
	size_t				erase_size;
	u32				program_cmd;
	struct dummy_cycles_table	dummy_cycles_table[STM32_DUMMY_TABLE_MAX_SIZE];
};

static const struct flash_info stm32_qspiflash_info[] = {
	{
		/* 2^26 = 64MiB */
		"mt25ql512abb", 1 << 26, 256, 64 * 1024, SPINOR_OP_FAST_PROG_4B,
		{
			{44000000,	2},
			{61000000,	3},
			{78000000,	4},
			{97000000,	5},
			{106000000,	6},
			{115000000,	7},
			{125000000,	8},
			{133000000,	9},
			{0, 0 /* sentinel */},
		},
	},
	{
		/* 2^24 = 16MiB */
		"n25q128a", 1 << 24, 256, 64 * 1024, SPINOR_OP_PP_4B,
		{
			{30000000,	3},
			{40000000,	4},
			{50000000,	5},
			{60000000,	6},
			{70000000,	7},
			{80000000,	8},
			{86000000,	9},
			{95000000,	10},
			{0, 0 /* sentinel */},
		},
	},
	{
		/* 2^25 = 32MiB */
		"mt25q256a", 1 << 25, 256, 64 * 1024, SPINOR_OP_PP_4B,
		{
			{30000000,	3},
			{40000000,	4},
			{50000000,	5},
			{60000000,	6},
			{70000000,	7},
			{80000000,	8},
			{86000000,	9},
			{95000000,	10},
			{0, 0 /* sentinel */},
		},
	},
};

struct stm32_qspi_priv {
	struct device		*dev;
	struct stm32_qspi_regs __iomem *regs;
	const struct flash_info *flash;
	u8			*mapped_area;
	struct clk		*clk;
	struct reset_control	*rst;
	unsigned long		freq;
	int			fast_read_dummy;
	struct mtd_info		mtd;
	struct completion	cmd_complete;
	bool			irq_failed;
	struct dma_chan		*dma_chan;
	struct dma_async_tx_descriptor *dma_desc;
	dma_addr_t		dma_buf;
	size_t			dma_len;
	atomic_t		wait_mask;
	bool			last_op_was_read;
	struct mutex		lock;
	bool			support_4bytes;
};

static int stm32_qspi_set_speed(struct stm32_qspi_priv *priv, unsigned long speed)
{
	unsigned long parent_rate = clk_get_rate(priv->clk);
	unsigned div = parent_rate / speed;
	unsigned long new_freq = parent_rate / div;

	if ((div > 256) || (div < 1)) {
		dev_err(priv->dev, "%s: unable to provide a rate %lu based of parent rate %lu\n",
			__func__, speed, parent_rate);
		return -1;
	}

	if (new_freq != speed) {
		dev_warn(priv->dev, "%s: set rate %lu instead of requested %lu due to integer division\n",
			 __func__, speed, new_freq);
		priv->freq = new_freq;
	}

	priv->regs->cr &= ~QSPI_CR_PRESCALER(0xFF);
	priv->regs->cr |= QSPI_CR_PRESCALER(div - 1);

	return 0;
}

static int stm32_qspi_wait_for_flag(struct stm32_qspi_priv *priv)
{
	if (!wait_for_completion_timeout(&priv->cmd_complete,
					 msecs_to_jiffies(QSPI_TIMEOUT_MS))) {
		u32 wait_mask = atomic_read(&priv->wait_mask);
		dev_err(priv->dev, "%s: timeout (sr 0x%x, wait_mask 0x%x)\n", __func__, priv->regs->sr, wait_mask);
		return -1;
	}

	if (priv->irq_failed) {
		u32 wait_mask = atomic_read(&priv->wait_mask);
		dev_err(priv->dev, "%s: failed (sr 0x%x, wait_mask 0x%x)\n", __func__, priv->regs->sr, wait_mask);
		return -1;
	} else {
		return 0;
	}
}

static int stm32_qspi_poll_for_status(struct stm32_qspi_priv *priv, u32 mask, int active)
{
	int timeout = QSPI_TIMEOUT_MS;
	u32 reg;
	int i;

	for (i = 0; i < timeout; ++i) {
		bool got;
		reg = readl(&priv->regs->sr);
		got = !!(reg & mask);

		if ((got && active) || (!got && !active))
			break;

		msleep_interruptible(1);
	}

	if (i == timeout) {
		dev_err(priv->dev, "%s: TIMEOUT: sr 0x%x, mask 0x%x, active %d\n", __func__, reg, mask, active);
		return -ETIMEDOUT;
	} else {
		return 0;
	}
}

static int stm32_qspi_wait_while_busy(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_poll_for_status(priv, QSPI_SR_BUSY, 0);
	if (err)
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_wait_for_fifo(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_poll_for_status(priv, QSPI_SR_FTF, 1);
	if (err)
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_wait_until_complete(struct stm32_qspi_priv *priv)
{
	int err;
	atomic_set(&priv->wait_mask, QSPI_SR_TCF);
	priv->regs->cr |= QSPI_CR_TCIE;
	err = stm32_qspi_wait_for_flag(priv);
	priv->regs->cr &= ~QSPI_CR_TCIE;
	if (err)
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);

	return err;
}

static int stm32_qspi_wait_until_match(struct stm32_qspi_priv *priv)
{
	int err;
	atomic_set(&priv->wait_mask, QSPI_SR_SMF);
	priv->regs->cr |= QSPI_CR_SMIE;
	err = stm32_qspi_wait_for_flag(priv);
	priv->regs->cr &= ~QSPI_CR_SMIE;
	if (err)
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_autopoll(struct stm32_qspi_priv *priv, u32 mask, u32 match)
{
	int err;

	writel(match, &priv->regs->psmar);
	writel(mask, &priv->regs->psmkr);
	writel(0x10, &priv->regs->pir);

	priv->regs->cr &= ~QSPI_CR_PMM;
	priv->regs->cr |= QSPI_CR_APMS;

	writel(1 - 1, &priv->regs->dlr);

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	init_completion(&priv->cmd_complete);
	writel(QSPI_CCR_FMODE_AUTO_POLL
	       | SPINOR_OP_RDSR
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_NONE
	       | QSPI_CCR_DMODE_SINGLE_LINE
	       | QSPI_CCR_DCYC(0),
	       &priv->regs->ccr);

	err = stm32_qspi_wait_until_match(priv);
	writel(0, &priv->regs->psmar);
	writel(0, &priv->regs->psmkr);
	dev_dbg(priv->dev, "%s: status 0x%x\n", __func__, readl(&priv->regs->dr));
	if (err)
		goto fail;

	priv->regs->cr &= ~(QSPI_CR_PMM | QSPI_CR_APMS);
	return 0;

fail:
	dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_abort(struct stm32_qspi_priv *priv)
{
	int err;

	priv->regs->cr |= QSPI_CR_TCIE;
	priv->regs->cr &= ~QSPI_CR_DMAEN;
	dmaengine_terminate_all(priv->dma_chan);

	init_completion(&priv->cmd_complete);
	priv->regs->cr |= QSPI_CR_ABORT;

	err = stm32_qspi_wait_until_complete(priv);
	priv->regs->cr |= QSPI_CR_DMAEN;
	if (err) {
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
		return err;
	}

	err = stm32_qspi_wait_while_busy(priv);
	if (err) {
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
		return err;
	}

	priv->regs->cr &= ~QSPI_CR_TCIE;

	return 0;
}

static int stm32_qspi_read_status_register(struct stm32_qspi_priv *priv, u32 *status)
{
	u8 *dr = (u8*)&priv->regs->dr;
	int err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	writel(1 - 1, &priv->regs->dlr);

	init_completion(&priv->cmd_complete);
	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | SPINOR_OP_RDSR
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_NONE
	       | QSPI_CCR_DMODE_SINGLE_LINE
	       | QSPI_CCR_DCYC(0),
	       &priv->regs->ccr);

	writel(priv->regs->ar, &priv->regs->ar);

	priv->regs->ccr &= ~(QSPI_CCR_FMODE(3));
	priv->regs->ccr |= QSPI_CCR_FMODE_INDIRECT_READ;

	/* Initiate the transfer by writing the address once more */
	writel(priv->regs->ar, &priv->regs->ar);

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		goto fail;

	err = stm32_qspi_wait_for_fifo(priv);
	if (err)
		goto fail;

	*status = *dr;

	return 0;

fail:
	dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_wait_while_writing(struct stm32_qspi_priv *priv)
{
	bool failed = false;
	u32 status;
	int err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	err = stm32_qspi_autopoll(priv, SR_WIP, 0);
	if (err)
		goto fail;

	err = stm32_qspi_read_status_register(priv, &status);
	if (err)
		goto fail;

	if (status & SR_WIP) {
		dev_err(priv->dev, "%s: still write in progress: status 0x%x\n", __func__, status);
		failed = true;
	}

	if (status & SR_WEL) {
		dev_err(priv->dev, "%s: write is still enabled: status 0x%x\n", __func__, status);
		failed = true;
	}

	if (failed) {
		err = -1;
		goto fail;
	}

	return 0;

fail:
	dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_write_enable(struct stm32_qspi_priv *priv)
{
	int err;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	init_completion(&priv->cmd_complete);
	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | SPINOR_OP_WREN
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_NONE
	       | QSPI_CCR_DMODE_NONE
	       | QSPI_CCR_DCYC(0),
	       &priv->regs->ccr);

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		goto fail;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	err = stm32_qspi_autopoll(priv, SR_WEL, SR_WEL);
	if (err)
		goto fail;

	return 0;

fail:
	dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_switch_to_memory_mapped(struct stm32_qspi_priv *priv)
{
	int err;

	priv->regs->cr |= QSPI_CR_DMAEN;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	writel(QSPI_CCR_FMODE_MEMORY_MAP
	       | (priv->support_4bytes
		  ? SPINOR_OP_FAST_READ_4B
		  : SPINOR_OP_FAST_READ)
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_FOUR_LINES
	       | (priv->support_4bytes
		  ? QSPI_CCR_ADSIZE_FOUR_BYTES
		  : QSPI_CCR_ADSIZE_THREE_BYTES)
	       | QSPI_CCR_DMODE_FOUR_LINES
	       | QSPI_CCR_DCYC(priv->fast_read_dummy),
	       &priv->regs->ccr);

	return 0;

fail:
	dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_erase_block(struct stm32_qspi_priv *priv, u32 address)
{
	int err;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	err = stm32_qspi_write_enable(priv);
	if (err)
		goto fail;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	init_completion(&priv->cmd_complete);
	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | SPINOR_OP_SE
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_SINGLE_LINE
	       | (priv->support_4bytes
		  ? QSPI_CCR_ADSIZE_FOUR_BYTES
		  : QSPI_CCR_ADSIZE_THREE_BYTES)
	       | QSPI_CCR_DMODE_NONE
	       | QSPI_CCR_DCYC(0),
	       &priv->regs->ccr);

	writel(address, &priv->regs->ar);

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		goto fail;

	err = stm32_qspi_wait_while_writing(priv);
	if (err)
		goto fail;

	return 0;

fail:
	dev_err(priv->dev, "%s: failed (addr 0x%x): %d\n", __func__, address, err);
	return err;
}

static void stm32_qspi_dma_callback(void *param)
{
	struct stm32_qspi_priv *priv = param;
	struct dma_chan *chan = priv->dma_chan;

	dma_unmap_single(chan->device->dev, priv->dma_buf, priv->dma_len, DMA_TO_DEVICE);
}

static int stm32_qspi_write_page(struct stm32_qspi_priv *priv, u32 address, const u8 *buf, size_t size)
{
	int err;
	struct dma_chan *chan = priv->dma_chan;
	struct dma_slave_config config;
	dma_addr_t buf_dma;
	/* buf is a const u8* where we need a void * for the dma mapping */
	void *nonconst_buf = (void *)buf;

	if (size > priv->flash->write_size) {
		dev_err(priv->dev, "%s: size %u >  max write size %u\n",
			__func__, size, priv->flash->write_size);
		return -EINVAL;
	}

	priv->regs->cr &= ~QSPI_CR_DMAEN;
	dmaengine_terminate_all(priv->dma_chan);

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr = (dma_addr_t)&priv->regs->dr;
	config.dst_maxburst = 1;
	err = dmaengine_slave_config(chan, &config);
	if (err)
		goto fail;

	buf_dma = dma_map_single(chan->device->dev, nonconst_buf, size, DMA_TO_DEVICE);
	err = dma_mapping_error(chan->device->dev, buf_dma);
	if (err)
		goto fail;
	dma_sync_single_for_device(priv->dev, buf_dma, size, DMA_TO_DEVICE);
	priv->dma_desc = dmaengine_prep_slave_single(chan, buf_dma, size,
						     DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);

	if (!priv->dma_desc) {
		dev_err(priv->dev, "%s[%d]: internal error\n", __func__, __LINE__);
		return -1;
	}

	priv->dma_desc->callback = stm32_qspi_dma_callback;
	priv->dma_desc->callback_param = priv;
	priv->dma_buf = buf_dma;
	priv->dma_len = size;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	err = stm32_qspi_write_enable(priv);
	if (err)
		goto fail;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	init_completion(&priv->cmd_complete);

	writel(size - 1, &priv->regs->dlr);

	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | priv->flash->program_cmd
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_FOUR_LINES
	       | (priv->support_4bytes
		  ? QSPI_CCR_ADSIZE_FOUR_BYTES
		  : QSPI_CCR_ADSIZE_THREE_BYTES)
	       | QSPI_CCR_DMODE_FOUR_LINES
	       | QSPI_CCR_DCYC(0),
	       &priv->regs->ccr);

	writel(address, &priv->regs->ar);

	priv->regs->cr |= QSPI_CR_DMAEN;
	err = dma_submit_error(dmaengine_submit(priv->dma_desc));
	if (err)
		goto fail;

	dma_async_issue_pending(chan);

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		goto fail;

	err = stm32_qspi_wait_while_writing(priv);
	if (err)
		goto fail;

	return 0;
fail:
	dev_err(priv->dev, "%s: failed (addr 0x%x, len 0x%x): %d\n", __func__, address, (u32)size, err);
	return err;
}

static int stm32_qspi_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
				size_t *retlen, const u_char *buf)
{
	struct stm32_qspi_priv *priv = mtd->priv;
	size_t write_size = priv->flash->write_size;
	size_t current_size;

	mutex_lock(&priv->lock);

	*retlen = 0;

	if (unlikely((to + len) > priv->flash->device_size)) {
		dev_err(priv->dev, "%s: Write past end of device\n", __func__);
		mutex_unlock(&priv->lock);
		return -EINVAL;
	}

	if (priv->last_op_was_read)
	{
		int err = stm32_qspi_abort(priv);
		priv->last_op_was_read = false;
		if (err) {
			mutex_unlock(&priv->lock);
			return -EIO;
		}
	}

	current_size =
		(((to & (write_size - 1)) + len) > write_size)
		? (write_size - (to & (write_size - 1)))
		: len;

	while (len) {
		int err = stm32_qspi_write_page(priv, to, buf, current_size);
		if (err) {
			dev_err(priv->dev, "%s: write failed (addr 0x%x, len 0x%x): %d\n", __func__, (u32)to, (u32)len, err);
			mutex_unlock(&priv->lock);
			return err;
		}

		*retlen	+= current_size;
		len	-= current_size;
		to	+= current_size;
		buf	+= current_size;
		current_size = (len > write_size) ? write_size : len;
	}

	mutex_unlock(&priv->lock);
	return 0;
}

static int stm32_qspi_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct stm32_qspi_priv *priv = mtd->priv;
	size_t erase_size = priv->flash->erase_size;
	unsigned long address = instr->addr;
	unsigned long len = instr->len;
	int err;

	instr->state = MTD_ERASE_FAILED;

	if (unlikely((address + len) > priv->flash->device_size)) {
		dev_err(priv->dev, "%s: Erase past end of device\n", __func__);
		return -EINVAL;
	}

	if (len & (erase_size - 1)) {
		dev_err(priv->dev, "%s: non-block erase: len %lu\n", __func__, len);
		return -EINVAL;
	}

	if (address & (erase_size - 1)) {
		dev_err(priv->dev, "%s: non-aligned erase: addr 0x%x\n", __func__, (u32)address);
		return -EINVAL;
	}

	mutex_lock(&priv->lock);

	if (priv->last_op_was_read)
	{
		priv->last_op_was_read = false;
		err = stm32_qspi_abort(priv);
		if (err)
			goto fail;
	}

	instr->state = MTD_ERASING;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	while (len) {
		err = stm32_qspi_erase_block(priv, address);
		if (err)
			goto fail;

		address	+= erase_size;
		len	-= erase_size;
	}

	instr->state = MTD_ERASE_DONE;

	mutex_unlock(&priv->lock);

	mtd_erase_callback(instr);
	return 0;

fail:
	dev_err(priv->dev, "%s: failed (addr 0x%x, len 0x%x): %d\n", __func__, (u32)address, (u32)len, err);
	mutex_unlock(&priv->lock);
	return err;
}

static int stm32_qspi_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
			       size_t *retlen, u_char *buf)
{
	struct stm32_qspi_priv *priv = mtd->priv;
	u8 *addr = priv->mapped_area + from;
	int err;

	mutex_lock(&priv->lock);

	if (!priv->last_op_was_read) {
		priv->last_op_was_read = true;
		err = stm32_qspi_switch_to_memory_mapped(priv);
		if (err)
			goto fail;
	}

	memcpy(buf, addr, len);
	*retlen = len;

	mutex_unlock(&priv->lock);
	return 0;

fail:
	dev_err(priv->dev, "%s: failed (addr 0x%x, len 0x%x): %d\n", __func__, (u32)addr, len, err);
	mutex_unlock(&priv->lock);
	return err;
}

static irqreturn_t stm32_qspi_irq_thread(int irq, void *data)
{
	struct stm32_qspi_priv *priv = data;
	/*
	 * FIFO interrupts can't be disabled, but they are not interesting,
	 * as well as BUSY status.
	 */
	u32 sr_mask = ~(QSPI_SR_FTF_MASK | QSPI_SR_BUSY);
	u32 sr = priv->regs->sr & sr_mask;
	u32 wait_mask = atomic_read(&priv->wait_mask);
	bool unexpected = (sr & (~wait_mask));

	priv->irq_failed = false;

	if (!sr)
		return IRQ_HANDLED;

	if (unexpected || ((sr & wait_mask) != wait_mask)) {
		dev_err(priv->dev, "%s: Unexpected interrupt: got sr 0x%x, but waited for 0x%x!\n", __func__, sr, wait_mask);
		msleep_interruptible(1);
		return IRQ_HANDLED;
	}

	if (sr & QSPI_SR_TCF) {
		priv->regs->fcr |= QSPI_FCR_CTCF;
	}

	if (sr & QSPI_SR_SMF) {
		priv->regs->fcr |= QSPI_FCR_CSMF;
	}

	if (sr & QSPI_SR_TEF) {
		dev_err(priv->dev, "%s: Transfer failed\n", __func__);
		priv->regs->fcr |= QSPI_FCR_CTEF;
		priv->irq_failed = true;
	}

	if (sr & QSPI_SR_TOF) {
		dev_err(priv->dev, "%s: Transfer timeout\n", __func__);
		priv->regs->fcr |= QSPI_FCR_CTOF;
		priv->irq_failed = true;
	}

	if (!unexpected) {
		complete(&priv->cmd_complete);
		atomic_set(&priv->wait_mask, 0);
	}

	return IRQ_HANDLED;
}

static int stm32_qspi_enter_4_bytes_mode(struct stm32_qspi_priv *priv)
{
	int err;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	err = stm32_qspi_write_enable(priv);
	if (err)
		goto fail;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		goto fail;

	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | SPINOR_OP_EN4B
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_NONE
	       | QSPI_CCR_DMODE_NONE
	       | QSPI_CCR_DCYC(0),
		&priv->regs->ccr);

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		goto fail;

	return 0;
fail:
	dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_probe(struct platform_device *pdev)
{
	const struct dummy_cycles_table *dummy_table = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct stm32_qspi_priv *priv = NULL;
	u32 tmp = 0, flash_size_off = 0;
	struct device *dev = &pdev->dev;
	const char *flash_name;
	struct mtd_info *mtd;
	struct resource *res;
	int i, err, irq;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "Failed to initialize Quad-SPI controller\n");
		return -ENOMEM;
	}

	priv->dev	= dev;
	mtd		= &priv->mtd;
	mtd->priv	= priv;
	mtd->owner	= THIS_MODULE;
	mtd->dev.parent	= dev;
	mtd->name	= DRV_NAME;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(dev, res);
	err = IS_ERR_OR_NULL(priv->regs) ? PTR_ERR(priv->regs) : 0;
	if (err) {
		dev_err(dev, "Failed to map registers: %d\n", err);
		return err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->mapped_area = devm_ioremap_resource(dev, res);
	err = IS_ERR_OR_NULL(priv->mapped_area) ? PTR_ERR(priv->mapped_area) : 0;
	if (err) {
		dev_err(dev, "Failed to get mapped area: %d\n", err);
		return err;
	}

	priv->clk = devm_clk_get(dev, NULL);
	err = IS_ERR_OR_NULL(priv->clk) ? PTR_ERR(priv->clk) : 0;
	if (err) {
		dev_err(dev, "Failed to get clock: %d\n", err);
		return err;
	}

	priv->rst = devm_reset_control_get(dev, NULL);
	err = IS_ERR_OR_NULL(priv->rst) ? PTR_ERR(priv->rst) : 0;
	if (err) {
		dev_err(dev, "Failed to get reset controller: %d\n", err);
		return err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}

	priv->dma_chan = dma_request_slave_channel(dev, "tx");
	if (IS_ERR_OR_NULL(priv->dma_chan)) {
		err = PTR_ERR(priv->dma_chan);
		dev_err(dev, "Failed to request DMA: %d\n", err);
		return err;
	}

	reset_control_assert(priv->rst);
	reset_control_deassert(priv->rst);
	clk_prepare_enable(priv->clk);
	priv->clk = devm_clk_get(dev, NULL);

	priv->regs->cr &= ~QSPI_CR_EN;

	err = of_property_read_u32(np, "freq", &tmp);
	if (err < 0) {
		dev_err(dev, "failed to get frequency: %d\n", err);
		return err;
	}
	priv->freq = tmp;

	if (of_property_read_string(np, "flash", &flash_name)) {
		dev_err(dev, "no flash specified\n");
		return -EINVAL;
	}
	for (i = 0, priv->flash = NULL; i < ARRAY_SIZE(stm32_qspiflash_info); i++) {
		const char *name = stm32_qspiflash_info[i].name;
		if (!strncmp(flash_name, name, strlen(name + 1))) {
			priv->flash = &stm32_qspiflash_info[i];
			break;
		}
	}
	if (!priv->flash) {
		dev_err(dev, "flash '%s' is not supported\n", flash_name);
		return -EINVAL;
	}

	writel(QSPI_CR_APMS
	       | QSPI_CR_FTHRES(3)
	       | QSPI_CR_SSHIFT
	       | QSPI_CR_DMAEN,
	       &priv->regs->cr);

	stm32_qspi_set_speed(priv, priv->freq);

	dummy_table = priv->flash->dummy_cycles_table;
	for (i = 0;
	     (i < STM32_DUMMY_TABLE_MAX_SIZE)
		     && dummy_table[i].freq;
	     i++) {
		if (priv->freq > dummy_table[i].freq)
			priv->fast_read_dummy = dummy_table[i].dummy_cycles;
		else
			break;
	}
	flash_size_off = fls(priv->flash->device_size) - 1;

	mutex_init(&priv->lock);
	mtd->type	= MTD_NORFLASH;
	mtd->size	= priv->flash->device_size;
	mtd->writebufsize = priv->flash->write_size;
	mtd->writesize	= 1;
	mtd->erasesize	= priv->flash->erase_size;
	mtd->flags	= MTD_CAP_NORFLASH;
	mtd->_erase	= stm32_qspi_mtd_erase;
	mtd->_read	= stm32_qspi_mtd_read;
	mtd->_write	= stm32_qspi_mtd_write;

	priv->regs->dcr |= QSPI_DCR_CK_MODE;

	/* Number of bytes in Flash memory = 2^(FSIZE+1) */
	priv->regs->dcr &= ~(QSPI_DCR_FSIZE(0x1F));
	priv->regs->dcr |= QSPI_DCR_FSIZE(flash_size_off - 1);

	priv->regs->cr |= QSPI_CR_EN;
	err = stm32_qspi_wait_while_busy(priv);
	if (err) {
		dev_err(dev, "%s: Failed to enable QSPI: %d\n", __func__, err);
		return err;
	}

	platform_set_drvdata(pdev, priv);

	err = devm_request_threaded_irq(dev, irq,
					NULL, stm32_qspi_irq_thread,
					IRQF_ONESHOT,
					DRV_NAME, priv);
	if (err) {
		dev_err(dev, "%s: Failed to request irq: %d\n", __func__, err);
		return err;
	}

	priv->regs->cr |= QSPI_CR_TOIE
		| QSPI_CR_TEIE;

	/* 4 bytes addressing is only supported for flashes bigger than 16MiB */
	priv->support_4bytes = !!(mtd->size > (1 << 24));

	if (priv->support_4bytes) {
		err = stm32_qspi_enter_4_bytes_mode(priv);
		if (err) {
			dev_err(dev, "%s: Failed to switch to 4 byte mode addressing: %d\n", __func__, err);
			return err;
		}
	}

	dev_info(dev, "QSPI:  %d MB mapped at %p\n",
		 1 << (flash_size_off - 20), priv->mapped_area);

	err = mtd_device_parse_register(mtd, NULL,
					&(struct mtd_part_parser_data){
						.of_node = np,
							},
					NULL, 0);
	if (err) {
		dev_err(dev, "%s: Failed to register mtd: %d\n", __func__, err);
		return err;
	}

	return 0;
}

static const struct of_device_id stm32_qspi_dt_ids[] = {
	{ .compatible = "st,stm32-qspi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, stm32_qspi_dt_ids);

static struct platform_driver stm32_qspi_driver = {
	.driver = {
		.name	= DRV_NAME,
		.bus	= &platform_bus_type,
		.of_match_table = stm32_qspi_dt_ids,
	},
	.probe          = stm32_qspi_probe,
};
module_platform_driver(stm32_qspi_driver);

MODULE_AUTHOR("Sergei Miroshnichenko <sergeimir@emcraft.com>");
MODULE_DESCRIPTION("STM32 QUAD-SPI");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, stm32_qspi_dt_ids);
