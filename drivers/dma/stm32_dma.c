/*
 * Driver for the StMicro STM32 DMA Controller
 *
 * Copyright (C) 2015
 * Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/dma-stm32.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#include "dmaengine.h"

/******************************************************************************
 *
 * By linux 'dma channels' we mean 'STM32 DMA streams' through this driver.
 * 'STM32 DMA channel' is just a configurable option of the 'STM32 DMA stream',
 * which allows to select the dedicated peripheral to xchange with.
 *
 ******************************************************************************/

/******************************************************************************
 * Constants
 ******************************************************************************/
/*
 * ISR/IFCR bits
 */
#define STM32_INT_TCIF			(1 << 5)
#define STM32_INT_HTIF			(1 << 4)
#define STM32_INT_FEIF			(1 << 0)
#define STM32_INT_MSK			(STM32_INT_TCIF | STM32_INT_HTIF |     \
					 STM32_INT_FEIF)

/*
 * CR register bits
 */
#define STM32_DMA_CR_CHSEL_OFS		25
#define STM32_DMA_CR_MBURST_OFS		23
#define STM32_DMA_CR_PBURST_OFS		21
#define STM32_DMA_CR_CT			(1 << 19)
#define STM32_DMA_CR_DBM		(1 << 18)
#define STM32_DMA_CR_PL_HI		(3 << 16)
#define STM32_DMA_CR_MSIZE_OFS		13
#define STM32_DMA_CR_PSIZE_OFS		11
#define STM32_DMA_CR_PSIZE(x)		(((x) >> STM32_DMA_CR_PSIZE_OFS) & 0x3)
#define STM32_DMA_CR_MINC		(1 << 10)
#define STM32_DMA_CR_PINC		(1 << 9)
#define STM32_DMA_CR_CIRC		(1 << 8)
#define STM32_DMA_CR_DIR_P2M		(0 << 6)
#define STM32_DMA_CR_DIR_M2P		(1 << 6)
#define STM32_DMA_CR_PFCTRL		(1 << 5)
#define STM32_DMA_CR_TCIE		(1 << 4)
#define STM32_DMA_CR_EN			(1 << 0)

/*
 * FCR register bits
 */
#define STM32_DMA_FCR_FEIE		(1 << 7)
#define STM32_DMA_FCR_DMDIS		(1 << 2)
#define STM32_DMA_FCR_FTH_OFS		0

/******************************************************************************
 * C-types
 ******************************************************************************/
/**
 * struct stm32_dma_ch_regs - DMA channel register map
 * @cr: configuration
 * @ndtr: number of data
 * @par: peripheral address
 * @m0ar: memory 0 address
 * @m1ar: memory 1 address
 * @fcr: FIFO control
 */
struct stm32_dma_ch_regs {
	u32	cr;
	u32	ndtr;
	u32	par;
	u32	m0ar;
	u32	m1ar;
	u32	fcr;
};

/**
 * struct stm32_dma_regs - DMA register map
 * @lisr: low interrupt status
 * @hisr: high interrupt status
 * @lifcr: low interrupt flag clear
 * @hifcr: high interrupt flag clear
 * @stream: streams registers
 */
struct stm32_dma_regs {
	u32	lisr;
	u32	hisr;
	u32	lifcr;
	u32	hifcr;

	struct stm32_dma_ch_regs stream[8];
};

/**
 * struct stm32_dma_chan - internal representation of STM32 DMA channel
 * @chan: common dmaengine channel object members
 * @dev: parent device
 * @idx: channel index
 * @regs: memory mapped register base
 * @isr_reg: ISR register for this channel
 * @ifcr_reg: IFCR register for this channel
 * @i_ofs: ISR/IFCR monitor bits offset
 * @i_val: last interrupt status for this channel
 * @tasklet: bottom half to finish transaction work
 * @dma_sconfig: configuration for slave transfers, passed via .device_config
 * @lock: serializes enqueue/dequeue operations
 * @txd: support for the async_tx api
 * @len: xfer length
 * @cr_msk: CR mask holding channel selection, etc.
 */
struct stm32_dma_chan {
	struct dma_chan			chan;
	struct stm32_dma		*dev;
	int				idx;

	struct stm32_dma_ch_regs __iomem *regs;	/* pointer to hw registers    */
	struct stm32_dma_ch_regs	prep;	/* prepared register values   */

	u32				cadr;	/* dblbuf cyclic xfer params */
	u32				cpos;
	u32				cpsz;
	u32				cfsz;

	u32 __iomem			*isr_reg;
	u32 __iomem			*ifcr_reg;
	u32				i_ofs;
	u32				i_val;

	struct tasklet_struct		tasklet;
	struct dma_slave_config		dma_sconfig;
	spinlock_t			lock;
	struct dma_async_tx_descriptor	txd;
	u32				len;

	u32				cr_msk;
};

/**
 * struct stm32_dma - internal representation of STM32 DMA Controller
 * @dev: common dmaengine dma_device object members
 * @regs: memory mapped register base
 * @clk: dma controller clock
 * @chan_num: number of channels
 * @ch: channels table to store stm32_dma_chan structures
 */
struct stm32_dma {
	struct dma_device		dev;
	struct stm32_dma_regs __iomem	*regs;
	struct clk			*clk;

	int				chan_num;
	/* Must be the last */
	struct stm32_dma_chan		chan[0];
};

/******************************************************************************
 * Different helpers
 ******************************************************************************/
static inline struct stm32_dma_chan *to_stm_dma_chan(struct dma_chan *dchan)
{
	return container_of(dchan, struct stm32_dma_chan, chan);
}

static inline struct stm32_dma *to_stm_dma(struct dma_device *ddev)
{
	return container_of(ddev, struct stm32_dma, dev);
}

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

/**
 * stm_chan_is_enabled - test if given channel is enabled
 * @stm_chan: channel we want to test status
 */
static inline int stm_chan_is_enabled(struct stm32_dma_chan *stm_chan)
{
	return !!(stm_chan->regs->cr & STM32_DMA_CR_EN);
}

/**
 * stm_chan_is_cyclic - test if given channel has cyclic property set
 * @stm_chan: channel we want to test status
 */
static inline int stm_chan_is_cyclic(struct stm32_dma_chan *stm_chan)
{
	return !!(stm_chan->regs->cr & STM32_DMA_CR_CIRC);
}

/******************************************************************************
 * TBD
 ******************************************************************************/
/**
 * stm_get_bytes_left - get the number of bytes residue
 * @chan: DMA channel
 */
static int stm_get_bytes_left(struct dma_chan *chan)
{
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);
	unsigned long		flags;
	int			bytes;

	spin_lock_irqsave(&stm_chan->lock, flags);
	bytes = stm_chan->regs->ndtr << STM32_DMA_CR_PSIZE(stm_chan->prep.cr);
	if (stm_chan->prep.cr & STM32_DMA_CR_DBM)
		bytes += stm_chan->cfsz - stm_chan->cpos - stm_chan->cpsz;
	spin_unlock_irqrestore(&stm_chan->lock, flags);

	return bytes;
}

/**
 * stm_submit - submit transaction to hw
 * @stm_chan: channel
 *
 * Called with stm_chan->lock held and bh disabled
 */
static void stm_submit(struct stm32_dma_chan *stm_chan)
{
	/*
	 * We may be called either to program a new transfer, or to update
	 * an already runing transfer in a double-buffering scheme
	 */
	if (stm_chan->prep.cr & STM32_DMA_CR_DBM) {
		/* Double buffering */
		if (!(stm_chan->regs->cr & STM32_DMA_CR_EN)) {
			/* Not running yet */
			stm_chan->regs->m0ar = stm_chan->cadr + stm_chan->cpos;
			stm_chan->cpos = (stm_chan->cpos + stm_chan->cpsz) %
					 stm_chan->cfsz;

			stm_chan->regs->m1ar = stm_chan->cadr + stm_chan->cpos;
			stm_chan->cpos = (stm_chan->cpos + stm_chan->cpsz) %
					 stm_chan->cfsz;
		} else if (stm_chan->i_val & STM32_INT_TCIF) {
			/* Running, and next transfer complete */
			if (stm_chan->regs->cr & STM32_DMA_CR_CT)
				stm_chan->regs->m0ar = stm_chan->cadr + stm_chan->cpos;
			else
				stm_chan->regs->m1ar = stm_chan->cadr + stm_chan->cpos;
			stm_chan->cpos = (stm_chan->cpos + stm_chan->cpsz) %
					 stm_chan->cfsz;
		}
	} else {
		/* Single buffering */
		stm_chan->regs->m0ar = stm_chan->prep.m0ar;
	}

	/*
	 * If DMA isn't running yet, then program the other fields, and run it
	 */
	if (!(stm_chan->regs->cr & STM32_DMA_CR_EN)) {
		stm_chan->regs->par  = stm_chan->prep.par;
		stm_chan->regs->ndtr = stm_chan->prep.ndtr;
		stm_chan->regs->fcr  = stm_chan->prep.fcr;
		stm_chan->regs->cr   = stm_chan->prep.cr | STM32_DMA_CR_EN;
	}
}


/**
 * stm_handle_once - at the end of a transaction, move forward
 * @stm_chan: channel where the transaction ended
 *
 * Called with stm_chan->lock held and bh disabled
 */
static void stm_handle_once(struct stm32_dma_chan *stm_chan)
{
	struct dma_async_tx_descriptor	*txd = &stm_chan->txd;
	dma_async_tx_callback		callback = txd->callback;
	void				*param = txd->callback_param;

	if (stm_chan_is_enabled(stm_chan))
		return;

	dma_cookie_complete(txd);
	dma_descriptor_unmap(txd);
	if (callback)
		callback(param);
	dma_run_dependencies(txd);
}

/**
 * stm_handle_cyclic - at the end of a period, run callback function
 * @stm_chan: channel used for cyclic operations
 *
 * Called with stm_chan->lock held and bh disabled
 */
static void stm_handle_cyclic(struct stm32_dma_chan *stm_chan)
{
	struct dma_async_tx_descriptor	*txd = &stm_chan->txd;
	dma_async_tx_callback		callback = txd->callback;
	void				*param = txd->callback_param;

	if (stm_chan_is_enabled(stm_chan))
		stm_submit(stm_chan);

	if (callback)
		callback(param);
}

/******************************************************************************
 * IRQ & Tasklet
 ******************************************************************************/
static void stm_tasklet(unsigned long data)
{
	struct stm32_dma_chan	*stm_chan = (struct stm32_dma_chan *)data;
	unsigned long		flags;

	spin_lock_irqsave(&stm_chan->lock, flags);

	if (stm_chan_is_cyclic(stm_chan))
		stm_handle_cyclic(stm_chan);
	else
		stm_handle_once(stm_chan);

	spin_unlock_irqrestore(&stm_chan->lock, flags);
}

static irqreturn_t stm_dma_interrupt(int irq, void *dev_id)
{
	struct stm32_dma_chan	*stm_chan = dev_id;
	u32			val;

	val = *stm_chan->isr_reg & (STM32_INT_MSK << stm_chan->i_ofs);
	if (!val)
		return IRQ_NONE;

	*stm_chan->ifcr_reg = val;
	stm_chan->i_val = val >> stm_chan->i_ofs;
	tasklet_schedule(&stm_chan->tasklet);

	return IRQ_HANDLED;
}

/******************************************************************************
 * DMA Engine API
 ******************************************************************************/
/**
 * stm_tx_submit - set the prepared descriptor to be executed by the engine
 * @tx: transaction descriptor
 */
static dma_cookie_t stm_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(tx->chan);
	unsigned long		flags;
	dma_cookie_t		cookie;

	spin_lock_irqsave(&stm_chan->lock, flags);

	cookie = dma_cookie_assign(tx);
	*stm_chan->ifcr_reg = STM32_INT_MSK << stm_chan->i_ofs;

	/*
	 * Set-up hw using values formed in stm_prep_xxx()
	 */
	stm_submit(stm_chan);

	spin_unlock_irqrestore(&stm_chan->lock, flags);

	return cookie;
}

/**
 * stm_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: DMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: tx descriptor status flags
 * @context: transaction context (ignored)
 */
static struct dma_async_tx_descriptor *
stm_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		  unsigned int sg_len, enum dma_transfer_direction direction,
		  unsigned long flags, void *context)
{
	static u32		empty_buf;

	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);
	struct stm32_dma_slave	*stm_slave = chan->private;
	struct dma_slave_config	*sconfig = &stm_chan->dma_sconfig;
	u32			len;

	dev_vdbg(chan2dev(chan), "prep_slave_sg (%d): %s f0x%lx\n", sg_len,
		 direction == DMA_MEM_TO_DEV ? "TO DEVICE" : "FROM DEVICE",
		 flags);

	if (unlikely(!stm_slave || !sg_len)) {
		dev_err(chan2dev(chan), "prep_slave_sg: sg length is zero!\n");
		return NULL;
	}
	if (unlikely(sg_len != 1)) {
		dev_err(chan2dev(chan), "prep_slave_sg: not supported sg length\n");
		return NULL;
	}
	if (!async_tx_test_ack(&stm_chan->txd)) {
		dev_err(chan2dev(chan), "prep_slave_sg: channel is busy\n");
		return NULL;
	}

	len = sg_dma_len(sgl);
	if (unlikely(!len)) {
		dev_err(chan2dev(chan),
			"prep_slave_sg: sg data length is zero\n");
		return NULL;
	}

	stm_chan->prep.cr = STM32_DMA_CR_PL_HI | stm_chan->cr_msk;
	if (sg_dma_address(sgl)) {
		stm_chan->prep.m0ar = sg_dma_address(sgl);
		stm_chan->prep.cr  |= STM32_DMA_CR_MINC;
	} else {
		stm_chan->prep.m0ar = (u32)&empty_buf;
	}

	if (sconfig->device_fc) {
		/*
		 * Flow controller is peripheral. This is the special
		 * case in STM32, and only SDIO supports this feature.
		 * Thus we hardcode SDIO requirements here:
		 * - 4-beats bursts
		 * - disabled direct mode
		 * - full FIFO threshold
		 * - ndtr is ignored (SDIO completes transfer)
		 */
		stm_chan->prep.cr |= STM32_DMA_CR_PFCTRL |
				     (1 << STM32_DMA_CR_MBURST_OFS) |
				     (1 << STM32_DMA_CR_PBURST_OFS);
		stm_chan->prep.fcr |= STM32_DMA_FCR_FEIE |
				      STM32_DMA_FCR_DMDIS |
				      (3 << STM32_DMA_FCR_FTH_OFS);
		stm_chan->prep.ndtr = 0;
	} else {
		/*
		 * Flow controller is DMA
		 */
		stm_chan->prep.cr |= STM32_DMA_CR_TCIE;
		stm_chan->prep.ndtr = len;
	}

	switch (direction) {
	case DMA_MEM_TO_DEV:
		stm_chan->prep.par = sconfig->dst_addr;
		stm_chan->prep.ndtr /= sconfig->dst_addr_width;
		stm_chan->prep.cr |= STM32_DMA_CR_DIR_M2P |
				     (__ffs(sconfig->src_addr_width) <<
				      STM32_DMA_CR_MSIZE_OFS) |
				     (__ffs(sconfig->dst_addr_width) <<
				      STM32_DMA_CR_PSIZE_OFS);
		break;
	case DMA_DEV_TO_MEM:
		stm_chan->prep.par = sconfig->src_addr;
		stm_chan->prep.ndtr /= sconfig->src_addr_width;
		stm_chan->prep.cr |= STM32_DMA_CR_DIR_P2M |
				     (__ffs(sconfig->src_addr_width) <<
				      STM32_DMA_CR_PSIZE_OFS) |
				     (__ffs(sconfig->dst_addr_width) <<
				      STM32_DMA_CR_MSIZE_OFS);
		break;
	default:
		return NULL;
	}

	if (stm_chan->prep.ndtr > STM32_DMA_NDT_MAX) {
		dev_err(chan2dev(chan),
			"prep_slave_sg: transfer 0x%x is too large\n",
			stm_chan->prep.ndtr);
                return NULL;
	}

	/* Client is in control of flags ack */
	stm_chan->txd.cookie = -EBUSY;
	stm_chan->txd.flags = flags;

	return &stm_chan->txd;
}

/**
 * stm_prep_dma_cyclic - prepare the cyclic DMA transfer
 * @chan: the DMA channel to prepare
 * @buf_addr: physical DMA address where the buffer starts
 * @buf_len: total number of bytes for the entire buffer
 * @period_len: number of bytes for each period
 * @direction: transfer direction, to or from device
 * @flags: tx descriptor status flags
 */
static struct dma_async_tx_descriptor *
stm_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
		    size_t period_len, enum dma_transfer_direction direction,
		    unsigned long flags)
{
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);
	struct stm32_dma_slave	*stm_slave = chan->private;
	struct dma_slave_config	*sconfig = &stm_chan->dma_sconfig;
	u32			periods = buf_len / period_len;

	dev_vdbg(chan2dev(chan), "prep_dma_cyclic: %s buf@0x%08x - %d (%d/%d)\n",
		 direction == DMA_MEM_TO_DEV ? "TO DEVICE" : "FROM DEVICE",
		 buf_addr, periods, buf_len, period_len);

	if (unlikely(!stm_slave || !buf_len || !period_len ||
		     (buf_len % period_len))) {
		dev_err(chan2dev(chan), "prep_dma_cyclic: bad params!\n");
		return NULL;
	}
	if (!async_tx_test_ack(&stm_chan->txd)) {
		dev_err(chan2dev(chan), "prep_dma_cyclic: channel is busy\n");
		return NULL;
	}
	if (unlikely(!is_slave_direction(direction))) {
		dev_err(chan2dev(chan), "prep_dma_cyclic: bad direction\n");
		return NULL;
	}

	stm_chan->prep.ndtr = period_len;
	stm_chan->prep.cr = STM32_DMA_CR_PL_HI | STM32_DMA_CR_MINC |
			    STM32_DMA_CR_TCIE  | STM32_DMA_CR_CIRC |
			    stm_chan->cr_msk;

	switch (direction) {
	case DMA_MEM_TO_DEV:
		stm_chan->prep.par = sconfig->dst_addr;
		stm_chan->prep.ndtr /= sconfig->dst_addr_width;
		if (!sconfig->src_addr_width)
			sconfig->src_addr_width = sconfig->dst_addr_width;
		stm_chan->prep.cr |= STM32_DMA_CR_DIR_M2P |
				     (__ffs(sconfig->src_addr_width) <<
				      STM32_DMA_CR_MSIZE_OFS) |
				     (__ffs(sconfig->dst_addr_width) <<
				      STM32_DMA_CR_PSIZE_OFS);
		break;
	case DMA_DEV_TO_MEM:
		stm_chan->prep.par = sconfig->src_addr;
		stm_chan->prep.ndtr /= sconfig->src_addr_width;
		if (!sconfig->dst_addr_width)
			sconfig->dst_addr_width = sconfig->src_addr_width;
		stm_chan->prep.cr |= STM32_DMA_CR_DIR_P2M |
				     (__ffs(sconfig->src_addr_width) <<
				      STM32_DMA_CR_PSIZE_OFS) |
				     (__ffs(sconfig->dst_addr_width) <<
				      STM32_DMA_CR_MSIZE_OFS);
		break;
	default:
		return NULL;
	}

	if (periods > 1) {
		stm_chan->prep.cr |= STM32_DMA_CR_DBM;
		stm_chan->cadr = buf_addr;
		stm_chan->cpos = 0;
		stm_chan->cpsz = period_len;
		stm_chan->cfsz = buf_len;
	} else {
		stm_chan->prep.m0ar = buf_addr;
	}

	/* Client is in control of flags ack */
	stm_chan->txd.cookie = -EBUSY;
	stm_chan->txd.flags = flags;

	return &stm_chan->txd;
}

static int stm_config(struct dma_chan *chan,
		      struct dma_slave_config *sconfig)
{
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);

	/* Check if chan is configured for slave transfers */
	if (!chan->private)
		return -EINVAL;

	memcpy(&stm_chan->dma_sconfig, sconfig, sizeof(*sconfig));

	return 0;
}

static int stm_terminate_all(struct dma_chan *chan)
{
	struct stm32_dma_chan		*stm_chan = to_stm_dma_chan(chan);
	struct dma_async_tx_descriptor	*txd = &stm_chan->txd;

	stm_chan->regs->cr &= ~STM32_DMA_CR_EN;

	async_tx_ack(txd);
	dma_descriptor_unmap(txd);
	dma_run_dependencies(txd);

	return 0;
}

/**
 * stm_tx_status - poll for transaction completion
 * @chan: DMA channel
 * @cookie: transaction identifier to check status of
 * @txstate: if not %NULL updated with transaction state
 *
 * If @txstate is passed in, upon return it reflect the driver
 * internal state and can be used with dma_async_is_complete() to check
 * the status of multiple cookies without re-checking hardware state.
 */
static enum dma_status stm_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	enum dma_status		ret;
	int			bytes = 0;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	/*
	 * There's no point calculating the residue if there's
	 * no txstate to store the value.
	 */
	if (!txstate)
		return DMA_ERROR;

	/*  Get number of bytes left in the active transactions */
	bytes = stm_get_bytes_left(chan);

	if (unlikely(bytes < 0)) {
		dev_err(chan2dev(chan), "get residual bytes error\n");
		return DMA_ERROR;
	} else {
		dma_set_residue(txstate, bytes);
	}

	return ret;
}

/**
 * stm_issue_pending - try to finish work
 * @chan: target DMA channel
 */
static void stm_issue_pending(struct dma_chan *chan)
{
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);

	/* Not needed for cyclic transfers */
	if (stm_chan_is_cyclic(stm_chan))
		return;

	stm_handle_once(stm_chan);
}

/**
 * stm_alloc_chan_resources - allocate resources for DMA channel
 * @chan: allocate resources for this channel
 */
static int stm_alloc_chan_resources(struct dma_chan *chan)
{
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);
	struct stm32_dma	*stm_dma = to_stm_dma(chan->device);
	struct stm32_dma_slave	*stm_slave;
	u32			cfg = 0;

	if (stm_chan_is_enabled(stm_chan)) {
		dev_err(chan2dev(chan), "DMA channel not idle ?\n");
		return -EIO;
	}

	stm_slave = chan->private;
	if (stm_slave) {
		/*
		 * We need controller-specific data to set up slave
		 * transfers.
		 */
		BUG_ON(!stm_slave->dma_dev ||
			stm_slave->chan_idx != stm_chan->idx ||
			stm_slave->dma_dev != stm_dma->dev.dev);

		/* if cfg configuration specified take it instead of default */
		if (stm_slave->cfg)
			cfg = stm_slave->cfg;
	}

	dma_cookie_init(chan);

	dma_async_tx_descriptor_init(&stm_chan->txd, chan);
	stm_chan->txd.flags = DMA_CTRL_ACK;
	stm_chan->txd.tx_submit = stm_tx_submit;
	stm_chan->txd.phys = 0;

	stm_chan->cr_msk = cfg;

	return 0;
}

/**
 * stm_free_chan_resources - free all channel resources
 * @chan: DMA channel
 */
static void stm_free_chan_resources(struct dma_chan *chan)
{
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);
	struct stm32_dma_slave	*stm_slave = chan->private;

	BUG_ON(stm_chan_is_enabled(stm_chan));
	BUG_ON(!stm_slave);

	kfree(stm_slave);
	chan->private = NULL;
}

static bool stm_dma_filter(struct dma_chan *chan, void *slave)
{
	struct stm32_dma_slave	*stm_slave = slave;
	struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);

	if (stm_slave->dma_dev != chan->device->dev)
		return false;

	if (stm_slave->chan_idx != stm_chan->idx)
		return false;

	chan->private = stm_slave;

	return true;
}

static struct dma_chan *stm_dma_xlate(struct of_phandle_args *dma_spec,
				      struct of_dma *of_dma)
{
	struct dma_chan		*chan;
	struct stm32_dma_chan	*stm_chan;
	struct stm32_dma_slave	*stm_slave;
	struct platform_device	*pdev;
	dma_cap_mask_t		mask;

	if (dma_spec->args_count != 2)
		return NULL;

	pdev = of_find_device_by_node(dma_spec->np);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	stm_slave = kzalloc(sizeof(*stm_slave), GFP_KERNEL);
	if (!stm_slave)
		return NULL;

	stm_slave->dma_dev = &pdev->dev;
	stm_slave->chan_idx = dma_spec->args[0];
	stm_slave->cfg = dma_spec->args[1] << STM32_DMA_CR_CHSEL_OFS;

	chan = dma_request_channel(mask, stm_dma_filter, stm_slave);
	if (!chan)
		return NULL;

	stm_chan = to_stm_dma_chan(chan);
	stm_chan->cr_msk = stm_slave->cfg;

	return chan;
}

/******************************************************************************
 * Module management
 ******************************************************************************/
static struct stm32_dma_platform_data stm32f4_config = {
	.nr_channels = 8,
	.iflags = {0, 6, 16, 22, 32, 38, 48, 54},
};

static const struct of_device_id stm32_dma_dt_ids[] = {
	   {
		.compatible = "st,stm32f4-dma",
		.data = &stm32f4_config,
	}, {
		/* sentinel */
	}
};

static const struct platform_device_id stm32_dma_devtypes[] = {
	{
		.name = "stm32f4_dma",
		.driver_data = (unsigned long) &stm32f4_config,
	}, {
		/* sentinel */
	}
};

static inline const struct stm32_dma_platform_data * __init
	stm32_dma_get_driver_data(struct platform_device *pdev)
{
	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(stm32_dma_dt_ids, pdev->dev.of_node);
		if (match == NULL)
			return NULL;
		return match->data;
	}

	return (void *)platform_get_device_id(pdev)->driver_data;
}

/**
 * stm_dma_off - disable DMA controller
 * @stm_dma: the STM32 DMA device
 */
static void stm_dma_off(struct stm32_dma *stm_dma)
{
	int	i;

	for (i = 0; i < stm_dma->chan_num; i++)
		stm_dma->regs->stream[i].cr &= ~STM32_DMA_CR_EN;
}

static int __init stm32_dma_probe(struct platform_device *pdev)
{
	struct resource		*io;
	struct stm32_dma	*dma;
	size_t			size;
	int			irq, err, i;

	const struct stm32_dma_platform_data	*pdat;

	/* Setup platform data for each SoC */
	dma_cap_set(DMA_SLAVE, stm32f4_config.cap_mask);

	/* Get DMA parameters from controller type */
	pdat = stm32_dma_get_driver_data(pdev);
	if (!pdat) {
		dev_err(&pdev->dev, "can't get platform data");
		return -ENODEV;
	}

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		dev_err(&pdev->dev, "can't get reg base");
		return -EINVAL;
	}

	size = sizeof(struct stm32_dma);
	size += pdat->nr_channels * sizeof(struct stm32_dma_chan);
	dma = kzalloc(size, GFP_KERNEL);
	if (!dma) {
		dev_err(&pdev->dev, "can't allocate dma dsc");
		return -ENOMEM;
	}

	dma->dev.cap_mask = pdat->cap_mask;
	dma->chan_num = pdat->nr_channels;

	size = resource_size(io);
	if (!request_mem_region(io->start, size, pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "request mem failure");
		err = -EBUSY;
		goto err_kfree;
	}

	dma->regs = ioremap(io->start, size);
	if (!dma->regs) {
		dev_err(&pdev->dev, "remap regs failure");
		err = -ENOMEM;
		goto err_release_r;
	}

	dma->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(dma->clk)) {
		dev_err(&pdev->dev, "can't get clock");
		err = PTR_ERR(dma->clk);
		goto err_clk;
	}
	err = clk_prepare_enable(dma->clk);
	if (err) {
		dev_err(&pdev->dev, "clock enable failure");
		goto err_clk_prepare;
	}

	stm_dma_off(dma);

	platform_set_drvdata(pdev, dma);

	/* Initialize channels */
	INIT_LIST_HEAD(&dma->dev.channels);
	for (i = 0; i < dma->chan_num; i++) {
		struct stm32_dma_chan	*ch = &dma->chan[i];

		irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			dev_err(&pdev->dev, "can't get chan %d irq", i);
			goto err_irq;
		}

		err = request_irq(irq, stm_dma_interrupt, 0, "stm32_dma", ch);
		if (err) {
			dev_err(&pdev->dev, "request irq %d for chan %d fail\n",
				irq, i);
			goto err_irq;
		}

		ch->chan.device = &dma->dev;
		ch->idx = i;

		dma_cookie_init(&ch->chan);
		list_add_tail(&ch->chan.device_node, &dma->dev.channels);

		ch->regs = &dma->regs->stream[i];

		if (pdat->iflags[i] / 32) {
			ch->isr_reg = &dma->regs->hisr;
			ch->ifcr_reg = &dma->regs->hifcr;
		} else {
			ch->isr_reg = &dma->regs->lisr;
			ch->ifcr_reg = &dma->regs->lifcr;
		}
		ch->i_ofs = pdat->iflags[i] % 32;

		spin_lock_init(&ch->lock);
		tasklet_init(&ch->tasklet, stm_tasklet, (unsigned long)ch);
	}

	/* Set base routines */
	dma->dev.device_alloc_chan_resources = stm_alloc_chan_resources;
	dma->dev.device_free_chan_resources = stm_free_chan_resources;
	dma->dev.device_tx_status = stm_tx_status;
	dma->dev.device_issue_pending = stm_issue_pending;
	dma->dev.dev = &pdev->dev;

	/* Set prep rourintes basing on capability */
	if (dma_has_cap(DMA_SLAVE, dma->dev.cap_mask)) {
		dma->dev.device_prep_slave_sg = stm_prep_slave_sg;
		/* controller can do slave DMA: can trigger cyclic transfers */
		dma_cap_set(DMA_CYCLIC, dma->dev.cap_mask);
		dma->dev.device_prep_dma_cyclic = stm_prep_dma_cyclic;
		dma->dev.device_config = stm_config;
		dma->dev.device_terminate_all = stm_terminate_all;
		dma->dev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE)  |
					   BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
					   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
		dma->dev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE)  |
					   BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
					   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
		dma->dev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
		dma->dev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	}

	dev_info(&pdev->dev, "STM32 DMA Controller ( %s), %d channels\n",
		 dma_has_cap(DMA_SLAVE, dma->dev.cap_mask)  ? "slave " : "",
		 pdat->nr_channels);

	dma_async_device_register(&dma->dev);

	/*
	 * Do not return an error if the dmac node is not present in order to
	 * not break the existing way of requesting channel with
	 * dma_request_channel().
	 */
	if (pdev->dev.of_node) {
		err = of_dma_controller_register(pdev->dev.of_node,
						 stm_dma_xlate, dma);
		if (err) {
			dev_err(&pdev->dev, "could not register of_dma_controller\n");
			goto err_of_dma_controller_register;
		}
	}

	return 0;

err_of_dma_controller_register:
	dma_async_device_unregister(&dma->dev);
err_irq:
	for (i = 0; i < dma->chan_num; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			continue;
		free_irq(irq, &dma->chan[i]);
	}
	clk_disable_unprepare(dma->clk);
err_clk_prepare:
	clk_put(dma->clk);
err_clk:
	iounmap(dma->regs);
	dma->regs = NULL;
err_release_r:
	release_mem_region(io->start, size);
err_kfree:
	kfree(dma);

	return err;
}

static int stm32_dma_remove(struct platform_device *pdev)
{
	struct stm32_dma	*stm_dma = platform_get_drvdata(pdev);
	struct dma_chan		*chan, *_chan;
	struct resource		*io;
	int			i;

	stm_dma_off(stm_dma);
	dma_async_device_unregister(&stm_dma->dev);
	for (i = 0; i < stm_dma->chan_num; i++)
		free_irq(platform_get_irq(pdev, i), &stm_dma->chan[i]);

	list_for_each_entry_safe(chan, _chan, &stm_dma->dev.channels,
				 device_node) {
		struct stm32_dma_chan	*stm_chan = to_stm_dma_chan(chan);

		tasklet_kill(&stm_chan->tasklet);
		list_del(&chan->device_node);
	}

	clk_disable_unprepare(stm_dma->clk);
	clk_put(stm_dma->clk);

	iounmap(stm_dma->regs);
	stm_dma->regs = NULL;

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(io->start, resource_size(io));

	kfree(stm_dma);

	return 0;
}

static void stm32_dma_shutdown(struct platform_device *pdev)
{
	struct stm32_dma	*stm_dma = platform_get_drvdata(pdev);

	stm_dma_off(stm_dma);
	clk_disable_unprepare(stm_dma->clk);
}

static struct platform_driver stm32_dma_driver = {
	.remove		= stm32_dma_remove,
	.shutdown	= stm32_dma_shutdown,
	.id_table	= stm32_dma_devtypes,
	.driver = {
		.name	= "stm32_dma",
		.of_match_table	= of_match_ptr(stm32_dma_dt_ids),
	},
};

static int __init stm32_dma_init(void)
{
	return platform_driver_probe(&stm32_dma_driver, stm32_dma_probe);
}
subsys_initcall(stm32_dma_init);

static void __exit stm32_dma_exit(void)
{
	platform_driver_unregister(&stm32_dma_driver);
}
module_exit(stm32_dma_exit);

MODULE_DESCRIPTION("STM32 DMA Controller driver");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:stm32_dma");
