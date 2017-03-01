/*
 * This file is part of STM32 ADC driver
 *
 * Copyright (C) 2016, STMicroelectronics - All Rights Reserved
 * Author: Fabrice Gasnier <fabrice.gasnier@st.com>.
 *
 * License type: GPLv2
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include "stm32-adc.h"

/**
 * stm32_adc_conf_smp() - Configure sampling time for each channel
 * @indio_dev: IIO device
 * @scan_mask: channels to be converted
 */
static int stm32_adc_conf_smp(struct iio_dev *indio_dev,
			      const unsigned long *scan_mask)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	const struct stm32_adc_regs *smp =
		adc->common->data->adc_reginfo->smpr_regs;
	struct stm32_adc_chan *stm32_chan;
	const struct iio_chan_spec *chan;
	u32 bit, val;
	int i;

	for_each_set_bit(bit, scan_mask, indio_dev->masklength) {
		chan = indio_dev->channels + bit;
		stm32_chan = to_stm32_chan(adc, chan);
		i = chan->channel;

		if (i >= adc->max_channels)
			return -EINVAL;

		val = stm32_adc_readl(adc, smp[i].reg);
		val &= ~smp[i].mask;
		val |= (stm32_chan->smpr << smp[i].shift) & smp[i].mask;
		stm32_adc_writel(adc, smp[i].reg, val);
	}

	return 0;
}

int stm32_adc_set_smpr(struct iio_dev *indio_dev,
		       const struct iio_chan_spec *chan, unsigned int smpr)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	struct stm32_adc_chan *stm32_chan = to_stm32_chan(adc, chan);

	stm32_chan->smpr = smpr;

	return 0;
}
EXPORT_SYMBOL_GPL(stm32_adc_set_smpr);

int stm32_adc_get_smpr(struct iio_dev *indio_dev,
		       const struct iio_chan_spec *chan)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	struct stm32_adc_chan *stm32_chan = to_stm32_chan(adc, chan);

	return stm32_chan->smpr;
}
EXPORT_SYMBOL_GPL(stm32_adc_get_smpr);

/**
 * stm32_adc_conf_scan_seq() - Build regular or injected channels scan sequence
 * @indio_dev: IIO device
 * @scan_mask: channels to be converted
 *
 * Conversion sequence :
 * Configure ADC scan sequence based on selected channels in scan_mask.
 * Add channels to SQR or JSQR registers, from scan_mask LSB to MSB, then
 * program sequence len.
 *
 * Note: This can be done only when ADC enabled and no conversion is ongoing.
 */
static int stm32_adc_conf_scan_seq(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	const struct stm32_adc_regs *sq;
	const struct iio_chan_spec *chan;
	u32 val, bit;
	int sq_max, i = 0;

	if (!stm32_adc_is_enabled(adc))
		return -EBUSY;

	if (adc->injected) {
		if (stm32_adc_injected_started(adc))
			return -EBUSY;

		sq = adc->common->data->adc_reginfo->jsqr_reg;
		sq_max = STM32_ADC_MAX_JSQ;
	} else {
		if (stm32_adc_regular_started(adc))
			return -EBUSY;

		sq = adc->common->data->adc_reginfo->sqr_regs;
		sq_max = STM32_ADC_MAX_SQ;
	}

	/* Build sequence for regular or injected channels */
	for_each_set_bit(bit, scan_mask, indio_dev->masklength) {
		chan = indio_dev->channels + bit;
		/*
		 * Assign one channel per SQ/JSQ entry in regular/injected
		 * sequence, starting with SQ1/JSQ1.
		 */
		i++;
		if (i > sq_max)
			return -EINVAL;

		dev_dbg(adc->common->dev, "%s chan %d to %s%d\n",
			__func__, chan->channel, adc->injected ? "JSQ" : "SQ",
			i);

		val = stm32_adc_readl(adc, sq[i].reg);
		val &= ~sq[i].mask;
		val |= (chan->channel << sq[i].shift) & sq[i].mask;
		stm32_adc_writel(adc, sq[i].reg, val);
	}

	if (!i)
		return -EINVAL;

	/* Sequence len */
	val = stm32_adc_readl(adc, sq[0].reg);
	val &= ~sq[0].mask;
	val |= ((i - 1) << sq[0].shift) & sq[0].mask;
	stm32_adc_writel(adc, sq[0].reg, val);

	return 0;
}

static int stm32_adc_conf_scan(struct iio_dev *indio_dev,
			       const unsigned long *scan_mask)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	int ret;

	ret = stm32_adc_clk_sel(adc);
	if (ret) {
		dev_err(&indio_dev->dev, "Clock sel failed\n");
		return ret;
	}

	ret = stm32_adc_enable(adc);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to enable adc\n");
		return ret;
	}

	ret = stm32_adc_conf_smp(indio_dev, scan_mask);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to configure samp time\n");
		goto err_dis;
	}

	ret = stm32_adc_conf_scan_seq(indio_dev, scan_mask);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to configure sequence\n");
		goto err_dis;
	}

	return 0;

err_dis:
	stm32_adc_disable(adc);

	return ret;
}

/**
 * stm32_adc_get_trig_index() - Get trigger index
 * @indio_dev: IIO device
 * @trig: trigger
 *
 * Returns trigger index, if trig matches one of the triggers registered by
 * stm32 adc driver, -EINVAL otherwise.
 */
static int stm32_adc_get_trig_index(struct iio_dev *indio_dev,
				    struct iio_trigger *trig)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	struct iio_trigger *tr;
	int i = 0;

	list_for_each_entry(tr, &adc->extrig_list, alloc_list) {
		if (tr == trig)
			return i;
		i++;
	}

	return -EINVAL;
}

/**
 * stm32_adc_set_trig() - Set a regular or injected trigger
 * @indio_dev: IIO device
 * @trig: IIO trigger
 *
 * Set trigger source/polarity (e.g. SW, or HW with polarity) :
 * - if HW trigger disabled (e.g. trig == NULL, conversion launched by sw)
 * - if HW trigger enabled, set source & polarity (trigger_pol / jtrigger_pol)
 *
 * Note: must be called when ADC is enabled
 */
static int stm32_adc_set_trig(struct iio_dev *indio_dev,
			      struct iio_trigger *trig)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	const struct stm32_adc_trig_info *trig_info;
	const struct stm32_adc_trig_reginfo *reginfo;
	u32 val, extsel = 0, exten = STM32_EXTEN_SWTRIG;
	unsigned long flags;
	int ret;

	if (!stm32_adc_is_enabled(adc))
		return -EBUSY;

	if (adc->injected) {
		if (stm32_adc_injected_started(adc))
			return -EBUSY;
		reginfo = adc->common->data->adc_reginfo->jtrig_reginfo;
		trig_info = adc->common->data->jext_triggers;
	} else {
		if (stm32_adc_regular_started(adc))
			return -EBUSY;
		reginfo = adc->common->data->adc_reginfo->trig_reginfo;
		trig_info = adc->common->data->ext_triggers;
	}

	if (trig) {
		ret = stm32_adc_get_trig_index(indio_dev, trig);
		if (ret < 0)
			return ret;

		/* trigger source */
		extsel = trig_info[ret].extsel;

		/* default to rising edge if no polarity */
		if (adc->exten == STM32_EXTEN_SWTRIG)
			adc->exten = STM32_EXTEN_HWTRIG_RISING_EDGE;
		exten = adc->exten;
	}

	spin_lock_irqsave(&adc->lock, flags);
	val = stm32_adc_readl(adc, reginfo->reg);
	val &= ~(reginfo->exten_mask | reginfo->extsel_mask);
	val |= (exten << reginfo->exten_shift) & reginfo->exten_mask;
	val |= (extsel << reginfo->extsel_shift) & reginfo->extsel_mask;
	stm32_adc_writel(adc, reginfo->reg, val);
	spin_unlock_irqrestore(&adc->lock, flags);

	return 0;
}

static int stm32_adc_set_trig_pol(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  unsigned int type)
{
	struct stm32_adc *adc = iio_priv(indio_dev);

	adc->exten = type;

	return 0;
}

static int stm32_adc_get_trig_pol(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct stm32_adc *adc = iio_priv(indio_dev);

	return adc->exten;
}

static const char * const stm32_trig_pol_items[] = {
	[STM32_EXTEN_SWTRIG] = "swtrig",
	[STM32_EXTEN_HWTRIG_RISING_EDGE] = "rising-edge",
	[STM32_EXTEN_HWTRIG_FALLING_EDGE] = "falling-edge",
	[STM32_EXTEN_HWTRIG_BOTH_EDGES] = "both-edges",
};

const struct iio_enum stm32_adc_trig_pol = {
	.items = stm32_trig_pol_items,
	.num_items = ARRAY_SIZE(stm32_trig_pol_items),
	.get = stm32_adc_get_trig_pol,
	.set = stm32_adc_set_trig_pol,
};
EXPORT_SYMBOL_GPL(stm32_adc_trig_pol);

/**
 * stm32_adc_conv_irq_enable() - Unmask end of conversion irq
 * @adc: stm32 adc instance
 *
 * Unmask either eoc or jeoc, depending on injected configuration.
 */
static void stm32_adc_conv_irq_enable(struct stm32_adc *adc)
{
	const struct stm32_adc_reginfo *reginfo =
		adc->common->data->adc_reginfo;
	u32 mask;

	if (adc->injected)
		mask = reginfo->jeocie;
	else
		mask = reginfo->eocie;

	stm32_adc_set_bits(adc, reginfo->ier, mask);
}

/**
 * stm32_adc_conv_irq_disable() - Mask end of conversion irq
 * @adc: stm32 adc instance
 *
 * Mask either eoc or jeoc, depending on injected configuration.
 */
static void stm32_adc_conv_irq_disable(struct stm32_adc *adc)
{
	const struct stm32_adc_reginfo *reginfo =
		adc->common->data->adc_reginfo;
	u32 mask;

	if (adc->injected)
		mask = reginfo->jeocie;
	else
		mask = reginfo->eocie;

	stm32_adc_clr_bits(adc, reginfo->ier, mask);
}

/**
 * stm32_adc_single_conv() - perform a single conversion
 * @indio_dev: IIO device
 * @chan: IIO channel
 * @result: conversion result
 *
 * The function performs a single conversion on a given channel, by
 * by:
 * - creating scan mask with only one channel
 * - using SW trigger
 * - then start single conv
 */
static int stm32_adc_single_conv(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	unsigned long *scan_mask;
	long timeout;
	u16 result;
	int ret;

	scan_mask = kcalloc(BITS_TO_LONGS(indio_dev->masklength), sizeof(long),
			    GFP_KERNEL);
	if (!scan_mask)
		return -ENOMEM;

	set_bit(chan->scan_index, scan_mask);

	reinit_completion(&adc->completion);

	adc->bufi = 0;
	adc->num_conv = 1;
	adc->buffer = &result;

	ret = stm32_adc_conf_scan(indio_dev, scan_mask);
	if (ret)
		goto free;

	/* No HW trigger: conversion can be launched in SW */
	ret = stm32_adc_set_trig(indio_dev, NULL);
	if (ret) {
		dev_err(&indio_dev->dev, "Can't set SW trigger\n");
		goto adc_disable;
	}

	stm32_adc_prepare_conv(adc, chan);
	stm32_adc_conv_irq_enable(adc);

	ret = stm32_adc_start_conv(adc);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to start single conv\n");
		goto irq_disable;
	}

	timeout = wait_for_completion_interruptible_timeout(
					&adc->completion, STM32_ADC_TIMEOUT);
	if (timeout == 0) {
		dev_warn(&indio_dev->dev, "Conversion timed out!\n");
		ret = -ETIMEDOUT;
	} else if (timeout < 0) {
		dev_warn(&indio_dev->dev, "Interrupted conversion!\n");
		ret = -EINTR;
	} else {
		*val = result & STM32_RESULT_MASK;
		ret = IIO_VAL_INT;
	}

	if (stm32_adc_stop_conv(adc))
		dev_err(&indio_dev->dev, "stop failed\n");

irq_disable:
	stm32_adc_conv_irq_disable(adc);

adc_disable:
	stm32_adc_disable(adc);

free:
	kfree(scan_mask);
	adc->buffer = NULL;

	return ret;
}

static int stm32_adc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_VOLTAGE || chan->type == IIO_TEMP)
			ret = stm32_adc_single_conv(indio_dev, chan, val);
		break;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		if (chan->type == IIO_VOLTAGE)
			ret = stm32_adc_avg_get(indio_dev, chan, val);
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = adc->common->vref_mv;
		*val2 = chan->scan_type.realbits;
		ret = IIO_VAL_FRACTIONAL_LOG2;
		break;
	default:
		break;
	}

	return ret;
}

static int stm32_adc_residue(struct stm32_adc *adc)
{
	struct dma_tx_state state;
	enum dma_status status;

	if (!adc->rx_buf)
		return 0;

	status = dmaengine_tx_status(adc->dma_chan,
				     adc->dma_chan->cookie,
				     &state);
	if (status == DMA_IN_PROGRESS) {
		/* Residue is size in bytes from end of buffer */
		int i = adc->rx_buf_sz - state.residue;
		int size;

		/* Return available bytes */
		if (i >= adc->bufi)
			size = i - adc->bufi;
		else
			size = adc->rx_buf_sz - adc->bufi + i;

		return size;
	}

	return 0;
}

/**
 * stm32_adc_isr() - Treat interrupt for one ADC instance within ADC block
 */
static irqreturn_t stm32_adc_isr(struct stm32_adc *adc)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(adc);
	const struct stm32_adc_reginfo *reginfo =
		adc->common->data->adc_reginfo;
	u32 mask, clr_mask;
	u32 ier = stm32_adc_readl(adc, reginfo->ier);
	u32 isr = stm32_adc_readl(adc, reginfo->isr);

	if (adc->injected) {
		mask = reginfo->ovr | reginfo->jeoc;
		clr_mask = mask;
	} else {
		mask = reginfo->ovr | reginfo->eoc;
		/* don't clear 'eoc' as it is cleared when reading 'dr' */
		clr_mask = reginfo->ovr;
	}

	/* clear irq */
	stm32_adc_writel(adc, reginfo->isr, isr & ~clr_mask);
	isr &= mask;

	/* Overruns */
	if (isr & reginfo->ovr) {
		if (stm32_avg_is_enabled(adc))
			stm32_adc_avg_overrun(adc);
	}

	/* Regular data (when dma isn't used) */
	if ((ier & reginfo->eocie) && (isr & reginfo->eoc) && !adc->rx_buf) {
		adc->buffer[adc->bufi] = stm32_adc_readl(adc, reginfo->dr);
		if (iio_buffer_enabled(indio_dev)) {
			adc->bufi++;
			if (adc->bufi >= adc->num_conv) {
				stm32_adc_conv_irq_disable(adc);
				iio_trigger_poll(indio_dev->trig);
			}
		} else {
			complete(&adc->completion);
		}
	}

	/* Injected data */
	if ((ier & reginfo->jeocie) && (isr & reginfo->jeoc)) {
		int i;

		for (i = 0; i < adc->num_conv; i++) {
			adc->buffer[i] = stm32_adc_readl(adc, reginfo->jdr[i]);
			adc->bufi++;
		}

		if (iio_buffer_enabled(indio_dev)) {
			stm32_adc_conv_irq_disable(adc);
			iio_trigger_poll(indio_dev->trig);
		} else {
			complete(&adc->completion);
		}
	}

	/*
	 * In case end of conversion flags have been handled, this has been
	 * handled for this ADC instance
	 */
	if (isr)
		return IRQ_HANDLED;

	/* This adc instance didn't trigger this interrupt */
	return IRQ_NONE;
}

/**
 * stm32_adc_common_isr() - Common isr for the whole ADC block
 *
 * There is one IRQ for all ADCs in ADC block, check all instances.
 */
static irqreturn_t stm32_adc_common_isr(int irq, void *data)
{
	struct stm32_adc_common *common = data;
	irqreturn_t ret = IRQ_NONE;
	struct stm32_adc *adc;

	list_for_each_entry(adc, &common->adc_list, adc_list)
		ret |= stm32_adc_isr(adc);

	return ret;
}

static irqreturn_t stm32_adc_exti_handler(int irq, void *data)
{
	/* Exti handler should not be invoqued, and is not used */
	return IRQ_HANDLED;
}

/**
 * stm32_adc_validate_trigger() - validate trigger for stm32 adc
 * @indio_dev: IIO device
 * @trig: new trigger
 *
 * Returns: 0 if trig matches one of the triggers registered by stm32 adc
 * driver, -EINVAL otherwise.
 */
static int stm32_adc_validate_trigger(struct iio_dev *indio_dev,
				      struct iio_trigger *trig)
{
	return stm32_adc_get_trig_index(indio_dev, trig) < 0 ? -EINVAL : 0;
}

static int stm32_adc_update_scan_mode(struct iio_dev *indio_dev,
				      const unsigned long *scan_mask)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	int ret;
	u32 bit;

	adc->num_conv = 0;
	for_each_set_bit(bit, scan_mask, indio_dev->masklength)
		adc->num_conv++;

	ret = stm32_adc_conf_scan(indio_dev, scan_mask);
	if (ret)
		return ret;

	return 0;
}

static int stm32_adc_of_xlate(struct iio_dev *indio_dev,
			      const struct of_phandle_args *iiospec)
{
	int i;

	for (i = 0; i < indio_dev->num_channels; i++)
		if (indio_dev->channels[i].channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

/**
 * stm32_adc_debugfs_reg_access - read or write register value
 *
 * To read a value from an ADC register:
 *   echo [ADC reg offset] > direct_reg_access
 *   cat direct_reg_access
 *
 * To write a value in a ADC register:
 *   echo [ADC_reg_offset] [value] > direct_reg_access
 */
static int stm32_adc_debugfs_reg_access(struct iio_dev *indio_dev,
					unsigned reg, unsigned writeval,
					unsigned *readval)
{
	struct stm32_adc *adc = iio_priv(indio_dev);

	if (!readval)
		stm32_adc_writel(adc, reg, writeval);
	else
		*readval = stm32_adc_readl(adc, reg);

	return 0;
}

static const struct iio_info stm32_adc_iio_info = {
	.read_raw = stm32_adc_read_raw,
	.validate_trigger = stm32_adc_validate_trigger,
	.update_scan_mode = stm32_adc_update_scan_mode,
	.debugfs_reg_access = stm32_adc_debugfs_reg_access,
	.of_xlate = stm32_adc_of_xlate,
	.driver_module = THIS_MODULE,
};

static int stm32_adc_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct stm32_adc *adc = iio_priv(indio_dev);

	stm32_adc_disable(adc);

	return 0;
}

static const struct iio_buffer_setup_ops iio_triggered_buffer_setup_ops = {
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &stm32_adc_buffer_postdisable,
};

static int stm32_adc_validate_device(struct iio_trigger *trig,
				     struct iio_dev *indio_dev)
{
	struct iio_dev *indio = iio_trigger_get_drvdata(trig);

	return indio != indio_dev ? -EINVAL : 0;
}

static void stm32_adc_dma_irq_work(struct irq_work *work)
{
	struct stm32_adc *adc = container_of(work, struct stm32_adc, work);
	struct iio_dev *indio_dev = iio_priv_to_dev(adc);

	/**
	 * iio_trigger_poll calls generic_handle_irq(). So, it requires hard
	 * irq context, and cannot be called directly from dma callback,
	 * dma cb has to schedule this work instead.
	 */
	iio_trigger_poll(indio_dev->trig);
}

static void stm32_adc_dma_buffer_done(void *data)
{
	struct iio_dev *indio_dev = data;
	struct stm32_adc *adc = iio_priv(indio_dev);

	/* invoques iio_trigger_poll() from hard irq context */
	irq_work_queue(&adc->work);
}

static int stm32_adc_buffer_alloc_dma_start(struct iio_dev *indio_dev)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	const struct stm32_adc_reginfo *reginfo =
		adc->common->data->adc_reginfo;
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config config;
	dma_cookie_t cookie;
	int ret, size, watermark;

	/* Reset adc buffer index */
	adc->bufi = 0;

	if (!adc->dma_chan) {
		/* Allocate adc buffer */
		adc->buffer = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);
		if (!adc->buffer)
			return -ENOMEM;

		return 0;
	}

	/*
	 * Allocate at least twice the buffer size for dma cyclic transfers, so
	 * we can work with at least two dma periods. There should be :
	 * - always one buffer (period) dma is working on
	 * - one buffer (period) driver can push with iio_trigger_poll().
	 */
	size = indio_dev->buffer->bytes_per_datum * indio_dev->buffer->length;
	size = max(indio_dev->scan_bytes * 2, size);

	adc->rx_buf = dma_alloc_coherent(adc->common->dev, PAGE_ALIGN(size),
					 &adc->rx_dma_buf,
					 GFP_KERNEL);
	if (!adc->rx_buf)
		return -ENOMEM;
	adc->rx_buf_sz = size;
	watermark = indio_dev->buffer->bytes_per_datum
		* indio_dev->buffer->watermark;
	watermark = max(indio_dev->scan_bytes, watermark);
	watermark = rounddown(watermark, indio_dev->scan_bytes);

	dev_dbg(&indio_dev->dev, "%s size=%d watermark=%d\n", __func__, size,
		watermark);

	/* Configure DMA channel to read data register */
	memset(&config, 0, sizeof(config));
	config.src_addr = (dma_addr_t)adc->common->phys_base;
	config.src_addr += adc->offset + reginfo->dr;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	ret = dmaengine_slave_config(adc->dma_chan, &config);
	if (ret)
		goto config_err;

	/* Prepare a DMA cyclic transaction */
	desc = dmaengine_prep_dma_cyclic(adc->dma_chan,
					 adc->rx_dma_buf,
					 size, watermark,
					 DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENODEV;
		goto config_err;
	}

	desc->callback = stm32_adc_dma_buffer_done;
	desc->callback_param = indio_dev;

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		ret = dma_submit_error(cookie);
		goto config_err;
	}

	/* Issue pending DMA requests */
	dma_async_issue_pending(adc->dma_chan);

	return 0;

config_err:
	dma_free_coherent(adc->common->dev, PAGE_ALIGN(size), adc->rx_buf,
			  adc->rx_dma_buf);

	return ret;
}

static void stm32_adc_dma_stop_buffer_free(struct iio_dev *indio_dev)
{
	struct stm32_adc *adc = iio_priv(indio_dev);

	if (!adc->dma_chan) {
		kfree(adc->buffer);
	} else {
		dmaengine_terminate_all(adc->dma_chan);
		irq_work_sync(&adc->work);
		dma_free_coherent(adc->common->dev, PAGE_ALIGN(adc->rx_buf_sz),
				  adc->rx_buf, adc->rx_dma_buf);
		adc->rx_buf = NULL;
	}

	adc->buffer = NULL;
}

static int stm32_adc_set_trigger_state(struct iio_trigger *trig,
				       bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct stm32_adc *adc = iio_priv(indio_dev);
	int ret;

	if (state) {
		ret = stm32_adc_buffer_alloc_dma_start(indio_dev);
		if (ret) {
			dev_err(&indio_dev->dev, "alloc failed\n");
			return ret;
		}

		ret = stm32_adc_set_trig(indio_dev, trig);
		if (ret) {
			dev_err(&indio_dev->dev, "Can't set trigger\n");
			goto err_buffer_free;
		}

		if (!adc->dma_chan)
			stm32_adc_conv_irq_enable(adc);

		ret = stm32_adc_start_conv(adc);
		if (ret) {
			dev_err(&indio_dev->dev, "Failed to start\n");
			goto err_irq_trig_disable;
		}
	} else {
		ret = stm32_adc_stop_conv(adc);
		if (ret < 0) {
			dev_err(&indio_dev->dev, "Failed to stop\n");
			return ret;
		}

		if (!adc->dma_chan)
			stm32_adc_conv_irq_disable(adc);

		ret = stm32_adc_set_trig(indio_dev, NULL);
		if (ret)
			dev_warn(&indio_dev->dev, "Can't clear trigger\n");

		stm32_adc_dma_stop_buffer_free(indio_dev);
	}

	return 0;

err_irq_trig_disable:
	if (!adc->dma_chan)
		stm32_adc_conv_irq_disable(adc);
	stm32_adc_set_trig(indio_dev, NULL);

err_buffer_free:
	stm32_adc_dma_stop_buffer_free(indio_dev);

	return ret;
}

static const struct iio_trigger_ops stm32_adc_trigger_ops = {
	.owner = THIS_MODULE,
	.validate_device = stm32_adc_validate_device,
	.set_trigger_state = stm32_adc_set_trigger_state,
};

static irqreturn_t stm32_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct stm32_adc *adc = iio_priv(indio_dev);

	if (!adc->dma_chan) {
		adc->bufi = 0;
		iio_push_to_buffers_with_timestamp(indio_dev, adc->buffer,
						   pf->timestamp);
	} else {
		int residue = stm32_adc_residue(adc);

		while (residue >= indio_dev->scan_bytes) {
			adc->buffer = (u16 *)&adc->rx_buf[adc->bufi];
			iio_push_to_buffers_with_timestamp(indio_dev,
							   adc->buffer,
							   pf->timestamp);
			residue -= indio_dev->scan_bytes;
			adc->bufi += indio_dev->scan_bytes;
			if (adc->bufi >= adc->rx_buf_sz)
				adc->bufi = 0;
		}
	}

	iio_trigger_notify_done(indio_dev->trig);

	/* re-enable eoc irq */
	if (!adc->dma_chan)
		stm32_adc_conv_irq_enable(adc);

	return IRQ_HANDLED;
}

static void stm32_adc_trig_unregister(struct iio_dev *indio_dev)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	struct iio_trigger *trig, *_t;

	list_for_each_entry_safe(trig, _t, &adc->extrig_list, alloc_list) {
		iio_trigger_unregister(trig);
		list_del(&trig->alloc_list);
	}
}

static int stm32_adc_trig_register(struct iio_dev *indio_dev)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	struct stm32_adc_common *common = adc->common;
	const struct stm32_adc_trig_info *ext = common->data->ext_triggers;
	struct iio_trigger *trig;
	int i, ret = 0;

	if (adc->injected)
		ext = common->data->jext_triggers;
	else
		ext = common->data->ext_triggers;

	for (i = 0; ext && ext[i].name; i++) {
		trig = devm_iio_trigger_alloc(common->dev, "%s_%s%d_%s",
					      indio_dev->name,
					      adc->injected ? "jext" : "ext",
					      ext[i].extsel, ext[i].name);
		if (!trig) {
			dev_err(common->dev, "trig %s_%s%d_%s alloc failed\n",
				indio_dev->name,
				adc->injected ? "jext" : "ext",
				ext[i].extsel, ext[i].name);
			ret = -ENOMEM;
			goto err;
		}

		trig->dev.parent = common->dev;
		trig->ops = &stm32_adc_trigger_ops;
		iio_trigger_set_drvdata(trig, indio_dev);

		ret = iio_trigger_register(trig);
		if (ret) {
			dev_err(common->dev,
				"trig %s_%s%d_%s register failed\n",
				indio_dev->name,
				adc->injected ? "jext" : "ext",
				ext[i].extsel, ext[i].name);
			goto err;
		}

		list_add_tail(&trig->alloc_list, &adc->extrig_list);
	}

	return 0;
err:
	stm32_adc_trig_unregister(indio_dev);

	return ret;
}

static void stm32_adc_chan_init_one(struct iio_dev *indio_dev,
				    struct iio_chan_spec *chan,
				    const struct stm32_adc_chan_spec *channel,
				    int scan_index)
{
	struct stm32_adc *adc = iio_priv(indio_dev);

	chan->type = channel->type;
	chan->channel = channel->channel;
	chan->datasheet_name = channel->name;
	chan->extend_name = channel->name;
	chan->scan_index = scan_index;
	chan->indexed = 1;

	/*
	 * Support averaging for `regular` channels only
	 */
	if (adc->injected) {
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	} else {
		chan->info_mask_separate = stm32_avg_is_enabled(adc) ?
					   BIT(IIO_CHAN_INFO_AVERAGE_RAW) :
					   BIT(IIO_CHAN_INFO_RAW);
	}

	chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = adc->common->data->highres;
	chan->scan_type.storagebits = STM32_STORAGEBITS;
	chan->scan_type.shift = 0;
	chan->ext_info = adc->common->data->ext_info;
}

static int stm32_adc_chan_of_init(struct iio_dev *indio_dev,
				  const struct stm32_adc_info *adc_info)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	struct stm32_adc_common *common = adc->common;
	struct device_node *node = indio_dev->dev.of_node;
	struct property *prop;
	const __be32 *cur;
	struct iio_chan_spec *channels;
	int scan_index = 0, num_channels = 0;
	u32 val;

	if (!common->stm32_chans[adc->id]) {
		/* Allocate extended attributes structure for an instance */
		struct stm32_adc_chan *stm32_chans;

		stm32_chans = devm_kcalloc(&indio_dev->dev,
					   adc_info->max_channels,
					   sizeof(*stm32_chans), GFP_KERNEL);
		if (!stm32_chans)
			return -ENOMEM;

		common->stm32_chans[adc->id] = stm32_chans;
	}

	of_property_for_each_u32(node, "st,adc-channels", prop, cur, val)
		num_channels++;

	channels = devm_kcalloc(&indio_dev->dev, num_channels,
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	of_property_for_each_u32(node, "st,adc-channels", prop, cur, val) {
		stm32_adc_chan_init_one(indio_dev, &channels[scan_index],
					&adc_info->channels[val],
					scan_index);
		scan_index++;
	}

	adc->max_channels = adc_info->max_channels;
	indio_dev->num_channels = scan_index;
	indio_dev->channels = channels;

	return 0;
}

static int stm32_adc_register(struct stm32_adc_common *common,
			      struct device_node *child)
{
	struct iio_dev *indio_dev;
	struct stm32_adc *adc;
	int i, ret;
	u32 reg;

	ret = of_property_read_u32(child, "reg", &reg);
	if (ret != 0) {
		dev_err(common->dev, "missing reg property\n");
		return -EINVAL;
	}

	for (i = 0; common->data->adc_info[i].channels; i++)
		if (common->data->adc_info[i].reg == reg)
			break;

	if (i >= STM32_ADC_ID_MAX || !common->data->adc_info[i].channels) {
		dev_err(common->dev, "bad adc reg offset\n");
		return -ENOENT;
	}

	indio_dev = devm_iio_device_alloc(common->dev, sizeof(*adc));
	if (!indio_dev) {
		dev_err(common->dev, "iio device allocation failed\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);
	adc->id = i;
	adc->offset = reg;
	adc->common = common;
	INIT_LIST_HEAD(&adc->extrig_list);
	spin_lock_init(&adc->lock);
	init_completion(&adc->completion);

	if (child->name)
		indio_dev->name = child->name;
	else
		indio_dev->name = common->data->adc_info[i].name;
	indio_dev->dev.parent = common->dev;
	indio_dev->dev.of_node = child;
	indio_dev->info = &stm32_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (of_property_read_bool(child, "st,injected")) {
		dev_dbg(common->dev, "%s Configured to use injected\n",
			indio_dev->name);
		adc->injected = true;
	}

	adc->clk = of_clk_get(child, 0);
	if (IS_ERR(adc->clk)) {
		adc->clk = NULL;
		dev_dbg(common->dev, "No child clk found\n");
	} else {
		ret = clk_prepare_enable(adc->clk);
		if (ret < 0)
			goto err_clk_put;
	}

	ret = stm32_adc_chan_of_init(indio_dev, &common->data->adc_info[i]);
	if (ret < 0) {
		dev_err(common->dev, "iio channels init failed\n");
		goto err_clk_disable;
	}

	ret = stm32_adc_trig_register(indio_dev);
	if (ret)
		goto err_clk_disable;

	adc->dma_chan = dma_request_slave_channel(&indio_dev->dev, "rx");
	if (adc->dma_chan)
		init_irq_work(&adc->work, stm32_adc_dma_irq_work);

	ret = iio_triggered_buffer_setup(indio_dev,
					 &iio_pollfunc_store_time,
					 &stm32_adc_trigger_handler,
					 &iio_triggered_buffer_setup_ops);
	if (ret) {
		dev_err(common->dev, "buffer setup failed\n");
		goto err_trig_unregister;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(common->dev, "iio dev register failed\n");
		goto err_buffer_cleanup;
	}

	list_add_tail(&adc->adc_list, &common->adc_list);

	return 0;

err_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);

err_trig_unregister:
	if (adc->dma_chan)
		dma_release_channel(adc->dma_chan);
	stm32_adc_trig_unregister(indio_dev);

err_clk_disable:
	if (adc->clk)
		clk_disable_unprepare(adc->clk);

err_clk_put:
	if (adc->clk)
		clk_put(adc->clk);

	return ret;
}

static void stm32_adc_unregister(struct stm32_adc *adc)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(adc);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	stm32_adc_trig_unregister(indio_dev);
	if (adc->dma_chan)
		dma_release_channel(adc->dma_chan);
	if (adc->clk) {
		clk_disable_unprepare(adc->clk);
		clk_put(adc->clk);
	}
}

static int stm32_adc_exti_probe(struct stm32_adc_common *common)
{
	int i, irq, ret;

	common->gpios = devm_gpiod_get_array_optional(common->dev, NULL,
						      GPIOD_IN);
	if (!common->gpios)
		return 0;

	for (i = 0; i < common->gpios->ndescs; i++) {
		irq = gpiod_to_irq(common->gpios->desc[i]);
		if (irq < 0) {
			dev_err(common->dev, "gpio %d to irq failed\n", i);
			return irq;
		}

		ret = devm_request_irq(common->dev, irq, stm32_adc_exti_handler,
				       0, dev_name(common->dev), common);
		if (ret) {
			dev_err(common->dev, "request IRQ %d failed\n", irq);
			return ret;
		}

		/*
		 * gpios are configured as interrupts, so exti trigger path is
		 * configured in HW. But getting interrupts when starting
		 * conversions is unused here, so mask it in on exti
		 * controller by default.
		 */
		disable_irq(irq);
	}

	return 0;
}

int stm32_adc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct stm32_adc_common *common;
	struct stm32_adc *adc;
	struct stm32_avg *avg;
	struct resource *res;
	int ret;

	match = of_match_device(dev->driver->of_match_table, &pdev->dev);
	if (!match || !match->data) {
		dev_err(&pdev->dev, "compatible data not provided\n");
		return -EINVAL;
	}

	common = devm_kzalloc(&pdev->dev, sizeof(*common), GFP_KERNEL);
	if (!common)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	common->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(common->base))
		return PTR_ERR(common->base);
	common->phys_base = res->start;

	common->data = match->data;
	common->dev = &pdev->dev;
	platform_set_drvdata(pdev, common);
	mutex_init(&common->lock);
	INIT_LIST_HEAD(&common->adc_list);

	common->vref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(common->vref)) {
		ret = PTR_ERR(common->vref);
		dev_err(&pdev->dev, "vref get failed, %d\n", ret);
		return ret;
	}

	ret = regulator_enable(common->vref);
	if (ret < 0) {
		dev_err(&pdev->dev, "vref enable failed\n");
		return ret;
	}

	ret = regulator_get_voltage(common->vref);
	if (ret < 0) {
		dev_err(&pdev->dev, "vref get voltage failed, %d\n", ret);
		goto err_regulator_disable;
	}
	common->vref_mv = ret / 1000;
	dev_dbg(&pdev->dev, "vref+=%dmV\n", common->vref_mv);

	common->aclk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(common->aclk)) {
		ret = PTR_ERR(common->aclk);
		dev_err(&pdev->dev, "Can't get 'adc' clock\n");
		goto err_regulator_disable;
	}

	ret = clk_prepare_enable(common->aclk);
	if (ret < 0) {
		dev_err(common->dev, "adc clk enable failed\n");
		goto err_regulator_disable;
	}

	common->irq = platform_get_irq(pdev, 0);
	if (common->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq\n");
		ret = common->irq;
		goto err_clk_disable;
	}

	ret = devm_request_irq(&pdev->dev, common->irq, stm32_adc_common_isr,
			       0, pdev->name, common);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto err_clk_disable;
	}

	ret = stm32_adc_exti_probe(common);
	if (ret)
		goto err_clk_disable;

	if (common->data->parse_dts_params) {
		ret = common->data->parse_dts_params(&pdev->dev, np);
		if (ret) {
			dev_err(&pdev->dev, "parsing spec dts opts failed\n");
			goto err_clk_disable;
		}
	}

	/* Check if want to operate in the averaging mode */
	avg = &common->avg;
	ret = of_property_read_u32(np, "avg-num", &avg->num);
	if (!ret && avg->num) {
		avg->pwm = devm_of_pwm_get(&pdev->dev, np, NULL);
		if (IS_ERR(avg->pwm)) {
			dev_err(&pdev->dev, "can't get pwm\n");
			goto err_clk_disable;
		}

		ret = of_property_read_u32(np, "avg-rate", &avg->rate);
		if (ret || !avg->rate) {
			dev_err(&pdev->dev, "failed get `avg-rate`\n");
			goto err_clk_disable;
		}
	}

	/* Parse adc child nodes to retrieve master/slave instances data */
	for_each_available_child_of_node(np, child) {
		ret = stm32_adc_register(common, child);
		if (ret)
			goto err_unregister;
	}

	/* Complete average initialization */
	if (avg->num) {
		struct iio_trigger *tr = NULL;
		const char *tr_name;

		ret = of_property_read_string(np, "avg-trigger", &tr_name);
		if (ret) {
			dev_err(&pdev->dev, "no `avg-trigger` specified\n");
			goto err_unregister;
		}

		list_for_each_entry(adc, &common->adc_list, adc_list) {
			if (adc->injected)
				continue;
			list_for_each_entry(tr, &adc->extrig_list, alloc_list) {
				if (!strstr(tr->name, tr_name))
					continue;
				avg->tr = tr;
				break;
			}
			break;
		}
		if (!avg->tr) {
			dev_err(&pdev->dev, "no `%s` trigger found\n", tr_name);
			goto err_unregister;
		}

		ret = stm32_adc_avg_init(common);
		if (ret) {
			dev_err(&pdev->dev, "adc avg enable failed\n");
			goto err_unregister;
		}

		ret = stm32_adc_set_trig(iio_trigger_get_drvdata(tr), tr);
		if (ret) {
			dev_err(&pdev->dev, "can't set trigger\n");
			goto err_unregister;
		}

		list_for_each_entry(adc, &common->adc_list, adc_list) {
			if (adc->injected)
				continue;
			stm32_adc_start_conv(adc);
		}

		ret = stm32_adc_avg_run(common);
		if (ret) {
			dev_err(&pdev->dev, "can't run averaging\n");
			goto err_unregister;
		}
	}

	dev_info(&pdev->dev, "registered\n");

	return 0;

err_unregister:
	list_for_each_entry(adc, &common->adc_list, adc_list)
		stm32_adc_unregister(adc);

err_clk_disable:
	clk_disable_unprepare(common->aclk);

err_regulator_disable:
	regulator_disable(common->vref);

	return ret;
}
EXPORT_SYMBOL_GPL(stm32_adc_probe);

int stm32_adc_remove(struct platform_device *pdev)
{
	struct stm32_adc_common *common = platform_get_drvdata(pdev);
	struct stm32_adc *adc;

	list_for_each_entry(adc, &common->adc_list, adc_list)
		stm32_adc_unregister(adc);
	clk_disable_unprepare(common->aclk);
	regulator_disable(common->vref);

	return 0;
}
EXPORT_SYMBOL_GPL(stm32_adc_remove);

MODULE_AUTHOR("Fabrice Gasnier <fabrice.gasnier@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 ADC driver");
MODULE_LICENSE("GPL v2");
