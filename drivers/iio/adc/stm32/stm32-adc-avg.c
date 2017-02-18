/*
 * (C) Copyright 2017
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

/*
 * STM32 ADC average driver
 */

#include <linux/iio/iio.h>
#include <linux/dma-mapping.h>

#include "stm32-adc.h"

static int avg_dma_init(struct stm32_adc_common *com);

/**
 * avg_adc_process() - process ADC data collected
 * @adc:	ADC identifier
 * @chan:	Channel identifier
 * @data:	Raw data samples
 * @num:	Number of samples
 */
static int avg_adc_process(int adc, int chan, int *data, int num)
{
	int i, val = 0;

	for (i = 0; i < num; i++)
		val += data[i];
	val /= num;

	return val;
}

/**
 * avg_dma_buf_done() - DMA completion callback
 * @data:	ADC common structure pointer
 */
static void avg_dma_buf_done(void *data)
{
	struct stm32_adc_common *com = data;
	struct stm32_avg *avg = &com->avg;
	const struct iio_chan_spec *chan;
	struct stm32_adc *adc;
	struct iio_dev *iio;
	unsigned long flags;
	int i, j, k, val;
	u32 *p;

	if (avg->dma_cookie < 0) {
		/*
		 * DMA terminated (overrun). Reinit DMA, clear overruns, and
		 * restart conversion
		 */
		val = avg_dma_init(com);
		if (val) {
			dev_err(com->dev, "Avg DMA reinit error\n");
			goto out;
		}

		list_for_each_entry(adc, &com->adc_list, adc_list)
			if (!adc->injected)
				stm32_adc_recover(adc);

		val = stm32_adc_avg_run(com);
		if (val) {
			dev_err(com->dev, "Avg rerun error\n");
			goto out;
		}

		avg->ovr_msk = 0;
		goto out;
	}

	/*
	 * Collect raw data
	 */
	p = &avg->dma_buf[avg->raw_pos / sizeof(u32)];
	avg->raw_pos = (avg->raw_pos + avg->raw_period) % avg->raw_length;

	for (i = 0, k = 0; i < avg->num; i++) {
		for (j = 0; j < avg->chan_num; j++) {
			list_for_each_entry(adc, &com->adc_list, adc_list) {
				if (adc->injected)
					continue;
				iio = iio_priv_to_dev(adc);
				if (j < iio->num_channels)
					avg->raw_buf[adc->id][j][i] = p[k];
				else
					avg->raw_buf[adc->id][0][i] += p[k];
				k++;
			}
		}
	}

	/*
	 * Average data from duplicating channels
	 */
	list_for_each_entry(adc, &com->adc_list, adc_list) {
		if (adc->injected)
			continue;

		iio = iio_priv_to_dev(adc);
		if (iio->num_channels == avg->chan_num)
			continue;

		for (i = 0; i < avg->num; i++) {
			avg->raw_buf[adc->id][0][i] /=
				avg->chan_num - iio->num_channels + 1;
		}
	}

	/*
	 * Process data collected, and put result to the interface buffer
	 */
	list_for_each_entry(adc, &com->adc_list, adc_list) {
		if (adc->injected)
			continue;
		iio = iio_priv_to_dev(adc);
		for (j = 0; j < iio->num_channels; j++) {
			chan = &iio->channels[j];

			val = avg_adc_process(adc->id, chan->channel,
					avg->raw_buf[adc->id][j], avg->num);

			spin_lock_irqsave(&avg->lock, flags);
			avg->flt_buf[adc->id][chan->channel] = val;
			spin_unlock_irqrestore(&avg->lock, flags);
		}
	}
out:
	return;
}

/**
 * avg_scan_seq_init() - build scan sequence
 * @com:	ADC common structure pointer
 */
static int avg_scan_seq_init(struct stm32_adc_common *com)
{
	const struct stm32_adc_regs *sq;
	const struct stm32_adc_regs *smp;
	struct stm32_avg *avg = &com->avg;
	struct stm32_adc *adc;
	u32 *p, val;
	int rv;

	/*
	 * Program each ADC to run `chan_num` conversions. If ADC has less than
	 * `chan_num` chans, then extend sequence with the 1st chan
	 */
	sq = com->data->adc_reginfo->sqr_regs;
	smp = com->data->adc_reginfo->smpr_regs;
	list_for_each_entry(adc, &com->adc_list, adc_list) {
		struct iio_dev *iio;
		int i, x;

		if (adc->injected)
			continue;

		iio = iio_priv_to_dev(adc);
		for (i = 0; i < avg->chan_num; i++) {
			const struct iio_chan_spec *chan;

			if (i < iio->num_channels) {
				struct stm32_adc_chan *stm32_chan;

				/*
				 * Allocate buffer for raw measurements
				 */
				p = kmalloc(avg->num * sizeof(u32), GFP_KERNEL);
				if (!p) {
					dev_err(com->dev,
						"%s: raw_buf alloc error\n",
						__func__);
					rv = -ENOMEM;
					goto out;
				}

				avg->raw_buf[adc->id][i] = p;
				chan = &iio->channels[i];

				/*
				 * Configure sampling time
				 */
				x = chan->channel;
				stm32_chan = to_stm32_chan(adc, chan);
				val = stm32_adc_readl(adc, smp[x].reg);
				val &= ~smp[x].mask;
				val |= (stm32_chan->smpr << smp[x].shift) &
					smp[x].mask;
				stm32_adc_writel(adc, smp[x].reg, val);
			} else {
				chan = &iio->channels[0];
			}

			/*
			 * Set-up next sequence item
			 */
			x = i + 1;
			val = stm32_adc_readl(adc, sq[x].reg);
			val &= ~sq[x].mask;
			val |= (chan->channel << sq[x].shift) & sq[x].mask;
			stm32_adc_writel(adc, sq[x].reg, val);
		}

		/* Set-up sequence len */
		val = stm32_adc_readl(adc, sq[0].reg);
		val &= ~sq[0].mask;
		val |= ((i - 1) << sq[0].shift) & sq[0].mask;
		stm32_adc_writel(adc, sq[0].reg, val);
	}

	/*
	 * Triple mode: ADC1, 2 and 3 working together
	 * Combined regular simultaneous + alternate trigger mode
	 */
	rv = stm32_adc_multi_dma_sel(com, STM32_DM_DMA1,
				     STM32_MM_TRIPLE_CRS_ATM);
out:
	return rv;
}

/**
 * avg_dma_init() - init and run DMA
 * @com:	ADC common structure pointer
 */
static int avg_dma_init(struct stm32_adc_common *com)
{
	struct stm32_avg *avg = &com->avg;
	struct dma_chan *dma = avg->dma;
	const struct stm32_adc_reginfo *reginfo = com->data->adc_reginfo;
	struct dma_async_tx_descriptor *dsc;
	struct dma_slave_config cfg;
	dma_cookie_t cookie;
	int rv;

	/*
	 * Allocate at least twice the buffer size for dma cyclic transfers, so
	 * we can work with at least two dma periods. Allocate in `u32` units
	 * since in ADC Simultaneous mode DMA operates with 32-bit transfers.
	 */
	avg->raw_pos = 0;
	if (!avg->dma_buf) {
		avg->raw_period = avg->num * 3 * avg->chan_num * sizeof(u32);
		avg->raw_length = avg->raw_period * 2;
		avg->dma_buf = dma_alloc_coherent(com->dev,
						  PAGE_ALIGN(avg->raw_length),
						  &avg->dma_adr, GFP_KERNEL);
		if (!avg->dma_buf) {
			dev_err(com->dev, "DMA buf allocation failed\n");
			rv = -ENOMEM;
			goto out;
		}
	}

	/* Configure DMA channel to read data register */
	memset(&cfg, 0, sizeof(cfg));
	cfg.src_addr = (dma_addr_t)com->phys_base + reginfo->cdr;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	rv = dmaengine_slave_config(dma, &cfg);
	if (rv) {
		dev_err(com->dev, "DMA slave cfg failed (%d)\n", rv);
		goto out;
	}

	/* Prepare a DMA cyclic transaction */
	dsc = dmaengine_prep_dma_cyclic(dma, avg->dma_adr,
					avg->raw_length, avg->raw_period,
					DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!dsc) {
		dev_err(com->dev, "DMA prep cyclic failed\n");
		rv = -ENODEV;
		goto out;
	}

	dsc->callback = avg_dma_buf_done;
	dsc->callback_param = com;

	cookie = dmaengine_submit(dsc);
	rv = dma_submit_error(cookie);
	if (rv) {
		dev_err(com->dev, "DMA submit error (%d)\n", rv);
		goto out;
	}

	/* Issue pending DMA requests */
	avg->dma_cookie = cookie;
	dma_async_issue_pending(dma);
	rv = 0;
out:
	if (rv) {
		if (avg->dma_buf) {
			dma_free_coherent(com->dev, avg->raw_length,
					  avg->dma_buf, avg->dma_adr);
			avg->dma_buf = NULL;
		}
	}

	return rv;
}

/**
 * avg_ovf_task() - overflow processing tasklet
 * @arg:       ADC common structure pointer
 */
static void avg_ovf_task(unsigned long arg)
{
	struct stm32_adc_common *com = (void *)arg;
	unsigned long flags;

	local_irq_save(flags);
	avg_dma_buf_done(com);
	local_irq_restore(flags);
}

/**
 * stm32_adc_avg_init() - init the averaging process
 * @com:	ADC common structure pointer
 */
int stm32_adc_avg_init(struct stm32_adc_common *com)
{
	struct stm32_adc *adc;
	struct stm32_avg *avg = &com->avg;
	struct iio_dev *iio;
	int adc_num, chan_num;
	int rv;

	/*
	 * Check configuration. Count number of `regular` ADCs, get
	 * maximum channels number
	 */
	adc_num = chan_num = 0;
	list_for_each_entry(adc, &com->adc_list, adc_list) {
		if (adc->injected)
			continue;

		iio = iio_priv_to_dev(adc);
		if (!iio->num_channels)
			continue;

		adc_num++;
		if (iio->num_channels > chan_num)
			chan_num = iio->num_channels;

		rv = stm32_adc_clk_sel(adc);
		if (rv) {
			dev_err(com->dev, "Clock sel failed\n");
			goto out;
		}

		rv = stm32_adc_enable(adc);
		if (rv) {
			dev_err(com->dev, "Failed to enable adc\n");
			goto out;
		}

		/* ADC1 DMA processes all xfers in triple mode */
		if (adc->id != 0)
			continue;

		if (!adc->dma_chan) {
			dev_err(com->dev, "No DMA found for avg\n");
			rv = -EINVAL;
			goto out;
		}

		avg->dma = adc->dma_chan;
	}

	if (adc_num != 3 || chan_num > STM32_ADC_MAX_SQ) {
		dev_err(com->dev, "Avg supports the `triple` mode only\n");
		rv = -EINVAL;
		goto out;
	}

	avg->chan_num = chan_num;
	spin_lock_init(&avg->lock);
	tasklet_init(&avg->task, avg_ovf_task, (unsigned long)com);

	rv = avg_scan_seq_init(com);
	if (rv) {
		dev_err(com->dev, "Avg build scan seq error\n");
		goto out;
	}

	rv = avg_dma_init(com);
	if (rv) {
		dev_err(com->dev, "Avg DMA init error\n");
		goto out;
	}

	rv = pwm_config(avg->pwm, (avg->rate / 2) * 1000, avg->rate * 1000);
	if (rv) {
		dev_err(com->dev, "Avg timer init error\n");
		goto out;
	}

	rv = 0;
out:
	return rv;
}

/**
 * stm32_adc_avg_run() - run the averaging process
 * @com:	ADC common structure pointer
 */
int stm32_adc_avg_run(struct stm32_adc_common *com)
{
	return pwm_enable(com->avg.pwm);
}

/**
 * stm32_adc_avg_overrun() - process Overrun events
 * @adc:	STM32 ADC device
 */
int stm32_adc_avg_overrun(struct stm32_adc *adc)
{
	struct stm32_avg *avg = &adc->common->avg;

	avg->ovr_msk |= 1 << adc->id;
	if (avg->ovr_msk != 0x7)
		goto out;

	if (avg->dma_cookie < 0)
		goto out;

	pwm_disable(avg->pwm);
	dmaengine_terminate_all(avg->dma);
	avg->dma_cookie = -EINVAL;
	tasklet_schedule(&avg->task);

out:
	return 0;
}

/**
 * stm32_adc_avg_get() - get the average value
 * @indio_dev:	ADC device
 * @chan:	ADC channel
 * @val:	destination
 */
int stm32_adc_avg_get(struct iio_dev *indio_dev,
		      struct iio_chan_spec const *chan, int *val)
{
	struct stm32_adc *adc = iio_priv(indio_dev);
	struct stm32_adc_common *common = adc->common;
	struct stm32_avg *avg = &common->avg;
	unsigned long flags;

	spin_lock_irqsave(&adc->common->avg.lock, flags);
	*val = avg->flt_buf[adc->id][chan->channel] & STM32_RESULT_MASK;
	spin_unlock_irqrestore(&adc->common->avg.lock, flags);

	return IIO_VAL_INT;
}
