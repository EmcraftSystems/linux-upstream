/*
 * ALSA SoC Digital Audio Interface (SAI) driver.
 *
 * (C) Copyright 2016
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

/* Define this to enable debug prints */
#undef STM32_SAI_DBG

/* xSAI registers */
#define STM32_SAI_REG(x, r)		((x) ? ((r) + 0x20) : (r))

/* xCR1 register */
#define STM32_SAI_CR1(x)		STM32_SAI_REG(x, 0x04)
#define STM32_SAI_CR1_DMAEN		(1 << 17)
#define STM32_SAI_CR1_SAIXEN		(1 << 16)
#define STM32_SAI_CR1_SYNCEN(x)		((x) << 11)
#define STM32_SAI_CR1_SYNCEN_MASK	STM32_SAI_CR1_SYNCEN(3)
#define STM32_SAI_CR1_SYNCEN_ASYNC	STM32_SAI_CR1_SYNCEN(0)
#define STM32_SAI_CR1_SYNCEN_ISYNC	STM32_SAI_CR1_SYNCEN(1)
#define STM32_SAI_CR1_CKSTR		(1 << 9)
#define STM32_SAI_CR1_DS(x)		((x) << 5)
#define STM32_SAI_CR1_DS_MASK		STM32_SAI_CR1_DS(7)
#define STM32_SAI_CR1_DS_8		STM32_SAI_CR1_DS(2)
#define STM32_SAI_CR1_DS_16		STM32_SAI_CR1_DS(4)
#define STM32_SAI_CR1_DS_32		STM32_SAI_CR1_DS(7)
#define STM32_SAI_CR1_PRTCFG(x)		((x) << 2)
#define STM32_SAI_CR1_PRTCFG_MASK	STM32_SAI_CR1_PRTCFG(3)
#define STM32_SAI_CR1_PRTCFG_FREE	STM32_SAI_CR1_PRTCFG(0)
#define STM32_SAI_CR1_MODE(x)		((x) << 0)
#define STM32_SAI_CR1_MODE_MASK		STM32_SAI_CR1_MODE(3)
#define STM32_SAI_CR1_MODE_STX		STM32_SAI_CR1_MODE(2)

/* xCR2 register */
#define STM32_SAI_CR2(x)		STM32_SAI_REG(x, 0x08)
#define STM32_SAI_CR2_FTH(x)		((x) << 0)
#define STM32_SAI_CR2_FTH_FULL		STM32_SAI_CR2_FTH(4)

/* xFRCR register */
#define STM32_SAI_FRCR(x)		STM32_SAI_REG(x, 0x0C)
#define STM32_SAI_FRCR_FSOFF		(1 << 18)
#define STM32_SAI_FRCR_FSDEF		(1 << 16)
#define STM32_SAI_FRCR_FSALL(x)		((x) << 8)
#define STM32_SAI_FRCR_FSALL_MASK	STM32_SAI_FRCR_FSALL(0x7F)
#define STM32_SAI_FRCR_FRL(x)		((x) << 0)
#define STM32_SAI_FRCR_FRL_MASK		STM32_SAI_FRCR_FRL(0xFF)

/* xSLOTR register */
#define STM32_SAI_SLOTR(x)		STM32_SAI_REG(x, 0x10)
#define STM32_SAI_SLOTR_SLOTEN(x)	((x) << 16)
#define STM32_SAI_SLOTR_SLOTEN_MASK	STM32_SAI_SLOTR_SLOTEN(0xFFFF)
#define STM32_SAI_SLOTR_NBSLOT(x)	((x) << 8)
#define STM32_SAI_SLOTR_NBSLOT_MASK	STM32_SAI_SLOTR_NBSLOT(0xF)
#define STM32_SAI_SLOTR_SLOTSZ(x)	((x) << 6)
#define STM32_SAI_SLOTR_SLOTSZ_MASK	STM32_SAI_SLOTR_SLOTSZ(0x3)
#define STM32_SAI_SLOTR_SLOTSZ_16	STM32_SAI_SLOTR_SLOTSZ(0x1)
#define STM32_SAI_SLOTR_SLOTSZ_32	STM32_SAI_SLOTR_SLOTSZ(0x2)

/* xIM register */
#define STM32_SAI_IM(x)			STM32_SAI_REG(x, 0x14)
#define STM32_SAI_IM_LFSDETIE		(1 << 6)
#define STM32_SAI_IM_AFSDETIE		(1 << 5)
#define STM32_SAI_IM_OVRUDRIE		(1 << 0)
#define STM32_SAI_IM_ERRORS		(STM32_SAI_IM_LFSDETIE | \
					 STM32_SAI_IM_AFSDETIE | \
					 STM32_SAI_IM_OVRUDRIE)

/* ASR/ACLRFR registers */
#define STM32_SAI_ASR_REG		0x18
#define STM32_SAI_ACLRFR_REG		0x1C

/* xDR register */
#define STM32_SAI_DR(x)			STM32_SAI_REG(x, 0x20)

/* General device struct */
struct stm32_sai_dev {
	void __iomem				*reg;
	struct snd_dmaengine_dai_dma_data	dma_data[2];
	struct reset_control			*rst;
	struct device				*dev;
	struct clk				*clk;
	int					irq;

	u32					fmt;
	u32					bclk_ratio;
};

static void stm32_sai_reg_dump(const char *who, struct stm32_sai_dev *sai)
{
	printk("Dump SAI reg (%s). GCR: %08x\n",
		who, readl(sai->reg + 0x0));
	printk(" A CR1:%08x;CR2:%08x;FRCR:%08x;SLOTR:%08x;IM:%08x;SR:%08x\n",
		readl(sai->reg + 0x04), readl(sai->reg + 0x08),
		readl(sai->reg + 0x0c), readl(sai->reg + 0x10),
		readl(sai->reg + 0x14), readl(sai->reg + 0x18));
	printk(" B CR1:%08x;CR2:%08x;FRCR:%08x;SLOTR:%08x;IM:%08x;SR:%08x\n",
		readl(sai->reg + 0x24), readl(sai->reg + 0x28),
		readl(sai->reg + 0x2c), readl(sai->reg + 0x30),
		readl(sai->reg + 0x34), readl(sai->reg + 0x38));
}

static irqreturn_t stm32_sai_irq(int irq, void *dev_id)
{
	struct stm32_sai_dev *sai = dev_id;
	u32 val;
	int i;

	for (i = 0; i < 2; i++) {
		val = readl(sai->reg + STM32_SAI_REG(i, STM32_SAI_ASR_REG));
		if (val)
			stm32_sai_reg_dump(__func__, sai);
		writel(val, sai->reg + STM32_SAI_REG(i, STM32_SAI_ACLRFR_REG));
	}

	return IRQ_HANDLED;
}

static int stm32_sai_set_fmt(struct snd_soc_dai *dai, u32 fmt)
{
	struct stm32_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	u32 val, dir = SNDRV_PCM_STREAM_PLAYBACK;
	int rv;

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* I2S mode */
		val = STM32_SAI_CR1_SYNCEN_ASYNC | STM32_SAI_CR1_MODE_STX;
		val |= STM32_SAI_CR1_PRTCFG_FREE;
		val |= STM32_SAI_CR1_CKSTR;
		writel(val, sai->reg + STM32_SAI_CR1(dir));

		val = readl(sai->reg + STM32_SAI_FRCR(dir));
		val |= STM32_SAI_FRCR_FSOFF | STM32_SAI_FRCR_FSDEF;
		writel(val, sai->reg + STM32_SAI_FRCR(dir));
		break;
	default:
		dev_err(sai->dev, "fmt %x not supported\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		rv = -EINVAL;
		goto out;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		/* Nothing to do for both normal cases */
		break;
	default:
		dev_err(sai->dev, "clk inv %x not supported\n",
			fmt & SND_SOC_DAIFMT_INV_MASK);
		rv = -EINVAL;
		goto out;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* codec clk & FRM master */
		break;
	default:
		dev_err(sai->dev, "clk %x not supported\n",
			fmt & SND_SOC_DAIFMT_MASTER_MASK);
		rv = -EINVAL;
		goto out;
	}

	/* Configure FIFO */
	val = STM32_SAI_CR2_FTH_FULL;
	writel(val, sai->reg + STM32_SAI_CR2(dir));

	/* Enable error interrupts */
	val = STM32_SAI_IM_ERRORS;
	writel(val, sai->reg + STM32_SAI_IM(dir));

	sai->fmt = fmt;
	rv = 0;
out:
	return rv;
}

static int stm32_sai_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct stm32_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	u32 word_width, chan_num;
	u32 cr, frcr, slotr, dir = substream->stream;
	int rv;

	cr = readl(sai->reg + STM32_SAI_CR1(dir));
	if (cr & STM32_SAI_CR1_SAIXEN) {
		cr &= ~STM32_SAI_CR1_SAIXEN;
		writel(cr, sai->reg + STM32_SAI_CR1(dir));
	}
	cr &= ~STM32_SAI_CR1_DS_MASK;

	slotr = readl(sai->reg + STM32_SAI_SLOTR(dir));
	slotr &= ~STM32_SAI_SLOTR_SLOTSZ_MASK;

	word_width = snd_pcm_format_width(params_format(params));
	switch (word_width) {
	case 8:
		cr |= STM32_SAI_CR1_DS_8;
		break;
	case 16:
		cr |= STM32_SAI_CR1_DS_16;
		slotr |= STM32_SAI_SLOTR_SLOTSZ_16;
		break;
	case 32:
		cr |= STM32_SAI_CR1_DS_32;
		slotr |= STM32_SAI_SLOTR_SLOTSZ_32;
		break;
	default:
		dev_err(sai->dev, "word width %d not supported\n", word_width);
		rv = -EINVAL;
		goto out;
	}
	writel(cr, sai->reg + STM32_SAI_CR1(dir));

	frcr = readl(sai->reg + STM32_SAI_FRCR(dir));
	frcr &= ~(STM32_SAI_FRCR_FRL_MASK | STM32_SAI_FRCR_FSALL_MASK);

	slotr &= ~STM32_SAI_SLOTR_NBSLOT_MASK;

	chan_num = params_channels(params);
	if (chan_num > 2) {
		dev_err(sai->dev, "%d channels not supported\n", chan_num);
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Assume 256 clocks / 16 slots per audio frame
	 */
	frcr |= STM32_SAI_FRCR_FSALL(127);
	frcr |= STM32_SAI_FRCR_FRL(255);
	if (chan_num == 2) {
		/*
		 * Stereo: play 1st slot in Left, and 1st slot in Right
		 */
		slotr |= STM32_SAI_SLOTR_NBSLOT(15);
		slotr |= STM32_SAI_SLOTR_SLOTEN((1 << 8) | (1 << 0));
	} else {
		/*
		 * Mono: play 1st slot in Left
		 */
		slotr |= STM32_SAI_SLOTR_NBSLOT(1);
		slotr |= STM32_SAI_SLOTR_SLOTEN(1 << 0);
	}

	writel(frcr, sai->reg + STM32_SAI_FRCR(dir));
	writel(slotr, sai->reg + STM32_SAI_SLOTR(dir));

	rv = 0;
out:
	return rv;
}

static int stm32_sai_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct stm32_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	u32 val, dir = substream->stream;

	val = readl(sai->reg + STM32_SAI_CR1(dir));
	val |= STM32_SAI_CR1_DMAEN;
	writel(val, sai->reg + STM32_SAI_CR1(dir));

	return 0;
}

static void stm32_sai_stop(struct stm32_sai_dev *sai,
			   struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	u32 val, dir = substream->stream;

	val = readl(sai->reg + STM32_SAI_CR1(dir));
	val &= ~STM32_SAI_CR1_SAIXEN;
	writel(val, sai->reg + STM32_SAI_CR1(dir));
}

static int stm32_sai_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct stm32_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	u32 val, dir = substream->stream;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		val = readl(sai->reg + STM32_SAI_CR1(dir));
		val |= STM32_SAI_CR1_SAIXEN;
		writel(val, sai->reg + STM32_SAI_CR1(dir));

#if defined(STM32_SAI_DBG)
		stm32_sai_reg_dump(__func__, sai);
#endif
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		stm32_sai_stop(sai, substream, dai);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops stm32_sai_dai_ops = {
	.prepare	= stm32_sai_prepare,
	.trigger	= stm32_sai_trigger,
	.hw_params	= stm32_sai_hw_params,
	.set_fmt	= stm32_sai_set_fmt,
};

static int stm32_sai_dai_probe(struct snd_soc_dai *dai)
{
	struct stm32_sai_dev *sai = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
			&sai->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
			NULL);

	return 0;
}

static struct snd_soc_dai_driver stm32_sai_dai = {
	.name	= "stm32-sai",
	.probe	= stm32_sai_dai_probe,
	.ops	= &stm32_sai_dai_ops,
	.playback = {
		.stream_name = "CPU-Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates =	SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000,
		.formats =	SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static const struct snd_soc_component_driver stm32_sai_component = {
	.name		= "stm32-sai-comp",
};

static int stm32_sai_probe(struct platform_device *pdev)
{
	struct stm32_sai_dev *sai = NULL;
	struct resource *mem = NULL;
	struct device *dev = &pdev->dev;
	int rv;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "no register base provided\n");
		rv = -ENXIO;
		goto out;
	}

	sai = devm_kzalloc(dev, sizeof(*dev), GFP_KERNEL);
	if (!sai) {
		dev_err(dev, "unable to allocate memory\n");
		rv = -ENOMEM;
		goto out;
	}

	sai->irq = irq_of_parse_and_map(dev->of_node, 0);
	if (sai->irq < 0) {
		dev_err(dev, "bad IRQ\n");
		rv = -EINVAL;
		goto out;
	}
	rv = request_irq(sai->irq, stm32_sai_irq, 0, dev_name(dev), sai);
	if (rv) {
		dev_err(dev," request irq %d failed\n",
			sai->irq);
		goto out;
	}

	if (!request_mem_region(mem->start, resource_size(mem),
				mem->name)) {
		dev_err(dev, "registers already in use\n");
		rv = -EINVAL;
		goto out;
	}
	sai->reg = ioremap(mem->start, resource_size(mem));
	if (!sai->reg) {
		dev_err(dev, "unable to map registers\n");
		rv = -EINVAL;
		goto out;
	}

	sai->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sai->clk)) {
		dev_err(dev, "no clock specified\n");
		rv = -EINVAL;
		goto out;
	}

	sai->rst = devm_reset_control_get(dev, NULL);
	if (IS_ERR(sai->rst)) {
		dev_err(dev, "no reset controller specified\n");
		rv = -EINVAL;
		goto out;
	}

	/* Set the DMA address, bus width, and burst */
	sai->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr =
		(dma_addr_t)(sai->reg + STM32_SAI_DR(SNDRV_PCM_STREAM_PLAYBACK));
	sai->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr_width =
		DMA_SLAVE_BUSWIDTH_2_BYTES;
	sai->dma_data[SNDRV_PCM_STREAM_PLAYBACK].maxburst = 1;

	/* Store the pdev */
	sai->dev = dev;
	dev_set_drvdata(dev, sai);

	rv = devm_snd_soc_register_component(dev,
			&stm32_sai_component, &stm32_sai_dai, 1);
	if (rv) {
		dev_err(dev, "Could not register DAI: %d\n", rv);
		goto out;
	}

	rv = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
	if (rv) {
		dev_err(dev, "Could not register PCM: %d\n", rv);
		goto out;
	}

	/* Reset SAI, and enable block clocking */
	reset_control_assert(sai->rst);
	reset_control_deassert(sai->rst);
	clk_prepare_enable(sai->clk);

	rv = 0;
out:
	if (rv && sai) {
		devm_kfree(dev, sai);

		if (!(sai->irq < 0))
			free_irq(sai->irq, sai);
	}

	return rv;
}

static const struct of_device_id stm32_sai_of_match[] = {
	{ .compatible = "st,stm32f7-sai", },
	{},
};

static struct platform_driver stm32_sai_driver = {
	.probe		= stm32_sai_probe,
	.driver		= {
		.name	= "stm32-sai",
		.of_match_table = stm32_sai_of_match,
	},
};

module_platform_driver(stm32_sai_driver);

MODULE_DESCRIPTION("STM32 SoC SAI Interface");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_ALIAS("platform:stm32-sai");
MODULE_LICENSE("GPL");
