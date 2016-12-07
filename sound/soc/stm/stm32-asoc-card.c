/*
 * Generic STM32 ASoC Sound Card driver
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
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/nau8822.h"

/*
 * Default DAI format:
 * - I2S mode
 * - normal bit clock (BCLK) + frame (FRM)
 */
#define DAI_FMT_BASE	(SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF)

/*
 * CODEC private data
 *
 * @mclk_freq: Clock rate of MCLK
 * @mclk_id: MCLK (or main clock) id for set_sysclk()
 */
struct codec_priv {
	unsigned long mclk_freq;
	u32 mclk_id;
	u32 pll_id;
};

/*
 * Generic ASOC card private data
 *
 * @dai_link[1]: DAI link structure
 * @pdev: platform device pointer
 * @codec_priv: CODEC private data
 * @card: ASoC card structure
 * @sample_rate: Current sample rate
 * @sample_format: Current sample format
 * @dai_fmt: DAI format between CPU and CODEC
 * @name: Card name
 */
struct stm32_asoc_card_priv {
	struct snd_soc_dai_link dai_link[1];
	struct platform_device *pdev;
	struct codec_priv codec_priv;
	struct snd_soc_card card;
	u32 sample_rate;
	u32 sample_format;
	u32 dai_fmt;
	char name[32];
};

static int stm32_asoc_card_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct stm32_asoc_card_priv *priv = snd_soc_card_get_drvdata(rtd->card);

	priv->sample_rate = params_rate(params);
	priv->sample_format = params_format(params);

	/*
	 * If codec-dai is DAI Master and all configurations are already in the
	 * set_bias_level(), bypass the remaining settings in hw_params().
	 * Note: (dai_fmt & CBM_CFM) includes CBM_CFM and CBM_CFS.
	 */

	if (priv->card.set_bias_level && (priv->dai_fmt & SND_SOC_DAIFMT_CBM_CFM))
		return 0;

	/* Specific configurations of DAIs not supported */
	return -EINVAL;
}

static struct snd_soc_ops stm32_asoc_card_ops = {
	.hw_params = stm32_asoc_card_hw_params,
};

static struct snd_soc_dai_link stm32_asoc_card_dai[] = {
	/* Default ASoC DAI Link*/
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.ops = &stm32_asoc_card_ops,
	},
};

static int stm32_asoc_card_set_bias_level(struct snd_soc_card *card,
					struct snd_soc_dapm_context *dapm,
					enum snd_soc_bias_level level)
{
	struct stm32_asoc_card_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct device *dev = card->dev;
	int rv;

	if (dapm->dev != codec_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (dapm->bias_level != SND_SOC_BIAS_STANDBY)
			break;

		/*
		 * Want to set:
		 * - IMCLK = 256 * Fs
		 * - BLCK  = IMCLK (default)
		 */
		rv = snd_soc_dai_set_clkdiv(codec_dai, NAU8822_IMCLKRATE,
					     priv->sample_rate);
		if (rv) {
			dev_err(dev, "failed to set IMCLK: %d\n", rv);
			goto out;
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		if (dapm->bias_level != SND_SOC_BIAS_PREPARE)
			break;
		break;

	default:
		break;
	}

	rv = 0;
out:
	return rv;
}

static int stm32_asoc_card_late_probe(struct snd_soc_card *card)
{
	struct stm32_asoc_card_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct codec_priv *codec_priv = &priv->codec_priv;
	struct device *dev = card->dev;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, codec_priv->pll_id,
				     codec_priv->mclk_freq, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "failed to set sysclk in %s\n", __func__);
		return ret;
	}

	return 0;
}

static int stm32_asoc_card_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np;
	struct device_node *np = pdev->dev.of_node;
	struct platform_device *cpu_pdev;
	struct stm32_asoc_card_priv *priv;
	struct i2c_client *codec_dev;
	struct clk *codec_clk;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	cpu_np = of_parse_phandle(np, "audio-cpu", 0);
	codec_np = of_parse_phandle(np, "audio-codec", 0);
	if (!cpu_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find CPU DAI device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	/* Get the MCLK rate only, and leave it controlled by CODEC drivers */
	codec_clk = clk_get(&codec_dev->dev, NULL);
	if (!IS_ERR(codec_clk)) {
		priv->codec_priv.mclk_freq = clk_get_rate(codec_clk);
		clk_put(codec_clk);
	}

	/* Default sample rate and format, will be updated in hw_params() */
	priv->sample_rate = 44100;
	priv->sample_format = SNDRV_PCM_FORMAT_S16_LE;

	/* Assign a default DAI format, and allow each card to overwrite it */
	priv->dai_fmt = DAI_FMT_BASE;

	/* Diversify the card configurations */
	if (of_device_is_compatible(np, "st,stm32-audio-nau8822")) {
		priv->card.set_bias_level = stm32_asoc_card_set_bias_level;
		priv->codec_priv.mclk_id = NAU8822_MCLK;
		priv->codec_priv.pll_id = NAU8822_PLL;
		priv->dai_fmt |= SND_SOC_DAIFMT_CBM_CFM;
	} else {
		dev_err(&pdev->dev, "unknown Device Tree compatible\n");
		return -EINVAL;
	}

	sprintf(priv->name, "%s-audio", codec_dev->name);

	/* Initialize sound card */
	priv->pdev = pdev;
	priv->card.dev = &pdev->dev;
	priv->card.name = priv->name;
	priv->card.dai_link = priv->dai_link;
	priv->card.late_probe = stm32_asoc_card_late_probe;

	memcpy(priv->dai_link, stm32_asoc_card_dai,
	       sizeof(struct snd_soc_dai_link) * ARRAY_SIZE(priv->dai_link));

	/* Normal DAI Link */
	priv->dai_link[0].cpu_of_node = cpu_np;
	priv->dai_link[0].codec_of_node = codec_np;
	priv->dai_link[0].codec_dai_name = codec_dev->name;
	priv->dai_link[0].platform_of_node = cpu_np;
	priv->dai_link[0].dai_fmt = priv->dai_fmt;
	priv->card.num_links = 1;

	/* Finish card registering */
	platform_set_drvdata(pdev, priv);
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

fail:
	of_node_put(codec_np);
	of_node_put(cpu_np);

	return ret;
}

static const struct of_device_id stm32_asoc_card_dt_ids[] = {
	{ .compatible = "st,stm32-audio-nau8822", },
	{}
};

static struct platform_driver stm32_asoc_card_driver = {
	.probe = stm32_asoc_card_probe,
	.driver = {
		.name = "stm32-asoc-card",
		.pm = &snd_soc_pm_ops,
		.of_match_table = stm32_asoc_card_dt_ids,
	},
};
module_platform_driver(stm32_asoc_card_driver);

MODULE_DESCRIPTION("Generic STM32 ASoC Sound Card driver");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_ALIAS("platform:stm32-asoc-card");
MODULE_LICENSE("GPL");
