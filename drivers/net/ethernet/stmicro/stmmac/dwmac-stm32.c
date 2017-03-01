/*
 * DWMAC glue for STM32F2xx, STM32F4xx, STM32F7xx Ethernet
 *
 * Copyright (C) 2015 Yuri Tikhonov <yur@emcraft.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/stmmac.h>

#include <mach/syscfg.h>

#include "stmmac_platform.h"

struct stm32_dwmac_priv_data {
	int interface;
	struct regmap *reg;
	struct clk *tx_clk;
	struct clk *rx_clk;
};

static void *stm32_dwmac_setup(struct platform_device *pdev)
{
	struct stm32_dwmac_priv_data *dwmac;
	struct device *dev = &pdev->dev;

	dwmac = devm_kzalloc(dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return ERR_PTR(-ENOMEM);

	dwmac->interface = of_get_phy_mode(dev->of_node);
	if (dwmac->interface < 0) {
		dev_err(dev, "phy-mode lookup failed\n");
		return ERR_PTR(dwmac->interface);
	}

	dwmac->reg = syscon_regmap_lookup_by_compatible("st,stm32-syscfg");
	if (IS_ERR(dwmac->reg)) {
		dev_err(dev, "syscfg regs lookup failed\n");
		return dwmac->reg;
	}

	dwmac->tx_clk = devm_clk_get(dev, "stm32_mac_tx");
	if (IS_ERR(dwmac->tx_clk)) {
		dev_err(dev, "could not get tx clock");
		return dwmac->tx_clk;
	}

	dwmac->rx_clk = devm_clk_get(dev, "stm32_mac_rx");
	if (IS_ERR(dwmac->rx_clk)) {
		dev_err(dev, "could not get rx clock");
		return dwmac->rx_clk;
	}

	return dwmac;
}

static int stm32_dwmac_init(struct platform_device *pdev, void *priv)
{
	struct stm32_dwmac_priv_data *dwmac = priv;
	u32 ethmode;

	if (dwmac->interface == PHY_INTERFACE_MODE_MII) {
		ethmode = STM32_SYSCFG_PMC_ETHMODE_MII;
	} else if (dwmac->interface == PHY_INTERFACE_MODE_RMII) {
		ethmode = STM32_SYSCFG_PMC_ETHMODE_RMII;
	} else {
		dev_err(&pdev->dev, "Only MII and RMII mode supported\n");
		return -EINVAL;
	}

	regmap_update_bits(dwmac->reg, STM32_SYSCFG_PMC,
			   STM32_SYSCFG_PMC_ETHMODE_MASK, ethmode);

	clk_prepare_enable(dwmac->tx_clk);
	clk_prepare_enable(dwmac->rx_clk);

	return 0;
}

static void stm32_dwmac_exit(struct platform_device *pdev, void *priv)
{
	struct stm32_dwmac_priv_data *dwmac = priv;

	clk_disable_unprepare(dwmac->rx_clk);
	clk_disable_unprepare(dwmac->tx_clk);
}

static const struct stmmac_of_data stm32_dwmac_data = {
	.has_gmac = 1,
	.setup = stm32_dwmac_setup,
	.init = stm32_dwmac_init,
	.exit = stm32_dwmac_exit,
};

static const struct of_device_id stm32_dwmac_match[] = {
	{ .compatible = "stm,stm32-dwmac", .data = &stm32_dwmac_data },
	{ }
};
MODULE_DEVICE_TABLE(of, stm32_dwmac_match);

static struct platform_driver stm32_dwmac_driver = {
	.probe  = stmmac_pltfr_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "stm32-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = stm32_dwmac_match,
	},
};
module_platform_driver(stm32_dwmac_driver);

MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_DESCRIPTION("DWMAC glue for STM32F2/4/7 Ethernet");
MODULE_LICENSE("GPL v2");
