/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/err.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <dt-bindings/reset/lpc17xx-resets.h>
#include <linux/delay.h>

#define BITS_PER_REG	32

struct lpc17xx_reset {
	void __iomem *reg_base;
	struct reset_controller_dev rcdev;
};

static int lpc17xx_reset_reset(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct lpc17xx_reset *data =
		container_of(rcdev, struct lpc17xx_reset, rcdev);
	void __iomem *reg_addr = data->reg_base + ((id < BITS_PER_REG) ? 0 : 4);
	unsigned int bit = id % BITS_PER_REG;
	u32 tmp;

	if (id >= RST_END)
		return -EINVAL;

	tmp = readl(reg_addr);
	writel(tmp | BIT(bit), reg_addr);
	udelay(1);
	writel(tmp & ~BIT(bit), reg_addr);

	return 0;
}

static struct reset_control_ops lpc17xx_reset_ops = {
	.reset		= lpc17xx_reset_reset,
};

static const struct of_device_id lpc17xx_reset_dt_ids[] = {
	{ .compatible = "nxp,lpc17xx-reset", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, lpc17xx_reset_dt_ids);

static int lpc17xx_reset_probe(struct platform_device *pdev)
{
	struct lpc17xx_reset *data;
	struct resource *res;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(data->reg_base))
		return data->reg_base ? PTR_ERR(data->reg_base) : -ENXIO;

	platform_set_drvdata(pdev, data);

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = RST_END;
	data->rcdev.ops = &lpc17xx_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;

	return reset_controller_register(&data->rcdev);
}

static struct platform_driver lpc17xx_reset_driver = {
	.probe	= lpc17xx_reset_probe,
	.driver = {
		.name		= "lpc17xx_reset",
		.of_match_table	= lpc17xx_reset_dt_ids,
	},
};

module_platform_driver(lpc17xx_reset_driver);
