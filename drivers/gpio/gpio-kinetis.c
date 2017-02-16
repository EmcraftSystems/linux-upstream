/*
 * Kinetis GPIO support through PORT and GPIO module
 *
 * Copyright (c) 2017 Emcraft Systems
 *
 * Author: Dmitry Konyshev <probables@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#define KINETIS_GPIO_PER_PORT		32

struct kinetis_gpio_port {
	struct gpio_chip gc;
	void __iomem *gpio_base;
};

#define GPIO_PDOR		0x00
#define GPIO_PSOR		0x04
#define GPIO_PCOR		0x08
#define GPIO_PTOR		0x0c
#define GPIO_PDIR		0x10
#define GPIO_PDDR		0x14

static const struct of_device_id kinetis_gpio_dt_ids[] = {
	{ .compatible = "fsl,k70-gpio" },
	{ .compatible = "fsl,kinetis-gpio" },
	{ /* sentinel */ }
};

static inline void kinetis_gpio_writel(u32 val, void __iomem *reg)
{
	writel_relaxed(val, reg);
}

static inline u32 kinetis_gpio_readl(void __iomem *reg)
{
	return readl_relaxed(reg);
}

static int kinetis_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct kinetis_gpio_port *port = gpiochip_get_data(gc);

	return !!(kinetis_gpio_readl(port->gpio_base + GPIO_PDIR) & BIT(gpio));
}

static void kinetis_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct kinetis_gpio_port *port = gpiochip_get_data(gc);
	unsigned long mask = BIT(gpio);

	if (val)
		kinetis_gpio_writel(mask, port->gpio_base + GPIO_PSOR);
	else
		kinetis_gpio_writel(mask, port->gpio_base + GPIO_PCOR);
}

static int kinetis_gpio_direction_input(struct gpio_chip *gc, unsigned gpio)
{
	struct kinetis_gpio_port *port = gpiochip_get_data(gc);

	kinetis_gpio_writel(kinetis_gpio_readl(port->gpio_base + GPIO_PDDR) &
			~BIT(gpio), port->gpio_base + GPIO_PDDR);

	return 0;
}

static int kinetis_gpio_direction_output(struct gpio_chip *gc, unsigned gpio,
				       int value)
{
	struct kinetis_gpio_port *port = gpiochip_get_data(gc);

	kinetis_gpio_set(gc, gpio, value);

	kinetis_gpio_writel(kinetis_gpio_readl(port->gpio_base + GPIO_PDDR) |
			BIT(gpio), port->gpio_base + GPIO_PDDR);

	return 0;
}

static int kinetis_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct kinetis_gpio_port *port;
	struct resource *iores;
	struct gpio_chip *gc;
	int ret;

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	port->gpio_base = devm_ioremap_resource(dev, iores);
	if (IS_ERR(port->gpio_base))
		return PTR_ERR(port->gpio_base);

	gc = &port->gc;
	gc->of_node = np;
	gc->parent = dev;
	gc->label = "kinetis-gpio";
	gc->ngpio = KINETIS_GPIO_PER_PORT;
	gc->base = of_alias_get_id(np, "gpio") * KINETIS_GPIO_PER_PORT;

	gc->request = gpiochip_generic_request;
	gc->free = gpiochip_generic_free;
	gc->direction_input = kinetis_gpio_direction_input;
	gc->get = kinetis_gpio_get;
	gc->direction_output = kinetis_gpio_direction_output;
	gc->set = kinetis_gpio_set;

	ret = gpiochip_add_data(gc, port);
	if (ret < 0)
		return ret;

	return 0;
}

static struct platform_driver kinetis_gpio_driver = {
	.driver		= {
		.name	= "gpio-kinetis",
		.of_match_table = kinetis_gpio_dt_ids,
	},
	.probe		= kinetis_gpio_probe,
};

static int __init gpio_kinetis_init(void)
{
	return platform_driver_register(&kinetis_gpio_driver);
}
device_initcall(gpio_kinetis_init);

MODULE_AUTHOR("Dmitry Konyshev <probables@emcraft.com>");
MODULE_DESCRIPTION("Freescale Kinetis GPIO");
MODULE_LICENSE("GPL v2");
