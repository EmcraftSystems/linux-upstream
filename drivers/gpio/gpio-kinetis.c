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
#include <mach/kinetis.h>

#define KINETIS_GPIO_PER_PORT		32

struct kinetis_gpio_port {
	struct gpio_chip gc;
	void __iomem *gpio_base;
	u8 irqc[KINETIS_GPIO_PER_PORT];
	int irq;
};

#define GPIO_PDOR		0x00
#define GPIO_PSOR		0x04
#define GPIO_PCOR		0x08
#define GPIO_PTOR		0x0c
#define GPIO_PDIR		0x10
#define GPIO_PDDR		0x14

#define PORT_INT_OFF		0x0
#define PORT_INT_LOGIC_ZERO	0x8
#define PORT_INT_RISING_EDGE	0x9
#define PORT_INT_FALLING_EDGE	0xa
#define PORT_INT_EITHER_EDGE	0xb
#define PORT_INT_LOGIC_ONE	0xc
#define PORT_INT_MASK		0xf

#define PORT_PCR_IRQC_OFFSET	16

static const struct of_device_id kinetis_gpio_dt_ids[] = {
	{ .compatible = "fsl,k70-gpio" },
	{ .compatible = "fsl,kinetis-gpio" },
	{ /* sentinel */ }
};

extern int kinetis_pinctrl_get_isfr(int port);
extern void kinetis_pinctrl_set_isfr(int port, int mask);
extern void kinetis_pinctrl_set_irqc(int port, int pin, int mask);

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

static void kinetis_gpio_irq_handler(struct irq_desc *desc)
{
	struct kinetis_gpio_port *port =
		gpiochip_get_data(irq_desc_get_handler_data(desc));
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int pin;
	unsigned long irq_isfr;

	chained_irq_enter(chip, desc);

	irq_isfr =
		kinetis_pinctrl_get_isfr(port->gc.base / KINETIS_GPIO_PER_PORT);

	for_each_set_bit(pin, &irq_isfr, KINETIS_GPIO_PER_PORT) {
		kinetis_pinctrl_set_isfr(port->gc.base / KINETIS_GPIO_PER_PORT,
				BIT(pin));

		generic_handle_irq(irq_find_mapping(port->gc.irqdomain,
					pin));
	}

	chained_irq_exit(chip, desc);
}

static void kinetis_gpio_irq_ack(struct irq_data *d)
{
	struct kinetis_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));
	int gpio = d->hwirq;

	kinetis_pinctrl_set_isfr(port->gc.base / KINETIS_GPIO_PER_PORT,
			BIT(gpio));
}

static int kinetis_gpio_irq_set_type(struct irq_data *d, u32 type)
{
	struct kinetis_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));
	u8 irqc;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		irqc = PORT_INT_RISING_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		irqc = PORT_INT_FALLING_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		irqc = PORT_INT_EITHER_EDGE;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		irqc = PORT_INT_LOGIC_ZERO;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		irqc = PORT_INT_LOGIC_ONE;
		break;
	default:
		return -EINVAL;
	}

	port->irqc[d->hwirq] = irqc;

	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	return 0;
}

static void kinetis_gpio_irq_mask(struct irq_data *d)
{
	struct kinetis_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));

	kinetis_pinctrl_set_irqc(port->gc.base / KINETIS_GPIO_PER_PORT,
			d->hwirq, 0);
}

static void kinetis_gpio_irq_unmask(struct irq_data *d)
{
	struct kinetis_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));

	kinetis_pinctrl_set_irqc(port->gc.base / KINETIS_GPIO_PER_PORT,
			d->hwirq, port->irqc[d->hwirq] << 16);
}

static int kinetis_gpio_irq_set_wake(struct irq_data *d, u32 enable)
{
	struct kinetis_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));

	if (enable) {
		enable_irq_wake(port->irq);
	} else {
		disable_irq_wake(port->irq);
	}

	return 0;
}

static struct irq_chip kinetis_gpio_irq_chip = {
	.name		= "gpio-kinetis",
	.irq_ack	= kinetis_gpio_irq_ack,
	.irq_mask	= kinetis_gpio_irq_mask,
	.irq_unmask	= kinetis_gpio_irq_unmask,
	.irq_set_type	= kinetis_gpio_irq_set_type,
	.irq_set_wake	= kinetis_gpio_irq_set_wake,
};

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

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

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

	/* Clear the interrupt status register for all GPIO's */
	kinetis_pinctrl_set_isfr(gc->base / KINETIS_GPIO_PER_PORT, ~0);

	ret = gpiochip_irqchip_add(gc, &kinetis_gpio_irq_chip, 0,
				   handle_edge_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_err(dev, "failed to add irqchip\n");
		gpiochip_remove(gc);
		return ret;
	}

	gpiochip_set_chained_irqchip(gc, &kinetis_gpio_irq_chip, port->irq,
				     kinetis_gpio_irq_handler);

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
