/*
 * i.MX RT105x GPIO support
 *
 * Copyright (c) 2017 Emcraft Systems
 *
 * Author: Dmitry Konyshev <probables@emcraft.com>.
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

#define IMXRT105X_GPIO_PER_PORT		32

struct imxrt105x_gpio_port {
	struct gpio_chip gc;
	void __iomem *gpio_base;
	int irq0_15, irq16_31;
};

#define GPIO_DR			0x00
#define GPIO_GDIR		0x04
#define GPIO_PSR		0x08
#define GPIO_ICR1		0x0c
#define GPIO_ICR2		0x10
#define GPIO_IMR		0x14
#define GPIO_ISR		0x18
#define GPIO_EDGE_SEL		0x1c

#define PORT_INT_LOGIC_ZERO	0x0
#define PORT_INT_LOGIC_ONE	0x1
#define PORT_INT_RISING_EDGE	0x2
#define PORT_INT_FALLING_EDGE	0x3

static struct irq_chip imxrt105x_gpio_irq_chip;

static const struct of_device_id imxrt105x_gpio_dt_ids[] = {
	{ .compatible = "fsl,imxrt105x-gpio" },
	{ /* sentinel */ }
};

static int imxrt105x_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct imxrt105x_gpio_port *port = gpiochip_get_data(gc);

	return !!(readl_relaxed(port->gpio_base + GPIO_DR) & BIT(gpio));
}

static void imxrt105x_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct imxrt105x_gpio_port *port = gpiochip_get_data(gc);
	unsigned long mask = readl_relaxed(port->gpio_base + GPIO_DR);

	if (val)
		mask |= BIT(gpio);
	else
		mask &= ~BIT(gpio);
	writel_relaxed(mask, port->gpio_base + GPIO_DR);
}

static int imxrt105x_gpio_direction_input(struct gpio_chip *gc, unsigned gpio)
{
	struct imxrt105x_gpio_port *port = gpiochip_get_data(gc);
	unsigned long mask = readl_relaxed(port->gpio_base + GPIO_GDIR);

	mask &= ~BIT(gpio);
	writel_relaxed(mask, port->gpio_base + GPIO_GDIR);

	return 0;
}

static int imxrt105x_gpio_direction_output(struct gpio_chip *gc, unsigned gpio,
				       int value)
{
	struct imxrt105x_gpio_port *port = gpiochip_get_data(gc);
	unsigned long mask = readl_relaxed(port->gpio_base + GPIO_GDIR);

	imxrt105x_gpio_set(gc, gpio, value);

	mask |= BIT(gpio);
	writel_relaxed(mask, port->gpio_base + GPIO_GDIR);

	return 0;
}

static void imxrt105x_gpio_irq_handler(struct irq_desc *desc)
{
	struct imxrt105x_gpio_port *port =
		gpiochip_get_data(irq_desc_get_handler_data(desc));
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int pin;
	unsigned long isr;

	chained_irq_enter(chip, desc);

	isr = readl_relaxed(port->gpio_base + GPIO_ISR);

	for_each_set_bit(pin, &isr, IMXRT105X_GPIO_PER_PORT) {
		writel_relaxed(BIT(pin), port->gpio_base + GPIO_ISR);

		generic_handle_irq(irq_find_mapping(port->gc.irqdomain, pin));
	}

	chained_irq_exit(chip, desc);
}

static void imxrt105x_gpio_irq_ack(struct irq_data *d)
{
	struct imxrt105x_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));
	int gpio = d->hwirq;

	writel_relaxed(BIT(gpio), port->gpio_base + GPIO_ISR);
}

static int imxrt105x_gpio_irq_set_type(struct irq_data *d, u32 type)
{
	struct imxrt105x_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));
	int gpio = d->hwirq;
	unsigned long mask = readl_relaxed(port->gpio_base +
			(gpio < 16 ? GPIO_ICR1 : GPIO_ICR2));
	u8 irqc;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		irqc = PORT_INT_RISING_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		irqc = PORT_INT_FALLING_EDGE;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		irqc = PORT_INT_LOGIC_ZERO;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		irqc = PORT_INT_LOGIC_ONE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		break;
	default:
		return -EINVAL;
	}

	if (type == IRQ_TYPE_EDGE_BOTH) {
		mask = readl_relaxed(port->gpio_base + GPIO_EDGE_SEL) |
			BIT(gpio);
	} else {
		if (gpio < 16)
			writel_relaxed(mask | (irqc << (gpio << 1)),
				port->gpio_base + GPIO_ICR1);
		else
			writel_relaxed(mask | (irqc << ((gpio - 16) << 1)),
				port->gpio_base + GPIO_ICR2);
		mask = readl_relaxed(port->gpio_base + GPIO_EDGE_SEL) &
			~BIT(gpio);
	}

	writel_relaxed(mask, port->gpio_base + GPIO_EDGE_SEL);

	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	return 0;
}

static void imxrt105x_gpio_irq_mask(struct irq_data *d)
{
	struct imxrt105x_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));
	int gpio = d->hwirq;

	writel_relaxed(readl_relaxed(port->gpio_base + GPIO_IMR) & ~BIT(gpio),
		port->gpio_base + GPIO_IMR);
}

static void imxrt105x_gpio_irq_unmask(struct irq_data *d)
{
	struct imxrt105x_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));
	int gpio = d->hwirq;

	writel_relaxed(readl_relaxed(port->gpio_base + GPIO_IMR) | BIT(gpio),
		port->gpio_base + GPIO_IMR);
}

static int imxrt105x_gpio_irq_set_wake(struct irq_data *d, u32 enable)
{
	struct imxrt105x_gpio_port *port =
		gpiochip_get_data(irq_data_get_irq_chip_data(d));

	if (enable)
		enable_irq_wake(port->irq0_15);
	else
		disable_irq_wake(port->irq0_15);

	return 0;
}

static struct irq_chip imxrt105x_gpio_irq_chip = {
	.name		= "gpio-imxrt105x",
	.irq_ack	= imxrt105x_gpio_irq_ack,
	.irq_mask	= imxrt105x_gpio_irq_mask,
	.irq_unmask	= imxrt105x_gpio_irq_unmask,
	.irq_set_type	= imxrt105x_gpio_irq_set_type,
	.irq_set_wake	= imxrt105x_gpio_irq_set_wake,
};

static int imxrt105x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imxrt105x_gpio_port *port;
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

	port->irq0_15 = platform_get_irq(pdev, 0);
	if (port->irq0_15 < 0)
		return port->irq0_15;

	port->irq16_31 = platform_get_irq(pdev, 1);
	if (port->irq16_31 < 0)
		return port->irq16_31;

	gc = &port->gc;
	gc->of_node = np;
	gc->parent = dev;
	gc->label = "imxrt105x-gpio";
	gc->ngpio = IMXRT105X_GPIO_PER_PORT;
	gc->base = of_alias_get_id(np, "gpio") * IMXRT105X_GPIO_PER_PORT;

	gc->request = gpiochip_generic_request;
	gc->free = gpiochip_generic_free;
	gc->direction_input = imxrt105x_gpio_direction_input;
	gc->get = imxrt105x_gpio_get;
	gc->direction_output = imxrt105x_gpio_direction_output;
	gc->set = imxrt105x_gpio_set;

	ret = gpiochip_add_data(gc, port);
	if (ret < 0)
		return ret;

	/* Clear the interrupt status register for all GPIO's */
	writel_relaxed(~0, port->gpio_base + GPIO_ISR);

	ret = gpiochip_irqchip_add(gc, &imxrt105x_gpio_irq_chip, 0,
				   handle_edge_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_err(dev, "failed to add irqchip\n");
		gpiochip_remove(gc);
		return ret;
	}
	gpiochip_set_chained_irqchip(gc, &imxrt105x_gpio_irq_chip, port->irq0_15,
				     imxrt105x_gpio_irq_handler);
	gpiochip_set_chained_irqchip(gc, &imxrt105x_gpio_irq_chip, port->irq16_31,
				     imxrt105x_gpio_irq_handler);

	return 0;
}

static struct platform_driver imxrt105x_gpio_driver = {
	.driver		= {
		.name	= "gpio-imxrt105x",
		.of_match_table = imxrt105x_gpio_dt_ids,
	},
	.probe		= imxrt105x_gpio_probe,
};

static int __init gpio_imxrt105x_init(void)
{
	return platform_driver_register(&imxrt105x_gpio_driver);
}
device_initcall(gpio_imxrt105x_init);

MODULE_AUTHOR("Dmitry Konyshev <probables@emcraft.com>");
MODULE_DESCRIPTION("NXP IMXRT105X GPIO");
MODULE_LICENSE("GPL v2");
