/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio/driver.h>
#include <linux/clk.h>

#define LPC17XX_MAX_PORTS	6
#define LPC17XX_PINS_PER_PORT	32
#define LPC17XX_PINS_LAST_PORT	5

struct gpio_regs {
	u32	dir;
	u32	reserved[3];
	u32	mask;
	u32	pin;
	u32	set;
	u32	clr;
};

struct lpc17xx_gpio_chip {
	struct gpio_chip chip;
	struct gpio_regs __iomem *regs;
	struct clk *clk;
};

static inline struct lpc17xx_gpio_chip *to_lpc17xx_gpio(
	struct gpio_chip *gpc)
{
	return container_of(gpc, struct lpc17xx_gpio_chip, chip);
}

static void lpc17xx_gpio_decode(struct lpc17xx_gpio_chip *gc, unsigned offset, u32 *port, u32 *pin) {
	*port = offset / LPC17XX_PINS_PER_PORT;
	*pin  = offset % LPC17XX_PINS_PER_PORT;
}

static int lpc17xx_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	u32 port, pin, mask;
	struct lpc17xx_gpio_chip *gc = to_lpc17xx_gpio(chip);
	lpc17xx_gpio_decode(gc, offset, &port, &pin);
	dev_dbg(chip->dev, "%s: offset %u (%u.%u)\n", __func__, offset, port, pin);

	mask = readl(&gc->regs[port].mask) & ~BIT(pin);
	writel(mask, &gc->regs[port].mask);

	return pinctrl_request_gpio(offset);
}

static void lpc17xx_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	u32 port, pin, mask;
	struct lpc17xx_gpio_chip *gc = to_lpc17xx_gpio(chip);
	lpc17xx_gpio_decode(gc, offset, &port, &pin);
	dev_dbg(chip->dev, "%s: offset %u (%u.%u)\n", __func__, offset, port, pin);

	mask = readl(&gc->regs[port].mask) | BIT(pin);
	writel(mask, &gc->regs[port].mask);

	pinctrl_free_gpio(offset);
}

static void lpc17xx_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	u32 port, pin;
	struct lpc17xx_gpio_chip *gc = to_lpc17xx_gpio(chip);
	lpc17xx_gpio_decode(gc, offset, &port, &pin);
	dev_dbg(chip->dev, "%s: offset %u (%u.%u), value %d\n", __func__, offset, port, pin, value);

	if (value)
		writel(BIT(pin), &gc->regs[port].set);
	else
		writel(BIT(pin), &gc->regs[port].clr);
}

static int lpc17xx_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 port, pin;
	struct lpc17xx_gpio_chip *gc = to_lpc17xx_gpio(chip);
	int value;
	lpc17xx_gpio_decode(gc, offset, &port, &pin);

	value = !!(readl(&gc->regs[port].pin) & BIT(pin));
	dev_dbg(chip->dev, "%s: offset %u (%u.%u), value %d\n", __func__, offset, port, pin, value);

	return value;
}

static int lpc17xx_gpio_direction(struct gpio_chip *chip, unsigned offset,
				  bool out)
{
	unsigned port, pin, dir;
	struct lpc17xx_gpio_chip *gc = to_lpc17xx_gpio(chip);
	lpc17xx_gpio_decode(gc, offset, &port, &pin);
	dev_dbg(chip->dev, "%s: offset %u (%u.%u): %s\n", __func__, offset, port, pin, out ? "out" : "in");

	dir = readl(&gc->regs[port].dir);
	if (out)
		dir |= BIT(pin);
	else
		dir &= ~BIT(pin);
	writel(dir, &gc->regs[port].dir);

	return 0;
}

static int lpc17xx_gpio_direction_input(struct gpio_chip *chip,
					unsigned offset)
{
	dev_dbg(chip->dev, "%s: offset %u\n", __func__, offset);
	return lpc17xx_gpio_direction(chip, offset, false);
}

static int lpc17xx_gpio_direction_output(struct gpio_chip *chip,
					 unsigned offset, int value)
{
	dev_dbg(chip->dev, "%s: offset %u, value %d\n", __func__, offset, value);
	lpc17xx_gpio_set(chip, offset, value);
	return lpc17xx_gpio_direction(chip, offset, true);
}

static struct gpio_chip lpc17xx_chip = {
	.label			= "lpc17xx-gpio",
	.request		= lpc17xx_gpio_request,
	.free			= lpc17xx_gpio_free,
	.direction_input	= lpc17xx_gpio_direction_input,
	.direction_output	= lpc17xx_gpio_direction_output,
	.set			= lpc17xx_gpio_set,
	.get			= lpc17xx_gpio_get,
	.ngpio			= (LPC17XX_MAX_PORTS - 1) * LPC17XX_PINS_PER_PORT + LPC17XX_PINS_LAST_PORT,
};

static int lpc17xx_gpio_probe(struct platform_device *pdev)
{
	struct lpc17xx_gpio_chip *gc;
	struct resource *res;
	int ret;

	gc = devm_kzalloc(&pdev->dev, sizeof(*gc), GFP_KERNEL);
	if (!gc)
		return -ENOMEM;

	gc->chip = lpc17xx_chip;
	platform_set_drvdata(pdev, gc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(gc->regs))
		return gc->regs ? PTR_ERR(gc->regs) : -ENOENT;

	gc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(gc->clk)) {
		dev_err(&pdev->dev, "input clock not found\n");
		return gc->clk ? PTR_ERR(gc->clk) : -ENOENT;
	}

	ret = clk_prepare_enable(gc->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		return ret;
	}

	gc->chip.dev = &pdev->dev;

	ret = gpiochip_add(&gc->chip);
	if (ret) {
		dev_err(&pdev->dev, "failed to add gpio chip\n");
		clk_disable_unprepare(gc->clk);
		return ret;
	}

	return 0;
}

static const struct of_device_id lpc17xx_gpio_of_match[] = {
	{ .compatible = "nxp,lpc17xx-gpio", },
	{ /* sentinel */ },
};

static struct platform_driver lpc17xx_gpio_driver = {
	.driver		= {
		.name	= "lpc17xx-gpio",
		.of_match_table = of_match_ptr(lpc17xx_gpio_of_match),
	},
	.probe		= lpc17xx_gpio_probe,
};

module_platform_driver(lpc17xx_gpio_driver);
