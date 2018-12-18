/*
 * (C) Copyright 2016
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * PWM framework driver for STM32. Note, the driver implements the
 * minimal basic, and should be extended to support the full set of
 * PWM features and channels available in STM32.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#include <asm/io.h>

/*
 * Register offsets
 */
#define TIM_CR1			0x00
#define TIM_CR2			0x04
#define TIM_EGR			0x14
#define TIM_CCMR1		0x18
#define TIM_CCER		0x20
#define TIM_PSC			0x28
#define TIM_ARR			0x2C
#define TIM_CCR1		0x34

/*
 * Register bits
 */
#define CR1_CEN			(1 << 0)
#define CR1_ARPE		(1 << 7)

#define CR2_MMS_MASK		(7 << 4)
#define CR2_MMS_OC_REF(x)	((3 + (x)) << 4)

#define EGR_UG			(1 << 0)

#define CCMR_OCxM_MASK		((1 << 16) | (7 << 4))
#define CCMR_OCxM_PWM		((0 << 16) | (6 << 4))
#define CCMR_OCxM_OCPE		(1 << 3)
#define CCMR_OCxM_ODD(v)	((v) << 0)
#define CCMR_OCxM_EVE(v)	((v) << 8)

#define CCER_CCxE(c)		((1 << 0) << ((c) << 2))
#define CCER_CCxP(c)		((1 << 1) << ((c) << 2))

enum stm32_timertype {
	tim1_8,
	tim2_5,
	tim6_7,
	tim9_12,
	tim10_14,
};

/*
 * STM32 PWM descriptor
 */
struct stm_pwm_chip {
	struct pwm_chip	chip;
	struct device	*dev;

	struct clk	*clk;
	void __iomem	*regs;

	int		tmr;
	int		chan;
	int		bits;

	bool		high_on_init;
	bool		trigger_output;
};

/*
 * Standard->STM32 PWM chip convert
 */
static inline struct stm_pwm_chip *to_stm_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct stm_pwm_chip, chip);
}

/*
 * PWM API: configure period and duty cycle
 */
static int stm_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct stm_pwm_chip *pc = to_stm_pwm_chip(chip);
	unsigned long long c;
	unsigned long period_cycles, prescale, duty, period;

	c = clk_get_rate(pc->clk);
	c = c * period_ns;
	do_div(c, 1000000000ULL);
	period_cycles = c;

	do_div(c, pc->bits == 16 ? 0xFFFFUL : 0xFFFFFFFFUL);
	prescale = c;
	if (prescale > 0xFFFFUL)
		return -EINVAL;

	period = period_cycles / (prescale + 1);

	c = period;
	c = c * duty_ns;
	do_div(c, period_ns);
	duty = c;

	dev_dbg(pc->dev, "period=%dns,duty=%dns -> "
		"prescale=%ld,period=%ld,duty=%ld\n",
		period_ns, duty_ns, prescale, period, duty);

	writel(prescale, pc->regs + TIM_PSC);
	writel(duty, pc->regs + TIM_CCR1 + (pc->chan << 2));
	writel(period, pc->regs + TIM_ARR);

	writel(EGR_UG, pc->regs + TIM_EGR);

	return 0;
}

/*
 * PWM API: enable PWM signal
 */
static int stm_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct stm_pwm_chip *pc = to_stm_pwm_chip(chip);
	unsigned int val;
	unsigned long f;

	/*
	 * Disable IRQs to get the min possible delay between CCER & CR1 regs
	 * update
	 */
	local_irq_save(f);

	if (pc->high_on_init) {
		pc->high_on_init = false;

		val = readl(pc->regs + TIM_CCER);
		val &= ~CCER_CCxP(pc->chan);
		writel(val, pc->regs + TIM_CCER);
	}

	val = readl(pc->regs + TIM_CR1);
	val |= CR1_CEN;
	writel(val, pc->regs + TIM_CR1);

	local_irq_restore(f);

	return 0;
}

/*
 * PWM API: disable PWM signal
 */
static void stm_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct stm_pwm_chip *pc = to_stm_pwm_chip(chip);
	unsigned int val;

	val = readl(pc->regs + TIM_CR1);
	val &= ~CR1_CEN;
	writel(val, pc->regs + TIM_CR1);
}

static struct pwm_ops stm_pwm_ops = {
	.config = stm_pwm_config,
	.enable = stm_pwm_enable,
	.disable = stm_pwm_disable,
	.owner = THIS_MODULE,
};

static const struct of_device_id stm_pwm_of_match[] = {
	{ .compatible = "st,stm32-pwm", },
	{ }
};
MODULE_DEVICE_TABLE(of, pwm_of_match);

/*
 * Used in dts binding: devices which use PWM (e.g. pwm-backlight) configure
 * PWM using this function
 */
static struct pwm_device *stm_pwm_of_xlate(struct pwm_chip *pc,
					   const struct of_phandle_args *args)
{
	struct pwm_device *pwm;

	pwm = pwm_request_from_chip(pc, 0, NULL);
	if (IS_ERR(pwm))
		return pwm;

	pwm_set_period(pwm, args->args[0]);

	return pwm;
}

static int stm_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct of_phandle_args spec;
	struct device_node *np = NULL;
	struct stm_pwm_chip *pc = NULL;
	unsigned int val, ofs;
	int rv, tmr, chan;
	enum stm32_timertype timtype;

	rv = of_parse_phandle_with_fixed_args(dev->of_node, "timer", 2, 0,
					      &spec);
	if (rv) {
		dev_err(dev, "no correct <timer> property in '%s'\n",
			dev->of_node->name);
		rv = -EINVAL;
		goto out;
	}

	np = spec.np;
	tmr = spec.args[0];
	chan = spec.args[1];

	switch (tmr) {
	case 2:
	case 3:
	case 4:
	case 5:
		timtype = tim2_5;
		break;
	case 6:
	case 7:
		timtype = tim6_7;
		break;
	case 12:
		timtype = tim9_12;
		break;
	case 13:
	case 14:
		timtype = tim10_14;
		break;
	case 1:
	case 8:
		timtype = tim1_8;
		break;
	case 9:
		timtype = tim9_12;
		break;
	case 10:
	case 11:
		timtype = tim10_14;
		break;
	default:
		rv = -ENODEV;
		goto out;
	}

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc) {
		rv = -ENOMEM;
		goto out;
	}
	pc->dev = dev;

	pc->clk = of_clk_get(np, 0);
	if (IS_ERR(pc->clk)) {
		dev_err(dev, "can't get timer clock\n");
		rv = PTR_ERR(pc->clk);
		goto out;
	}

	pc->chip.dev = &pdev->dev;
	pc->chip.ops = &stm_pwm_ops;
	pc->chip.base = -1;
	pc->chip.npwm = 1;

	pc->chip.of_xlate = stm_pwm_of_xlate;
	pc->chip.of_pwm_n_cells = 1;

	pc->regs = of_iomap(np, 0);
	if (IS_ERR(pc->regs)) {
		dev_err(dev, "can't map timer registers\n");
		rv = PTR_ERR(pc->regs);
		goto out;
	}

	/*
	 * 'trigger-output' allows to generate internal TRGO
	 */
	pc->trigger_output = device_property_read_bool(pc->chip.dev,
						       "trigger-output");

	/*
	 * 'high-on-init' allows to initialize PWM with HIGH output until
	 * first PWM run.
	 */
	pc->high_on_init = device_property_read_bool(pc->chip.dev,
						     "high-on-init");

	/* Detect whether the timer is 16 or 32 bits */
	writel(~0U, pc->regs + TIM_ARR);
	pc->bits = readl(pc->regs + TIM_ARR) == ~0U ? 32 : 16;
	pc->tmr = tmr;
	pc->chan = chan - 1;

	/* Enable timer clock */
	rv = clk_prepare_enable(pc->clk);
	if (rv) {
		dev_err(dev, "can't enable clk (%d)\n", rv);
		goto out;
	}

	if (pc->trigger_output) {
		/*
		 * Configure timer for triggering internal event
		 */
		val = readl(pc->regs + TIM_CR2);
		val &= ~CR2_MMS_MASK;
		val |= CR2_MMS_OC_REF(chan);
		writel(val, pc->regs + TIM_CR2);
	}

	/*
	 * Configure the specified timer channel
	 * Supported for now:
	 *  - TIM2-5 timers with 1-4 channels
	 *  - TIM9,12 timers with 1-2 channels
	 *  - TIM10,11,13,14 timers with 1 channel
	 */

	if (timtype == tim2_5 || timtype == tim9_12) {
		ofs = TIM_CCMR1 + ((pc->chan / 2) << 2);
		val = readl(pc->regs + ofs);
		if (chan % 2) {
			val &= ~CCMR_OCxM_ODD(CCMR_OCxM_MASK);
			val |= CCMR_OCxM_ODD(CCMR_OCxM_PWM);
			val |= CCMR_OCxM_ODD(CCMR_OCxM_OCPE);
		} else {
			val &= ~CCMR_OCxM_EVE(CCMR_OCxM_MASK);
			val |= CCMR_OCxM_EVE(CCMR_OCxM_PWM);
			val |= CCMR_OCxM_EVE(CCMR_OCxM_OCPE);
		}

		writel(val, pc->regs + ofs);

		val = readl(pc->regs + TIM_CCER);

		val |= CCER_CCxE(pc->chan);

		if (pc->high_on_init)
			val |= CCER_CCxP(pc->chan);


		writel(val, pc->regs + TIM_CCER);

		val = readl(pc->regs + TIM_CR1);
		val |= CR1_ARPE;
		writel(val, pc->regs + TIM_CR1);
	} else {
		if (timtype == tim10_14) { /* TIM10 */
			ofs = TIM_CCMR1 ;
			val = readl(pc->regs + ofs);
			val &= ~CCMR_OCxM_ODD(CCMR_OCxM_MASK);
			val |= CCMR_OCxM_ODD(CCMR_OCxM_PWM);
			val |= CCMR_OCxM_ODD(CCMR_OCxM_OCPE);

			writel(val, pc->regs + ofs);

			val = readl(pc->regs + TIM_CCER);

			val |= 1;

			if (pc->high_on_init)
				val |= 2;


			writel(val, pc->regs + TIM_CCER);

			val = readl(pc->regs + TIM_CR1);
			val |= CR1_ARPE;
			writel(val, pc->regs + TIM_CR1);
		} else {
			dev_err(dev, "Unsupported timer\n");
			rv = -ENODEV;
			goto out;
		}
	}

	rv = pwmchip_add(&pc->chip);
	if (rv < 0) {
		dev_err(dev, "pwmchip_add() failed: %d\n", rv);
		goto out;
	}

	dev_info(dev, "basing on TIM%d.%d(x%d)\n", tmr, chan, pc->bits);
	platform_set_drvdata(pdev, pc);

	rv = 0;
out:
	if (rv) {
		if (pc)
			devm_kfree(dev, pc);
	}

	if (np)
		of_node_put(np);

	return rv;
}

static int stm_pwm_remove(struct platform_device *pdev)
{
	struct stm_pwm_chip *chip;

	chip = platform_get_drvdata(pdev);
	if (chip == NULL)
		return -ENODEV;

	return pwmchip_remove(&chip->chip);
}

static struct platform_driver stm_pwm_driver = {
	.driver		= {
		.name	= "stm32-pwm",
		.of_match_table = stm_pwm_of_match,
	},
	.probe		= stm_pwm_probe,
	.remove		= stm_pwm_remove,
};

module_platform_driver(stm_pwm_driver);

MODULE_DESCRIPTION("STM32 PWM driver");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL");
