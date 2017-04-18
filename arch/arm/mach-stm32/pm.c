/*
 * arch/arm/mach-stm32/pm.c
 *
 * STM32 Power Management code
 *
 * Copyright (C) 2016 Emcraft Systems
 * Yuri Tikhonov, Emcraft Systems, <yur@emcraft.com>
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

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>

#include <mach/gpio.h>
#include <mach/pwr.h>
#include <mach/sram.h>

/*
 * Assembler-level imports (from SRAM)
 */
SRAM_TEXT void stm32_suspend_to_ram(void);
extern u32 stm32_suspend_moder[STM32_GPIO_PORTS];

/*
 * Import from stm32 pinctrl driver
 */
int stm32_pctrl_alt_to_analog(struct platform_device *p, u32 *moder, u32 len);

/*
 * Cortex-M3 System Control Register
 */
#define CM3_SCR_BASE		0xE000ED10
#define CM3_SCR_SLEEPDEEP	(1 << 2)

/*
 * LTDC registers
 */
#define LTDC_GCR		0x18

/*
 * Different data we saved on entering, and restore on exiting
 * from PM
 */
static struct {
	struct {
		u32	cr;
	} pwr;
	struct {
		u32	gcr;
	} ltdc;
} stm32_pm_bck;

/*
 * Reg bases
 */
static u8 *pwr_regs;
static u8 *ltdc_regs;

static struct platform_device *pinctrl_pdev;
static struct gpio_desc *pwr_en;
static int wake_gpio = - 1;

/*
 * Device data structure
 */
static struct platform_driver stm32_pm_driver = {
	.driver = {
		.name = "stm32_pm",
	},
};

/*
 * Validate suspend state
 * @state		State being entered
 * @returns		1->valid, 0->invalid
 */
static int stm32_pm_valid(suspend_state_t state)
{
	int ret;

	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static irqreturn_t stm32_pm_wake_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

/*
 * Prepare system hardware to suspend. Some settings are reset on
 * exiting from Stop, so we save these here, and restore then
 */
static int stm32_pm_prepare_to_suspend(void)
{
	int	rv;

	/*
	 * Save over-drive settings
	 */
	stm32_pm_bck.pwr.cr = readl(pwr_regs + STM32_PWR_CR);

	/*
	 * FB driver may be off, so always stop LTDC here to avoid SDRAM access
	 */
	stm32_pm_bck.ltdc.gcr = readl(ltdc_regs + LTDC_GCR);
	writel(stm32_pm_bck.ltdc.gcr & ~1, ltdc_regs + LTDC_GCR);

	/*
	 * Get array of MODER[] values to switch each ALT pin into ANALOG mode
	 * before going sleep
	 */
	rv = stm32_pctrl_alt_to_analog(pinctrl_pdev, stm32_suspend_moder,
				       ARRAY_SIZE(stm32_suspend_moder));

	/*
	 * Switch off peripheral power
	 */
	if (pwr_en)
		gpiod_direction_output(pwr_en, 0);

	return rv;
}

/*
 * Prepare system hardware to resume
 */
static void stm32_pm_prepare_to_resume(void)
{
	/*
	 * Switch on peripheral power
	 */
	if (pwr_en)
		gpiod_direction_output(pwr_en, 1);

	/*
	 * Restore LTDC
	 */
	if (stm32_pm_bck.ltdc.gcr & 1)
		writel(stm32_pm_bck.ltdc.gcr, ltdc_regs + LTDC_GCR);

	/*
	 * Restore over-drive
	 */
	if (stm32_pm_bck.pwr.cr & STM32_PWR_CR_ODSWEN) {
		writel(STM32_PWR_CR_ODEN | readl(pwr_regs + STM32_PWR_CR),
		       pwr_regs + STM32_PWR_CR);
		while (!(readl(pwr_regs + STM32_PWR_CSR) &
			 STM32_PWR_CSR_ODRDY));
		writel(STM32_PWR_CR_ODSWEN | readl(pwr_regs + STM32_PWR_CR),
		       pwr_regs + STM32_PWR_CR);
	}
}

/*
 * Enter suspend
 * @state		State being entered
 * @returns		0->success, <0->error code
 */
static int stm32_pm_enter(suspend_state_t state)
{
	volatile u32 *scr = (void *)CM3_SCR_BASE;

	/*
	 * Prepare the system hardware to suspend
	 */
	if (stm32_pm_prepare_to_suspend())
		return -EFAULT;

	/*
	 * Allow STOP mode. Enter SLEEP DEEP on WFI.
	 */
	*scr |= CM3_SCR_SLEEPDEEP;

	/*
	 * Jump to suspend code in SRAM
	 */
	stm32_suspend_to_ram();

	/*
	 * Switch to Normal mode. Disable SLEEP DEEP on WFI.
	 */
	*scr &= ~CM3_SCR_SLEEPDEEP;

	/*
	 * Prepare the system hardware to resume
	 */
	stm32_pm_prepare_to_resume();

	return 0;
}

/*
 * Power Management operations
 */
static struct platform_suspend_ops stm32_pm_ops = {
	.valid = stm32_pm_valid,
	.enter = stm32_pm_enter,
};

/*
 * Driver init
 * @returns		0->success, <0->error code
 */
static int __init stm32_pm_init(void)
{
	struct device_node *np;
	struct clk *clk;
	int ret;
	enum of_gpio_flags flags;

	/*
	 * Relocate code to SRAM
	 */
	stm32_sram_init();
	stm32_sram_relocate();

	/*
	 * Get PWR register base, and enable PWR module
	 */
	np = of_find_compatible_node(NULL, NULL, "st,stm32-pwr");
	if (!np) {
		printk(KERN_ERR "%s: of_find(stm32-pwr) fail\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	clk = of_clk_get(np, 0);
	if (clk)
		clk_prepare_enable(clk);
	pwr_regs = of_iomap(np, 0);

	ret = of_get_named_gpio(np, "pwr-en-gpios", 0);
	if (!(ret < 0))
		pwr_en = gpio_to_desc(ret);

	of_node_put(np);

	/*
	 * Get LTDC register base
	 */
	np = of_find_compatible_node(NULL, NULL, "st,stm32f4-ltdc");
	if (!np) {
		printk(KERN_ERR "%s: of_find(stm32f4-ltdc) fail\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	ltdc_regs = of_iomap(np, 0);
	of_node_put(np);

	/*
	 * Get GPIOs controller
	 */
	np = of_find_compatible_node(NULL, NULL, "st,stm32-pinctrl");
	if (!np) {
		printk(KERN_ERR "%s: of_find(stm32-pinctrl) fail\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	pinctrl_pdev = of_find_device_by_node(np);

	of_node_put(np);
	if (!pinctrl_pdev) {
		printk(KERN_ERR "%s: of_find_device() fail\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	/* Get wake-up gpio */
	np = of_find_compatible_node(NULL, NULL, "st,stm32-wake");
	if (np) {
		wake_gpio = of_get_gpio_flags(np, 0, &flags);
		of_node_put(np);
	}

	/* Register a wake-up handler, if correct gpio was found */
	if (wake_gpio >= 0) {
		int ret = request_irq(gpio_to_irq(wake_gpio),
				stm32_pm_wake_handler,
				((flags & OF_GPIO_ACTIVE_LOW) ?
				 IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING) |
				IRQF_NO_SUSPEND,
				"Wakeup GPIO", NULL);
		if (ret) {
			printk(KERN_ERR "%s: unable to request wake irq\n",
					__func__);
			wake_gpio = -1;
		}
	}

	/*
	 * Want the lowest consumption in Stop mode
	 */
	writel((STM32_PWR_CR_DBP & readl(pwr_regs + STM32_PWR_CR)) |
		STM32_PWR_CR_MODE, pwr_regs + STM32_PWR_CR);

	/*
	 * Register the PM driver
	 */
	if (platform_driver_register(&stm32_pm_driver) != 0) {
		printk(KERN_ERR "%s: register failed\n", __func__);
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Register PM operations
	 */
	suspend_set_ops(&stm32_pm_ops);

	/*
	 * Here means success
	 */
	printk(KERN_INFO "Power Management for STM32\n");
	ret = 0;
out:
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit stm32_pm_cleanup(void)
{
	platform_driver_unregister(&stm32_pm_driver);
	if (wake_gpio >= 0)
		free_irq(gpio_to_irq(wake_gpio), NULL);
}

module_init(stm32_pm_init);
module_exit(stm32_pm_cleanup);

MODULE_AUTHOR("Yuri Tikhonov");
MODULE_DESCRIPTION("STM32 PM driver");
MODULE_LICENSE("GPL");
