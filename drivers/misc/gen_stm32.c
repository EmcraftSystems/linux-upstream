/*
 * (C) Copyright 2017
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * STM32 Dummy Pulse Generator
 *
 * The GPIO is specified in the .dts, e.g.:
 *
 * / {
 *     pin-controller {
 *         gpio {
 *             pinctrl_outputs: outs {
 *                 st,pins {
 *                     DS4.led = <&gpiob 2 OUT NO_PULL PUSH_PULL LOW_SPEED>;
 *                 };
 *             };
 *         };
 *     };
 * };
 * &stm32_dummy_gen {
 *     status = "okay";
 *     pulse-gpio = <&gpiob 2 GPIO_ACTIVE_HIGH>;
 * };
 *
 * To configure to run <num> pulses, with <period_usec> period, and
 * <hi_usec> duration of HIGH pulse part, do:
 * # echo num,hi_usec,period_usec > /sys/kernel/pulse_gen/cfg
 *
 * To read current configuration:
 * # cat /sys/kernel/pulse_gen/cfg
 *
 * To run pulses:
 * # cat /sys/kernel/pulse_gen/run
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

static struct kobject *gen_kobj;
static struct kobj_attribute cfg_attr, run_attr;

static struct gpio_desc *gpio;
static int num = 10, hi = 1000, period = 10000;

static ssize_t gen_cfg_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
        return sprintf(buf, "num=%d,hi=%dus,period=%dus\n",
			num, hi, period);
}
static ssize_t gen_cfg_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int n, h, p, r;

	r = sscanf(buf, "%d,%d,%d", &n, &h, &p);
	if (r == 3 && n > 0 && h > 0 && p > 0 && p > h) {
		num = n;
		hi = h;
		period = p;
	} else {
		printk("%s: error, bad format\n", __func__);
	}

	return count;
}

static ssize_t gen_run_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	unsigned long flags;
	int i, hi_ms, hi_us, lo_ms, lo_us;

	if (!gpio)
		return sprintf(buf, "no GPIO specified in dtb");

	printk("Disable interrupts, and run %d pulses (%dus/%dus)...\n",
		num, hi, period);
	udelay(1000);

	hi_ms = hi / 1000;
	hi_us = hi - (hi_ms * 1000);

	lo_ms = (period - hi) / 1000;
	lo_us = period - hi - (lo_ms * 1000);

	local_irq_save(flags);
	for (i = 0; i < num; i++) {
		/*
		 * Set HI
		 */
		gpiod_set_value(gpio, 1);
		if (hi_ms)
			mdelay(hi_ms);
		if (hi_us)
			udelay(hi_us);

		/*
		 * Set LO
		 */
		gpiod_set_value(gpio, 0);
		if (lo_ms)
			mdelay(lo_ms);
		if (lo_us)
			udelay(lo_us);
	}
	local_irq_restore(flags);

	return sprintf(buf, "%d pulses done", num);
}

static struct kobj_attribute cfg_attr = __ATTR(cfg, 0660,
						gen_cfg_show, gen_cfg_store);
static struct kobj_attribute run_attr = __ATTR(run, 0440,
						gen_run_show, NULL);

static int __init stm32_gen_init(void)
{
	struct device_node *np;
	int rv;

	np = of_find_compatible_node(NULL, NULL, "stm32-dummy-pulse-gen");
	if (!np) {
		printk("%s: no 'stm32-dummy-pulse-gen' node found in dtb\n",
			__func__);
		rv = -ENODEV;
		goto out;
	}
	rv = of_get_named_gpio(np, "pulse-gpio", 0);
	if (!(rv < 0)) {
		gpio = gpio_to_desc(rv);
	} else {
		printk("%s: no 'pulse-gpio' property found\n", __func__);
		rv = -ENODEV;
		goto out;
	}
	gpiod_direction_output(gpio, 0);

	gen_kobj = kobject_create_and_add("pulse_gen", kernel_kobj);
	if (!gen_kobj) {
		printk("%s: kobject_create_and_add() err\n", __func__);
		rv = -ENOMEM;
		goto out;
	}

	if (sysfs_create_file(gen_kobj, &cfg_attr.attr) ||
	    sysfs_create_file(gen_kobj, &run_attr.attr)) {
		printk("%s: sysfs_create_file() err\n", __func__);
		rv = -EFAULT;
		goto out;
	}

	rv = 0;
out:
	return rv;
}

static void __exit stm32_gen_exit(void)
{
	kobject_put(gen_kobj);
}

module_init(stm32_gen_init);
module_exit(stm32_gen_exit);

MODULE_DESCRIPTION("STM32 Dummy Pulse Generator Driver");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL");
