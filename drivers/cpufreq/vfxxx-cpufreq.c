/*
 * Copyright (C) 2017 Emcraft Systems.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <linux/tty.h>
#include <linux/console.h>
#include <linux/serial_core.h>

struct {
	int pll_main_freq;
	int pll_pfd_freq;
} *freq_table_int;

static struct clk *sys_sel;
static struct clk *clk_suspend;
static struct clk *pll_pfd_sel;
static struct clk *clk_pll_main;
static struct clk *clk_pll_pfd;

static struct device *cpu_dev;
static struct cpufreq_frequency_table *freq_table;
static unsigned int transition_latency;

static void uarts_reconfig(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "fsl,vf610-uart") {
		struct platform_device *pdev = of_find_device_by_node(np);
		struct uart_port *port = pdev ? platform_get_drvdata(pdev) : NULL;

		if (port && port->state) {
			struct tty_struct *tty = port->state->port.tty;
			if (port->ops->set_termios && tty) {
				port->ops->set_termios(port, &tty->termios, NULL);
			}
		}
	}
}

static int imx6q_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	if (freq_table_int[index].pll_main_freq) {
		clk_set_parent(sys_sel, clk_suspend);
		uarts_reconfig();

		clk_set_rate(clk_pll_main, freq_table_int[index].pll_main_freq * 1000);

		if (freq_table_int[index].pll_pfd_freq) {
			clk_set_rate(clk_pll_pfd, freq_table_int[index].pll_pfd_freq * 1000);
			clk_set_parent(pll_pfd_sel, clk_pll_pfd);
		} else {
			clk_set_parent(pll_pfd_sel, clk_pll_main);
		}

		clk_set_parent(sys_sel, pll_pfd_sel);
		uarts_reconfig();
	}

	return 0;
}

static int imx6q_cpufreq_init(struct cpufreq_policy *policy)
{
	policy->clk = sys_sel;
	return cpufreq_generic_init(policy, freq_table, transition_latency);
}

static struct cpufreq_driver imx6q_cpufreq_driver = {
	.flags = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = imx6q_set_target,
	.get = cpufreq_generic_get,
	.init = imx6q_cpufreq_init,
	.name = "vfxx-cpufreq",
	.attr = cpufreq_generic_attr,
};

static int imx6q_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret;
	const struct property *prop;
	const __be32 *val;
	u32 i;
	int cnt;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENODEV;
	}

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		dev_err(cpu_dev, "failed to find cpu0 node\n");
		return -ENOENT;
	}

	sys_sel = clk_get(cpu_dev, "sys_sel");
	clk_suspend = clk_get(cpu_dev, "sys_suspend");
	pll_pfd_sel = clk_get(cpu_dev, "sys_pll_pfd_sel");
	clk_pll_main = clk_get(cpu_dev, "sys_pll_main");
	clk_pll_pfd = clk_get(cpu_dev, "sys_pll_pfd");
	if (IS_ERR(sys_sel) || IS_ERR(pll_pfd_sel) || IS_ERR(clk_pll_pfd)
	    || IS_ERR(clk_suspend) || IS_ERR(clk_pll_main)) {
		pr_err("Failed to get clocks\n");
		ret = -EINVAL;
		goto put_clk;
	}

	prop = of_find_property(np, "cpuclock_tbl", NULL);
	if (!prop || !prop->value) {
		pr_err("Invalid cpuclock_tbl");
		ret = -EINVAL;
		goto put_clk;
	}

	cnt = prop->length / sizeof(u32);
	val = prop->value;

	freq_table_int = devm_kzalloc(cpu_dev, sizeof(*freq_table_int) * (cnt / 2 + 1), GFP_KERNEL);
	freq_table = devm_kzalloc(cpu_dev, sizeof(*freq_table) * (cnt / 2 + 1), GFP_KERNEL);
	if (!freq_table) {
		ret = -ENOMEM;
		goto put_clk;
	}

	for (i = 0; i < (cnt / 2); i++) {
		freq_table_int[i].pll_main_freq = be32_to_cpup(val++);
		freq_table_int[i].pll_pfd_freq = be32_to_cpup(val++);
		if (freq_table_int[i].pll_pfd_freq) {
			freq_table[i].frequency = freq_table_int[i].pll_pfd_freq;
		} else {
			freq_table[i].frequency = freq_table_int[i].pll_main_freq;
		}
	}

	freq_table[i].frequency = CPUFREQ_TABLE_END;

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	ret = cpufreq_register_driver(&imx6q_cpufreq_driver);
	if (ret) {
		dev_err(cpu_dev, "failed register driver: %d\n", ret);
		goto put_mem;
	}

	of_node_put(np);
	return 0;

put_mem:
	devm_kfree(cpu_dev, freq_table);

put_clk:
	if (!IS_ERR(sys_sel))
		clk_put(sys_sel);
	if (!IS_ERR(clk_suspend))
		clk_put(clk_suspend);
	if (!IS_ERR(pll_pfd_sel))
		clk_put(pll_pfd_sel);
	if (!IS_ERR(clk_pll_main))
		clk_put(clk_pll_main);
	if (!IS_ERR(clk_pll_pfd))
		clk_put(clk_pll_pfd);
	of_node_put(np);
	return ret;
}

static struct platform_driver imx6q_cpufreq_platdrv = {
	.driver = {
		.name	= "vfxxx-cpufreq",
	},
	.probe		= imx6q_cpufreq_probe,
};

module_platform_driver(imx6q_cpufreq_platdrv);

MODULE_AUTHOR("Vladimir Skvortsov <vskvortsov@emcraft.com>");
MODULE_DESCRIPTION("Vybrid cpufreq driver");
MODULE_LICENSE("GPL");
