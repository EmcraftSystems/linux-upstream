/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * Added to support STOP mode with DDR self-refresh
 * Copyright (c) 2015
 * Sergei Miroshnichenko, Emcraft Systems, sergeimir@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/genalloc.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/tlb.h>
#include <linux/tty.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/of_gpio.h>

#include "common.h"

#define CCR				0x0
#define BM_CCR_FIRC_EN			(0x1 << 16)
#define BM_CCR_FXOSC_EN			(0x1 << 12)

#define CCSR				0x8
#define BM_CCSR_DDRC_CLK_SEL		(0x1 << 6)
#define BM_CCSR_FAST_CLK_SEL		(0x1 << 5)
#define BM_CCSR_SLOW_CLK_SEL		(0x1 << 4)
#define BM_CCSR_SYS_CLK_SEL_MASK	(0x7 << 0)

#define CLPCR				0x2c
#define BM_CLPCR_ARM_CLK_DIS_ON_LPM	(0x1 << 5)
#define BM_CLPCR_SBYOS			(0x1 << 6)
#define BM_CLPCR_DIS_REF_OSC		(0x1 << 7)
#define BM_CLPCR_ANADIG_STOP_MODE	(0x1 << 8)
#define BM_CLPCR_FXOSC_BYPSEN		(0x1 << 10)
#define BM_CLPCR_FXOSC_PWRDWN		(0x1 << 11)
#define BM_CLPCR_MASK_CORE0_WFI		(0x1 << 22)
#define BM_CLPCR_MASK_CORE1_WFI		(0x1 << 23)
#define BM_CLPCR_MASK_SCU_IDLE		(0x1 << 24)
#define BM_CLPCR_MASK_L2CC_IDLE		(0x1 << 25)

#define CCGR				0x40

#define GPC_PGCR			0x0
#define BM_PGCR_DS_STOP			(0x1 << 7)
#define BM_PGCR_DS_LPSTOP		(0x1 << 6)
#define BM_PGCR_WB_STOP			(0x1 << 4)
#define BM_PGCR_HP_OFF			(0x1 << 3)

#define GPC_LPMR			0x40
#define BM_LPMR_RUN			0x0
#define BM_LPMR_STOP			0x2

#define VF610_SUSPEND_OCRAM_SIZE	0x4000

struct vf610_pm_base {
	phys_addr_t pbase;
	void __iomem *vbase;
};

struct vf610_cpu_pm_info {
	phys_addr_t pbase;
	phys_addr_t resume_addr;
	u32 cpu_type;
	u32 pm_info_size;
	struct vf610_pm_base anatop_base;
	struct vf610_pm_base src_base;
	struct vf610_pm_base ccm_base;
	struct vf610_pm_base gpc_base;
	struct vf610_pm_base mscm_base;
	struct vf610_pm_base ddrmc_base;
	struct vf610_pm_base iomuxc_base;
	u32 ccm_ccsr;
	struct clk *sys_sel_clk;
	struct clk *sys_sel_clk_active;
	struct clk *sys_sel_clk_suspend;
} __aligned(8);

enum vf610_cpu_pwr_mode {
	VF610_RUN,
	VF610_LP_RUN,
	VF610_STOP,
	VF610_LP_STOP,
};

static void __iomem *ccm_base;
static void __iomem *suspend_ocram_base;

static int powerdown_gpio = -1;
static bool powerdown_gpio_active_low;

static u32 (*suspend_in_iram)(suspend_state_t state,
			      unsigned long iram_paddr,
			      unsigned long suspend_iram_base) = NULL;

extern void mvf_suspend(suspend_state_t state);

struct vf610_pm_socdata {
	const char *src_compat;
	const char *gpc_compat;
	const char *anatop_compat;
	const char *mscm_compat;
	const char *ccm_compat;
	const char *ddrmc_compat;
	const char *iomuxc_compat;
};

static const struct vf610_pm_socdata vf610_pm_data __initconst = {
	.src_compat	= "fsl,vf610-src",
	.gpc_compat	= "fsl,vf610-gpc",
	.anatop_compat	= "fsl,vf610-anatop",
	.mscm_compat	= "fsl,vf610-mscm-ir",
	.ccm_compat	= "fsl,vf610-ccm",
	.ddrmc_compat	= "emcraft,ddrmc",
	.iomuxc_compat	= "fsl,vf610-iomuxc",
};

static void uarts_reconfig(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "fsl,vf610-uart") {
		struct platform_device *pdev = of_find_device_by_node(np);
		struct uart_port *port = pdev ? platform_get_drvdata(pdev) : NULL;

		if (port && port->state) {
			struct tty_struct *tty = port->state->port.tty;
			int may_wakeup = (tty ? device_may_wakeup(tty->dev) : 0);
			if (may_wakeup && port->ops->set_termios) {
				port->ops->set_termios(port, &tty->termios, NULL);
			}
		}
	}
}

int vf610_set_lpm(enum vf610_cpu_pwr_mode mode)
{
	struct vf610_cpu_pm_info *pm_info = suspend_ocram_base;
	void __iomem *gpc_base	= pm_info->gpc_base.vbase;
	u32 ccr		= readl_relaxed(ccm_base + CCR);
	u32 ccsr	= readl_relaxed(ccm_base + CCSR);
	u32 cclpcr	= readl_relaxed(ccm_base + CLPCR);
	u32 gpc_pgcr 	= readl_relaxed(gpc_base + GPC_PGCR);

	switch (mode) {
	case VF610_STOP:
		cclpcr |= BM_CLPCR_ANADIG_STOP_MODE;
		cclpcr &= ~BM_CLPCR_ARM_CLK_DIS_ON_LPM;
		cclpcr &= ~BM_CLPCR_SBYOS;
		cclpcr |= BM_CLPCR_MASK_SCU_IDLE;
		cclpcr |= BM_CLPCR_MASK_L2CC_IDLE;
		cclpcr &= ~BM_CLPCR_MASK_CORE1_WFI;
		cclpcr &= ~BM_CLPCR_MASK_CORE0_WFI;
		writel_relaxed(cclpcr, ccm_base + CLPCR);

		gpc_pgcr |= BM_PGCR_DS_STOP;
		gpc_pgcr |= BM_PGCR_HP_OFF;
		writel_relaxed(gpc_pgcr, gpc_base + GPC_PGCR);

		/* fall-through */
	case VF610_LP_RUN:
		/* Store clock settings */
		pm_info->ccm_ccsr = ccsr;

		ccr |= BM_CCR_FIRC_EN;
		writel_relaxed(ccr, ccm_base + CCR);
		ccsr &= ~BM_CCSR_FAST_CLK_SEL;
		ccsr &= ~BM_CCSR_SLOW_CLK_SEL;
		writel_relaxed(ccsr, ccm_base + CCSR);

		if (!IS_ERR(pm_info->sys_sel_clk) && !IS_ERR(pm_info->sys_sel_clk_suspend)) {
			pm_info->sys_sel_clk_active = clk_get_parent(pm_info->sys_sel_clk);
			clk_set_parent(pm_info->sys_sel_clk, pm_info->sys_sel_clk_suspend);
			uarts_reconfig();
		}

		if (gpio_is_valid(powerdown_gpio))
			gpio_set_value(powerdown_gpio, !powerdown_gpio_active_low);

		break;
	case VF610_RUN:
		if (gpio_is_valid(powerdown_gpio))
			gpio_set_value(powerdown_gpio, powerdown_gpio_active_low);

		if (!IS_ERR(pm_info->sys_sel_clk) && !IS_ERR(pm_info->sys_sel_clk_active)) {
			clk_set_parent(pm_info->sys_sel_clk, pm_info->sys_sel_clk_active);
			uarts_reconfig();
		}

		writel_relaxed(BM_LPMR_RUN, gpc_base + GPC_LPMR);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vf610_suspend_finish(unsigned long val)
{
	struct vf610_cpu_pm_info *pm_info = suspend_ocram_base;

	local_flush_tlb_all();
	flush_cache_all();

	local_fiq_disable();

	outer_flush_all();
	outer_disable();

	suspend_in_iram(PM_SUSPEND_MEM, (unsigned long)pm_info->pbase, (unsigned long)suspend_ocram_base);

	outer_resume();

	return 0;
}


static int vf610_pm_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
		vf610_set_lpm(VF610_STOP);

		/* zzZZZzzz */
		cpu_do_idle();

		vf610_set_lpm(VF610_RUN);
		break;
	case PM_SUSPEND_MEM:
		vf610_set_lpm(VF610_STOP);

		cpu_suspend(0, vf610_suspend_finish);

		vf610_set_lpm(VF610_RUN);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vf610_pm_valid(suspend_state_t state)
{
	return ((state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM));
}

static const struct platform_suspend_ops vf610_pm_ops = {
	.enter = vf610_pm_enter,
	.valid = vf610_pm_valid,
};

static int __init imx_pm_get_base(struct vf610_pm_base *base,
				const char *compat)
{
	struct device_node *node;
	struct resource res;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, compat);
	if (!node) {
		ret = -ENODEV;
		goto out;
	}

	ret = of_address_to_resource(node, 0, &res);
	if (ret)
		goto put_node;

	base->pbase = res.start;
	base->vbase = ioremap(res.start, resource_size(&res));

	if (!base->vbase)
		ret = -ENOMEM;

put_node:
	of_node_put(node);
out:
	return ret;
}

static void __init vf610_get_power_pin(void)
{
	enum of_gpio_flags powerdown_gpio_flags;
	struct device_node *node;
	int err;

	node = of_find_compatible_node(NULL, NULL, "emcraft,power");
	if (!node)
		return;

	powerdown_gpio = of_get_named_gpio_flags(node, "powerdown-gpios", 0, &powerdown_gpio_flags);
	if (!gpio_is_valid(powerdown_gpio))
		return;

	err = gpio_request(powerdown_gpio, "nPOWERDOWN");
	if (err) {
		pr_err("%s: can't request powerdown gpio %d: %d\n", __func__, powerdown_gpio, err);
		powerdown_gpio = -1;
		return;
	}

	powerdown_gpio_active_low = powerdown_gpio_flags & OF_GPIO_ACTIVE_LOW;
	gpio_direction_output(powerdown_gpio, powerdown_gpio_active_low);
}

static int __init vf610_suspend_init(const struct vf610_pm_socdata *socdata)
{
	phys_addr_t ocram_pbase;
	struct device_node *node;
	struct platform_device *pdev;
	struct vf610_cpu_pm_info *pm_info;
	struct gen_pool *ocram_pool;
	unsigned long ocram_base;
	int ret = 0;

	suspend_set_ops(&vf610_pm_ops);

	if (!socdata) {
		pr_warn("%s: invalid argument!\n", __func__);
		return -EINVAL;
	}

	node = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!node) {
		pr_warn("%s: failed to find ocram node!\n", __func__);
		return -ENODEV;
	}

	pdev = of_find_device_by_node(node);
	if (!pdev) {
		pr_warn("%s: failed to find ocram device!\n", __func__);
		ret = -ENODEV;
		goto put_node;
	}

	ocram_pool = gen_pool_get(&pdev->dev);
	if (!ocram_pool) {
		pr_warn("%s: ocram pool unavailable!\n", __func__);
		ret = -ENODEV;
		goto put_node;
	}

	ocram_base = gen_pool_alloc(ocram_pool, VF610_SUSPEND_OCRAM_SIZE);
	if (!ocram_base) {
		pr_warn("%s: unable to alloc ocram!\n", __func__);
		ret = -ENOMEM;
		goto put_node;
	}

	ocram_pbase = gen_pool_virt_to_phys(ocram_pool, ocram_base);

	suspend_ocram_base = __arm_ioremap_exec(ocram_pbase,
						VF610_SUSPEND_OCRAM_SIZE, false);

	memcpy((void *)(ocram_base + sizeof(*pm_info)), mvf_suspend, SZ_32K);
	suspend_in_iram = (void *)(suspend_ocram_base + sizeof(*pm_info));

	pm_info = suspend_ocram_base;
	pm_info->pbase = ocram_pbase;
	pm_info->pm_info_size = sizeof(*pm_info);

	node = of_find_compatible_node(NULL, NULL, "pm-vf610");
	if (node) {
		pm_info->sys_sel_clk = of_clk_get_by_name(node, "sys-sel");
		pm_info->sys_sel_clk_suspend = of_clk_get_by_name(node, "sys-sel-suspend");
		if (!IS_ERR_OR_NULL(pm_info->sys_sel_clk) && !IS_ERR_OR_NULL(pm_info->sys_sel_clk_suspend))
			pm_info->sys_sel_clk_active = clk_get_parent(pm_info->sys_sel_clk);
	}

	/*
	 * ccm physical address is not used by asm code currently,
	 * so get ccm virtual address directly, as we already have
	 * it from ccm driver.
	 */
	ret = imx_pm_get_base(&pm_info->ccm_base, socdata->ccm_compat);
	if (ret) {
		pr_warn("%s: failed to get ccm base %d!\n", __func__, ret);
		goto put_node;
	}

	ccm_base = pm_info->ccm_base.vbase;

	ret = imx_pm_get_base(&pm_info->anatop_base, socdata->anatop_compat);
	if (ret) {
		pr_warn("%s: failed to get anatop base %d!\n", __func__, ret);
		goto put_node;
	}

	ret = imx_pm_get_base(&pm_info->gpc_base, socdata->gpc_compat);
	if (ret) {
		pr_warn("%s: failed to get gpc base %d!\n", __func__, ret);
		goto gpc_map_failed;
	}

	ret = imx_pm_get_base(&pm_info->ddrmc_base, socdata->ddrmc_compat);
	if (ret) {
		pr_warn("%s: failed to get ddrmc base %d!\n", __func__, ret);
		goto gpc_map_failed;
	}

	ret = imx_pm_get_base(&pm_info->iomuxc_base, socdata->iomuxc_compat);
	if (ret) {
		pr_warn("%s: failed to get iomuxc base %d!\n", __func__, ret);
		goto gpc_map_failed;
	}

	ret = imx_pm_get_base(&pm_info->mscm_base, socdata->mscm_compat);
	if (ret) {
		pr_warn("%s: failed to get mscm base %d!\n", __func__, ret);
		goto gpc_map_failed;
	}

	goto put_node;

gpc_map_failed:
	iounmap(&pm_info->anatop_base.vbase);
put_node:
	of_node_put(node);

	return ret;
}

static int __init vf610_pm_init(void)
{
	int err = 0;

	vf610_get_power_pin();

	if (IS_ENABLED(CONFIG_SUSPEND)) {
		err = vf610_suspend_init(&vf610_pm_data);
		if (err)
			pr_warn("No DDR LPM support with suspend");
	}

	return err;
}

late_initcall(vf610_pm_init);
