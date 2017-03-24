/*
 * Copyright (C) Maxime Coquelin 2015
 * Author:  Maxime Coquelin <mcoquelin.stm32@gmail.com>
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/errno.h>

#include <asm/exception.h>
#include <asm/v7m.h>

#define SYST_CSR	0x00
#define SYST_RVR	0x04
#define SYST_CVR	0x08
#define SYST_CALIB	0x0c

#define SYST_CSR_ENABLE		BIT(0)
#define SYST_CSR_TICKINT	BIT(1)

#define SYSTICK_LOAD_RELOAD_MASK 0x00FFFFFF

static void __iomem *systick_regs;
static u32 systick_frq;
static int systick_is_timer;

static int (*systick_cb)(void *arg);
static void *systick_arg;

/*
 * If `save` is set, then save SysTick masking status to `flags`, and mask
 * SysTick interrupts. If `save` is reset, then restore SysTick masking status
 * from `flags`.
 */
void _armv7m_systick_irq_upd(unsigned long *flags, int save)
{
	u32 val;

	if (!systick_is_timer)
		goto out;

	val = readl(systick_regs + SYST_CSR);
	if (save) {
		*flags = val & SYST_CSR_TICKINT;
		val = val & ~SYST_CSR_TICKINT;
	} else {
		val &= ~SYST_CSR_TICKINT;
		val |= *flags & SYST_CSR_TICKINT;
	}
	writel(val, systick_regs + SYST_CSR);
out:
	return;
}
EXPORT_SYMBOL(_armv7m_systick_irq_upd);

/*
 * Run ARM V7M SysTick timer with `usec` timeout, and call `cb` (with `arg`)
 * upon completion. SysTick will run periodically while `cb` returns 0
 */
int armv7m_systick_run(u32 usec, int (*cb)(void *arg), void *arg)
{
	unsigned long qf, tf, clk;
	int rv;

	if (!systick_is_timer) {
		rv = -ENODEV;
		goto exit;
	}

	if (!systick_regs) {
		rv = -EAGAIN;
		goto exit;
	}

	clk = usec * (systick_frq / 1000000);
	if (!clk || clk > SYSTICK_LOAD_RELOAD_MASK || !cb) {
		pr_err("%s: bad params (%d/%p,%dHz)\n", __func__, usec, cb,
			systick_frq);
		rv = -EINVAL;
		goto exit;
	}

	local_irq_save(qf);
	armv7m_systick_irq_save(tf);

	if (readl(systick_regs + SYST_CSR) & SYST_CSR_ENABLE) {
		armv7m_systick_irq_restore(tf);
		rv = -EBUSY;
		goto out;
	}

	systick_cb = cb;
	systick_arg = arg;

	writel(clk, systick_regs + SYST_RVR);
	writel(readl(systick_regs + SYST_CSR) |
		SYST_CSR_ENABLE | SYST_CSR_TICKINT, systick_regs + SYST_CSR);

	rv = 0;
out:
	local_irq_restore(qf);
exit:
	return rv;
}
EXPORT_SYMBOL(armv7m_systick_run);

/*
 * Stop ARM V7M SysTick timer
 */
int armv7m_systick_stop(void)
{
	unsigned long qf, tf;
	int rv;

	if (!systick_is_timer) {
		rv = -ENODEV;
		goto exit;
	}

	if (!systick_regs) {
		rv = -EAGAIN;
		goto exit;
	}

	local_irq_save(qf);
	armv7m_systick_irq_save(tf);

	if (!(readl(systick_regs + SYST_CSR) & SYST_CSR_ENABLE)) {
		armv7m_systick_irq_restore(tf);
		rv = -ESRCH;
		goto out;
	}

	writel(readl(systick_regs + SYST_CSR) &
		~(SYST_CSR_ENABLE | SYST_CSR_TICKINT), systick_regs + SYST_CSR);

	rv = 0;
out:
	local_irq_restore(qf);
exit:
	return rv;
}
EXPORT_SYMBOL(armv7m_systick_stop);

/*
 * High-level exception handler for exception 15 (SysTick).
 */
asmlinkage void __exception do_systick(void)
{
	/*
	 * Run user callback
	 */
	if (systick_cb && !systick_cb(systick_arg))
		return;

	/*
	 * Stop SysTick if here
	 */
	writel(0, systick_regs + SYST_CSR);
}

static void __init system_timer_of_register(struct device_node *np)
{
	struct clk *clk = NULL;
	const char *use;
	int ret;

	systick_regs = of_iomap(np, 0);
	if (!systick_regs) {
		pr_warn("system-timer: invalid base address\n");
		return;
	}

	ret = of_property_read_u32(np, "clock-frequency", &systick_frq);
	if (ret) {
		clk = of_clk_get(np, 0);
		if (IS_ERR(clk))
			goto out_unmap;

		ret = clk_prepare_enable(clk);
		if (ret)
			goto out_clk_put;

		systick_frq = clk_get_rate(clk);
		if (!systick_frq)
			goto out_clk_disable;
	}

	if (of_get_property(np, "use-as-timer", NULL)) {
		systick_is_timer = 1;
		use = "timer";
		goto out;
	}
	use = "clocksource";

	writel_relaxed(SYSTICK_LOAD_RELOAD_MASK, systick_regs + SYST_RVR);
	writel_relaxed(SYST_CSR_ENABLE, systick_regs + SYST_CSR);

	ret = clocksource_mmio_init(systick_regs + SYST_CVR, "arm_system_timer",
			systick_frq, 200, 24, clocksource_mmio_readl_down);
	if (ret) {
		pr_err("failed to init clocksource (%d)\n", ret);
		if (clk)
			goto out_clk_disable;
		else
			goto out_unmap;
	}

out:
	pr_info("ARM System timer initialized as %s\n", use);

	return;

out_clk_disable:
	clk_disable_unprepare(clk);
out_clk_put:
	clk_put(clk);
out_unmap:
	iounmap(systick_regs);
	systick_regs = NULL;
	pr_warn("ARM System timer register failed (%d)\n", ret);
}

CLOCKSOURCE_OF_DECLARE(arm_systick, "arm,armv7m-systick",
			system_timer_of_register);
