/*
 * timer-kinetis.c - Timer driver for Kinetis K70
 *
 * Based on legacy pre-OF code by Alexander Potashev <aspotashev@emcraft.com>
 *
 * Copyright (C) 2015 Paul Osmialowski <pawelo@king.net.pl>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <mach/kinetis.h>

#define KINETIS_PIT_CHANNELS 4

#define KINETIS_PIT0_IRQ 68
#define KINETIS_PIT1_IRQ 69
#define KINETIS_PIT2_IRQ 70
#define KINETIS_PIT3_IRQ 71

/*
 * PIT Timer Control Register
 */
/* Timer Interrupt Enable Bit */
#define KINETIS_PIT_TCTRL_TIE_MSK	(1 << 1)
/* Timer Enable Bit */
#define KINETIS_PIT_TCTRL_TEN_MSK	(1 << 0)
/*
 * PIT Timer Flag Register
 */
/* Timer Interrupt Flag */
#define KINETIS_PIT_TFLG_TIF_MSK	(1 << 0)

/*
 * PIT control registers base
 */
#define KINETIS_PIT_BASE		(KINETIS_AIPS0PERIPH_BASE + 0x00037000)
#define KINETIS_PIT_MCR			IOMEM(KINETIS_PIT_BASE + 0x0)

/*
 * Periodic Interrupt Timer (PIT) registers
 */
struct kinetis_pit_channel_regs {
	u32 ldval;	/* Timer Load Value Register */
	u32 cval;	/* Current Timer Value Register */
	u32 tctrl;	/* Timer Control Register */
	u32 tflg;	/* Timer Flag Register */
};

#define KINETIS_PIT_PTR(base, reg) \
	(&(((struct kinetis_pit_channel_regs *)(base))->reg))
#define KINETIS_PIT_RD(base, reg) readl(KINETIS_PIT_PTR(base, reg))
#define KINETIS_PIT_WR(base, reg, val) \
	writel((val), KINETIS_PIT_PTR(base, reg))
#define KINETIS_PIT_SET(base, reg, mask) \
	KINETIS_PIT_WR(base, reg, (KINETIS_PIT_RD(base, reg)) | (mask))
#define KINETIS_PIT_RESET(base, reg, mask) \
	KINETIS_PIT_WR(base, reg, (KINETIS_PIT_RD(base, reg)) & (~(mask)))

struct kinetis_clock_event_ddata {
	struct clock_event_device evtdev;
	void __iomem *base;
};

/*
 * Enable or disable a PIT channel
 */
static void kinetis_pit_enable(void __iomem *base, int enable)
{
	if (enable)
		KINETIS_PIT_SET(base, tctrl, KINETIS_PIT_TCTRL_TEN_MSK);
	else
		KINETIS_PIT_RESET(base, tctrl, KINETIS_PIT_TCTRL_TEN_MSK);
}

/*
 * Initialize a PIT channel, but do not enable it
 */
static void kinetis_pit_init(void __iomem *base, u32 ticks)
{
	/*
	 * Enable the PIT module clock
	 */
	writel(0, KINETIS_PIT_MCR);

	KINETIS_PIT_WR(base, tctrl, 0);
	KINETIS_PIT_WR(base, tflg, KINETIS_PIT_TFLG_TIF_MSK);
	KINETIS_PIT_WR(base, ldval, ticks);
	KINETIS_PIT_WR(base, cval, 0);
	KINETIS_PIT_WR(base, tctrl, KINETIS_PIT_TCTRL_TIE_MSK);
}

/*
 * Clock event device set mode functions
 */
static void kinetis_clockevent_tmr_set_state_periodic(
	struct clock_event_device *clk)
{
	struct kinetis_clock_event_ddata *pit =
		container_of(clk, struct kinetis_clock_event_ddata, evtdev);

	kinetis_pit_enable(pit->base, 1);
}

static void kinetis_clockevent_tmr_set_state_shutdown(
	struct clock_event_device *clk)
{
	struct kinetis_clock_event_ddata *pit =
		container_of(clk, struct kinetis_clock_event_ddata, evtdev);

	kinetis_pit_enable(pit->base, 0);
}

/*
 * Configure the timer to generate an interrupt in the specified amount of ticks
 */
static int kinetis_clockevent_tmr_set_next_event(
	unsigned long delta, struct clock_event_device *c)
{
	struct kinetis_clock_event_ddata *pit =
		container_of(c, struct kinetis_clock_event_ddata, evtdev);
	unsigned long flags;

	raw_local_irq_save(flags);
	kinetis_pit_init(pit->base, delta);
	kinetis_pit_enable(pit->base, 1);
	raw_local_irq_restore(flags);

	return 0;
}

static struct kinetis_clock_event_ddata
		kinetis_clockevent_tmrs[KINETIS_PIT_CHANNELS] = {
	{
		.evtdev = {
			.name		= "fsl,kinetis-pit-timer0",
			.rating		= 200,
			.features	=
			    CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
			.set_state_periodic	=
				kinetis_clockevent_tmr_set_state_periodic,
			.set_state_shutdown	=
				kinetis_clockevent_tmr_set_state_shutdown,
			.set_state_oneshot	=
				kinetis_clockevent_tmr_set_state_shutdown,
			.set_next_event	= kinetis_clockevent_tmr_set_next_event,
		},
	},
	{
		.evtdev = {
			.name		= "fsl,kinetis-pit-timer1",
		},
	},
	{
		.evtdev = {
			.name		= "fsl,kinetis-pit-timer2",
		},
	},
	{
		.evtdev = {
			.name		= "fsl,kinetis-pit-timer3",
		},
	},
};

/*
 * Timer IRQ handler
 */
static irqreturn_t kinetis_clockevent_tmr_irq_handler(int irq, void *dev_id)
{
	struct kinetis_clock_event_ddata *tmr = dev_id;

	KINETIS_PIT_WR(tmr->base, tflg, KINETIS_PIT_TFLG_TIF_MSK);

	tmr->evtdev.event_handler(&(tmr->evtdev));

	return IRQ_HANDLED;
}

/*
 * System timer IRQ action
 */
static struct irqaction kinetis_clockevent_irqaction[KINETIS_PIT_CHANNELS] = {
	{
		.name = "Kinetis Kernel Time Tick (pit0)",
		.flags = IRQF_TIMER | IRQF_IRQPOLL,
		.dev_id = &kinetis_clockevent_tmrs[0],
		.handler = kinetis_clockevent_tmr_irq_handler,
	}, {
		.name = "Kinetis Kernel Time Tick (pit1)",
		.flags = IRQF_TIMER | IRQF_IRQPOLL,
		.dev_id = &kinetis_clockevent_tmrs[1],
		.handler = kinetis_clockevent_tmr_irq_handler,
	}, {
		.name = "Kinetis Kernel Time Tick (pit2)",
		.flags = IRQF_TIMER | IRQF_IRQPOLL,
		.dev_id = &kinetis_clockevent_tmrs[2],
		.handler = kinetis_clockevent_tmr_irq_handler,
	}, {
		.name = "Kinetis Kernel Time Tick (pit3)",
		.flags = IRQF_TIMER | IRQF_IRQPOLL,
		.dev_id = &kinetis_clockevent_tmrs[3],
		.handler = kinetis_clockevent_tmr_irq_handler,
	},
};

static void __init kinetis_clockevent_init(struct device_node *np)
{
	const u64 max_delay_in_sec = 5;
	struct clk *clk;
	void __iomem *base;
	unsigned long rate;
	int irq, chan;

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("failed to get clock for clockevent\n");
		return;
	}

	if (clk_prepare_enable(clk)) {
		pr_err("failed to enable timer clock for clockevent\n");
		goto err_clk_enable;
	}

	rate = clk_get_rate(clk);
	if (!(rate / HZ)) {
		pr_err("failed to get proper clock rate for clockevent\n");
		goto err_clk_enable;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("failed to get map registers for clockevent\n");
		goto err_iomap;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (irq <= 0) {
		pr_err("failed to get irq for clockevent\n");
		goto err_get_irq;
	}

	chan = of_alias_get_id(np, "pit");
	if ((chan < 0) || (chan >= KINETIS_PIT_CHANNELS)) {
		pr_err("failed to calculate channel number for clockevent\n");
		goto err_get_irq;
	}
	kinetis_clockevent_tmrs[chan].base = base;

	/*
	 * Set the fields required for the set_next_event method
	 * (tickless kernel support)
	 */
	clockevents_calc_mult_shift(&(kinetis_clockevent_tmrs[chan].evtdev),
					rate, max_delay_in_sec);
	kinetis_clockevent_tmrs[chan].evtdev.max_delta_ns =
					max_delay_in_sec * NSEC_PER_SEC;
	kinetis_clockevent_tmrs[chan].evtdev.min_delta_ns =
			clockevent_delta2ns(0xf,
				&(kinetis_clockevent_tmrs[chan].evtdev));

	clockevents_register_device(&(kinetis_clockevent_tmrs[chan].evtdev));

	kinetis_pit_init(base, (rate / HZ) - 1);
	kinetis_pit_enable(base, 1);

	setup_irq(irq, &(kinetis_clockevent_irqaction[chan]));

	return;

err_get_irq:

	iounmap(base);
err_iomap:

	clk_disable_unprepare(clk);
err_clk_enable:

	clk_put(clk);
}

CLOCKSOURCE_OF_DECLARE(kinetis_pit_timer, "fsl,kinetis-pit-timer",
		       kinetis_clockevent_init);
