/*
 * Copyright (C) 2015 Toradex AG
 * Author: Stefan Agner <stefan@agner.ch>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * The GPC (General Power Controller) irqchip driver takes care of the
 * interrupt wakeup functionality.
 *
 * o All peripheral interrupts of the Vybrid SoC can be used as wakeup
 *   source from STOP mode. In LPSTOP mode however, the GPC is unpowered
 *   too and cannot be used to as a wakeup source. The WKPU (Wakeup Unit)
 *   is responsible for wakeups from LPSTOP modes.
 */

#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#define IMR_NUM			4
#define VF610_GPC_IMR1		0x044
#define VF610_GPC_MAX_IRQS	(IMR_NUM * 32)

static void __iomem *gpc_base;

static int vf610_gpc_irq_set_wake(struct irq_data *d, unsigned int on)
{
	unsigned int idx = d->hwirq / 32;
	void __iomem *reg_imr = gpc_base + VF610_GPC_IMR1 + (idx * 4);
	u32 mask = 1 << d->hwirq % 32;

	if (on)
		writel_relaxed(readl_relaxed(reg_imr) & ~mask, reg_imr);
	else
		writel_relaxed(readl_relaxed(reg_imr) | mask, reg_imr);

	/*
	 * Do *not* call into the parent, as the GIC doesn't have any
	 * wake-up facility...
	 */
	return 0;
}

static struct irq_chip vf610_gpc_chip = {
	.name			= "vf610-gpc",
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_enable		= irq_chip_enable_parent,
	.irq_disable		= irq_chip_disable_parent,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_wake		= vf610_gpc_irq_set_wake,
};

static int vf610_gpc_domain_alloc(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs, void *arg)
{
	int i;
	irq_hw_number_t hwirq;
	struct irq_fwspec *fwspec = arg;
	struct irq_fwspec parent_fwspec;

	if (!irq_domain_get_of_node(domain->parent))
		return -EINVAL;

	if (fwspec->param_count != 2)
		return -EINVAL;

	hwirq = fwspec->param[0];
	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &vf610_gpc_chip, NULL);

	parent_fwspec = *fwspec;
	parent_fwspec.fwnode = domain->parent->fwnode;
	return irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					    &parent_fwspec);
}

static int vf610_gpc_domain_translate(struct irq_domain *d,
					  struct irq_fwspec *fwspec,
					  unsigned long *hwirq,
					  unsigned int *type)
{
	if (WARN_ON(fwspec->param_count < 2))
		return -EINVAL;
	*hwirq = fwspec->param[0];
	*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

static const struct irq_domain_ops gpc_irq_domain_ops = {
	.translate = vf610_gpc_domain_translate,
	.alloc = vf610_gpc_domain_alloc,
	.free = irq_domain_free_irqs_common,
};

static int __init vf610_gpc_of_init(struct device_node *node,
			       struct device_node *parent)
{
	struct irq_domain *domain, *domain_parent;
	int i;

	domain_parent = irq_find_host(parent);
	if (!domain_parent) {
		pr_err("vf610_gpc: interrupt-parent not found\n");
		return -EINVAL;
	}

	gpc_base = of_io_request_and_map(node, 0, "gpc");
	if (WARN_ON(!gpc_base))
	        return -ENOMEM;

	domain = irq_domain_add_hierarchy(domain_parent, 0, VF610_GPC_MAX_IRQS,
					  node, &gpc_irq_domain_ops, NULL);
	if (!domain) {
		iounmap(gpc_base);
		return -ENOMEM;
	}

	/* Initially mask all interrupts for wakeup */
	for (i = 0; i < IMR_NUM; i++)
		writel_relaxed(~0, gpc_base + VF610_GPC_IMR1 + i * 4);

	return 0;
}
IRQCHIP_DECLARE(vf610_gpc, "fsl,vf610-gpc", vf610_gpc_of_init);
