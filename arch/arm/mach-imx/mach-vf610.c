/*
 * Copyright 2012-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/system_info.h>

/* FSL-wide unique */
#define OCOTP_CFG0	0x410
/* Wafer number */
#define OCOTP_CFG1	0x420

static void __init vf610_init_late(void)
{
	struct device_node *np;
	void __iomem *base;

	np = of_find_compatible_node(NULL, NULL, "fsl,vf610-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
	} else {
		system_serial_low = readl(base + OCOTP_CFG1);
		system_serial_high = readl(base + OCOTP_CFG0);
	}

	if (IS_ENABLED(CONFIG_ARM_VFXXX_CPUFREQ))
		platform_device_register_simple("vfxxx-cpufreq", -1, NULL, 0);
}

static const char * const vf610_dt_compat[] __initconst = {
	"fsl,vf500",
	"fsl,vf510",
	"fsl,vf600",
	"fsl,vf610",
	"fsl,vf610m4",
	NULL,
};

DT_MACHINE_START(VYBRID_VF610, "Freescale Vybrid VF5xx/VF6xx (Device Tree)")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.dt_compat	= vf610_dt_compat,
	.init_late	= vf610_init_late,
MACHINE_END
