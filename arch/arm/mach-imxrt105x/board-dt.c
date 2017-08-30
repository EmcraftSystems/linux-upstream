/*
 * Copyright (C) Emcraft Systems 2017
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <asm/v7m.h>
#include <asm/mach/arch.h>

static const char *const imxrt105x_compat[] __initconst = {
	"nxp,imxrt105x",
	NULL
};

DT_MACHINE_START(IMXRT105XDT, "IMXRT105X (Device Tree Support)")
	.dt_compat = imxrt105x_compat,
	.restart = armv7m_restart,
MACHINE_END
