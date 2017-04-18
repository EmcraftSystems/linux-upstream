/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <asm/mach/arch.h>
#include <asm/v7m.h>

static const char *const lpc17xx_compat[] __initconst = {
	"nxp,lpc178x/7x",
	NULL
};

DT_MACHINE_START(LPC18XXDT, "NXP LPC178x/7x (Device Tree)")
	.dt_compat = lpc17xx_compat,
	.restart = armv7m_restart,
MACHINE_END
