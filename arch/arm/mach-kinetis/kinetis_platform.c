/*
 * kinetis_platform.c - Freescale Kinetis K70F120M Development Board
 *
 * Based on legacy pre-OF code by Alexander Potashev <aspotashev@emcraft.com>
 *
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * Copyright (C) 2015 Paul Osmialowski <pawelo@king.net.pl>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <asm/v7m.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init kinetis_map_io(void)
{
}

/*
 * Freescale Kinetis platform initialization
 */
static void __init kinetis_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char *const kinetis_compat[] __initconst = {
	"fsl,kinetis-twr-k70f120m",
	"emcraft,k70-som",
	NULL
};

/*
 * Freescale Kinetis platform machine description
 */
DT_MACHINE_START(KINETIS, "Freescale Kinetis")
	.dt_compat	= kinetis_compat,
	.restart	= armv7m_restart,
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging.
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.map_io		= kinetis_map_io,
	.init_machine	= kinetis_init,
MACHINE_END
