/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#ifndef _MACH_KINETIS_KINETIS_H
#define _MACH_KINETIS_KINETIS_H

#include <asm/byteorder.h>

/*
 * This Kinetis port assumes that the CPU works in little-endian mode.
 * Switching to big-endian will require different bit offsets in peripheral
 * devices' registers. Also, some bit groups may lay on byte edges, so issue
 * with big-endian cannot be fixed only by defining bit offsets differently
 * for the big-endian mode.
 */
#ifndef __LITTLE_ENDIAN
#error This Kinetis port assumes that the CPU works in little-endian mode
#endif

/*
 * Peripheral memory map
 */
#define KINETIS_AIPS0PERIPH_BASE	0x40000000
#define KINETIS_AIPS1PERIPH_BASE	0x40080000

#ifndef __ASSEMBLY__

#include <asm/types.h>

/*
 * Limits for the `kinetis_periph_enable()` function:
 *     1. The number of SIM_SCGC[] registers
 *     2. The number of bits in those registers
 */
#define KINETIS_SIM_CG_NUMREGS	7
#define KINETIS_SIM_CG_NUMBITS	32

/*
 * System Integration Module (SIM) register map
 *
 * This map actually covers two hardware modules:
 *     1. SIM low-power logic, at 0x40047000
 *     2. System integration module (SIM), at 0x40048000
 */
struct kinetis_sim_regs {
	u32 sopt1;	/* System Options Register 1 */
	u32 rsv0[1024];
	u32 sopt2;	/* System Options Register 2 */
	u32 rsv1;
	u32 sopt4;	/* System Options Register 4 */
	u32 sopt5;	/* System Options Register 5 */
	u32 sopt6;	/* System Options Register 6 */
	u32 sopt7;	/* System Options Register 7 */
	u32 rsv2[2];
	u32 sdid;	/* System Device Identification Register */
	u32 scgc[KINETIS_SIM_CG_NUMREGS];	/* Clock Gating Regs 1...7 */
	u32 clkdiv1;	/* System Clock Divider Register 1 */
	u32 clkdiv2;	/* System Clock Divider Register 2 */
	u32 fcfg1;	/* Flash Configuration Register 1 */
	u32 fcfg2;	/* Flash Configuration Register 2 */
	u32 uidh;	/* Unique Identification Register High */
	u32 uidmh;	/* Unique Identification Register Mid-High */
	u32 uidml;	/* Unique Identification Register Mid Low */
	u32 uidl;	/* Unique Identification Register Low */
	u32 clkdiv3;	/* System Clock Divider Register 3 */
	u32 clkdiv4;	/* System Clock Divider Register 4 */
	u32 mcr;	/* Misc Control Register */
};

/*
 * SIM registers base
 */
#define KINETIS_SIM_BASE		(KINETIS_AIPS0PERIPH_BASE + 0x00047000)
#define KINETIS_SIM_PTR(reg) \
	(&(((struct kinetis_sim_regs *)(KINETIS_SIM_BASE))->reg))
#define KINETIS_SIM_RD(reg) readl_relaxed(KINETIS_SIM_PTR(reg))
#define KINETIS_SIM_WR(reg, val) writel_relaxed((val), KINETIS_SIM_PTR(reg))
#define KINETIS_SIM_SET(reg, mask) \
	KINETIS_SIM_WR(reg, (KINETIS_SIM_RD(reg)) | (mask))
#define KINETIS_SIM_RESET(reg, mask) \
	KINETIS_SIM_WR(reg, (KINETIS_SIM_RD(reg)) & (~(mask)))
#define KINETIS_SIM_ISSET(reg, mask) \
	(KINETIS_SIM_RD(reg) & (mask))

/*
 * SIM registers
 */
/*
 * System Options Register 2
 */
/* USB HS clock source select */
#define KINETIS_SIM_SOPT2_USBHSRC_BITS	2
#define KINETIS_SIM_SOPT2_USBHSRC_MSK	(3 << KINETIS_SIM_SOPT2_USBHSRC_BITS)
#define KINETIS_SIM_SOPT2_USBHSRC_PLL0	(1 << KINETIS_SIM_SOPT2_USBHSRC_BITS)
#define KINETIS_SIM_SOPT2_USBHSRC_PLL1	(2 << KINETIS_SIM_SOPT2_USBHSRC_BITS)

/* USB FS clock source select */
#define KINETIS_SIM_SOPT2_USBFSRC_BITS	22
#define KINETIS_SIM_SOPT2_USBFSRC_MSK	(3 << KINETIS_SIM_SOPT2_USBFSRC_BITS)
#define KINETIS_SIM_SOPT2_USBFSRC_PLL0	(1 << KINETIS_SIM_SOPT2_USBFSRC_BITS)
#define KINETIS_SIM_SOPT2_USBFSRC_PLL1	(2 << KINETIS_SIM_SOPT2_USBFSRC_BITS)
#define KINETIS_SIM_SOPT2_USBF_CLKSEL	(1 << 18)

/*
 * System Clock Divider Register 1
 */
/* Clock 1 output divider value (for the core/system clock) */
#define KINETIS_SIM_CLKDIV1_OUTDIV1_BITS	28
#define KINETIS_SIM_CLKDIV1_OUTDIV1_MSK \
	(((1 << 4) - 1) << KINETIS_SIM_CLKDIV1_OUTDIV1_BITS)
/* Clock 2 output divider value (for the peripheral clock) */
#define KINETIS_SIM_CLKDIV1_OUTDIV2_BITS	24
#define KINETIS_SIM_CLKDIV1_OUTDIV2_MSK \
	(((1 << 4) - 1) << KINETIS_SIM_CLKDIV1_OUTDIV2_BITS)

/*
 * System Clock Divider Register 2
 */
/* USB HS clock divider fraction */
#define KINETIS_SIM_CLKDIV2_USBHSFRAC_BIT	8
#define KINETIS_SIM_CLKDIV2_USBHSFRAC_MSK \
	(1 << KINETIS_SIM_CLKDIV2_USBHSFRAC_BIT)
/* USB HS clock divider divisor */
#define KINETIS_SIM_CLKDIV2_USBHSDIV_BIT	9
#define KINETIS_SIM_CLKDIV2_USBHSDIV_MSK \
	(7 << KINETIS_SIM_CLKDIV2_USBHSDIV_BIT)

/* USB FS clock divider fraction */
#define KINETIS_SIM_CLKDIV2_USBFSFRAC_BIT	0
#define KINETIS_SIM_CLKDIV2_USBFSFRAC_MSK \
	(1 << KINETIS_SIM_CLKDIV2_USBFSFRAC_BIT)
/* USB FS clock divider divisor */
#define KINETIS_SIM_CLKDIV2_USBFSDIV_BIT	1
#define KINETIS_SIM_CLKDIV2_USBFSDIV_MSK \
	(7 << KINETIS_SIM_CLKDIV2_USBFSDIV_BIT)

/*
 * System Clock Divider Register 3
 */
/* LCD Controller clock divider divisor */
#define KINETIS_SIM_CLKDIV3_LCDCDIV_BITS	16
#define KINETIS_SIM_CLKDIV3_LCDCDIV_BITWIDTH	12
#define KINETIS_SIM_CLKDIV3_LCDCDIV_MSK \
	(((1 << KINETIS_SIM_CLKDIV3_LCDCDIV_BITWIDTH) - 1) << \
	KINETIS_SIM_CLKDIV3_LCDCDIV_BITS)
/* LCD Controller clock divider fraction */
#define KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS	8
#define KINETIS_SIM_CLKDIV3_LCDCFRAC_BITWIDTH	8
#define KINETIS_SIM_CLKDIV3_LCDCFRAC_MSK \
	(((1 << KINETIS_SIM_CLKDIV3_LCDCFRAC_BITWIDTH) - 1) << \
	KINETIS_SIM_CLKDIV3_LCDCFRAC_BITS)

/*
 * Misc Control Register
 */
/* 60 MHz ULPI clock (ULPI_CLK) output enable */
#define KINETIS_SIM_MCR_ULPICLKOBE_MSK	(1 << 30)
/* Start LCDC display */
#define KINETIS_SIM_MCR_LCDSTART_MSK	(1 << 16)

#endif /* __ASSEMBLY__ */

#endif /* _MACH_KINETIS_KINETIS_H */
