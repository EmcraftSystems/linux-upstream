/*
 * (C) Copyright 2012-2017
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Alexander Yurtsev <alex@emcraft.com>
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
 * Cortex-M4 System Control Block (SCB) register map
 */
struct kinetis_scb_regs {
	u32 cpuid;	/* CPUID Register */
	u32 icsr;	/* Interrupt Control and State Register */
	u32 rsrv0;
	u32 aircr;	/* Application Interrupt and Reset Control Register */
	u32 scr;	/* System Control Register */
	u32 ccr;	/* Configuration and Control Register */
};

/*
 * SCB registers base
 */
#define KINETIS_SCB_BASE	0xE000ED00
#define KINETIS_SCB		((volatile struct kinetis_scb_regs *) \
				KINETIS_SCB_BASE)
/* SCR reg bits */
#define KINETIS_SCB_SCR_DEEPSLEEP	(1 << 2)


/*
 * System Mode Controller (SMC) register map
 */
struct kinetis_smc_regs {
	u8 pmprot;	/* Power Mode Protection Register */
	u8 pmctrl;	/* Power Mode Control Register */
	u8 vllsctrl;	/* VLLS Control Register */
	u8 pmstat;	/* Power Mode Status Register */
};

/*
 * SMC registers base
 */
#define KINETIS_SMC_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x0007E000)
#define KINETIS_SMC		((volatile struct kinetis_smc_regs *) \
				KINETIS_SMC_BASE)

/* SMC PMPROT reg bits */

#define KINETIS_SMC_PMRPOT_AVLP		(1 << 5)
#define KINETIS_SMC_PMCTRL_STOPM_RUN	0x0
#define KINETIS_SMC_PMCTRL_STOPM_VLPS	0x2

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
	u32 sopt1cfg;	/* SOPT1 Configuration Register */
	u32 rsv0[1023]; /* reserved */
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
 * System Options Register 1
 */
/* USB voltage regulator in standby mode during Stop, VLPS, LLS or VLLS */
#define KINETIS_SIM_SOPT1_USBSSTBY	(1 << 30)

/*
 *  SOPT1 Configuration Register
 */
/* USB voltage regulator stop standby write enable */
#define KINETIS_SIM_SOPT1CFG_USSWE	(1 << 26)

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
 * Flash Configuration Register 1
 */
#define KINETIS_SIM_FCFG1_FTFDIS	(1 << 0)

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

/*
 * Multipurpose Clock Generator (MCG) register map
 *
 * See Chapter 25 of the K70 Reference Manual
 */
struct kinetis_mcg_regs {
	u8 c1;		/* MCG Control 1 Register */
	u8 c2;		/* MCG Control 2 Register */
	u8 c3;		/* MCG Control 3 Register */
	u8 c4;		/* MCG Control 4 Register */
	u8 c5;		/* MCG Control 5 Register */
	u8 c6;		/* MCG Control 6 Register */
	u8 status;	/* MCG Status Register */
	u8 rsv0;
	u8 atc;		/* MCG Auto Trim Control Register */
	u8 rsv1;
	u8 atcvh;	/* MCG Auto Trim Compare Value High Register */
	u8 atcvl;	/* MCG Auto Trim Compare Value Low Register */
	u8 c7;		/* MCG Control 7 Register */
	u8 c8;		/* MCG Control 8 Register */
	u8 rsv2;
	u8 c10;		/* MCG Control 10 Register */
	u8 c11;		/* MCG Control 11 Register */
	u8 c12;		/* MCG Control 12 Register */
	u8 status2;	/* MCG Status 2 Register */
	u8 rsv3;
};

#define KINETIS_MCG_PTR(base, reg) \
	(&(((struct kinetis_mcg_regs *)(base))->reg))
#define KINETIS_MCG_RD(base, reg) readb_relaxed(KINETIS_MCG_PTR(base, reg))
#define KINETIS_MCG_WR(base, reg, val) \
	writeb_relaxed((val), KINETIS_MCG_PTR(base, reg))
#define KINETIS_MCG_ISSET(base, reg, mask) \
	(KINETIS_MCG_RD(base, reg) & (mask))

#define KINETIS_MCG_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00064000)
#define KINETIS_MCG_RD_CONST(reg) KINETIS_MCG_RD(KINETIS_MCG_BASE, reg)
#define KINETIS_MCG_WR_CONST(reg, val) \
	KINETIS_MCG_WR(KINETIS_MCG_BASE, reg, val)

/*
 * MCG Control 1 Register
 */
/* Internal Reference Stop Enable */
#define KINETIS_MCG_C1_IREFSTEN		(1 << 0)

/*
 * MCG Control 2 Register
 */
/* Low Power Select */
#define KINETIS_MCG_C2_LP		(1 << 1)
/* Internal Reference Clock Select */
#define KINETIS_MCG_C2_IRCS		(1 << 0)

/*
 * MCG Control 5 Register
 */
/* PLL External Reference Divider */
#define KINETIS_MCG_C5_PRDIV_BITS	0
#define KINETIS_MCG_C5_PRDIV_MSK \
	(((1 << 3) - 1) << KINETIS_MCG_C5_PRDIV_BITS)
/* PLL0 Stop Enable */
#define KINETIS_MCG_C5_PLLSTEN0_MSK	(1 << 5)
/* PLL0 Clock Enable */
#define KINETIS_MCG_C5_PLLCLKEN_MSK	(1 << 6)
/* PLL0 External Reference Select (for K70@120MHz) */
#define KINETIS_MCG_C5_PLLREFSEL_BIT	7
#define KINETIS_MCG_C5_PLLREFSEL_MSK	(1 << KINETIS_MCG_C5_PLLREFSEL_BIT)

/*
 * MCG Control 6 Register
 */
/* VCO Divider */
#define KINETIS_MCG_C6_VDIV_BITS	0
#define KINETIS_MCG_C6_VDIV_MSK \
	(((1 << 5) - 1) << KINETIS_MCG_C6_VDIV_BITS)
/* PLL Select */
#define KINETIS_MCG_C6_PLLS_MSK		(1 << 6)

/*
 * MCG Control 11 Register
 */
/* PLL1 External Reference Divider */
#define KINETIS_MCG_C11_PRDIV_BITS	0
#define KINETIS_MCG_C11_PRDIV_MSK \
	(((1 << 3) - 1) << KINETIS_MCG_C11_PRDIV_BITS)
/* PLL Clock Select: PLL0 or PLL1 */
#define KINETIS_MCG_C11_PLLCS_MSK	(1 << 4)
/* PLL1 Stop Enable */
#define KINETIS_MCG_C11_PLLSTEN1_MSK	(1 << 5)
/* PLL1 Clock Enable */
#define KINETIS_MCG_C11_PLLCLKEN1_MSK	(1 << 6)
/* PLL1 External Reference Select (for K70@120MHz) */
#define KINETIS_MCG_C11_PLLREFSEL1_BIT	7
#define KINETIS_MCG_C11_PLLREFSEL1_MSK	(1 << KINETIS_MCG_C11_PLLREFSEL1_BIT)

/*
 * MCG Control 12 Register
 */
/* VCO1 Divider */
#define KINETIS_MCG_C12_VDIV1_BITS	0
#define KINETIS_MCG_C12_VDIV1_MSK \
	(((1 << 5) - 1) << KINETIS_MCG_C12_VDIV1_BITS)

/*
 * Oscillator (OSC) register map
 */
struct kinetis_osc_regs {
	u8 cr;		/* OSC Control Register */
};

/*
 * OSC registers base
 */
#define KINETIS_OSC0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00065000)
#define KINETIS_OSC0		((volatile struct kinetis_osc_regs *) \
				KINETIS_OSC0_BASE)
#define KINETIS_OSC1_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x000E5000)
#define KINETIS_OSC1		((volatile struct kinetis_osc_regs *) \
				KINETIS_OSC1_BASE)
/*
 * OSC Control registers
 */
/* External Reference Stop Enable */
#define KINETIS_OSCx_CR_EREFSTEN	(1 << 5)

/*
 * Watchdog (WDOG) register map
 */
struct kinetis_wdog_regs {
	u16 stctrlh;	/* Watchdog Status & Control Register High */
	u16 stctrll;	/* Watchdog Status & Control Register Low */
};

/*
 * WDOG registers base
 */
#define KINETIS_WDOG_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00052000)
#define KINETIS_WDOG		((volatile struct kinetis_wdog_regs *) \
				KINETIS_WDOG_BASE)

/*
 * WDIG Control Register HIGH
 */
/* Enables or disables WDOG in Stop mode */
#define KINETIS_WDOG_STCTRLH_STOPEN	(1 << 6)

#endif /* __ASSEMBLY__ */
#endif /* _MACH_KINETIS_KINETIS_H */
