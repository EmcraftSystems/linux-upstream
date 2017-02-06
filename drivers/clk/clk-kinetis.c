/*
 * clk-kinetis.c - Clock driver for Kinetis K70 MCG
 *
 * Based on legacy pre-OF code by Alexander Potashev <aspotashev@emcraft.com>
 *
 * Copyright (C) 2015 Paul Osmialowski <pawelo@king.net.pl>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <mach/kinetis.h>
#include <mach/power.h>

#include <dt-bindings/clock/kinetis-mcg.h>

/*
 * Frequencies on OSC0 (EXTAL0/XTAL0) and OSC1 (EXTAL1/XTAL1)
 *
 * These frequencies should be set to the same values as in U-Boot.
 */
#define KINETIS_OSC0_RATE	50000000	/* 50 MHz */
#define KINETIS_OSC1_RATE	12000000	/* 12 MHz */

/*
 * MCG Control 5 Register
 */
/* PLL External Reference Divider */
#define KINETIS_MCG_C5_PRDIV_BITS	0
#define KINETIS_MCG_C5_PRDIV_MSK \
	(((1 << 3) - 1) << KINETIS_MCG_C5_PRDIV_BITS)
/* PLL Stop Enable */
#define KINETIS_MCG_C5_PLLSTEN_MSK	(1 << 5)
/* PLL Clock Enable */
#define KINETIS_MCG_C5_PLLCLKEN_MSK	(1 << 6)
/* PLL External Reference Select (for K70@120MHz) */
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

static struct clk *clk[CLOCK_END];
static struct clk_onecell_data clk_data = {
	.clks = clk,
	.clk_num = ARRAY_SIZE(clk),
};

static void __init kinetis_mcg_init(struct device_node *np)
{
	const int vco_div = 2;
	const int vdiv_min = 16;
	u32 clock_val[CLOCK_END];
	int i;
	void __iomem *base;
	int pll_sel;
	int osc_sel;
	unsigned long mcgout;

	for (i = 0; i < ARRAY_SIZE(clk); ++i)
		clk[i] = ERR_PTR(-ENOENT);

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("Failed to map address range for kinetis,mcg node\n");
		return;
	}

	/*
	 * Check whether PLL0 or PLL1 is used for MCGOUTCLK
	 */
	pll_sel = !!(KINETIS_MCG_ISSET(base, c11, KINETIS_MCG_C11_PLLCS_MSK));

	/*
	 * Check whether OSC0 or OSC1 is used to source the main PLL
	 */
	if (pll_sel)
		osc_sel = !!(KINETIS_MCG_ISSET(base, c11,
				KINETIS_MCG_C11_PLLREFSEL1_MSK));
	else
		osc_sel = !!(KINETIS_MCG_ISSET(base, c5,
				KINETIS_MCG_C5_PLLREFSEL_MSK));

	/*
	 * Start with the MCG input clock
	 */
	mcgout = osc_sel ? KINETIS_OSC1_RATE : KINETIS_OSC0_RATE;

	/*
	 * Apply dividers and multipliers of the selected PLL
	 */
	if (pll_sel) {
		/*
		 * PLL1 internal divider (PRDIV)
		 */
		mcgout /= ((KINETIS_MCG_RD(base, c11) &
		  KINETIS_MCG_C11_PRDIV_MSK) >> KINETIS_MCG_C11_PRDIV_BITS) + 1;
		/*
		 * PLL1 multiplication factor (VDIV)
		 */
		mcgout *= ((KINETIS_MCG_RD(base, c12) &
		  KINETIS_MCG_C12_VDIV1_MSK) >> KINETIS_MCG_C12_VDIV1_BITS) +
								    vdiv_min;
	} else {
		/*
		 * PLL0 internal divider (PRDIV)
		 */
		mcgout /= ((KINETIS_MCG_RD(base, c5) &
			KINETIS_MCG_C5_PRDIV_MSK) >>
			KINETIS_MCG_C5_PRDIV_BITS) + 1;
		/*
		 * PLL0 multiplication factor (VDIV)
		 */
		mcgout *= ((KINETIS_MCG_RD(base, c6) &
			KINETIS_MCG_C6_VDIV_MSK) >>
			KINETIS_MCG_C6_VDIV_BITS) + vdiv_min;
	}

	/*
	 * Apply the PLL output divider
	 */
	mcgout /= vco_div;

	clock_val[CLOCK_MCGOUTCLK] = mcgout;

	clock_val[CLOCK_CCLK] = mcgout /
		(((KINETIS_SIM_RD(clkdiv1) & KINETIS_SIM_CLKDIV1_OUTDIV1_MSK) >>
		KINETIS_SIM_CLKDIV1_OUTDIV1_BITS) + 1);

	/*
	 * Peripheral (bus) clock
	 */
	clock_val[CLOCK_PCLK] = mcgout /
		(((KINETIS_SIM_RD(clkdiv1) & KINETIS_SIM_CLKDIV1_OUTDIV2_MSK) >>
		KINETIS_SIM_CLKDIV1_OUTDIV2_BITS) + 1);

	clk[CLOCK_MCGOUTCLK] = clk_register_fixed_rate(NULL, "MCGOUTCLK",
			NULL, CLK_IS_ROOT, clock_val[CLOCK_MCGOUTCLK]);

	clk[CLOCK_CCLK] = clk_register_fixed_rate(NULL, "CCLK", "MCGOUTCLK",
			0, clock_val[CLOCK_CCLK]);

	clk[CLOCK_PCLK] = clk_register_fixed_rate(NULL, "PCLK", "MCGOUTCLK",
			0, clock_val[CLOCK_PCLK]);

	clk[CLOCK_PIT] = clk_register_gate(NULL, "PIT", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_PIT)]),
			KINETIS_CG_IDX(KINETIS_CG_PIT), 0, NULL);

	/*
	 * Port control clock gates,
	 *   see K70 Sub-Family Reference Manual, Rev. 3 pg. 223:
	 */
	clk[CLOCK_PORTA] = clk_register_gate(NULL, "PORTA", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_PORTA)]),
			KINETIS_CG_IDX(KINETIS_CG_PORTA), 0, NULL);

	clk[CLOCK_PORTB] = clk_register_gate(NULL, "PORTB", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_PORTB)]),
			KINETIS_CG_IDX(KINETIS_CG_PORTB), 0, NULL);

	clk[CLOCK_PORTC] = clk_register_gate(NULL, "PORTC", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_PORTC)]),
			KINETIS_CG_IDX(KINETIS_CG_PORTC), 0, NULL);

	clk[CLOCK_PORTD] = clk_register_gate(NULL, "PORTD", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_PORTD)]),
			KINETIS_CG_IDX(KINETIS_CG_PORTD), 0, NULL);

	clk[CLOCK_PORTE] = clk_register_gate(NULL, "PORTE", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_PORTE)]),
			KINETIS_CG_IDX(KINETIS_CG_PORTE), 0, NULL);

	clk[CLOCK_PORTF] = clk_register_gate(NULL, "PORTF", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_PORTF)]),
			KINETIS_CG_IDX(KINETIS_CG_PORTF), 0, NULL);

	/*
	 * DMA clock gates, see K70 Sub-Family Reference Manual, Rev. 3 pg. 223:
	 */
	clk[CLOCK_EDMA] = clk_register_gate(NULL, "EDMA", "CCLK", 0,
		KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_DMA)]),
		KINETIS_CG_IDX(KINETIS_CG_DMA), 0, NULL);

	clk[CLOCK_DMAMUX0] = clk_register_gate(NULL, "DMAMUX0", "PCLK", 0,
		KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_DMAMUX0)]),
		KINETIS_CG_IDX(KINETIS_CG_DMAMUX0), 0, NULL);

	clk[CLOCK_DMAMUX1] = clk_register_gate(NULL, "DMAMUX1", "PCLK", 0,
		KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_DMAMUX1)]),
		KINETIS_CG_IDX(KINETIS_CG_DMAMUX1), 0, NULL);

	/*
	 * UART0 and UART1 are clocked from the core clock, the remaining UARTs
	 * are clocked from the bus clock.
	 */
	clk[CLOCK_UART0] = clk_register_gate(NULL, "UART0", "CCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_UART0)]),
			KINETIS_CG_IDX(KINETIS_CG_UART0), 0, NULL);

	clk[CLOCK_UART1] = clk_register_gate(NULL, "UART1", "CCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_UART1)]),
			KINETIS_CG_IDX(KINETIS_CG_UART1), 0, NULL);

	clk[CLOCK_UART2] = clk_register_gate(NULL, "UART2", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_UART2)]),
			KINETIS_CG_IDX(KINETIS_CG_UART2), 0, NULL);

	clk[CLOCK_UART3] = clk_register_gate(NULL, "UART3", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_UART3)]),
			KINETIS_CG_IDX(KINETIS_CG_UART3), 0, NULL);

	clk[CLOCK_UART4] = clk_register_gate(NULL, "UART4", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_UART4)]),
			KINETIS_CG_IDX(KINETIS_CG_UART4), 0, NULL);

	clk[CLOCK_UART5] = clk_register_gate(NULL, "UART5", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_UART5)]),
			KINETIS_CG_IDX(KINETIS_CG_UART5), 0, NULL);

	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(kinetis_mcg, "fsl,kinetis-cmu", kinetis_mcg_init);
