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

	/*
	 * DSPIx are clocked from the bus clock.
	 */
	clk[CLOCK_SPI0] = clk_register_gate(NULL, "DSPI0", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_SPI0)]),
			KINETIS_CG_IDX(KINETIS_CG_SPI0), 0, NULL);
	clk[CLOCK_SPI1] = clk_register_gate(NULL, "DSPI1", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_SPI1)]),
			KINETIS_CG_IDX(KINETIS_CG_SPI1), 0, NULL);
	clk[CLOCK_SPI2] = clk_register_gate(NULL, "DSPI2", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_SPI2)]),
			KINETIS_CG_IDX(KINETIS_CG_SPI2), 0, NULL);

	/*
	 * I2Cx are clocked from the bus clock.
	 */
	clk[CLOCK_I2C0] = clk_register_gate(NULL, "I2C0", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_I2C0)]),
			KINETIS_CG_IDX(KINETIS_CG_I2C0), 0, NULL);
	clk[CLOCK_I2C1] = clk_register_gate(NULL, "I2C1", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_I2C1)]),
			KINETIS_CG_IDX(KINETIS_CG_I2C1), 0, NULL);

	/*
	 * USBFS clock
	 */
	clk[CLOCK_USBFS] = clk_register_gate(NULL, "USBFS", "MCGOUTCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_USBFS)]),
			KINETIS_CG_IDX(KINETIS_CG_USBFS), 0, NULL);

	/*
	 * USBHS clock
	 */
	clk[CLOCK_USBHS] = clk_register_gate(NULL, "USBHS", "MCGOUTCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_USBHS)]),
			KINETIS_CG_IDX(KINETIS_CG_USBHS), 0, NULL);

	/*
	 * ADC clocks
	 */
	clk[CLOCK_ADC0] = clk_register_gate(NULL, "ADC0", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_ADC0)]),
			KINETIS_CG_IDX(KINETIS_CG_ADC0), 0, NULL);
	clk[CLOCK_ADC1] = clk_register_gate(NULL, "ADC1", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_ADC1)]),
			KINETIS_CG_IDX(KINETIS_CG_ADC1), 0, NULL);
	clk[CLOCK_ADC2] = clk_register_gate(NULL, "ADC2", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_ADC2)]),
			KINETIS_CG_IDX(KINETIS_CG_ADC2), 0, NULL);
	clk[CLOCK_ADC3] = clk_register_gate(NULL, "ADC3", "PCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_ADC3)]),
			KINETIS_CG_IDX(KINETIS_CG_ADC3), 0, NULL);

	/*
	 * FlexCAN clock
	 */
	clk[CLOCK_CAN0] = clk_register_gate(NULL, "CAN0", "MCGOUTCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_CAN0)]),
			KINETIS_CG_IDX(KINETIS_CG_CAN0), 0, NULL);
	clk[CLOCK_CAN1] = clk_register_gate(NULL, "CAN1", "MCGOUTCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_CAN1)]),
			KINETIS_CG_IDX(KINETIS_CG_CAN1), 0, NULL);

	/*
	 * LCDC clock
	 */
	clk[CLOCK_LCDC] = clk_register_gate(NULL, "LCDC", "MCGOUTCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_LCDC)]),
			KINETIS_CG_IDX(KINETIS_CG_LCDC), 0, NULL);

	/*
	 * ESDHC clock
	 */
	clk[CLOCK_ESDHC] = clk_register_gate(NULL, "ESDHC", "MCGOUTCLK", 0,
			KINETIS_SIM_PTR(scgc[KINETIS_CG_REG(KINETIS_CG_ESDHC)]),
			KINETIS_CG_IDX(KINETIS_CG_ESDHC), 0, NULL);

	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(kinetis_mcg, "fsl,kinetis-cmu", kinetis_mcg_init);
