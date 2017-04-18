/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/clock/lpc17xx-clock.h>

/* Phase Locked Loop 0 (PLL0, Main PLL) */
#define LPC17XX_PLL0CON		0x080
#define LPC17XX_PLL0CFG		0x084
#define LPC17XX_PLL0STAT	0x088
#define LPC17XX_PLL0FEED	0x08C
/* Phase Locked Loop 1 (PLL1, Alt PLL) */
#define LPC17XX_PLL1CON		0x0A0
#define LPC17XX_PLL1CFG		0x0A4
#define LPC17XX_PLL1STAT	0x0A8
#define LPC17XX_PLL1FEED	0x0AC
/* Power control */
#define LPC17XX_PCON		0x0C0
#define LPC17XX_PCONP		0x0C4
/* Clock dividers */
#define LPC17XX_EMCCLKSEL	0x100
#define LPC17XX_CCLKSEL		0x104
#define LPC17XX_USBCLKSEL	0x108
/* System clock source selection */
#define LPC17XX_CLKSRCSEL	0x10C
/* Syscon Miscellaneous Registers */
#define LPC17XX_PCLKSEL		0x1A8
#define LPC17XX_SPIFICLKSEL	0x1B4

static const char* clk_names[32] = {
	"lcd",
	"tim0",
	"tim1",
	"uart0",
	"uart1",
	"pwm0",
	"pwm1",
	"i2c0",
	"uart4",
	"rtc",
	"ssp1",
	"emc",
	"adc",
	"can1",
	"can2",
	"gpio",
	"spifi",
	"mcpwm",
	"qei",
	"i2c1",
	"ssp2",
	"ssp0",
	"tim2",
	"tim3",
	"uart2",
	"uart3",
	"i2c2",
	"i2s",
	"sdc",
	"gpdma",
	"enet",
	"usb"
};

static struct clk *clk[CLOCK_END];
static struct clk_onecell_data clk_data = {
	.clks = clk,
	.clk_num = ARRAY_SIZE(clk),
};

static void __init lpc17xx_clk_init(struct device_node *np)
{
	struct clk *osc_clk = of_clk_get_by_name(np, "xtal");
	const char *p_parents[4] = { "xtal", "pll", "alt_pll", "alt_pll" };
	const char *cclk_parents[2] = { "xtal", "pll" };
	int pll_p_val[4] = {1, 2, 4, 8};
	void __iomem *sysctrl_base;
	int i, cclk_div, pclk_div, pll0_m, pll1_m, pll0_p, pll1_p;
	u32 pll0_cfg, pll1_cfg;
	bool emc_half;

	BUG_ON(!osc_clk);

	sysctrl_base = of_iomap(np, 0);
	BUG_ON(!sysctrl_base);

	cclk_div = readl(sysctrl_base + LPC17XX_CCLKSEL) & 0xF;
	pclk_div = readl(sysctrl_base + LPC17XX_PCLKSEL) & 0xF;
	BUG_ON(!cclk_div);
	BUG_ON(!pclk_div);

	emc_half = readl(sysctrl_base + LPC17XX_EMCCLKSEL) & 1;

	pll0_cfg = readl(sysctrl_base + LPC17XX_PLL0CFG);
	pll1_cfg = readl(sysctrl_base + LPC17XX_PLL1CFG);
	pll0_m = (pll0_cfg & 0xF) + 1;
	pll1_m = (pll1_cfg & 0xF) + 1;
	pll0_p = pll_p_val[(pll0_cfg >> 5) & 0x3];
	pll1_p = pll_p_val[(pll1_cfg >> 5) & 0x3];

	clk[CLOCK_PLL0] = clk_register_fixed_factor(NULL, "pll", "xtal", 0, pll0_m, pll0_p);
	clk[CLOCK_PLL1] = clk_register_fixed_factor(NULL, "alt_pll", "xtal", 0, pll1_m, pll1_p);
	clk[CLOCK_CCLKSEL] = clk_register_mux(NULL, "cclk_sel",
					      cclk_parents, ARRAY_SIZE(cclk_parents),
					      0,
					      sysctrl_base + LPC17XX_CCLKSEL, 8, 1,
					      0, NULL);
	clk[CLOCK_CCLK] = clk_register_fixed_factor(NULL, "cclk", "cclk_sel", 0, 1, cclk_div);
	clk[CLOCK_EMC] = clk_register_fixed_factor(NULL, "emc", "cclk", 0, 1, emc_half ? 2 : 1);
	clk[CLOCK_PCLK] = clk_register_fixed_factor(NULL, "pclk", "cclk_sel", 0, 1, pclk_div);

	pr_debug("%s: xtal %lu, cclk %lu, emc %lu, pclk %lu, alt_pll %lu\n", __func__,
	       clk_get_rate(osc_clk),
	       clk_get_rate(clk[CLOCK_CCLK]),
	       clk_get_rate(clk[CLOCK_EMC]),
	       clk_get_rate(clk[CLOCK_PCLK]),
	       clk_get_rate(clk[CLOCK_PLL1]));

	clk[CLOCK_USBSEL] = clk_register_mux(NULL, "usb_sel",
					      p_parents, ARRAY_SIZE(p_parents),
					      0,
					      sysctrl_base + LPC17XX_USBCLKSEL, 8, 2,
					      0, NULL);
	if (!IS_ERR_OR_NULL(clk[CLOCK_USBSEL])) {
		int usb_div = readl(sysctrl_base + LPC17XX_USBCLKSEL) & 0xF;
		if (usb_div)
			clk[CLOCK_USB] = clk_register_fixed_factor(NULL, "usb", "usb_sel", 0, 1, usb_div);
		else
			clk[CLOCK_USB] = ERR_PTR(-ENOENT);
	} else {
		clk[CLOCK_USB] = ERR_PTR(-ENOENT);
	}

	clk[CLOCK_SPIFISEL] = clk_register_mux(NULL, "spifi_sel",
					      p_parents, ARRAY_SIZE(p_parents),
					      0,
					      sysctrl_base + LPC17XX_SPIFICLKSEL, 8, 2,
					      0, NULL);
	if (!IS_ERR_OR_NULL(clk[CLOCK_SPIFISEL])) {
		int spifi_div = readl(sysctrl_base + LPC17XX_SPIFICLKSEL) & 0xF;
		if (spifi_div)
			clk[CLOCK_SPIFI] = clk_register_fixed_factor(NULL, "spifi", "spifi_sel", 0, 1, spifi_div);
		else
			clk[CLOCK_SPIFI] = ERR_PTR(-ENOENT);
	} else {
		clk[CLOCK_SPIFI] = ERR_PTR(-ENOENT);
	}

	for (i = 0; i < LPC17XX_REGULAR_CLOCKS; ++i) {
		if ((i != CLOCK_SPIFI) && (i != CLOCK_USB) && (i != CLOCK_EMC)) {
			clk[i] = clk_register_gate(NULL, clk_names[i], "pclk", 0,
						   sysctrl_base + LPC17XX_PCONP, i,
						   0, NULL);
		}
	}

	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}
CLK_OF_DECLARE(lpc17xx, "nxp,lpc17xx-clock", lpc17xx_clk_init);
