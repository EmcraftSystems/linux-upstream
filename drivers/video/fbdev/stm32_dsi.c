/*
 * Copyright (C) 2016 STMicroelectronics
 * MCD Application Team <www.st.com>
 *
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * STM32 DSI device driver
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/freezer.h>
#include <linux/kthread.h>

#include "stm32_dsi.h"

#define STM32_DSI_CCR_DIV_MASK		0xff
#define STM32_DSI_CCR_TOCKDIV(x)	(((x) & STM32_DSI_CCR_DIV_MASK) << 8)
#define STM32_DSI_CCR_TXECKDIV(x)	(((x) & STM32_DSI_CCR_DIV_MASK) << 0)

#define STM32_DSI_PCR_ETTXE		BIT(0)
#define STM32_DSI_PCR_ETRXE		BIT(1)
#define STM32_DSI_PCR_BTAE		BIT(2)
#define STM32_DSI_PCR_ECCRXE		BIT(3)
#define STM32_DSI_PCR_CRCRXE		BIT(4)

#define STM32_DSI_GPSR_CMDFE		BIT(0)
#define STM32_DSI_GPSR_CMDFF		BIT(1)
#define STM32_DSI_GPSR_PWRFE		BIT(2)
#define STM32_DSI_GPSR_PWRFF		BIT(3)
#define STM32_DSI_GPSR_PRDFE		BIT(4)
#define STM32_DSI_GPSR_PRDFF		BIT(5)
#define STM32_DSI_GPSR_RCB		BIT(6)

#define STM32_DSI_GVCIDR_VCID_MASK	0x3
#define STM32_DSI_GVCIDR_VCID(x)	(((x) & STM32_DSI_GVCIDR_VCID_MASK) << 0)

#define STM32_DSI_LCOLCR_COLC_MASK	0xf
#define STM32_DSI_LCOLCR_COLC(x)	(((x) & STM32_DSI_LCOLCR_COLC_MASK) << 0)
#define STM32_DSI_LCOLCR_COLC_24BIT	(STM32_DSI_LCOLCR_COLC(5))

#define STM32_DSI_CLCR_DPCC		BIT(0)
#define STM32_DSI_CLCR_ACR		BIT(1)

#define STM32_DSI_PCTLR_DEN		BIT(1)
#define STM32_DSI_PCTLR_CKE		BIT(2)

#define STM32_DSI_PCONFR_NL_MASK	0x3
#define STM32_DSI_PCONFR_NL(x)		(((x) & STM32_DSI_PCONFR_NL_MASK) << 0)
#define STM32_DSI_PCONFR_NL_ONE_LANE	(STM32_DSI_PCONFR_NL(0))
#define STM32_DSI_PCONFR_NL_TWO_LANES	(STM32_DSI_PCONFR_NL(1))
#define STM32_DSI_PCONFR_SW_TIME_MASK	0xff
#define STM32_DSI_PCONFR_SW_TIME(x)	(((x) & STM32_DSI_PCONFR_SW_TIME_MASK) << 8)

#define STM32_DSI_CLTCR_MASK		0x3ff
#define STM32_DSI_CLTCR_HS2LP(x)	(((x) & STM32_DSI_CLTCR_MASK) << 16)
#define STM32_DSI_CLTCR_LP2HS(x)	(((x) & STM32_DSI_CLTCR_MASK) << 0)

#define STM32_DSI_DLTCR_LPHS_MASK	0xff
#define STM32_DSI_DLTCR_HS2LP(x)	(((x) & STM32_DSI_DLTCR_LPHS_MASK) << 24)
#define STM32_DSI_DLTCR_LP2HS(x)	(((x) & STM32_DSI_DLTCR_LPHS_MASK) << 16)
#define STM32_DSI_DLTCR_MRD_TIME_MASK	0x7ff
#define STM32_DSI_DLTCR_MRD_TIME(x)	(((x) & STM32_DSI_DLTCR_MRD_TIME_MASK) << 0)

#define STM32_DSI_LCCR_CMDSIZE_MASK	0xffff
#define STM32_DSI_LCCR_CMDSIZE(x)	(((x) & STM32_DSI_LCCR_CMDSIZE_MASK) << 0)

#define STM32_DSI_CMCR_TEARE		BIT(0)
#define STM32_DSI_CMCR_ARE		BIT(1)
#define STM32_DSI_CMCR_GSW0TX		BIT(8)
#define STM32_DSI_CMCR_GSW1TX		BIT(9)
#define STM32_DSI_CMCR_GSW2TX		BIT(10)
#define STM32_DSI_CMCR_GSR0TX		BIT(11)
#define STM32_DSI_CMCR_GSR1TX		BIT(12)
#define STM32_DSI_CMCR_GSR2TX		BIT(13)
#define STM32_DSI_CMCR_GLWTX		BIT(14)
#define STM32_DSI_CMCR_DSW0TX		BIT(16)
#define STM32_DSI_CMCR_DSW1TX		BIT(17)
#define STM32_DSI_CMCR_DSR0TX		BIT(18)
#define STM32_DSI_CMCR_DLWTX		BIT(19)
#define STM32_DSI_CMCR_MRDPS		BIT(24)

#define STM32_DSM_MCR_CMDM		BIT(0)

#define STM32_DSI_WCFGR_DSIM		BIT(0)
#define STM32_DSI_WCFGR_COLMUX_MASK	0xff
#define STM32_DSI_WCFGR_COLMUX(x)	(((x) & STM32_DSI_WCFGR_COLMUX_MASK) << 1)
#define STM32_DSI_WCFGR_COLMUX_24BIT	(STM32_DSI_WCFGR_COLMUX(5))
#define STM32_DSI_WCFGR_TESRC		BIT(4)
#define STM32_DSI_WCFGR_TEPOL_FALLING	BIT(5)
#define STM32_DSI_WCFGR_AR		BIT(6)
#define STM32_DSI_WCFGR_VSPOL_RISING	BIT(7)

#define STM32_DSI_WCR_COLM		BIT(0)
#define STM32_DSI_WCR_SHTDN		BIT(1)
#define STM32_DSI_WCR_LTDCEN		BIT(2)
#define STM32_DSI_WCR_DSIEN		BIT(3)

#define STM32_DSI_WIER_TEIE		BIT(0)
#define STM32_DSI_WIER_ERIE		BIT(1)
#define STM32_DSI_WIER_PLLLIE		BIT(9)
#define STM32_DSI_WIER_PLLUIE		BIT(10)
#define STM32_DSI_WIER_RRIE		BIT(13)

#define STM32_DSI_WISR_TEIF		BIT(0)
#define STM32_DSI_WISR_ERIF		BIT(1)
#define STM32_DSI_WISR_BUSY		BIT(2)
#define STM32_DSI_WISR_PLLLS		BIT(8)
#define STM32_DSI_WISR_PLLLIF		BIT(9)
#define STM32_DSI_WISR_PLLUIF		BIT(10)
#define STM32_DSI_WISR_RRS		BIT(12)
#define STM32_DSI_WISR_RRIF		BIT(13)

#define STM32_DSI_WIFCR_CTEIF		BIT(0)
#define STM32_DSI_WIFCR_CERIF		BIT(1)
#define STM32_DSI_WIFCR_CPLLLIF		BIT(9)
#define STM32_DSI_WIFCR_CPLLUIF		BIT(10)
#define STM32_DSI_WIFCR_CRRIF		BIT(13)

#define STM32_DSI_WPCR0_UIX4_MASK	0x3f
#define STM32_DSI_WPCR0_UIX4(x)		(((x) & STM32_DSI_WPCR0_UIX4_MASK) << 0)

#define STM32_DSI_WRPCR_PLLEN		BIT(0)
#define STM32_DSI_WRPCR_NDIV_MASK	0x7f
#define STM32_DSI_WRPCR_NDIV(x)		(((x) & STM32_DSI_WRPCR_NDIV_MASK) << 2)
#define STM32_DSI_WRPCR_IDF_MASK	0xf
#define STM32_DSI_WRPCR_IDF(x)		(((x) & STM32_DSI_WRPCR_IDF_MASK) << 11)
#define STM32_DSI_WRPCR_ODF_MASK	0x3
#define STM32_DSI_WRPCR_ODF(x)		(((x) & STM32_DSI_WRPCR_ODF_MASK) << 16)
#define STM32_DSI_WRPCR_ODF_1		(STM32_DSI_WRPCR_ODF(0))
#define STM32_DSI_WRPCR_ODF_2		(STM32_DSI_WRPCR_ODF(1))
#define STM32_DSI_WRPCR_ODF_4		(STM32_DSI_WRPCR_ODF(2))
#define STM32_DSI_WRPCR_ODF_8		(STM32_DSI_WRPCR_ODF(3))
#define STM32_DSI_WRPCR_REGEN		BIT(24)

#define STM32_DSI_HS2LP_TIME		35

#define STM32_DSI_TIMEOUT_MS		100

#define STM32_DSI_SHORT_PKT_WRITE_P1	0x15
#define STM32_DSI_LONG_PKT_WRITE	0x39

struct stm32_dsi_regs {
	u32	vr;
	u32	cr;
	u32	ccr;
	u32	lvcidr;
	u32	lcolcr;
	u32	lpcr;
	u32	lpmcr;
	u32	reserved1[4];
	u32	pcr;
	u32	gvcidr;
	u32	mcr;
	u32	vmcr;
	u32	vpcr;
	u32	vccr;
	u32	vnpcr;
	u32	vhsacr;
	u32	vhbpcr;
	u32	vlcr;
	u32	vvsacr;
	u32	vvbpcr;
	u32	vvfpcr;
	u32	vvacr;
	u32	lccr;
	u32	cmcr;
	u32	ghcr;
	u32	gpdr;
	u32	gpsr;
	u32	tccr0;
	u32	tccr1;
	u32	tccr2;
	u32	tccr3;
	u32	tccr4;
	u32	tccr5;
	u32	reserved2;
	u32	clcr;
	u32	cltcr;
	u32	dltcr;
	u32	pctlr;
	u32	pconfr;
	u32	pucr;
	u32	pttcr;
	u32	psr;
	u32	reserved3[2];
	u32	isr0;
	u32	isr1;
	u32	ier0;
	u32	ier1;
	u32	reserved4[3];
	u32	fir0;
	u32	fir1;
	u32	reserved5[8];
	u32	vscr;
	u32	reserved6[2];
	u32	lcvcidr;
	u32	lcccr;
	u32	reserved7;
	u32	lpmccr;
	u32	reserved8[7];
	u32	vmccr;
	u32	vpccr;
	u32	vcccr;
	u32	vnpccr;
	u32	vhsaccr;
	u32	vhbpccr;
	u32	vlccr;
	u32	vvsaccr;
	u32	vvbpccr;
	u32	vvfpccr;
	u32	vvaccr;
	u32	reserved9[167];
	u32	wcfgr;
	u32	wcr;
	u32	wier;
	u32	wisr;
	u32	wifcr;
	u32	reserved10;
	u32	wpcr0;
	u32	wpcr1;
	u32	wpcr2;
	u32	wpcr3;
	u32	wpcr4;
	u32	reserved11;
	u32	wrpcr;
};

struct stm32_dsi_struct {
	struct device		*dev;
	struct stm32_dsi_regs __iomem *regs;
	struct clk		*clk;
	struct gpio_desc	*backlight_gpio;
	struct gpio_desc	*reset_gpio;
	int			refresh_period_ms;
};

static inline void stm32_dsi_send(struct stm32_dsi_struct *stm32_dsi,
				  u32 channel_id, u32 data_type,
				  u32 d0, u32 d1)
{
	stm32_dsi->regs->ghcr = data_type
		| (channel_id << 6)
		| (d0 << 8)
		| (d1 << 16);
}

int stm32_dsi_gpsr_wait(struct stm32_dsi_struct *stm32_dsi,
			u32 __iomem *reg, u32 mask, bool active)
{
	int i, timeout = STM32_DSI_TIMEOUT_MS * 1000;

	for (i = 0; i < timeout; ++i) {
		u32 status = *reg & mask;

		if ((active && status) || (!active && !status))
			break;

		udelay(1);
	}

	if (i == timeout)
		dev_err(stm32_dsi->dev, "%s: TIMEOUT: reg %p, mask 0x%x, active %d, status 0x%x\n",
			__func__, reg, mask, active, *reg);

	return (i == timeout) ? -1 : 0;
}

int stm32_dsi_write_short(struct stm32_dsi_struct *stm32_dsi,
			  u32 channel_id, u32 mode,
			  u32 p1, u32 p2)
{
	stm32_dsi_gpsr_wait(stm32_dsi, &stm32_dsi->regs->gpsr, STM32_DSI_GPSR_CMDFE, true);
	stm32_dsi_send(stm32_dsi, channel_id, mode, p1, p2);

	return 0;
}

int stm32_dsi_write_long(struct stm32_dsi_struct *stm32_dsi,
		      u32 channel_id, u32 Mode,
		      u32 length, u32 p1,
		      u8* parameters_table)
{
	u32 uicounter = 0;

	stm32_dsi_gpsr_wait(stm32_dsi, &stm32_dsi->regs->gpsr, STM32_DSI_GPSR_CMDFE, true);

	/* Set the DCS code hexadecimal on payload byte 1, and the other parameters on the write FIFO command*/
	while (uicounter < length) {
		if (uicounter == 0x00) {
			stm32_dsi->regs->gpdr = (p1 | \
					   ((u32)(*(parameters_table + uicounter)) << 8) | \
					   ((u32)(*(parameters_table + uicounter+1)) <<16) | \
					   ((u32)(*(parameters_table + uicounter+2)) <<24));
			uicounter += 3;
		} else {
			stm32_dsi->regs->gpdr = ((u32)(*(parameters_table + uicounter)) | \
					   ((u32)(*(parameters_table + uicounter+1)) << 8) | \
					   ((u32)(*(parameters_table + uicounter+2)) << 16) | \
					   ((u32)(*(parameters_table + uicounter+3)) << 24));
			uicounter += 4;
		}
	}

	stm32_dsi_send(stm32_dsi, channel_id, Mode,
		       ((length + 1) & 0x00FF),
		       (((length + 1) & 0xFF00)>>8));

	return 0;
}

void stm32_dsi_write_cmd(void *dsi, int length, u8 *p)
{
	struct stm32_dsi_struct *stm32_dsi = dsi;

	if (length <= 1)
		stm32_dsi_write_short(stm32_dsi, 0, STM32_DSI_SHORT_PKT_WRITE_P1, p[0], p[1]);
	else
		stm32_dsi_write_long(stm32_dsi, 0, STM32_DSI_LONG_PKT_WRITE, length, p[length], p);
}

static int stm32_dsi_refresh_task(void *param)
{
	struct stm32_dsi_struct *stm32_dsi = param;

	set_freezable();

	while (!kthread_should_stop()) {
		stm32_dsi->regs->wcr |= STM32_DSI_WCR_LTDCEN;

		schedule_timeout_interruptible(msecs_to_jiffies(stm32_dsi->refresh_period_ms));

		try_to_freeze();
	}

	return 0;
}

static int stm32_dsi_probe(struct platform_device *pdev)
{
	struct stm32_dsi_struct *stm32_dsi;
	struct resource *res;
	int err;

	struct device_node *ltdc_node;
	struct platform_device *ltdc_pdev;
	bool ltdc_ready;
	struct device_node *panel_node;
	struct platform_device *panel_pdev;
	struct stm32_dsi_panel *panel;
	struct display_timings *timings;
	struct device_node *display_node;
	int i;
	int hactive = -1;
	u32 lanes = 0, te = 0, refresh_period_ms = 0;

	ltdc_node = of_find_compatible_node(NULL, NULL, "st,stm32f4-ltdc");
	panel_node = of_parse_phandle(pdev->dev.of_node, "panel", 0);
	if (!ltdc_node) {
		dev_err(&pdev->dev, "failed to find LTDC node\n");
		return -ENOENT;
	}
	if (!panel_node) {
		dev_err(&pdev->dev, "failed to find panel phandle\n");
		return -ENOENT;
	}

	ltdc_pdev = of_find_device_by_node(ltdc_node);
	panel_pdev = of_find_device_by_node(panel_node);
	if (!ltdc_pdev) {
		dev_err(&pdev->dev, "failed to find LTDC device\n");
		return -ENOENT;
	}
	if (!panel_pdev) {
		dev_err(&pdev->dev, "failed to find panel device\n");
		return -ENOENT;
	}

	ltdc_ready = !!platform_get_drvdata(ltdc_pdev);
	panel = platform_get_drvdata(panel_pdev);
	if (!ltdc_ready) {
		dev_info(&pdev->dev, "LTDC device is not yet ready, deferring\n");
		return -EPROBE_DEFER;
	}
	if (!panel) {
		dev_info(&pdev->dev, "panel device is not yet ready, deferring\n");
		return -EPROBE_DEFER;
	}

	display_node = of_parse_phandle(ltdc_node, "display", 0);
	timings = of_get_display_timings(display_node);
	for (i = 0; i < timings->num_timings; i++) {
		if (i == timings->native_mode) {
			struct videomode vm;
			if (!videomode_from_timings(timings, &vm, i))
				hactive = vm.hactive;
		}
	}

	if (hactive < 0) {
		dev_err(&pdev->dev, "Failed to get LCD resolution\n");
		return -ENOENT;
	}

	stm32_dsi = devm_kzalloc(&pdev->dev, sizeof(*stm32_dsi) +
			sizeof(u32) * 16, GFP_KERNEL);
	if (!stm32_dsi) {
		dev_err(&pdev->dev, "Failed to initialize framebuffer device\n");
		return -ENOMEM;
	}

	stm32_dsi->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	stm32_dsi->regs = devm_ioremap_resource(stm32_dsi->dev, res);
	err = IS_ERR_OR_NULL(stm32_dsi->regs) ? PTR_ERR(stm32_dsi->regs) : 0;
	if (err) {
		dev_err(stm32_dsi->dev, "Failed to map registers: %d\n", err);
		return err;
	}

	stm32_dsi->clk = devm_clk_get(stm32_dsi->dev, NULL);
	if (IS_ERR_OR_NULL(stm32_dsi->clk)) {
		dev_err(stm32_dsi->dev, "no clock specified in DTS\n");
	}
	clk_prepare_enable(stm32_dsi->clk);

	stm32_dsi->backlight_gpio = devm_gpiod_get_optional(stm32_dsi->dev,
						      "backlight", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(stm32_dsi->backlight_gpio)) {
		dev_info(stm32_dsi->dev, "Operating without backlight gpio: %li\n",
			 PTR_ERR(stm32_dsi->backlight_gpio));
		stm32_dsi->backlight_gpio = NULL;
	}

	stm32_dsi->reset_gpio = devm_gpiod_get_optional(stm32_dsi->dev,
						      "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(stm32_dsi->reset_gpio)) {
		dev_info(stm32_dsi->dev, "Operating without reset gpio: %li\n",
			 PTR_ERR(stm32_dsi->reset_gpio));
		stm32_dsi->reset_gpio = NULL;
	}

	err = of_property_read_u32(pdev->dev.of_node, "lanes", &lanes);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get number of lanes from DTS: %d\n", err);
		return err;
	}
	err = of_property_read_u32(pdev->dev.of_node, "te", &te);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get TE selector from DTS: %d\n", err);
		return err;
	}
	if (!te) {
		err = of_property_read_u32(pdev->dev.of_node, "refresh-period-ms", &refresh_period_ms);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to get refresh period from DTS, required without TE: %d\n", err);
			return err;
		}
		stm32_dsi->refresh_period_ms = refresh_period_ms;
	}

	stm32_dsi->regs->wrpcr = STM32_DSI_WRPCR_REGEN;
	stm32_dsi_gpsr_wait(stm32_dsi, &stm32_dsi->regs->wisr, STM32_DSI_WISR_RRS, true);

	/*
	 * F_VCO = (CLK_IN / IDF) * 2 * NDIV,
	 * 500MHz <= F_VCO <= 1GHz
	 *
	 * PHI = F_VCO / (2 *ODF)
	 * 31.25 <= PHI <= 500MHz
	 *
	 * For STM32F769I CLK_IN = 25MHz
	 * F_VCO = (25MHz / 5) * 2 * 100 = 1GHz
	 * PHI = 1GHz / ( 2 * 1) = 500MHz
	 */
	stm32_dsi->regs->wrpcr |= STM32_DSI_WRPCR_NDIV(100)
		| STM32_DSI_WRPCR_IDF(5)
		| STM32_DSI_WRPCR_ODF_1
		| STM32_DSI_WRPCR_PLLEN;

	stm32_dsi_gpsr_wait(stm32_dsi, &stm32_dsi->regs->wisr, STM32_DSI_WISR_PLLLS, true);

	stm32_dsi->regs->pctlr = STM32_DSI_PCTLR_DEN | STM32_DSI_PCTLR_CKE;
	stm32_dsi->regs->clcr = STM32_DSI_CLCR_DPCC;
	stm32_dsi->regs->pconfr = ((lanes == 2) ? STM32_DSI_PCONFR_NL_TWO_LANES : STM32_DSI_PCONFR_NL_ONE_LANE)
		| STM32_DSI_PCONFR_SW_TIME(10);
	stm32_dsi->regs->ccr = STM32_DSI_CCR_TXECKDIV(4);
	stm32_dsi->regs->wpcr0 = STM32_DSI_WPCR0_UIX4(8);

	stm32_dsi->regs->ier0 = 0;
	stm32_dsi->regs->ier1 = 0;
	stm32_dsi->regs->mcr |= STM32_DSM_MCR_CMDM;

	stm32_dsi->regs->lcolcr = STM32_DSI_LCOLCR_COLC_24BIT;
	stm32_dsi->regs->wcfgr = STM32_DSI_WCFGR_COLMUX_24BIT
		| STM32_DSI_WCFGR_DSIM
		| (te ? STM32_DSI_WCFGR_AR : 0)
		| (te ? STM32_DSI_WCFGR_TESRC : 0);

	stm32_dsi->regs->gvcidr = STM32_DSI_GVCIDR_VCID(0);
	stm32_dsi->regs->lvcidr = 0;

	stm32_dsi->regs->lccr = STM32_DSI_LCCR_CMDSIZE(hactive);

	stm32_dsi->regs->cmcr |= (te ? STM32_DSI_CMCR_TEARE : 0)
		| STM32_DSI_CMCR_GSW0TX
		| STM32_DSI_CMCR_GSW1TX
		| STM32_DSI_CMCR_GSW2TX
		| STM32_DSI_CMCR_GSR0TX
		| STM32_DSI_CMCR_GSR1TX
		| STM32_DSI_CMCR_GSR2TX
		| STM32_DSI_CMCR_GLWTX
		| STM32_DSI_CMCR_DSW0TX
		| STM32_DSI_CMCR_DSW1TX
		| STM32_DSI_CMCR_DSR0TX
		| STM32_DSI_CMCR_DLWTX;

	stm32_dsi->regs->cltcr = 0;
	stm32_dsi->regs->dltcr = 0;

	stm32_dsi_gpsr_wait(stm32_dsi, &stm32_dsi->regs->wisr, STM32_DSI_WISR_BUSY, false);

	stm32_dsi->regs->lpcr = 0;

	if (stm32_dsi->reset_gpio) {
		gpiod_set_value(stm32_dsi->reset_gpio, 0);
		msleep(500);
		gpiod_set_value(stm32_dsi->reset_gpio, 1);
		msleep(500);
		gpiod_set_value(stm32_dsi->reset_gpio, 0);
		msleep(500);
	}

	stm32_dsi->regs->cr |= 1;
	stm32_dsi->regs->wcr |= STM32_DSI_WCR_DSIEN;
	stm32_dsi_gpsr_wait(stm32_dsi, &stm32_dsi->regs->wisr, STM32_DSI_WISR_BUSY, false);

	panel->init(stm32_dsi, stm32_dsi_write_cmd, STM32_DSI_ORIENTATION_LANDSCAPE, !!te);

	stm32_dsi->regs->cmcr &= ~( 0
		| STM32_DSI_CMCR_GSW0TX
		| STM32_DSI_CMCR_GSW1TX
		| STM32_DSI_CMCR_GSW2TX
		| STM32_DSI_CMCR_GSR0TX
		| STM32_DSI_CMCR_GSR1TX
		| STM32_DSI_CMCR_GSR2TX
		| STM32_DSI_CMCR_GLWTX
		| STM32_DSI_CMCR_DSW0TX
		| STM32_DSI_CMCR_DSW1TX
		| STM32_DSI_CMCR_DSR0TX
		| STM32_DSI_CMCR_DLWTX);

	stm32_dsi->regs->wier = 0;

	stm32_dsi->regs->pcr = STM32_DSI_PCR_BTAE;
	stm32_dsi->regs->wcr |= STM32_DSI_WCR_LTDCEN;

	dev_set_drvdata(stm32_dsi->dev, stm32_dsi);

	if (!te)
		kthread_run(stm32_dsi_refresh_task, stm32_dsi, "stm32_dsifb_refresh");

	return 0;
}

static const struct of_device_id stm32_dsi_dt_ids[] = {
	{ .compatible = "st,stm32-dsi", },
	{ /* sentinel */ },
};

static struct platform_driver stm32_dsi_driver = {
	.probe = stm32_dsi_probe,
	.driver = {
		.name = "stm32_dsifb",
		.of_match_table = of_match_ptr(stm32_dsi_dt_ids),
	},
};

module_platform_driver(stm32_dsi_driver);

MODULE_AUTHOR("Sergei Miroshnichenko <sergeimir@emcraft.com>");
MODULE_DESCRIPTION("STM32 DSI");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, stm32_dsi_dt_ids);
