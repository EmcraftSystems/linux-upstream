#ifndef STM32_DSI_H
#define STM32_DSI_H

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

#define STM32_DSI_ORIENTATION_PORTRAIT	0
#define STM32_DSI_ORIENTATION_LANDSCAPE	1

struct stm32_dsi_panel {
	int (*init)(void *data, void (*writecmd)(void *data, int NbrParams, u8 *pParams), int orientation, bool te);
};


#endif /* STM32_DSI_H */
