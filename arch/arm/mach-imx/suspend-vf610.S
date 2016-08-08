/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * Added to support STOP mode with DDR self-refresh
 * Copyright (c) 2015
 * Sergei Miroshnichenko, Emcraft Systems, sergeimir@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/linkage.h>
#include <asm/memory.h>

/*
 * DO NOT MODIFY
 *
 */
#define VBASE_OFFSET		4	/* offsetof(struct vf610_pm_base, vbase) */
#define ANATOP_VBASE_OFFSET	16	/* offsetof(struct vf610_cpu_pm_info, anatop_base) */
#define SRC_VBASE_OFFSET	24	/* offsetof(struct vf610_cpu_pm_info, src_base) */
#define CCM_VBASE_OFFSET	32	/* offsetof(struct vf610_cpu_pm_info, ccm_base) */
#define GPC_VBASE_OFFSET	40	/* offsetof(struct vf610_cpu_pm_info, gpc_base) */
#define MSCM_VBASE_OFFSET	48	/* offsetof(struct vf610_cpu_pm_info, mscm_base) */
#define DDRMC_VBASE_OFFSET	56	/* offsetof(struct vf610_cpu_pm_info, ddrmc_base) */
#define IOMUXC_VBASE_OFFSET	64	/* offsetof(struct vf610_cpu_pm_info, iomuxc_base) */

#define IRAM_SUSPEND_SIZE 	(3*1024)

#define DDRMC_CR35		0x8C
#define DDRMC_CR80		0x140
#define DDRMC_CR81		0x144
#define DDRMC_CR124		0x1F0

#define PLL1_OFFSET		0x270
#define PLL2_OFFSET		0x30
#define PLL3_OFFSET		0x10
#define PLL4_OFFSET		0x70
#define PLL5_OFFSET		0xE0
#define PLL6_OFFSET		0xA0
#define PLL7_OFFSET		0x20

/*************************************************************
mvf_suspend:

Suspend the processor (eg, wait for interrupt).

r1: iram_paddr
r2: suspend_iram_base
*************************************************************/
.macro get_base reg offset
	mov	r8, r2	@ suspend_iram_base
	add	r8, #(\offset + VBASE_OFFSET)
	ldr	\reg, [r8]
.endm /* get_base */


.macro store_context
	mrs	r4, spsr		@ Store spsr
	mov	r5, lr			@ Store lr
	push	{r4-r5}

	/* c1 and c2 registers */
	mrc	p15, 0, r4, c1, c0, 2	@ CPACR
	mrc	p15, 0, r5, c2, c0, 0	@ TTBR0
	mrc	p15, 0, r6, c2, c0, 1	@ TTBR1
	mrc	p15, 0, r7, c2, c0, 2	@ TTBCR
	push	{r4-r7}

	/* c3 and c10 registers */
	mrc	p15, 0, r4, c3, c0, 0	@ DACR
	mrc	p15, 0, r5, c10, c2, 0	@ PRRR
	mrc	p15, 0, r6, c10, c2, 1	@ NMRR
	mrc	p15, 0, r7, c1, c0, 1	@ ACTLR
	push	{r4-r7}

	/* c12, c13 and CPSR registers */
	mrc	p15, 0, r4, c13, c0, 1	@ Context ID
	mrc	p15, 0, r5, c13, c0, 2	@ User r/w thread ID
	mrc	p15, 0, r6, c12, c0, 0	@ Secure or NS VBAR
	mrs	r7, cpsr		@ Store CPSR
	push	{r4-r7}

	/* c1 control register */
	mrc	p15, 0, r4, c1, c0, 0	@ SCTLR
	push	{r4}
.endm /* store_context */


.macro restore_context
	/* c1 control register */
	pop	{r4}
	mrc	p15, 0, r4, c1, c0, 0	@ SCTLR

	/* c12, c13 and CPSR registers */
	pop	{r4-r7}
	mrc	p15, 0, r4, c13, c0, 1	@ Context ID
	mrc	p15, 0, r5, c13, c0, 2	@ User r/w thread ID
	mrc	p15, 0, r6, c12, c0, 0	@ Secure or NS VBAR
	msr	cpsr, r7		@ Store CPSR

	/* c3 and c10 registers */
	pop	{r4-r7}
	mcr	p15, 0, r4, c3, c0, 0	@ DACR
	mcr	p15, 0, r5, c10, c2, 0	@ PRRR
	mcr	p15, 0, r6, c10, c2, 1	@ NMRR
	mcr	p15, 0, r7, c1, c0, 1	@ ACTLR

	/* c1 and c2 registers */
	pop	{r4-r7}
	mcr	p15, 0, r4, c1, c0, 2	@ CPACR
	mcr	p15, 0, r5, c2, c0, 0	@ TTBR0
	mcr	p15, 0, r6, c2, c0, 1	@ TTBR1
	mcr	p15, 0, r7, c2, c0, 2	@ TTBCR
	pop	{r4-r5}

	msr	spsr, r4
	mov	lr, r5
.endm /* restore_context */


.macro prime_tlb
	get_base r3 ANATOP_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 CCM_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 GPC_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 MSCM_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 DDRMC_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 IOMUXC_VBASE_OFFSET
	ldr	r4, [r3]
.endm /* prime_tlb */


.macro disable_pll offset
	get_base r3 ANATOP_VBASE_OFFSET
	ldr     r4, [r3, #\offset]
	bic     r4, r4, #0x2000
	orr     r4, r4, #0x1000
	str     r4, [r3, #\offset]
.endm /* disable_pll */

.macro enable_pll offset
	get_base r3 ANATOP_VBASE_OFFSET
	ldr     r4, [r3, #\offset]
	orr     r4, r4, #0x2000
	bic     r4, r4, #0x1000
	str     r4, [r3, #\offset]
.endm /* disable_pll */

.macro ddr_enable_self_refresh
	get_base r3 DDRMC_VBASE_OFFSET

	/* Clear interrupt */
	mov	r4, #(1 << 9)
	str	r4, [r3, #DDRMC_CR81]

	/* Request enter into self-refresh mode */
	mov	r4, #0xEA00
	str	r4, [r3, #DDRMC_CR35]

	/* Wait until it happens */
wait_sr_entry:
	ldr	r4, [r3, #DDRMC_CR35]
	ands	r4, #0x200000
	beq	wait_sr_entry
	nop

	/* Disable DSE in all DDR iomux pins */
	get_base r5 IOMUXC_VBASE_OFFSET
	add	r5, r5, #0x21C
	get_base r6 IOMUXC_VBASE_OFFSET
	add	r6, r6, #0x2D8
wait_disable_dse:
	ldr	r4, [r5]
	bic	r4, r4, #(7 << 6)
	str	r4, [r5], #0x4
	cmp	r5, r6
	bne	wait_disable_dse
	nop

	disable_pll PLL2_OFFSET
.endm /* ddr_enable_self_refresh */


.macro ddr_disable_self_refresh
	enable_pll PLL2_OFFSET

	/* Enable DSE=101 in all DDR iomux pins */
	get_base r5 IOMUXC_VBASE_OFFSET
	add	r5, #0x21C

	get_base r6 IOMUXC_VBASE_OFFSET
	add	r6, #0x2D8
wait_enable_dse:
	ldr	r4, [r5]
	bic	r4, r4, #(7 << 6)
	orr	r4, r4, #(5 << 6)
	str	r4, [r5], #0x4
	cmp	r5, r6
	bne	wait_enable_dse
	nop

	get_base r3 DDRMC_VBASE_OFFSET

	/* Clear interrupt */
	mov	r4, #(1 << 9)
	str	r4, [r3, #DDRMC_CR81]

	/* Request exit from self-refresh mode */
	mov	r4, #0xE900
	str	r4, [r3, #DDRMC_CR35]

	/* Wait until it happens */
wait_sr_exit:
	ldr	r4, [r3, #DDRMC_CR35]
	ands	r4, #0x200000
	beq	wait_sr_exit
	nop
.endm /* ddr_disable_self_refresh */


ENTRY(mvf_suspend)
	stmfd	sp!, {r1-r12}

	mov	r0, r2	@ Get suspend_iram_base
	add	r0, r0, #IRAM_SUSPEND_SIZE

	mov	r4, sp
	stmfd	r0!, {r4}
	mov	sp, r0

	store_context

	prime_tlb

	ddr_enable_self_refresh

	/* enable WEAK 2p5 regulator */
	get_base r3 ANATOP_VBASE_OFFSET
	ldr	r4, [r3, #0x130]
	orr	r4, #(1<<18)
	str	r4, [r3, #0x130]

	/* Set STOP mode */
	get_base r3 GPC_VBASE_OFFSET
	ldr	r4, =0x02
	str	r4, [r3, #0x40]

	/* send MQX a signal to suspend */
	get_base r3 MSCM_VBASE_OFFSET
	ldr	r4, =0x20001
	str	r4, [r3, #0x20]

	/* Meditate */
	wfi
	/* Awaken by IRQ, resume */

	restore_context

	ddr_disable_self_refresh

	mov	r0, r2	@ Get suspend_iram_base
	add	r0, r0, #IRAM_SUSPEND_SIZE

	ldmea	r0!, {r4}
	mov	sp, r4

out:
	ldmfd	sp!, {r1-r12}

	mov	pc, lr
	nop

	.type	mvf_do_suspend, #object
ENTRY(mvf_do_suspend)
	.word	mvf_suspend
	.size	mvf_suspend, . - mvf_suspend
