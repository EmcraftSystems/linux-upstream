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

#define IRAM_SUSPEND_SIZE 	(3*1024)

#define DDRMC_CR35		0x8C
#define DDRMC_CR80		0x140
#define DDRMC_CR81		0x144
#define DDRMC_CR124		0x1F0

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
	get_base r3 CCM_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 GPC_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 DDRMC_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 ANATOP_VBASE_OFFSET
	ldr	r4, [r3]

	get_base r3 MSCM_VBASE_OFFSET
.endm /* prime_tlb */


.macro disable_pll offset
	get_base r3 ANATOP_VBASE_OFFSET
	ldr	r4, [r3, #\offset]
	bic	r4, r4, #0x2000
	str	r4, [r3, #\offset]
.endm /* disable_pll */

.macro enable_pll offset
	get_base r3 ANATOP_VBASE_OFFSET
	ldr	r4, [r3, #\offset]
	orr	r4, r4, #0x2000
	str	r4, [r3, #\offset]
.endm /* enable_pll */


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
.endm /* ddr_enable_self_refresh */


.macro ddr_disable_self_refresh
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


.macro	mvf_stop_mode_enter
	prime_tlb

	ddr_enable_self_refresh

	/* Switch to FIRC */
	get_base r3 CCM_VBASE_OFFSET
	ldr	r4, [r3, #0x08]
	bic	r4, r4, #0x7
	str	r4, [r3, #0x08]

	disable_pll 0x20	@ pll7
	disable_pll 0x10	@ pll3
	disable_pll 0x20	@ pll2
	disable_pll 0x70	@ pll4
	disable_pll 0xa0	@ pll6
	disable_pll 0xe0	@ pll5
	disable_pll 0x270	@ pll1

	/* enable WEAK 2p5 regulator */
	get_base r3 ANATOP_VBASE_OFFSET
	ldr	r4, [r3, #0x130]
	orr	r4, #(1<<18)
	str	r4, [r3, #0x130]

	/* stop mode is sent/masked to Anatop */
	get_base r3 CCM_VBASE_OFFSET
	ldr	r4, [r3, #0x2c]
	orr	r4, r4, #0x100
	str	r4, [r3, #0x2c]

	/* Set STOP mode */
	get_base r3 GPC_VBASE_OFFSET
	ldr	r4, =0x02
	str	r4, [r3, #0x40]

	/* ensure power domain 0 */
	ldr	r4, [r3, #0x0]
	bic	r4, r4, #0x07

	/* enable deep sleep for memories */
	orr	r4, r4, #0x80
	orr	r4, r4, #0x40

	/* disable well bias */
	bic	r4, r4, #0x10

	/* turn off HPREG in stop mode */
	orr	r4, r4, #0x08

	str	r4, [r3, #0x0]
.endm /* mvf_stop_mode_enter */

/******************************************************************
Invalidate l1 dcache, r0, r3-r8
******************************************************************/
.macro invalidate_l1_dcache
	mov 	r0, #0
	mov 	r3, #0
	mov 	r4, #0
	mov 	r5, #0
	mov 	r6, #0
	mov 	r7, #0
	mov 	r8, #0

	mcr 	p15, 2, r0, c0, c0, 0
	mrc 	p15, 1, r0, c0, c0, 0

	ldr 	r3, =0x7fff
	and 	r4, r3, r0, lsr #13

	ldr 	r3, =0x3ff

	and 	r5, r3, r0, lsr #3	@ NumWays - 1
	add 	r4, r4, #1		@ NumSets

	and 	r0, r0, #0x7
	add 	r0, r0, #4		@ SetShift

	clz 	r3, r5			@ WayShift
	add 	r6, r5, #1		@ NumWays
1:
	sub 	r4, r4, #1		@ NumSets--
	mov 	r5, r6			@ Temp = NumWays
2:
	subs	r5, r5, #1		@ Temp--
	mov 	r8, r5, lsl r3
	mov 	r7, r4, lsl r0
	orr 	r8, r8, r7
	mcr 	p15, 0, r8, c7, c6, 2
	bgt 	2b
	nop
	cmp 	r4, #0
	bgt 	1b
	nop
	dsb
	isb
.endm /* invalidate_l1_dcache */


/******************************************************************
Flush and disable L1 dcache
******************************************************************/
.macro flush_disable_l1_dcache
	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	push	{r0-r12, lr}
	ldr r0, =v7_flush_dcache_all
	mov lr, pc
	mov pc, r0
	pop {r0-r12, lr}

	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data accesses
	 * strongly ordered and would not hit the cache.
	 */
	mrc p15, 0, r0, c1, c0, 0
	bic r0, r0, #(1 << 2)		@ Disable the C bit
	mcr p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Invalidate L1 data cache. Even though only invalidate is
	 * necessary exported flush API is used here. Doing clean
	 * on already clean cache would be almost NOP.
	 */
	push	{r0-r12, lr}
	ldr r0, =v7_flush_dcache_all
	mov lr, pc
	mov pc, r0
	pop {r0-r12, lr}

	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * by any CPU in the cluster have completed.
	 */
	dsb
	dmb
.endm /* flush_disable_l1_dcache */


.macro inv_enable_l1_dcache
	/* Invalidate L1 I-cache first */
	mov r0, #0x0
	mcr p15, 0, r0, c7, c5, 0 @ Invalidate I-Cache

	isb
	dsb
	dmb

	invalidate_l1_dcache

	mrc	p15, 0, r1, c1, c0, 0
	orr	r1, r1, #(1 << 2)	@ Enable the C bit
	mcr	p15, 0, r1, c1, c0, 0

	isb
	dsb
	dmb
.endm /* inv_enable_l1_dcache */


ENTRY(mvf_suspend)
	stmfd	sp!, {r0-r12}     @ Save registers

	b	dormant
	nop

	/* Place the literal pool here so that literals are
	within 16KB range */
	.ltorg

dormant:
	mov	r0, r2			@ Get suspend_iram_base
	add	r0, r0, #IRAM_SUSPEND_SIZE

	mov	r4, sp			@ Store stack pointer
	stmfd	r0!, {r4}
	mov	sp, r0			@ Set new stack

	store_context
	flush_disable_l1_dcache

	mvf_stop_mode_enter

	get_base r3, MSCM_VBASE_OFFSET

	/* Force CM4 to meditate by sending interrupt
	 * that is processed with 'wfi'.
	 * Route CPU-to-CPU IRQ#1 to CM4
	 */
	mov	r4, #0x2
	strh	r4, [r3, #0x82]

	ldr	r4, =0x00020001
	str	r4, [r3, #0x20]

	/* Meditate */
	wfi
	/* Awaken by IRQ, resume */

	restore_context

	/* mask all the GPC interrupts */
	get_base r3 GPC_VBASE_OFFSET
	ldr	r4, =0xffffffff
	str	r4, [r3, #0x44]
	str	r4, [r3, #0x48]
	str	r4, [r3, #0x4c]
	str	r4, [r3, #0x50]

	get_base r3 ANATOP_VBASE_OFFSET

	enable_pll 0x270	@ pll1
wait_pll1:
	ldr	r4, [r3, #0x270]
	ands	r4, r4, #(1<<31)
	beq	wait_pll1
	nop

	enable_pll 0x30		@ pll2
wait_pll2:
	ldr	r4, [r3, #0x30]
	ands	r4, r4, #(1<<31)
	beq	wait_pll2
	nop

	/* switch system clock source to pll1 */
	get_base r3 CCM_VBASE_OFFSET
	ldr	r4, [r3, #0x08]
	bic	r4, #0x77
	orr	r4, #0x24
	str	r4, [r3, #0x08]

	ddr_disable_self_refresh

	mov	r0, r2		@ Get suspend_iram_base
	add	r0, r0, #IRAM_SUSPEND_SIZE

	ldmea	r0!, {r4}
	mov	sp, r4		@ Restore stack pointer

	inv_enable_l1_dcache

/************************************************
return back to mvf_suspend_enter for suspend
*************************************************/
out:
	ldmfd	sp!, {r0-r12}

	mov	pc, lr
	nop

.global ipg_new_clock, ipg_old_clock
ipg_new_clock:
	.word	4
ipg_old_clock:
	.word	83


	.type	mvf_do_suspend, #object
ENTRY(mvf_do_suspend)
	.word	mvf_suspend
	.size	mvf_suspend, . - mvf_suspend
