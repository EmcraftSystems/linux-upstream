/*
 * Copyright (C) 2010 ARM Ltd.
 * Catalin Marinas <catalin.marinas@arm.com>
 *
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sched.h>

#include <asm/thread_notify.h>
#include <asm/v7m.h>
#include <asm/vfp.h>

#define FPCCR_ADDR	0xe000ef34	/* Floating-point Context Control Register */
#define FPCCR_ASPEN	(1 << 31)	/* Enable FPU stacking */
#define FPCCR_LSPEN	(1 << 30)	/* Enable lazy FPU stacking */
#define FPCCR_LSPACT	(1 << 0)	/* 1 => Deferred (lazy) FPU stacking is waiting to be saved */

#define FPCAR_ADDR	0xe000ef38	/* Floating-point Context Address Register */

#define CPACR_ADDR	0xe000ed88	/* Coprocessor Access Control Register */
#define CPACR_CP0_FULL	(0x3 << 20)	/* Full access to coprocessor 0 */
#define CPACR_CP1_FULL	(0x3 << 22)	/* Full access to coprocessor 1 */

#define MVFR0_ADDR	0xe000ef40	/* Media and VFP Feature Register 0 */

static union vfp_state *last_vfp_context;

static void save_vfp_context(union vfp_state *vfp)
{
	asm("	stc	p11, cr8, [%0], #16*4\n" : : "r" (vfp) : "cc");
}

static void load_vfp_context(union vfp_state *vfp)
{
	asm("	ldc	p11, cr8, [%0], #16*4\n" : : "r" (vfp) : "cc");
}

static int vfpm_notifier(struct notifier_block *self, unsigned long cmd,
			 void *t)
{
	struct thread_info *thread_from = current_thread_info();
	struct thread_info *thread_to = t;
	union vfp_state *vfp_from = &thread_from->vfpstate;
	union vfp_state *vfp_to = &thread_to->vfpstate;
	struct pt_regs *regs_from = task_pt_regs(thread_from->task);
	struct pt_regs *regs_to = task_pt_regs(thread_to->task);
	bool used_fpu = !!(regs_from->ARM_EXC_lr == EXC_RET_THREADMODE_PROCESSSTACK_FPU);
	bool will_use_fpu = !!(regs_to->ARM_EXC_lr == EXC_RET_THREADMODE_PROCESSSTACK_FPU);

	u32 *fpccr = (u32 *)FPCCR_ADDR;
	u32 *fpcar = (u32 *)FPCAR_ADDR;

	bool deferred_fpu_stacking = *fpccr & FPCCR_LSPACT;

	switch (cmd) {
	case THREAD_NOTIFY_FLUSH:
		memset(vfp_to, 0, sizeof(*vfp_to));
		will_use_fpu = 0;
		/* fall through */

	case THREAD_NOTIFY_EXIT:
		if (last_vfp_context == vfp_to) {
			/* disable deferred FPU stacking */
			*fpccr &= ~FPCCR_LSPACT;
			last_vfp_context = NULL;
		}
		break;

	case THREAD_NOTIFY_SWITCH:
		if (deferred_fpu_stacking) {
			save_vfp_context(vfp_from);
			vfp_from->hard.fpcar = *fpcar;
			last_vfp_context = vfp_from;
		}
		if (will_use_fpu && last_vfp_context != vfp_to) {
			load_vfp_context(vfp_to);
			*fpcar = vfp_to->hard.fpcar;
			last_vfp_context = vfp_to;
		}
		break;

	case THREAD_NOTIFY_COPY:
		if (used_fpu) {
			/* {s8-s15, FPSCR (8-byte) */
			int fp_stack_size = (16 + 2) * 4;
			/* {r0-r3, r12 (IP), r14 (LR), r15 (PC), xPSR */
			int main_registers_stack_size = (8) * 4;
			bool need_align = !!(regs_to->ARM_sp & 4);

			save_vfp_context(vfp_from);
			vfp_from->hard.fpcar = *fpcar;
			last_vfp_context = vfp_from;

			/* Copy s8-s15 and FPSCR from thread ctx */
			memcpy(vfp_to, vfp_from, sizeof(*vfp_to));

			/* Copy s0-s7 from stack */
			regs_to->ARM_sp -= fp_stack_size;

			/* 4-byte pad: an exception frame is always 8-byte aligned */
			if (need_align)
				regs_to->ARM_sp -= 4;

			vfp_to->hard.fpcar = regs_to->ARM_sp + main_registers_stack_size;

		} else {
			memset(vfp_to, 0, sizeof(*vfp_to));
		}
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block vfpm_notifier_block = {
	.notifier_call	= vfpm_notifier,
};

static int __init vfpm_init(void)
{
	u32 *cpacr = (u32 *)CPACR_ADDR;
	u32 *mvfr0 = (u32 *)MVFR0_ADDR;
	u32 *fpccr = (u32 *)FPCCR_ADDR;

	/* check for single-precision VFP operations */
	if (((*mvfr0 & MVFR0_SP_MASK) >> MVFR0_SP_BIT) != 0x2)
		return 0;

	printk(KERN_INFO "ARMv7-M VFP Extension supported\n");

	if (((*mvfr0 & MVFR0_DP_MASK) >> MVFR0_DP_BIT) == 0x2)
		printk(KERN_INFO "VFP: Double precision floating points are supported\n");

	*cpacr |= CPACR_CP0_FULL | CPACR_CP1_FULL;
	*fpccr |= FPCCR_ASPEN | FPCCR_LSPEN;

	elf_hwcap |= HWCAP_VFP;
	thread_register_notifier(&vfpm_notifier_block);

	return 0;
}

late_initcall(vfpm_init);
