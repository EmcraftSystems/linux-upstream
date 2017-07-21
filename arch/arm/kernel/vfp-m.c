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
#include <linux/io.h>

#include <asm/thread_notify.h>
#include <asm/v7m.h>
#include <asm/vfp.h>

#undef ENABLE_LAZY

#define FPCCR_ADDR	0xe000ef34	/* Floating-point Context Control Register */
#define FPCCR_ASPEN	(1 << 31)	/* Enable FPU stacking */
#define FPCCR_LSPEN	(1 << 30)	/* Enable lazy FPU stacking */
#define FPCCR_LSPACT	(1 << 0)	/* 1 => Deferred (lazy) FPU stacking is waiting to be saved */

#define FPCAR_ADDR	0xe000ef38	/* Floating-point Context Address Register */

#define CPACR_ADDR	0xe000ed88	/* Coprocessor Access Control Register */
#define CPACR_CP0_FULL	(0x3 << 20)	/* Full access to coprocessor 0 */
#define CPACR_CP1_FULL	(0x3 << 22)	/* Full access to coprocessor 1 */
#define CPACR_VFP_EN	(CPACR_CP0_FULL | CPACR_CP1_FULL)

#define MVFR0_ADDR	0xe000ef40	/* Media and VFP Feature Register 0 */

#define VFP_STACK_SZ	((16 + 2) * 4)	/* {s8-s15, FPSCR (8-byte) */
#define MAIN_STACK_SZ	((8) * 4)	/* {r0-r3, r12 (IP), r14 (LR), r15 (PC), xPSR */

static volatile u32 *cpacr = (u32 *)CPACR_ADDR;
static volatile u32 *mvfr0 = (u32 *)MVFR0_ADDR;
static volatile u32 *fpccr = (u32 *)FPCCR_ADDR;
static volatile u32 **fpcar = (volatile u32 **)FPCAR_ADDR;

static union vfp_state *last_vfp_context;

static void save_vfp_context(void *vfp_regs)
{
	/* vstmia %0!, {d8-d15} - same as s16-s31 - second half of FPU registers */
	asm("	stc	p11, cr8, [%0], #16*4\n" : : "r" (vfp_regs) : "cc");
}

static void load_vfp_context(void *vfp_regs)
{
	/* vldmia %0!, {d8-d15} - same as s16-s31 - second half of FPU registers */
	asm("	ldc	p11, cr8, [%0], #16*4\n" : : "r" (vfp_regs) : "cc");
}

static bool is_vfp_enabled(void)
{
	return (readl(cpacr) & CPACR_VFP_EN) == CPACR_VFP_EN;
}

static void enable_vfp(void)
{
	if (!is_vfp_enabled())
		writel(readl(cpacr) | CPACR_VFP_EN, cpacr);
}

static void disable_vfp(void)
{
	if (is_vfp_enabled())
		writel(readl(cpacr) & ~CPACR_VFP_EN, cpacr);
}

static u32* vfp_append_stack(struct pt_regs *regs)
{
	u32 *orig_sp = (void*)regs->ARM_sp;
	bool need_align = regs->ARM_sp & 4;
	u32 *orig_regs_frame = orig_sp - MAIN_STACK_SZ/4 - (need_align ? 1 : 0);
	u32 *new_regs_frame = orig_regs_frame - VFP_STACK_SZ/4;
	u32 *vfp_regs_frame = new_regs_frame + MAIN_STACK_SZ/4;

	memcpy(new_regs_frame, orig_regs_frame, MAIN_STACK_SZ);

	regs->ARM_sp -= VFP_STACK_SZ;

	return vfp_regs_frame;
}

int v7m_vfp_exception(void)
{
	struct thread_info *thread_from = current_thread_info();
	struct pt_regs *regs_from = task_pt_regs(thread_from->task);
	bool used_fpu = regs_from->ARM_EXC_lr == EXC_RET_THREADMODE_PROCESSSTACK_FPU;
	u32 *allocated_vfp_regs_frame;

	if (used_fpu) {
		pr_err("%s: ERROR: used_fpu\n", __func__);
		return -1;
	}

	if (is_vfp_enabled()) {
		pr_err("%s: ERROR: VFP exception while VFP was enabled\n", __func__);
		return -1;
	}

	enable_vfp();

	if (last_vfp_context) {
		#ifdef ENABLE_LAZY
		if (!(*fpccr & FPCCR_LSPACT)) {
			pr_err("%s: ERROR: no deferred fpu stacking\n", __func__);
		}

		*fpcar = (void*)last_vfp_context->hard.fpcar;
		save_vfp_context(last_vfp_context->hard.fpregs);

		if ((*fpccr & FPCCR_LSPACT)) {
			pr_err("%s: ERROR: still deferred fpu stacking after pushing\n", __func__);
		}
		#else
		save_vfp_context(last_vfp_context->hard.fpregs);
		#endif
	}

	allocated_vfp_regs_frame = vfp_append_stack(regs_from);

	/* Don't leak the current content of VFP registers */
	memset(allocated_vfp_regs_frame, 0, VFP_STACK_SZ);
	load_vfp_context(allocated_vfp_regs_frame);

	regs_from->ARM_EXC_lr = EXC_RET_THREADMODE_PROCESSSTACK_FPU;

	return 0;
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
	bool used_fpu = regs_from->ARM_EXC_lr == EXC_RET_THREADMODE_PROCESSSTACK_FPU;
	bool will_use_fpu = regs_to->ARM_EXC_lr == EXC_RET_THREADMODE_PROCESSSTACK_FPU;
	bool deferred_fpu_stacking = *fpccr & FPCCR_LSPACT;
	bool from_userspace = used_fpu || (regs_from->ARM_EXC_lr == EXC_RET_THREADMODE_PROCESSSTACK);
	bool to_userspace = will_use_fpu || (regs_to->ARM_EXC_lr == EXC_RET_THREADMODE_PROCESSSTACK);

	if (!from_userspace && !to_userspace && !deferred_fpu_stacking) {
		goto exit;
	}

	if (will_use_fpu && !to_userspace) {
		pr_err("%s: ERROR: will_use_fpu && !to_userspace\n", __func__);
		goto exit;
	}

	if (used_fpu && !from_userspace) {
		pr_err("%s: ERROR: used_fpu && !from_userspace\n", __func__);
		goto exit;
	}

	if (used_fpu && !is_vfp_enabled()) {
		pr_err("%s: ERROR: used_fpu && !is_vfp_enabled\n", __func__);
		goto exit;
	}

	#ifdef ENABLE_LAZY
	if (deferred_fpu_stacking && !last_vfp_context) {
		pr_err("%s: ERROR: deferred_fpu_stacking && !last_vfp_context\n", __func__);
		goto exit;
	}
	#else
	if (deferred_fpu_stacking) {
		pr_err("%s: ERROR: !lazy && deferred_fpu_stacking\n", __func__);
		goto exit;
	}
	#endif

	switch (cmd) {
	case THREAD_NOTIFY_FLUSH:
		memset(vfp_to, 0, sizeof(*vfp_to));
		will_use_fpu = 0;
		/* fall through */

	case THREAD_NOTIFY_EXIT:
		if (last_vfp_context == vfp_to) {
			#ifdef ENABLE_LAZY
			/* disable deferred FPU stacking */
			*fpccr &= ~FPCCR_LSPACT;
			#endif
			last_vfp_context = NULL;
		}
		break;

	case THREAD_NOTIFY_SWITCH:
		#ifdef ENABLE_LAZY

		if (from_userspace)
			vfp_from->hard.fpcar = (u32)*fpcar;

		if (used_fpu) {
			vfp_from->hard.fpcar = (u32)*fpcar;
			last_vfp_context = vfp_from;
		}

		if (will_use_fpu && deferred_fpu_stacking && (last_vfp_context != vfp_to)) {
			save_vfp_context(last_vfp_context->hard.fpregs);
			load_vfp_context(vfp_to->hard.fpregs);
			last_vfp_context = vfp_to;
			*fpcar = (void*)vfp_to->hard.fpcar;

			if ((*fpccr & FPCCR_LSPACT)) {
				pr_err("%s: sw: ERROR: still deferred_fpu_stacking\n", __func__);
			}

		}

		#else /* !ENABLE_LAZY */

		if (used_fpu) {
			last_vfp_context = vfp_from;
		}

		if (will_use_fpu && (last_vfp_context != vfp_to)) {
			if (last_vfp_context) {
				enable_vfp();
				save_vfp_context(last_vfp_context->hard.fpregs);
			}
			load_vfp_context(vfp_to->hard.fpregs);
			last_vfp_context = vfp_to;
		}

		#endif /* ENABLE_LAZY */

		if (will_use_fpu) {
			enable_vfp();
		}

		if (!will_use_fpu && to_userspace) {
			disable_vfp();
		}
		break;

	case THREAD_NOTIFY_COPY:
		if (used_fpu) {
			enable_vfp();
			save_vfp_context(vfp_from->hard.fpregs);
			#ifdef ENABLE_LAZY
			vfp_from->hard.fpcar = (u32)*fpcar;
			#endif
			last_vfp_context = vfp_from;

			/* Copy s16-s31 from thread ctx */
			memcpy(vfp_to->hard.fpregs, vfp_from->hard.fpregs, sizeof(vfp_to->hard.fpregs));

			/* Allocate space for VFP registers */
			regs_to->ARM_sp -= VFP_STACK_SZ;

			/* 4-byte pad: an exception frame is always 8-byte aligned */
			if (regs_to->ARM_sp & 4)
				regs_to->ARM_sp -= 4;

			#ifdef ENABLE_LAZY
			vfp_to->hard.fpcar = regs_to->ARM_sp + MAIN_STACK_SZ;
			#else
			/* Copy s0-s15 and FPSCR from thread ctx */
			memcpy((void*)(regs_to->ARM_sp), (void*)(regs_from->ARM_sp + MAIN_STACK_SZ), VFP_STACK_SZ);
			#endif /* ENABLE_LAZY */
		} else {
			memset(vfp_to, 0, sizeof(*vfp_to));
		}
		break;
	}

exit:
	return NOTIFY_DONE;
}

static struct notifier_block vfpm_notifier_block = {
	.notifier_call	= vfpm_notifier,
};

static int __init vfpm_init(void)
{
	/* check for single-precision VFP operations */
	if (((*mvfr0 & MVFR0_SP_MASK) >> MVFR0_SP_BIT) != 0x2)
		return 0;

	pr_info("ARMv7-M VFP Extension supported\n");

	if (((*mvfr0 & MVFR0_DP_MASK) >> MVFR0_DP_BIT) == 0x2)
		pr_info("VFP: Double precision floating points are supported\n");

	/* Enable dynamically on the "No Coprocessor" exception */
	disable_vfp();

	*fpccr |= FPCCR_ASPEN;
	#ifdef ENABLE_LAZY
	*fpccr |= FPCCR_LSPEN;
	#else
	*fpccr &= ~FPCCR_LSPEN;
	#endif

	*fpcar = NULL;

	elf_hwcap |= HWCAP_VFP;
	thread_register_notifier(&vfpm_notifier_block);

	return 0;
}

late_initcall(vfpm_init);
