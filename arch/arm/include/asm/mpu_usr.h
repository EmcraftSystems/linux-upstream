/*
 * arch/arm/include/asm/mpu_usr.h
 *
 * Copyright (C) 2011-2017 Emcraft Systems
 * Vladimir Khusainov, vlad@emcraft.com
 * Yuri Tikhonov, yur@emcraft.com
 *
 * License terms: GNU General Public License (GPL), version 2
 */
#ifndef __ASM_MPU_USR_H_
#define __ASM_MPU_USR_H_

#ifdef CONFIG_MPU

#include <asm/exception.h>

/*
 * Define a mappable region
 */
void mpu_region_define(u32 b, u32 t);

/*
 * Prepare to allocate "the MPU page tables". Enable the hardware MPU.
 */
void __init mpu_setup(void);

/*
 * These two functions were defined in the generic kernel code by the Blackfin
 * kernel maintainers (MPU support has been around in Blackfin for a while). I
 * preserve this interface so that I don't have to modify the architecture-
 * neutral code.
 *
 * Protect a specified page with specified permissions.
 */
void protect_page(struct mm_struct *mm, unsigned long adr, unsigned long flgs);

/*
 * Update the MPU after permissions have changed for a mm area.
 */
void update_protections(struct mm_struct *mm);

/*
 * Allocate and free a per-process "MPU page table".
 * These are called from asm/mmu_context.h.
 */
int mpu_init_new_context(struct task_struct *tsk, struct mm_struct *mm);
void mpu_destroy_context(struct mm_struct *mm);

/*
 * MPU-specific part of the context switch
 */
void mpu_switch_mm(struct mm_struct *prev, struct mm_struct *next);

/*
 * MPU-specific part of start_thread
 */
void mpu_start_thread(struct pt_regs *regs);

/*
 * Memory Fault (memmanage) exception handler. This is called from the
 * architecture-specific low-level exception handler. It is assumed that
 * this code doesn't get pre-empted.
 */
asmlinkage void __exception do_memmanage(struct pt_regs *regs);

#endif /* CONFIG_MPU */
#endif /* __ASM_MPU_USR_H_ */
