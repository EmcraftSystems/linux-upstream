/*
 * arch/arm/mm/mpu_usr.c
 *
 * Copyright (C) 2011-2017 Emcraft Systems
 * Vladimir Khusainov, vlad@emcraft.com
 * Yuri Tikhonov, yur@emcraft.com
 *
 * This module provides MPU-based process protection for ARMv7.
 * Both process-to-kernel and process-to-process protection is supported.
 * An optional "stack redzone" feature is provided.
 *
 * This code has been tested on the Cortex-M3 core of:
 * - STmicro's STMF32F7
 * Minimally, it should work as is on any Cortex-M3/M4 core.
 *
 * Some changes to the MPU h/w handling code may be needed (i.e. I
 * don't know if changes are needed or not; I haven't studied that), in case
 * some ARMv7 processor has an MPU that is different from that of
 * Cortex-M3 (which is 8 regions; no separation of instruction data;
 * pages of various size; protection must be size-aligned).
 *
 * License terms: GNU General Public License (GPL), version 2
 */

#include <linux/memblock.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/io.h>

#include <asm/mpu_usr.h>
#include <asm/system_misc.h>

/******************************************************************************
 * Constants local to this module
 ******************************************************************************/
/*
 * #define debug print-outs:
 * DEBUG_MPU to enable additional print-outs on tasks creation
 * DEBUG_PROC_NAME "process" to print-out any mapping for the process specified
 */
#undef DEBUG_MPU
#undef DEBUG_PROC_NAME

/*
 * Number of MPU regions and subregions
 */
#define MPU_REG_NUM			8
#define MPU_SUB_NUM			8
#define MPU_SUB_MSK			((1 << MPU_SUB_NUM) - 1)

/*
 * Non-standard pages used by this module
 */
#define PAGE32K_SIZE			(32 * 1024)
#define PAGE32K_MASK			(~(PAGE32K_SIZE - 1))
#define PAGE256B_SIZE			256
#define PAGE256B_MASK			(~(PAGE256B_SIZE - 1))

/*
 * Different reg fields
 */
#define SHCSR_MEMFAULTENA		(1 << 16)

#define MMFSR_MMARVALID			(1 << 7)
#define MMFSR_MLSPERR			(1 << 5)
#define MMFSR_MSTKERR			(1 << 4)
#define MMFSR_MUNSTKERR			(1 << 3)
#define MMFSR_DACCVIOL			(1 << 1)
#define MMFSR_IACCVIOL			(1 << 0)

#define MPU_CTRL_ENABLE			(1 << 0)
#define MPU_CTRL_PRIVDEFENA		(1 << 2)

#define MPU_RBAR_VALID			(1 << 4)
#define MPU_RBAR_ADDR_MASK		(~0x1F)

#define MPU_RASR_XN(x)			((x) << 28)
#define MPU_RASR_AP(x)			((x) << 24)
#define MPU_RASR_AP_PRVL_RW		MPU_RASR_AP(1)
#define MPU_RASR_AP_MASK		MPU_RASR_AP(7)
#define MPU_RASR_TEX_SCB_MASK		(0x3F << 16)
#define MPU_RASR_SRD(x)			((x) << 8)
#define MPU_RASR_SIZE(x)		((x) << 1)
#define MPU_RASR_ENABLE			(1 << 0)

/*
 * Register base
 */
#define NVIC_REGS_BASE			0xE000E004

/*
 * Maximum number of mappable regions
 */
#define MPU_ADDR_REGION_TBLSZ		16

/*
 * Minimum red-zone size
 */
#define MPU_REDZONE_MIN			256

/******************************************************************************
 * C-types local to this module
 ******************************************************************************/
/*
 * Description of the MPU programming interfaces. Specifically, we describe
 * that part of NVIC that concerns the MPU (Memory Fault) configuration and
 * processing as well as exception handlers configuration.
 */
struct nvic_regs {
	u32	some_regs1[840];	/* e000e004 */
	u32	system_handler_csr;	/* e000ed24 */
	u32	fault_status;		/* e000ed28 */
	u32	some_regs2[2];
	u32	mfar;			/* e000ed34 */
	u32	some_regs3[22];
	u32	mpu_type;		/* e000ed90 */
	u32	mpu_control;
	u32	reg_number;
	u32	reg_base;
	u32	reg_attr;
};
#define NVIC			((struct nvic_regs *)(NVIC_REGS_BASE))

/*
 * Address region data structure. This represents a "mappable region".
 * There will always be a region for the main RAM but other regions
 * are possible as well, in case a configuration desires to allow mappings
 * into Flash, internal RAM, control registers, etc.
 */
struct mpu_addr_region {
	u32			bot;		/* Region base */
	u32			top;		/* Region top */
};

/*
 * Per-process MPU context data structure. It is referenced, as an opaque
 * pointer, from mm->context (in case CONFIG_MPU is on).
 */
struct mpu_context {
	struct {
		u32		base;		/* Region base */
		u32		attr;		/* Region attr */
	} mpu_regs[MPU_REG_NUM];		/* MPU regions copy */
	u32			indx;		/* Next reg to use */
	u32			barrier;	/* Locked regs bar */
#ifdef CONFIG_MPU_STACK_REDZONE
	u32			redzone_bot;	/* Stack redzone btm */
	u32			redzone_top;	/* Stack redzone top */
#endif
	u32			page_tbl[0];	/* Page tables pool */
};
#define mpu_context_p(mm)	((struct mpu_context *)(mm)->context.p)

/******************************************************************************
 * Prototypes and variables local to this module
 ******************************************************************************/

/*
 * For some Cortex-M processors (STM32F7 being an example), the MPU
 * will be already enabled and first MPU protection regions will be
 * taken when mpu_hw_init() is called. This is needed to allow
 * accesses to addresses at and above 0xA0000000, where external
 * RAM resides for such processors, create non-cacheable (DMA) area, etc.
 * Thus unless a bootloader (such as U-Boot) performs the above-described MPU
 * enabling and initialization, the regions above 0xA0000000 will have an
 * attribute XN (eXecute Never) and Cortex-M3 will not ever be able to run code
 * from external RAM, on such processors.
 *
 * To account for such situations, we define a module-wide variable that will be
 * an index of the first MPU protection region available for use in this module.
 */
static int mpu_hw_reg_indx;

/*
 * Array of address regions
 */
static struct mpu_addr_region mpu_addr_region_tbl[MPU_ADDR_REGION_TBLSZ];

/*
 * Next entry to fill in in the array of address regions
 */
static int mpu_addr_region_next;

/*
 * These variables below are visible module-wide and provide geometry
 * of the "page tables" for the defined MPU address regions.
 * Each such page table is a bit-wise mask for the pages of the corresponing
 * MPU address region. The bit tells if a page is mapped (1) or unmapped (0).
 * We do not distinguish between READ / WRITE / EXEC mappings.
 * This is because page mappings are implemented using 32K MPU protection
 * regions, with the 8 sub-regions of a region used to define mappings
 * for individul pages (an alternative would be to map using 4K MPU
 * protection regions with more frequent page misses and constant swap-outs).
 * With this decision though, there is a single mapping for a 32K
 * region (for each there is just one specific set of the permissions
 * defined - no separate permissions for the sub-regions). A 32K region
 * in general case spans more than one "virtual memory" mappings with
 * different permissions (i.e. text and data). Hence unified permissions
 * (always all of READ and WRITE and EXEC) for anything that gets mapped.
 * This is a bit a violation for the POSIX interfaces (e.g. a region
 * that is mmapp()-ed READ-ONLY will be possible to WRITE), however given
 * the accent on making sure that a process can't trash memory of
 * the kernel or other processes, is still an acceptable trade-off, since
 * the above goal is met.
 *
 * The variables below are:
 * - size of the entire pool for the page tables. Each region draws memory
 *   for its page table from that pool;
 * - number of full pages required for the pool;
 * - array of offsets to the base of a page table for each region.
 * We calculate this geometry just once so as not to repeat these
 * calcualtions any time the MPU context is created for a new process.
 */
static u32 mpu_page_tbl_len;
static u32 mpu_page_tbl_order;
static u32 mpu_page_tbl_off[MPU_ADDR_REGION_TBLSZ];

/******************************************************************************
 * Functions local to this module
 ******************************************************************************/

/*
 * Set up an MPU protection region
 */
static inline void mpu_region_write(int i, u32 b, u32 a)
{
	writel((b & MPU_RBAR_ADDR_MASK) | MPU_RBAR_VALID | i, &NVIC->reg_base);
	writel(a, &NVIC->reg_attr);
}

/*
 * Read an MPU protection region
 */
static inline void mpu_region_read(int i, u32 *b, u32 *a)
{
	writel(i, &NVIC->reg_number);
	*b = readl(&NVIC->reg_base);
	*a = readl(&NVIC->reg_attr);
}

/*
 * Set up the MPU and related hardware interfaces.
 */
static void mpu_hw_on(void)
{
	u32 b, a, i, v;

	/*
	 * Check if the MPU already on. This will be the case for those
	 * processors that have external RAM at 0xA0000000 or above. For such
	 * processors, the MPU will have a single region defined, enabling
	 * R|W|X accesses to the entire 4GB address space. We must always
	 * preserve this mapping!
	 *
	 * There is yet another complication here. Not only we must preserve
	 * that mapping we also must make sure that it is valid only for
	 * privildged accesses and is disabled for user-space applications.
	 * So, skip all regions set by the loader.
	 */
	if (readl(&NVIC->mpu_control) & MPU_CTRL_ENABLE) {
		/*
		 * Make sure the mapping is valid only for the kernel
		 */
		for (i = 0; i < MPU_REG_NUM; i++) {
			mpu_region_read(i, &b, &a);
			if (!(a & MPU_RASR_ENABLE))
				continue;
			a &= ~MPU_RASR_AP_MASK;
			a |= MPU_RASR_AP_PRVL_RW;
			mpu_region_write(i, b, a);
		}

		/*
		 * If the default region is off, then enable it, and disable the
		 * 4G-region (assume it's first)
		 */
		v = readl(&NVIC->mpu_control);
		if (!(v & MPU_CTRL_PRIVDEFENA)) {
			writel(v | MPU_CTRL_PRIVDEFENA, &NVIC->mpu_control);
			mpu_region_write(0, 0, 0);
		}

		/*
		 * Remove all regions except DRAM
		 */
		for (i = 0; i < MPU_REG_NUM; i++) {
			mpu_region_read(i, &b, &a);
			if (!(a & MPU_RASR_ENABLE))
				continue;
			b &= MPU_RBAR_ADDR_MASK;
			if (b >= CONFIG_DRAM_BASE &&
			    b < (CONFIG_DRAM_BASE + CONFIG_DRAM_SIZE))
				continue;
			if (b >= CONFIG_FLASH_MEM_BASE &&
			    b < (CONFIG_FLASH_MEM_BASE + CONFIG_FLASH_SIZE))
				pr_warn("Programming on-chip flash may work "
					"incorrectly when MPU enabled\n");
			mpu_region_write(i, 0, 0);
		}

		/*
		 * Shift all remain regions to #0
		 */
		for (i = 0, v = 0; i < MPU_REG_NUM; i++) {
			mpu_region_read(i, &b, &a);
			if (!(a & MPU_RASR_ENABLE))
				continue;
			mpu_region_write(v, b, a);
			mpu_region_write(i, 0, 0);
			v++;
		}

		/*
		 * Make sure this mapping is never affected
		 */
		mpu_hw_reg_indx = v;
	} else {
		/*
		 * Turn the MPU on, enable the default map for
		 * privilidged accesses
		 */
		writel(MPU_CTRL_PRIVDEFENA | MPU_CTRL_ENABLE,
			&NVIC->mpu_control);
	}

	/*
	 * As long as software does not attempt to expressly change
	 * the priority of MemManage, it remains at the highest priority
	 * and no other exception handlers will preempt the MemManage
	 * exception handler (with exception of Reset, NMI and HardFault).
	 *
	 * Enable Memory Fault (MemManage) system handler
	 */
	writel(readl(&NVIC->system_handler_csr) | SHCSR_MEMFAULTENA,
		&NVIC->system_handler_csr);
}

/*
 * Define an MPU address region.
 */
static void mpu_addr_region_define(u32 b, u32 t)
{
	struct mpu_addr_region *r;

	/*
	 * Check that there is no over-allocation. If there is,
	 * just ignore that new region.
	 */
	if (mpu_addr_region_next == MPU_ADDR_REGION_TBLSZ) {
		printk("%s: unable to define MPU address region: [%08x,%08x]\n",
		       __func__, b, t);
		goto done;
	}

	/*
	 * Store the regions settings.
	 */
	r = &mpu_addr_region_tbl[mpu_addr_region_next++];
	r->bot = b;
	r->top = t;

done:
	return;
}

/*
 * Calculate geometry for all MPU address regions that have been defined
 */
static void mpu_addr_region_geometry(void)
{
	struct mpu_addr_region *r;
	u32 s;
	int i;

	for (i = 0, s = 0; i < mpu_addr_region_next; i++) {
		r = &mpu_addr_region_tbl[i];
		mpu_page_tbl_off[i] = s;
		s += sizeof(u32) *
		     ((((r->top - r->bot) >> PAGE_SHIFT) + 31) / 32);
#if defined(DEBUG_MPU)
		printk("%s: %d: b=0x%x,t=0x%x,sz=%dK,s=%d\n", __func__,
			i, r->bot, r->top,
			(r->top - r->bot) / 1024, s);
#endif
	}

	/*
	 * Now, calculate the size of the entire page tables pool
	 * The pool is allocated as a number of 4K pages.
	 */
	mpu_page_tbl_len = sizeof(struct mpu_context) + s;
	mpu_page_tbl_order = max(1, get_order(mpu_page_tbl_len));
#if defined(DEBUG_MPU)
	printk("%s: tbl=%d,ord=%d\n", __func__,
		mpu_page_tbl_len, mpu_page_tbl_order);
#endif

}

/*
 * Find an MPU address region an address belongs to
 */
static inline struct mpu_addr_region *mpu_addr_region_find(u32 a)
{
	struct mpu_addr_region *r;
	int i, f;

	for (i = 0, f = 0, r = 0; i < mpu_addr_region_next && !f; i++) {
		r = &mpu_addr_region_tbl[i];
		f = (r->bot <= a) && (a < r->top);
	}

	return f ? r : NULL;
}

/*
 * Debug service: print active MPU address regions
 */
#if defined(DEBUG_MPU)
static void mpu_addr_region_tbl_print(void)
{
	struct mpu_addr_region *r;
	int i;

	for (i = 0; i < mpu_addr_region_next; i++) {
		r = &mpu_addr_region_tbl[i];
		printk("%s: %d: %x,%x\n", __func__, i, r->bot, r->top);
	}
}
#endif

/*
 * Initialize the MPU context for a new process
 */
static inline void mpu_context_init(struct mm_struct *mm)
{
	struct mpu_context *p;
	u32 r;

	/*
	 * If this is called for a first time (i.e. first user process),
	 * calculate the page table geometry
	 */
	if (mpu_page_tbl_len == 0)
		mpu_addr_region_geometry();

	/*
	 * Allocate an MPU context structure for the process and
	 * zero it out.
	 */
	r = __get_free_pages(GFP_KERNEL, mpu_page_tbl_order);
	if (! r) {
		printk("%s: no free_pages; stopping the kernel\n", __func__);
		while (1);
	}
	memset((void *)r, 0, mpu_page_tbl_len);

	/*
	 * Store a pointer to the MPU context in mm->context.
	 */
	mm->context.p = (void *)r;

	/*
	 * Get access to the MPU context with an appropriate data pointer
	 */
	p = mpu_context_p(mm);

	/*
	 * Set up the next region to use
	 */
	p->indx = mpu_hw_reg_indx;
}

/*
 * Free the MPU context for a process
 */
static inline void mpu_context_free(struct mm_struct *mm)
{
	free_pages((u32) mm->context.p, mpu_page_tbl_order);
}

/*
 * Set specified permissions for a page in the appropriate page table
 */
static inline void mpu_context_map_page(struct mm_struct *mm, u32 a, u32 f)
{
	struct mpu_addr_region *r;
	struct mpu_context *p;
	u32 page, idx, bit;
	u32 *mask;
	int i;

	/*
	 * Check if there is even an MPU address region for this address
	 */
	r = mpu_addr_region_find(a);
	if (!r)
		goto done;

	/*
	 * Get access to: context; region index; page table
	 */
	p = mpu_context_p(mm);
	i = r - mpu_addr_region_tbl;
	mask = p->page_tbl + mpu_page_tbl_off[i];

	/*
	 * Calculate: relative page; page index; bit
	 */
	page = (a - r->bot) >> PAGE_SHIFT;
	idx = page >> 5;
	bit = 1 << (page & 31);

	/*
	 * Change mappings for the page.
	 * Note that we do not distinguish between RD/WR/EXEC,
	 * any permission results in a valid mapping.
	 */
	if (f & (VM_READ | VM_WRITE | VM_EXEC))
		mask[idx] |= bit;
	else
		mask[idx] &= ~bit;

done:
	return;
}

/*
 * Is a specified page validly mapped within a specified address region?
 */
static inline int mpu_context_addr_valid_for_region(struct mm_struct *mm,
	struct mpu_addr_region *r, u32 a, u32 f)
{
	struct mpu_context *p = mpu_context_p(mm);
	u32 page, idx, bit;
	int i;
	u32 *mask;

#ifdef CONFIG_MPU_STACK_REDZONE
	/*
	 * If page overlaps redzone, then permission denied
	 */
	page = a & PAGE_MASK;
	if (page < p->redzone_top && (page + PAGE_SIZE) > p->redzone_bot)
		return 0;
#endif

	/*
	 * Get access to: region index; page table
	 */
	i = r - mpu_addr_region_tbl;
	mask = p->page_tbl + mpu_page_tbl_off[i];

	/*
	 * Calculate: relative page; page index; bit
	 */
	page = (a - r->bot) >> PAGE_SHIFT;
	idx = page >> 5;
	bit = 1 << (page & 31);

	/*
	 * Get mappings for the page.
	 * Note that we do not distinguish between RD/WR/EXEC,
	 * any permission results in a valid mapping.
	 */
	return (mask[idx] & bit) ? 1 : 0;
}

/*
 * Is a specified address validly mapped for the context?
 */
static inline int mpu_context_addr_valid(struct mm_struct *mm, u32 a, u32 f)
{
#ifdef CONFIG_MPU_STACK_REDZONE
	struct mpu_context *p = mpu_context_p(mm);
#endif
	struct mpu_addr_region *r;
	int b = 0;

#ifdef CONFIG_MPU_STACK_REDZONE
	/*
	 * Check if the address is not in the stack redzone
	 */
	if (a < p->redzone_top && a >= p->redzone_bot)
		goto done;
#endif

	/*
	 * Check if there is even an MPU address region for this address
	 */
	r = mpu_addr_region_find(a);
	if (!r)
		goto done;

	b = mpu_context_addr_valid_for_region(mm, r, a, f);
done:
	return b;
}

/*
 * Fill in a next available MPU protection region with a specified
 * mapping. If needed, this swaps out an existing mapping using
 * a simple round-robin algorithm, which affects only MPU regions
 * that have the priority above the barrier (i.e. all regions
 * below barrier are locked).
 */
static inline void mpu_page_map(struct mm_struct *mm, u32 a,
	u32 sz, u32 acs, u32 srd, u32 xn)
{
	struct mpu_context *p = mpu_context_p(mm);
	u32 i, b, t, s = 1 << (sz + 1);

	/*
	 * Prepare region settings. Get attributes of the appropriate
	 * kernel region, and extend them with the user's attrs.
	 */
	for (i = mpu_hw_reg_indx, t = 0; i > 0; i--) {
		mpu_region_read(i - 1, &b, &t);
		b &= ~0xF;
		if (a < b)
			continue;
		if (a >= (b + (1 << (((t >> 1) & 0x1F) + 1))))
			continue;
		break;
	}

	/*
	 * Extract TEX|S|C|B, and OR with the specified configs
	 */
	t &= MPU_RASR_TEX_SCB_MASK;
	t |= MPU_RASR_XN(xn) | MPU_RASR_AP(acs) | MPU_RASR_SRD(srd) |
	     MPU_RASR_SIZE(sz) | MPU_RASR_ENABLE;
	b = a & ~(s - 1);

	/*
	 * This is next MPU region for this process
	 */
	i = p->indx;

	/*
	 * Copy them to the context so we can restore them on a switch.
	 */
	p->mpu_regs[i].base = b;
	p->mpu_regs[i].attr = t;

	/*
	 * Set up an MPU region in the hardware.
	 */
	mpu_region_write(i, b, t);

	/*
	 * Set the context for a next region allocation.
	 */
	p->indx = (i == (MPU_REG_NUM - 1)) ? p->barrier : i + 1;
}

/*
 * Set a "priority barrier" for the protection region for
 * the specified barrier. Further MPU mappings will round-robin
 * above that barrier but will not touch any mappings below it.
 */
static inline void mpu_page_barrier(struct mm_struct *mm, u32 barrier)
{
	mpu_context_p(mm)->barrier = barrier;
}

/*
 * Set up the MPU protections region from the context
 * of the specified process (in preparation to re-starting it).
 */
static inline void mpu_page_copyall(struct mm_struct *mm)
{
	struct mpu_context *p = mpu_context_p(mm);
	int i;

	for (i = mpu_hw_reg_indx; i < MPU_REG_NUM; i ++)
		mpu_region_write(i, p->mpu_regs[i].base, p->mpu_regs[i].attr);
}

/*
 * Calculate settings for sub-regions in a page
 */
static inline int mpu_page_srd(struct mm_struct *mm, u32 a, u32 f, u32 reg_size)
{
	struct mpu_addr_region *r;
	u32 pb, pt, p, x, srd = 0;
	u32 reg_mask = ~(reg_size - 1);
	u32 subreg_size = reg_size / MPU_SUB_NUM;
	int s;

	/*
	 * Check if there is even an MPU address region for this address
	 */
	r = mpu_addr_region_find(a);
	if (!r)
		goto done;

	/*
	 * These are the boundaries for the region
	 */
	pb = a & reg_mask;
	pt = pb + reg_size;

	/*
	 * Only those parts of 4K pages that are mapped in the context
	 * get mapped as sub-regions.
	 */
	for (srd = MPU_SUB_MSK, s = 0, p = pb + subreg_size - 1;
	     p < pt; p += subreg_size, s++) {
		x = mpu_context_addr_valid_for_region(mm, r, p, f);
		srd &= ~(x << s);
	}

done:
	return srd;
}

/*
 * Map in a 32K protection region. Access is ANY (RD/WR/EXEC).
 * Sub-regions define mappings (or not) for the surounding pages.
 */
static inline void mpu_page_map_32k(struct mm_struct *mm, u32 a, u32 f)
{
	u32 srd = mpu_page_srd(mm, a, f, PAGE32K_SIZE);

	mpu_page_map(mm, a, 0xE, 0x3, srd, 0);
}

/*
 * Debug service: print current MPU regions
 */
static void mpu_page_printall(const char *s, struct mm_struct *mm)
{
	struct mpu_context *p = mpu_context_p(mm);
	u32 a, b;
	int i;

	for (i = 0; i < MPU_REG_NUM; i ++) {
		mpu_region_read(i, &b, &a);
		printk("%s: reg=%d: cb=%08x,ca=%08x,mb=%08x,ma=%08x\n", s, i,
			p->mpu_regs[i].base, p->mpu_regs[i].attr, b, a);
	}
}

/******************************************************************************
 * A set of services exported by this module
 ******************************************************************************/

/*
 * Define a mappable region
 */
void mpu_region_define(u32 b, u32 t)
{
	mpu_addr_region_define(b, t);
}

/*
 * Enable the MPU hardware as well as initalize related data structures
 * This is called from the architecture-specific initilization code.
 */
void __init mpu_setup(void)
{
	u32 bot, top;

	/*
	 * Figure out the RAM geometry. This code implements a very
	 * simplistic approach; we assume there is a single bank of RAM.
	 */
	bot = memblock_start_of_DRAM();
	top = memblock_end_of_DRAM();
	mpu_region_define(bot, top);

	/*
	 * Turn the MPU and related h/w intefaces on.
	 */
	mpu_hw_on();
}

/*
 * Store new permissions for a particular page in the MPU context.
 * This is called by the architecture-neutral code in do_mmap.
 */
void protect_page(struct mm_struct *mm, unsigned long addr, unsigned long flags)
{
	mpu_context_map_page(mm, addr, flags);
}

/*
 * Update the MPU settings after permissions have changed for a mm area
 * This is called by the architecture-neutral code related to do_mmap.
 */
void update_protections(struct mm_struct *mm)
{

}

/*
 * Create the MPU context for a new process.
 * This is called from init_new_context.
 */
int mpu_init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
	mpu_context_init(mm);

	return 0;
}

/*
 * Free up the the MPU context for a process.
 * This is called from destroy_context.
 */
void mpu_destroy_context(struct mm_struct *mm)
{
	mpu_context_free(mm);
}

/*
 * MPU-specific part of switch_mm
 */
void mpu_switch_mm(struct mm_struct *prev, struct mm_struct *next)
{
	/*
	 * Set up the MPU protection regions with the values
	 * that were used by the new process last time it
	 * was called. This will be all-disabled for a first call
	 * because the initial table in the context is all zeroes.
	 */
	mpu_page_copyall(next);
}

/*
 * MPU-specific part of start_thread.
 * This process is about to go to the user mode.
 */
void mpu_start_thread(struct pt_regs *regs)
{
	struct mm_struct *mm = current->mm;
#ifdef CONFIG_MPU_STACK_REDZONE
	struct mpu_context *p = mpu_context_p(mm);
	u32 top, bot;
#endif
	u32 reg_pow, reg_len, reg_msk;
	u32 sub_pow, sub_len, sub_msk;
	u32 srd, adr;
	int n = 0, len;

	/*
	 * Set up mappings for the stack. Select the region size which is most
	 * close to the stack length
	 */
	len = mm->start_stack - mm->context.end_brk;
	for (reg_pow = 7; reg_pow < 32; reg_pow++)
		if ((1 << (reg_pow + 1)) >= len)
			break;

	if (reg_pow == 32) {
		reg_len = reg_msk = sub_len = sub_msk = 0;
		goto out;
	}

	reg_len = 1 << (reg_pow + 1);
	reg_msk = ~(reg_len - 1);

	sub_len = reg_len >> 3;
	sub_pow = __ffs(sub_len);
	sub_msk = ~(sub_len - 1);

	/*
	 * Disable interrupts since it is important to leave
	 * the MPU hardware in a consistent state
	 */
	local_irq_disable();

	/*
	 * Map in the page for the stack bottom half
	 */
#ifdef CONFIG_MPU_STACK_REDZONE
	p->redzone_bot = p->redzone_top = 0;
#endif

	adr = mm->context.end_brk;
	srd = mpu_page_srd(mm, adr, VM_READ | VM_WRITE, reg_len);

#ifdef CONFIG_MPU_STACK_REDZONE
	/*
	 * If the user has configured a stack "red zone", disable the
	 * subregion(s) at the bottom of the stack
	 */
	bot = top = (adr + sub_len - 1) & sub_msk;
	adr &= reg_msk;
	while ((top < adr + reg_len) && (top - bot < MPU_REDZONE_MIN)) {
		srd |= 1 << ((top - adr) >> sub_pow);
		top = (top + sub_len) & sub_msk;
	}
#endif
	if (srd != MPU_SUB_MSK) {
		mpu_page_map(mm, adr, reg_pow, 0x3, srd, 0);
		n++;
	}

	/*
	 * Map in the page for the stack top half
	 */
	if ((mm->context.end_brk & reg_msk) != (mm->start_stack & reg_msk)) {
		adr = mm->start_stack;
		srd = mpu_page_srd(mm, adr, VM_READ | VM_WRITE, reg_len);

#ifdef CONFIG_MPU_STACK_REDZONE
		adr &= reg_msk;
		while ((top < adr + reg_len) && (top - bot < MPU_REDZONE_MIN)) {
			srd |= 1 << ((top - adr) >> sub_pow);
			top = (top + sub_len) & sub_msk;
		}
#endif
		if (srd != MPU_SUB_MSK) {
			mpu_page_map(mm, adr, reg_pow, 0x3, srd, 0);
			n++;
		}
	}

	/*
	 * Stack is mapped. Set a barrier in the MPU context so that the
	 * round-robin page mapping doesn't touch the mappings for the stack.
	 */
#ifdef CONFIG_MPU_STACK_REDZONE
	p->redzone_top = top;
	p->redzone_bot = bot;
#endif
	mpu_page_barrier(mm, mpu_hw_reg_indx + n);

	/*
	 * Enable interrupts
	 */
	local_irq_enable();
	if (!n)
		goto out;

#ifdef DEBUG_MPU
	printk("Start process:\n"
		"   name: `%s`, control: 0x%x, indx: %d/%d\n"
		"   code section: 0x%08lx .. 0x%08lx %6ld\n"
		"   data section: 0x%08lx .. 0x%08lx %6ld\n"
		"  stack section: 0x%08lx .. 0x%08lx %6ld/%ld(%d%s region%s)\n",
		current->comm, readl(&NVIC->mpu_control), mpu_hw_reg_indx, n,
		mm->start_code, mm->end_code, mm->end_code - mm->start_code,
		mm->start_data, mm->end_data, mm->end_data - mm->start_data,
		mm->context.end_brk, mm->start_stack,
#ifdef CONFIG_MPU_STACK_REDZONE
		mm->start_stack - p->redzone_top,
#else
		mm->start_stack - mm->context.end_brk,
#endif
		mm->start_stack - mm->context.end_brk,
		reg_len >= 1024 ? reg_len / 1024 : reg_len,
		reg_len >= 1024 ? "K" : "", n > 1 ? "s" : "");
#ifdef CONFIG_MPU_STACK_REDZONE
	printk(" stack red zone: 0x%08x .. 0x%08x %6d\n",
		p->redzone_bot, p->redzone_top,
		p->redzone_top - p->redzone_bot);
#endif
	mpu_addr_region_tbl_print();
	mpu_page_printall(__func__, mm);
#endif

out:
	if (!n) {
		printk("MPU: can't map %dB of stack @{%08lx;%08lx}%s\n"
			" reg(%d): %d,%d,0x%08x; sub: %d,0x%08x\n"
			" kill the process `%s`\n",
			len, mm->context.end_brk, mm->start_stack,
#ifdef CONFIG_MPU_STACK_REDZONE
			" + redzone area",
#else
			"",
#endif
			n, reg_pow, reg_len, reg_msk,
			sub_len, sub_msk,
			current->comm);
		mpu_page_printall(__func__, mm);

		/*
		 * Send a SIGSEGV to the process
		 */
		send_sig(SIGSEGV, current, 0);
	}
}

/*
 * Memory Fault (memmanage) exception handler.
 * This is called from the architecture-specific low-level exception handler.
 * It is assumed that this code doesn't get pre-emtped.
 */
asmlinkage void __exception do_memmanage(struct pt_regs *regs)
{
#ifdef CONFIG_DEBUG_USER
	static struct {
		u8	msk;
		char	*str;
	} faults[] = {
		{ MMFSR_MLSPERR, "fault during FP lazy state preservation" },
		{ MMFSR_MSTKERR, "derived fault on exception entry" },
		{ MMFSR_MUNSTKERR, "derived fault on exception return" },
		{ MMFSR_DACCVIOL, "data access violation" },
		{ MMFSR_IACCVIOL, "access violation on instruction fetch" }
	};
	int i, n;
#endif /* CONFIG_DEBUG_USER */

#ifdef CONFIG_MPU_STACK_REDZONE
	struct mpu_context *p;
#endif
	u32 fault_status, mfar, pc, addr = 0;
	struct mm_struct *mm = current->mm;
	unsigned long flags;

	/*
	 * Read the fault status register from the MPU hardware
	 */
	fault_status = readl(&NVIC->fault_status);

	/*
	 * Calculate a candidate offending address
	 */
	mfar = readl(&NVIC->mfar);
	pc = instruction_pointer(regs);

	/*
	 * Clean up the memory fault status register for a next exception
	 */
	writel(fault_status, &NVIC->fault_status);

	/*
	 * Did we get a valid address in the memory fault address register?
	 * If so, this is a data access failure (can't tell read or write).
	 */
	if (fault_status & MMFSR_MMARVALID) {
		addr = mfar;
		flags = VM_READ | VM_WRITE;
	}
	/*
	 * If there is no valid address, did we get an instuction access
	 * failure? If so, this is a code access failure.
	 */
	else if (fault_status & MMFSR_IACCVIOL) {
		addr = pc;
		flags = VM_EXEC;
	}
	/*
	 * If the fault happened on exception return, then this might be
	 * caused by pthread switch; try to remap the stack region, and then
	 * retry to return from exception
	 */
	else if (fault_status & MMFSR_MUNSTKERR) {
		addr = regs->ARM_sp;
		flags = VM_READ | VM_WRITE;
	}
	/*
	 * Some error bit set in the memory fault address register.
	 * There is no recovery from that since no registers
	 * were saved on the stack on this exception, that is,
	 * we have no PC saved to return to user mode.
	 */
	else {
		goto kill;
	}

	/*
	 * We have a potentially benign exception. See if there is a valid
	 * mapping for the offending address and if so, "map" it in.
	 * The only exception is the stack redzone (if configured).
	 * Note: This code doesn't esnure XN (i.e. code protection),
	 * although it would be easy to do so. All pages that are mapped
	 * are READ/WRITE/EXEC.
	 */
	if (mpu_context_addr_valid(mm, addr, flags)) {
#if defined(DEBUG_PROC_NAME)
		if (!strcmp(current->comm, DEBUG_PROC_NAME))
			printk("%s.1: %08x.%02x/%d/%s\n",
				__func__, addr, fault_status,
				mpu_context_p(mm)->indx,
				current->comm);
#endif
		mpu_page_map_32k(mm, addr, flags);

		/*
		 * If the fault address is close to the 32K boundary, then check
		 * and map next 32K area as well. This is to be able to handle
		 * situations when the CPU must cross the 32K boundary to
		 * complete an instruction/data fetch
		 */
		if ((addr & PAGE32K_MASK) !=
		    ((addr + (1 << CONFIG_ARM_L1_CACHE_SHIFT)) & PAGE32K_MASK)){
			addr = addr + (1 << CONFIG_ARM_L1_CACHE_SHIFT);
			if (mpu_context_addr_valid(mm, addr, flags)) {
#if defined(DEBUG_PROC_NAME)
				if (!strcmp(current->comm, DEBUG_PROC_NAME))
					printk("%s.2: %08x.%02x/%d/%s\n",
						__func__, addr, fault_status,
						mpu_context_p(mm)->indx,
						current->comm);
#endif
				mpu_page_map_32k(mm, addr, flags);
			}
		}
	}
	/*
	 * No mapping for the offending address. Kill the process.
	 */
	else {
		goto kill;
	}

	/*
	 * All is done successfully. Go to the end.
	 */
	goto done;

kill:
	/*
	 * This is a real MPU fault because the procees did a bad access
	 */
#ifdef CONFIG_DEBUG_USER
	if (!(user_debug & UDBG_SEGV))
		goto sig;

	printk("\n\nMPU fault detected (MMFSR=%02x):\n   ", fault_status);
	for (i = 0, n = 0; i < ARRAY_SIZE(faults); i++) {
		if (!(fault_status & faults[i].msk))
			continue;
		printk("%s%s", n++ ? ", " : "", faults[i].str);
	}
	printk("\n");
	if (fault_status & 0x3) {
		printk("Fault address:\n"
			"   0x%08x\n", addr);
	}
#ifdef CONFIG_MPU_STACK_REDZONE
	p = mpu_context_p(mm);
#endif
	printk("Fault process:\n"
		"   name: `%s`\n"
		"   code section: 0x%08lx .. 0x%08lx %6ld\n"
		"   data section: 0x%08lx .. 0x%08lx %6ld\n"
		"  stack section: 0x%08lx .. 0x%08lx %6ld/%ld\n",
		current->comm,
		mm->start_code, mm->end_code, mm->end_code - mm->start_code,
		mm->start_data, mm->end_data, mm->end_data - mm->start_data,
		mm->context.end_brk, mm->start_stack,
#ifdef CONFIG_MPU_STACK_REDZONE
		mm->start_stack - p->redzone_top,
#else
		mm->start_stack - mm->context.end_brk,
#endif
		mm->start_stack - mm->context.end_brk);
#ifdef CONFIG_MPU_STACK_REDZONE
	printk(" stack red zone: 0x%08x .. 0x%08x %6d %s\n",
		p->redzone_bot, p->redzone_top, p->redzone_top - p->redzone_bot,
		(addr >= p->redzone_bot && addr < p->redzone_top) ? "[*]" : "");
#endif
	printk("MPU regions on fault:\n");
	mpu_page_printall("   ", mm);
	if (regs->ARM_pc >= mm->start_code && regs->ARM_pc < mm->end_code)
		printk("PC 0x%x is in the code section, offset 0x%x\n", (u32)regs->ARM_pc, (u32)(regs->ARM_pc - mm->start_code));
	if (regs->ARM_lr >= mm->start_code && regs->ARM_lr < mm->end_code)
		printk("LR 0x%x is in the code section, offset 0x%x\n", (u32)regs->ARM_lr, (u32)(regs->ARM_lr - mm->start_code));
	printk("Register values on fault:\n");
	__show_regs(regs);

sig:
#endif /* CONFIG_DEBUG_USER */

	/*
	 * Send a SIGSEGV to the process
	 */
	send_sig(SIGSEGV, current, 0);

done:
	return;
}
