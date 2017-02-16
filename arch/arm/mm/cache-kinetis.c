/*
 * (C) Copyright 2012-2017
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Dmitry Konyshev <probables@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>

/*
 * Local Memory Controller: Cache control register
 */
#define KINETIS_LMEM_PCCCR		(*(volatile u32 *)0xe0082000)
#define KINETIS_LMEM_PCCLCR		(*(volatile u32 *)0xe0082004)
#define KINETIS_LMEM_PCCSAR		(*(volatile u32 *)0xe0082008)
#define KINETIS_LMEM_PSCCR		(*(volatile u32 *)0xe0082800)
#define KINETIS_LMEM_PSCLCR		(*(volatile u32 *)0xe0082804)
#define KINETIS_LMEM_PSCSAR		(*(volatile u32 *)0xe0082808)

/* Initiate Cache Command */
#define KINETIS_LMEM_CCR_GO_MSK		(1 << 31)
/* Push Way 1 */
#define KINETIS_LMEM_CCR_PUSHW1_MSK	(1 << 27)
/* Invalidate Way 1 */
#define KINETIS_LMEM_CCR_INVW1_MSK	(1 << 26)
/* Push Way 0 */
#define KINETIS_LMEM_CCR_PUSHW0_MSK	(1 << 25)
/* Invalidate Way 0 */
#define KINETIS_LMEM_CCR_INVW0_MSK	(1 << 24)
/* Enable Write Buffer */
#define KINETIS_LMEM_CCR_ENWRBUF_MSK	(1 << 1)
/* Cache enable */
#define KINETIS_LMEM_CCR_ENCACHE_MSK	(1 << 0)

#define KINETIS_LMEM_LCR_LADSEL_MSK	(1 << 26)
#define KINETIS_LMEM_LCR_LCMD		24

#define KINETIS_LMEM_SAR_LGO_MSK	(1 << 0)

/* Cache line size */
#define KINETIS_CACHE_LINE_SIZE		0x10

/*
 * Get DCache current mode description
 */
char *dcache_is_arch_specific(void)
{
	return !(KINETIS_LMEM_PSCCR & (1 << 0)) ? "NO" : "WRITE-BACK";
}

/*
 * Get ICache current mode description
 */
char *icache_is_arch_specific(void)
{
	return !(KINETIS_LMEM_PCCCR & (1 << 0)) ? "NO" : "WRITE-TROUGH";
}

/*
 * Disable Kinetis Processor Code (PC) cache and save previous state
 */
void kinetis_pc_cache_save(unsigned long *flags)
{
	/* Save previous state of cache */
	*flags = KINETIS_LMEM_PCCCR &
		(KINETIS_LMEM_CCR_ENWRBUF_MSK | KINETIS_LMEM_CCR_ENCACHE_MSK);

	/* Clear and disable cache */
	KINETIS_LMEM_PCCCR =
		KINETIS_LMEM_CCR_GO_MSK |
		KINETIS_LMEM_CCR_PUSHW1_MSK | KINETIS_LMEM_CCR_INVW1_MSK |
		KINETIS_LMEM_CCR_PUSHW0_MSK | KINETIS_LMEM_CCR_INVW0_MSK;
	/* Wait for the cache operation to finish */
	while (KINETIS_LMEM_PCCCR & KINETIS_LMEM_CCR_GO_MSK);
}

/*
 * Restore a previous state of Kinetis Processor Code (PC) cache
 */
void kinetis_pc_cache_restore(unsigned long *flags)
{
	/* Invalidate and optionally enable cache */
	KINETIS_LMEM_PCCCR =
		KINETIS_LMEM_CCR_GO_MSK |
		KINETIS_LMEM_CCR_INVW1_MSK | KINETIS_LMEM_CCR_INVW0_MSK |
		(*flags);
	/* Wait for the cache operation to finish */
	while (KINETIS_LMEM_PCCCR & KINETIS_LMEM_CCR_GO_MSK);
}

/*
 * Flush code cache lines
 */
void kinetis_pc_cache_flush_mlines(void *adr, unsigned long len)
{
	void	*end_adr = (void *)((u32)adr + len);

	adr = (void *)((u32)adr & ~(KINETIS_CACHE_LINE_SIZE - 1));
	do {
		KINETIS_LMEM_PCCLCR = KINETIS_LMEM_LCR_LADSEL_MSK |
				      (0x2 << KINETIS_LMEM_LCR_LCMD);
		KINETIS_LMEM_PCCSAR = ((u32)adr & ~0x3) |
				      KINETIS_LMEM_SAR_LGO_MSK;
		while (KINETIS_LMEM_PCCSAR & KINETIS_LMEM_SAR_LGO_MSK);

		adr = (void *)((u32)adr + KINETIS_CACHE_LINE_SIZE);
	} while (adr < end_adr);
}

/*
 * Invalidate code cache lines
 */
void kinetis_pc_cache_inval_mlines(void *adr, unsigned long len)
{
	void	*end_adr = (void *)((u32)adr + len);

	adr = (void *)((u32)adr & ~(KINETIS_CACHE_LINE_SIZE - 1));
	do {
		KINETIS_LMEM_PCCLCR = KINETIS_LMEM_LCR_LADSEL_MSK |
				      (0x1 << KINETIS_LMEM_LCR_LCMD);
		KINETIS_LMEM_PCCSAR = ((u32)adr & ~0x3) |
				      KINETIS_LMEM_SAR_LGO_MSK;
		while (KINETIS_LMEM_PCCSAR & KINETIS_LMEM_SAR_LGO_MSK);

		adr = (void *)((u32)adr + KINETIS_CACHE_LINE_SIZE);
	} while (adr < end_adr);
}

/*
 * Flush Kinetis Processor Code (PC) cache
 */
void kinetis_flush_icache_all(void)
{
	unsigned long	flags;

	kinetis_pc_cache_save(&flags);
	kinetis_pc_cache_restore(&flags);
}

/*
 * Disable Kinetis Processor System (PS) cache and save previous state
 */
void kinetis_ps_cache_save(unsigned long *flags)
{
	/* Save previous state of cache */
	*flags = KINETIS_LMEM_PSCCR &
		(KINETIS_LMEM_CCR_ENWRBUF_MSK | KINETIS_LMEM_CCR_ENCACHE_MSK);

	/* Clear and disable cache */
	KINETIS_LMEM_PSCCR =
		KINETIS_LMEM_CCR_GO_MSK |
		KINETIS_LMEM_CCR_PUSHW1_MSK | KINETIS_LMEM_CCR_INVW1_MSK |
		KINETIS_LMEM_CCR_PUSHW0_MSK | KINETIS_LMEM_CCR_INVW0_MSK;
	/* Wait for the cache operation to finish */
	while (KINETIS_LMEM_PSCCR & KINETIS_LMEM_CCR_GO_MSK);
}

/*
 * Restore a previous state of Kinetis Processor System (PS) cache
 */
void kinetis_ps_cache_restore(unsigned long *flags)
{
	/* Invalidate and optionally enable cache */
	KINETIS_LMEM_PSCCR =
		KINETIS_LMEM_CCR_GO_MSK |
		KINETIS_LMEM_CCR_INVW1_MSK | KINETIS_LMEM_CCR_INVW0_MSK |
		(*flags);
	/* Wait for the cache operation to finish */
	while (KINETIS_LMEM_PSCCR & KINETIS_LMEM_CCR_GO_MSK);
}

static void kinetis_ps_cache_cmd_mlines(const void *addr, unsigned long len, int cmd)
{
	u32	p, end_adr = (u32)addr + len;

	p = (u32)addr & ~(KINETIS_CACHE_LINE_SIZE - 1);
	do {
		KINETIS_LMEM_PSCLCR = KINETIS_LMEM_LCR_LADSEL_MSK |
				      (cmd << KINETIS_LMEM_LCR_LCMD);
		KINETIS_LMEM_PSCSAR = (p & ~0x3) |
				      KINETIS_LMEM_SAR_LGO_MSK;
		while (KINETIS_LMEM_PSCSAR & KINETIS_LMEM_SAR_LGO_MSK);

		p = p + KINETIS_CACHE_LINE_SIZE;
	} while (p < end_adr);
}

/*
 * Flush data cache lines
 */
static void kinetis_ps_cache_flush_mlines(const void *adr, unsigned long len)
{
	kinetis_ps_cache_cmd_mlines(adr, len, 2);
}

/*
 * Invalidate data cache lines
 */
static void kinetis_ps_cache_inval_mlines(const void *adr, unsigned long len)
{
	kinetis_ps_cache_cmd_mlines(adr, len, 1);
}

/*
 * Clear data cache lines
 */
static void kinetis_ps_cache_clear_mlines(const void *adr, unsigned long len)
{
	kinetis_ps_cache_cmd_mlines(adr, len, 3);
}

/*
 * Flush Kinetis Processor System (PS) cache
 */
void kinetis_flush_kern_cache_all(void)
{
	unsigned long	flags;

	kinetis_ps_cache_save(&flags);
	kinetis_ps_cache_restore(&flags);
}

/*
 * Flush data cache levels up to the level of unification
 * inner shareable and invalidate the I-cache
 */
void kinetis_flush_kern_cache_louis(void)
{
	kinetis_flush_kern_cache_all();
}

/*
 * Clean and invalidate all user space cache entries
 * before a change of page tables
 */
void kinetis_flush_user_cache_all(void)
{
	/* Do nothing */
}

/*
 * Ensure that the I and D caches are coherent within specified
 * region. This is typically used when code has been written to
 * a memory region, and will be executed.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void kinetis_coherent_kern_range(unsigned long start, unsigned long end)
{
	kinetis_ps_cache_flush_mlines((void *)start, end - start);
	kinetis_pc_cache_inval_mlines((void *)start, end - start);
}

/*
 * Flush a range of TLB entries in the specified address space.
 * - start - start address (may not be aligned)
 * - end   - end address (exclusive, may not be aligned)
 * - flags - vm_area_struct flags describing address space
 */
void kinetis_flush_user_cache_range(unsigned long start, unsigned long end,
				unsigned int flags)
{
	/* Do nothing */
}

/*
 * Ensure coherency between the Icache and the Dcache in the
 * region described by start, end.
 */
int kinetis_coherent_user_range(unsigned long start, unsigned long end)
{
	kinetis_coherent_kern_range(start, end);
	return 0;
}

/*
 * Ensure that the data held in the page kaddr is written back
 * to the page in question.
 * - addr  - kernel address
 * - size  - region size
 */
void kinetis_flush_kern_dcache_area(void *addr, size_t size)
{
	kinetis_ps_cache_flush_mlines(addr, size);
}

/*
 * Invalidate the data cache within the specified region.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void kinetis_dma_inv_range(const void *start, const void *end)
{
	kinetis_ps_cache_inval_mlines(start, end - start);
}

/*
 * Clean the data cache within the specified region.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void kinetis_dma_clean_range(const void *start, const void *end)
{
	kinetis_ps_cache_clear_mlines(start, end - start);
}

/*
 * Clean & invalidate the data cache within the specified region.
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
void kinetis_dma_flush_range(const void *start, const void *end)
{
	kinetis_ps_cache_inval_mlines(start, end - start);
	kinetis_ps_cache_clear_mlines(start, end - start);
}

/*
 * Prepare region for DMAing
 * - start - kernel virtual start address
 * - size  - size of region
 * - dir   - DMA direction
 */
void kinetis_dma_map_area(const void *start, size_t size, int dir)
{
	void	*end = (void *)((u32)start + size);

	if (dir == DMA_FROM_DEVICE)
		kinetis_dma_inv_range(start, end);
	kinetis_dma_clean_range(start, end);
}

/*
 * DMA to region complete, unmap
 * - start - kernel virtual start address
 * - size  - size of region
 * - dir   - DMA direction
 */
void kinetis_dma_unmap_area(const void *start, size_t size, int dir)
{
	void	*end = (void *)((u32)start + size);

	if (dir == DMA_TO_DEVICE)
		kinetis_dma_inv_range(start, end);
}
