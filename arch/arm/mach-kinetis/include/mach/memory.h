/*
 * Based on original code by Alexander Potashev <aspotashev@emcraft.com>
 *
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * Copyright (C) 2015 Paul Osmialowski <pawelo@king.net.pl>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#ifndef _MACH_KINETIS_MEMORY_H
#define _MACH_KINETIS_MEMORY_H

#ifndef __ASSEMBLY__

/*
 * Write-through cached DDRAM-alias region.
 * This is used to run kernel core from.
 */
#define PHYS_ALIAS_OFFSET       UL(0x70000000)

#define ARCH_PFN_OFFSET	((unsigned long)(PHYS_ALIAS_OFFSET >> PAGE_SHIFT))

/*
 * On Kinetis K70, consistent DMA memory resides in a special
 * DDRAM alias region (non-cacheable DDRAM at 0x80000000).
 *
 */
#define PHYS_DMA_OFFSET UL(0x80000000)

/*
 * Mask of the field used to distinguish DDRAM aliases
 */
#define DRAM_ALIAS_MASK UL(0xf8000000)

/*
 * This macro converts an address in the kernel run-time memory
 * to an alias in the non-cacheable memory region
 */
#define KINETIS_DMA_ALIAS_ADDR(addr)					\
		(((unsigned long)(addr) & ~DRAM_ALIAS_MASK) |	\
		(PHYS_DMA_OFFSET & DRAM_ALIAS_MASK))

/*
 * This macro converts an address in the kernel code or
 * in the non-cacheable DMA region to an alias in
 * the run-time kernel memory region
 */
#define PHYS_ALIAS_ADDR(a)						\
	(((unsigned int)(a) & ~DRAM_ALIAS_MASK) | 			\
	(PHYS_ALIAS_OFFSET & DRAM_ALIAS_MASK))

/*
 * Architecture-specific low-level DMA macros
 */
#define __arch_dma_to_pfn(dev, addr) __phys_to_pfn(addr)

#define __arch_pfn_to_dma(dev, pfn) \
		((dma_addr_t)KINETIS_DMA_ALIAS_ADDR(__pfn_to_phys(pfn)))

#define __arch_dma_to_virt(dev, addr) \
		((void *)PHYS_ALIAS_ADDR(addr))

#define __arch_virt_to_dma(dev, addr) \
		((dma_addr_t)KINETIS_DMA_ALIAS_ADDR(addr))

#endif /* __ASSEMBLY__ */

#endif /*_MACH_KINETIS_MEMORY_H */
