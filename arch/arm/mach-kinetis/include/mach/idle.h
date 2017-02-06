/*
 * Based on original code by Vladimir Khusainov <vlad@emcraft.com>
 *
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
 *
 * Copyright (C) 2015 Paul Osmialowski <pawelo@king.net.pl>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#ifndef _MACH_KINETIS_IDLE_H
#define _MACH_KINETIS_IDLE_H

#ifndef __ASSEMBLY__

#include <linux/io.h>

static inline void handle_cpu_idle_quirks(void)
{
	/*
	 * This is a dirty hack that invalidates the I/D bus cache
	 * on Kinetis K70. This must be done after arch idle.
	 */
	writel(readl(IOMEM(0xe0082000)) | 0x85000000, IOMEM(0xe0082000));
}

#endif /* __ASSEMBLY__ */

#endif /*_MACH_KINETIS_IDLE_H */
