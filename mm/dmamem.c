/*
 * Copyright (C) 2016, 2015
 * EmCraft Systems
 * Yuri Tikhonov (yur@emcraft.com)
 *
 * Based on M. Welsh (mdw@cs.cornell.edu) bigphysarea.c driver, extended
 * and adapted for linux-2.6.x by J. Joe Feise <jfeise@feise.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/bootmem.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/dmamem.h>
#include <linux/memblock.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>

#define DM_NAME		"dmamem"

/*
 * dmamem block descriptor
 */
struct dm_dsc {
	struct dm_dsc	*next;
	caddr_t		base;		/* base of allocated block */
	size_t		size;		/* size in bytes */
};

static unsigned long	dm_base;
static unsigned long	dm_sz_all;
static unsigned long	dm_sz_fb;

static struct dm_dsc	*dm_lst_free;
static struct dm_dsc	*dm_lst_used;

static struct resource	dm_res = {
	.name	= DM_NAME " area",
	.flags	= IORESOURCE_MEM | IORESOURCE_BUSY,
};

static int		dm_of_found, dm_of_ok;

#ifdef CONFIG_PROC_FS
static int dm_proc_show(struct seq_file *m, void *v);

static int dm_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dm_proc_show, NULL);
}

static const struct file_operations dm_proc_fops = {
	.open		= dm_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};
#endif

/*
 * Early fetch of `dmamem` node from device-tree
 */
static int early_init_dt_scan_dmamem(unsigned long node, const char *uname,
				     int depth, void *data)
{
	const char *comp = of_get_flat_dt_prop(node, "compatible", NULL);
	const __be32 *prop;

	if (!comp || strcmp(comp, "dmamem"))
		return 0;
	dm_of_found = 1;

	prop = of_get_flat_dt_prop(node, "base-addr", NULL);
	if (!prop) {
		pr_info("%s: no <base-addr> property found.\n", DM_NAME);
		goto out;
	}
	dm_base = be32_to_cpup(prop);

	prop = of_get_flat_dt_prop(node, "full-size", NULL);
	if (!prop) {
		pr_info("%s: no <full-size> property found.\n", DM_NAME);
		goto out;
	}
	dm_sz_all = be32_to_cpup(prop);

	prop = of_get_flat_dt_prop(node, "fb-size", NULL);
	if (prop)
		dm_sz_fb = be32_to_cpup(prop);

	dm_of_ok = 1;
out:
	/*
	 * Break now
	 */
	return 1;
}

static void dm_early_init(void)
{
	of_scan_flat_dt(early_init_dt_scan_dmamem, NULL);
	if (!dm_of_found)
		pr_info("%s: No dts node found.\n", DM_NAME);
}

static int __init dm_sys_init(void)
{
	int	rv = 0;

	/* Check if we've got params from device-tree */
	if (!dm_of_ok) {
		rv = -ENODEV;
		goto out;
	}

	/* Register the resource for mem allocation */
	dm_res.start = dm_base;
	dm_res.end = dm_base + dm_sz_all;
	request_resource(&iomem_resource, &dm_res);

	/* Create free list */
	dm_lst_free = kmalloc(sizeof(struct dm_dsc), GFP_KERNEL);
	if (!dm_lst_free) {
		rv = -ENOMEM;
		goto out;
	}
	dm_lst_free->next = NULL;
	dm_lst_free->base = (void *)(dm_base + dm_sz_fb);
	dm_lst_free->size = dm_sz_all - dm_sz_fb;

#ifdef CONFIG_PROC_FS
	/*
	 * Create /proc entry for it
	 */
	if (!proc_create(DM_NAME, 0, NULL, &dm_proc_fops)) {
		/*
		 * continue without proc support, it is not fatal in itself
		 */
		printk(KERN_INFO "%s: failed create proc entry\n", DM_NAME);
	}
#endif

out:
	if (rv) {
		pr_info("%s: no coherent mem will be available (%d).\n",
			DM_NAME, rv);
		dm_sz_all = 0;
	}

	return rv;
}
__initcall(dm_sys_init);

/*
 * Reserve memory for dmamem
 */
void dmamem_init(void)
{
	dm_early_init();

	if (!dm_sz_all)
		return;

	memblock_reserve(dm_base, dm_sz_all);
}

/*
 * Allocate mem from dmamem region
 */
int dmamem_alloc(size_t size, dma_addr_t *dma_handle, void **ret)
{
	struct dm_dsc	*dm, *ndm, *adm, **pdm;
	caddr_t		abase = 0;
	size_t		align = PAGE_SIZE;
	int		count;

	if (!dma_handle || !ret || !dm_sz_all)
		return 0;

	count = (size + PAGE_SIZE - 1) / PAGE_SIZE;

	ndm = NULL;
	adm = NULL;

	/*
	 * Search a free block which is large enough, even with alignment.
	 */
	pdm = &dm_lst_free;
	while (*pdm != NULL) {
		dm= *pdm;
		abase = (caddr_t)((((unsigned long)dm->base +
				    align - 1) / align) * align);

		if (abase + count * PAGE_SIZE <= dm->base + dm->size)
			break;

		pdm = &dm->next;
	}

	dm = *pdm;
	if (!dm)
		return 0;

	/*
	 * When we have to align, the pages needed for alignment can
	 * be put back to the free pool.
	 * We check here if we need a second dmamem data structure later
	 * and allocate it now, so that we don't have to check for a
	 * failed kmalloc later.
	 */
	if (abase - dm->base + count * PAGE_SIZE < dm->size) {
		ndm = kmalloc(sizeof(struct dm_dsc), GFP_KERNEL);
		if (!ndm)
			return 0;
	}

	if (abase != dm->base) {
		adm = kmalloc(sizeof(struct dm_dsc), GFP_KERNEL);
		if (!adm) {
			if (ndm)
				kfree(ndm);
			return 0;
		}
		adm->base = dm->base;
		adm->size = abase - dm->base;
		dm->base = abase;
		dm->size -= adm->size;
		adm->next = dm;
		*pdm = adm;
		pdm = &adm->next;
	}

	if (ndm) {
		/*
		 * Range is larger than needed, create a new list element for
		 * the used list and shrink the element in the free list.
		 */
		ndm->base = dm->base;
		ndm->size = count * PAGE_SIZE;
		dm->base = ndm->base + ndm->size;
		dm->size = dm->size - ndm->size;
	} else {
		/*
		 * Range fits perfectly, remove it from free list.
		 */
		*pdm = dm->next;
		ndm = dm;
	}

	/*
	 * Insert block into used list
	 */
	ndm->next = dm_lst_used;
	dm_lst_used = ndm;

	*dma_handle = (dma_addr_t)ndm->base;
	*ret = ndm->base;

	return 1;
}

/*
 * Free pages previously allocated with dmamem_xxx()
 */
int dmamem_free(void *vaddr)
{
	struct dm_dsc *prev, *next, *dm, **pdm;
	caddr_t base = (caddr_t)vaddr;

	if (!dm_sz_all)
		return 0;

	/*
	 * Search the block in the used list.
	 */
	for (pdm = &dm_lst_used; *pdm; pdm = &(*pdm)->next) {
		if ((*pdm)->base == base)
			break;
	}

	if (*pdm == NULL)
		return 0;

	/*
	 * Remove range from the used list:
	 */
	dm = *pdm;
	*pdm = (*pdm)->next;

	/*
	 * The free-list is sorted by address, search insertion point
	 * and insert block in free list.
	 */
	for (pdm = &dm_lst_free, prev = NULL; *pdm;
	     prev = *pdm, pdm = &(*pdm)->next) {
		if ((*pdm)->base >= base)
			break;
	}

	/*
	 * Concatenate free range with neighbors, if possible.
	 * Try for upper neighbor (next in list) first, then
	 * for lower neighbor (predecessor in list).
	 */
	dm->next = *pdm;
	*pdm = dm;

	if (dm->next &&
	    dm->base + dm->size == dm->next->base) {
		next = dm->next;
		dm->size += dm->next->size;
		dm->next = next->next;
		kfree(next);
	}

	if (prev &&
	    prev->base + prev->size == dm->base) {
		prev->size += prev->next->size;
		prev->next = dm->next;
		kfree(dm);
	}

	return 1;
}

/*
 * Get 'fb' area reserved in dmamem
 */
int dmamem_fb_get(dma_addr_t *base, unsigned long *size)
{
	if (!dm_sz_all)
		return -ENODEV;

	if (base)
		*base = dm_base;

	if (size)
		*size = dm_sz_fb;

	return 0;
}

#ifdef CONFIG_PROC_FS
/*
 * Called on /proc read
 */
static int dm_proc_show(struct seq_file *m, void *v)
{
	struct dm_dsc	*dm;
	int		free_count, free_total, free_max;
	int		used_count, used_total, used_max;

	free_count = 0;
	free_total = 0;
	free_max   = 0;
	for (dm = dm_lst_free; dm; dm = dm->next) {
		free_count++;
		free_total += dm->size;
		if (dm->size > free_max)
			free_max = dm->size;
	}

	used_count = 0;
	used_total = 0;
	used_max   = 0;
	for (dm = dm_lst_used; dm; dm = dm->next) {
		used_count++;
		used_total += dm->size;
		if (dm->size > used_max)
			used_max = dm->size;
	}

	if (!dm_sz_all) {
		seq_printf(m, "No %s area allocated!\n", DM_NAME);
		goto out;
	}

	seq_printf(m,
		"%s area, fb mem size %ld kB, gen dma mem size %ld kB\n"
		"                       free list:             used list:\n"
		"number of blocks:      %8d               %8d\n"
		"size of largest block: %8d kB            %8d kB\n"
		"total:                 %8d kB            %8d kB\n",
		DM_NAME, dm_sz_fb / 1024, (dm_sz_all - dm_sz_fb) / 1024,
		free_count, used_count,
		free_max / 1024, used_max / 1024,
		free_total / 1024, used_total /1024);
out:
	return 0;
}
#endif
