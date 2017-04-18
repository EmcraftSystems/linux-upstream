/*
 * Custom memory mapper for STM32F4X9 to workaround bug that
 * SDRAM and static memory (NOR flash) should not be accessed simultaneously
 * See Errata 2.8.7 for details.
 *
 * We put SDRAM in self-refresh before accessing NOR flash
 * and wakeup SDRAM after we have finished access.
 *
 * Based on physmap_of.c code.
 *
 * Copyright (C) 2013, 2015
 * Pavel Boldin, Emcraft Systems, paboldin@emcraft.com
 * Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/concat.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

#include <mach/sram.h>

#define CYCLES_PER_US	168
#define STM32_SDRAM_FMC_BASE		0xA0000140
#define STM32_SDRAM_FMC			((volatile struct stm32_fmc_regs *) \
						STM32_SDRAM_FMC_BASE)

#define FMC_SDCMR_MODE_NORMAL		0
#define FMC_SDCMR_MODE_START_CLOCK	1
#define FMC_SDCMR_MODE_PRECHARGE	2
#define FMC_SDCMR_MODE_AUTOREFRESH	3
#define FMC_SDCMR_MODE_WRITE_MODE	4
#define FMC_SDCMR_MODE_SELFREFRESH	5

#define FMC_SDCMR_BANK_1		(1 << 4)
#define FMC_SDCMR_BANK_2		(1 << 3)

#define FMC_SDSR_BUSY			(1 << 5)

#define FMC_BUSY_WAIT()		do { \
		__asm__ __volatile__ ("dsb" : : : "memory"); \
		while(STM32_SDRAM_FMC->sdsr & FMC_SDSR_BUSY); \
	} while(0);

struct of_flash_list {
	struct mtd_info *mtd;
	struct map_info map;
	struct resource *res;
};

struct of_flash {
	struct mtd_info		*cmtd;
	int list_size; /* number of elements in of_flash_list */
	struct of_flash_list	list[0];
};

struct stm32_fmc_regs {
	/* Control registers */
	u32 sdcr1;
	u32 sdcr2;

	/* Timing registers */
	u32 sdtr1;
	u32 sdtr2;

	/* Mode register */
	u32 sdcmr;

	/* Refresh timing register */
	u32 sdrtr;

	/* Status register */
	u32 sdsr;
};

static volatile unsigned long SRAM_DATA sr_flash, sr_len;
static volatile map_word SRAM_DATA sr_val;

static volatile unsigned long SRAM_DATA sram_buffer_start, sram_buffer_size;

extern char _sram_start, _sram_end;
extern char __sram_loc, _esram_loc;

/******************************************************************************
 * SRAM helpers
 ******************************************************************************/
static inline int enter_critical_code(void)
{
	unsigned long flags;
	local_fiq_disable();
	local_irq_save(flags);
	return flags;
}

static inline void leave_critical_code(unsigned long flags)
{
	local_irq_restore(flags);
	local_fiq_enable();
}

static inline void loop_delay(volatile unsigned int u)
{
	/* Cycle is two instructions each */
	u >>= 1;
	while(u-- != 0);
}

static inline void stop_ram(void)
{
	STM32_SDRAM_FMC->sdcmr = FMC_SDCMR_BANK_1 | FMC_SDCMR_MODE_SELFREFRESH;
	FMC_BUSY_WAIT();
	loop_delay(CYCLES_PER_US);
}

static inline void start_ram(void)
{
	STM32_SDRAM_FMC->sdcmr = FMC_SDCMR_BANK_1 | FMC_SDCMR_MODE_PRECHARGE;
	FMC_BUSY_WAIT();
	loop_delay(CYCLES_PER_US);

	STM32_SDRAM_FMC->sdcmr = FMC_SDCMR_BANK_1 | FMC_SDCMR_MODE_NORMAL;
	FMC_BUSY_WAIT();
	loop_delay(CYCLES_PER_US * 60);
}

SRAM_TEXT void sram_stm32f4_flash_read(void)
{
	stop_ram();
	sr_val.x[0] = *(volatile uint16_t __force*)(sr_flash);
	loop_delay(10);
	start_ram();
}

SRAM_TEXT void sram_stm32f4_flash_write(void)
{
	stop_ram();
	*(volatile uint16_t __force*)(sr_flash) = *(uint16_t*)&sr_val.x[0];
	loop_delay(20);
	start_ram();
}

SRAM_TEXT void sram_stm32f4_flash_copy_from(void)
{
	int i;
	stop_ram();
	for(i = sram_buffer_start;sr_len;)
	{
		*(volatile uint16_t __force*)i = *(volatile uint16_t*)sr_flash;

		sr_len -= 2;
		sr_flash += 2;
		i += 2;

		loop_delay(10);
	}
	start_ram();
}

SRAM_TEXT void sram_stm32f4_flash_copy_to(void)
{
	int i;
	stop_ram();
	for(i = sram_buffer_start;sr_len;)
	{
		*(volatile uint16_t __force*)sr_flash = *(volatile uint16_t*)i;

		sr_len -= 2;
		sr_flash += 2;
		i += 2;

		loop_delay(20);
	}
	start_ram();
}

map_word stm32f4_flash_read(struct map_info *map, unsigned long ofs)
{
	int flags;

	sr_flash = map->phys + ofs;

	flags = enter_critical_code();
	sram_stm32f4_flash_read();
	leave_critical_code(flags);

	return sr_val;
}

void stm32f4_flash_write(struct map_info *map, const map_word datum, unsigned long ofs)
{
	int flags;
	sr_flash = map->phys + ofs;
	sr_val = datum;

	flags = enter_critical_code();
	sram_stm32f4_flash_write();
	leave_critical_code(flags);
}

void stm32f4_flash_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	int read_len;
	unsigned long lto = (unsigned long)to;
	int flags;

	for(; len; ) {
		read_len = min((unsigned long)len, sram_buffer_size);

		sr_flash = map->phys + from;
		sr_len = ALIGN(read_len, 2);

		flags = enter_critical_code();
		sram_stm32f4_flash_copy_from();
		leave_critical_code(flags);

		memcpy((void*)lto, (void*)sram_buffer_start, read_len);

		from += read_len;
		lto += read_len;
		len -= read_len;
	}
}

void stm32f4_flash_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	int read_len;
	unsigned long lfrom = (unsigned long)from;
	int flags;

	for(; len; ) {
		read_len = min((unsigned long)len, sram_buffer_size);

		sr_flash = map->phys + to;
		sr_len = ALIGN(read_len, 2);

		memcpy((void*)sram_buffer_start, (void*)lfrom, read_len);

		flags = enter_critical_code();
		sram_stm32f4_flash_copy_to();
		leave_critical_code(flags);

		lfrom += read_len;
		to += read_len;
		len -= read_len;
	}
}

static int stm32f4_flash_init(void)
{
	int rv;

	printk(KERN_INFO "Initializing STM32F4 mapper, "
		"copying code from %p to %p, size %d",
		&__sram_loc, &_sram_start, &_esram_loc - &__sram_loc);

	/*
	 * Init SRAM, copy code to SRAM, and allocate buf
	 */
	rv = stm32_sram_init();
	if (rv) {
		printk(KERN_ERR "SRAM init err %d\n", rv);
		goto out;
	}
	rv = stm32_sram_relocate();
	if (rv) {
		printk(KERN_ERR "SRAM reloc err %d\n", rv);
		goto out;
	}

	sram_buffer_size = CONFIG_MTD_STM32F4_SRAM_BUFFER_SIZE;
	sram_buffer_start = (u32)stm32_sram_data_alloc(sram_buffer_size);
	if (!sram_buffer_start) {
		printk(KERN_ERR "SRAM alloc err\n");
		rv = -ENOMEM;
		goto out;
	}

	rv = 0;
	printk(KERN_INFO "Using SRAM as buffer with start %lx and size %lx\n",
		sram_buffer_start, sram_buffer_size);
out:
	return rv;
}

/******************************************************************************
 * Driver API
 ******************************************************************************/
static int of_flash_remove(struct platform_device *dev)
{
	struct of_flash *info;
	int i;

	info = dev_get_drvdata(&dev->dev);
	if (!info)
		return 0;
	dev_set_drvdata(&dev->dev, NULL);

	if (info->cmtd) {
		mtd_device_unregister(info->cmtd);
		if (info->cmtd != info->list[0].mtd)
			mtd_concat_destroy(info->cmtd);
	}

	for (i = 0; i < info->list_size; i++) {
		if (info->list[i].mtd)
			map_destroy(info->list[i].mtd);

		if (info->list[i].map.virt)
			iounmap(info->list[i].map.virt);

		if (info->list[i].res) {
			release_resource(info->list[i].res);
			kfree(info->list[i].res);
		}
	}
	return 0;
}

static const char * const rom_probe_types[] = {
	"cfi_probe", "jedec_probe", "map_rom" };

/*
 * Helper function to handle probing of the obsolete "direct-mapped"
 * compatible binding.
 */
static struct mtd_info *obsolete_probe(struct platform_device *dev,
				       struct map_info *map)
{
	struct device_node *dp = dev->dev.of_node;
	const char *of_probe;
	struct mtd_info *mtd;
	int i;

	of_probe = of_get_property(dp, "probe-type", NULL);
	if (!of_probe) {
		for (i = 0; i < ARRAY_SIZE(rom_probe_types); i++) {
			mtd = do_map_probe(rom_probe_types[i], map);
			if (mtd)
				return mtd;
		}
		return NULL;
	} else if (strcmp(of_probe, "CFI") == 0) {
		return do_map_probe("cfi_probe", map);
	} else if (strcmp(of_probe, "JEDEC") == 0) {
		return do_map_probe("jedec_probe", map);
	} else {
		if (strcmp(of_probe, "ROM") != 0)
			dev_warn(&dev->dev, "obsolete_probe: don't know probe "
				 "type '%s', mapping as rom\n", of_probe);
		return do_map_probe("map_rom", map);
	}
}

/*
 * When partitions are set we look for a linux,part-probe property which
 * specifies the list of partition probers to use. If none is given then the
 * default is use. These take precedence over other device tree
 * information.
 */
static const char * const part_probe_types_def[] = {
	"cmdlinepart", "RedBoot", "ofpart", "ofoldpart", NULL };

static const char * const *of_get_probes(struct device_node *dp)
{
	const char *cp;
	int cplen;
	unsigned int l;
	unsigned int count;
	const char **res;

	cp = of_get_property(dp, "linux,part-probe", &cplen);
	if (cp == NULL)
		return part_probe_types_def;

	count = 0;
	for (l = 0; l != cplen; l++)
		if (cp[l] == 0)
			count++;

	res = kzalloc((count + 1)*sizeof(*res), GFP_KERNEL);
	count = 0;
	while (cplen > 0) {
		res[count] = cp;
		l = strlen(cp) + 1;
		cp += l;
		cplen -= l;
		count++;
	}
	return res;
}

static void of_free_probes(const char * const *probes)
{
	if (probes != part_probe_types_def)
		kfree(probes);
}

static const struct of_device_id of_flash_match[];
static int of_flash_probe(struct platform_device *dev)
{
	const char * const *part_probe_types;
	const struct of_device_id *match;
	struct device_node *dp = dev->dev.of_node;
	struct resource res;
	struct of_flash *info;
	const __be32 *width;
	int err;
	int i;
	int count;
	const __be32 *p;
	int reg_tuple_size;
	struct mtd_info **mtd_list = NULL;
	resource_size_t res_size;
	struct mtd_part_parser_data ppdata;
	const char *mtd_name = NULL;

	match = of_match_device(of_flash_match, &dev->dev);
	if (!match)
		return -EINVAL;

	reg_tuple_size = (of_n_addr_cells(dp) + of_n_size_cells(dp)) * sizeof(u32);

	of_property_read_string(dp, "linux,mtd-name", &mtd_name);

	/*
	 * Get number of "reg" tuples. Scan for MTD devices on area's
	 * described by each "reg" region. This makes it possible (including
	 * the concat support) to support the Intel P30 48F4400 chips which
	 * consists internally of 2 non-identical NOR chips on one die.
	 */
	p = of_get_property(dp, "reg", &count);
	if (count % reg_tuple_size != 0) {
		dev_err(&dev->dev, "Malformed reg property on %s\n",
				dev->dev.of_node->full_name);
		err = -EINVAL;
		goto err_flash_remove;
	}
	count /= reg_tuple_size;

	err = -ENOMEM;
	info = devm_kzalloc(&dev->dev,
			    sizeof(struct of_flash) +
			    sizeof(struct of_flash_list) * count, GFP_KERNEL);
	if (!info)
		goto err_flash_remove;

	dev_set_drvdata(&dev->dev, info);

	mtd_list = kzalloc(sizeof(*mtd_list) * count, GFP_KERNEL);
	if (!mtd_list)
		goto err_flash_remove;

	/*
	 * Copy helpers to SRAM
	 */
	stm32f4_flash_init();

	for (i = 0; i < count; i++) {
		err = -ENXIO;
		if (of_address_to_resource(dp, i, &res)) {
			/*
			 * Continue with next register tuple if this
			 * one is not mappable
			 */
			continue;
		}

		dev_dbg(&dev->dev, "of_flash device: %pR\n", &res);

		err = -EBUSY;
		res_size = resource_size(&res);
		info->list[i].res = request_mem_region(res.start, res_size,
						       dev_name(&dev->dev));
		if (!info->list[i].res)
			goto err_out;

		err = -ENXIO;
		width = of_get_property(dp, "bank-width", NULL);
		if (!width) {
			dev_err(&dev->dev, "Can't get bank width from device"
				" tree\n");
			goto err_out;
		}

		info->list[i].map.name = mtd_name ?: dev_name(&dev->dev);
		info->list[i].map.phys = res.start;
		info->list[i].map.size = res_size;
		info->list[i].map.bankwidth = be32_to_cpup(width);
		info->list[i].map.device_node = dp;

		err = -ENOMEM;
		info->list[i].map.virt = ioremap(info->list[i].map.phys,
						 info->list[i].map.size);
		if (!info->list[i].map.virt) {
			dev_err(&dev->dev, "Failed to ioremap() flash"
				" region\n");
			goto err_out;
		}

		info->list[i].map.read = stm32f4_flash_read;
		info->list[i].map.write = stm32f4_flash_write;
		info->list[i].map.copy_from = stm32f4_flash_copy_from;
		info->list[i].map.copy_to = stm32f4_flash_copy_to;

		info->list[i].mtd = obsolete_probe(dev,
						   &info->list[i].map);
		/* Fall back to mapping region as ROM */
		if (!info->list[i].mtd) {
			dev_warn(&dev->dev, "do_map_probe() failed\n");
			info->list[i].mtd = do_map_probe("map_rom",
							 &info->list[i].map);
		}
		mtd_list[i] = info->list[i].mtd;

		err = -ENXIO;
		if (!info->list[i].mtd) {
			dev_err(&dev->dev, "do_map_probe() failed\n");
			goto err_out;
		} else {
			info->list_size++;
		}
		info->list[i].mtd->owner = THIS_MODULE;
		info->list[i].mtd->dev.parent = &dev->dev;
	}

	err = 0;
	info->cmtd = NULL;
	if (info->list_size == 1) {
		info->cmtd = info->list[0].mtd;
	} else if (info->list_size > 1) {
		/*
		 * We detected multiple devices. Concatenate them together.
		 */
		info->cmtd = mtd_concat_create(mtd_list, info->list_size,
					       dev_name(&dev->dev));
	}
	if (info->cmtd == NULL)
		err = -ENXIO;

	if (err)
		goto err_out;

	ppdata.of_node = dp;
	part_probe_types = of_get_probes(dp);
	mtd_device_parse_register(info->cmtd, part_probe_types, &ppdata,
			NULL, 0);
	of_free_probes(part_probe_types);

	kfree(mtd_list);

	return 0;

err_out:
	kfree(mtd_list);
err_flash_remove:
	of_flash_remove(dev);

	return err;
}

static const struct of_device_id of_flash_match[] = {
	{ .compatible	= "stm32f4,nor", },
	{},
};
MODULE_DEVICE_TABLE(of, of_flash_match);

static struct platform_driver of_flash_driver = {
	.driver = {
		.name = "stm32f4-flash",
		.of_match_table = of_flash_match,
	},
	.probe		= of_flash_probe,
	.remove		= of_flash_remove,
};

module_platform_driver(of_flash_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pavel Boldin <paboldin@emcraft.com>");
MODULE_DESCRIPTION("STM32F4XX workaround for SDRAM/flash");
