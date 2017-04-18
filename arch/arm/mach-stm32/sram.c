/*
 * arch/arm/mach-stm32/sram.c
 *
 * STM32 On-Chip SRAMs Management
 *
 * Copyright (C) 2017 Emcraft Systems
 * Yuri Tikhonov, Emcraft Systems, <yur@emcraft.com>
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

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>

#include <asm/io.h>

#include <mach/sram.h>
#include <mach/pwr.h>

/******************************************************************************
 * Linker symbols
 ******************************************************************************/

extern char vector_table_end, vector_table;
extern char _sram_start, _sram_end;
extern char __sram_loc;

/******************************************************************************
 * Variables local to this module
 ******************************************************************************/

/*
 * Allocatable Backup SRAM
 */
static void *bkpsram_base;
static size_t bkpsram_size;
static int bkpsram_inited;

/*
 * Allocatable SRAM
 */
static void *sram_base;
static size_t sram_size;
static int sram_inited;

/******************************************************************************
 * Backup SRAM API
 ******************************************************************************/

/*
 * Init Backup SRAM
 */
int stm32_bkpsram_init(void)
{
	struct device_node *np;
	struct resource res;
	unsigned long flags;
	struct clk *clk;
	void *regs;
	int rv;

	local_irq_save(flags);
	if (bkpsram_inited) {
		rv = 0;
		goto out;
	}

	/*
	 * Get params, and init backup SRAM
	 */
	np = of_find_compatible_node(NULL, NULL, "st,stm32-pwr");
	if (!np) {
		pr_err("%s: no pwr info found\n", __func__);
		rv = -EINVAL;
		goto out;
	}
	clk = of_clk_get(np, 0);
	if (!clk) {
		pr_err("%s: no pwr clk found\n", __func__);
		rv = -EINVAL;
		goto out;
	}
	clk_prepare_enable(clk);
	regs = of_iomap(np, 0);
	of_node_put(np);
	writel(STM32_PWR_CR_DBP | readl(regs + STM32_PWR_CR),
		regs + STM32_PWR_CR);
	writel(STM32_PWR_CSR_BRE | readl(regs + STM32_PWR_CSR),
		regs + STM32_PWR_CSR);

	np = of_find_compatible_node(NULL, NULL, "st,stm32-bkpsram");
	if (!np) {
		pr_err("%s: no bkpsram info found\n", __func__);
		rv = -EINVAL;
		goto out;
	}
	clk = of_clk_get(np, 0);
	if (!clk) {
		pr_err("%s: no bkpsram clk found\n", __func__);
		rv = -EINVAL;
		goto out;
	}
	clk_prepare_enable(clk);

	of_address_to_resource(np, 0, &res);
	bkpsram_base = of_iomap(np, 0);
	bkpsram_size = res.end - res.start + 1;

	of_node_put(np);
	bkpsram_inited = 1;
	rv = 0;
out:
	local_irq_restore(flags);
	return rv;
}

/*
 * Allocate static bufer from Backup SRAM
 */
void *stm32_bkpsram_data_alloc(size_t size)
{
	static u32	used;

	unsigned long flags;
	unsigned char *p;

	local_irq_save(flags);

	if (!bkpsram_inited) {
		printk("%s: BKPSRAM not inited\n", __func__);
		p = NULL;
		goto out;
	}

	if (used + size > bkpsram_size) {
		printk("%s: failed alloc +%dB (%d of %d are in use)\n",
			__func__, size, used, bkpsram_size);
		p = NULL;
		goto out;
	}

	p = bkpsram_base + used;
	used += size;
out:
	local_irq_restore(flags);

	return p;
}

/******************************************************************************
 * SRAM API
 ******************************************************************************/

/*
 * Init SRAM
 */
int stm32_sram_init(void)
{
	struct device_node *np;
	struct resource res;
	unsigned long adr, flags;
	struct clk *clk;
	int rv;

	local_irq_save(flags);

	if (sram_inited) {
		rv = 0;
		goto out;
	}

	np = of_find_compatible_node(NULL, NULL, "st,stm32-sram");
	if (!np) {
		pr_err("%s: no sram info found\n", __func__);
		rv = -EINVAL;
		goto out;
	}

	clk = of_clk_get(np, 0);
	if (clk)
		clk_prepare_enable(clk);

	of_address_to_resource(np, 0, &res);

	adr = (unsigned long)of_iomap(np, 0);
	adr += &vector_table_end - &vector_table;
	adr += &_sram_end - &_sram_start;
	adr = ALIGN(adr, 0x100);
	sram_base = (void *)adr;

	sram_size = res.end - res.start + 1;
	sram_size -= (u32)sram_base - res.start;

	of_node_put(np);
	sram_inited = 1;
	rv = 0;
out:
	local_irq_restore(flags);
	return rv;
}

/*
 * Relocate SRAM_TEXT & SRAM_DATA from external SDRAM to internal SRAM
 */
int stm32_sram_relocate(void)
{
	static int	relocated;

	unsigned long flags;
	int rv;

	local_irq_save(flags);
	if (!sram_inited) {
		printk("%s: SRAM not inited\n", __func__);
		rv = -EFAULT;
		goto out;
	}

	if (relocated) {
		rv = 0;
		goto out;
	}

	memcpy((void*)&_sram_start, (void*)&__sram_loc,
		&_sram_end - &_sram_start);
	relocated = 1;
	rv = 0;
out:
	local_irq_restore(flags);
	return rv;
}

/*
 * Allocate static buffer from SRAM
 */
void *stm32_sram_data_alloc(size_t size)
{
	static u32	used;

	unsigned long flags;
	unsigned char *p;

	local_irq_save(flags);

	if (!sram_inited) {
		printk("%s: BKPSRAM not inited\n", __func__);
		p = NULL;
		goto out;
	}

	if (used + size > sram_size) {
		printk("%s: failed alloc +%dB (%d of %d are in use)\n",
			__func__, size, used, sram_size);
		p = NULL;
		goto out;
	}

	p = sram_base + used;
	used += size;

out:
	local_irq_restore(flags);
	return p;
}
