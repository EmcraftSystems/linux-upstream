/*
 * (C) Copyright 2017
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

/*
 * STM32 Embedded Flash memory MTD driver
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

/******************************************************************************
 * Constants and macros
 ******************************************************************************/
/*
 * Define this to enable additional debug info
 */
#undef DEBUG
#ifdef DEBUG
# define dbg(arg...)		printk(arg)
#else
# define dbg(arg...)		do { } while (0)
#endif

/*
 * Erase/program timeout, in 10 usec
 */
#define STM_FLH_TOUT		500000	/* 5sec */

/*
 * Maximum number of different block areas supported
 */
#define STM_FLH_BLK_MAX		16

/*
 * ACR definitions
 */
#define STM_FLH_ACR_DCRST	(1 << 12)
#define STM_FLH_ACR_ICRST	(1 << 11)
#define STM_FLH_ACR_DCEN	(1 << 10)
#define STM_FLH_ACR_ICEN	(1 << 9)
#define STM_FLH_ACR_PRFTEN	(1 << 8)

/*
 * KEYR definitions
 */
#define STM_FLH_KEYR_KEY1	0x45670123
#define STM_FLH_KEYR_KEY2	0xCDEF89AB

/*
 * SR definitions
 */
#define STM_FLH_SR_BSY		(1 << 16)
#define STM_FLH_SR_RDERR	(1 << 8)
#define STM_FLH_SR_PGSERR	(1 << 7)
#define STM_FLH_SR_PGPERR	(1 << 6)
#define STM_FLH_SR_PGAERR	(1 << 5)
#define STM_FLH_SR_WRPERR	(1 << 4)
#define STM_FLH_SR_ERRS		(STM_FLH_SR_RDERR  |			 \
				 STM_FLH_SR_PGSERR | STM_FLH_SR_PGPERR | \
				 STM_FLH_SR_PGAERR | STM_FLH_SR_WRPERR)
#define STM_FLH_SR_OPERR	(1 << 1)
#define STM_FLH_SR_EOP		(1 << 0)

/*
 * CR definitions
 */
#define STM_FLH_CR_LOCK		(1 << 31)
#define STM_FLH_CR_ERRIE	(1 << 25)
#define STM_FLH_CR_EOPIE	(1 << 24)
#define STM_FLH_CR_STRT		(1 << 16)
#define STM_FLH_CR_PSIZE(x)	((x) << 8)
#define STM_FLH_CR_PSIZE_MSK	STM_FLH_CR_PSIZE(3)
#define STM_FLH_CR_PSIZE_32	STM_FLH_CR_PSIZE(2)
#define STM_FLH_CR_PSIZE_8	STM_FLH_CR_PSIZE(0)
#define STM_FLH_CR_SNB(x)	((x) << 3)
#define STM_FLH_CR_SER		(1 << 1)
#define STM_FLH_CR_PG		(1 << 0)

/******************************************************************************
 * C-types
 ******************************************************************************/
/*
 * Flash geometry.
 * Consider Flash as divided into array of regions (blk[]), each region consists
 * of sectors of the same size.
 */
struct stm_flh_geo {
	char		*name;
	int		num;
	u32		acr;		/* Acceleration features	      */
	u32		snbm;		/* SNB mask			      */
	struct stm_flh_blk {
		u32	snbs;		/* Start sector FLASH_CR[SNB]	      */
		u32	snbe;		/* End sector FLASH_CR[SNB]	      */
		u32	size;		/* Sector size			      */
		u32	base;		/* Area offset start address	      */
	} blk[STM_FLH_BLK_MAX];
};

/*
 * Flash register map
 */
struct stm_flh_reg {
	u32		acr;		/* Access control		      */
	u32		keyr;		/* Key				      */
	u32		optkeyr;	/* Option key			      */
	u32		sr;		/* Status			      */
	u32		cr;		/* Control			      */
	u32		optcr;		/* Option control		      */
	u32		optcr1;		/* Option control		      */
};

/*
 * Flash interface
 */
struct stm_flh_iface {
	volatile struct stm_flh_reg	*reg;	/* Control registers	      */
	struct stm_flh_geo		*geo;	/* Geometry descriptor	      */
	wait_queue_head_t		wait;
	struct mutex			mutex;
	int				status;

	int (*stm_prog)(struct mtd_info *mtd, unsigned long dst, size_t len,
			size_t *retlen, const u_char *buf);
};

/******************************************************************************
 * Local variables and function prototypes
 ******************************************************************************/

/*
 * Supported STM32 Flash geometries
 */
static struct stm_flh_geo stm_geo[] = {
	   {	/* STM32F427xx, STM32F429xx 2MB: */
		.name	= "stm32f42x.2M",
		.snbm	= 0x1F,
		.acr	= STM_FLH_ACR_DCEN |
			  STM_FLH_ACR_ICEN | STM_FLH_ACR_PRFTEN,
		.blk	= {
			{  0,  3,  SZ_16K, 0x000000 },
			{  4,  4,  SZ_64K, 0x010000 },
			{  5, 11, SZ_128K, 0x020000 },
			{ 16, 19,  SZ_16K, 0x100000 },
			{ 20, 20,  SZ_64K, 0x110000 },
			{ 21, 27, SZ_128K, 0x120000 },
			{  0,  0,       0,        0 },
		},
	}, {	/* STM32F75xxx, STM32F74xxx 1MB: */
		.name	= "stm32f74x.1M",
		.snbm	= 0xF,
		.acr	= STM_FLH_ACR_ICEN | STM_FLH_ACR_PRFTEN,
		.blk	= {
			{  0,  3,  SZ_32K, 0x000000 },
			{  4,  4, SZ_128K, 0x020000 },
			{  5,  7, SZ_256K, 0x040000 },
			{  0,  0,       0,        0 },
		},
	}, {	/* STM32F7x0 64KB: */
		.name	= "stm32f7x0.64K",
		.snbm	= 0xF,
		.acr	= STM_FLH_ACR_DCEN |
			  STM_FLH_ACR_ICEN | STM_FLH_ACR_PRFTEN,
		.blk	= {
			{  0,  1,  SZ_32K, 0x000000 },
			{  0,  0,       0,        0 },
		},
	}, {	/* STM32F76xxx, STM32F77xxx 2MB: */
		.name	= "stm32f76x.2M",
		.snbm	= 0xF,
		.acr	= STM_FLH_ACR_ICEN | STM_FLH_ACR_PRFTEN,
		.blk	= {
			{  0,  3,  SZ_32K, 0x000000 },
			{  4,  4, SZ_128K, 0x020000 },
			{  5, 11, SZ_256K, 0x040000 },
			{  0,  0,       0,        0 },
		},
	},
};

/******************************************************************************
 * Functions local to this module
 ******************************************************************************/
/*
 * Unlock the Flash Command Register
 */
static inline void stm_cr_unlock(struct stm_flh_iface *iface)
{
	if (iface->reg->cr & STM_FLH_CR_LOCK) {
		iface->reg->keyr = STM_FLH_KEYR_KEY1;
		iface->reg->keyr = STM_FLH_KEYR_KEY2;
	}
}

/*
 * Lock the Flash Command Register
 */
static inline void stm_cr_lock(struct stm_flh_iface *iface)
{
	iface->reg->cr |= STM_FLH_CR_LOCK;
}

/*
 * Get the FLASH_CR[SNB] for sector which holds the address specified, and
 * the block descriptor
 */
static struct stm_flh_blk *stm_blk_get(struct stm_flh_iface *iface, u32 adr,
				       int *snb)
{
	struct stm_flh_blk *blk;
	int i = 0;

	while (i < iface->geo->num && adr >= iface->geo->blk[i].base)
		i++;

	blk = &iface->geo->blk[i - 1];
	if (snb)
		*snb = blk->snbs + ((adr - blk->base) / blk->size);

	return blk;
}

/*
 * Wait for completion with <tout> x10usec timeout
 */
static inline int stm_op_wait(struct stm_flh_iface *iface, u32 tout)
{
	int i, rv;

	/*
	 * Using a polling scheme in programming case allows to avoid
	 * per-byte/per-word interrupt overheads
	 */
	if (iface->reg->cr & STM_FLH_CR_PG) {
		udelay(1);
		for (i = 0; i < tout; i++) {
			if (!(iface->reg->sr & STM_FLH_SR_BSY)) {
				iface->status = 0;
				break;
			}
			udelay(10);
		}
		rv = (i != tout) ? 1 : 0;
	} else {
		tout /= 100000;
		rv = wait_event_interruptible_timeout(iface->wait,
					iface->status != -EBUSY, tout * HZ);
	}
	if (!rv)
		rv = -ETIMEDOUT;
	else if (iface->status)
		rv = -EIO;
	else if (!iface->status && (iface->reg->sr & STM_FLH_SR_BSY))
		rv = -EBUSY;
	else
		rv = 0;

	return rv;
}

/*
 * Program with `x32` parallelism
 */
static int stm_prog_x32(struct mtd_info *mtd, unsigned long dst, size_t len,
			size_t *retlen, const u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct stm_flh_iface *iface = map->fldrv_priv;
	unsigned long adst;
	u32 val, ofs, rem;
	int rv;

	rem = dst % sizeof(u32);
	if (!rem)
		goto middle;

	/*
	 * Unaligned head
	 */
	ofs = (dst % sizeof(u32)) << 3;
	adst = dst & ~(sizeof(u32) - 1);
	val = *(u32 *)(map->virt + adst);
	while ((dst % sizeof(u32)) && len) {
		val &= ~(0xFF << ofs);
		val |= *buf << ofs;
		ofs += sizeof(u8) << 3;
		buf += sizeof(u8);
		dst += sizeof(u8);
		len -= sizeof(u8);
	}

	iface->status = -EBUSY;
	*(u32 *)(map->virt + adst) = val;
	rv = stm_op_wait(iface, STM_FLH_TOUT);
	if (rv)
		goto out;
	*retlen += sizeof(u32) - rem;

middle:
	/*
	 * Word aligned part
	 */
	while (len >= sizeof(u32)) {
		iface->status = -EBUSY;
		*(u32 *)(map->virt + dst) = *(u32 *)buf;
		rv = stm_op_wait(iface, STM_FLH_TOUT);
		if (rv)
			goto out;
		*retlen += sizeof(u32);

		buf += sizeof(u32);
		dst += sizeof(u32);
		len -= sizeof(u32);
	}

	rem = len;
	if (!rem)
		goto done;

	/*
	 * Unaligned tail
	 */
	ofs = 0;
	adst = dst;
	val = *(u32 *)(map->virt + adst);
	while (len) {
		val &= ~(0xFF << ofs);
		val |= *buf << ofs;
		ofs += sizeof(u8) << 3;
		buf += sizeof(u8);
		dst += sizeof(u8);
		len -= sizeof(u8);
	}

	iface->status = -EBUSY;
	*(u32 *)(map->virt + adst) = val;
	rv = stm_op_wait(iface, STM_FLH_TOUT);
	if (rv)
		goto out;
	*retlen += rem;
done:
	rv = 0;
out:
	return rv;
}

/*
 * Program with `x8` parallelism
 */
static int stm_prog_x8(struct mtd_info *mtd, unsigned long dst, size_t len,
		       size_t *retlen, const u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct stm_flh_iface *iface = map->fldrv_priv;
	int rv;

	while (len) {
		iface->status = -EBUSY;
		*(u8 *)(map->virt + dst) = *(u8 *)buf;
		rv = stm_op_wait(iface, STM_FLH_TOUT);
		if (rv)
			goto out;
		*retlen += sizeof(u8);

		buf += sizeof(u8);
		dst += sizeof(u8);
		len -= sizeof(u8);
	}

	rv = 0;
out:
	return rv;
}

/*
 * Interrupt handler
 */
static irqreturn_t stm_flh_irq(int irq, void *dev_id)
{
	struct stm_flh_iface *iface = dev_id;
	irqreturn_t rv = IRQ_HANDLED;
	u32 sr = iface->reg->sr;

	if (sr & STM_FLH_SR_EOP) {
		iface->reg->sr = STM_FLH_SR_EOP;
		iface->status = 0;
	} else if (sr & STM_FLH_SR_OPERR) {
		iface->reg->sr = STM_FLH_SR_ERRS | STM_FLH_SR_OPERR;
		iface->status = sr & (STM_FLH_SR_ERRS | STM_FLH_SR_OPERR);
		printk(KERN_ERR "%s: error detected.\n"
			"Regs: SR=%08x,CR=%08x,ACR=%08x,OPTCRx=%08x/%08x\n",
			__func__, sr, iface->reg->cr, iface->reg->acr,
			iface->reg->optcr, iface->reg->optcr1);
	} else {
		rv = IRQ_NONE;
		goto out;
	}

	wake_up(&iface->wait);
out:
	return rv;
}

/******************************************************************************
 * MTD driver API
 ******************************************************************************/

static int stm_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct map_info *map = mtd->priv;
	struct stm_flh_iface *iface = map->fldrv_priv;
	struct stm_flh_blk *blk;
	unsigned long adr, len;
	int rv, snb;

	mutex_lock(&iface->mutex);

	adr = instr->addr;
	len = instr->len;
	dbg("%s: 0x%lx @0x%lx\n", __func__, adr, len);

	if (iface->reg->sr & STM_FLH_SR_BSY) {
		printk(KERN_WARNING "%s: flash is busy [cr=%08x,sr=%08x]\n",
			__func__, iface->reg->cr, iface->reg->sr);
		instr->state = MTD_ERASE_FAILED;
		rv = -EBUSY;
		goto exit;
	}

	stm_cr_unlock(iface);
	iface->reg->cr |= STM_FLH_CR_SER;
	while (len) {
		blk = stm_blk_get(iface, adr, &snb);

		iface->status = -EBUSY;
		iface->reg->cr &= ~STM_FLH_CR_SNB(iface->geo->snbm);
		iface->reg->cr |= STM_FLH_CR_SNB(snb) | STM_FLH_CR_STRT;
		rv = stm_op_wait(iface, STM_FLH_TOUT);
		if (rv) {
			iface->reg->cr &= ~STM_FLH_CR_STRT;
			instr->state = MTD_ERASE_FAILED;
			goto out;
		}

		adr += blk->size;
		len -= blk->size;
	}

	instr->state = MTD_ERASE_DONE;
	rv = 0;
out:
	iface->reg->cr &= ~STM_FLH_CR_SER;
	stm_cr_lock(iface);

	/* Reset caches */
	if (iface->reg->acr & STM_FLH_ACR_DCEN) {
		iface->reg->acr &= ~STM_FLH_ACR_DCEN;
		iface->reg->acr |= STM_FLH_ACR_DCRST;
		iface->reg->acr |= STM_FLH_ACR_DCEN;
	}
	if (iface->reg->acr & STM_FLH_ACR_ICEN) {
		iface->reg->acr &= ~STM_FLH_ACR_ICEN;
		iface->reg->acr |= STM_FLH_ACR_ICRST;
		iface->reg->acr |= STM_FLH_ACR_ICEN;
	}
exit:
	mutex_unlock(&iface->mutex);

	if (rv)
		printk(KERN_ERR "%s: fail @%lx\n", __func__, adr);

	mtd_erase_callback(instr);

	return rv;
}

static int stm_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
			size_t *retlen, u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct stm_flh_iface *iface = map->fldrv_priv;
	int rv;

	dbg("%s: 0x%x +0x%lx->%p\n", __func__, len, (unsigned long)from, buf);

	mutex_lock(&iface->mutex);
	if (iface->reg->sr & STM_FLH_SR_BSY) {
		printk(KERN_WARNING "%s: flash is busy [cr=%08x,sr=%08x]\n",
			__func__, iface->reg->cr, iface->reg->sr);
		rv = -EBUSY;
		goto out;
	}

	map_copy_from(map, buf, from, len);
	*retlen = len;
	rv = 0;
out:
	mutex_unlock(&iface->mutex);

	return rv;
}

static int stm_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
			 size_t *retlen, const u_char *buf)
{
	struct map_info *map = mtd->priv;
	struct stm_flh_iface *iface = map->fldrv_priv;
	unsigned long dst;
	int rv;

	mutex_lock(&iface->mutex);

	dst = to;
	dbg("%s: 0x%x %p->+0x%lx\n", __func__, len, buf, dst);

	if (!iface->stm_prog) {
		printk(KERN_ERR "%s: no stm_prog() set\n", __func__);
		rv = -EINVAL;
		goto exit;
	}

	if (iface->reg->sr & STM_FLH_SR_BSY) {
		printk(KERN_WARNING "%s: flash is busy [cr=%08x,sr=%08x]\n",
			__func__, iface->reg->cr, iface->reg->sr);
		rv = -EBUSY;
		goto exit;
	}

	stm_cr_unlock(iface);
	iface->reg->cr &= ~STM_FLH_CR_EOPIE;
	iface->reg->cr |= STM_FLH_CR_PG;

	rv = iface->stm_prog(mtd, dst, len, retlen, buf);

	iface->reg->cr &= ~STM_FLH_CR_PG;
	iface->reg->cr |= STM_FLH_CR_EOPIE;
	stm_cr_lock(iface);
exit:
	mutex_unlock(&iface->mutex);

	if (rv)
		printk(KERN_ERR "%s: fail @%lx\n", __func__, dst);

	return rv;
}

/******************************************************************************
 * Kernel module interface
 ******************************************************************************/

static struct mtd_info *stm_chipdrv_probe(struct map_info *map)
{
	struct mtd_info *mtd = NULL;
	struct stm_flh_iface *iface;
	struct stm_flh_geo *geo;
	struct stm_flh_blk *blk;
	struct device_node *np;
	const char *geo_name = NULL;
	unsigned long flags;
	volatile struct stm_flh_reg *reg = NULL;
	const u32 *p;
	void *stm_prog = NULL;
	int i, rv, irq;

	/*
	 * Check params
	 */
	p = of_get_property(map->device_node, "flash-controller", NULL);
	np = p ? of_find_node_by_phandle(be32_to_cpup(p)) : NULL;
	reg = np ? of_iomap(np, 0) : NULL;
	if (!reg) {
		printk(KERN_ERR "%s: failed map reg\n", __func__);
		rv = -EINVAL;
		goto done;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (irq < 0) {
		printk(KERN_ERR "%s: failed get irq\n", __func__);
		rv = -EINVAL;
		goto done;
	}

	switch (reg->cr & STM_FLH_CR_PSIZE_MSK) {
	case STM_FLH_CR_PSIZE_32:
		stm_prog = stm_prog_x32;
		break;
	case STM_FLH_CR_PSIZE_8:
		stm_prog = stm_prog_x8;
		break;
	default:
		printk(KERN_ERR "%s: CR=x%08x parallelism isn't supported\n",
			__func__, reg->cr);
		rv = -EFAULT;
		goto done;
	}

	if (of_property_read_string(map->device_node, "geometry", &geo_name)) {
		printk(KERN_ERR "%s: no `geometry` specified\n", __func__);
		rv = -EINVAL;
		goto done;
	}
	for (i = 0, geo = NULL; i < ARRAY_SIZE(stm_geo); i++) {
		if (!strcmp(geo_name, stm_geo[i].name)) {
			geo = &stm_geo[i];
			break;
		}
	}
	if (!geo) {
		printk(KERN_ERR "%s: geometry `%s` not supported\n",
			__func__, geo_name);
		rv = -EINVAL;
		goto done;
	}

	if (geo->acr & STM_FLH_ACR_DCEN) {
		reg->acr &= ~STM_FLH_ACR_DCEN;
		reg->acr |= STM_FLH_ACR_DCRST;
		reg->acr |= STM_FLH_ACR_DCEN;
	}
	if (geo->acr & STM_FLH_ACR_ICEN) {
		reg->acr &= ~STM_FLH_ACR_ICEN;
		reg->acr |= STM_FLH_ACR_ICRST;
		reg->acr |= STM_FLH_ACR_ICEN;
	}
	if (geo->acr & STM_FLH_ACR_PRFTEN)
		reg->acr |= STM_FLH_ACR_PRFTEN;

	/*
	 * Complete geometry descriptor initialization
	 */
	if (!geo->num) {
		for (i = 0; i < STM_FLH_BLK_MAX; i++)
			if (!geo->blk[i].size)
				break;

		geo->num = i;
	}

	/*
	 * Create mtd info
	 */
	mtd = kzalloc(sizeof(struct mtd_info), GFP_KERNEL);
	if (!mtd) {
		printk(KERN_ERR "%s: mtd info alloc failure\n", __func__);
		rv = -ENOMEM;
		goto done;
	}

	mtd->priv = map;
	mtd->type = MTD_NORFLASH;

	mtd->_erase = stm_mtd_erase;
	mtd->_read  = stm_mtd_read;
	mtd->_write = stm_mtd_write;

	mtd->name = map->name;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->writesize = 1;

	/*
	 * Setup mtd
	 */
	mtd->numeraseregions = geo->num;
	mtd->eraseregions = kmalloc(sizeof(struct mtd_erase_region_info) *
				    geo->num, GFP_KERNEL);
	if (!mtd->eraseregions) {
		printk(KERN_ERR "%s: %d erase regions alloc failure\n",
			__func__, geo->num);
		rv = -ENOMEM;
		goto err_mtd;
	}

	for (i = 0; i < mtd->numeraseregions; i++) {
		struct mtd_erase_region_info *rgn;

		rgn = &mtd->eraseregions[i];
		blk = &geo->blk[i];

		rgn->offset = blk->base;
		rgn->erasesize = blk->size;
		rgn->numblocks = blk->snbe - blk->snbs + 1;

		if (mtd->erasesize < rgn->erasesize)
			mtd->erasesize = rgn->erasesize;
		mtd->size += rgn->numblocks * rgn->erasesize;
	}

	iface = kmalloc(sizeof(struct stm_flh_iface), GFP_KERNEL);
	if (!iface) {
		printk(KERN_ERR "%s: private alloc failure\n", __func__);
		rv = -ENOMEM;
		goto err_regions;
	}

	map->fldrv_priv = iface;
	iface->geo = geo;
	iface->reg = reg;
	iface->stm_prog = stm_prog;
	mutex_init(&iface->mutex);
	init_waitqueue_head(&iface->wait);

	/*
	 * Request and enable IRQs
	 */
	rv = request_irq(irq, stm_flh_irq, 0, np->name, iface);
	if (rv) {
		printk(KERN_ERR "%s: irq(%d) request fail\n", __func__, irq);
		goto err_iface;
	}

	local_irq_save(flags);
	stm_cr_unlock(iface);
	iface->reg->sr = (u32)-1;
	iface->reg->cr |= STM_FLH_CR_ERRIE | STM_FLH_CR_EOPIE;
	stm_cr_lock(iface);
	local_irq_restore(flags);

	__module_get(THIS_MODULE);

	rv = 0;
	goto done;
err_iface:
	kfree(iface);
err_regions:
	kfree(mtd->eraseregions);
err_mtd:
	kfree(mtd);
	mtd = NULL;
done:
	dbg("%s: %d (reg=%p,mem=%p,geo=`%s`)"
	    "(acr=%08x,csr=%08x/%08x,optcr=%08x/%08x)\n",
	    __func__, rv, reg, map->virt, geo_name,
	    reg ? reg->acr : 0,
	    reg ? reg->cr : 0, reg ? reg->sr : 0,
	    reg ? reg->optcr : 0, reg ? reg->optcr1 : 0);

	return mtd;
}

static struct mtd_chip_driver stm_chipdrv = {
	.probe	= stm_chipdrv_probe,
	.name	= "stm_probe",
	.module	= THIS_MODULE
};

static int __init stm_flash_init(void)
{
	dbg("%s\n", __func__);
	register_mtd_chip_driver(&stm_chipdrv);

	return 0;
}

static void __exit stm_flash_exit(void)
{
	dbg("%s\n", __func__);
	unregister_mtd_chip_driver(&stm_chipdrv);
}

module_init(stm_flash_init);
module_exit(stm_flash_exit);

MODULE_DESCRIPTION("STM32 Embedded Flash memory MTD driver");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL v2");

