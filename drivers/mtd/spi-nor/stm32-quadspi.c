/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define	DRV_NAME			"stm32-quadspi"

#define QSPI_CR_EN			(1 << 0)
#define QSPI_CR_ABORT			(1 << 1)
#define QSPI_CR_DMAEN			(1 << 2)
#define QSPI_CR_TCEN			(1 << 3)
#define QSPI_CR_SSHIFT			(1 << 4)
#define QSPI_CR_DFM			(1 << 6)
#define QSPI_CR_FSEL			(1 << 7)
#define QSPI_CR_TEIE			(1 << 16)
#define QSPI_CR_TCIE			(1 << 17)
#define QSPI_CR_FTIE			(1 << 18)
#define QSPI_CR_SMIE			(1 << 19)
#define QSPI_CR_TOIE			(1 << 20)
#define QSPI_CR_APMS			(1 << 22)
#define QSPI_CR_PMM			(1 << 23)
#define QSPI_CR_FTHRES(x)		(((x) & 0x1F) << 8)
#define QSPI_CR_PRESCALER(x)		(((x) & 0xFF) << 24)

#define QSPI_DCR_CK_MODE		(1 << 0)
#define QSPI_DCR_CSHT(x)		(((x) & 0x7) << 8)
#define QSPI_DCR_FSIZE(x)		(((x) & 0x1F) << 16)

#define QSPI_SR_TEF			(1 << 0)
#define QSPI_SR_TCF			(1 << 1)
#define QSPI_SR_FTF			(1 << 2)
#define QSPI_SR_SMF			(1 << 3)
#define QSPI_SR_TOF			(1 << 4)
#define QSPI_SR_BUSY			(1 << 5)
#define QSPI_SR_FLEVEL(x)		(((x) >> 8) & 0x3F)

#define QSPI_FCR_CTEF			(1 << 0)
#define QSPI_FCR_CTCF			(1 << 1)
#define QSPI_FCR_CSMF			(1 << 3)
#define QSPI_FCR_CTOF			(1 << 4)

#define QSPI_CCR_INSTRUCTION(x)		(((x) & 0xFF) << 0)
#define QSPI_CCR_IMODE(x)		(((x) & 0x3) << 8)
#define QSPI_CCR_IMODE_NONE		(QSPI_CCR_IMODE(0))
#define QSPI_CCR_IMODE_SINGLE_LINE	(QSPI_CCR_IMODE(1))
#define QSPI_CCR_IMODE_TWO_LINES	(QSPI_CCR_IMODE(2))
#define QSPI_CCR_IMODE_FOUR_LINES	(QSPI_CCR_IMODE(3))
#define QSPI_CCR_ADMODE(x)		(((x) & 0x3) << 10)
#define QSPI_CCR_ADMODE_NONE		(QSPI_CCR_ADMODE(0))
#define QSPI_CCR_ADMODE_SINGLE_LINE	(QSPI_CCR_ADMODE(1))
#define QSPI_CCR_ADMODE_TWO_LINES	(QSPI_CCR_ADMODE(2))
#define QSPI_CCR_ADMODE_FOUR_LINES	(QSPI_CCR_ADMODE(3))
#define QSPI_CCR_ADSIZE(x)		(((x) & 0x3) << 12)
#define QSPI_CCR_ADSIZE_ONE_BYTE	(QSPI_CCR_ADSIZE(0))
#define QSPI_CCR_ADSIZE_TWO_BYTES	(QSPI_CCR_ADSIZE(1))
#define QSPI_CCR_ADSIZE_THREE_BYTES	(QSPI_CCR_ADSIZE(2))
#define QSPI_CCR_ADSIZE_FOUR_BYTES	(QSPI_CCR_ADSIZE(3))
#define QSPI_CCR_ABMODE(x)		(((x) & 0x3) << 14)
#define QSPI_CCR_ABSIZE(x)		(((x) & 0x3) << 16)
#define QSPI_CCR_DCYC(x)		(((x) & 0x1F) << 18)
#define QSPI_CCR_DMODE(x)		(((x) & 0x3) << 24)
#define QSPI_CCR_DMODE_NONE		(QSPI_CCR_DMODE(0))
#define QSPI_CCR_DMODE_SINGLE_LINE	(QSPI_CCR_DMODE(1))
#define QSPI_CCR_DMODE_TWO_LINES	(QSPI_CCR_DMODE(2))
#define QSPI_CCR_DMODE_FOUR_LINES	(QSPI_CCR_DMODE(3))
#define QSPI_CCR_FMODE(x)		(((x) & 0x3) << 26)
#define QSPI_CCR_FMODE_INDIRECT_WRITE	(QSPI_CCR_FMODE(0))
#define QSPI_CCR_FMODE_INDIRECT_READ	(QSPI_CCR_FMODE(1))
#define QSPI_CCR_FMODE_AUTO_POLL	(QSPI_CCR_FMODE(2))
#define QSPI_CCR_FMODE_MEMORY_MAP	(QSPI_CCR_FMODE(3))
#define QSPI_CCR_SIOO			(1 << 28)
#define QSPI_CCR_DHHC			(1 << 30)
#define QSPI_CCR_DDRM			(1 << 31)

#define STM32_QSPI_BASE			(STM32_AHB3PERIPH_BASE + 0x1000)

#define QSPI_TIMEOUT_MS			1000
#define QSPI_POLLING_INTERVAL		16

#define STM32_DUMMY_TABLE_MAX_SIZE	16

struct stm32_qspi_regs {
	u32	cr;
	u32	dcr;
	u32	sr;
	u32	fcr;
	u32	dlr;
	u32	ccr;
	u32	ar;
	u32	abr;
	u32	dr;
	u32	psmkr;
	u32	psmar;
	u32	pir;
	u32	lptr;
};

struct dummy_cycles_table {
	unsigned	freq;
	unsigned	dummy_cycles;
};

struct flash_info {
	const char			*name;
	size_t				device_size;
	size_t				write_size;
	size_t				erase_size;
	u32				program_cmd;
	struct dummy_cycles_table	dummy_cycles_table[STM32_DUMMY_TABLE_MAX_SIZE];
};

static const struct flash_info stm32_qspiflash_info[] = {
	{
		/* 2^26 = 64MiB */
		"mt25ql512abb", 1 << 26, 256, 64 * 1024, SPINOR_OP_FAST_PROGRAM,
		{
			{44000000,	2},
			{61000000,	3},
			{78000000,	4},
			{97000000,	5},
			{106000000,	6},
			{115000000,	7},
			{125000000,	8},
			{133000000,	9},
			{0, 0 /* sentinel */},
		},
	},
	{
		/* 2^24 = 16MiB */
		"n25q128a", 1 << 24, 256, 64 * 1024, SPINOR_OP_PP_4B,
		{
			{30000000,	3},
			{40000000,	4},
			{50000000,	5},
			{60000000,	6},
			{70000000,	7},
			{80000000,	8},
			{86000000,	9},
			{95000000,	10},
			{0, 0 /* sentinel */},
		},
	},
};

struct stm32_qspi_priv {
	struct device		*dev;
	struct stm32_qspi_regs __iomem *regs;
	const struct flash_info *flash;
	u8			*mapped_area;
	struct clk		*clk;
	struct reset_control	*rst;
	unsigned long		freq;
	int			fast_read_dummy;
	struct mtd_info		mtd;
};

static int stm32_qspi_set_speed(struct stm32_qspi_priv *priv, unsigned long speed)
{
	unsigned long parent_rate = clk_get_rate(priv->clk);
	unsigned div = parent_rate / speed;
	unsigned long new_freq = parent_rate / div;

	if ((div > 256) || (div < 1)) {
		dev_err(priv->dev, "%s: unable to provide a rate %lu based of parent rate %lu\n",
			__func__, speed, parent_rate);
		return -1;
	}

	if (new_freq != speed) {
		dev_warn(priv->dev, "%s: set rate %lu instead of requested %lu due to integer division\n",
			__func__, speed, new_freq);
		priv->freq = new_freq;
	}

	priv->regs->cr &= ~QSPI_CR_PRESCALER(0xFF);
	priv->regs->cr |= QSPI_CR_PRESCALER(div - 1);

	return 0;
}

static int stm32_qspi_wait_for_status(struct stm32_qspi_priv *priv, u32 mask, int active)
{
	int i, timeout = QSPI_TIMEOUT_MS * 1000;
	u32 reg;

	for (i = 0; i < timeout; ++i) {
		u32 status;
		reg = readl(&priv->regs->sr);
		status = reg & mask;

		if ((active && status) || (!active && !status))
			break;

		udelay(1);
	}

	if (i == timeout)
		dev_err(priv->dev, "%s: TIMEOUT: mask 0x%x, active %d, status 0x%x\n",
		      __func__, mask, active, reg);

	return (i == timeout) ? -1 : 0;
}

static int stm32_qspi_wait_while_busy(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_wait_for_status(priv, QSPI_SR_BUSY, 0);
	return err;
}

static int stm32_qspi_wait_until_complete(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_wait_for_status(priv, QSPI_SR_TCF, 1);
	if (!err)
		priv->regs->fcr |= QSPI_FCR_CTCF;
	else
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);

	return err;
}

static int stm32_qspi_wait_until_match(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_wait_for_status(priv, QSPI_SR_SMF, 1);
	if (!err)
		priv->regs->fcr |= QSPI_FCR_CSMF;
	else
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_wait_for_fifo(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_wait_for_status(priv, QSPI_SR_FTF, 1);
	if (err)
		dev_err(priv->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_qspi_autopoll(struct stm32_qspi_priv *priv, u32 mask, u32 match)
{
	int err;

	writel(match, &priv->regs->psmar);
	writel(mask, &priv->regs->psmkr);
	writel(0x10, &priv->regs->pir);
	writel(1 - 1, &priv->regs->dlr);

	priv->regs->cr &= ~QSPI_CR_PMM;
	priv->regs->cr |= QSPI_CR_APMS;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		return err;

	writel(QSPI_CCR_FMODE_AUTO_POLL
	       | SPINOR_OP_RDSR
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_NONE
	       | QSPI_CCR_DMODE_SINGLE_LINE
	       | QSPI_CCR_DCYC(0),
	       &priv->regs->ccr);

	err = stm32_qspi_wait_until_match(priv);
	if (err)
		return err;

	priv->regs->cr &= ~(QSPI_CR_PMM | QSPI_CR_APMS);

	dev_dbg(priv->dev, "%s: status 0x%x\n", __func__, readl(&priv->regs->dr));
	return 0;
}

static int stm32_qspi_wait_while_writing(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_wait_while_busy(priv);
	if (err)
		return err;

	return stm32_qspi_autopoll(priv, SR_WIP, 0);
}

static int stm32_qspi_wait_until_write_enable(struct stm32_qspi_priv *priv)
{
	int err = stm32_qspi_wait_while_busy(priv);
	if (err)
		return err;

	return stm32_qspi_autopoll(priv, SR_WEL, SR_WEL);
}

static int stm32_qspi_write_enable(struct stm32_qspi_priv *priv)
{
	int err;

	stm32_qspi_wait_while_busy(priv);

	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | SPINOR_OP_WREN
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_NONE
	       | QSPI_CCR_DMODE_NONE
	       | QSPI_CCR_DCYC(0),
	       &priv->regs->ccr);

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		return err;

	return stm32_qspi_wait_until_write_enable(priv);
}

static int stm32_qspi_switch_to_memory_mapped(struct stm32_qspi_priv *priv)
{
	int err;

	priv->regs->cr |= QSPI_CR_DMAEN;

	err = stm32_qspi_wait_while_busy(priv);
	if (err) {
		dev_err(priv->dev, "%s[%d]: internal error\n", __func__, __LINE__);
		return err;
	}

	writel(QSPI_CCR_FMODE_MEMORY_MAP
	       | SPINOR_OP_FAST_READ
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_FOUR_LINES
	       | QSPI_CCR_ADSIZE_THREE_BYTES
	       | QSPI_CCR_DMODE_FOUR_LINES
	       | QSPI_CCR_DCYC(priv->fast_read_dummy),
	       &priv->regs->ccr);

	return 0;
}

static int stm32_qspi_erase_block(struct stm32_qspi_priv *priv, u32 address)
{
	int err;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		return err;

	err = stm32_qspi_write_enable(priv);
	if (err)
		return err;

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		return err;

	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | SPINOR_OP_SE
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_SINGLE_LINE
	       | QSPI_CCR_ADSIZE_THREE_BYTES
	       | QSPI_CCR_DMODE_NONE
	       | QSPI_CCR_DCYC(0),
		&priv->regs->ccr);

	writel(address, &priv->regs->ar);

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		return err;

	return stm32_qspi_wait_while_writing(priv);
}

static int stm32_qspi_write_page(struct stm32_qspi_priv *priv, u32 address, const u8 *buf, size_t size)
{
	int err;

	if (size > priv->flash->write_size) {
		dev_err(priv->dev, "%s: size %u >  max write size %u\n",
		      __func__, size, priv->flash->write_size);
		return -EINVAL;
	}

	err = stm32_qspi_wait_while_busy(priv);
	if (err)
		return err;

	err = stm32_qspi_write_enable(priv);
	if (err)
		return err;
	stm32_qspi_wait_while_busy(priv);

	writel(size - 1, &priv->regs->dlr);

	writel(QSPI_CCR_FMODE_INDIRECT_WRITE
	       | priv->flash->program_cmd
	       | QSPI_CCR_IMODE_SINGLE_LINE
	       | QSPI_CCR_ADMODE_FOUR_LINES
	       | QSPI_CCR_ADSIZE_THREE_BYTES
	       | QSPI_CCR_DMODE_FOUR_LINES
	       | QSPI_CCR_DCYC(0),
		&priv->regs->ccr);

	writel(address, &priv->regs->ar);

	priv->regs->ccr &= ~(QSPI_CCR_FMODE(3));
	priv->regs->ccr |= QSPI_CCR_FMODE_INDIRECT_WRITE;

	while (size) {
		u8 *dr = (u8*)&priv->regs->dr;
		int err = stm32_qspi_wait_for_fifo(priv);
		if (err) {
			dev_err(priv->dev, "%s: write failed (fifo): %d\n", __func__, err);
			return err;
		}
		writeb(*buf++, dr);
		--size;
	}

	err = stm32_qspi_wait_until_complete(priv);
	if (err)
		return err;

	err = stm32_qspi_wait_while_writing(priv);
	if (err)
		return err;

	return 0;
}

static int stm32_qspi_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
			 size_t *retlen, const u_char *buf)
{
	struct stm32_qspi_priv *priv = mtd->priv;
	size_t write_size = priv->flash->write_size;
	size_t current_size;

	*retlen = 0;

	if (unlikely((to + len) > priv->flash->device_size)) {
		dev_err(priv->dev, "%s: Write past end of device\n", __func__);
		return -EINVAL;
	}

	current_size =
		(((to & (write_size - 1)) + len) > write_size)
		? (write_size - (to & (write_size - 1)))
		: len;

	while (len) {
		int err = stm32_qspi_write_page(priv, to, buf, current_size);
		if (err) {
			dev_err(priv->dev, "%s: write failed: %d\n", __func__, err);
			return err;
		}

		*retlen	+= current_size;
		len	-= current_size;
		to	+= current_size;
		buf	+= current_size;
		current_size = (len > write_size) ? write_size : len;
	}

	return stm32_qspi_switch_to_memory_mapped(priv);
}

static int stm32_qspi_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct stm32_qspi_priv *priv = mtd->priv;
	size_t erase_size = priv->flash->erase_size;
	unsigned long address = instr->addr;
	unsigned long len = instr->len;

	instr->state = MTD_ERASE_FAILED;

	if (unlikely((address + len) > priv->flash->device_size)) {
		dev_err(priv->dev, "%s: Erase past end of device\n", __func__);
		return -EINVAL;
	}

	if (len & (erase_size - 1)) {
		dev_err(priv->dev, "%s: non-block erase: len %lu\n", __func__, len);
		return -EINVAL;
	}

	if (address & (erase_size - 1)) {
		dev_err(priv->dev, "%s: non-aligned erase: addr 0x%x\n", __func__, (u32)address);
		return -EINVAL;
	}

	instr->state = MTD_ERASING;

	while (len) {
		int err = stm32_qspi_erase_block(priv, address);
		if (err) {
			dev_err(priv->dev, "%s[%d]: internal error\n", __func__, __LINE__);
			return err;
		}
		address	+= erase_size;
		len	-= erase_size;
	}

	instr->state = MTD_ERASE_DONE;

	return stm32_qspi_switch_to_memory_mapped(priv);

	return 0;
}

static int stm32_qspi_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
			       size_t *retlen, u_char *buf)
{
	struct stm32_qspi_priv *priv = mtd->priv;
	u8 *addr = priv->mapped_area + from;

	memcpy(buf, addr, len);

	*retlen = len;
	return 0;
}

static int stm32_qspi_probe(struct platform_device *pdev)
{
	const struct dummy_cycles_table *dummy_table = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct stm32_qspi_priv *priv = NULL;
	u32 tmp = 0, flash_size_off = 0;
	struct device *dev = &pdev->dev;
	const char *flash_name;
	struct mtd_info *mtd;
	struct resource *res;
	int i, err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "Failed to initialize Quad-SPI controller\n");
		return -ENOMEM;
	}

	priv->dev = dev;
	mtd = &priv->mtd;
	mtd->priv	= priv;
	mtd->owner	= THIS_MODULE;
	mtd->dev.parent	= dev;
	mtd->name	= DRV_NAME;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(dev, res);
	err = IS_ERR_OR_NULL(priv->regs) ? PTR_ERR(priv->regs) : 0;
	if (err) {
		dev_err(dev, "Failed to map registers: %d\n", err);
		return err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->mapped_area = devm_ioremap_resource(dev, res);
	err = IS_ERR_OR_NULL(priv->mapped_area) ? PTR_ERR(priv->mapped_area) : 0;
	if (err) {
		dev_err(dev, "Failed to get mapped area: %d\n", err);
		return err;
	}

	priv->clk = devm_clk_get(dev, NULL);
	err = IS_ERR_OR_NULL(priv->clk) ? PTR_ERR(priv->clk) : 0;
	if (err) {
		dev_err(dev, "Failed to get clock: %d\n", err);
		return err;
	}

	priv->rst = devm_reset_control_get(dev, NULL);
	err = IS_ERR_OR_NULL(priv->rst) ? PTR_ERR(priv->rst) : 0;
	if (err) {
		dev_err(dev, "Failed to get reset controller: %d\n", err);
		return err;
	}

	reset_control_assert(priv->rst);
	reset_control_deassert(priv->rst);
	clk_prepare_enable(priv->clk);
	priv->clk = devm_clk_get(dev, NULL);

	priv->regs->cr &= ~QSPI_CR_EN;

	err = of_property_read_u32(np, "freq", &tmp);
	if (err < 0) {
		dev_err(dev, "failed to get frequency: %d\n", err);
		return err;
	}
	priv->freq = tmp;

	if (of_property_read_string(np, "flash", &flash_name)) {
		dev_err(dev, "no flash specified\n");
		return -EINVAL;
	}
	for (i = 0, priv->flash = NULL; i < ARRAY_SIZE(stm32_qspiflash_info); i++) {
		const char *name = stm32_qspiflash_info[i].name;
		if (!strncmp(flash_name, name, strlen(name + 1))) {
			priv->flash = &stm32_qspiflash_info[i];
			break;
		}
	}
	if (!priv->flash) {
		dev_err(dev, "flash '%s' is not supported\n", flash_name);
		return -EINVAL;
	}

	writel(QSPI_CR_APMS
	       | QSPI_CR_FTHRES(3)
	       | QSPI_CR_SSHIFT
	       | QSPI_CR_DMAEN,
	       &priv->regs->cr);

	stm32_qspi_set_speed(priv, priv->freq);

	dummy_table = priv->flash->dummy_cycles_table;
	for (i = 0;
	     (i < STM32_DUMMY_TABLE_MAX_SIZE)
		     && dummy_table[i].freq;
	     i++) {
		if (priv->freq > dummy_table[i].freq)
			priv->fast_read_dummy = dummy_table[i].dummy_cycles;
		else
			break;
	}
	flash_size_off = fls(priv->flash->device_size) - 1;

	mtd->type	= MTD_NORFLASH;
	mtd->size	= priv->flash->device_size;
	mtd->writesize	= priv->flash->write_size;
	mtd->erasesize	= priv->flash->erase_size;
	mtd->flags	= MTD_WRITEABLE;
	mtd->_erase	= stm32_qspi_mtd_erase;
	mtd->_read	= stm32_qspi_mtd_read;
	mtd->_write	= stm32_qspi_mtd_write;

	priv->regs->dcr |= QSPI_DCR_CK_MODE;

	/* Number of bytes in Flash memory = 2^(FSIZE+1) */
	priv->regs->dcr &= ~(QSPI_DCR_FSIZE(0x1F));
	priv->regs->dcr |= QSPI_DCR_FSIZE(flash_size_off - 1);

	priv->regs->cr |= QSPI_CR_EN;
	err = stm32_qspi_wait_while_busy(priv);
	if (err) {
		dev_err(dev, "%s: unable to enable QSPI: %d\n", __func__, err);
		return err;
	}

	platform_set_drvdata(pdev, priv);

	stm32_qspi_switch_to_memory_mapped(priv);

	dev_info(dev, "QSPI:  %d MB mapped at %p\n",
		 1 << (flash_size_off - 20), priv->mapped_area);


	return mtd_device_parse_register(mtd, NULL,
					 &(struct mtd_part_parser_data){
						 .of_node = np,
						},
					 NULL, 0);
}

static const struct of_device_id stm32_qspi_dt_ids[] = {
	{ .compatible = "st,stm32-qspi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, stm32_qspi_dt_ids);

static struct platform_driver stm32_qspi_driver = {
	.driver = {
		.name	= DRV_NAME,
		.bus	= &platform_bus_type,
		.of_match_table = stm32_qspi_dt_ids,
	},
	.probe          = stm32_qspi_probe,
};
module_platform_driver(stm32_qspi_driver);

MODULE_AUTHOR("Sergei Miroshnichenko <sergeimir@emcraft.com>");
MODULE_DESCRIPTION("STM32 QUAD-SPI");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, stm32_qspi_dt_ids);
