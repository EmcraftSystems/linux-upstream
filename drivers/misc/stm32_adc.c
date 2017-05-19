/*
 * Copyright (C) 2017 Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * STM32 ADCx custom non-IIO driver
 *
 * License terms: GNU General Public License (GPL), version 2
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/uaccess.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>

#include <uapi/linux/stm32_adc.h>

/******************************************************************************
 * Constants local to this module
 ******************************************************************************/
/*
 * ADC driver name
 */
#define STM32_ADC_NAME		"stm32_adc"

/*
 * ADC is running at PBLK2/STM32_ADC_FRQ_DIV (/2, /4, /6, /8)
 */
#define STM32_ADC_FRQ_DIV	2

/*
 * Number of ADCs
 */
#define STM32_ADC_NUM		3

/*
 * ADC register constants
 */
#define STM32_ADC_SR_OVR	(1 << 5)

#define STM32_ADC_CR1_OVRIE	(1 << 26)
#define STM32_ADC_CR1_SCAN	(1 << 8)

#define STM32_ADC_CR2_EXTEN_RI	(1 << 28)
#define STM32_ADC_CR2_EXTSEL(x)	((x) << 24)
#define STM32_ADC_CR2_DDS	(1 << 9)
#define STM32_ADC_CR2_DMA	(1 << 8)
#define STM32_ADC_CR2_ADON	(1 << 0)

#define STM32_ADC_SMPR(x, p)	((x) << ((p) * 3))

#define STM32_ADC_SQR_L(x)	((x) << 20)
#define STM32_ADC_SQR_SQ(x, p)	((x) << ((p) * 5))

#define STM32_ADC_CCR_TSVREFE	(1 << 23)
#define STM32_ADC_CCR_PRE(x)	(((x >> 1) - 1) << 16)

/******************************************************************************
 * C-types local to this module
 ******************************************************************************/

/*
 * STM32 ADC registers
 */
struct stm32_adc_reg_adcx {
	u32	sr;
	u32	cr[2];
	u32	smpr[2];
	u32	jofr[4];
	u32	htr;
	u32	ltr;
	u32	sqr[3];
	u32	jsqr;
	u32	jdr[4];
	u32	dr;
	u32	_rsv[44];
} __attribute__((packed));

struct stm32_adc_reg_common {
	u32	csr;
	u32	ccr;
	u32	cdr;
} __attribute__((packed));

struct stm32_adc_regs {
	struct stm32_adc_reg_adcx	adc[STM32_ADC_NUM];
	struct stm32_adc_reg_common	com;
} __attribute__((packed));

/*
 * STM32 ADC common hw
 */
struct stm32_adc_hw {
	struct pwm_device	*pwm;		/* PWM			      */

	unsigned int		regs_base;	/* Register base (phys)	      */
	unsigned int		regs_size;	/* Register area size	      */
	struct stm32_adc_regs __iomem *regs;	/* Register base (virt)	      */

	int			irq;		/* IRQ#			      */
};

/*
 * STM32 ADCx device descriptor
 */
struct stm32_adc_dsc {
	char			name[32];	/* Device name		      */
	struct device		*dev;		/* Pointer to device instance */
	struct cdev		cdev;		/* Char device		      */

	struct clk		*clk;		/* Clock		      */
	struct stm32_adc_regs __iomem *regs;	/* Register base (virt)	      */

	struct completion	eve;		/* Completion event	      */
	struct dma_chan		*dma;		/* DMA channel		      */
	int			trig;		/* Trigger		      */
	int			loop;		/* Continuous mode	      */
	int			run;		/* Running state	      */
	int			stop;		/* Stop command		      */

	int			id;		/* ADC ID		      */
	int			fd;		/* Device ID		      */

	int			seq[STM32_ADC_SEQ_MAX];	/* Channel sequence   */
	int			smp[STM32_ADC_SEQ_MAX];	/* Sampling times     */
	int			chan;		/* Size of sequence	      */
	int			meas;		/* Number of measurements     */

	struct stm32_adc_mmap	*mmap;		/* Mmaped buffer	      */
	dma_addr_t		dma_mmap;	/* DMA buffer address	      */
	u32			mmap_size;	/* Size of mmaped buf	      */
};

/******************************************************************************
 * Prototypes and variables local to this module
 ******************************************************************************/

static void stm32_adc_dma_handler(void *data);

static struct class *stm32_adc_class;
static int stm32_adc_major;

static struct stm32_adc_hw stm32_adc_hw;
static struct stm32_adc_dsc stm32_adc_dsc[STM32_ADC_NUM];
static int stm32_adc_smp_val[] = {
	3, 15, 28, 56, 85, 112, 144, 480
};

/******************************************************************************
 * Hw control routines
 ******************************************************************************/

static void stm32_adc_dump_regs(const char *who, struct stm32_adc_dsc *dsc)
{
	struct stm32_adc_reg_adcx __iomem *regs = &dsc->regs->adc[dsc->id];

	printk("%s ADC%d@%p:SR=%02x;CR=%08x.%08x;SQR=%08x.%08x.%08x;CCR=%08x\n",
		who, dsc->id + 1, regs,
		readl(&regs->sr), readl(&regs->cr[0]), readl(&regs->cr[1]),
		readl(&regs->sqr[0]), readl(&regs->sqr[1]),
		readl(&regs->sqr[2]),
		readl(&dsc->regs->com.ccr));
}

static int stm32_adc_configure(struct stm32_adc_dsc *dsc, int cont)
{
	struct stm32_adc_reg_adcx __iomem *regs = &dsc->regs->adc[dsc->id];
	struct dma_async_tx_descriptor *tx;
	struct dma_slave_config cfg;
	int imax[] = { 16, 12, 6 };
	unsigned long flags;
	int i, k, s, rv;
	u32 val, len;

	local_irq_save(flags);

	if (dsc->run) {
		rv = -EBUSY;
		goto out;
	}

	/*
	 * Program, and enable DMA
	 */
	memset(&cfg, 0, sizeof(cfg));
	cfg.src_addr = (dma_addr_t)&regs->dr;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	rv = dmaengine_slave_config(dsc->dma, &cfg);
	if (rv) {
		pr_err("%s: dma config err %d\n", __func__, rv);
		goto out;
	}

	len = dsc->meas * dsc->chan * sizeof(dsc->mmap->data[0]);
	if (cont) {
		tx = dmaengine_prep_dma_cyclic(dsc->dma,
			(dma_addr_t)dsc->mmap->data, len, len,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	} else {
		tx = dmaengine_prep_slave_single(dsc->dma,
			(dma_addr_t)dsc->mmap->data, len,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	}
	if (!tx) {
		pr_err("%s: dma %s prep err\n", __func__,
			cont ? "cyclic" : "single");
		rv = -EFAULT;
		goto out;
	}

	tx->callback = stm32_adc_dma_handler;
	tx->callback_param = dsc;

	dmaengine_submit(tx);
	dma_async_issue_pending(dsc->dma);

	/*
	 * Program sequence in SQRx registers
	 */
	for (s = 2, i = 0; s >= 0; s--) {
		writel(0, &regs->sqr[s]);
		for (k = 0; i < imax[s] && i < dsc->chan; i++, k++) {
			val = readl(&regs->sqr[s]);
			val |= STM32_ADC_SQR_SQ(dsc->seq[i], k);
			writel(val, &regs->sqr[s]);
		}
	}
	val = readl(&regs->sqr[0]);
	val |= STM32_ADC_SQR_L(dsc->chan - 1);
	writel(val, &regs->sqr[0]);

	/*
	 * Set SMPR
	 */
	writel(0, &regs->smpr[0]);
	writel(0, &regs->smpr[1]);
	for (i = 0; i < dsc->chan; i++) {
		s = dsc->seq[i] / 10;
		k = dsc->seq[i] % 10;

		val = readl(&regs->smpr[s]);
		val |= STM32_ADC_SMPR(dsc->smp[i], k);
		writel(val, &regs->smpr[s]);
	}

	/*
	 * Program controls, and run ADC
	 */
	writel(STM32_ADC_CR1_OVRIE | STM32_ADC_CR1_SCAN, &regs->cr[0]);
	writel(STM32_ADC_CR2_EXTEN_RI | STM32_ADC_CR2_EXTSEL(dsc->trig) |
		STM32_ADC_CR2_DDS | STM32_ADC_CR2_DMA | STM32_ADC_CR2_ADON,
		&regs->cr[1]);

	dsc->loop = cont;
	dsc->stop = 0;
	dsc->run = 1;

	rv = 0;
out:
	if (rv)
		stm32_adc_dump_regs(__func__, dsc);

	local_irq_restore(flags);
	return rv;
}

static irqreturn_t stm32_adc_irq_handler(int irq, void *ptr)
{
	struct stm32_adc_dsc *dsc;
	irqreturn_t rv = IRQ_NONE;
	u32 sr;
	int i;

	for (i = 0; i < STM32_ADC_NUM; i++) {
		dsc = &stm32_adc_dsc[i];
		if (!dsc->dev)
			continue;

		sr = readl(&dsc->regs->adc[dsc->id].sr);
		if (!(sr & STM32_ADC_SR_OVR))
			continue;
		pr_warning("%s: overrun\n", dsc->name);

		sr &= ~STM32_ADC_SR_OVR;
		writel(sr, &dsc->regs->adc[dsc->id].sr);

		rv = IRQ_HANDLED;
	}

	return rv;
}

static void stm32_adc_dma_handler(void *data)
{
	struct stm32_adc_dsc *dsc = data;

	if (!dsc->loop || dsc->stop) {
		writel(0, &dsc->regs->adc[dsc->id].cr[1]);
		dmaengine_terminate_all(dsc->dma);
		dsc->run = 0;
	}

	complete_all(&dsc->eve);
}

/******************************************************************************
 * Char device file operations
 ******************************************************************************/

static int stm32_adc_fops_open(struct inode *inode, struct file *file)
{
	int rv, adc = iminor(inode);

	if (adc < 0 || adc >= STM32_ADC_NUM || !stm32_adc_dsc[adc].dev) {
		rv = -ENODEV;
		goto out;
	}

	file->private_data = &stm32_adc_dsc[adc];
	rv = 0;
out:
	return rv;
}

static ssize_t stm32_adc_fops_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct stm32_adc_dsc *dsc = file->private_data;
	unsigned int *data;
	char str[32];
	int i, k, n, pos, rv;

	/*
	 * If not running, then configure, start, and wait for complete
	 * If already running, then just print current values
	 */
	if (!dsc->run) {
		init_completion(&dsc->eve);
		rv = stm32_adc_configure(dsc, 0);
		if (rv)
			goto out;

		/*
		 * Wait for completion
		 */
		rv = wait_for_completion_interruptible(&dsc->eve);
		if (rv)
			goto out;
	}

	/*
	 * Print out current values
	 */
	rv = -EFAULT;
	pos = 0;

	for (k = 0; k < dsc->chan; k++) {
		n = sprintf(str, "%s<%3u>%s",
			k != 0 ? "" : "       ", dsc->seq[k],
			k != dsc->chan - 1 ? "," : "\n");

		if (pos + n > count || copy_to_user(&buf[pos], str, n))
			goto out;
		pos += n;
	}

	data = (void *)dsc->mmap->data;
	for (i = 0; i < dsc->meas; i++) {
		n = sprintf(str, "[%3u]: ", i + 1);
		if (pos + n > count || copy_to_user(&buf[pos], str, n))
			goto out;
		pos += n;

		for (k = 0; k < dsc->chan; k++) {
			n = sprintf(str, "%5u%s", *data++,
				k != dsc->chan - 1 ? "," : "\n");
			if (i == dsc->meas - 1 && k == dsc->chan - 1)
				n += 1;

			if (pos + n > count || copy_to_user(&buf[pos], str, n))
				goto out;
			pos += n;
		}
	}

	rv = pos;
out:
	if (rv < 0)
		pr_err("%s: error %d\n", __func__, rv);

	return rv;
}

static unsigned long stm32_adc_fops_get_unmapped_area(struct file *file,
	unsigned long addr, unsigned long len, unsigned long pgoff,
	unsigned long flags)
{
	struct stm32_adc_dsc *dsc = file->private_data;

	return (unsigned long)dsc->mmap->data;
}

static int stm32_adc_fops_mmap(struct file *file, struct vm_area_struct *vma)
{
	return vma->vm_flags & VM_SHARED ? 0 : -EACCES;;
}

static long stm32_adc_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct stm32_adc_dsc *dsc = file->private_data;
	int rv;

	/*
	 * On any success IOCTL below we wait until measurement completes
	 * before return
	 */
	init_completion(&dsc->eve);

	switch (cmd) {
	case STM32_ADC_SINGLE_SEQ:
	case STM32_ADC_LOOP_SEQ:
		/*
		 * If ADC is already running, then just wait for next measure;
		 * otherwise run in a single/loop mode.
		 */
		if (dsc->run)
			break;
		rv = stm32_adc_configure(dsc,
					 cmd == STM32_ADC_LOOP_SEQ ? 1 : 0);
		if (rv)
			goto out;
		break;
	case STM32_ADC_STOP_SEQ:
		if (!dsc->run) {
			rv = 0;
			goto out;
		}
		dsc->stop = 1;
		break;
	default:
		rv = EINVAL;
		goto out;
	}

	rv = wait_for_completion_interruptible(&dsc->eve);
out:
	return rv;
}

static const struct file_operations stm32_adc_fops = {
	.owner			= THIS_MODULE,
	.open			= stm32_adc_fops_open,
	.read			= stm32_adc_fops_read,
	.get_unmapped_area	= stm32_adc_fops_get_unmapped_area,
	.mmap			= stm32_adc_fops_mmap,
	.unlocked_ioctl		= stm32_adc_fops_ioctl,
};

/******************************************************************************
 * Platform driver interface
 ******************************************************************************/

static int stm32_adc_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *adc_np = dev->of_node, *np;
	unsigned long frq, smp_sum, smp_ns, pwm_ns;
	struct stm32_adc_dsc *dsc;
	struct property *prop;
	const __be32 *cur;
	int i, n, inum, rv, id;
	u32 trig_usec, trig_code, val, smp;

	/*
	 * Check if PWM configuration is OK
	 */
	stm32_adc_hw.pwm = of_pwm_get(adc_np, NULL);
	if (IS_ERR(stm32_adc_hw.pwm)) {
		pr_err("%s: unable to get pwm\n", __func__);
		rv = -ENODEV;
		goto out;
	}
	rv = of_property_read_u32(adc_np, "st,adc-trig-code", &trig_code);
	if (rv) {
		pr_err("%s: can't get `st,adc-trig-code`\n", __func__);
		rv = -EINVAL;
		goto out;
	}
	trig_usec = pwm_get_period(stm32_adc_hw.pwm) / 1000;
	if (!trig_usec) {
		pr_err("%s: bad pwm freq\n", __func__);
		rv = -EINVAL;
		goto out;
	}
	pwm_ns = trig_usec * 1000;

	stm32_adc_hw.regs = of_iomap(adc_np, 0);
	if (IS_ERR(stm32_adc_hw.regs)) {
		pr_err("%s: unable to get regs\n", __func__);
		rv = -ENODEV;
		goto out;
	}

	stm32_adc_hw.irq = of_irq_get(adc_np, 0);
	if (!stm32_adc_hw.irq) {
		pr_err("%s: unable to get irq resource\n", __func__);
		rv = -ENODEV;
		goto out;
	}
	rv = devm_request_irq(dev, stm32_adc_hw.irq, stm32_adc_irq_handler,
			0, STM32_ADC_NAME, stm32_adc_hw.regs);
	if (rv) {
		pr_err("%s: unable to request irq\n", __func__);
		goto out;
	}

	inum = 0;
	for_each_available_child_of_node(adc_np, np) {
		/* Skip injected channels */
		if (of_property_read_bool(np, "st,injected"))
			continue;

		id = of_alias_get_id(np, "adc");
		if (id < 0 || id >= STM32_ADC_NUM)
			continue;
		dsc = &stm32_adc_dsc[id];

		dsc->meas = 0;
		of_property_read_u32(np, "st,adc-meas", &dsc->meas);
		if (!dsc->meas)
			continue;

		dsc->chan = 0;
		of_property_for_each_u32(np, "st,adc-channels", prop, cur, val)
			dsc->chan++;
		if (!dsc->chan || dsc->chan > STM32_ADC_SEQ_MAX)
			continue;

		dsc->clk = of_clk_get(np, 0);
		if (IS_ERR(dsc->clk)) {
			pr_err("%s: unable to get clk\n", __func__);
			rv = -ENODEV;
			goto err_dev;
		}
		rv = clk_prepare_enable(dsc->clk);
		if (rv < 0) {
			pr_err("%s: unable to run clk\n", __func__);
			rv = -EFAULT;
			goto err_dev;
		}
		frq = clk_get_rate(dsc->clk) / STM32_ADC_FRQ_DIV;
		smp_ns = 1000000000 / frq;

		inum++;
		n = snprintf(dsc->name, sizeof(dsc->name), "adc%d", id + 1);
		if (n >= sizeof(dsc->name)) {
			pr_err("%s: unable to name %s.%d\n", __func__,
				STM32_ADC_NAME, id);
			goto err_dev;
		}

		dsc->dma = of_dma_request_slave_channel(np, "rx");
		if (IS_ERR(dsc->dma)) {
			pr_err("%s: unable to get dma for %s\n", __func__,
				dsc->name);
			goto err_dev;
		}

		dsc->id = id;
		dsc->fd = MKDEV(stm32_adc_major, id);
		dsc->regs = stm32_adc_hw.regs;
		dsc->trig = trig_code;
		init_completion(&dsc->eve);

		i = n = smp_sum = 0;
		if (!of_property_read_u32(np, "st,adc-smp", &smp)) {
			for (i = 0; i < ARRAY_SIZE(stm32_adc_smp_val); i++) {
				if (stm32_adc_smp_val[i] >= smp)
					break;
			}

			if (i == ARRAY_SIZE(stm32_adc_smp_val) ||
			    stm32_adc_smp_val[i] != smp) {
				/* No exact match, select the most close */
				if ((i == ARRAY_SIZE(stm32_adc_smp_val)) ||
				    (i && ((stm32_adc_smp_val[i] - smp) >
					   (smp - stm32_adc_smp_val[i - 1]))))
					i--;

				pr_warn("`st,adc-smp`=%d not supported, "
					"use %d\n", smp, stm32_adc_smp_val[i]);
			}
		}
		smp = i;
		of_property_for_each_u32(np, "st,adc-channels", prop, cur, val) {
			dsc->seq[n] = val;
			dsc->smp[n] = smp;
			smp_sum += stm32_adc_smp_val[dsc->smp[n]];
			n++;
		}

		if (2 * smp_sum * smp_ns >= pwm_ns) {
			pr_err("%s: %s sampling time (2x%ldx%ldns) is more "
				"than PWM trigger rate (%ldns)\n", __func__,
				dsc->name, smp_sum, smp_ns, pwm_ns);
			rv = -EINVAL;
			goto err_dev;
		}

		dsc->dev = device_create(stm32_adc_class, NULL, dsc->fd, NULL,
					dsc->name);
		if (!dsc->dev) {
			pr_err("%s: unable to create device '%s'\n", __func__,
				dsc->name);
			rv = -ENODEV;
			goto err_dev;
		}

		dsc->mmap_size = dsc->meas * dsc->chan *
				 sizeof(dsc->mmap->data[0]);
		dsc->mmap = dma_alloc_coherent(dsc->dev, dsc->mmap_size,
					&dsc->dma_mmap, GFP_KERNEL);
		if (!dsc->mmap) {
			pr_err("%s: unabled to alloca dmamem\n", __func__);
			rv = -ENOMEM;
			goto err_dev;
		}
		memset(dsc->mmap, 0, dsc->mmap_size);

		cdev_init(&dsc->cdev, &stm32_adc_fops);
		rv = cdev_add(&dsc->cdev, dsc->fd, 1);
		if (rv < 0) {
			pr_err("%s: unabled to add device '%s' (%d)\n",
				__func__, dsc->name, rv);
			goto err_dev;
		}

		dev_set_drvdata(dsc->dev, dsc);
		pr_info("STM32 %s run @%ld.%ldMHz/%3dsmp/%2dchan; "
			"char device {%d,%d}\n",
			dsc->name, frq / 1000000, (frq / 100000) % 10,
			stm32_adc_smp_val[smp],
			dsc->chan, stm32_adc_major, id);
	}
	if (!inum) {
		rv = -ENODEV;
		goto err_dev;
	}

	/*
	 * Configure common settings
	 */
	val = STM32_ADC_CCR_TSVREFE | STM32_ADC_CCR_PRE(STM32_ADC_FRQ_DIV);
	writel(val, &stm32_adc_hw.regs->com.ccr);

	/*
	 * Run PWM
	 */
	rv = pwm_config(stm32_adc_hw.pwm, pwm_ns / 2, pwm_ns);
	if (rv) {
		pr_err("%s: unabled to config pwm (%ldns)\n", __func__,
			pwm_ns);
		goto err_dev;
	}
	rv = pwm_enable(stm32_adc_hw.pwm);
	if (rv) {
		pr_err("%s: unable to enable pwm\n", __func__);
		goto err_dev;
	}

	rv =  0;
	goto out;
err_dev:
	for (i = 0; i < STM32_ADC_NUM; i++) {
		dsc = &stm32_adc_dsc[i];
		if (!dsc->dev)
			continue;

		dev_set_drvdata(dsc->dev, NULL);
		cdev_del(&dsc->cdev);

		if (dsc->mmap) {
			dma_free_coherent(dsc->dev, dsc->mmap_size,
					dsc->mmap, dsc->dma_mmap);
		}
		device_destroy(stm32_adc_class, dsc->fd);

		if (dsc->dma)
			dma_release_channel(dsc->dma);

		memset(dsc, 0, sizeof(struct stm32_adc_dsc));
		pr_info("STM32 ADC%d removed\n", i + 1);
	}

	free_irq(stm32_adc_hw.irq, stm32_adc_hw.regs);
	iounmap(stm32_adc_hw.regs);
out:
	return rv;
}

static int stm32_adc_plat_remove(struct platform_device *pdev)
{
	struct stm32_adc_dsc *dsc;
	int i;

	for (i = 0; i < STM32_ADC_NUM; i++) {
		dsc = &stm32_adc_dsc[i];
		if (!dsc->dev)
			continue;

		dev_set_drvdata(dsc->dev, NULL);
		cdev_del(&dsc->cdev);

		if (dsc->mmap) {
			dma_free_coherent(dsc->dev, dsc->mmap_size,
					dsc->mmap, dsc->dma_mmap);
		}
		device_destroy(stm32_adc_class, dsc->fd);

		if (dsc->dma)
			dma_release_channel(dsc->dma);

		memset(dsc, 0, sizeof(struct stm32_adc_dsc));
	}

	free_irq(stm32_adc_hw.irq, stm32_adc_hw.regs);
	iounmap(stm32_adc_hw.regs);

	return 0;
}

static const struct of_device_id stm32_adc_dt_ids[] = {
	{ .compatible = "st,stm32-adc-custom", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, stm32_adc_dt_ids);

static struct platform_driver stm32_adc_plat = {
	.probe	= stm32_adc_plat_probe,
	.remove	= stm32_adc_plat_remove,
	.driver	= {
		.name	= "stm32_adc",
		.of_match_table = of_match_ptr(stm32_adc_dt_ids),
	},
};

/******************************************************************************
 * Kernel module interface
 ******************************************************************************/

static int __init stm32_adc_drv_init(void)
{
	dev_t dev;
	int rv;

	stm32_adc_class = class_create(THIS_MODULE, STM32_ADC_NAME);
	if (IS_ERR(stm32_adc_class)) {
		rv = PTR_ERR(stm32_adc_class);
		pr_err("%s: unable to register class (%d)\n", __func__, rv);
		goto out;
	}

	rv = alloc_chrdev_region(&dev, 0, STM32_ADC_NUM, STM32_ADC_NAME);
	if (rv) {
		pr_err("%s: can't register chrdev region (%d)\n", __func__, rv);
		goto err_class;
	}
	stm32_adc_major = MAJOR(dev);

	rv = platform_driver_register(&stm32_adc_plat);
	if (rv) {
		pr_err("%s: can't register plat driver (%d)\n", __func__, rv);
		goto err_chrdev;
	}

	rv = 0;
	goto out;

err_chrdev:
	unregister_chrdev_region(dev, STM32_ADC_NUM);
err_class:
	class_destroy(stm32_adc_class);
	stm32_adc_class = NULL;
out:
	return rv;
}
device_initcall(stm32_adc_drv_init);
