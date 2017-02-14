/*
 * Device driver for the I2C controller of the STM32F7 microcontroller.
 * Authors: Vladimir Khusainov, vlad@emcraft.com
 *          Vladimir Skvortsov, vskvortsov@emcraft.com
 *          Yuri Tikhonov, yur@emcraft.com
 * Copyright 2013-2016 Emcraft Systems
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

/******************************************************************************
 * Constants and macros
 ******************************************************************************/
/*
 * DMA buffer is allocated from non-cached region in order to guarantee
 * validity of the DMA transactions when cache is enabled.
 */
#define DMABUF_SIZE		1024

/*
 * CR1 register bits
 */
#define _CR1_RXDMAEN		(1 << 15)
#define _CR1_TXDMAEN		(1 << 14)
#define _CR1_ERRIE		(1 << 7)
#define _CR1_TCIE		(1 << 6)
#define _CR1_STOPIE		(1 << 5)
#define _CR1_PE			(1 << 0)

/*
 * CR2 register bits
 */
#define _CR2_AUTOEND		(1 << 25)
#define _CR2_RELOAD		(1 << 24)
#define _CR2_NBYTES(x)		(((x) & 0xff) << 16)
#define _CR2_STOP		(1 << 14)
#define _CR2_START		(1 << 13)
#define _CR2_RD_WRN		(1 << 10)
#define _CR2_SADDR7(x)		(((x) & 0x7f) << 1)
#define _CR2_SADDR10(x)		(((x) & 0x3ff) | (1 << 11))

/*
 * Timing register values
 * Calculated using a program from AN4235 for 100KHz of the I2C clock and 50MHz
 * of the source clock.
 */
#define STM32_I2Cx_DEFAULT_CLK		100000
#define STM32_I2Cx_TIMING_50MHz_100kHz	0x10806190
#define STM32_I2Cx_DEFAULT_TIMING	STM32_I2Cx_TIMING_50MHz_100kHz

/*
 * ISR register bits
 */
#define _ISR_TCR		(1 << 7)
#define _ISR_STOPF		(1 << 5)
#define _ISR_NACKF		(1 << 4)

/*
 * ICR register bits
 */
#define _ICR_STOPCF		(1 << 5)
#define _ICR_NACKCF		(1 << 4)

/******************************************************************************
 * C-types
 ******************************************************************************/
/*
 * DMA specific stuff
 */
struct stm32_dma_data {
	struct dma_async_tx_descriptor *dsc;
	struct dma_chan		*chan;
	dma_cookie_t		cookie;
	struct scatterlist	sg;
};

/*
 * STM32F7 I2C regs
 */
struct stm32f7_i2c_regs {
	u32			cr1;
	u32			cr2;
	u32			oar1;
	u32			oar2;
	u32			timingr;
	u32			timeoutr;
	u32			isr;
	u32			icr;
	u32			pecr;
	u32			rxdr;
	u32			txdr;
};

/*
 * I2C controller private data structure
 */
struct stm32f7_i2c {
	struct device		*dev;		/* Common device	      */
	struct clk		*clk;		/* Clock		      */
	struct reset_control	*rst;		/* Reset		      */
	int			bus;		/* Bus (ID)		      */
	u32			timingr;	/* Calculated timing settings */

	volatile struct stm32f7_i2c_regs __iomem *regs; /* Regs base (virt)   */
	u32			regs_base;	/* Regs base (phys)	      */
	u32			regs_size;	/* Regs size		      */

	int			irq;		/* IRQ #		      */
	u32			ref_clk;	/* Ref clock		      */
	volatile int		msg_status;	/* Message status	      */
	struct i2c_adapter	adap;		/* I2C adapter data	      */
	wait_queue_head_t	wait;		/* Wait queue		      */
	int			nbytes;		/* Remain bytes after reload  */

	void			*dmabuf;	/* Non-cached DMA buf (virt)  */
	dma_addr_t		dmabuf_phy;	/* DMA buf (phys)	      */

	spinlock_t		lock;		/* i2c `xfer` lock	      */

	struct stm32_dma_data	dma_rx;		/* RX DMA descriptor	      */
	struct stm32_dma_data	dma_tx;		/* TX DMA descriptor	      */
};

/******************************************************************************
 * Functions local to this module
 ******************************************************************************/
/*
 * Disable i2c interrupts
 */
static void stm32_i2c_disable_irqs(struct stm32f7_i2c *c)
{
	disable_irq_nosync(c->irq);
	disable_irq_nosync(c->irq + 1);
}

/*
 * Enable i2c interrupts
 */
static void stm32_i2c_enable_irqs(struct stm32f7_i2c *c)
{
	enable_irq(c->irq);
	enable_irq(c->irq + 1);
}

/*
 * Hardware initialization of the I2C controller
 */
static int stm32_i2c_hw_init(struct stm32f7_i2c *c)
{
	struct dma_slave_config config;
	struct stm32_dma_data *dma;
	int rv;

	/*
	 * Reset the I2C controller and then bring it out of reset.
	 * Enable the I2C controller clock.
	 */
	reset_control_assert(c->rst);
	reset_control_deassert(c->rst);
	clk_prepare_enable(c->clk);

	/*
	 * Reset the controller to clear possible errors or locks
	 */
	c->regs->cr2 = 0;
	c->regs->cr1 &= ~_CR1_PE;
	while (c->regs->cr1 & _CR1_PE);

	/*
	 * Set-up speed
	 */
	c->regs->timingr = c->timingr;

	/*
	 * Enable the I2C controller
	 */
	c->regs->cr1 |= _CR1_PE | _CR1_TXDMAEN | _CR1_RXDMAEN;

	/*
	 * Init Rx DMA
	 */
	dma = &c->dma_rx;
	memset(dma, 0, sizeof(*dma));

	dma->chan = dma_request_slave_channel(c->dev, "rx");
	if (!dma->chan) {
		dev_err(c->dev, "rx dma request fail\n");
		rv = -ENODEV;
		goto out;
	}

	memset(&config, 0, sizeof(config));
	config.direction = DMA_DEV_TO_MEM;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr = (dma_addr_t)&c->regs->rxdr;
	config.src_maxburst = 1;

	rv = dmaengine_slave_config(dma->chan, &config);
	if (rv) {
		dev_err(c->dev, "rx dma cfg err %d\n", rv);
		goto out;
	}

	/*
	 * Init Tx DMA
	 */
	dma = &c->dma_tx;
	memset(dma, 0, sizeof(*dma));

	dma->chan = dma_request_slave_channel(c->dev, "tx");
	if (!dma->chan) {
		dev_err(c->dev, "tx dma request fail\n");
		rv = -ENODEV;
		goto out;
	}

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr = (dma_addr_t)&c->regs->txdr;
	config.dst_maxburst = 1;

	rv = dmaengine_slave_config(dma->chan, &config);
	if (rv) {
		dev_err(c->dev, "tx dma cfg err %d\n", rv);
		goto out;
	}

	rv = 0;
out:
	if (rv) {
		if (c->dma_tx.chan) {
			dma_release_channel(c->dma_tx.chan);
			c->dma_tx.chan = NULL;
		}

		if (c->dma_rx.chan) {
			dma_release_channel(c->dma_rx.chan);
			c->dma_rx.chan = NULL;
		}
	}

	return rv;
}

/*
 * Hardware shutdown of the I2C controller
 */
static void stm32_i2c_hw_release(struct stm32f7_i2c *c)
{
	/*
	 * Release the channels
	 */
	if (c->dma_tx.chan) {
		dma_release_channel(c->dma_tx.chan);
		c->dma_tx.chan = NULL;
	}
	if (c->dma_rx.chan) {
		dma_release_channel(c->dma_rx.chan);
		c->dma_rx.chan = NULL;
	}

	/*
	 * Disable the controller
	 */
	c->regs->cr1 &= ~_CR1_PE;

	/*
	 * Put the I2C controller into reset.
	 * Disable clock to the I2C controller.
	 */
	clk_disable_unprepare(c->clk);
	reset_control_assert(c->rst);
}

/*
 * Interrupt handler routine
 * @param irq		IRQ number
 * @param d		controller data structure
 * @returns		IRQ handler exit code
 */
static irqreturn_t stm32_i2c_irq(int irq, void *d)
{
	struct stm32f7_i2c *c = d;
	u32 isr = c->regs->isr;

	if (isr & _ISR_STOPF) {
		u32 icr = _ICR_STOPCF;

		/* disable interrupts */
		c->regs->cr1 &= ~(_CR1_ERRIE | _CR1_STOPIE);
		stm32_i2c_disable_irqs(c);

		if (isr & _ISR_NACKF) {
			/* handle NACK event */
			icr |= _ICR_NACKCF;
			c->msg_status = -ENODEV;
		}

		if (c->msg_status == -EBUSY) {
			c->msg_status = 0;
		}

		/* clear the event */
		c->regs->icr = icr;

		/* cleanup the finished transfer */
		c->regs->cr2 = 0;

		wake_up(&c->wait);
	} else if (isr & _ISR_TCR) {
		u32 cr2 = c->regs->cr2;

		if (c->nbytes > 255) {
			c->nbytes -= 255;
			cr2 |= _CR2_NBYTES(255);
		} else {
			c->nbytes = 0;
			cr2 |= _CR2_NBYTES(c->nbytes);
			cr2 &= ~_CR2_RELOAD;
			c->regs->cr1 &= ~_CR1_TCIE;
		}

		c->regs->cr2 = cr2;
	} else {
		/*
		 * Some error condition -> let's stop and report a failure
		 */
		c->regs->cr2 = _CR2_STOP;

		c->msg_status = -EIO;

		dev_dbg(c->dev,
			"error condition in irq handler: isr=%x\n", isr);
	}

	return IRQ_HANDLED;
}

/*
 * DMA xfer completion callback
 */
static void stm32_i2c_complete(void *arg)
{
	struct stm32_dma_data *dma = arg;

	dma->cookie = -EINVAL;
	dma->dsc = NULL;
}

/******************************************************************************
 * I2C framework API
 ******************************************************************************/
/*
 * Adapter transfer callback
 * @param a		I2C adapter
 * @param m		array of messages
 * @param n		number of messages
 * @returns		message segments transferred or error code (<1)
 */
static int stm32f7_i2c_transfer(struct i2c_adapter *a, struct i2c_msg *m, int n)
{
	struct stm32f7_i2c *c = a->algo_data;
	struct stm32_dma_data *dma;
	u32 cr2, dir;
	int ret = 0;
	int i;

	spin_lock(&c->lock);

	for (i = 0; i < n; i++) {
		/*
		 * Maximum size of a single transfer is limited by the DMA
		 * buffer size.
		 */
		if (m[i].len > DMABUF_SIZE) {
			if (ret == 0)
				ret = -EINVAL;
			goto out;
		}

		if (m[i].len > 255) {
			cr2 = _CR2_NBYTES(255) | _CR2_RELOAD;
			c->nbytes = m[i].len - 255;
			c->regs->cr1 |= _CR1_TCIE;
		} else {
			c->nbytes = 0;
			cr2 = _CR2_NBYTES(m[i].len);
		}

		/* Addressing mode */
		cr2 |= (m[i].flags & I2C_M_TEN) ? _CR2_SADDR10(m[i].addr) :
						  _CR2_SADDR7(m[i].addr);

		/* Direction */
		if (m[i].flags & I2C_M_RD) {
			dir = DMA_DEV_TO_MEM;
			cr2 |= _CR2_RD_WRN;
			dma = &c->dma_rx;
		} else {
			dir = DMA_MEM_TO_DEV;
			memcpy(c->dmabuf, m[i].buf, m[i].len);
			dma = &c->dma_tx;
		}

		/* Autogenerate STOP when finished */
		cr2 |= _CR2_AUTOEND;

		c->msg_status = -EBUSY;

		if (m[i].len) {
			sg_init_table(&dma->sg, 1);

			sg_dma_address(&dma->sg) = (u32)c->dmabuf;
			sg_dma_len(&dma->sg) = m[i].len;

			dma->dsc = dmaengine_prep_slave_sg(dma->chan, &dma->sg,
				1, dir, DMA_PREP_INTERRUPT);

			dma->dsc->callback = stm32_i2c_complete;
			dma->dsc->callback_param = dma;
			dma->cookie = dmaengine_submit(dma->dsc);

			dma_async_issue_pending(dma->chan);
		}

		/* Enable interrupts */
		c->regs->cr1 |= _CR1_ERRIE | _CR1_STOPIE;
		stm32_i2c_enable_irqs(c);

		/* Put config and generate START */
		c->regs->cr2 = cr2 | _CR2_START;

		/*
		 * Wait for the transfer to complete, one way or another
		 */
		if (wait_event_timeout(c->wait, c->msg_status != -EBUSY,
				       5 * HZ) == 0) {
			/* Disable interrupts */
			c->regs->cr1 &= ~(_CR1_ERRIE | _CR1_STOPIE | _CR1_TCIE);
			stm32_i2c_disable_irqs(c);

			/* Clean up the transfer */
			c->regs->cr2 = 0;
			if (ret == 0)
				ret = -ETIMEDOUT;
			goto out;
		}

		if (c->msg_status != 0) {
			if (ret == 0)
				ret = c->msg_status;
		} else {
			ret++;
		}

		/* Cleanup DMA */
		if (m[i].len) {
			if (dma->cookie >= 0)
				dmaengine_terminate_all(dma->chan);

			if (m[i].flags & I2C_M_RD)
				memcpy(m[i].buf, c->dmabuf, m[i].len);
		}

		if (ret != (i + 1))
			break;
	}

out:
	spin_unlock(&c->lock);

	return ret;
}

/*
 * Adapter functionality callback
 * @param a		I2C adapter
 * @returns		OR-ed functionality flags
 */
static u32 stm32f7_i2c_functionality(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/*
 * Algorithm data structure
 */
static const struct i2c_algorithm stm32_i2c_algorithm = {
	.functionality	= stm32f7_i2c_functionality,
	.master_xfer	= stm32f7_i2c_transfer,
};

/******************************************************************************
 * Driver entry/exit points
 ******************************************************************************/
/*
 * Instantiate a new instance of the I2C controller
 */
static int stm32f7_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct stm32f7_i2c *c = NULL;
	struct resource *mem;
	int bus, irq, rv;

	/*
	 * Get the bus # from the platform device:
	 */
	bus = of_alias_get_id(np, "i2c");
	if (!(0 <= bus && bus <= 3)) {
		dev_err(dev, "invalid bus number %d\n", bus);
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Get the IRQ number from the platform device
	 */
	irq = irq_of_parse_and_map(np, 0);
	if (irq < 0) {
		dev_err(dev, "invalid IRQ number %d\n", irq);
		rv = irq;
		goto out;
	}

	/*
	 * Get the register base from the platform device
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "no register base provided\n");
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Allocate the controller-private data structure
	 */
	c = kzalloc(sizeof(struct stm32f7_i2c), GFP_KERNEL);
	if (!c) {
		dev_err(dev, "unable to allocate memory\n");
		rv = -ENOMEM;
		goto out;
	}
	c->dev = dev;
	c->bus = bus;
	spin_lock_init(&c->lock);

	/*
	 * Request a memory region for the CSR block
	 */
	if (!request_mem_region(mem->start, resource_size(mem),
				mem->name)) {
		dev_err(dev, "registers already in use\n");
		rv = -ENOMEM;
		goto err_release_mem;
	}
	c->regs_base = mem->start;
	c->regs_size = resource_size(mem);

	/*
	 * Map regs
	 */
	c->regs = ioremap(mem->start, resource_size(mem));
	if (!c->regs) {
		dev_err(dev, "unable to map registers\n");
		rv = -EINVAL;
		goto err_release_mem_region;
	}

	/*
	 * Get clocks & resets
	 */
	c->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(c->clk)) {
		dev_err(dev, "no clock specified\n");
		rv = -EINVAL;
		goto err_unmap_regs;
	}
	c->ref_clk = clk_get_rate(c->clk);

	c->rst = devm_reset_control_get(dev, NULL);
	if (IS_ERR(c->rst)) {
		dev_err(dev, "no reset controller specified\n");
		rv = -EINVAL;
		goto err_unmap_regs;
	}

	/*
	 * Register interrupt handler for events
	 */
	rv = request_irq(irq, stm32_i2c_irq, 0, dev_name(dev), c);
	if (rv) {
		dev_err(dev, "request for IRQ %d failed\n", irq);
		goto err_unmap_regs;
	}
	disable_irq_nosync(irq);
	c->irq = irq;

	/*
	 * Register interrupt handler for errors
	 */
	rv = request_irq(irq + 1, stm32_i2c_irq, 0, dev_name(dev), c);
	if (rv) {
		dev_err(dev, "request for IRQ %d failed\n", irq + 1);
		goto err_release_irq1;
	}
	disable_irq_nosync(irq + 1);

	rv = of_property_read_u32(np, "timingr", &c->timingr);
	if (rv < 0) {
		if (c->ref_clk != 50000000) {
			dev_err(c->dev, "ref(%d) clock is not supported without setting the timing\n",
				c->ref_clk);
			rv = -EINVAL;
			goto err_release_irq1;
		}
		dev_info(dev, "Assuming default I2C frequency: %dHz\n", STM32_I2Cx_DEFAULT_CLK);
		c->timingr = STM32_I2Cx_DEFAULT_TIMING;
	}

	/*
	 * Link the private data to dev
	 */
	platform_set_drvdata(pdev, c);

	/*
	 * Initialize the I2C adapter data structure
	 */
	c->adap.owner = THIS_MODULE;
	c->adap.nr = bus;
	snprintf(c->adap.name, sizeof(c->adap.name), "stm32_i2c.%u", bus);
	c->adap.algo = &stm32_i2c_algorithm;
	c->adap.algo_data = c;
	c->adap.dev.parent = dev;
	c->adap.dev.of_node = np;

	c->dmabuf = dma_alloc_coherent(c->dev, DMABUF_SIZE, &c->dmabuf_phy,
				       GFP_KERNEL);
	if (!c->dmabuf) {
		dev_err(dev, "unable to allocate DMA buffer\n");
		rv = -ENOMEM;
		goto err_release_irq2;
	}

	/*
	 * Initialize the controller hardware
	 */
	rv = stm32_i2c_hw_init(c);
	if (rv)
		goto err_release_dmabuf;

	/*
	 * Set up the wait queue
	 */
	init_waitqueue_head(&c->wait);

	/*
	 * Register the I2C adapter
	 */
	if (i2c_add_numbered_adapter(&c->adap)) {
		dev_err(dev, "unable to add adapter\n");
		rv = -ENXIO;
		goto err_release_hw;
	}

	/*
	 * If we are here, we are successful
	 */
	dev_info(dev, "I2C Controller %s at %p,irq=%d\n",
		 dev_name(&c->adap.dev), c->regs, c->irq);
	rv = 0;
	goto out;

	/*
	 * Error processing
	 */
err_release_hw:
	stm32_i2c_hw_release(c);
err_release_dmabuf:
	dma_free_coherent(c->dev, DMABUF_SIZE, c->dmabuf, c->dmabuf_phy);
err_release_irq2:
	free_irq(c->irq + 1, c);
err_release_irq1:
	free_irq(c->irq, c);
err_unmap_regs:
	iounmap(c->regs);
err_release_mem_region:
	release_mem_region(mem->start, resource_size(mem));
err_release_mem:
	kfree(c);
	platform_set_drvdata(pdev, NULL);
out:
	return rv;
}

/*
 * Shutdown of an instance of the controller device
 */
static int stm32f7_i2c_remove(struct platform_device *dev)
{
	struct stm32f7_i2c *c  = platform_get_drvdata(dev);
	int ret = 0;

	dma_free_coherent(c->dev, DMABUF_SIZE, c->dmabuf, c->dmabuf_phy);

	/*
	 * Shut the hardware down
	 */
	stm32_i2c_hw_release(c);

	/*
	 * Release kernel resources.
	 */
	platform_set_drvdata(dev, NULL);
	i2c_del_adapter(&c->adap);
	free_irq(c->irq, c);
	free_irq(c->irq + 1, c);
	iounmap(c->regs);
	release_mem_region(c->regs_base, c->regs_size);
	kfree(c);

	return ret;
}

static const struct of_device_id stm32f7_i2c_match[] = {
	{ .compatible = "st,stm32f7-i2c", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32f7_i2c_match);

static struct platform_driver stm32f7_i2c_drv = {
	.probe	= stm32f7_i2c_probe,
	.remove	= stm32f7_i2c_remove,
	.driver	= {
		.name = "stm32f7-i2c",
		.of_match_table = stm32f7_i2c_match,
	},
};
module_platform_driver(stm32f7_i2c_drv);

MODULE_AUTHOR("Vladimir Skvortsov, <vskvortsov@emcraft.com>");
MODULE_DESCRIPTION("Device driver for the I2C controller of STM32F7");
MODULE_LICENSE("GPL");
