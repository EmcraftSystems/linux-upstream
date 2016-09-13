/*
 * Device driver for the I2C controller of the STM32F microcontrollers.
 *
 * Copyright 2013-2015 Emcraft Systems
 * Vladimir Khusainov, vlad@emcraft.com
 * Yuri Tikhonov, yur@emcraft.com
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>

/*
 * Debug output control. While debugging, have I2C_STM32_DEBUG defined.
 * In deployment, make sure that I2C_STM32_DEBUG is undefined
 * to avoid performance and size overhead of debug messages.
 */
#undef I2C_STM32_DEBUG

#if defined(I2C_STM32_DEBUG)

/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int stm32_i2c_debug = 4;

/*
 * User can change verbosity of the driver (when loading the module,
 * or the kernel, in case the driver is linked in statically,
 * but not through a /sysfs parameter file)
 */
module_param(stm32_i2c_debug, int, S_IRUGO);
MODULE_PARM_DESC(stm32_i2c_debug, "I2C controller driver verbosity level");

#if !defined(MODULE)

/*
 * Parser for the boot time driver verbosity parameter
 * @param str		user defintion of the parameter
 * @returns		1->success
 */
static int __init stm32_i2c_debug_setup(char * str)
{
	get_option(&str, &stm32_i2c_debug);
	return 1;
}
__setup("stm32_i2c_debug=", stm32_i2c_debug_setup);

#endif /* !defined(MODULE) */

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (stm32_i2c_debug >= level) printk(KERN_INFO "%s: " fmt,	\
					     __func__, ## args)

#else

#define d_printk(level, fmt, args...)

#endif /* defined(I2C_STM32_DEBUG) */

/*
 * I2C controller private data structure
 */
struct i2c_stm32 {
	struct device				*dev;	/* Common device */
	int					bus;	/* Bus (ID) */
	struct clk				*clk;	/* Clock */
	struct reset_control			*rst;	/* Reset */
	volatile struct stm32_i2c_regs __iomem	*regs;	/* Regs base(virt) */
	unsigned int				regs_base; /* Regs base(phys) */
	unsigned int				regs_size; /* Regs size */
	int					irq;	/* IRQ # */
	unsigned int				ref_clk;/* Ref clock */
	unsigned int				i2c_clk;/* Bus clock */
	struct i2c_msg				*msg;	/* Current message */
	int					msg_n;	/* Segments in msg */
	int					msg_i;	/* Idx in a segment */
	volatile int				msg_status; /* Message status */
	struct i2c_adapter			adap;	/* I2C adapter data */
	wait_queue_head_t			wait;	/* Wait queue */
};

/*
 * Description of the the STM32 I2C hardware interfaces.
 */
struct stm32_i2c_regs {
	unsigned int			cr1;
	unsigned int			cr2;
	unsigned int			oar1;
	unsigned int			oar2;
	unsigned int			dr;
	unsigned int			sr1;
	unsigned int			sr2;
	unsigned int			ccr;
	unsigned int			trise;
	unsigned int			fltr;
};

/*
 * Convert to MHz
 */
#define MHZ(v)				((v) * 1000000)

/*
 * Some bits in various CSRs
 */
#define RCC_APB1RSTR_I2C1		(1<<21)
#define RCC_APB1RSTR_I2C2		(1<<22)
#define RCC_APB1RSTR_I2C3		(1<<23)
#define RCC_APB1ENR_I2C1		(1<<21)
#define RCC_APB1ENR_I2C2		(1<<22)
#define RCC_APB1ENR_I2C3		(1<<23)
#define	I2C_STM32_CR1_SWRST		(1<<15)
#define	I2C_STM32_CR1_ACK		(1<<10)
#define	I2C_STM32_CR1_STO		(1<<9)
#define	I2C_STM32_CR1_STA		(1<<8)
#define	I2C_STM32_CR1_PE		(1<<0)
#define	I2C_STM32_CR1_MASK		(I2C_STM32_CR1_ACK | \
					 I2C_STM32_CR1_STO | I2C_STM32_CR1_STA | \
					 I2C_STM32_CR1_PE  | I2C_STM32_CR1_SWRST)
#define	I2C_STM32_CR2_ITBUFEN		(1<<10)
#define	I2C_STM32_CR2_ITEVTEN		(1<<9)
#define	I2C_STM32_CR2_ITERREN		(1<<8)
#define	I2C_STM32_CR2_FREQ(v)		((v)<<0)
#define	I2C_STM32_SR1_AF		(1<<10)
#define	I2C_STM32_SR1_TXE		(1<<7)
#define	I2C_STM32_SR1_RXNE		(1<<6)
#define	I2C_STM32_SR1_BTF		(1<<2)
#define	I2C_STM32_SR1_ADDR		(1<<1)
#define	I2C_STM32_SR2_TRA		(1<<2)
#define	I2C_STM32_SR1_SB		(1<<0)
#define	I2C_STM32_CCR_CCR(v)		((v)<<0)
#define	I2C_STM32_CCR_FS		(1<<15)
#define	I2C_STM32_CCR_DUTY		(1<<14)
#define	I2C_STM32_TRISE_TRISE(v)	((v)<<0)

static inline void stm32_i2c_disable_irqs(struct i2c_stm32 *c)
{
	disable_irq_nosync(c->irq);
	disable_irq_nosync(c->irq + 1);
}

static inline void stm32_i2c_enable_irqs(struct i2c_stm32 *c)
{
	enable_irq(c->irq);
	enable_irq(c->irq + 1);
}

/*
 * Reset the I2C controller to known state
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static void stm32_i2c_hw_clear(struct i2c_stm32 *c)
{
	int v;

	/*
	 * Reset the controller to clear possible errors or locks
	 */
	v = c->regs->cr1 & ~I2C_STM32_CR1_MASK;
	c->regs->cr1 = v | I2C_STM32_CR1_SWRST;
	udelay(20);
	c->regs->cr1 = v & ~I2C_STM32_CR1_SWRST;

	/*
	 * Set the clocks.
	 */
	if (c->i2c_clk <= 100000) {
		v = c->ref_clk / MHZ(1);
		c->regs->cr2 = I2C_STM32_CR2_FREQ(v);
		c->regs->trise = I2C_STM32_TRISE_TRISE(v + 1);
		v = c->ref_clk / (c->i2c_clk << 1);
		v = v < 0x04 ? 0x04 : v;
		c->regs->ccr = I2C_STM32_CCR_CCR(v);
	}
	else {
		v = c->ref_clk / MHZ(1);
		v = ((v * 300) / 1000) + 1;
		c->regs->trise = I2C_STM32_TRISE_TRISE(v);
		v = c->ref_clk / (c->i2c_clk * 25);
		v |= ((v & 0x0FFF) == 0) ? 0x0001 : 0;
		v |= I2C_STM32_CCR_FS | I2C_STM32_CCR_DUTY;
		c->regs->ccr = I2C_STM32_CCR_CCR(v);
	}

	/*
	 * Enable hardware interrupts, including for error conditions
	 */
	c->regs->cr2 |= I2C_STM32_CR2_ITBUFEN | I2C_STM32_CR2_ITEVTEN |
			I2C_STM32_CR2_ITERREN;

	/*
	 * Enable the I2C controller
	 */
	c->regs->cr1 |= I2C_STM32_CR1_PE;
}

/*
 * Hardware initialization of the I2C controller
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static int stm32_i2c_hw_init(struct i2c_stm32 *c)
{
	int ret = 0;

	/*
	 * First, figure out if we are able to configure the clocks
	 * If not, we want to bail out without enabling enything.
	 */
	if (c->i2c_clk == 0 || c->i2c_clk > 400000) {
		dev_err(c->dev, "bus clock %d not supported\n",
			c->i2c_clk);
		ret = -ENXIO;
		goto Done;
	}

	/*
	 * Reset the I2C controller and then bring it out of reset.
	 * Enable the I2C controller clock.
	 */
	reset_control_assert(c->rst);
	reset_control_deassert(c->rst);
	clk_prepare_enable(c->clk);

	stm32_i2c_hw_clear(c);
Done:
	d_printk(2, "bus=%d,"
		"cr1=0x%x,cr2=0x%x,sr1=0x%x,ccr=0x%x,trise=0x%x,ret=%d\n",
		c->bus,
		!ret ? c->regs->cr1 : 0, !ret ? c->regs->cr2 : 0,
		!ret ? c->regs->sr1 : 0, !ret ? c->regs->ccr : 0,
		!ret ? c->regs->trise : 0, ret);
	return ret;
}

/*
 * Hardware shutdown of the I2C controller
 * @param c		controller data structure
 */
static void stm32_i2c_hw_release(struct i2c_stm32 *c)
{
	/*
	 * Disable the controller
	 */
	c->regs->cr1 &= ~I2C_STM32_CR1_PE;

	/*
	 * Put the I2C controller into reset.
	 * Disable clock to the I2C controller.
	 */
	clk_disable_unprepare(c->clk);
	reset_control_assert(c->rst);

	d_printk(2, "bus=%d\n", c->bus);
}

/*
 * Interrupt handler routine
 * @param irq		IRQ number
 * @param d		controller data structure
 * @returns		IRQ handler exit code
 */
static irqreturn_t stm32_i2c_irq(int irq, void *d)
{
	struct i2c_stm32 *c = (struct i2c_stm32 *) d;
	unsigned int sr1 = c->regs->sr1;
	unsigned int sr2 = 0;
	int disable_intr = 0;
	irqreturn_t ret = IRQ_HANDLED;

	/*
	 * Check if there is an interrupt event
	 * pending at the controller. Bail out if there is none.
	 * It does happen sometimes for some reason.
	 */
	if (!sr1) {
		ret = IRQ_NONE;
		goto Done;
	}

	/*
	 * Implement the state machine defined by the I2C controller
	 */
	if (sr1 & I2C_STM32_SR1_SB) {
		/*
		 * Remove the start condition
		 */
		c->regs->cr1 &= ~I2C_STM32_CR1_STA;

		/*
		 * Start sent -> send out addr and direction
		 */
		c->regs->dr = (c->msg->addr << 1) |
			      (c->msg->flags & I2C_M_RD ? 1 : 0);
	} else if (sr1 & I2C_STM32_SR1_AF) {
		/*
		 * No ack -> let's stop and report a failure
		 */
		c->regs->sr1 &= ~I2C_STM32_SR1_AF;
		c->msg_status = -ENODEV;
		disable_intr = 1;
	} else if ((sr1 & I2C_STM32_SR1_ADDR) || (sr1 & I2C_STM32_SR1_TXE) ||
		   (sr1 & I2C_STM32_SR1_BTF)  || (sr1 & I2C_STM32_SR1_RXNE)) {
		/*
		 * There is a pecularity in the hardware controller that
		 * TXE can be set while BTF is not. Wait until BTF sets.
		 * TO-DO: This is terribly bad of course. Waiting in an ISR
		 * is out of question. Need to fix that somehow.
		 */
		while (sr1 == I2C_STM32_SR1_TXE)
			sr1 = c->regs->sr1;

		/*
		 * Slave has acknowledged the address
		 * or byte transfer has finished.
		 *
		 * If this is the master receiver mode, it is important
		 * to set/reset ACK before clearing the interrupt condition:
		 */
		if (c->msg_i + 1 == c->msg->len) {
			/*
			 * Will be receiving the last byte from the slave.
			 * Return NACK to tell the slave to stop sending.
			 */
			c->regs->cr1 &= ~I2C_STM32_CR1_ACK;
		} else {
			/*
			 * Will be receiving more data from the slave.
			 * Return ACK to tell the slave to send more.
			 */
			c->regs->cr1 |= I2C_STM32_CR1_ACK;
		}

		/*
		 * Clear the interrupt condition
		 */
		sr2 = c->regs->sr2;

		/*
		 * Depending on the direction, enter
		 * transmitter or receiver mode.
		 */
		if (sr2 & I2C_STM32_SR2_TRA) {
			/*
			 * This is the master transmiter mode:
			 */
			if (c->msg_i < c->msg->len) {
				/*
				 * There is more data to send, send it
				 */
				c->regs->dr = c->msg->buf[(c->msg_i)++];
			} else if (--(c->msg_n) == 0) {
				/*
				 * This is last transfer in the message,
				 * report success.
				 */
				c->msg_status = 0;
				disable_intr = 1;
			} else {
				/*
				 * This is not the last transfer in the message.
				 * Advance to the next segment and
				 * initate a repeated start.
				 */
				c->msg++;
				c->msg_i = 0;
				c->regs->cr1 |= I2C_STM32_CR1_STA;
			}
		} else if (sr1 & I2C_STM32_SR1_RXNE) {
			/*
			 * This is the master receiver mode:
			 * Retrieve the data.
			 */
			c->msg->buf[c->msg_i++] = c->regs->dr;

			if (c->msg_i < c->msg->len) {
				/*
				 * More data to get. Get out of IRQ and wait
				 * for a next interrupt.
				 */
			} else if (--(c->msg_n) == 0) {
				/*
				 * This is last transfer in the message,
				 * report success.
				 */
				c->msg_status = 0;
				disable_intr = 1;
			} else {
				/*
				 * This is not the last transfer in the message.
				 * Advance to the next segment
				 * and initate a repeated start.
				 */
				c->msg++;
				c->msg_i = 0;
				c->regs->cr1 |= I2C_STM32_CR1_STA;
			}
		}
	} else {
		/*
		 * Some error condition -> let's stop and report a failure
		 */
		c->regs->sr1 = 0x0;
		dev_dbg(c->dev,
			"error condition in irq handler: sr1=%x\n", sr1);
		c->msg_status = -EIO;
		disable_intr = 1;
	}

	/*
	 * If the current transfer is done, disable interrupts
	 */
	if (disable_intr)
		stm32_i2c_disable_irqs(c);

	/*
	 * Exit on failure or all bytes have been transferred
	 */
	if (c->msg_status != -EBUSY) {
		/*
		 * Clear the interrupt condition
		 */
		wake_up(&c->wait);
	}

Done:
	d_printk(4, "sr1=0x%x,sr2=0x%x,ret=%d\n", sr1, sr2, ret);
	return ret;
}

/*
 * Adapter transfer callback
 * @param a		I2C adapter
 * @param m		array of messages
 * @param n		number of messages
 * @returns		message segments transferred or error code (<1)
 */
static int stm32_i2c_transfer(struct i2c_adapter *a, struct i2c_msg *m, int n)
{
#if defined(I2C_STM32_DEBUG)
	int i, j;
#endif
	struct i2c_stm32 *c = a->algo_data;
	int ret = 0;

	/*
	 * Store the software parameters of the message.
	 * These will be used by the IRQ handler.
	 */
	c->msg = &m[0];
	c->msg_i = 0;
	c->msg_n = n;
	c->msg_status = -EBUSY;

	/*
	 * A transfer is kicked off by initiating a start condition.
	 * Actual transfer is handled by the state machine implemented
	 * in the IRQ routine.
	 */
	c->regs->cr1 |= I2C_STM32_CR1_STA;

	/*
	 * Let interrupts happen
	 */
	stm32_i2c_enable_irqs(c);

	/*
	 * Wait for the transfer to complete, one way or another
	 */
	if (wait_event_timeout(c->wait, c->msg_status != -EBUSY, 5 * HZ) == 0) {
		stm32_i2c_disable_irqs(c);
		ret = -ETIMEDOUT;
	} else {
		ret = c->msg_status;
		if (!ret)
			ret = n;
	}

	/*
	 * Stop activity on the bus
	 */
	c->regs->cr1 |= I2C_STM32_CR1_STO;

	/*
	 * Reset the bus to a known state in case of error.
	 */
	if (ret < 0 && ret != -ENODEV)
		stm32_i2c_hw_clear(c);

#if defined(I2C_STM32_DEBUG)
	for (i = 0; i < n; i++) {
		d_printk(4, "%d:flags=0x%x,addr=0x%x,len=%d\n",
			 i, m[i].flags, m[i].addr, m[i].len);
		for (j = 0; j < m[i].len; j++)
			d_printk(4, "%d=%x\n", j, m[i].buf[j]);
	}
#endif
	d_printk(3, "n=%d,ret=%d\n", n, ret);

	return ret;
}

/*
 * Adapter functionality callback
 * @param a		I2C adapter
 * @returns		OR-ed functionality flags
 */
static unsigned int stm32_i2c_functionality(struct i2c_adapter *a)
{
	d_printk(3, "ok\n" );
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/*
 * Algorithm data structure
 */
static const struct i2c_algorithm stm32_i2c_algorithm = {
	.functionality	= stm32_i2c_functionality,
	.master_xfer	= stm32_i2c_transfer,
};

/*
 * Instantiate a new instance of the I2C controller
 * @dev			I2C controller platform device
 * @returns		0->success, <0->error code
 */
static int stm32_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct i2c_stm32 *c = NULL;
	struct resource *mem;
	int bus;
	int irq;
	int ret = 0;

	/*
	 * Get the bus # from the platform device:
	 */
	bus = of_alias_get_id(np, "i2c");
	if (! (0 <= bus && bus <= 2)) {
		dev_err(dev, "invalid bus number %d\n", bus);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Get the IRQ number from the platform device
	 */
	irq = irq_of_parse_and_map(np, 0);
	if (irq <= 0) {
		dev_err(dev, "invalid IRQ number %d\n", irq);
		ret = -EINVAL;
		goto Error_release_nothing;
	}

	/*
	 * Get the register base from the platform device
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "no register base provided\n");
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Allocate the controller-private data structure
	 */
	c = kzalloc(sizeof(struct i2c_stm32), GFP_KERNEL);
	if (!c) {
		dev_err(dev, "unable to allocate memory\n");
		ret = -ENOMEM;
		goto Error_release_nothing;
	}
	c->dev = dev;
	c->bus = bus;

	/*
	 * Request a memory region for the CSR block
	 */
	if (!request_mem_region(mem->start, resource_size(mem),
				mem->name)) {
		dev_err(dev, "registers already in use\n");
		ret = -ENOMEM;
		goto Error_release_memory;
	}
	c->regs_base = mem->start;
	c->regs_size = resource_size(mem);

	/*
	 * Map in the CSR block
	 */
	c->regs = ioremap(mem->start, resource_size(mem));
	if (!c->regs) {
		dev_err(dev, "unable to map registers\n");
		ret = -EINVAL;
		goto Error_release_mem_region;
	}

	/*
	 * Get clocks & resets
	 */
	c->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(c->clk)) {
		dev_err(dev, "no clock specified\n");
		goto Error_release_regs;
	}
	c->ref_clk = clk_get_rate(c->clk);

	if (of_property_read_u32(np, "st,i2c-clk", &c->i2c_clk) < 0) {
		dev_err(dev, "no i2c_clk value specified\n");
		goto Error_release_regs;
	}

	c->rst = devm_reset_control_get(dev, NULL);
	if (IS_ERR(c->rst)) {
		dev_err(dev, "no reset controller specified\n");
		goto Error_release_regs;
	}

	/*
	 * Register interrupt handler for events
	 */
	ret = request_irq(irq, stm32_i2c_irq, 0, dev_name(dev), c);
	if (ret) {
		dev_err(dev, "request for IRQ %d failed\n", irq);
		goto Error_release_regs;
	}
	disable_irq_nosync(irq);
	c->irq = irq;

	/*
	 * Register interrupt handler for errors
	 */
	ret = request_irq(irq + 1, stm32_i2c_irq, 0, dev_name(dev), c);
	if (ret) {
		dev_err(dev, "request for IRQ %d failed\n", irq + 1);
		goto Error_release_irq1;
	}
	disable_irq_nosync(irq + 1);

	/*
	 * Link the private data to dev
	 */
	platform_set_drvdata(pdev, c);

	/*
	 * Initialize the I2C adapter data structure
	 */
	c->adap.owner = THIS_MODULE;
	c->adap.nr = bus;
	snprintf(c->adap.name, sizeof(c->adap.name), "i2c_stm32.%u", bus);
	c->adap.algo = &stm32_i2c_algorithm;
	c->adap.algo_data = c;
	c->adap.dev.parent = dev;
	c->adap.dev.of_node = np;

	/*
	 * Initialize the controller hardware
	 */
	ret = stm32_i2c_hw_init(c);
	if (ret)
		goto Error_release_irq2;

	/*
	 * Set up the wait queue
	 */
	init_waitqueue_head(&c->wait);

	/*
	 * Register the I2C adapter
	 */
	if (i2c_add_numbered_adapter(&c->adap)) {
		dev_err(dev, "unable to add adapter\n");
		ret = -ENXIO;
		goto Error_release_hw;
	}

	/*
	 * If we are here, we are successful
	 */
	dev_info(dev, "I2C Controller %s at %p,irq=%d\n",
		 dev_name(&c->adap.dev), c->regs, c->irq);
	goto Done;

	/*
	 * Error processing
	 */
Error_release_hw:
	stm32_i2c_hw_release(c);
Error_release_irq2:
	free_irq(c->irq + 1, c);
Error_release_irq1:
	free_irq(c->irq, c);
Error_release_regs:
	iounmap(c->regs);
Error_release_mem_region:
	release_mem_region(mem->start, resource_size(mem));
Error_release_memory:
	kfree(c);
	platform_set_drvdata(pdev, NULL);
Error_release_nothing:

Done:
	d_printk(1, "dev=%s,regs=%p,irq=%d,ref_clk=%d,i2c_clk=%d,ret=%d\n",
		 dev_name(dev), c ? c->regs : NULL, c ? c->irq : 0,
		 c ? c->ref_clk : 0, c ? c->i2c_clk : 0, ret);
	return ret;
}

/*
 * Shutdown of an instance of the controller device
 * @dev			I2C controller platform device
 * @returns		0->success, <0->error code
 */
static int stm32_i2c_remove(struct platform_device *pdev)
{
	struct i2c_stm32 *c  = platform_get_drvdata(pdev);
	int ret = 0;

	/*
	 * Shut the hardware down
	 */
	stm32_i2c_hw_release(c);

	/*
	 * Release kernel resources.
	 */
	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&c->adap);
	free_irq(c->irq, c);
	free_irq(c->irq + 1, c);
	iounmap(c->regs);
	release_mem_region(c->regs_base, c->regs_size);
	kfree(c);

	d_printk(1, "dev=%s,ret=%d\n", dev_name(&pdev->dev), ret);
	return ret;
}

static const struct of_device_id stm32_i2c_match[] = {
	{ .compatible = "st,stm32-i2c", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_i2c_match);

/*
 * Driver data structure
 */
static struct platform_driver stm32_i2c_drv = {
	.probe	= stm32_i2c_probe,
	.remove	= stm32_i2c_remove,
	.driver = {
		.name = "stm32-i2c",
		.of_match_table = stm32_i2c_match,
	},
};
module_platform_driver(stm32_i2c_drv);

MODULE_AUTHOR("Vladimir Khusainov, <vlad@emcraft.com>");
MODULE_DESCRIPTION("Device driver for the I2C controller of STM32");
MODULE_LICENSE("GPL");
