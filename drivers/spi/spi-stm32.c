/*
 * Device driver for the SPI controller of the STM32Fx
 *
 * Copyright 2013-2017 Emcraft Systems
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_data/dma-stm32.h>

/******************************************************************************
 * Constants and macros
 ******************************************************************************/
/*
 * Driver params
 */
#define DRIVER_NAME	"stm32-spi"
#define DRIVER_CS_MAX	8

/*
 * Debug output control. While debugging, have SPI_STM32_DEBUG defined (to
 * default verbosity level). In deployment, make sure that SPI_STM32_DEBUG is
 * undefined to avoid the performance and size overhead of debug messages.
 */
#undef SPI_STM32_DEBUG

/*
 * Some bits in various CSRs
 */
#define SPI_CR1_BIDIMODE		(1 << 15)
#define SPI_CR1_BIDIOE			(1 << 14)
#define SPI_CR1_DFF			(1 << 11)
#define SPI_CR1_SSM			(1 << 9)
#define SPI_CR1_SSI			(1 << 8)
#define SPI_CR1_SPE			(1 << 6)
#define SPI_CR1_BR(x)			((x) << 3)
#define SPI_CR1_MSTR			(1 << 2)
#define SPI_CR1_CPOL			(1 << 1)
#define SPI_CR1_CPHA			(1 << 0)

#define SPI_CR2_FRXTH			(1 << 12)
#define SPI_CR2_DS(x)			((x) << 8)
#define SPI_CR2_TXEIE			(1 << 7)
#define SPI_CR2_RXNEIE			(1 << 6)
#define SPI_CR2_ERRIE			(1 << 5)
#define SPI_CR2_TXDMAEN			(1 << 1)
#define SPI_CR2_RXDMAEN			(1 << 0)

#define SPI_SR_FTLVL_FULL		(3 << 11)
#define SPI_SR_FRE			(1 << 8)
#define SPI_SR_BSY			(1 << 7)
#define SPI_SR_OVR			(1 << 6)
#define SPI_SR_UDR			(1 << 3)
#define SPI_SR_TXE			(1 << 1)
#define SPI_SR_RXNE			(1 << 0)

/******************************************************************************
 * C-types
 ******************************************************************************/
/*
 * DMA controls
 */
struct stm32_dma_data {
	struct dma_async_tx_descriptor	*dsc;
	struct dma_chan			*chan;
	dma_cookie_t			cookie;
	struct scatterlist		sg;
	dma_addr_t			buf_dma;
	int				len_dma;
};

/*
 * Private data structure for an SPI controller instance
 */
struct spi_stm32 {
	int				bus;		/* Bus (ID) */
	struct clk			*clk;		/* Clock */
	struct reset_control		*rst;		/* Reset */
	volatile struct stm32_spi_regs __iomem *regs;	/* Registers base */
	unsigned int			speed_hz;	/* Max clock rate */
	unsigned char			stopping;	/* Is being stopped? */
	spinlock_t			lock;		/* Exclusive access */
	wait_queue_head_t		wait;		/* Wait queue */
	int				irq;		/* IRQ # */
	struct device			*dev;		/* SPI controller dev */
	struct spi_device		*slave;		/* Current SPI slave */
	struct spi_message		*msg;		/* SPI message */
	volatile int			xfer_status;	/* Xfer status */
	int				len;		/* Xfer len */
	int				wb;		/* Xfer width */
	struct spi_transfer		*tx_t;		/* Cur Tx xfer */
	struct spi_transfer		*rx_t;		/* Cur Rx xfer */

	int				tx_l;		/* Tx len */
	int				rx_l;		/* Rx len */

	int				tx_i;		/* Cur Tx index */
	int				rx_i;		/* Cur Rx index */

	int				ti;		/* Tx count */
	int				ri;		/* Rx count */

	const struct stm32_of_data	*priv;		/* STM chip specific */
	struct stm32_dma_data		dma_rx;
	struct stm32_dma_data		dma_tx;
	bool				dma_use;
	unsigned int			cs_gpio[DRIVER_CS_MAX];
	unsigned int			cs_timeout[DRIVER_CS_MAX];
};

/*
 * STM chip specific data
 */
struct stm32_of_data {
	int	(*hw_bt_set)(struct spi_stm32 *c, int bt);
};

/*
 * Description of the the STM32 SPI hardware registers
 */
struct stm32_spi_regs {
	u16	cr1;
	u16	align1;
	u16	cr2;
	u16	align2;
	u16	sr;
	u16	align3;
	u16	dr;
	u16	align4;
	u16	crcpr;
	u16	align5;
	u16	rxcrcr;
	u16	align6;
	u16	txcrcr;
	u16	align7;
	u16	i2scfgr;
	u16	align8;
	u16	i2spr;
};

/******************************************************************************
 * Local variables and prototypes
 ******************************************************************************/

static int stm32_spi_dma_rx_next(struct spi_stm32 *c);
static int stm32_spi_dma_tx_next(struct spi_stm32 *c);

static void inline stm32_spi_transfer_done(struct spi_stm32 *c);

/******************************************************************************
 * Debug stuff
 ******************************************************************************/

#if defined(SPI_STM32_DEBUG)
/*
 * Driver verbosity level: 0->silent; >0->verbose (1 to 4, growing verbosity)
 */
static int stm32_spi_debug = SPI_STM32_DEBUG;

/*
 * User can change verbosity of the driver (when loading the module,
 * or the kernel, in case the driver is linked in statically,
 * but not through a /sysfs parameter file)
 */
module_param(stm32_spi_debug, int, S_IRUGO);
MODULE_PARM_DESC(stm32_spi_debug, "SPI controller driver verbosity level");

#if !defined(MODULE)
/*
 * Parser for the boot time driver verbosity parameter
 * @param str		user defintion of the parameter
 * @returns		1->success
 */
static int __init stm32_spi_debug_setup(char * str)
{
	get_option(&str, &stm32_spi_debug);
	return 1;
}
__setup("stm32_spi_debug=", stm32_spi_debug_setup);
#endif /* !defined(MODULE) */

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (stm32_spi_debug >= level) printk(KERN_INFO "%s: " fmt,	\
					   __func__, ## args)
#else
#define d_printk(level, fmt, args...)
#endif /* defined(SPI_STM32_DEBUG) */

/******************************************************************************
 * Hw configuration functions
 ******************************************************************************/

/*
 * Hardware initialization of the SPI controller
 * @param c		controller data structure
 * @returns		0->success, <0->error code
 */
static int stm32_spi_hw_init(struct spi_stm32 *c)
{
	int ret = 0;

	/*
	 * Reset the SPI controller and then bring it out of reset.
	 * Enable the SPI controller clock.
	 */
	reset_control_assert(c->rst);
	reset_control_deassert(c->rst);
	clk_prepare_enable(c->clk);

	/*
	 * Set the master mode
	 */
	c->regs->cr1 = SPI_CR1_MSTR;

	/*
	 * Software chip select management, NSS pin can be re-used as a GPIO.
	 * Software is responsible for driving CS (which can be NSS or any
	 * other GPIO) appropriately.
	 */
	c->regs->cr1 |= SPI_CR1_SSM | SPI_CR1_SSI;

	/*
	 * Select 1-line bidirectional transmit-only mode is DMA is set
	 * for TX-only.
	 */
	if (c->dma_use && !c->dma_rx.chan)
		c->regs->cr1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;

	/*
	 * Set the transfer protocol. We are using the Motorola
	 * SPI mode, with no API to configure it to
	 * some other mode. The Motorola mode is default, so
	 * no explicit update of the registers.
	 */

	/*
	 * Enable DMA if necessary
	 */
	if (c->dma_use && c->dma_rx.chan)
		c->regs->cr2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	else if (c->dma_use)
		c->regs->cr2 |= SPI_CR2_TXDMAEN;
	else
		c->regs->cr2 |= SPI_CR2_RXNEIE;

	/*
	 * Enable interrupts
	 */
	c->regs->cr2 |= SPI_CR2_ERRIE;

	/*
	 * Enable the SPI contoller
	 */
	c->regs->cr1 |= SPI_CR1_SPE;

	d_printk(2, "bus=%d,spi_cr1=0x%x,ret=%d\n", c->bus, c->regs->cr1, ret);

	return ret;
}

/*
 * Get the number of chip selects supported by this controller
 * @param c		controller data structure
 * @returns		max chip select
 */
static int stm32_spi_hw_cs_max(struct spi_stm32 *c)
{
	/*
	 * With the software chip select management, we
	 * can actually support as many CS as there is GPIO
	 * but let's limit this to some reasonable value
	 */
	return DRIVER_CS_MAX;
}

/*
 * Set chip select
 * @param c		controller data structure
 * @param n		GPIO number for CS
 * @param activate	1->CS=low (activated); 0->CS=high (deactivated)
 * @returns		0->good,!=0->bad
 */
static inline int stm32_spi_hw_cs_set(struct spi_stm32 *c, int n, int activate)
{
	int ret = 0;

	/*
	 * Drive the CS GPIO manually
	 */
	gpio_set_value(n, !activate);

	d_printk(4, "bus=%d,cs=%d,b=%d,ret=%d\n",  c->bus, n, activate, ret);

	return ret;
}

/*
 * Set controller clock rate
 * @param c		controller data structure
 * @param spd		clock rate in Hz
 * @returns		0->good,!=0->bad
 */
static inline int stm32_spi_hw_clk_set(struct spi_stm32 *c, unsigned int spd)
{
	int i;
	unsigned int h = c->speed_hz;
	int ret = 0;

	/*
	 * Calculate the clock rate that works for this slave
	 */
	for (i = 1; i <= 8; i ++) {
		if (h / (1 << i) <= spd)
			break;
	}

	/*
	 * Can't provide a rate that is slow enough for the slave
	 */
	if (i == 9) {
		ret = -EIO;
		goto Done;
	}

	/*
	 * Set the clock rate
	 */
	c->regs->cr1 |= SPI_CR1_BR(i - 1);

Done:
	d_printk(1, "bus=%d,cnt_hz=%d,slv_hz=%d,rsl_hz=%d,"
		"i=%d,cr1=%x,ret=%d\n",
		c->bus, h, spd, h / (1 << i), i - 1,
		c->regs->cr1, ret);

	return ret;
}

/*
 * Check frame size
 * @param c		controller data structure
 * @param bt		frame size
 * @returns		0->good,!=0->bad
 */
static inline int stm32_spi_hw_bt_check(struct spi_stm32 *c, int bt)
{
	int ret = ((8 == bt) || (bt == 16)) ? 0 : 1;

	d_printk(2, "bus=%d,bt=%d,ret=%d\n", c->bus, bt, ret);

	return ret;
}

/*
 * Set frame size (making an assumption that the supplied size is
 * supported by this controller)
 * @param c		controller data structure
 * @param bt		frame size
 * @returns		0->good,!=0->bad
 */
static int stm32f24_spi_hw_bt_set(struct spi_stm32 *c, int bt)
{
	unsigned int v = c->regs->cr1;

	if (bt == 8)
		v &= ~SPI_CR1_DFF;
	else
		v |= SPI_CR1_DFF;
	c->regs->cr1 = v;

	d_printk(2, "bus=%d,bt=%d,spi_cr1=%x\n", c->bus, bt, c->regs->cr1);

	return 0;
}

static int stm32f7_spi_hw_bt_set(struct spi_stm32 *c, int bt)
{
	unsigned int v = c->regs->cr2;

	v &= ~SPI_CR2_DS(0xF);
	v |= SPI_CR2_DS(bt - 1) | (bt == 8 ? SPI_CR2_FRXTH : 0);
	c->regs->cr2 = v;

	d_printk(2, "bus=%d,bt=%d,spi_cr2=%x\n", c->bus, bt, c->regs->cr2);

	return 0;
}

/*
 * Set SPI mode
 * @param c		controller data structure
 * @param mode		mode
 * @returns		0->good;!=0->bad
 */
static inline int stm32_spi_hw_mode_set(struct spi_stm32 *c, unsigned int mode)
{
	int ret = 0;

	/*
	 * Disable the SPI contoller
	 */
	c->regs->cr1 &= ~SPI_CR1_SPE;

	/*
	 * Set the mode
	 */
	if (mode & SPI_CPHA)
		c->regs->cr1 |= SPI_CR1_CPHA;
	else
		c->regs->cr1 &= ~SPI_CR1_CPHA;

	if (mode & SPI_CPOL)
		c->regs->cr1 |= SPI_CR1_CPOL;
	else
		c->regs->cr1 &= ~SPI_CR1_CPOL;

	/*
	 * Re-enable the SPI contoller
	 */
	c->regs->cr1 |= SPI_CR1_SPE;

	d_printk(2, "bus=%d,mode=%x,spi_cr1=%x,ret=%d\n",
		 c->bus, mode, c->regs->cr1, ret);

	return ret;
}

/*
 * Is transmit FIFO full?
 * @param c		controller data structure
 * @returns		!0->full;0->not full
 */
static inline int stm32_spi_hw_txfifo_full(struct spi_stm32 *c)
{
	return (c->regs->sr & SPI_SR_FTLVL_FULL) == SPI_SR_FTLVL_FULL;
}

/*
 * Put a frame into the transmit FIFO
 * @param c		controller data structure
 * @param wb		frame size in full bytes
 * @param tx		transmit buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void stm32_spi_hw_txfifo_put(struct spi_stm32 *c,
					   int wb, const void *tx, int i)
{
	int j;
	unsigned int d = 0;
	unsigned char *p = (unsigned char *)tx;

	if (p) {
		for (j = 0; j < wb; j++) {
			d <<= 8;
			d |= p[i * wb + j];
		}
	}

	if (wb == 1)
		writeb(d, &c->regs->dr);
	else
		writew(d, &c->regs->dr);
}

/*
 * Is receive FIFO empty?
 * @param c		controller data structure
 * @returns		!0->empty,0->not empty
 */
static inline int stm32_spi_hw_rxfifo_empty(struct spi_stm32 *c)
{
	return !(c->regs->sr & SPI_SR_RXNE);
}

/*
 * Is receive FIFO overflown?
 * @param c		controller data structure
 * @returns		!0->error,0->no error
 */
static inline int stm32_spi_hw_rxfifo_error(struct spi_stm32 *c)
{
	return c->regs->sr & (SPI_SR_FRE | SPI_SR_UDR);
}

/*
 * Retrieve a frame from the receive FIFO
 * @param c		controller data structure
 * @param wb		frame size in full bytes
 * @param rx		receive buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void stm32_spi_hw_rxfifo_get(struct spi_stm32 *c, unsigned int wb,
					   void *rx, int i)
{
	unsigned char *p = rx;

	if (wb == 1) {
		u8 d = readb(&c->regs->dr);
		if (p)
			p[i] = d;
	} else {
		u16 d = readw(&c->regs->dr);
		if (p) {
			p[i * wb] = d & 0xFF;
			p[i * wb + 1] = (d >> 8) & 0xFF;
		}
	}
}

/*
 * Clean-up receive FIFO
 * @param c		controller data structure
 * @param rx		receive buf (can be NULL)
 * @param i		index of frame in buf
 */
static inline void stm32_spi_hw_rxfifo_purge(struct spi_stm32 *c)
{
	unsigned long d;

	while (c->regs->sr & SPI_SR_RXNE)
		d = c->regs->dr;
}

/*
 * Hardware shutdown of the SPI controller
 * @param c		controller data structure
 */
static void stm32_spi_hw_release(struct spi_stm32 *c)
{
	if (c->dma_use) {
		if (c->dma_tx.chan && c->dma_tx.cookie >= 0) {
			dmaengine_terminate_all(c->dma_tx.chan);
			c->dma_tx.cookie = 0;
		}
		if (c->dma_rx.chan && c->dma_rx.cookie >= 0) {
			dmaengine_terminate_all(c->dma_rx.chan);
			c->dma_rx.cookie = 0;
		}
	}

	/*
	 * Disable the SPI contoller
	 */
	c->regs->cr1 &= ~SPI_CR1_SPE;

	clk_disable_unprepare(c->clk);
	reset_control_assert(c->rst);

	d_printk(2, "bus=%d\n", c->bus);
}

/******************************************************************************
 * SPI Slave API
 ******************************************************************************/

/*
 * Prepare to transfer to a slave
 * @param c		controller data structure
 * @param s		slave data structure
 * @returns		0->success, <0->error code
 */
static int stm32_spi_prepare_for_slave(struct spi_stm32 *c,
				       struct spi_device *s)
{
	struct device *dev = &s->dev;
	unsigned int spd;
	int ret = 0;

	/*
	 * Set for this slave: frame size, clock, mode
	 */
	if (!c->priv || !c->priv->hw_bt_set ||
	    c->priv->hw_bt_set(c, s->bits_per_word)) {
		dev_err(dev, "unsupported frame size: %d\n",
			s->bits_per_word);
		ret = -EINVAL;
		goto Done;
	}
	if (stm32_spi_hw_clk_set(c, spd = min(s->max_speed_hz, c->speed_hz))) {
		dev_err(dev, "slave rate too low: %d\n", spd);
		ret = -EINVAL;
		goto Done;
	}
	if (stm32_spi_hw_mode_set(c, s->mode)) {
		dev_err(dev, "unsupported mode: %x\n", s->mode);
		ret = -EINVAL;
		goto Done;
	}

Done:
	d_printk(2, "slv=%s,ret=%d\n", dev_name(dev), ret);
	return ret;
}

/*
 * Capture a slave
 * @param c		controller data structure
 * @param s		slave data structure
 */
static void stm32_spi_capture_slave(struct spi_stm32 *c, struct spi_device *s)
{
	/*
	 * Activate CS for this slave
	 */
	if (stm32_spi_hw_cs_set(c, c->cs_gpio[s->chip_select], 1)) {
		dev_err(&c->slave->dev, "incorrect chip select: #%u(%d)\n",
			s->chip_select, c->cs_gpio[s->chip_select]);
		goto Done;
	}

Done:
	d_printk(3, "slv=%s\n", dev_name(&c->slave->dev));
}

/*
 * Release a slave
 * @param c		controller data structure
 * @param s		slave data structure
 */
static void stm32_spi_release_slave(struct spi_stm32 *c, struct spi_device *s)
{
	/*
	 * Release CS for this slave
	 */
	if (stm32_spi_hw_cs_set(c, c->cs_gpio[s->chip_select], 0)) {
		dev_err(&c->slave->dev, "incorrect chip select: #%u(%d)\n",
			s->chip_select, c->cs_gpio[s->chip_select]);
		goto Done;
	}

Done:
	d_printk(3, "slv=%s\n",
		c->slave ? dev_name(&c->slave->dev) : "");
}

/******************************************************************************
 * Interrupted SPI xfers
 ******************************************************************************/


/*
 * Advance to next Tx frame
 * @param c		controller data structure
 * @param x		xfer to advance to
 */
static void inline stm32_spi_int_tx_next(struct spi_stm32 *c)
{
	unsigned long f;

	/*
	 * CS may be deactivated because of previous 'cs_change', so force it
	 */
	stm32_spi_capture_slave(c, c->slave);

	/*
	 * If the trasmit in the current transfer
	 * has been finished, go to the next one.
	 */
	while (c->tx_i == c->tx_l) {
		c->tx_t = list_entry(c->tx_t->transfer_list.next,
				     struct spi_transfer, transfer_list);
		c->tx_l = c->tx_t->len / c->wb;
		c->tx_i = 0;
	}

	/*
	 * Put a frame to the transmit fifo
	 */
	spin_lock_irqsave(&c->lock, f);
	stm32_spi_hw_txfifo_put(c, c->wb, c->tx_t->tx_buf, c->tx_i);
	c->tx_i++;
	c->ti++;
	spin_unlock_irqrestore(&c->lock, f);
}

/*
 * Advance to next Rx frame
 * @param c		controller data structure
 * @param x		xfer to advance to
 */
static void inline stm32_spi_int_rx_next(struct spi_stm32 *c)
{
	unsigned long f;

	/*
	 * If the receive in the current transfer has been finished, go to the
	 * next one.
	 */
	while (c->rx_i == c->rx_l) {
		/*
		 * Advance to the next transfer
		 */
		c->rx_t = list_entry(c->rx_t->transfer_list.next,
				  struct spi_transfer, transfer_list);
		c->rx_l = c->rx_t->len / c->wb;
		c->rx_i = 0;
	}

	/*
	 * Read in a frame
	 */
	spin_lock_irqsave(&c->lock, f);
	stm32_spi_hw_rxfifo_get(c, c->wb, c->rx_t->rx_buf, c->rx_i);
	c->rx_i++;
	c->ri++;
	spin_unlock_irqrestore(&c->lock, f);
}

/*
 * Interrupt handler routine
 */
static irqreturn_t stm32_spi_irq(int irq, void *dev_id)
{
	struct spi_stm32 *c = dev_id;
#if defined(SPI_STM32_DEBUG)
	int sr = c->regs->sr;
#endif

	if (c->dma_use) {
		/*
		 * If DMA xfers are used, then we can got SPI IRQs in case
		 * of errors only; just notify
		 */
		if (!(c->regs->sr & SPI_SR_OVR))
			dev_err(c->dev, "SPI Error IRQ, SR=%x\n", c->regs->sr);
		goto out;
	}

	if (!stm32_spi_hw_rxfifo_empty(c)) {
		/*
		 * Read in a frame
		 */
		stm32_spi_int_rx_next(c);

		/*
		 * If the entire transfer has been received, that's it.
		 */
		if (c->ri == c->len) {
			stm32_spi_hw_rxfifo_purge(c);
			c->xfer_status = 0;
			wake_up(&c->wait);
		}
	}

	stm32_spi_transfer_done(c);

	/*
	 * Push a next frame out
	 */
	if (c->ti < c->len) {

		/*
		 * Write a frame
		 */
		while (stm32_spi_hw_txfifo_full(c));
		stm32_spi_int_tx_next(c);
	}

	d_printk(4, "ok: sr=%x\n", sr);
out:
	if (c->regs->sr & SPI_SR_OVR) {
		u32 tmp;
		stm32_spi_hw_rxfifo_get(c, c->wb, &tmp, 0);
		tmp = c->regs->sr;
	}

	return IRQ_HANDLED;
}

/*
 * Transfer a message in interrupted mode
 * @param c		controller data structure
 * @param		pointer to actual transfer length (set here)
 * @returns		0->success, <0->error code
 */
static int stm32_spi_int_xfer(struct spi_stm32 *c, int *rlen)
{
	struct spi_device *s = c->slave;
	unsigned int timeout = c->cs_timeout[s->chip_select] ?
			       c->cs_timeout[s->chip_select] : 1;
	int ret = 0;

	/*
	 * Start the transfer
	 */
	c->xfer_status = -EBUSY;
	*rlen = 0;
	stm32_spi_int_tx_next(c);

	/*
	 * Wait for the transfer to complete, one way or another
	 */
	if (wait_event_timeout(c->wait,
		c->xfer_status != -EBUSY,  timeout * HZ) == 0) {
		ret = -ETIMEDOUT;
	} else {
		ret = c->xfer_status;
		if (!ret) {
			/*
			 * Success ->
			 * Return the number of bytes actully transferred
			 */
			*rlen = c->ri;
		}
	}
	c->xfer_status = 0;

	d_printk(3, "msg=%p,len=%d,rlen=%d,ret=%d\n",
		c->msg, c->len, *rlen, ret);

	return ret;
}

/******************************************************************************
 * DMA SPI xfer
 ******************************************************************************/

static void stm32_spi_dma_xf_done(struct spi_stm32 *c)
{
	/*
	 * If rx or tx is still running - do nothing
	 */
	if ((c->dma_rx.chan && c->dma_rx.dsc) || c->dma_tx.dsc)
		goto out;

	/*
	 * If the entire transfer has been received, that's it.
	 */
	if (c->ti == c->len) {
		c->xfer_status = 0;
		wake_up(&c->wait);
		goto out;
	}

	/*
	 * Transfer next
	 */
	stm32_spi_dma_rx_next(c);
	stm32_spi_dma_tx_next(c);
out:
	return;
}

static void stm32_spi_dma_rx_done(void *arg)
{
	struct spi_stm32 *c = arg;
	struct device *dev = c->dev;
	struct stm32_dma_data *dma = &c->dma_rx;

	d_printk(1, "%s %d of %d\n", __func__, c->ri, c->len);

	dma->cookie = -EINVAL;
	dma->dsc = NULL;
	dma_sync_single_for_cpu(dev, dma->buf_dma, dma->len_dma, DMA_FROM_DEVICE);
	dma_unmap_single(dev, dma->buf_dma, dma->len_dma, DMA_FROM_DEVICE);

	stm32_spi_dma_xf_done(c);
}

static int stm32_spi_dma_rx_next(struct spi_stm32 *c)
{
	struct stm32_dma_data *dma = &c->dma_rx;
	struct device *dev = c->dev;
	dma_addr_t buf_dma;
	char *buf;
	int rv, len;

	if (!dma->chan)
		return 0;

	/*
	 * Check if Rx DMA is idle
	 */
	if (dma->dsc) {
		dev_err(dev, "%s: dma is busy\n", __func__);
		rv = -EBUSY;
		goto out;
	}

	/*
	 * If the receive in the current transfer has been finished, go to the
	 * next one.
	 */
	while (c->rx_i == c->rx_l) {
		/*
		 * Advance to the next transfer
		 */
		c->rx_t = list_entry(c->rx_t->transfer_list.next,
				     struct spi_transfer, transfer_list);
		c->rx_l = c->rx_t->len / c->wb;
		c->rx_i = 0;
	}

	buf = c->rx_t->rx_buf;
	if (buf)
		buf = &buf[c->rx_i * c->wb];
	len = (c->rx_l - c->rx_i) <= STM32_DMA_NDT_MAX ? c->rx_l - c->rx_i :
							 STM32_DMA_NDT_MAX;
	buf_dma = dma_map_single(dev, buf, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, buf_dma)) {
		dev_err(dev, "DMA mapping failed\n");
		rv = -EIO;
		goto out;
	}

	dma->dsc = dmaengine_prep_slave_single(dma->chan, buf_dma, len,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!dma->dsc) {
		dev_err(dev, "%s: failed to send with DMA\n", __func__);
		rv = -EIO;
		goto out;
	}

	d_printk(1, "ri=%d,len=%d,wb=%d,rx_i=%d,rx_l=%d,buf=%p,len=%x\n",
		 c->ri, c->len, c->wb, c->rx_i, c->rx_l, buf, len);
	c->rx_i += len;
	c->ri += len;

	dma->dsc->callback = stm32_spi_dma_rx_done;
	dma->dsc->callback_param = c;
	dma->dsc->cookie = dmaengine_submit(dma->dsc);
	dma->buf_dma = buf_dma;
	dma->len_dma = len;
	dma_async_issue_pending(dma->chan);

	rv = 0;
out:
	return rv;
}

static int stm32_spi_dma_rx_init(struct spi_stm32 *c)
{
	struct stm32_dma_data *dma = &c->dma_rx;
	struct device *dev = c->dev;
	struct dma_slave_config cfg;
	int rv;

	if (!dma->chan)
		return 0;

	/* Configure the slave DMA */
	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = DMA_DEV_TO_MEM;
	cfg.src_addr_width = c->wb;
	cfg.dst_addr_width = c->wb;
	cfg.src_addr = (u32)&c->regs->dr;
	cfg.src_maxburst = 1;
	rv = dmaengine_slave_config(dma->chan, &cfg);
	if (rv)
		dev_err(dev, "%s: configuration failed (%d)\n", __func__, rv);

	return rv;
}

static void stm32_spi_dma_tx_done(void *arg)
{
	struct spi_stm32 *c = arg;
	struct device *dev = c->dev;
	struct stm32_dma_data *dma = &c->dma_tx;

	d_printk(1, "%s %d of %d\n", __func__, c->ti, c->len);

	dma->cookie = -EINVAL;
	dma->dsc = NULL;
	dma_unmap_single(dev, dma->buf_dma, dma->len_dma, DMA_TO_DEVICE);

	stm32_spi_dma_xf_done(c);
}

static int stm32_spi_dma_tx_next(struct spi_stm32 *c)
{
	struct stm32_dma_data *dma = &c->dma_tx;
	struct device *dev = c->dev;
	dma_addr_t buf_dma;
	const char *buf;
	int rv, len;

	/*
	 * Check if DMA is idle
	 */
	if (dma->dsc) {
		dev_err(dev, "%s: dma is busy\n", __func__);
		rv = -EBUSY;
		goto out;
	}

	/*
	 * CS may be deactivated because of previous 'cs_change', so force it
	 */
	stm32_spi_capture_slave(c, c->slave);

	/*
	 * If the trasmit in the current transfer has been finished, go to the
	 * next one.
	 */
	while (c->tx_i == c->tx_l) {
		c->tx_t = list_entry(c->tx_t->transfer_list.next,
				     struct spi_transfer, transfer_list);
		c->tx_l = c->tx_t->len / c->wb;
		c->tx_i = 0;
	}

	buf = c->tx_t->tx_buf;
	if (buf)
		buf = &buf[c->tx_i * c->wb];
	len = (c->tx_l - c->tx_i) <= STM32_DMA_NDT_MAX ? c->tx_l - c->tx_i :
							 STM32_DMA_NDT_MAX;
	buf_dma = dma_map_single(dev, (void *)buf, len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, buf_dma)) {
		dev_err(dev, "DMA mapping failed\n");
		rv = -EIO;
		goto out;
	}
	dma_sync_single_for_device(dev, buf_dma, len, DMA_TO_DEVICE);

	dma->dsc = dmaengine_prep_slave_single(dma->chan, buf_dma, len,
			DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!dma->dsc) {
		dev_err(dev, "%s: failed to send with DMA\n", __func__);
		rv = -EIO;
		goto out;
	}

	d_printk(1, "ti=%d,len=%d,wb=%d,tx_i=%d,tx_l=%d,buf=%p,len=%x\n",
		 c->ti, c->len, c->wb, c->tx_i, c->tx_l, buf, len);
	c->tx_i += len;
	c->ti += len;

	dma->dsc->callback = stm32_spi_dma_tx_done;
	dma->dsc->callback_param = c;
	dma->dsc->cookie = dmaengine_submit(dma->dsc);
	dma->buf_dma = buf_dma;
	dma->len_dma = len;
	dma_async_issue_pending(dma->chan);

	rv = 0;
out:
	return rv;
}

static int stm32_spi_dma_tx_init(struct spi_stm32 *c)
{
	struct stm32_dma_data *dma = &c->dma_tx;
	struct device *dev = c->dev;
	struct dma_slave_config cfg;
	int rv;

	/* Configure the slave DMA */
	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = DMA_MEM_TO_DEV;
	cfg.src_addr_width = c->wb;
	cfg.dst_addr_width = c->wb;
	cfg.dst_addr = (u32)&c->regs->dr;
	cfg.dst_maxburst = 1;
	rv = dmaengine_slave_config(dma->chan, &cfg);
	if (rv)
		dev_err(dev, "%s: configuration failed (%d)\n", __func__, rv);

	return rv;
}

/*
 * Transfer a message in DMA mode
 * @param c		controller data structure
 * @param s		slave data structure
 * @param		pointer to actual transfer length (set here)
 * @returns		0->success, <0->error code
 */
static int stm32_spi_dma_xfer(struct spi_stm32 *c, int *rlen)
{
	struct spi_device *s = c->slave;
	u32 tm;
	int rv;

	/*
	 * Prepare DMAs
	 */
	rv = stm32_spi_dma_rx_init(c);
	if (rv)
		goto out;
	rv = stm32_spi_dma_tx_init(c);
	if (rv)
		goto out;

	/*
	 * Start the transfer
	 */
	c->xfer_status = -EBUSY;
	*rlen = 0;

	stm32_spi_dma_rx_next(c);
	stm32_spi_dma_tx_next(c);

	/*
	 * Wait for the transfer to complete, one way or another
	 */
	tm = c->cs_timeout[s->chip_select] ? c->cs_timeout[s->chip_select] : 1;
	if (wait_event_timeout(c->wait,
				c->xfer_status != -EBUSY, tm * HZ) == 0) {
		rv = -ETIMEDOUT;
		goto out;
	}

	rv = c->xfer_status;
	if (!rv)
		*rlen = c->ri;

out:
	if (rv) {
		if (c->dma_rx.chan) {
			dmaengine_terminate_all(c->dma_rx.chan);
			c->dma_rx.cookie = -EINVAL;
			c->dma_rx.dsc = NULL;
		}

		dmaengine_terminate_all(c->dma_tx.chan);
		c->dma_tx.cookie = -EINVAL;
		c->dma_tx.dsc = NULL;
	}

	c->xfer_status = 0;
	d_printk(3, "msg=%p,len=%d,rlen=%d,rv=%d\n", c->msg, c->len, *rlen, rv);

	return rv;
}


/******************************************************************************
 * SPI Master API
 ******************************************************************************/
/*
 * Prepare for a transfer
 * @param c		controller data structure
 * @param s		slave data structure
 */
static void inline stm32_spi_transfer_init(struct spi_stm32 *c)
{
	struct spi_device *s = c->slave;
	struct spi_transfer *t;

	/*
	 * Count the total length of the message.
	 */
	c->len = 0;
	c->wb = (s->bits_per_word + 7) / 8;
	list_for_each_entry(t, &c->msg->transfers, transfer_list)
		c->len += t->len;
	c->len /= c->wb;

	/*
	 * Prepare to traverse the message list.
	 * We will need to advance separately over
	 * transmit and receive data
	 */
	c->tx_t = list_entry((&c->msg->transfers)->next, struct spi_transfer,
			     transfer_list);
	c->tx_l = c->tx_t->len / c->wb;
	c->tx_i = 0;
	c->ti = 0;

	c->rx_t = list_entry((&c->msg->transfers)->next, struct spi_transfer,
			     transfer_list);
	c->rx_l = c->rx_t->len / c->wb;
	c->rx_i = 0;
	c->ri = 0;
}

/*
 * Complete frame transfer
 */
static void inline stm32_spi_transfer_done(struct spi_stm32 *c)
{
	if (!c->tx_t)
		return;

	if (c->tx_t->delay_usecs)
		udelay(c->tx_t->delay_usecs);

	if (c->tx_t->cs_change && c->rx_i == c->rx_l)
		stm32_spi_release_slave(c, c->slave);
}

/*
 * Transfer a message
 * @param master	master descriptor
 * @param msg		message
 * @returns		0 -> success, negative error code -> error
 */
static int stm32_spi_transfer_message(struct spi_master *master,
				      struct spi_message *msg)
{
#if defined(SPI_STM32_DEBUG)
	struct spi_transfer *t;
#endif
	struct spi_stm32 *c = spi_master_get_devdata(master);
	struct spi_device *s = msg->spi;
	int rlen = 0;
	int ret = 0;

	/*
	 * Check that the controller is still running
	 */
	if (c->stopping) {
		ret = -ESHUTDOWN;
		goto Exit;
	}

	/*
	 * Check if the current slave of this controller is
	 * the message's SPI device, and if not, set
	 * the speed and mode select for the slave
	 */
	if (c->slave != s) {
		c->slave = s;
		ret = stm32_spi_prepare_for_slave(c, s);
		if (ret) {
			c->slave = NULL;
			goto Done;
		}
	}

	/*
	 * Activate chip select for the slave
	 */
	stm32_spi_capture_slave(c, s);

	/*
	 * Transfer the message over the wire
	 */
	c->msg = msg;
	stm32_spi_transfer_init(c);
	ret = c->dma_use ? stm32_spi_dma_xfer(c, &rlen) :
			   stm32_spi_int_xfer(c, &rlen);

	msg->actual_length = ret ? 0 : rlen;
	msg->status = ret;
	spi_finalize_current_message(master);
Done:
	/*
	 * Release chip select for the slave
	 */
	stm32_spi_release_slave(c, s);

Exit:
#if defined(SPI_STM32_DEBUG)
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		int i;
		char * p;

		d_printk(3, "t=%p,tx=%p,rx=%p,len=%d,rlen=%d\n",
		         t, t->tx_buf, t->rx_buf, t->len, msg->actual_length);
		if (rlen < 10) {
			p = (char *) (t->tx_buf ? t->tx_buf : t->rx_buf);
			for (i = 0; i < t->len; i++) {
				d_printk(4, "%02x ", p[i]);
			}
		}
	}
#endif
	return ret;
}

/*
 * Set up the SPI controller for a specified SPI slave device
 * @param s		SPI slave
 * @returns		0->success, <0->error code
 */
static int stm32_spi_setup(struct spi_device *s)
{
	struct spi_stm32 *c = spi_master_get_devdata(s->master);
	struct device *dev = &s->dev;
	int ret = 0;

	/*
	 * Check that the controller is still running
	 */
	if (c->stopping) {
		ret = -ESHUTDOWN;
		goto Done;
	}

	/*
	 * Check the width of transfer for this device
	 */
	if (stm32_spi_hw_bt_check(c, s->bits_per_word)) {
		dev_err(dev, "unsupported bits per word %d\n",
			s->bits_per_word);
		ret = -EINVAL;
		goto Done;
	}

	/*
	 * Make sure Chip Select is inactive for this slave
	 */
	stm32_spi_release_slave(c, s);

Done:
	d_printk(1, "slv=%s,spd=%d,cs=#%u(%d),bt=%d,md=0x%x,ret=%d\n",
		 dev_name(dev), s->max_speed_hz,
		 s->chip_select, c->cs_gpio[s->chip_select],
		 s->bits_per_word, s->mode, ret);
	return ret;
}

/*
 * Clean up the SPI controller after a specified SPI slave device
 * @param s		SPI slave
 */
static void stm32_spi_cleanup(struct spi_device *s)
{
	d_printk(1, "slv=%s\n", dev_name(&s->dev));
}

/******************************************************************************
 * Platform device API
 ******************************************************************************/

/*
 * Instantiate an SPI controller
 * @dev			SPI controller platform device
 * @returns		0->success, <0->error code
 */
static int stm32_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_dev;
	struct stm32_dma_data *dma;
	struct spi_master *m = NULL;
	struct spi_stm32 *c = NULL;
	struct resource *mem;
	int i, bus, irq, cs_num;
	int ret = 0;

	/*
	 * Get the bus # from the platform device:
	 * [0,1]->hard-core SPI contorller of SmartFusion;
	 * [2-9]->soft-IP SPI controller specific to a custom design.
	 */
	bus = of_alias_get_id(np, "spi");
	if (!(0 <= bus && bus <= 10)) {
		dev_err(dev, "invalid SPI controller %d\n", bus);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Get the IRQ number from the platform device
	 */
	irq = irq_of_parse_and_map(np, 0);
	if (irq < 0) {
		dev_err(dev, "invalid IRQ %d for SPI contoller %d\n",
			irq, bus);
		ret = irq;
		goto Error_release_nothing;
	}

	/*
	 * Get the register base from the platform device
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "no register base for SPI controller %d\n", bus);
		ret = -ENXIO;
		goto Error_release_nothing;
	}

	/*
	 * Allocate an SPI master, and get pointer to the controller-specific
	 * data structure
	 */
	m = spi_alloc_master(dev, sizeof *c);
	if (!m) {
		dev_err(dev, "unable to allocate master for "
			"SPI controller %d\n", bus);
		ret = -ENOMEM;
		goto Error_release_nothing;
	}
	c = spi_master_get_devdata(m);
	c->dev = dev;

	of_dev = of_match_node(dev->driver->of_match_table, np);
	if (!of_dev) {
		dev_err(dev, "unable to get match in of table\n");
		ret = -EINVAL;
		goto Error_release_master;
	}
	c->priv = of_dev->data;

	cs_num = of_gpio_named_count(np, "cs-gpios");
	if (cs_num < 0 || cs_num > stm32_spi_hw_cs_max(c)) {
		dev_err(dev, "none or too much CS specified (%d)\n", cs_num);
		ret = -EINVAL;
		goto Error_release_master;
	}
	for (i = 0; i < cs_num; i++) {
		ret = of_get_named_gpio(np, "cs-gpios", i);
		if (ret < 0) {
			dev_err(dev, "failed to get csgpio#%u (%d)\n",
				i, ret);
			goto Error_release_master;
		}
		c->cs_gpio[i] = ret;
		ret = devm_gpio_request_one(dev, c->cs_gpio[i],
					    GPIOF_OUT_INIT_LOW, DRIVER_NAME);
		if (ret < 0) {
			dev_err(dev, "failed to configure csgpio#%u (%d)\n",
				i, ret);
			goto Error_release_master;
		}
	}

	c->dma_use = false;
	if (of_get_property(np, "dmas", NULL) &&
	    of_get_property(np, "st,use-dma", NULL)) {
		dma = &c->dma_tx;
		memset(dma, 0, sizeof(struct stm32_dma_data));
		dma->chan = dma_request_slave_channel(dev, "tx");
		if (!dma->chan) {
			dev_err(dev, "%s: no dma-tx found\n", __func__);
			goto Error_release_dma;
		}

		dma = &c->dma_rx;
		memset(dma, 0, sizeof(struct stm32_dma_data));
		dma->chan = dma_request_slave_channel(dev, "rx");
		if (!dma->chan) {
			dev_warn(dev, "%s: no dma-rx found; switching to tx-only\n", __func__);
		}

		c->dma_use = true;
	}

	if (of_property_count_u32_elems(np, "timeouts") != cs_num) {
		dev_warn(dev, "cs(%d) and timeout(%d) arrays mismatch\n",
			 cs_num, of_property_count_u32_elems(np, "timeouts"));
	}

	if (of_property_read_u32_array(np, "timeouts", c->cs_timeout, cs_num)) {
		dev_warn(dev, "failed to get timeouts\n");
		memset(c->cs_timeout, 0, sizeof(c->cs_timeout));
	}

	/*
	 * Set up:
	 * - the bus number and node so that platform can set up SPI slave
	 *   devices on this bus;
	 * - SPI modes understood by this driver (mode_bits);
	 * - number of chip selects supported by the controller (num_chipselect)
	 * - SPI slave action calbacks
	 */
	m->dev.of_node = np;
	m->bus_num = c->bus = bus;
	m->mode_bits = SPI_CPOL | SPI_CPHA;
	m->num_chipselect = cs_num;
	m->setup = stm32_spi_setup;
	m->cleanup = stm32_spi_cleanup;
	m->transfer_one_message = stm32_spi_transfer_message;

	/*
	 * Map in the controller registers
	 */
	c->regs = ioremap(mem->start, resource_size(mem));
	if (!c->regs) {
		dev_err(dev, "unable to map registers for "
			"SPI controller %d, base=%08x\n", bus, mem->start);
		ret = -EINVAL;
		goto Error_release_dma;
	}

	/*
	 * Register interrupt handler
	 */
	ret = request_irq(irq, stm32_spi_irq, 0, dev_name(dev), c);
	if (ret) {
		dev_err(dev, "request irq %d failed for "
			"SPI controller %d\n", irq, bus);
		goto Error_release_regs;
	}
	c->irq = irq;

	/*
	 * Set up the wait queue
	 */
	init_waitqueue_head(&c->wait);

	/*
	 * Set up the lock
	 */
	spin_lock_init(&c->lock);
	c->xfer_status = 0;

	/*
	 * Get clocks & resets
	 */
	c->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(c->clk)) {
		dev_err(dev, "no clock specified\n");
		goto Error_release_irq;
	}
	c->speed_hz = clk_get_rate(c->clk);

	c->rst = devm_reset_control_get(dev, NULL);
	if (IS_ERR(c->rst)) {
		dev_err(dev, "no reset controller specified\n");
		goto Error_release_irq;
	}

	/*
	 * Initialize the controller hardware
	 */
	if (stm32_spi_hw_init(c)) {
		dev_err(dev, "unable to initialize hardware for "
			"SPI controller %d\n", bus);
		ret = -ENXIO;
		goto Error_release_irq;
	}

	/*
	 * We will be running soon
	 */
	c->slave = NULL;
	c->stopping = 0;

	/*
	 * Register the SPI controller
	 */
	ret = spi_register_master(m);
	if (ret) {
		dev_err(dev, "unable to register master "
			"for SPI controller %d\n", bus);
		goto Error_release_hardware;
	}

	/*
	 * Remember the master in the platform device.
	 * We are going to need that pointer when we
	 * are doing removal on the platform device.
	 */
	platform_set_drvdata(pdev, m);

	/*
	 * If we are here, we are successful
	 */
	dev_info(dev, "SPI Controller %d at %p,irq=%d,hz=%d\n",
		 m->bus_num, c->regs, c->irq, c->speed_hz);

	goto Done;

	/*
	 * Error processing
	 */
Error_release_hardware:
	stm32_spi_hw_release(c);
Error_release_irq:
	free_irq(c->irq, c);
Error_release_regs:
	iounmap(c->regs);
Error_release_dma:
	if (c->dma_use) {
		if (c->dma_tx.chan) {
			dma_release_channel(c->dma_tx.chan);
			c->dma_tx.chan = NULL;
		}
		if (c->dma_rx.chan) {
			dma_release_channel(c->dma_rx.chan);
			c->dma_rx.chan = NULL;
		}
	}
Error_release_master:
	spi_master_put(m);
	platform_set_drvdata(pdev, NULL);
Error_release_nothing:

	/*
	 * Exit point
	 */
Done:
	d_printk(1, "dev=%s,regs=%p,irq=%d,hz=%d,ret=%d\n", dev_name(dev),
		c? c->regs : NULL, c ? c->irq : 0, c ? c->speed_hz : 0, ret);

	return ret;
}

/*
 * Shutdown of an instance of the controller device
 * @dev			SPI controller platform device
 * @returns		0->success
 */
static int stm32_spi_remove(struct platform_device *pdev)
{
	struct spi_master *m  = platform_get_drvdata(pdev);
	struct spi_stm32 *c = spi_master_get_devdata(m);

	c->stopping = 1;

	/*
	 * Release kernel resources.
	 */
	spi_unregister_master(m);
	free_irq(c->irq, c);
	iounmap(c->regs);
	spi_master_put(m);
	platform_set_drvdata(pdev, NULL);

	/*
	 * Shut the hardware down
	 */
	stm32_spi_hw_release(c);

	if (c->dma_use) {
		if (c->dma_tx.chan) {
			dma_release_channel(c->dma_tx.chan);
			c->dma_tx.chan = NULL;
		}

		if (c->dma_rx.chan) {
			dma_release_channel(c->dma_rx.chan);
			c->dma_rx.chan = NULL;
		}
	}

	/*
	 * Exit point
	 */
	d_printk(1, "dev=%s\n", dev_name(&pdev->dev));

	return 0;
}

static const struct stm32_of_data stm32f2_of_data = {
	.hw_bt_set	= stm32f24_spi_hw_bt_set,
};

static const struct stm32_of_data stm32f4_of_data = {
	.hw_bt_set	= stm32f24_spi_hw_bt_set,
};

static const struct stm32_of_data stm32f7_of_data = {
	.hw_bt_set	= stm32f7_spi_hw_bt_set,
};

static const struct of_device_id stm32_spi_match[] = {
	{ .compatible = "st,stm32f2-spi", .data = &stm32f2_of_data },
	{ .compatible = "st,stm32f4-spi", .data = &stm32f4_of_data },
	{ .compatible = "st,stm32f7-spi", .data = &stm32f7_of_data },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_spi_match);

/*
 * Platform driver data structure
 */
static struct platform_driver stm32_spi_drv = {
	.probe	= stm32_spi_probe,
	.remove	= stm32_spi_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = stm32_spi_match,
	},
};
module_platform_driver(stm32_spi_drv);

MODULE_AUTHOR("Vladimir Khusainov, <vlad@emcraft.com>");
MODULE_DESCRIPTION("Device driver for the STM32 SPI controller");
MODULE_LICENSE("GPL");
