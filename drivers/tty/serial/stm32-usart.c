/*
 * Copyright (C) Maxime Coquelin 2015
 * Authors:  Maxime Coquelin <mcoquelin.stm32@gmail.com>
 *	     Gerald Baeza <gerald.baeza@st.com>
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Inspired by st-asc.c from STMicroelectronics (c)
 */

#if defined(CONFIG_SERIAL_STM32_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-direction.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/spinlock.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <linux/tty.h>
#include <linux/gpio/consumer.h>

#include "stm32-usart.h"

static struct stm32_port stm32_ports[STM32_MAX_PORTS];
static struct uart_driver stm32_usart_driver;

static void stm32_stop_tx(struct uart_port *port);
static void stm32_transmit_chars(struct uart_port *port);

static inline struct stm32_port *to_stm32_port(struct uart_port *port)
{
	return container_of(port, struct stm32_port, port);
}

static void stm32_set_bits(struct uart_port *port, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val |= bits;
	writel_relaxed(val, port->membase + reg);
}

static void stm32_clr_bits(struct uart_port *port, u32 reg, u32 bits)
{
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val &= ~bits;
	writel_relaxed(val, port->membase + reg);
}

/*
 * Check if overflows happens. If not then return the number of
 * bytes remained to receive, and zero otherwise.
 */
static int stm32_rx_dma_residue(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct dma_tx_state state;
	enum dma_status status;
	int pos, num;

	/* Get current writer (DMA) position in the circular buffer */
	status = dmaengine_tx_status(stm32_port->rx_ch,
				     stm32_port->rx_ch->cookie, &state);
	if (status != DMA_IN_PROGRESS) {
		num = 0;
		goto out;
	}

	/* Update reader absolute position */
	if (stm32_port->rx_rd_prv < stm32_port->rx_rd_cur) {
		stm32_port->rx_rd_abs += RX_BUF_L - stm32_port->rx_rd_cur +
					 stm32_port->rx_rd_prv;
	} else {
		stm32_port->rx_rd_abs += stm32_port->rx_rd_prv -
					 stm32_port->rx_rd_cur;
	}
	stm32_port->rx_rd_prv = stm32_port->rx_rd_cur;

	/* Update writer absolute position */
	if (stm32_port->rx_wr_prv < state.residue) {
		stm32_port->rx_wr_abs += RX_BUF_L - state.residue +
					 stm32_port->rx_wr_prv;
	} else {
		stm32_port->rx_wr_abs += stm32_port->rx_wr_prv -
					 state.residue;
	}
	stm32_port->rx_wr_prv = state.residue;

	/* Check if writer overflows the circular rx buffer */
	pos = stm32_port->rx_wr_ovf ? stm32_port->rx_wr_ovf:
				      stm32_port->rx_rd_abs;
	if (stm32_port->rx_wr_abs - pos > RX_BUF_L) {
		dev_warn(port->dev, "rx buffer overflow, %d bytes lost\n",
			 stm32_port->rx_wr_abs - pos);
		stm32_port->rx_wr_ovf = stm32_port->rx_wr_abs;
	}

	if (stm32_port->rx_wr_ovf) {
		num = 0;
		goto out;
	}

	/* No overflow */
	num = stm32_port->rx_wr_abs - stm32_port->rx_rd_abs;

	/* Reset absolute pointers if equal */
	if (!num && (stm32_port->rx_wr_abs > RX_BUF_L)) {
		stm32_port->rx_wr_abs = stm32_port->rx_wr_abs % RX_BUF_L;
		stm32_port->rx_rd_abs = stm32_port->rx_wr_abs;
	}
out:
	return num;
}

static int stm32_pending_rx(struct uart_port *port, u32 *sr, bool threaded)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	unsigned long flags;
	int num;

	*sr = readl_relaxed(port->membase + ofs->isr);

	if (threaded && stm32_port->rx_ch) {
		local_irq_save(flags);
		num = stm32_rx_dma_residue(port);
		if (stm32_port->rx_wr_ovf) {
			/* Shift reader */
			stm32_port->rx_rd_abs = stm32_port->rx_wr_ovf;
			stm32_port->rx_wr_ovf = 0;

			stm32_port->rx_rd_cur = stm32_port->rx_rd_prv =
				RX_BUF_L - (stm32_port->rx_rd_abs % RX_BUF_L);

			/* Clear overflow */
			num = stm32_rx_dma_residue(port);
		}
		local_irq_restore(flags);
		return num;
	} else if (*sr & USART_SR_RXNE) {
		return 1;
	}
	return 0;
}

static unsigned long stm32_get_char(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	unsigned long c;

	if (stm32_port->rx_ch) {
		c = stm32_port->rx_buf[RX_BUF_L - stm32_port->rx_rd_cur--];
		if (stm32_port->rx_rd_cur == 0)
			stm32_port->rx_rd_cur = RX_BUF_L;
	} else {
		c = readl_relaxed(port->membase + ofs->rdr);
		/* apply RDR data mask */
		c &= stm32_port->rdr_mask;
	}

	return c;
}

static void stm32_receive_chars(struct uart_port *port, bool threaded)
{
	struct tty_port *tport = &port->state->port;
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	unsigned long c;
	int cnt_done = 0, cnt_recv = 0;
	u32 sr = 0;
	char flag;

	if (port->irq_wake)
		pm_wakeup_event(tport->tty->dev, 0);

	while (1) {
		/* Update pending count if we've overflowed */
		if (stm32_port->rx_wr_ovf)
			cnt_recv = 0;

		/* Get number of bytes received so far */
		if (!cnt_recv--) {
			cnt_recv = stm32_pending_rx(port, &sr, threaded);
			if (!cnt_recv--)
				break;
		}

		/* To avoid rxing too long process one period maximum */
		if (cnt_done >= RX_BUF_P) {
			schedule_work(&stm32_port->rx_work);
			break;
		}

		sr |= USART_SR_DUMMY_RX;
		c = stm32_get_char(port);
		flag = TTY_NORMAL;
		port->icount.rx++;
		cnt_done++;

		if (sr & USART_SR_ERR_MASK) {
			cnt_recv = 0;
			if (sr & USART_SR_LBD) {
				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			} else if (sr & USART_SR_ORE) {
				if (ofs->icr != UNDEF_REG)
					writel_relaxed(USART_ICR_ORECF,
						       port->membase +
						       ofs->icr);
				port->icount.overrun++;
			} else if (sr & USART_SR_PE) {
				port->icount.parity++;
			} else if (sr & USART_SR_FE) {
				port->icount.frame++;
			}

			sr &= port->read_status_mask;

			if (sr & USART_SR_LBD)
				flag = TTY_BREAK;
			else if (sr & USART_SR_PE)
				flag = TTY_PARITY;
			else if (sr & USART_SR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, c))
			continue;
		uart_insert_char(port, sr, USART_SR_ORE, c, flag);
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(tport);
	spin_lock(&port->lock);
}

static void stm32_rx_dma_work(struct work_struct *work)
{
	struct stm32_port *stm32_port = container_of(work,
					struct stm32_port, rx_work);
	struct uart_port *port = &stm32_port->port;

	spin_lock(&port->lock);
	stm32_receive_chars(port, true);
	spin_unlock(&port->lock);
}

static void stm32_tx_work(struct work_struct *work)
{
	struct stm32_port *stm32_port = container_of(work,
					struct stm32_port, tx_work);
	struct uart_port *port = &stm32_port->port;
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	if (!stm32_port->tx_busy) {
		if (port->rs485.flags & SER_RS485_ENABLED) {
			if (stm32_port->de) {
				int v;
				v = (port->rs485.flags &
				     SER_RS485_RTS_ON_SEND) ? 1 : 0;
				gpiod_set_value(stm32_port->de, v);
			}

			if (!(port->rs485.flags & SER_RS485_RX_DURING_TX))
				stm32_clr_bits(port, ofs->cr1, USART_CR1_RE);

			if (port->rs485.delay_rts_before_send > 0)
				mdelay(port->rs485.delay_rts_before_send);
		}
		stm32_port->tx_busy = true;
	}

	stm32_transmit_chars(port);
}

static void stm32_rs_de_process(struct stm32_port *stm32_port)
{
	struct uart_port *port = &stm32_port->port;
	u32 tot_us = 1000000;
	u32 slp_us = 10;
	unsigned int isr;
	int v;
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	if (stm32_port->tx_ch) {
		ktime_t timeout = ktime_add_us(ktime_get(),
					       tot_us);
		v = 0;
		while (1) {
			if (!stm32_port->tx_dma_busy)
				break;

			if (ktime_compare(ktime_get(),
					  timeout) > 0) {
				v = -ETIMEDOUT;
				break;
			}

			udelay(slp_us);
		}
	} else {
		v = readl_relaxed_poll_timeout_atomic(
			port->membase + ofs->isr, isr,
			(isr & USART_SR_TC), slp_us, tot_us);
	}
	if (v)
		dev_err(port->dev, "rs485 tx timeout (%x)\n",
			readl_relaxed(port->membase + ofs->isr));

	if (port->rs485.delay_rts_after_send > 0)
		mdelay(port->rs485.delay_rts_after_send);

	if (!(port->rs485.flags & SER_RS485_RX_DURING_TX))
		stm32_set_bits(port, ofs->cr1, USART_CR1_RE);

	if (stm32_port->de) {
		v = (port->rs485.flags &
		     SER_RS485_RTS_AFTER_SEND) ? 1 : 0;
		gpiod_set_value(stm32_port->de, v);
	}

	stm32_port->tx_busy = false;
}

static void stm32_rx_dma_complete(void *arg)
{
	struct uart_port *port = arg;
	struct stm32_port *stm32_port = to_stm32_port(port);

	stm32_rx_dma_residue(port);

	schedule_work(&stm32_port->rx_work);
}

static void stm32_tx_dma_complete(void *arg)
{
	struct uart_port *port = arg;
	struct stm32_port *stm32port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32port->info->ofs;
	unsigned int isr;
	int ret;

	ret = readl_relaxed_poll_timeout_atomic(port->membase + ofs->isr,
						isr,
						(isr & USART_SR_TC),
						10, 100000);
	if (ret)
		dev_err(port->dev, "terminal count not set\n");

	if (ofs->icr == UNDEF_REG)
		stm32_clr_bits(port, ofs->isr, USART_SR_TC);
	else
		stm32_set_bits(port, ofs->icr, USART_CR_TC);

	stm32_clr_bits(port, ofs->cr3, USART_CR3_DMAT);
	stm32port->tx_dma_busy = false;

	/* Let's see if we have pending data to send */
	schedule_work(&stm32port->tx_work);
}

static void stm32_transmit_chars_pio(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int isr;

	if (stm32_port->tx_dma_busy) {
		stm32_clr_bits(port, ofs->cr3, USART_CR3_DMAT);
		stm32_port->tx_dma_busy = false;
	}

	if (in_interrupt()) {
		isr = readl_relaxed(port->membase + ofs->isr);

		if (!(isr & USART_SR_TXE))
			dev_err(port->dev, "tx empty not set\n");
	}

	writel_relaxed(xmit->buf[xmit->tail], port->membase + ofs->tdr);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	port->icount.tx++;

	stm32_set_bits(port, ofs->cr1, USART_CR1_TXEIE);
}

static void stm32_transmit_chars_dma(struct uart_port *port)
{
	struct stm32_port *stm32port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32port->info->ofs;
	struct circ_buf *xmit = &port->state->xmit;
	struct dma_async_tx_descriptor *desc = NULL;
	dma_cookie_t cookie;
	unsigned int count;

	if (stm32port->tx_dma_busy)
		return;

	stm32port->tx_dma_busy = true;

	count = uart_circ_chars_pending(xmit);

	if (count > TX_BUF_L)
		count = TX_BUF_L;

	if (xmit->tail < xmit->head) {
		memcpy(&stm32port->tx_buf[0], &xmit->buf[xmit->tail], count);
	} else {
		size_t one = UART_XMIT_SIZE - xmit->tail;
		size_t two;

		if (one > count)
			one = count;
		two = count - one;

		memcpy(&stm32port->tx_buf[0], &xmit->buf[xmit->tail], one);
		if (two)
			memcpy(&stm32port->tx_buf[one], &xmit->buf[0], two);
	}

	desc = dmaengine_prep_slave_single(stm32port->tx_ch,
					   stm32port->tx_dma_buf,
					   count,
					   DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT);

	if (!desc) {
		dev_err(port->dev, "tx dma failed\n");
		return;
	}

	desc->callback = stm32_tx_dma_complete;
	desc->callback_param = port;

	/* Push current DMA TX transaction in the pending queue */
	cookie = dmaengine_submit(desc);

	/* Issue pending DMA TX requests */
	dma_async_issue_pending(stm32port->tx_ch);

	stm32_clr_bits(port, ofs->isr, USART_SR_TC);
	stm32_set_bits(port, ofs->cr3, USART_CR3_DMAT);

	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	port->icount.tx += count;
}

static void stm32_transmit_chars(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	struct circ_buf *xmit = &port->state->xmit;

	if (port->x_char) {
		if (stm32_port->tx_dma_busy)
			stm32_clr_bits(port, ofs->cr3, USART_CR3_DMAT);
		writel_relaxed(port->x_char, port->membase + ofs->tdr);
		port->x_char = 0;
		port->icount.tx++;
		if (stm32_port->tx_dma_busy)
			stm32_set_bits(port, ofs->cr3, USART_CR3_DMAT);
		return;
	}

	if (uart_tx_stopped(port)) {
		stm32_stop_tx(port);
		return;
	}

	if (uart_circ_empty(xmit)) {
		stm32_stop_tx(port);
		return;
	}

	if (stm32_port->tx_ch)
		stm32_transmit_chars_dma(port);
	else if (!stm32_port->tx_started) {
		stm32_port->tx_started = true;
		stm32_transmit_chars_pio(port);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static irqreturn_t stm32_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	u32 sr;

	sr = readl_relaxed(port->membase + ofs->isr);

	if ((sr & USART_SR_RXNE) && !(stm32_port->rx_ch)) {
		spin_lock(&port->lock);
		stm32_receive_chars(port, false);
		spin_unlock(&port->lock);
	}

	if ((sr & USART_SR_TXE) && !(stm32_port->tx_ch)) {
		struct circ_buf *xmit = &port->state->xmit;

		if (!uart_circ_empty(xmit))
			stm32_transmit_chars_pio(port);

		if (uart_tx_stopped(port) || uart_circ_empty(xmit)) {
			stm32_stop_tx(port);
			uart_write_wakeup(port);
		} else if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
			uart_write_wakeup(port);
		}
	}

	if (stm32_port->rx_ch) {
		/* Clear IDLE interrupt */
		if (ofs->icr != UNDEF_REG) {
			writel_relaxed(USART_ICR_IDLECF,
					port->membase + ofs->icr);
		} else {
			readl_relaxed(port->membase + ofs->rdr);
		}

		schedule_work(&stm32_port->rx_work);
	}

	return IRQ_HANDLED;
}

static int stm32_config_rs485(struct uart_port *port, struct serial_rs485 *cfg)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	int v;

	port->rs485 = *cfg;

	if (!(port->rs485.flags & SER_RS485_ENABLED)) {
		if (stm32_port->de)
			gpiod_direction_input(stm32_port->de);
	} else {
		if (stm32_port->de) {
			v = (port->rs485.flags &
			     SER_RS485_RTS_AFTER_SEND) ? 1 : 0;
			gpiod_direction_output(stm32_port->de, v);
		}
	}

	return 0;
}

static unsigned int stm32_tx_empty(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	return readl_relaxed(port->membase + ofs->isr) & USART_SR_TXE;
}

static void stm32_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	if ((mctrl & TIOCM_RTS) && (port->status & UPSTAT_AUTORTS))
		stm32_set_bits(port, ofs->cr3, USART_CR3_RTSE);
	else
		stm32_clr_bits(port, ofs->cr3, USART_CR3_RTSE);
}

static unsigned int stm32_get_mctrl(struct uart_port *port)
{
	/* This routine is used to get signals of: DCD, DSR, RI, and CTS */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS | TIOCM_RTS;
}

/* Transmit stop */
static void stm32_stop_tx(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	stm32_clr_bits(port, ofs->cr1, USART_CR1_TXEIE);

	if (stm32_port->tx_busy) {
		if (port->rs485.flags & SER_RS485_ENABLED)
			stm32_rs_de_process(stm32_port);
		else
			stm32_port->tx_busy = false;
	}
	stm32_port->tx_started = false;
}

/* There are probably characters waiting to be transmitted. */
static void stm32_start_tx(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct circ_buf *xmit = &port->state->xmit;

	if (uart_circ_empty(xmit))
		return;

	schedule_work(&stm32_port->tx_work);
}

/* Throttle the remote when input buffer is about to overflow. */
static void stm32_throttle(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	stm32_clr_bits(port, ofs->cr1, stm32_port->rx_ie);
	spin_unlock_irqrestore(&port->lock, flags);
}

/* Unthrottle the remote, the input buffer can now accept data. */
static void stm32_unthrottle(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	stm32_set_bits(port, ofs->cr1, stm32_port->rx_ie);
	spin_unlock_irqrestore(&port->lock, flags);
}

/* Receive stop */
static void stm32_stop_rx(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	stm32_clr_bits(port, ofs->cr1, stm32_port->rx_ie);
}

/* Handle breaks - ignored by us */
static void stm32_break_ctl(struct uart_port *port, int break_state)
{
}

static int stm32_startup(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	u32 val;

	val = stm32_port->rx_ie | USART_CR1_TE | USART_CR1_RE;
	stm32_set_bits(port, ofs->cr1, val);

	if (stm32_port->link)
		gpiod_set_value(stm32_port->link, 1);

	return 0;
}

static void stm32_shutdown(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	struct stm32_usart_config *cfg = &stm32_port->info->cfg;
	u32 val;

	if (stm32_port->link)
		gpiod_set_value(stm32_port->link, 0);

	val = stm32_port->rx_ie;
	val |= USART_CR1_TXEIE | USART_CR1_TE | USART_CR1_RE;
	val |= BIT(cfg->uart_enable_bit);
	stm32_clr_bits(port, ofs->cr1, val);
}

unsigned int stm32_get_databits(struct ktermios *termios)
{
	unsigned int bits;

	tcflag_t cflag = termios->c_cflag;

	switch (cflag & CSIZE) {
	/*
	 * CSIZE settings are not necessarily supported in hardware.
	 * CSIZE unsupported configurations are handled here to set word length
	 * to 8 bits word as default configuration and to print debug message.
	 */
	case CS5:
		bits = 5;
		break;
	case CS6:
		bits = 6;
		break;
	case CS7:
		bits = 7;
		break;
	/* default including CS8 */
	default:
		bits = 8;
		break;
	}

	return bits;
}

static void stm32_set_termios(struct uart_port *port, struct ktermios *termios,
			    struct ktermios *old)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	struct stm32_usart_config *cfg = &stm32_port->info->cfg;
	unsigned int baud, bits;
	u32 usartdiv, mantissa, fraction, oversampling;
	tcflag_t cflag = termios->c_cflag;
	u32 cr1, cr2, cr3;
	unsigned long flags;

	if (!stm32_port->hw_flow_control)
		cflag &= ~CRTSCTS;

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 8);

	spin_lock_irqsave(&port->lock, flags);

	/* Stop serial port and reset value */
	writel_relaxed(0, port->membase + ofs->cr1);

	cr1 = stm32_port->rx_ie;
	cr1 |= USART_CR1_TE | USART_CR1_RE;
	cr1 |= BIT(cfg->uart_enable_bit);
	cr2 = 0;
	cr3 = 0;

	if (cflag & CSTOPB)
		cr2 |= USART_CR2_STOP_2B;

	bits = stm32_get_databits(termios);
	stm32_port->rdr_mask = (BIT(bits) - 1);

	if (cflag & PARENB) {
		bits++;
		cr1 |= USART_CR1_PCE;
	}

	/*
	 * Word length configuration:
	 * CS8 + parity, 9 bits word aka [M1:M0] = 0b01
	 * CS7 or (CS6 + parity), 7 bits word aka [M1:M0] = 0b10
	 * CS8 or (CS7 + parity), 8 bits word aka [M1:M0] = 0b00
	 * M0 and M1 already cleared by cr1 initialization.
	 */
	if (bits == 9)
		cr1 |= USART_CR1_M0;
	else if ((bits == 7) && cfg->has_7bits_data)
		cr1 |= USART_CR1_M1;
	else if (bits != 8)
		dev_dbg(port->dev, "Unsupported data bits config: %u bits\n"
			, bits);

	if (cflag & PARODD)
		cr1 |= USART_CR1_PS;

	port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
	if (cflag & CRTSCTS) {
		port->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
		cr3 |= USART_CR3_CTSE;
		cr3 |= USART_CR3_RTSE;
	}

	usartdiv = DIV_ROUND_CLOSEST(port->uartclk, baud);

	/*
	 * The USART supports 16 or 8 times oversampling.
	 * By default we prefer 16 times oversampling, so that the receiver
	 * has a better tolerance to clock deviations.
	 * 8 times oversampling is only used to achieve higher speeds.
	 */
	if (usartdiv < 16) {
		oversampling = 8;
		stm32_set_bits(port, ofs->cr1, USART_CR1_OVER8);
	} else {
		oversampling = 16;
		stm32_clr_bits(port, ofs->cr1, USART_CR1_OVER8);
	}

	mantissa = (usartdiv / oversampling) << USART_BRR_DIV_M_SHIFT;
	fraction = usartdiv % oversampling;
	writel_relaxed(mantissa | fraction, port->membase + ofs->brr);

	uart_update_timeout(port, cflag, baud);

	port->read_status_mask = USART_SR_ORE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= USART_SR_PE | USART_SR_FE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= USART_SR_LBD;

	/* Characters to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask = USART_SR_PE | USART_SR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= USART_SR_LBD;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= USART_SR_ORE;
	}

	/* Ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= USART_SR_DUMMY_RX;

	if (stm32_port->rx_ch)
		cr3 |= USART_CR3_DMAR;

	writel_relaxed(cr3, port->membase + ofs->cr3);
	writel_relaxed(cr2, port->membase + ofs->cr2);
	writel_relaxed(cr1, port->membase + ofs->cr1);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *stm32_type(struct uart_port *port)
{
	return (port->type == PORT_STM32) ? DRIVER_NAME : NULL;
}

static void stm32_release_port(struct uart_port *port)
{
}

static int stm32_request_port(struct uart_port *port)
{
	return 0;
}

static void stm32_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_STM32;
}

static int
stm32_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* No user changeable parameters */
	return -EINVAL;
}

static void stm32_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
	struct stm32_port *stm32port = container_of(port,
			struct stm32_port, port);
	struct stm32_usart_offsets *ofs = &stm32port->info->ofs;
	struct stm32_usart_config *cfg = &stm32port->info->cfg;
	unsigned long flags = 0;

	switch (state) {
	case UART_PM_STATE_ON:
		clk_prepare_enable(stm32port->clk);
		break;
	case UART_PM_STATE_OFF:
		spin_lock_irqsave(&port->lock, flags);
		stm32_clr_bits(port, ofs->cr1, BIT(cfg->uart_enable_bit));
		spin_unlock_irqrestore(&port->lock, flags);
		clk_disable_unprepare(stm32port->clk);
		break;
	}
}

static const struct uart_ops stm32_uart_ops = {
	.tx_empty	= stm32_tx_empty,
	.set_mctrl	= stm32_set_mctrl,
	.get_mctrl	= stm32_get_mctrl,
	.stop_tx	= stm32_stop_tx,
	.start_tx	= stm32_start_tx,
	.throttle	= stm32_throttle,
	.unthrottle	= stm32_unthrottle,
	.stop_rx	= stm32_stop_rx,
	.break_ctl	= stm32_break_ctl,
	.startup	= stm32_startup,
	.shutdown	= stm32_shutdown,
	.set_termios	= stm32_set_termios,
	.pm		= stm32_pm,
	.type		= stm32_type,
	.release_port	= stm32_release_port,
	.request_port	= stm32_request_port,
	.config_port	= stm32_config_port,
	.verify_port	= stm32_verify_port,
};

static int stm32_init_port(struct stm32_port *stm32port,
			  struct platform_device *pdev)
{
	struct uart_port *port = &stm32port->port;
	struct resource *res;
	int ret;

	port->iotype	= UPIO_MEM;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->ops	= &stm32_uart_ops;
	port->dev	= &pdev->dev;
	port->irq	= platform_get_irq(pdev, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	port->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);
	port->mapbase = res->start;

	spin_lock_init(&port->lock);

	stm32port->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(stm32port->clk))
		return PTR_ERR(stm32port->clk);

	/* Ensure that clk rate is correct by enabling the clk */
	ret = clk_prepare_enable(stm32port->clk);
	if (ret)
		return ret;

	stm32port->port.uartclk = clk_get_rate(stm32port->clk);
	if (!stm32port->port.uartclk)
		ret = -EINVAL;

	return ret;
}

static struct stm32_port *stm32_of_get_stm32_port(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct stm32_port *stm32port;
	struct uart_port *port;
	u32 rs485_delay[2];
	int v, id;

	if (!np)
		return NULL;

	id = of_alias_get_id(np, "serial");
	if (id < 0)
		id = 0;

	if (WARN_ON(id >= STM32_MAX_PORTS))
		return NULL;
	stm32port = &stm32_ports[id];

	stm32port->port.line = id;

	stm32port->hw_flow_control = of_property_read_bool(np,
							"st,hw-flow-ctrl");

	stm32port->link = devm_gpiod_get_optional(&pdev->dev,
						"st,link", GPIOD_IN);
	if (stm32port->link)
		gpiod_direction_output(stm32port->link, 0);

	/*
	 * Allow to access device driver with RS-485 API only if the `de` pin is
	 * specified.
	 */
	stm32port->de = devm_gpiod_get_optional(&pdev->dev,
						"st,rs485-de", GPIOD_IN);
	if (!stm32port->de)
		goto out;

	/*
	 * Allow to access device with RS-485 API only if `rs485-de` is
	 */
	port = &stm32port->port;
	memset(&port->rs485, 0, sizeof(port->rs485));
	port->rs485_config = stm32_config_rs485;

	if (of_property_read_u32_array(np, "rs485-rts-delay",
				rs485_delay, 2) == 0) {
		port->rs485.delay_rts_before_send = rs485_delay[0];
		port->rs485.delay_rts_after_send = rs485_delay[1];
	}

	port->rs485.flags = SER_RS485_RTS_ON_SEND;
	if (of_property_read_bool(np, "rs485-rx-during-tx"))
		port->rs485.flags |= SER_RS485_RX_DURING_TX;

	if (of_property_read_bool(np, "linux,rs485-enabled-at-boot-time"))
		port->rs485.flags |= SER_RS485_ENABLED;

	if (!(port->rs485.flags & SER_RS485_ENABLED))
		goto out;

	v = port->rs485.flags & SER_RS485_RTS_AFTER_SEND ? 1 : 0;
	gpiod_direction_output(stm32port->de, v);
out:
	return stm32port;
}

#ifdef CONFIG_OF
static const struct of_device_id stm32_match[] = {
	{ .compatible = "st,stm32-usart", .data = &stm32f4_info},
	{ .compatible = "st,stm32-uart", .data = &stm32f4_info},
	{ .compatible = "st,stm32f7-usart", .data = &stm32f7_info},
	{ .compatible = "st,stm32f7-uart", .data = &stm32f7_info},
	{},
};

MODULE_DEVICE_TABLE(of, stm32_match);
#endif

static int stm32_of_dma_rx_probe(struct stm32_port *stm32port,
				 struct platform_device *pdev)
{
	struct stm32_usart_offsets *ofs = &stm32port->info->ofs;
	struct device_node *np = pdev->dev.of_node;
	struct uart_port *port = &stm32port->port;
	struct device *dev = &pdev->dev;
	struct dma_slave_config config;
	struct dma_async_tx_descriptor *desc = NULL;
	dma_cookie_t cookie;
	int ret;

	stm32port->rx_rd_cur = RX_BUF_L;
	stm32port->rx_rd_prv = RX_BUF_L;
	stm32port->rx_wr_prv = RX_BUF_L;

	stm32port->rx_rd_abs = 0;
	stm32port->rx_wr_abs = 0;
	stm32port->rx_wr_ovf = 0;

	/* Check if requested to use DMA */
	if (!of_get_property(np, "st,use-dma-rx", NULL)) {
		stm32port->rx_ch = NULL;
		return -EINVAL;
	}

	/* Request DMA RX channel */
	stm32port->rx_ch = dma_request_slave_channel(dev, "rx");
	if (!stm32port->rx_ch) {
		dev_info(dev, "rx dma alloc failed\n");
		return -ENODEV;
	}
	stm32port->rx_buf = dma_alloc_coherent(&pdev->dev, RX_BUF_L,
						 &stm32port->rx_dma_buf,
						 GFP_KERNEL);
	if (!stm32port->rx_buf) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	/* Configure DMA channel */
	memset(&config, 0, sizeof(config));
	config.src_addr = port->mapbase + ofs->rdr;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;

	ret = dmaengine_slave_config(stm32port->rx_ch, &config);
	if (ret < 0) {
		dev_err(dev, "rx dma channel config failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	/* Prepare a DMA cyclic transaction */
	desc = dmaengine_prep_dma_cyclic(stm32port->rx_ch,
					 stm32port->rx_dma_buf,
					 RX_BUF_L, RX_BUF_P, DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dev, "rx dma prep cyclic failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	INIT_WORK(&stm32port->rx_work, stm32_rx_dma_work);
	INIT_WORK(&stm32port->tx_work, stm32_tx_work);
	desc->callback = stm32_rx_dma_complete;
	desc->callback_param = port;

	/* Push current DMA transaction in the pending queue */
	cookie = dmaengine_submit(desc);

	/* Issue pending DMA requests */
	dma_async_issue_pending(stm32port->rx_ch);

	return 0;

config_err:
	dma_free_coherent(&pdev->dev,
			  RX_BUF_L, stm32port->rx_buf,
			  stm32port->rx_dma_buf);

alloc_err:
	dma_release_channel(stm32port->rx_ch);
	stm32port->rx_ch = NULL;

	return ret;
}

static int stm32_of_dma_tx_probe(struct stm32_port *stm32port,
				 struct platform_device *pdev)
{
	struct stm32_usart_offsets *ofs = &stm32port->info->ofs;
	struct device_node *np = pdev->dev.of_node;
	struct uart_port *port = &stm32port->port;
	struct device *dev = &pdev->dev;
	struct dma_slave_config config;
	int ret;

	stm32port->tx_dma_busy = false;

	/* Check if requested to use DMA */
	if (!of_get_property(np, "st,use-dma-tx", NULL)) {
		stm32port->tx_ch = NULL;
		return -EINVAL;
	}

	/* Request DMA TX channel */
	stm32port->tx_ch = dma_request_slave_channel(dev, "tx");
	if (!stm32port->tx_ch) {
		dev_info(dev, "tx dma alloc failed\n");
		return -ENODEV;
	}
	stm32port->tx_buf = dma_alloc_coherent(&pdev->dev, TX_BUF_L,
						 &stm32port->tx_dma_buf,
						 GFP_KERNEL);
	if (!stm32port->tx_buf) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	/* Configure DMA channel */
	memset(&config, 0, sizeof(config));
	config.dst_addr = port->mapbase + ofs->tdr;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;

	ret = dmaengine_slave_config(stm32port->tx_ch, &config);
	if (ret < 0) {
		dev_err(dev, "tx dma channel config failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	/* DMA buffer acts as a FIFO for UART, so inform serial core about it */
	port->fifosize = TX_BUF_L;

	return 0;

config_err:
	dma_free_coherent(&pdev->dev,
			  TX_BUF_L, stm32port->tx_buf,
			  stm32port->tx_dma_buf);

alloc_err:
	dma_release_channel(stm32port->tx_ch);
	stm32port->tx_ch = NULL;

	return ret;
}

static int stm32_serial_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct stm32_port *stm32port;
	struct uart_port *port;
	int ret;

	stm32port = stm32_of_get_stm32_port(pdev);
	if (!stm32port)
		return -ENODEV;

	match = of_match_device(stm32_match, &pdev->dev);
	if (match && match->data)
		stm32port->info = (struct stm32_usart_info *)match->data;
	else
		return -EINVAL;

	ret = stm32_init_port(stm32port, pdev);
	if (ret)
		return ret;

	port = &stm32port->port;
	ret = uart_add_one_port(&stm32_usart_driver, port);
	if (ret)
		return ret;

	ret = request_irq(port->irq, stm32_interrupt, 0,
			  pdev->name, port);
	if (ret)
		return ret;

	ret = stm32_of_dma_rx_probe(stm32port, pdev);
	if (ret)
		dev_info(&pdev->dev, "interrupt mode used for rx (no dma)\n");
	stm32port->rx_ie = stm32port->rx_ch ? USART_CR1_IDLEIE :
					      USART_CR1_RXNEIE;

	stm32port->tx_busy = false;
	stm32port->tx_started = false;
	ret = stm32_of_dma_tx_probe(stm32port, pdev);
	if (ret)
		dev_info(&pdev->dev, "interrupt mode used for tx (no dma)\n");

	platform_set_drvdata(pdev, port);

	return 0;
}

static int stm32_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	free_irq(port->irq, port);

	stm32_clr_bits(port, ofs->cr3, USART_CR3_DMAR);

	if (stm32_port->rx_ch)
		dma_release_channel(stm32_port->rx_ch);

	if (stm32_port->rx_dma_buf)
		dma_free_coherent(&pdev->dev,
				  RX_BUF_L, stm32_port->rx_buf,
				  stm32_port->rx_dma_buf);

	stm32_clr_bits(port, ofs->cr3, USART_CR3_DMAT);

	if (stm32_port->tx_ch)
		dma_release_channel(stm32_port->tx_ch);

	if (stm32_port->tx_dma_buf)
		dma_free_coherent(&pdev->dev,
				  TX_BUF_L, stm32_port->tx_buf,
				  stm32_port->tx_dma_buf);

	clk_disable_unprepare(stm32_port->clk);

	return uart_remove_one_port(&stm32_usart_driver, port);
}


#ifdef CONFIG_SERIAL_STM32_CONSOLE
static void stm32_console_putchar(struct uart_port *port, int ch)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;

	while (!(readl_relaxed(port->membase + ofs->isr) & USART_SR_TXE))
		cpu_relax();

	writel_relaxed(ch, port->membase + ofs->tdr);
}

static void stm32_console_write(struct console *co, const char *s, unsigned cnt)
{
	struct uart_port *port = &stm32_ports[co->index].port;
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_usart_offsets *ofs = &stm32_port->info->ofs;
	struct stm32_usart_config *cfg = &stm32_port->info->cfg;
	unsigned long flags;
	u32 old_cr1, new_cr1;
	int locked = 1;

	local_irq_save(flags);
	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&port->lock);
	else
		spin_lock(&port->lock);

	/* Save and disable interrupts, enable the transmitter */
	old_cr1 = readl_relaxed(port->membase + ofs->cr1);
	new_cr1 = old_cr1 & ~USART_CR1_IE_MASK;
	new_cr1 |=  USART_CR1_TE | BIT(cfg->uart_enable_bit);
	writel_relaxed(new_cr1, port->membase + ofs->cr1);

	uart_console_write(port, s, cnt, stm32_console_putchar);

	/* Restore interrupt state */
	writel_relaxed(old_cr1, port->membase + ofs->cr1);

	if (locked)
		spin_unlock(&port->lock);
	local_irq_restore(flags);
}

static int stm32_console_setup(struct console *co, char *options)
{
	struct stm32_port *stm32port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= STM32_MAX_PORTS)
		return -ENODEV;

	stm32port = &stm32_ports[co->index];

	/*
	 * This driver does not support early console initialization
	 * (use ARM early printk support instead), so we only expect
	 * this to be called during the uart port registration when the
	 * driver gets probed and the port should be mapped at that point.
	 */
	if (stm32port->port.mapbase == 0 || stm32port->port.membase == NULL)
		return -ENXIO;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&stm32port->port, co, baud, parity, bits, flow);
}

static struct console stm32_console = {
	.name		= STM32_SERIAL_NAME,
	.device		= uart_console_device,
	.write		= stm32_console_write,
	.setup		= stm32_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &stm32_usart_driver,
};

#define STM32_SERIAL_CONSOLE (&stm32_console)

#else
#define STM32_SERIAL_CONSOLE NULL
#endif /* CONFIG_SERIAL_STM32_CONSOLE */

static struct uart_driver stm32_usart_driver = {
	.driver_name	= DRIVER_NAME,
	.dev_name	= STM32_SERIAL_NAME,
	.major		= 0,
	.minor		= 0,
	.nr		= STM32_MAX_PORTS,
	.cons		= STM32_SERIAL_CONSOLE,
};

static struct platform_driver stm32_serial_driver = {
	.probe		= stm32_serial_probe,
	.remove		= stm32_serial_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(stm32_match),
	},
};

static int __init usart_init(void)
{
	static char banner[] __initdata = "STM32 USART driver initialized";
	int ret;

	pr_info("%s\n", banner);

	ret = uart_register_driver(&stm32_usart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&stm32_serial_driver);
	if (ret)
		uart_unregister_driver(&stm32_usart_driver);

	return ret;
}

static void __exit usart_exit(void)
{
	platform_driver_unregister(&stm32_serial_driver);
	uart_unregister_driver(&stm32_usart_driver);
}

module_init(usart_init);
module_exit(usart_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("STMicroelectronics STM32 serial port driver");
MODULE_LICENSE("GPL v2");
