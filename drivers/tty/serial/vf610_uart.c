/*
 * Driver for Freescale MVF serial ports
 *
 * Based on drivers/char/imx.c.
 *
 *  Copyright 2012 Freescale Semiconductor, Inc.
 *  Copyright 2015 Emcraft Systems
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
 *
 */
#if defined(CONFIG_SERIAL_VF610_UART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <linux/io.h>
#include <asm/irq.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_gpio.h>

#include "serial_mctrl_gpio.h"

#define DRIVER_NAME	"vf610-uart"
#define DEV_NAME	"ttymxc"
#define UART_NR		6

#define TXTL 2 /* reset default */
#define RXTL 1 /* reset default */

/* half the RX buffer size */
#define CTSTL 16

/* All uart module registers for MVF is 8-bit width */
#define MXC_UARTBDH		0x00 /* Baud rate reg: high */
#define MXC_UARTBDL		0x01 /* Baud rate reg: low */
#define MXC_UARTCR1		0x02 /* Control reg 1 */
#define MXC_UARTCR2		0x03 /* Control reg 2 */
#define MXC_UARTSR1		0x04 /* Status reg 1 */
#define MXC_UARTSR2		0x05 /* Status reg 2 */
#define MXC_UARTCR3		0x06 /* Control reg 3 */
#define MXC_UARTDR		0x07 /* Data reg */
#define MXC_UARTMAR1		0x08 /* Match address reg 1 */
#define MXC_UARTMAR2		0x09 /* Match address reg 2 */
#define MXC_UARTCR4		0x0A /* Control reg 4 */
#define MXC_UARTCR5		0x0B /* Control reg 5 */
#define MXC_UARTEDR		0x0C /* Extended data reg */
#define MXC_UARTMODEM		0x0D /* Modem reg */
#define MXC_UARTIR		0x0E /* Infrared reg */
#define MXC_UARTPFIFO		0x10 /* FIFO parameter reg */
#define MXC_UARTCFIFO		0x11 /* FIFO control reg */
#define MXC_UARTSFIFO		0x12 /* FIFO status reg */
#define MXC_UARTTWFIFO		0x13 /* FIFO transmit watermark reg */
#define MXC_UARTTCFIFO		0x14 /* FIFO transmit count reg */
#define MXC_UARTRWFIFO		0x15 /* FIFO receive watermark reg */
#define MXC_UARTRCFIFO		0x16 /* FIFO receive count reg */
#define MXC_UARTC7816		0x18 /* 7816 control reg */
#define MXC_UARTIE7816		0x19 /* 7816 interrupt enable reg */
#define MXC_UARTIS7816		0x1A /* 7816 interrupt status reg */
#define MXC_UARTWP7816T0	0x1B /* 7816 wait parameter reg */
#define MXC_UARTWP7816T1	0x1B /* 7816 wait parameter reg */
#define MXC_UARTWN7816		0x1C /* 7816 wait N reg */
#define MXC_UARTWF7816		0x1D /* 7816 wait FD reg */
#define MXC_UARTET7816		0x1E /* 7816 error threshold reg */
#define MXC_UARTTL7816		0x1F /* 7816 transmit length reg */
#define MXC_UARTCR6		0x21 /* CEA709.1-B contrl reg */
#define MXC_UARTPCTH		0x22 /* CEA709.1-B packet cycle counter high */
#define MXC_UARTPCTL		0x23 /* CEA709.1-B packet cycle counter low */
#define MXC_UARTB1T		0x24 /* CEA709.1-B beta 1 time */
#define MXC_UARTSDTH		0x25 /* CEA709.1-B secondary delay timer high */
#define MXC_UARTSDTL		0x26 /* CEA709.1-B secondary delay timer low */
#define MXC_UARTPRE		0x27 /* CEA709.1-B preamble */
#define MXC_UARTTPL		0x28 /* CEA709.1-B transmit packet length */
#define MXC_UARTIE		0x29 /* CEA709.1-B transmit interrupt enable */
#define MXC_UARTSR3		0x2B /* CEA709.1-B status reg */
#define MXC_UARTSR4		0x2C /* CEA709.1-B status reg */
#define MXC_UARTRPL		0x2D /* CEA709.1-B received packet length */
#define MXC_UARTRPREL		0x2E /* CEA709.1-B received preamble length */
#define MXC_UARTCPW		0x2F /* CEA709.1-B collision pulse width */
#define MXC_UARTRIDT		0x30 /* CEA709.1-B receive indeterminate time */
#define MXC_UARTTIDT		0x31 /* CEA709.1-B transmit indeterminate time*/

/* Bit definations of BDH */
#define MXC_UARTBDH_LBKDIE	0x80 /* LIN break detect interrupt enable */
#define MXC_UARTBDH_RXEDGIE	0x40 /* RxD input Active edge interrupt enable*/
#define MXC_UARTBDH_SBR_MASK	0x1f /* Uart baud rate high 5-bits */
/* Bit definations of CR1 */
#define MXC_UARTCR1_LOOPS	0x80 /* Loop mode select */
#define MXC_UARTCR1_RSRC	0x20 /* Receiver source select */
#define MXC_UARTCR1_M		0x10 /* 9-bit 8-bit mode select */
#define MXC_UARTCR1_WAKE	0x08 /* Receiver wakeup method */
#define MXC_UARTCR1_ILT		0x04 /* Idle line type */
#define MXC_UARTCR1_PE		0x02 /* Parity enable */
#define MXC_UARTCR1_PT		0x01 /* Parity type */
/* Bit definations of CR2 */
#define MXC_UARTCR2_TIE		0x80 /* Tx interrupt or DMA request enable */
#define MXC_UARTCR2_TCIE	0x40 /* Transmission complete int enable */
#define MXC_UARTCR2_RIE		0x20 /* Rx full int or DMA request enable */
#define MXC_UARTCR2_ILIE	0x10 /* Idle line interrupt enable */
#define MXC_UARTCR2_TE		0x08 /* Transmitter enable */
#define MXC_UARTCR2_RE		0x04 /* Receiver enable */
#define MXC_UARTCR2_RWU		0x02 /* Receiver wakeup control */
#define MXC_UARTCR2_SBK		0x01 /* Send break */
/* Bit definations of SR1 */
#define MXC_UARTSR1_TDRE	0x80 /* Tx data reg empty */
#define MXC_UARTSR1_TC		0x40 /* Transmit complete */
#define MXC_UARTSR1_RDRF	0x20 /* Rx data reg full */
#define MXC_UARTSR1_IDLE	0x10 /* Idle line flag */
#define MXC_UARTSR1_OR		0x08 /* Receiver overrun */
#define MXC_UARTSR1_NF		0x04 /* Noise flag */
#define MXC_UARTSR1_FE		0x02 /* Frame error */
#define MXC_UARTSR1_PE		0x01 /* Parity error */
/* Bit definations of SR2 */
#define MXC_UARTSR2_LBKDIF	0x80 /* LIN brk detect interrupt flag */
#define MXC_UARTSR2_RXEDGIF	0x40 /* RxD pin active edge interrupt flag */
#define MXC_UARTSR2_MSBF	0x20 /* MSB first */
#define MXC_UARTSR2_RXINV	0x10 /* Receive data inverted */
#define MXC_UARTSR2_RWUID	0x08 /* Receive wakeup idle detect */
#define MXC_UARTSR2_BRK13	0x04 /* Break transmit character length */
#define MXC_UARTSR2_LBKDE	0x02 /* LIN break detection enable */
#define MXC_UARTSR2_RAF		0x01 /* Receiver active flag */
/* Bit definations of CR3 */
#define MXC_UARTCR3_R8		0x80 /* Received bit8, for 9-bit data format */
#define MXC_UARTCR3_T8		0x40 /* transmit bit8, for 9-bit data format */
#define MXC_UARTCR3_TXDIR	0x20 /* Tx pin direction in single-wire mode */
#define MXC_UARTCR3_TXINV	0x10 /* Transmit data inversion */
#define MXC_UARTCR3_ORIE	0x08 /* Overrun error interrupt enable */
#define MXC_UARTCR3_NEIE	0x04 /* Noise error interrupt enable */
#define MXC_UARTCR3_FEIE	0x02 /* Framing error interrupt enable */
#define MXC_UARTCR3_PEIE	0x01 /* Parity errror interrupt enable */
/* Bit definations of CR4 */
#define MXC_UARTCR4_MAEN1	0x80 /* Match address mode enable 1 */
#define MXC_UARTCR4_MAEN2	0x40 /* Match address mode enable 2 */
#define MXC_UARTCR4_M10		0x20 /* 10-bit mode select */
#define MXC_UARTCR4_BRFA_MASK	0x1F /* Baud rate fine adjust */
#define MXC_UARTCR4_BRFA_OFF	0
/* Bit definations of CR5 */
#define MXC_UARTCR5_TDMAS	0x80 /* Transmitter DMA select */
#define MXC_UARTCR5_RDMAS	0x20 /* Receiver DMA select */
/* Bit definations of Modem */
#define MXC_UARTMODEM_RXRTSE	0x08 /* Enable receiver request-to-send */
#define MXC_UARTMODEM_TXRTSPOL	0x04 /* Select transmitter RTS polarity */
#define MXC_UARTMODEM_TXRTSE	0x02 /* Enable transmitter request-to-send */
#define MXC_UARTMODEM_TXCTSE	0x01 /* Enable transmitter CTS clear-to-send */
/* Bit definations of EDR */
#define MXC_UARTEDR_NOISY	0x80 /* Current dataword received with noise */
#define MXC_UARTEDR_PARITYE	0x40 /* Dataword received with parity error */
/* Bit definations of Infrared reg(IR) */
#define MXC_UARTIR_IREN		0x04 /* Infrared enable */
#define MXC_UARTIR_TNP_MASK	0x03 /* Transmitter narrow pluse */
#define MXC_UARTIR_TNP_OFF	0
/* Bit definations of FIFO parameter reg */
#define MXC_UARTPFIFO_TXFE	0x80 /* Transmit fifo enable */
#define MXC_UARTPFIFO_TXFIFOSIZE_MASK	0x7
#define MXC_UARTPFIFO_TXFIFOSIZE_OFF	4
#define MXC_UARTPFIFO_RXFE	0x08 /* Receiver fifo enable */
#define MXC_UARTPFIFO_RXFIFOSIZE_MASK	0x7
#define MXC_UARTPFIFO_RXFIFOSIZE_OFF	0
/* Bit definations of FIFO control reg */
#define MXC_UARTCFIFO_TXFLUSH	0x80 /* Transmit FIFO/buffer flush */
#define MXC_UARTCFIFO_RXFLUSH	0x40 /* Receive FIFO/buffer flush */
#define MXC_UARTCFIFO_RXOFE	0x04 /* Receive fifo overflow INT enable */
#define MXC_UARTCFIFO_TXOFE	0x02 /* Transmit fifo overflow INT enable */
#define MXC_UARTCFIFO_RXUFE	0x01 /* Receive fifo underflow INT enable */
/* Bit definations of FIFO status reg */
#define MXC_UARTSFIFO_TXEMPT	0x80 /* Transmit fifo/buffer empty */
#define MXC_UARTSFIFO_RXEMPT	0x40 /* Receive fifo/buffer empty */
#define MXC_UARTSFIFO_RXOF	0x04 /* Rx buffer overflow flag */
#define MXC_UARTSFIFO_TXOF	0x02 /* Tx buffer overflow flag */
#define MXC_UARTSFIFO_RXUF	0x01 /* Rx buffer underflow flag */

/* Bit definations of UCR1 */
#define MXC_UARTUCR1_ADEN       0x8000
#define MXC_UARTUCR1_ADBR       0x4000
#define MXC_UARTUCR1_TRDYEN     0x2000
#define MXC_UARTUCR1_IDEN       0x1000
#define MXC_UARTUCR1_RRDYEN     0x0200
#define MXC_UARTUCR1_RXDMAEN    0x0100
#define MXC_UARTUCR1_IREN       0x0080
#define MXC_UARTUCR1_TXMPTYEN   0x0040
#define MXC_UARTUCR1_RTSDEN     0x0020
#define MXC_UARTUCR1_SNDBRK     0x0010
#define MXC_UARTUCR1_TXDMAEN    0x0008
#define MXC_UARTUCR1_ATDMAEN    0x0004
#define MXC_UARTUCR1_DOZE       0x0002
#define MXC_UARTUCR1_UARTEN     0x0001

/* Bit definations of UCR2 */
#define MXC_UARTUCR2_ESCI       0x8000
#define MXC_UARTUCR2_IRTS       0x4000
#define MXC_UARTUCR2_CTSC       0x2000
#define MXC_UARTUCR2_CTS        0x1000
#define MXC_UARTUCR2_PREN       0x0100
#define MXC_UARTUCR2_PROE       0x0080
#define MXC_UARTUCR2_STPB       0x0040
#define MXC_UARTUCR2_WS         0x0020
#define MXC_UARTUCR2_RTSEN      0x0010
#define MXC_UARTUCR2_ATEN       0x0008
#define MXC_UARTUCR2_TXEN       0x0004
#define MXC_UARTUCR2_RXEN       0x0002
#define MXC_UARTUCR2_SRST       0x0001

/* Bit definations of UCR3 */
#define MXC_UARTUCR3_DTREN      0x2000
#define MXC_UARTUCR3_PARERREN   0x1000
#define MXC_UARTUCR3_FRAERREN   0x0800
#define MXC_UARTUCR3_DSR        0x0400
#define MXC_UARTUCR3_DCD        0x0200
#define MXC_UARTUCR3_RI         0x0100
#define MXC_UARTUCR3_RXDSEN     0x0040
#define MXC_UARTUCR3_AWAKEN     0x0010
#define MXC_UARTUCR3_DTRDEN     0x0008
#define MXC_UARTUCR3_RXDMUXSEL  0x0004
#define MXC_UARTUCR3_INVT       0x0002

/* Bit definations of UCR4 */
#define MXC_UARTUCR4_CTSTL_OFFSET       10
#define MXC_UARTUCR4_CTSTL_MASK         (0x3F << 10)
#define MXC_UARTUCR4_INVR               0x0200
#define MXC_UARTUCR4_ENIRI              0x0100
#define MXC_UARTUCR4_REF16              0x0040
#define MXC_UARTUCR4_IRSC               0x0020
#define MXC_UARTUCR4_TCEN               0x0008
#define MXC_UARTUCR4_OREN               0x0002
#define MXC_UARTUCR4_DREN               0x0001

/* Bit definations of UFCR */
#define MXC_UARTUFCR_RFDIV              0x0200	/* Ref freq div is set to 2 */
#define MXC_UARTUFCR_RFDIV_OFFSET       7
#define MXC_UARTUFCR_RFDIV_MASK         (0x7 << 7)
#define MXC_UARTUFCR_TXTL_OFFSET        10
#define MXC_UARTUFCR_DCEDTE             0x0040

/* Bit definations of URXD */
#define MXC_UARTURXD_ERR        0x4000
#define MXC_UARTURXD_OVRRUN     0x2000
#define MXC_UARTURXD_FRMERR     0x1000
#define MXC_UARTURXD_BRK        0x0800
#define MXC_UARTURXD_PRERR      0x0400

/* Bit definations of USR1 */
#define MXC_UARTUSR1_PARITYERR  0x8000
#define MXC_UARTUSR1_RTSS       0x4000
#define MXC_UARTUSR1_TRDY       0x2000
#define MXC_UARTUSR1_RTSD       0x1000
#define MXC_UARTUSR1_FRAMERR    0x0400
#define MXC_UARTUSR1_RRDY       0x0200
#define MXC_UARTUSR1_AGTIM      0x0100
#define MXC_UARTUSR1_DTRD       0x0080
#define MXC_UARTUSR1_AWAKE      0x0010

/* Bit definations of USR2 */
#define MXC_UARTUSR2_TXFE       0x4000
#define MXC_UARTUSR2_IDLE       0x1000
#define MXC_UARTUSR2_RIDELT     0x0400
#define MXC_UARTUSR2_RIIN       0x0200
#define MXC_UARTUSR2_DCDDELT    0x0040
#define MXC_UARTUSR2_DCDIN      0x0020
#define MXC_UARTUSR2_TXDC       0x0008
#define MXC_UARTUSR2_ORE        0x0002
#define MXC_UARTUSR2_RDR        0x0001
#define MXC_UARTUSR2_BRCD       0x0004

/* Bit definations of UTS */
#define MXC_UARTUTS_LOOP        0x1000

#define MXC_SLAVE_BUF_SIZE	(1024)
#define MXC_DMA_RX_TIMEOUT_MSEC	20
#define MXC_DMA_RX_TIMEOUT	msecs_to_jiffies(MXC_DMA_RX_TIMEOUT_MSEC)
#define MXC_DMA_RX_PERIOD	64

#define RS384_HALF_DUPLEX_SEND	1
#define RS485_HALF_DUPLEX_RECV	0

struct imx_port {
	struct uart_port	port;
	unsigned int		old_status;
	bool			have_rts;
	bool			have_cts;
	bool			rts_inverted;
	unsigned int		fifo_en:1; /* enable FIFO mode */
	struct clk		*clk;
	unsigned int		tx_fifo_size, rx_fifo_size;

	int			enable_dma;
	struct dma_chan		*dma_chan_tx;
	struct dma_chan		*dma_chan_rx;
	struct dma_async_tx_descriptor  *dma_tx_desc;
	dma_addr_t		dma_tx_buf_bus;
	struct scatterlist	tx_sgl;
	unsigned int		tx_bytes;
	struct work_struct	tsk_dma_tx;
	unsigned int		dma_tx_nents;
	bool			dma_is_txing;
	bool			dma_is_rxing;

	dma_addr_t		membase_phys;
	u8			*dma_def_rx_buf;
	void			*dma_rx_buf;
	size_t			dma_rx_len;
	int			dma_rx_period;
	dma_addr_t		dma_rx_buf_phys;
	dma_cookie_t		dma_rx_cookie;
	int			dma_rx_prev_residue;
	struct timer_list	dma_rx_timer;
	struct gpio_desc	*rs485_ctrl_gpio;
	struct mctrl_gpios	*gpios;
	volatile int		tx_done;
};

static const struct of_device_id imx_uart_dt_ids[] = {
	{
		.compatible = "fsl,vf610-uart",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_uart_dt_ids);

/*
 * interrupts disabled on entry
 */
static void imx_stop_tx(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned char temp;

	temp = readb(sport->port.membase + MXC_UARTCR2);
	writeb(temp & ~(MXC_UARTCR2_TIE | MXC_UARTCR2_TCIE),
	       sport->port.membase + MXC_UARTCR2);

	if (sport->rs485_ctrl_gpio) {
		unsigned char c2;

		c2 = readb(sport->port.membase + MXC_UARTCR2);
		c2 &= ~(MXC_UARTCR2_RE | MXC_UARTCR2_TE | MXC_UARTCR2_TIE |
			MXC_UARTCR2_TCIE | MXC_UARTCR2_RIE);
		writeb(c2, sport->port.membase + MXC_UARTCR2);

		gpiod_set_value(sport->rs485_ctrl_gpio, RS485_HALF_DUPLEX_RECV);

		temp = readb(sport->port.membase + MXC_UARTCFIFO);
		writeb(temp | MXC_UARTCFIFO_RXFLUSH, sport->port.membase + MXC_UARTCFIFO);
		c2 |= MXC_UARTCR2_RIE | MXC_UARTCR2_RE;
		writeb(c2, sport->port.membase + MXC_UARTCR2);

		sport->tx_done = 1;
	}
}

/*
 * interrupts disabled on entry
 */
static void imx_stop_rx(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned char temp;

	temp = readb(sport->port.membase + MXC_UARTCR2);
	writeb(temp & ~MXC_UARTCR2_RE, sport->port.membase + MXC_UARTCR2);
}

static inline void imx_transmit_buffer(struct imx_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;

	while (!uart_circ_empty(xmit) &&
	       (sport->fifo_en == 1 ?
		(readb(sport->port.membase + MXC_UARTTCFIFO) < sport->tx_fifo_size) :
		(readb(sport->port.membase + MXC_UARTSR1) & MXC_UARTSR1_TDRE))) {
		/* send xmit->buf[xmit->tail]
		 * out the port here */
		writeb(xmit->buf[xmit->tail], sport->port.membase + MXC_UARTDR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		imx_stop_tx(&sport->port);
}

static void dma_rx_report(struct imx_port *sport, u8 *buf, int cnt)
{
	struct tty_port *port = &sport->port.state->port;
	tty_insert_flip_string(port, buf, cnt);
	sport->port.icount.rx += cnt;
}

static void dma_rx_fetch(struct imx_port *sport)
{
	struct tty_port *port = &sport->port.state->port;
	unsigned char *buf = (unsigned char*)sport->dma_rx_buf;
	struct dma_tx_state state;
	int cnt, off;

	dmaengine_tx_status(sport->dma_chan_rx, sport->dma_rx_cookie, &state);

	if (sport->dma_rx_prev_residue == state.residue)
		return;

	spin_lock(&sport->port.lock);

	dma_sync_single_for_cpu(sport->port.dev, sport->dma_rx_buf_phys, sport->dma_rx_len, DMA_FROM_DEVICE);

	if (sport->dma_rx_prev_residue < state.residue) {
		off = MXC_SLAVE_BUF_SIZE - sport->dma_rx_prev_residue;
		cnt = sport->dma_rx_prev_residue;
		dma_rx_report(sport, buf + off, cnt);

		sport->dma_rx_prev_residue = MXC_SLAVE_BUF_SIZE;
	}

	if (sport->dma_rx_prev_residue > state.residue) {
		off = MXC_SLAVE_BUF_SIZE - sport->dma_rx_prev_residue;
		cnt = sport->dma_rx_prev_residue - state.residue;
		dma_rx_report(sport, buf + off, cnt);

		sport->dma_rx_prev_residue = state.residue;
	}

	spin_unlock(&sport->port.lock);

	tty_flip_buffer_push(port);
}

static void imx_dma_rx_timeout(unsigned long data)
{
	struct imx_port *sport = (struct imx_port *)data;
	unsigned long temp;
	unsigned long flags;

	dma_rx_fetch(sport);

	spin_lock_irqsave(&sport->port.lock, flags);

	del_timer(&sport->dma_rx_timer);

	/* Cancel DMA on RX */
	temp = readb(sport->port.membase + MXC_UARTCR5);
	temp &= ~MXC_UARTCR5_RDMAS;
	sport->dma_is_rxing = 0;
	writeb(temp, sport->port.membase + MXC_UARTCR5);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void dma_rx_callback(void *data)
{
	struct imx_port *sport = (struct imx_port *)data;
	unsigned long flags;

	dma_rx_fetch(sport);

	spin_lock_irqsave(&sport->port.lock, flags);

	if (sport->dma_is_rxing) {
		/* Restart the timer if dma is not canceled by timeout */
		mod_timer(&sport->dma_rx_timer, jiffies + MXC_DMA_RX_TIMEOUT);
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int imx_load_dma_rx(struct imx_port *sport, u8 *buf, size_t len)
{
	struct dma_chan *chan = sport->dma_chan_rx;
	struct dma_async_tx_descriptor *rxdesc;
	dma_addr_t buf_dma = dma_map_single(chan->device->dev, buf, len, DMA_FROM_DEVICE);

	if (dma_mapping_error(chan->device->dev, buf_dma)) {
		dev_err(sport->port.dev, "DMA mapping failed\n");
		return -1;
	}

	sport->dma_rx_buf = buf;
	sport->dma_rx_len = len;
	sport->dma_rx_buf_phys = buf_dma;
	sport->dma_rx_prev_residue = MXC_SLAVE_BUF_SIZE;

	rxdesc = dmaengine_prep_dma_cyclic(chan, buf_dma,
					   len, sport->dma_rx_period,
					   DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT);

	if (!rxdesc) {
		dev_err(sport->port.dev, "Not able to get DMA descriptor\n");
		goto fail_prep;
	}

	rxdesc->callback = dma_rx_callback;
	rxdesc->callback_param = sport;

	sport->dma_rx_cookie = dmaengine_submit(rxdesc);

	if (dma_submit_error(sport->dma_rx_cookie)) {
		dev_err(sport->port.dev, "DMA submit failed\n");
		goto fail_prep;
	}

	dma_async_issue_pending(chan);

	return 0;

fail_prep:
	dma_unmap_single(chan->device->dev, buf_dma, len, DMA_FROM_DEVICE);

	return -1;
}

static void imx_stop_dma_rx(struct imx_port *sport)
{
	struct dma_chan *chan = sport->dma_chan_rx;
	unsigned long temp, flags;

	dmaengine_terminate_all(chan);
	dma_unmap_single(chan->device->dev, sport->dma_rx_buf_phys, sport->dma_rx_len, DMA_FROM_DEVICE);

	spin_lock_irqsave(&sport->port.lock, flags);

	/* Disable DMA on RX */
	del_timer(&sport->dma_rx_timer);
	sport->dma_is_rxing = 0;
	temp = readb(sport->port.membase + MXC_UARTCR5);
	temp &= ~MXC_UARTCR5_RDMAS;
	writeb(temp, sport->port.membase + MXC_UARTCR5);

	spin_unlock_irqrestore(&sport->port.lock, flags);

}

static void dma_tx_callback(void *data);
/****************************************************************************/
/* Configure and start DMA engine. */
static void tx_uart_dmarun(struct imx_port *sport)
{
	unsigned char *src = (unsigned char *)sg_virt(&sport->tx_sgl);
	unsigned char *dst = (unsigned char *)(sport->port.mapbase + MXC_UARTDR);
	int size = sport->tx_bytes;
	int ret;
	struct dma_slave_config dma_tx_sconfig;
	dma_addr_t dma_bus;

	if (sport->dma_tx_nents == 0)
		return;

	dma_bus = dma_map_single(sport->dma_chan_tx->device->dev,
				 src, UART_XMIT_SIZE, DMA_TO_DEVICE);

	if (dma_mapping_error(sport->dma_chan_tx->device->dev, dma_bus)) {
		dev_err(sport->port.dev, "dma_map_single tx failed\n");
		return;
	}

	dma_tx_sconfig.dst_addr = (unsigned int)dst;
	dma_tx_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_tx_sconfig.dst_maxburst = 1;
	dma_tx_sconfig.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(sport->dma_chan_tx, &dma_tx_sconfig);

	if (ret < 0) {
		dev_err(sport->port.dev,
			"Dma slave config failed, err = %d\n", ret);
		return;
	}

	dma_sync_single_for_device(sport->port.dev, dma_bus,
				   UART_XMIT_SIZE, DMA_TO_DEVICE);

	sport->dma_tx_desc =
		dmaengine_prep_slave_single(sport->dma_chan_tx,
					    dma_bus, size,
					    DMA_MEM_TO_DEV,
					    DMA_PREP_INTERRUPT);

	if (!sport->dma_tx_desc) {
		dev_err(sport->port.dev, "Not able to get desc for tx\n");
		return;
	}

	sport->dma_tx_desc->callback = dma_tx_callback;
	sport->dma_tx_desc->callback_param = sport;
	sport->dma_is_txing = 1;
	sport->dma_tx_buf_bus = dma_bus;

	dmaengine_submit(sport->dma_tx_desc);
	dma_async_issue_pending(sport->dma_chan_tx);
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int imx_tx_empty(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;

	return (readb(sport->port.membase + MXC_UARTSR1) & MXC_UARTSR1_TC) ?
		TIOCSER_TEMT : 0;
}

/****************************************************************************/
static void dma_tx_callback(void *data)
{
	struct imx_port *sport = data;
	struct circ_buf *xmit = &sport->port.state->xmit;

	async_tx_ack(sport->dma_tx_desc);
	dma_unmap_single(sport->port.dev, sport->dma_tx_buf_bus,
			 UART_XMIT_SIZE, DMA_TO_DEVICE);

	dma_unmap_sg(sport->port.dev, &sport->tx_sgl, 1, DMA_TO_DEVICE);

	spin_lock(&sport->port.lock);
	xmit->tail = (xmit->tail + sport->tx_bytes) & (UART_XMIT_SIZE - 1);
	sport->port.icount.tx += sport->tx_bytes;
	spin_unlock(&sport->port.lock);

	/* !!! FEEXME !!!
	   Apparently, DMA generates interrupts right after the data is sent
	   to the UART controller, however it doesn't wait while the data is
	   physically transferred to the serial line. This can lead to mysterious
	   loss of last character or even 2 or 4 KB block in the transfer.
	   Added workaround to wait for the transfer is actually complete prior
	   to clean the dma_is_txing flag.
	*/
	while (!imx_tx_empty(&sport->port)) {
		udelay(10);
	}

	sport->dma_is_txing = 0;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	schedule_work(&sport->tsk_dma_tx);
}

static void dma_tx_work(struct work_struct *w)
{
	struct imx_port *sport = container_of(w, struct imx_port, tsk_dma_tx);
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags, temp;
	int tx_left;

	if (sport->dma_is_txing)
		return;

	if (sport->rs485_ctrl_gpio && sport->tx_done) {
		unsigned char c2;

		sport->tx_done = 0;

		c2 = readb(sport->port.membase + MXC_UARTCR2);
		c2 &= ~(MXC_UARTCR2_RE | MXC_UARTCR2_TE | MXC_UARTCR2_TIE |
			MXC_UARTCR2_TCIE | MXC_UARTCR2_RIE);
		writeb(c2, sport->port.membase + MXC_UARTCR2);
		temp = readb(sport->port.membase + MXC_UARTCFIFO);
		writeb(temp | MXC_UARTCFIFO_RXFLUSH, sport->port.membase + MXC_UARTCFIFO);

		gpiod_set_value(sport->rs485_ctrl_gpio, RS384_HALF_DUPLEX_SEND);

		c2 |= MXC_UARTCR2_TIE | MXC_UARTCR2_TE;
		writeb(c2, sport->port.membase + MXC_UARTCR2);
	}

	spin_lock_irqsave(&sport->port.lock, flags);
	tx_left = sport->tx_bytes = uart_circ_chars_pending(xmit);

	if (sport->tx_bytes > 0) {
		sport->dma_tx_nents = 0;
		if (xmit->tail > xmit->head) {
			sg_init_one(&sport->tx_sgl, xmit->buf + xmit->tail,
				    UART_XMIT_SIZE - xmit->tail);
			sport->dma_tx_nents = UART_XMIT_SIZE - xmit->tail;
			sport->tx_bytes = sport->dma_tx_nents;
		} else {
			sg_init_one(&sport->tx_sgl, xmit->buf + xmit->tail,
				    sport->tx_bytes);
			sport->dma_tx_nents = sport->tx_bytes;
		}

		dma_map_sg(sport->port.dev, &sport->tx_sgl, 1, DMA_TO_DEVICE);
		/* fire it */
		tx_uart_dmarun(sport);
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);

	if (!tx_left && sport->rs485_ctrl_gpio) {
		unsigned char c2;

		c2 = readb(sport->port.membase + MXC_UARTCR2);
		c2 &= ~(MXC_UARTCR2_RE | MXC_UARTCR2_TE | MXC_UARTCR2_TIE |
			MXC_UARTCR2_TCIE | MXC_UARTCR2_RIE);
		writeb(c2, sport->port.membase + MXC_UARTCR2);

		gpiod_set_value(sport->rs485_ctrl_gpio, RS485_HALF_DUPLEX_RECV);

		temp = readb(sport->port.membase + MXC_UARTCFIFO);
		writeb(temp | MXC_UARTCFIFO_RXFLUSH, sport->port.membase + MXC_UARTCFIFO);
		c2 |= MXC_UARTCR2_RIE | MXC_UARTCR2_RE;
		writeb(c2, sport->port.membase + MXC_UARTCR2);

		sport->tx_done = 1;
	}

	return;
}

/*
 * interrupts disabled on entry
 */
static void imx_start_tx(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;

	if (sport->enable_dma) {
		schedule_work(&sport->tsk_dma_tx);
		return;
	}

	if (readb(sport->port.membase + MXC_UARTSR1) & MXC_UARTSR1_TDRE)
		imx_transmit_buffer(sport);
}


static irqreturn_t imx_txint(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	if (sport->port.x_char) {
		/* Send next char */
		writeb(sport->port.x_char, sport->port.membase + MXC_UARTDR);
		goto out;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		imx_stop_tx(&sport->port);
		goto out;
	}

	imx_transmit_buffer(sport);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t imx_rxint(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned int flg, ignored = 0;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags;
	unsigned char rx, sr;

	if (sport->dma_chan_rx) {
		unsigned long temp;

		spin_lock_irqsave(&sport->port.lock, flags);

		if (!sport->dma_is_rxing) {
			sport->dma_rx_timer.expires = jiffies + MXC_DMA_RX_TIMEOUT;
			add_timer(&sport->dma_rx_timer);

			temp = readb(sport->port.membase + MXC_UARTCR5);
			temp |= MXC_UARTCR5_RDMAS;
			writeb(temp, sport->port.membase + MXC_UARTCR5);

			sport->dma_is_rxing = 1;
		}

		spin_unlock_irqrestore(&sport->port.lock, flags);

		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&sport->port.lock, flags);

	while (!(readb(sport->port.membase + MXC_UARTSFIFO) &
		 MXC_UARTSFIFO_RXEMPT)) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		/*  To clear the FE, OR, NF, FE, PE flags when set,
		 *  read SR1 then read DR
		 */
		sr = readb(sport->port.membase + MXC_UARTSR1);
		rx = readb(sport->port.membase + MXC_UARTDR);

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;
		if (sr & (MXC_UARTSR1_PE | MXC_UARTSR1_OR | MXC_UARTSR1_FE)) {
			if (sr & MXC_UARTSR1_PE)
				sport->port.icount.parity++;
			else if (sr & MXC_UARTSR1_FE)
				sport->port.icount.frame++;
			if (sr & MXC_UARTSR1_OR)
				sport->port.icount.overrun++;

			if (sr & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}

			sr &= sport->port.read_status_mask;

			if (sr & MXC_UARTSR1_PE)
				flg = TTY_PARITY;
			else if (sr & MXC_UARTSR1_FE)
				flg = TTY_FRAME;
			if (sr & MXC_UARTSR1_OR)
				flg = TTY_OVERRUN;

#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
#endif
		}

		tty_insert_flip_char(port, rx, flg);
	}

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);

	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}

static irqreturn_t imx_overrun_int(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned long flags;
	unsigned char d;
	u8 cfifo;

	printk(KERN_ERR "MVF UART%d RX buffer overrun\n", sport->port.line);

	spin_lock_irqsave(&sport->port.lock, flags);

	/* Read S1 and then D to reset OR */
	d = readb(sport->port.membase + MXC_UARTSR1);
	d = readb(sport->port.membase + MXC_UARTDR);

	/* Reset Recieve Overflow bit and flush RX */
	writeb(MXC_UARTSFIFO_RXOF, sport->port.membase + MXC_UARTSFIFO);
	cfifo = readb(sport->port.membase + MXC_UARTCFIFO);
	writeb(cfifo | MXC_UARTCFIFO_RXFLUSH, sport->port.membase + MXC_UARTCFIFO);

	spin_unlock_irqrestore(&sport->port.lock, flags);

	sport->port.icount.overrun++;
	return IRQ_HANDLED;
}

static irqreturn_t imx_underflow_int(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	u8 cr2, cfifo;

	/* disable rx */
	cr2 = readb(sport->port.membase + MXC_UARTCR2);
	cr2 &= ~MXC_UARTCR2_RE;
	writeb(cr2, sport->port.membase + MXC_UARTCR2);

	/* clean the underflow bit */
	writeb(MXC_UARTSFIFO_RXUF, sport->port.membase + MXC_UARTSFIFO);

	/* flush rx fifo */
	cfifo = readb(sport->port.membase + MXC_UARTCFIFO);
	cfifo |= MXC_UARTCFIFO_RXFLUSH;
	writeb(cfifo , sport->port.membase + MXC_UARTCFIFO);

	/* enable rx */
	cr2 |= MXC_UARTCR2_RE;
	writeb(cr2, sport->port.membase + MXC_UARTCR2);
	return IRQ_HANDLED;
}

static irqreturn_t imx_int(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned int sts;
	unsigned char sfifo;

	sfifo = readb(sport->port.membase + MXC_UARTSFIFO);

	sts = readb(sport->port.membase + MXC_UARTSR1);

	if (sts & MXC_UARTSR1_FE) {
		readb(sport->port.membase + MXC_UARTDR);
	}

	if (sfifo & MXC_UARTSFIFO_RXUF)
		imx_underflow_int(irq, dev_id);

	if (sts & MXC_UARTSR1_OR)
		imx_overrun_int(irq, dev_id);

	if (sts & MXC_UARTSR1_RDRF)
		imx_rxint(irq, dev_id);

	if (sts & MXC_UARTSR1_TDRE &&
	    !(readb(sport->port.membase + MXC_UARTCR5) &
	      MXC_UARTCR5_TDMAS))
		imx_txint(irq, dev_id);

	return IRQ_HANDLED;
}

/*
 * We have a modem side uart, so the meanings of RTS and CTS are inverted.
 */
static unsigned int imx_get_mctrl(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned int tmp = 0;

	if (sport->gpios)
		mctrl_gpio_get(sport->gpios, &tmp);

	if (readb(sport->port.membase + MXC_UARTMODEM) & MXC_UARTMODEM_TXCTSE)
		tmp |= TIOCM_CTS;

	if (readb(sport->port.membase + MXC_UARTMODEM) & MXC_UARTMODEM_TXRTSE)
		tmp |= TIOCM_RTS;

	return tmp;
}

static void imx_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long temp;

	if (sport->gpios)
		mctrl_gpio_set(sport->gpios, mctrl);

	temp = readb(sport->port.membase + MXC_UARTMODEM) &
		~(MXC_UARTMODEM_RXRTSE | MXC_UARTMODEM_TXRTSE);

	if (mctrl & TIOCM_RTS)
		temp |= MXC_UARTMODEM_TXRTSE;

	writeb(temp, sport->port.membase + MXC_UARTMODEM);
}

/*
 * Interrupts always disabled.
 */
static void imx_break_ctl(struct uart_port *port, int break_state)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long flags;
	unsigned char temp;

	spin_lock_irqsave(&sport->port.lock, flags);

	temp = readb(sport->port.membase + MXC_UARTCR2) & ~MXC_UARTCR2_SBK;

	if (break_state != 0)
		temp |= MXC_UARTCR2_SBK;

	writeb(temp, sport->port.membase + MXC_UARTCR2);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int imx_setup_watermark(struct imx_port *sport, unsigned int mode)
{
	unsigned char val, old_cr2, cr2;

	/* set receiver / transmitter trigger level.
	 */
	old_cr2 = cr2 = readb(sport->port.membase + MXC_UARTCR2);
	cr2 &= ~(MXC_UARTCR2_TIE | MXC_UARTCR2_TCIE | MXC_UARTCR2_TE |
		 MXC_UARTCR2_RIE | MXC_UARTCR2_RE);
	writeb(cr2, sport->port.membase + MXC_UARTCR2);

	val = TXTL;
	writeb(val, sport->port.membase + MXC_UARTTWFIFO);
	val = RXTL;
	writeb(val, sport->port.membase + MXC_UARTRWFIFO);

	/* Enable Tx and Rx FIFO */
	val = readb(sport->port.membase + MXC_UARTPFIFO);
	sport->tx_fifo_size = 0x1 << (((val >> MXC_UARTPFIFO_TXFIFOSIZE_OFF) &
				       MXC_UARTPFIFO_TXFIFOSIZE_MASK) + 1);
	sport->rx_fifo_size = 0x1 << (((val >> MXC_UARTPFIFO_RXFIFOSIZE_OFF) &
				       MXC_UARTPFIFO_RXFIFOSIZE_MASK) + 1);
	writeb(val | MXC_UARTPFIFO_TXFE | MXC_UARTPFIFO_RXFE,
	       sport->port.membase + MXC_UARTPFIFO);

	/* Flush the Tx and Rx FIFO to a known state */
	writeb(MXC_UARTCFIFO_TXFLUSH | MXC_UARTCFIFO_RXFLUSH,
	       sport->port.membase + MXC_UARTCFIFO);

	/* restore CR2 */
	writeb(old_cr2, sport->port.membase + MXC_UARTCR2);

	return 0;
}

static int imx_startup(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	int retval;
	unsigned long flags, temp;

	spin_lock_irqsave(&sport->port.lock, flags);

#ifndef CONFIG_SERIAL_CORE_CONSOLE
	if (sport->fifo_en)
		imx_setup_watermark(sport, 0);
#endif

	/* disable the DREN bit (Data Ready interrupt enable) before
	 * requesting IRQs
	 */
	temp = readb(sport->port.membase + MXC_UARTCR2);

	writeb(temp & ~MXC_UARTCR2_RIE, sport->port.membase + MXC_UARTCR2);

	/* Enable over- and under-flow interrupts on RX. */
	temp = readb(sport->port.membase + MXC_UARTCFIFO);
	temp |= MXC_UARTCFIFO_RXOFE | MXC_UARTCFIFO_RXUFE;
	writeb(temp, sport->port.membase + MXC_UARTCFIFO);

	/*
	 * Allocate the IRQ(s)
	 * Vybrid chips only have one interrupt.
	 */
	retval = devm_request_irq(port->dev, sport->port.irq, imx_int, 0,
				  DRIVER_NAME, sport);
	if (retval) {
		free_irq(sport->port.irq, sport);
		goto error_out1;
	}

	/* Enable the DMA ops for uart. */
	if (sport->enable_dma) {
		sport->dma_is_txing = 0;
		sport->dma_is_rxing = 0;

		/* enable DMA request generation */
		temp = readb(sport->port.membase + MXC_UARTCR5);
		temp |= MXC_UARTCR5_TDMAS;
		/* RX DMA will be enabled when first RX interrupt occurs */
		temp &= ~MXC_UARTCR5_RDMAS;
		writeb(temp, sport->port.membase + MXC_UARTCR5);

		INIT_WORK(&sport->tsk_dma_tx, dma_tx_work);
	}

	/*
	 * Finally, clear and enable interrupts
	 */

	temp = readb(sport->port.membase + MXC_UARTCR3);
	temp |= MXC_UARTCR3_FEIE;
	writeb(temp, sport->port.membase + MXC_UARTCR3);

	temp = readb(sport->port.membase + MXC_UARTCR2);
	temp |= MXC_UARTCR2_RIE | MXC_UARTCR2_TIE |
		MXC_UARTCR2_RE | MXC_UARTCR2_TE;
	writeb(temp, sport->port.membase + MXC_UARTCR2);

	sport->port.state->port.flags |= ASYNC_INITIALIZED;

	/*
	 * Enable modem status interrupts
	 */
	spin_unlock_irqrestore(&sport->port.lock, flags);

	if (sport->dma_chan_rx)
		imx_load_dma_rx(sport, sport->dma_def_rx_buf, MXC_SLAVE_BUF_SIZE);

	return 0;

error_out1:
	return retval;
}

static void imx_shutdown(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned char temp;
	unsigned long flags;

	if (sport->dma_chan_rx)
		imx_stop_dma_rx(sport);

	if (work_pending(&sport->tsk_dma_tx))
		cancel_work_sync(&sport->tsk_dma_tx);

	spin_lock_irqsave(&sport->port.lock, flags);
	temp = readb(sport->port.membase + MXC_UARTCR2);
	temp &= ~(MXC_UARTCR2_TE | MXC_UARTCR2_RE);
	writeb(temp, sport->port.membase + MXC_UARTCR2);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	devm_free_irq(sport->port.dev, sport->port.irq, sport);

	/*
	 * Disable all interrupts, port and break condition.
	 */

	spin_lock_irqsave(&sport->port.lock, flags);
	temp = readb(sport->port.membase + MXC_UARTCR2);
	temp &= ~(MXC_UARTCR2_TIE | MXC_UARTCR2_TCIE | MXC_UARTCR2_RIE);
	writeb(temp, sport->port.membase + MXC_UARTCR2);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void
imx_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long flags;
	unsigned char cr1, old_cr1, old_cr2, cr4, bdh, modem;
	unsigned int baud;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	unsigned int sbr, brfa;
	int bits, period;

	cr1 = old_cr1 = readb(sport->port.membase + MXC_UARTCR1);
	old_cr2 = readb(sport->port.membase + MXC_UARTCR2);
	cr4 = readb(sport->port.membase + MXC_UARTCR4);
	bdh = readb(sport->port.membase + MXC_UARTBDH);
	modem = readb(sport->port.membase + MXC_UARTMODEM);

	sport->port.uartclk = clk_get_rate(sport->clk);

	/*
	 * We only support CS8 and CS7,but CS7 must enable PE.
	 * supported mode:
	 *  - (7,e/o,1)
	 *  - (8,n,1)
	 *  - (8,m/s,1)
	 *  - (8,e/o,1)
	 */
	while ((termios->c_cflag & CSIZE) != CS8 &&
	       (termios->c_cflag & CSIZE) != CS7) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8 ||
	    (termios->c_cflag & CSIZE) == CS7)
		cr1 = old_cr1 & ~MXC_UARTCR1_M;

	if (termios->c_cflag & CMSPAR) {
		if ((termios->c_cflag & CSIZE) != CS8) {
			termios->c_cflag &= ~CSIZE;
			termios->c_cflag |= CS8;
		}
		cr1 |= MXC_UARTCR1_M;
	}

	modem &= ~(MXC_UARTMODEM_RXRTSE | MXC_UARTMODEM_TXRTSE | MXC_UARTMODEM_TXCTSE);
	if (sport->have_rts) {
		modem |= MXC_UARTMODEM_TXRTSE;
		if (sport->rts_inverted)
			modem |= MXC_UARTMODEM_TXRTSPOL;
		else
			modem &= ~MXC_UARTMODEM_TXRTSPOL;
	}

	if (sport->have_cts)
		modem |= MXC_UARTMODEM_TXCTSE;

	if (termios->c_cflag & CSTOPB)
		termios->c_cflag &= ~CSTOPB;

	/* parity must enable when CS7 to match 8-bits format */
	if ((termios->c_cflag & CSIZE) == CS7)
		termios->c_cflag |= PARENB;

	if ((termios->c_cflag & PARENB)) {
		if (termios->c_cflag & CMSPAR) {
			cr1 &= ~MXC_UARTCR1_PE;
			cr1 |= MXC_UARTCR1_M;
		} else {
			cr1 |= MXC_UARTCR1_PE;
			if ((termios->c_cflag & CSIZE) == CS8)
				cr1 |= MXC_UARTCR1_M;
			if (termios->c_cflag & PARODD)
				cr1 |= MXC_UARTCR1_PT;
			else
				cr1 &= ~MXC_UARTCR1_PT;
		}
	}

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);

	spin_lock_irqsave(&sport->port.lock, flags);

	if (sport->dma_chan_rx) {
		/*
		   DMA timeout has the fixed value in order to guarantee a
		   response within this time. Calculate dma period len
		   so that a single dma transaction could be complete within
		   the timeout.
		*/
		switch (termios->c_cflag & CSIZE) {
		case CS5:
			bits = 7;
			break;
		case CS6:
			bits = 8;
			break;
		case CS7:
			bits = 9;
			break;
		default:
			bits = 10;
			break; /* CS8 */
		}

		if (termios->c_cflag & CSTOPB)
			bits++;
		if (termios->c_cflag & PARENB)
			bits++;

		period = MXC_DMA_RX_TIMEOUT_MSEC * baud * 2 / bits / 1000 / 3;

		sport->dma_rx_period = 1 << (32 - __builtin_clz (period - 1));

		if (sport->dma_rx_period > period) {
			sport->dma_rx_period >>= 1;
		}

		if (sport->dma_rx_period >= MXC_SLAVE_BUF_SIZE) {
			sport->dma_rx_period = MXC_SLAVE_BUF_SIZE / 2;
		}
	}

	sport->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |=
			(MXC_UARTSR1_FE | MXC_UARTSR1_PE);
	if (termios->c_iflag & (BRKINT | PARMRK))
		sport->port.read_status_mask |= MXC_UARTSR1_FE;

	/*
	 * Characters to ignore
	 */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |= MXC_UARTSR1_PE;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= MXC_UARTSR1_FE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= MXC_UARTSR1_OR;
	}

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/* wait transmit engin complete */
	while (!(readb(sport->port.membase + MXC_UARTSR1) & MXC_UARTSR1_TC))
		barrier();

	writeb(old_cr2 & ~(MXC_UARTCR2_TE | MXC_UARTCR2_RE),
	       sport->port.membase + MXC_UARTCR2);

	/* disable transmit and receive */
	sbr = sport->port.uartclk / (16 * baud);
	brfa = ((sport->port.uartclk - (16 * sbr * baud)) * 2)/baud;

	bdh &= ~MXC_UARTBDH_SBR_MASK;
	bdh |= (sbr >> 8) & 0x1F;

	cr4 &= ~MXC_UARTCR4_BRFA_MASK;
	brfa &= MXC_UARTCR4_BRFA_MASK;
	writeb(cr4 | brfa, sport->port.membase + MXC_UARTCR4);
	writeb(bdh, sport->port.membase + MXC_UARTBDH);
	writeb(sbr & 0xFF, sport->port.membase + MXC_UARTBDL);
	writeb(cr1, sport->port.membase + MXC_UARTCR1);
	writeb(modem, sport->port.membase + MXC_UARTMODEM);

	/* restore control register */
	writeb(old_cr2, sport->port.membase + MXC_UARTCR2);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static const char *imx_type(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;

	return sport->port.type == PORT_IMX ? "IMX" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void imx_release_port(struct uart_port *port)
{
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int imx_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void imx_config_port(struct uart_port *port, int flags)
{
	struct imx_port *sport = (struct imx_port *)port;

	if (flags & UART_CONFIG_TYPE &&
	    imx_request_port(&sport->port) == 0)
		sport->port.type = PORT_IMX;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_IMX and PORT_UNKNOWN
 */
static int
imx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct imx_port *sport = (struct imx_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_IMX)
		ret = -EINVAL;
	if (sport->port.irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (sport->port.uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if (sport->port.iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static int imx_uart_ioctl(struct uart_port *uport, unsigned int cmd,
			  unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static struct uart_ops imx_pops = {
	.tx_empty	= imx_tx_empty,
	.set_mctrl	= imx_set_mctrl,
	.get_mctrl	= imx_get_mctrl,
	.stop_tx	= imx_stop_tx,
	.start_tx	= imx_start_tx,
	.stop_rx	= imx_stop_rx,
	.break_ctl	= imx_break_ctl,
	.startup	= imx_startup,
	.shutdown	= imx_shutdown,
	.set_termios	= imx_set_termios,
	.type		= imx_type,
	.release_port	= imx_release_port,
	.request_port	= imx_request_port,
	.config_port	= imx_config_port,
	.verify_port	= imx_verify_port,
	.ioctl		= imx_uart_ioctl,
};

static struct imx_port *imx_ports[UART_NR];

#ifdef CONFIG_SERIAL_VF610_UART_CONSOLE
static void imx_console_putchar(struct uart_port *port, int ch)
{
	struct imx_port *sport = (struct imx_port *)port;

	while (!(readb(sport->port.membase + MXC_UARTSR1) & MXC_UARTSR1_TDRE))
		barrier();

	writeb(ch, sport->port.membase + MXC_UARTDR);
}

/*
 * Interrupts are disabled on entering
 */
static void
imx_console_write(struct console *co, const char *s, unsigned int count)
{
	struct imx_port *sport = imx_ports[co->index];
	unsigned int  old_cr2, cr2;

	/*
	 *	First, save UCR1/2 and then disable interrupts
	 */
	cr2 = old_cr2 = readb(sport->port.membase + MXC_UARTCR2);


	cr2 |= (MXC_UARTCR2_TE |  MXC_UARTCR2_RE);
	cr2 &= ~(MXC_UARTCR2_TIE | MXC_UARTCR2_TCIE | MXC_UARTCR2_RIE);

	writeb(cr2, sport->port.membase + MXC_UARTCR2);


	uart_console_write(&sport->port, s, count, imx_console_putchar);

	/*
	 *	Finally, wait for transmitter finish complete
	 *	and restore CR2
	 */
	while (!(readb(sport->port.membase + MXC_UARTSR1) & MXC_UARTSR1_TC))
		;

	writeb(old_cr2, sport->port.membase + MXC_UARTCR2);
}

/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
imx_console_get_options(struct imx_port *sport, int *baud,
			int *parity, int *bits)
{
	if (readb(sport->port.membase + MXC_UARTCR2) &
	    (MXC_UARTCR2_TE | MXC_UARTCR2)) {
		/* ok, the port was enabled */
		unsigned char cr1, bdh, bdl, brfa;
		unsigned int sbr, uartclk;
		unsigned int baud_raw;

		cr1 = readb(sport->port.membase + MXC_UARTCR1);

		*parity = 'n';
		if (cr1 & MXC_UARTCR1_PE) {
			if (cr1 & MXC_UARTCR1_PT)
				*parity = 'o';
			else
				*parity = 'e';
		}

		if (cr1 & MXC_UARTCR1_M)
			*bits = 9;
		else
			*bits = 8;

		bdh = readb(sport->port.membase + MXC_UARTBDH) &
			MXC_UARTBDH_SBR_MASK;
		bdl = readb(sport->port.membase + MXC_UARTBDL);
		sbr = bdh;
		sbr <<= 8;
		sbr |= bdl;
		brfa = readb(sport->port.membase + MXC_UARTCR4) &
			MXC_UARTCR4_BRFA_MASK;

		uartclk = clk_get_rate(sport->clk);
		/*
		 * Baud = mod_clk/(16*(sbr[13]+(brfa)/32)
		 */
		baud_raw = uartclk/(16 * (sbr + brfa/32));

		if (*baud != baud_raw)
			printk(KERN_INFO "Serial: Console IMX "
			       "rounded baud rate from %d to %d\n",
			       baud_raw, *baud);
	}
}

static int __init
imx_console_setup(struct console *co, char *options)
{
	struct imx_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(imx_ports))
		co->index = 0;
	sport = imx_ports[co->index];

	if (sport == NULL)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		imx_console_get_options(sport, &baud, &parity, &bits);

	if (sport->fifo_en == 1)
		imx_setup_watermark(sport, 0);

	return uart_set_options(&sport->port, co, baud, parity, bits, flow);
}

static struct uart_driver imx_reg;
static struct console imx_console = {
	.name		= DEV_NAME,
	.write		= imx_console_write,
	.device		= uart_console_device,
	.setup		= imx_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &imx_reg,
};

#define IMX_CONSOLE	(&imx_console)
#else
#define IMX_CONSOLE	NULL
#endif /* CONFIG_SERIAL_VF610_UART_CONSOLE */

static struct uart_driver imx_reg = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.nr             = ARRAY_SIZE(imx_ports),
	.cons           = IMX_CONSOLE,
};

#ifdef CONFIG_PM_SLEEP
static int serial_imx_suspend(struct device *dev)
{
	struct imx_port *sport = dev_get_drvdata(dev);

	if (sport) {
		struct tty_struct *tty = tty_port_tty_get(&sport->port.state->port);
		int may_wakeup = (tty ? device_may_wakeup(tty->dev) : 0);

		uart_suspend_port(&imx_reg, &sport->port);

		if (may_wakeup) {
			if (sport->dma_chan_rx) {
				/* Disable DMA on RX */
				unsigned long temp;
				temp = readb(sport->port.membase + MXC_UARTCR5);
				temp &= ~MXC_UARTCR5_RDMAS;
				writeb(temp, sport->port.membase + MXC_UARTCR5);
			}
		} else {
			if (console_suspend_enabled ||
					!uart_console(&sport->port))
				clk_disable_unprepare(sport->clk);
		}
	}

	return 0;
}

static int serial_imx_resume(struct device *dev)
{
	struct imx_port *sport = dev_get_drvdata(dev);

	if (sport) {
		struct tty_struct *tty = tty_port_tty_get(&sport->port.state->port);
		int may_wakeup = (tty ? device_may_wakeup(tty->dev) : 0);

		if (may_wakeup) {
			if (sport->dma_chan_rx) {
				/* Restore DMA settings */
				unsigned long temp;
				temp = readb(sport->port.membase + MXC_UARTCR5);
				temp |= MXC_UARTCR5_RDMAS;
				writeb(temp, sport->port.membase + MXC_UARTCR5);
			}
		} else {
			if (console_suspend_enabled ||
					!uart_console(&sport->port))
				clk_prepare_enable(sport->clk);
		}

		uart_resume_port(&imx_reg, &sport->port);
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static void serial_setup_dma_rx(struct imx_port *sport)
{
	struct dma_slave_config rxconf;
	unsigned char val;
	int err;

	sport->dma_chan_rx = dma_request_slave_channel(sport->port.dev, "rx");
	if (!sport->dma_chan_rx) {
		dev_warn(sport->port.dev, "Failed to requets RX DMA channel\n");
		goto fail_request;
	}

	rxconf.direction = DMA_DEV_TO_MEM;
	rxconf.src_addr = sport->membase_phys + MXC_UARTDR;
	rxconf.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	rxconf.src_maxburst = 1;

	err = dmaengine_slave_config(sport->dma_chan_rx, &rxconf);
	if (err < 0) {
		dev_err(sport->port.dev, "Can't configure rx channel: %d\n", err);
		goto fail_conf;
	}

	sport->dma_def_rx_buf = devm_kzalloc(sport->port.dev, MXC_SLAVE_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (!sport->dma_def_rx_buf) {
		dev_err(sport->port.dev, "Can't allocate buffer for rx DMA\n");
		goto fail_conf;
	}

	/* The RX period will be updated when baud rate is specified.
	   Use default value for a while. */
	sport->dma_rx_period = MXC_DMA_RX_PERIOD;

	init_timer(&sport->dma_rx_timer);
	sport->dma_rx_timer.function	= imx_dma_rx_timeout;
	sport->dma_rx_timer.data	= (unsigned long)sport;

	val = readb(sport->port.membase + MXC_UARTPFIFO);
	val |= MXC_UARTPFIFO_RXFE;
	writeb(val, sport->port.membase + MXC_UARTPFIFO);

	return;

fail_conf:
	dma_release_channel(sport->dma_chan_rx);
	sport->dma_chan_rx = NULL;
fail_request:
	dev_info(sport->port.dev, "Can't use RX DMA\n");
}

static int serial_imx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct imx_port *sport;
	struct resource *res;
	int err = 0;

	sport = kzalloc(sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	sport->port.line = of_alias_get_id(np, "serial");
	if (sport->port.line < 0) {
		err = sport->port.line;
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", err);
		return err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENODEV;
		goto free;
	}

	sport->membase_phys = res->start;
	sport->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (!sport->port.membase) {
		err = -ENOMEM;
		goto free;
	}

	sport->port.dev		= &pdev->dev;
	sport->port.mapbase	= res->start;
	sport->port.type	= PORT_IMX;
	sport->port.iotype	= UPIO_MEM;
	sport->port.irq		= platform_get_irq(pdev, 0);
	sport->port.fifosize	= 32;
	sport->port.ops		= &imx_pops;
	sport->port.flags	= UPF_BOOT_AUTOCONF;
	sport->fifo_en		= 1;
	sport->tx_done		= 1;

	sport->clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(sport->clk)) {
		err = PTR_ERR(sport->clk);
		dev_err(&pdev->dev, "failed to get uart clk: %d\n", err);
		goto unmap;
	}

	err = clk_prepare_enable(sport->clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable uart clk: %d\n", err);
		goto clkput;
	}

	sport->port.uartclk = clk_get_rate(sport->clk);
	imx_ports[sport->port.line] = sport;

	sport->dma_chan_tx = dma_request_slave_channel(sport->port.dev, "tx");
	if (sport->dma_chan_tx) {
		sport->enable_dma = 1;
	} else {
		err = -ENODEV;
		dev_err(sport->port.dev, "Cannot operate without DMA tx channel\n");
		goto clkput;
	}

	serial_setup_dma_rx(sport);

	sport->have_rts = false;
	sport->have_cts = false;
	sport->rts_inverted = false;

	if (of_find_property(np, "fsl,uart-has-rtscts", NULL))
		sport->have_rts = sport->have_cts = true;

	if (of_find_property(np, "fsl,uart-has-rts", NULL))
		sport->have_rts = true;

	if (of_find_property(np, "fsl,uart-has-cts", NULL))
		sport->have_cts = true;

	if (of_find_property(np, "fsl,uart-rts-inverted", NULL))
		sport->rts_inverted = true;

	sport->gpios = mctrl_gpio_init(&pdev->dev, 0);
	if (IS_ERR_OR_NULL(sport->gpios))
		sport->gpios = NULL;

	sport->rs485_ctrl_gpio = devm_gpiod_get(&pdev->dev,
						"rs485-ctrl", GPIOD_OUT_LOW);
	if (IS_ERR_OR_NULL(sport->rs485_ctrl_gpio)) {
		int cts_err = sport->rs485_ctrl_gpio ? PTR_ERR(sport->rs485_ctrl_gpio) : -EINVAL;
		sport->rs485_ctrl_gpio = NULL;
		dev_info(&pdev->dev, "No gpio for RS-485 mode: err %d\n", cts_err);
	}

	platform_set_drvdata(pdev, &sport->port);

	err = uart_add_one_port(&imx_reg, &sport->port);

	if (!err)
		return 0;

clkput:
	clk_put(sport->clk);
	clk_disable(sport->clk);
unmap:
	iounmap(sport->port.membase);
free:
	kfree(sport);

	return err;
}

static int serial_imx_remove(struct platform_device *pdev)
{
	struct imx_port *sport = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (sport) {
		uart_remove_one_port(&imx_reg, &sport->port);
		clk_put(sport->clk);
	}

	clk_disable(sport->clk);

	iounmap(sport->port.membase);
	kfree(sport);

	return 0;
}

static SIMPLE_DEV_PM_OPS(serial_imx_pm_ops, serial_imx_suspend, serial_imx_resume);

static struct platform_driver serial_imx_driver = {
	.probe		= serial_imx_probe,
	.remove		= serial_imx_remove,

	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = imx_uart_dt_ids,
		.pm	= &serial_imx_pm_ops,
	},
};

static int __init imx_serial_init(void)
{
	int ret;

	printk(KERN_INFO "Serial: VF610 driver\n");

	ret = uart_register_driver(&imx_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&serial_imx_driver);
	if (ret != 0)
		uart_unregister_driver(&imx_reg);

	return 0;
}

static void __exit imx_serial_exit(void)
{
	platform_driver_unregister(&serial_imx_driver);
	uart_unregister_driver(&imx_reg);
}

module_init(imx_serial_init);
module_exit(imx_serial_exit);

MODULE_AUTHOR("Sascha Hauer");
MODULE_DESCRIPTION("IMX generic serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-uart");
