/*
 * Copyright (C) EmCraft Systems 2015
 * Author: Yuri Tikhonov <yur@emcraft.com>
 * License terms:  GNU General Public License (GPL), version 2
 *
 * STM32 DMA routines used by STM32 UART drivers
 */

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#include "stm32-usart.h"

/*
 * RX DMA routines
 */
static void stm32_complete_rx_dma(void *arg)
{
	struct uart_port *port = arg;
	struct stm32_port *stm32_port = to_stm32_port(port);

	tasklet_schedule(&stm32_port->tasklet);
}

bool stm32_use_dma_rx(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);

	return stm32_port->dma_rx.use;
}

void stm32_rx_from_dma(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_dma_data *dma = &stm32_port->dma_rx;
	struct tty_port *tport = &port->state->port;
	struct circ_buf *ring = &stm32_port->rx_ring;
	struct dma_chan *chan = dma->chan;
	struct dma_tx_state state;
	enum dma_status dmastat;
	size_t count;

	dmastat = dmaengine_tx_status(chan, dma->cookie, &state);
	/* Restart a new tasklet if DMA status is error */
	if (dmastat == DMA_ERROR) {
		dev_dbg(port->dev, "Get residue error, restart tasklet\n");
		tasklet_schedule(&stm32_port->tasklet);
		return;
	}

	/* CPU claims ownership of RX DMA buffer */
	dma_sync_sg_for_cpu(port->dev, &dma->sg, 1, DMA_FROM_DEVICE);

	/*
	 * ring->head points to the end of data already written by the DMA.
	 * ring->tail points to the beginning of data to be read by the
	 * framework.
	 * The current transfer size should not be larger than the dma buffer
	 * length.
	 */
	ring->head = sg_dma_len(&dma->sg) - state.residue;
	BUG_ON(ring->head > sg_dma_len(&dma->sg));
	/*
	 * At this point ring->head may point to the first byte right after the
	 * last byte of the dma buffer:
	 * 0 <= ring->head <= sg_dma_len(&stm32_port->sg_rx)
	 *
	 * However ring->tail must always points inside the dma buffer:
	 * 0 <= ring->tail <= sg_dma_len(&stm32_port->sg_rx) - 1
	 *
	 * Since we use a ring buffer, we have to handle the case
	 * where head is lower than tail. In such a case, we first read from
	 * tail to the end of the buffer then reset tail.
	 */
	if (ring->head < ring->tail) {
		count = sg_dma_len(&dma->sg) - ring->tail;

		tty_insert_flip_string(tport, ring->buf + ring->tail, count);
		ring->tail = 0;
		port->icount.rx += count;
	}

	/* Finally we read data from tail to head */
	if (ring->tail < ring->head) {
		count = ring->head - ring->tail;

		tty_insert_flip_string(tport, ring->buf + ring->tail, count);
		/* Wrap ring->head if needed */
		if (ring->head >= sg_dma_len(&dma->sg))
			ring->head = 0;
		ring->tail = ring->head;
		port->icount.rx += count;
	}

	/* USART retreives ownership of RX DMA buffer */
	dma_sync_sg_for_device(port->dev, &dma->sg, 1, DMA_FROM_DEVICE);

	/*
	 * Drop the lock here since it might end up calling
	 * uart_start(), which takes the lock.
	 */
	spin_unlock(&port->lock);
	tty_flip_buffer_push(tport);
	spin_lock(&port->lock);
}

void stm32_release_rx_dma(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_dma_data *dma = &stm32_port->dma_rx;
	struct dma_chan *chan = dma->chan;

	if (chan) {
		dmaengine_terminate_all(chan);
		dma_release_channel(chan);
		dma_unmap_sg(port->dev, &dma->sg, 1, DMA_FROM_DEVICE);
	}

	dma->dsc = NULL;
	dma->chan = NULL;
	dma->cookie = -EINVAL;
}

int stm32_prepare_rx_dma(struct uart_port *port, unsigned int dr_offset,
			 unsigned int ring_size)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_dma_data *dma = &stm32_port->dma_rx;
	struct dma_async_tx_descriptor *dsc;
	struct circ_buf *ring;
	dma_cap_mask_t mask;
	struct dma_slave_config config;
	int ret, nent;

	ring = &stm32_port->rx_ring;

	dma_cap_zero(mask);
	dma_cap_set(DMA_CYCLIC, mask);

	dma->chan = dma_request_slave_channel(port->dev, "rx");
	if (!dma->chan)
		goto chan_err;

	spin_lock_init(&dma->lock);
	sg_init_table(&dma->sg, 1);
	BUG_ON(!PAGE_ALIGNED(ring->buf));
	sg_set_page(&dma->sg, virt_to_page(ring->buf),
		    sizeof(struct stm32_uart_char) * ring_size,
		    (int)ring->buf & ~PAGE_MASK);

	nent = dma_map_sg(port->dev, &dma->sg, 1, DMA_FROM_DEVICE);
	if (!nent) {
		dev_dbg(port->dev, "need to release resource of dma\n");
		goto chan_err;
	}
	dev_dbg(port->dev, "%s: mapped %d@%p to %x\n", __func__,
		sg_dma_len(&dma->sg), ring->buf, sg_dma_address(&dma->sg));

	/* Configure the slave DMA */
	memset(&config, 0, sizeof(config));
	config.direction = DMA_DEV_TO_MEM;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr = port->mapbase + dr_offset;
	config.src_maxburst = 1;
	ret = dmaengine_slave_config(dma->chan, &config);
	if (ret) {
		dev_err(port->dev, "DMA rx slave configuration failed\n");
		goto chan_err;
	}

	/*
	 * Prepare a cyclic dma transfer with half ring buffer size
	 * period
	 */
	dsc = dmaengine_prep_dma_cyclic(dma->chan, sg_dma_address(&dma->sg),
				sg_dma_len(&dma->sg), sg_dma_len(&dma->sg) / 2,
				DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	dsc->callback = stm32_complete_rx_dma;
	dsc->callback_param = port;
	dma->dsc = dsc;
	dma->cookie = dmaengine_submit(dsc);

	dev_dbg(port->dev, "using %s for rx DMA transfers\n",
		dma_chan_name(dma->chan));
	return 0;
chan_err:
	dev_err(port->dev, "DMA RX channel not available, switch to pio\n");
	dma->use = 0;
	if (dma->chan)
		stm32_release_rx_dma(port);
	return -EINVAL;
}

/*
 * TX DMA routines
 */
static void stm32_complete_tx_dma(void *arg)
{
	struct stm32_port *stm32_port = arg;
	struct uart_port *port = &stm32_port->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct stm32_dma_data *dma = &stm32_port->dma_tx;
	struct dma_chan *chan = dma->chan;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	if (chan)
		dmaengine_terminate_all(chan);
	xmit->tail += sg_dma_len(&dma->sg);
	xmit->tail &= UART_XMIT_SIZE - 1;

	port->icount.tx += sg_dma_len(&dma->sg);

	spin_lock_irq(&dma->lock);
	async_tx_ack(dma->dsc);
	dma->cookie = -EINVAL;
	dma->dsc = NULL;
	spin_unlock_irq(&dma->lock);
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	/*
	 * xmit is a circular buffer so, if we have just send data from
	 * xmit->tail to the end of xmit->buf, now we have to transmit the
	 * remaining data from the beginning of xmit->buf to xmit->head.
	 */
	if (!uart_circ_empty(xmit))
		tasklet_schedule(&stm32_port->tasklet);
	else if (stm32_port->de) {
		unsigned long timeout = 1000000;
		if (stm32_port->tx_active)
			while (stm32_port->tx_active(port) && --timeout);
		gpiod_set_value(stm32_port->de, 0);
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

bool stm32_use_dma_tx(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);

	return stm32_port->dma_tx.use;
}

/*
 * Called from tasklet with TXRDY interrupt is disabled.
 */
void stm32_tx_with_dma(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct circ_buf *xmit = &port->state->xmit;
	struct stm32_dma_data *dma = &stm32_port->dma_tx;
	struct dma_chan *chan = dma->chan;
	struct dma_async_tx_descriptor *dsc;
	struct scatterlist *sg = &dma->sg;

	/* Make sure we have an idle channel */
	if (dma->dsc)
		return;

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(port)) {
		if (stm32_port->de)
			gpiod_set_value(stm32_port->de, 1);

		/*
		 * DMA is idle now.
		 * Port xmit buffer is already mapped,
		 * and it is one page... Just adjust
		 * offsets and lengths. Since it is a circular buffer,
		 * we have to transmit till the end, and then the rest.
		 * Take the port lock to get a
		 * consistent xmit buffer state.
		 */
		sg->offset = xmit->tail & (UART_XMIT_SIZE - 1);
		sg_dma_address(sg) = (sg_dma_address(sg) &
					~(UART_XMIT_SIZE - 1))
					+ sg->offset;
		sg_dma_len(sg) = CIRC_CNT_TO_END(xmit->head,
						xmit->tail,
						UART_XMIT_SIZE);
		BUG_ON(!sg_dma_len(sg));

		dsc = dmaengine_prep_slave_sg(chan, sg, 1,
			DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!dsc) {
			dev_err(port->dev, "Failed to send via dma!\n");
			return;
		}

		dma_sync_sg_for_device(port->dev, sg, 1, DMA_TO_DEVICE);
		dma->dsc = dsc;
		dsc->callback = stm32_complete_tx_dma;
		dsc->callback_param = stm32_port;
		dsc->cookie = dmaengine_submit(dsc);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

void stm32_release_tx_dma(struct uart_port *port)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_dma_data *dma = &stm32_port->dma_tx;
	struct dma_chan *chan = dma->chan;

	if (chan) {
		dmaengine_terminate_all(chan);
		dma_release_channel(chan);
		dma_unmap_sg(port->dev, &dma->sg, 1, DMA_TO_DEVICE);
	}

	dma->dsc = NULL;
	dma->chan = NULL;
	dma->cookie = -EINVAL;
}

int stm32_prepare_tx_dma(struct uart_port *port, unsigned int dr_offset)
{
	struct stm32_port *stm32_port = to_stm32_port(port);
	struct stm32_dma_data *dma = &stm32_port->dma_tx;
	dma_cap_mask_t mask;
	struct dma_slave_config config;
	int ret, nent;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	dma->chan = dma_request_slave_channel(port->dev, "tx");
	if (!dma->chan)
		goto chan_err;
	dev_dbg(port->dev, "using %s for tx DMA transfers\n",
		dma_chan_name(dma->chan));

	spin_lock_init(&dma->lock);
	sg_init_table(&dma->sg, 1);
	/* UART circular tx buffer is an aligned page. */
	BUG_ON(!PAGE_ALIGNED(port->state->xmit.buf));
	sg_set_page(&dma->sg, virt_to_page(port->state->xmit.buf),
		    UART_XMIT_SIZE, (int)port->state->xmit.buf & ~PAGE_MASK);
	nent = dma_map_sg(port->dev, &dma->sg, 1, DMA_TO_DEVICE);
	if (!nent) {
		dev_dbg(port->dev, "need to release resource of dma\n");
		goto chan_err;
	} else {
		dev_dbg(port->dev, "%s: mapped %d@%p to %x\n", __func__,
			sg_dma_len(&dma->sg),
			port->state->xmit.buf,
			sg_dma_address(&dma->sg));
	}

	/* Configure the slave DMA */
	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr = port->mapbase + dr_offset;
	config.dst_maxburst = 1;

	ret = dmaengine_slave_config(dma->chan, &config);
	if (ret) {
		dev_err(port->dev, "DMA tx slave configuration failed\n");
		goto chan_err;
	}

	return 0;

chan_err:
	dev_err(port->dev, "TX channel not available, switch to pio\n");
	dma->use = 0;
	if (dma->chan)
		stm32_release_tx_dma(port);
	return -EINVAL;
}
