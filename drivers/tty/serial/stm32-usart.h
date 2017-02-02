/*
 * Copyright (C) EmCraft Systems 2015
 * Author:  Yuri Tikhonov <yur@emcraft.com>
 * License terms:  GNU General Public License (GPL), version 2
 */
#ifndef _STM32_USART_H_
#define _STM32_USART_H_

#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

/*
 * Char descriptor
 */
struct stm32_uart_char {
	u32			sr;
	u32			ch;
};

/*
 * STM32 DMA data
 */
struct stm32_dma_data {
	struct dma_async_tx_descriptor *dsc;
	struct dma_chan		*chan;
	dma_cookie_t		cookie;
	struct scatterlist	sg;
	spinlock_t		lock;
	bool			use;
};

/*
 * STM32 USART port descriptor
 */
struct stm32_port {
	struct uart_port	port;
	struct clk		*clk;
	struct gpio_desc	*de;
	bool			hw_flow_control;

	struct circ_buf		rx_ring;
	struct tasklet_struct	tasklet;
	dma_addr_t		rx_ring_phy;

	struct stm32_dma_data	dma_rx;
	struct stm32_dma_data	dma_tx;

	int  (*tx_active)(struct uart_port *);

	int  (*prepare_rx)(struct uart_port *, unsigned int, unsigned int);
	void (*schedule_rx)(struct uart_port *);
	void (*release_rx)(struct uart_port *);

	int  (*prepare_tx)(struct uart_port *, unsigned int);
	void (*schedule_tx)(struct uart_port *);
	void (*release_tx)(struct uart_port *);
};

/*
 * Helpers
 */
static inline struct stm32_port *to_stm32_port(struct uart_port *port)
{
	return container_of(port, struct stm32_port, port);
}

/*
 * Common prototypes
 */
int  stm32_uart_of_init(struct uart_port *port, struct platform_device *pdev);
void stm32_uart_of_deinit(struct uart_port *port);

bool stm32_use_dma_rx(struct uart_port *port);
int  stm32_prepare_rx_dma(struct uart_port *port, unsigned int dr_offset,
			  unsigned int ring_size);
void stm32_rx_from_dma(struct uart_port *port);
void stm32_release_rx_dma(struct uart_port *port);

bool stm32_use_dma_tx(struct uart_port *port);
int  stm32_prepare_tx_dma(struct uart_port *port, unsigned int dr_offset);
void stm32_tx_with_dma(struct uart_port *port);
void stm32_release_tx_dma(struct uart_port *port);

#endif /* _STM32_USART_H_ */
