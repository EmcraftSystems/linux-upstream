/*
 * drivers/spi/spi-fsl-dspi.c
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Freescale DSPI driver
 * This file contains a driver for the Freescale DSPI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/dma-mapping.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/workqueue.h>

#define DRIVER_NAME "fsl-dspi"

#define TRAN_STATE_RX_VOID		0x01
#define TRAN_STATE_TX_VOID		0x02
#define TRAN_STATE_WORD_ODD_NUM	0x04

#define DSPI_FIFO_SIZE			4
#define DSPI_SLAVE_BUF_SIZE		128
#define DSPI_SLAVE_SEGMENT_SIZE		DSPI_SLAVE_BUF_SIZE

#define SPI_MCR		0x00
#define SPI_MCR_MASTER		(1 << 31)
#define SPI_MCR_PCSSE		(1 << 25)
#define SPI_MCR_PCSIS		(0x3F << 16)
#define SPI_MCR_CLR_TXF		(1 << 11)
#define SPI_MCR_CLR_RXF		(1 << 10)
#define SPI_MCR_HALT		(1 << 0)

#define SPI_TCR			0x08
#define SPI_TCR_GET_TCNT(x)	(((x) & 0xffff0000) >> 16)

#define SPI_CTAR(x)		(0x0c + (((x) & 0x3) * 4))
#define SPI_CTAR_FMSZ(x)	(((x) & 0x0000000f) << 27)
#define SPI_CTAR_CPOL(x)	((x) << 26)
#define SPI_CTAR_CPHA(x)	((x) << 25)
#define SPI_CTAR_LSBFE(x)	((x) << 24)
#define SPI_CTAR_PCSSCK(x)	(((x) & 0x00000003) << 22)
#define SPI_CTAR_PASC(x)	(((x) & 0x00000003) << 20)
#define SPI_CTAR_PDT(x)	(((x) & 0x00000003) << 18)
#define SPI_CTAR_PBR(x)	(((x) & 0x00000003) << 16)
#define SPI_CTAR_CSSCK(x)	(((x) & 0x0000000f) << 12)
#define SPI_CTAR_ASC(x)	(((x) & 0x0000000f) << 8)
#define SPI_CTAR_DT(x)		(((x) & 0x0000000f) << 4)
#define SPI_CTAR_BR(x)		((x) & 0x0000000f)
#define SPI_CTAR_SCALE_BITS	0xf

#define SPI_CTAR0_SLAVE	0x0c

#define SPI_SR			0x2c
#define SPI_SR_EOQF		0x10000000
#define SPI_SR_TCFQF		0x80000000
#define SPI_SR_TFFF		(1 << 25)
#define SPI_SR_TXCTR(x)		(((x) >> 12) & 0xf)

#define SPI_RSER		0x30
#define SPI_RSER_EOQFE		(1 << 28)
#define SPI_RSER_TCFQE		(1 << 31)
#define SPI_RSER_TFFFRE		(1 << 25)
#define SPI_RSER_TFFF_DMA	(1 << 24)
#define SPI_RSER_RFDFRE		(1 << 17)
#define SPI_RSER_RFDF_DMA	(1 << 16)

#define SPI_RSER_TFUFRE		(1 << 27)
#define SPI_RSER_SPEFRE		(1 << 21)
#define SPI_RSER_RFOFRE		(1 << 19)

#define SPI_PUSHR		0x34
#define SPI_PUSHR_CONT		(1 << 31)
#define SPI_PUSHR_CTAS(x)	(((x) & 0x00000003) << 28)
#define SPI_PUSHR_EOQ		(1 << 27)
#define SPI_PUSHR_CTCNT	(1 << 26)
#define SPI_PUSHR_PCS(x)	(((1 << x) & 0x0000003f) << 16)
#define SPI_PUSHR_TXDATA(x)	((x) & 0x0000ffff)

#define SPI_PUSHR_SLAVE	0x34

#define SPI_POPR		0x38
#define SPI_POPR_RXDATA(x)	((x) & 0x0000ffff)

#define SPI_TXFR0		0x3c
#define SPI_TXFR1		0x40
#define SPI_TXFR2		0x44
#define SPI_TXFR3		0x48
#define SPI_RXFR0		0x7c
#define SPI_RXFR1		0x80
#define SPI_RXFR2		0x84
#define SPI_RXFR3		0x88

#define SPI_FRAME_BITS(bits)	SPI_CTAR_FMSZ((bits) - 1)
#define SPI_FRAME_BITS_MASK	SPI_CTAR_FMSZ(0xf)
#define SPI_FRAME_BITS_16	SPI_CTAR_FMSZ(0xf)
#define SPI_FRAME_BITS_8	SPI_CTAR_FMSZ(0x7)

#define SPI_CS_INIT		0x01
#define SPI_CS_ASSERT		0x02
#define SPI_CS_DROP		0x04

#define SPI_TCR_TCNT_MAX	0x10000

struct chip_data {
	u32 mcr_val;
	u32 ctar_val;
	u16 void_write_data;
};

enum dspi_trans_mode {
	DSPI_EOQ_MODE = 0,
	DSPI_TCFQ_MODE,
};

struct fsl_dspi_devtype_data {
	enum dspi_trans_mode trans_mode;
};

static const struct fsl_dspi_devtype_data vf610_data = {
	.trans_mode = DSPI_EOQ_MODE,
};

static const struct fsl_dspi_devtype_data ls1021a_v1_data = {
	.trans_mode = DSPI_TCFQ_MODE,
};

static const struct fsl_dspi_devtype_data ls2085a_data = {
	.trans_mode = DSPI_TCFQ_MODE,
};

struct fsl_dspi {
	struct spi_master	*master;
	struct platform_device	*pdev;

	struct regmap		*regmap;
	int			irq;
	struct clk		*clk;

	struct spi_transfer	*cur_transfer;
	struct spi_message	*cur_msg;
	struct chip_data	*cur_chip;
	size_t			len;
	void			*tx;
	void			*tx_end;
	void			*rx;
	void			*rx_end;
	char			dataflags;
	u8			cs;
	u16			void_write_data;
	u32			cs_change;
	struct fsl_dspi_devtype_data *devtype_data;

	wait_queue_head_t	waitq;
	u32			waitflags;

	u32			spi_tcnt;

	struct dma_chan		*dma_chan_rx;
	struct dma_chan		*dma_chan_tx;

	u8			*dma_def_rx_buf;
	u8			*dma_def_tx_buf;
	void			*dma_rx_buf;
	void			*dma_tx_buf;
	dma_addr_t		dma_rx_buf_phys;
	dma_addr_t		dma_tx_buf_phys;
	size_t			dma_rx_received_bytes;
	size_t			dma_rx_len;
	size_t			dma_tx_len;

	dma_addr_t		phy_addr;
	bool			is_response_prepared;
	bool			enable_slave;
	struct workqueue_struct	*slave_wq;
	struct work_struct	slave_work;
	struct completion	slave_complete;

	void			*slave_spi;
	u8			*slave_resp;
	spi_cmd_cb_t		cmd_cb;
	spi_resp_cb_t		resp_cb;
};

static inline int is_double_byte_mode(struct fsl_dspi *dspi)
{
	unsigned int val;

	regmap_read(dspi->regmap, SPI_CTAR(dspi->cs), &val);

	return ((val & SPI_FRAME_BITS_MASK) == SPI_FRAME_BITS(8)) ? 0 : 1;
}

static void hz_to_spi_baud(char *pbr, char *br, int speed_hz,
		unsigned long clkrate)
{
	/* Valid baud rate pre-scaler values */
	int pbr_tbl[4] = {2, 3, 5, 7};
	int brs[16] = {	2,	4,	6,	8,
		16,	32,	64,	128,
		256,	512,	1024,	2048,
		4096,	8192,	16384,	32768 };
	int scale_needed, scale, minscale = INT_MAX;
	int i, j;

	scale_needed = clkrate / speed_hz;
	if (clkrate % speed_hz)
		scale_needed++;

	for (i = 0; i < ARRAY_SIZE(brs); i++)
		for (j = 0; j < ARRAY_SIZE(pbr_tbl); j++) {
			scale = brs[i] * pbr_tbl[j];
			if (scale >= scale_needed) {
				if (scale < minscale) {
					minscale = scale;
					*br = i;
					*pbr = j;
				}
				break;
			}
		}

	if (minscale == INT_MAX) {
		pr_warn("Can not find valid baud rate,speed_hz is %d,clkrate is %ld, we use the max prescaler value.\n",
			speed_hz, clkrate);
		*pbr = ARRAY_SIZE(pbr_tbl) - 1;
		*br =  ARRAY_SIZE(brs) - 1;
	}
}

static void ns_delay_scale(char *psc, char *sc, int delay_ns,
		unsigned long clkrate)
{
	int pscale_tbl[4] = {1, 3, 5, 7};
	int scale_needed, scale, minscale = INT_MAX;
	int i, j;
	u32 remainder;

	scale_needed = div_u64_rem((u64)delay_ns * clkrate, NSEC_PER_SEC,
			&remainder);
	if (remainder)
		scale_needed++;

	for (i = 0; i < ARRAY_SIZE(pscale_tbl); i++)
		for (j = 0; j <= SPI_CTAR_SCALE_BITS; j++) {
			scale = pscale_tbl[i] * (2 << j);
			if (scale >= scale_needed) {
				if (scale < minscale) {
					minscale = scale;
					*psc = i;
					*sc = j;
				}
				break;
			}
		}

	if (minscale == INT_MAX) {
		pr_warn("Cannot find correct scale values for %dns delay at clkrate %ld, using max prescaler value",
			delay_ns, clkrate);
		*psc = ARRAY_SIZE(pscale_tbl) - 1;
		*sc = SPI_CTAR_SCALE_BITS;
	}
}

static int dspi_load_dma(struct fsl_dspi *dspi, u8 *buf, size_t len, enum dma_data_direction dir, dma_async_tx_callback cb, bool cyclic)
{
	struct dma_chan *chan = (dir == DMA_TO_DEVICE) ? dspi->dma_chan_tx : dspi->dma_chan_rx;
	struct dma_async_tx_descriptor *txdesc;
	dma_addr_t buf_dma = dma_map_single(chan->device->dev, buf, len, dir);

	if (dma_mapping_error(chan->device->dev, buf_dma)) {
		dev_err(&dspi->pdev->dev, "DMA mapping failed\n");
		return -1;
	}

	if (dir == DMA_TO_DEVICE) {
		dma_sync_single_for_device(&dspi->pdev->dev, buf_dma, len, dir);
		dspi->dma_tx_buf = buf;
		dspi->dma_tx_len = len;
		dspi->dma_tx_buf_phys = buf_dma;
	} else {
		dspi->dma_rx_buf = buf;
		dspi->dma_rx_received_bytes = 0;
		dspi->dma_rx_len = len;
		dspi->dma_rx_buf_phys = buf_dma;
	}

	if (cyclic)
		txdesc = dmaengine_prep_dma_cyclic(chan, buf_dma,
						   len, DSPI_SLAVE_SEGMENT_SIZE,
						   (dir == DMA_FROM_DEVICE) ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV,
						   DMA_PREP_INTERRUPT);
	else
		txdesc = dmaengine_prep_slave_single(chan, buf_dma, len,
						     (dir == DMA_FROM_DEVICE) ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV,
						     DMA_PREP_INTERRUPT);

	if (!txdesc) {
		dev_err(&dspi->pdev->dev, "Not able to get DMA descriptor\n");
		goto fail_prep;
	}

	txdesc->callback = cb;
	txdesc->callback_param = dspi;

	if (dma_submit_error(dmaengine_submit(txdesc))) {
		dev_err(&dspi->pdev->dev, "DMA submit failed\n");
		goto fail_prep;
	}

	dma_async_issue_pending(chan);

	return 0;

fail_prep:
	dma_unmap_single(chan->device->dev, buf_dma, len, dir);

	return -1;
}

static void dspi_stop(struct fsl_dspi *dspi)
{
	u32 mcr;

	regmap_read(dspi->regmap, SPI_MCR, &mcr);
	mcr |= SPI_MCR_HALT;
	regmap_write(dspi->regmap, SPI_MCR, mcr);

	mcr |= SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
	regmap_write(dspi->regmap, SPI_MCR, mcr);
}

static void dspi_restart(struct fsl_dspi *dspi)
{
	u32 mcr;

	regmap_read(dspi->regmap, SPI_MCR, &mcr);
	mcr &= ~SPI_MCR_HALT;
	regmap_write(dspi->regmap, SPI_MCR, mcr);
}

static int dspi_set_default_rx_dma(struct fsl_dspi *dspi)
{
	int err;

	memset(dspi->dma_def_rx_buf, 0x00, DSPI_SLAVE_BUF_SIZE);

	err = dspi_load_dma(dspi, dspi->dma_def_rx_buf,
			    DSPI_SLAVE_BUF_SIZE,
			    DMA_FROM_DEVICE,
			    NULL,
			    true);
	if (err)
		dev_err(&dspi->pdev->dev, "Can't load default rx DMA\n");

	return err;
}

static int dspi_set_default_tx_dma(struct fsl_dspi *dspi)
{
	int err;

	memset(dspi->dma_def_tx_buf, 0xFF, DSPI_SLAVE_BUF_SIZE);

	err = dspi_load_dma(dspi, dspi->dma_def_tx_buf,
			    DSPI_SLAVE_BUF_SIZE,
			    DMA_TO_DEVICE,
			    NULL,
			    true);
	if (err)
		dev_err(&dspi->pdev->dev, "Can't load default tx DMA\n");

	return err;
}

static int dspi_read_update_recv_bytes(struct fsl_dspi *dspi)
{
	u32 spi_tcr, spi_tcnt, tcnt_diff;
	int tx_word = is_double_byte_mode(dspi);

	regmap_read(dspi->regmap, SPI_TCR, &spi_tcr);
	spi_tcnt = SPI_TCR_GET_TCNT(spi_tcr);
	/*
	 * The width of SPI Transfer Counter in SPI_TCR is 16bits,
	 * so the max couner is 65535. When the counter reach 65535,
	 * it will wrap around, counter reset to zero.
	 * spi_tcnt my be less than dspi->spi_tcnt, it means the
	 * counter already wrapped around.
	 * SPI Transfer Counter is a counter of transmitted frames.
	 * The size of frame maybe two bytes.
	 */
	tcnt_diff = ((spi_tcnt + SPI_TCR_TCNT_MAX) - dspi->spi_tcnt)
		% SPI_TCR_TCNT_MAX;
	tcnt_diff *= (tx_word + 1);
	if (dspi->dataflags & TRAN_STATE_WORD_ODD_NUM)
		tcnt_diff--;

	dspi->spi_tcnt = spi_tcnt;

	return tcnt_diff;
}

static void dspi_set_master_mode(struct fsl_dspi *dspi)
{
	u32 mcr;

	dev_info(&dspi->pdev->dev, "Switch to master mode\n");

	dmaengine_terminate_all(dspi->dma_chan_rx);
	dmaengine_terminate_all(dspi->dma_chan_tx);
	dma_unmap_single(dspi->dma_chan_rx->device->dev, dspi->dma_rx_buf_phys, dspi->dma_rx_len, DMA_TO_DEVICE);
	dma_unmap_single(dspi->dma_chan_tx->device->dev, dspi->dma_tx_buf_phys, dspi->dma_tx_len, DMA_TO_DEVICE);

	regmap_read(dspi->regmap, SPI_MCR, &mcr);
	mcr = SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
	regmap_write(dspi->regmap, SPI_MCR, mcr);

	mcr |= (SPI_MCR_MASTER | SPI_MCR_HALT);
	regmap_write(dspi->regmap, SPI_MCR, mcr);
}

static void dspi_disable_slave_mode(struct fsl_dspi *dspi);

static void dspi_set_slave_mode(struct fsl_dspi *dspi, bool enable_dma)
{
	u32 ctar = SPI_CTAR_FMSZ(8 - 1)
		| SPI_CTAR_CPOL(1)
		| SPI_CTAR_CPHA(1);
	u32 mcr, rser, sr;

	regmap_read(dspi->regmap, SPI_SR, &sr);

	dev_info(&dspi->pdev->dev, "Switch to slave mode\n");

	regmap_read(dspi->regmap, SPI_MCR, &mcr);
	mcr = SPI_MCR_HALT;
	regmap_write(dspi->regmap, SPI_MCR, mcr);

	mcr |= SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
	regmap_write(dspi->regmap, SPI_MCR, mcr);

	mcr &= ~SPI_MCR_MASTER;
	mcr |= (SPI_MCR_PCSIS | SPI_MCR_PCSSE);
	regmap_write(dspi->regmap, SPI_MCR, mcr);

	regmap_write(dspi->regmap, SPI_CTAR0_SLAVE, ctar);

	rser = SPI_RSER_EOQFE | SPI_RSER_TCFQE;

	if (enable_dma)
		rser |= SPI_RSER_TFFFRE | SPI_RSER_TFFF_DMA | SPI_RSER_RFDFRE | SPI_RSER_RFDF_DMA;

	rser |= SPI_RSER_TCFQE;
	regmap_write(dspi->regmap, SPI_RSER, rser);

	mcr &= ~SPI_MCR_HALT;
	regmap_write(dspi->regmap, SPI_MCR, mcr);

	if (enable_dma) {
		if (dspi_set_default_rx_dma(dspi) || dspi_set_default_tx_dma(dspi)) {
			dev_err(&dspi->pdev->dev, "Can't start DMA, disabling slave mode\n");
			dspi_disable_slave_mode(dspi);
		}
	}
}

static u32 dspi_data_to_pushr(struct fsl_dspi *dspi, int tx_word)
{
	u16 d16;

	if (!(dspi->dataflags & TRAN_STATE_TX_VOID))
		d16 = tx_word ? *(u16 *)dspi->tx : *(u8 *)dspi->tx;
	else
		d16 = dspi->void_write_data;

	dspi->tx += tx_word + 1;
	dspi->len -= tx_word + 1;

	return	SPI_PUSHR_TXDATA(d16) |
		SPI_PUSHR_PCS(dspi->cs) |
		SPI_PUSHR_CTAS(dspi->cs) |
		SPI_PUSHR_CONT;
}

static void dspi_data_from_popr(struct fsl_dspi *dspi, int rx_word)
{
	u16 d;
	unsigned int val;

	regmap_read(dspi->regmap, SPI_POPR, &val);
	d = SPI_POPR_RXDATA(val);

	if (!(dspi->dataflags & TRAN_STATE_RX_VOID))
		rx_word ? (*(u16 *)dspi->rx = d) : (*(u8 *)dspi->rx = d);

	dspi->rx += rx_word + 1;
}

static int dspi_eoq_write(struct fsl_dspi *dspi)
{
	int tx_count = 0;
	int tx_word;
	u32 dspi_pushr = 0;

	tx_word = is_double_byte_mode(dspi);

	while (dspi->len && (tx_count < DSPI_FIFO_SIZE)) {
		/* If we are in word mode, only have a single byte to transfer
		 * switch to byte mode temporarily.  Will switch back at the
		 * end of the transfer.
		 */
		if (tx_word && (dspi->len == 1)) {
			dspi->dataflags |= TRAN_STATE_WORD_ODD_NUM;
			regmap_update_bits(dspi->regmap, SPI_CTAR(dspi->cs),
					SPI_FRAME_BITS_MASK, SPI_FRAME_BITS(8));
			tx_word = 0;
		}

		dspi_pushr = dspi_data_to_pushr(dspi, tx_word);

		if (dspi->len == 0 || tx_count == DSPI_FIFO_SIZE - 1) {
			/* last transfer in the transfer */
			dspi_pushr |= SPI_PUSHR_EOQ;
			if ((dspi->cs_change) && (!dspi->len))
				dspi_pushr &= ~SPI_PUSHR_CONT;
		} else if (tx_word && (dspi->len == 1))
			dspi_pushr |= SPI_PUSHR_EOQ;

		regmap_write(dspi->regmap, SPI_PUSHR, dspi_pushr);

		tx_count++;
	}

	return tx_count * (tx_word + 1);
}

static int dspi_eoq_read(struct fsl_dspi *dspi)
{
	int rx_count = 0;
	int rx_word = is_double_byte_mode(dspi);

	while ((dspi->rx < dspi->rx_end)
			&& (rx_count < DSPI_FIFO_SIZE)) {
		if (rx_word && (dspi->rx_end - dspi->rx) == 1)
			rx_word = 0;

		dspi_data_from_popr(dspi, rx_word);
		rx_count++;
	}

	return rx_count;
}

static int dspi_tcfq_write(struct fsl_dspi *dspi)
{
	int tx_word;
	u32 dspi_pushr = 0;

	tx_word = is_double_byte_mode(dspi);

	if (tx_word && (dspi->len == 1)) {
		dspi->dataflags |= TRAN_STATE_WORD_ODD_NUM;
		regmap_update_bits(dspi->regmap, SPI_CTAR(dspi->cs),
				SPI_FRAME_BITS_MASK, SPI_FRAME_BITS(8));
		tx_word = 0;
	}

	dspi_pushr = dspi_data_to_pushr(dspi, tx_word);

	if ((dspi->cs_change) && (!dspi->len))
		dspi_pushr &= ~SPI_PUSHR_CONT;

	regmap_write(dspi->regmap, SPI_PUSHR, dspi_pushr);

	return tx_word + 1;
}

static void dspi_tcfq_read(struct fsl_dspi *dspi)
{
	int rx_word = is_double_byte_mode(dspi);

	if (rx_word && (dspi->rx_end - dspi->rx) == 1)
		rx_word = 0;

	dspi_data_from_popr(dspi, rx_word);
}

static int dspi_transfer_one_message(struct spi_master *master,
		struct spi_message *message)
{
	struct fsl_dspi *dspi = spi_master_get_devdata(master);
	struct spi_device *spi = message->spi;
	struct spi_transfer *transfer;
	int status = 0;
	enum dspi_trans_mode trans_mode;
	u32 spi_tcr;

	if (dspi->enable_slave)
		dspi_set_master_mode(dspi);

	regmap_read(dspi->regmap, SPI_TCR, &spi_tcr);
	dspi->spi_tcnt = SPI_TCR_GET_TCNT(spi_tcr);

	message->actual_length = 0;

	list_for_each_entry(transfer, &message->transfers, transfer_list) {
		dspi->cur_transfer = transfer;
		dspi->cur_msg = message;
		dspi->cur_chip = spi_get_ctldata(spi);
		dspi->cs = spi->chip_select;
		dspi->cs_change = 0;
		if (dspi->cur_transfer->transfer_list.next
				== &dspi->cur_msg->transfers)
			dspi->cs_change = 1;
		dspi->void_write_data = dspi->cur_chip->void_write_data;

		dspi->dataflags = 0;
		dspi->tx = (void *)transfer->tx_buf;
		dspi->tx_end = dspi->tx + transfer->len;
		dspi->rx = transfer->rx_buf;
		dspi->rx_end = dspi->rx + transfer->len;
		dspi->len = transfer->len;

		if (!dspi->rx)
			dspi->dataflags |= TRAN_STATE_RX_VOID;

		if (!dspi->tx)
			dspi->dataflags |= TRAN_STATE_TX_VOID;

		regmap_write(dspi->regmap, SPI_MCR, dspi->cur_chip->mcr_val);
		regmap_update_bits(dspi->regmap, SPI_MCR,
				SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF,
				SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF);
		regmap_write(dspi->regmap, SPI_CTAR(dspi->cs),
				dspi->cur_chip->ctar_val);
		if (transfer->speed_hz)
			regmap_write(dspi->regmap, SPI_CTAR(dspi->cs),
					dspi->cur_chip->ctar_val);

		trans_mode = dspi->devtype_data->trans_mode;
		switch (trans_mode) {
		case DSPI_EOQ_MODE:
			regmap_write(dspi->regmap, SPI_RSER, SPI_RSER_EOQFE);
			dspi_eoq_write(dspi);
			break;
		case DSPI_TCFQ_MODE:
			regmap_write(dspi->regmap, SPI_RSER, SPI_RSER_TCFQE);
			dspi_tcfq_write(dspi);
			break;
		default:
			dev_err(&dspi->pdev->dev, "unsupported trans_mode %u\n",
				trans_mode);
			status = -EINVAL;
			goto out;
		}

		if (wait_event_interruptible(dspi->waitq, dspi->waitflags))
			dev_err(&dspi->pdev->dev, "wait transfer complete fail!\n");
		dspi->waitflags = 0;

		if (transfer->delay_usecs)
			udelay(transfer->delay_usecs);
	}

out:
	message->status = status;
	spi_finalize_current_message(master);

	if (dspi->enable_slave)
		dspi_set_slave_mode(dspi, true);

	return status;
}

static int dspi_setup(struct spi_device *spi)
{
	struct chip_data *chip;
	struct fsl_dspi *dspi = spi_master_get_devdata(spi->master);
	u32 cs_sck_delay = 0, sck_cs_delay = 0;
	unsigned char br = 0, pbr = 0, pcssck = 0, cssck = 0;
	unsigned char pasc = 0, asc = 0, fmsz = 0;
	unsigned long clkrate;

	if ((spi->bits_per_word >= 4) && (spi->bits_per_word <= 16)) {
		fmsz = spi->bits_per_word - 1;
	} else {
		pr_err("Invalid wordsize\n");
		return -ENODEV;
	}

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (chip == NULL) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
	}

	of_property_read_u32(spi->dev.of_node, "fsl,spi-cs-sck-delay",
			&cs_sck_delay);

	of_property_read_u32(spi->dev.of_node, "fsl,spi-sck-cs-delay",
			&sck_cs_delay);

	chip->mcr_val = SPI_MCR_MASTER | SPI_MCR_PCSIS |
		SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;

	chip->void_write_data = 0;

	clkrate = clk_get_rate(dspi->clk);
	hz_to_spi_baud(&pbr, &br, spi->max_speed_hz, clkrate);

	/* Set PCS to SCK delay scale values */
	ns_delay_scale(&pcssck, &cssck, cs_sck_delay, clkrate);

	/* Set After SCK delay scale values */
	ns_delay_scale(&pasc, &asc, sck_cs_delay, clkrate);

	chip->ctar_val =  SPI_CTAR_FMSZ(fmsz)
		| SPI_CTAR_CPOL(spi->mode & SPI_CPOL ? 1 : 0)
		| SPI_CTAR_CPHA(spi->mode & SPI_CPHA ? 1 : 0)
		| SPI_CTAR_LSBFE(spi->mode & SPI_LSB_FIRST ? 1 : 0)
		| SPI_CTAR_PCSSCK(pcssck)
		| SPI_CTAR_CSSCK(cssck)
		| SPI_CTAR_PASC(pasc)
		| SPI_CTAR_ASC(asc)
		| SPI_CTAR_PBR(pbr)
		| SPI_CTAR_BR(br);

	spi_set_ctldata(spi, chip);

	return 0;
}

static void dspi_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata((struct spi_device *)spi);

	dev_dbg(&spi->dev, "spi_device %u.%u cleanup\n",
			spi->master->bus_num, spi->chip_select);

	kfree(chip);
}

static void dspi_resp_dma_callback(void *param)
{
	struct fsl_dspi *dspi = (struct fsl_dspi *)param;

	if (dspi->resp_cb)
		dspi->resp_cb(dspi->slave_spi, dspi->slave_resp, 0);

	dmaengine_terminate_all(dspi->dma_chan_tx);
	dspi_stop(dspi);
	dma_unmap_single(dspi->dma_chan_tx->device->dev, dspi->dma_tx_buf_phys, dspi->dma_tx_len, DMA_TO_DEVICE);
	dspi_restart(dspi);
	dspi_set_default_tx_dma(dspi);

	dspi->is_response_prepared = false;
	complete(&dspi->slave_complete);
}

static bool dspi_dma_rx_process(struct fsl_dspi *dspi, const unsigned char *buf, size_t buf_len)
{
	u8 *resp;
	size_t resp_size;

	if (!buf_len)
		return false;

	dma_sync_single_for_cpu(&dspi->pdev->dev, virt_to_phys(buf), buf_len, DMA_FROM_DEVICE);

	resp_size = dspi->cmd_cb(dspi->slave_spi, buf, buf_len, &resp);

	if (resp_size) {
		size_t total_size = resp_size + DSPI_FIFO_SIZE;
		if (resp_size % DSPI_FIFO_SIZE) {
			dev_err(&dspi->pdev->dev, "Got non-aligned buffer size, cancel current response\n");
			if (dspi->resp_cb)
				dspi->resp_cb(dspi->slave_spi, dspi->slave_resp, -EFAULT);
		}
		dmaengine_terminate_all(dspi->dma_chan_tx);
		dspi_stop(dspi);
		dma_unmap_single(dspi->dma_chan_tx->device->dev, dspi->dma_tx_buf_phys, dspi->dma_tx_len, DMA_TO_DEVICE);
		memcpy(dspi->dma_def_tx_buf, resp, resp_size);
		memset(dspi->dma_def_tx_buf + resp_size, 0xFF, DSPI_FIFO_SIZE);
		dspi->slave_resp = resp;


		dspi_restart(dspi);

		/*
		 * Start DMA with data size + FIFO size, so the callback will be invoked
		 * after sending response, instead of moment of loading the last block of
		 * the response to the FIFO
		 */
		if (dspi_load_dma(dspi, dspi->dma_def_tx_buf,
				  total_size,
				  DMA_TO_DEVICE,
				  dspi_resp_dma_callback,
				  false)) {
			dev_err(&dspi->pdev->dev, "Can't load tx DMA\n");
			return false;
		}

		dspi->is_response_prepared = true;
	}

	return dspi->is_response_prepared;
}

static void dspi_work(struct work_struct *work)
{
	struct fsl_dspi *dspi = container_of(work, struct fsl_dspi, slave_work);
	const unsigned char *buf = (unsigned char*)dspi->dma_rx_buf;
	size_t buf_len;

	while ((buf_len = dspi_read_update_recv_bytes(dspi))) {
		bool wrap_around = (dspi->dma_rx_received_bytes + buf_len) >= dspi->dma_rx_len;

		dma_sync_single_for_cpu(&dspi->pdev->dev, dspi->dma_rx_buf_phys, dspi->dma_rx_len, DMA_FROM_DEVICE);

		if (!buf_len)
			return;

		if (dspi->is_response_prepared) {
			if (!wait_for_completion_timeout(&dspi->slave_complete,
							 msecs_to_jiffies(10))) {
			}
		}

		if (dspi->cmd_cb) {
			if (wrap_around) {
				size_t first_len = dspi->dma_rx_len - dspi->dma_rx_received_bytes;
				size_t second_len = buf_len - first_len;
				const unsigned char *buf_first = buf + dspi->dma_rx_received_bytes;

				if (!dspi_dma_rx_process(dspi, buf_first, first_len))
					dspi_dma_rx_process(dspi, buf, second_len);
				dspi->dma_rx_received_bytes = second_len;
			} else {
				buf += dspi->dma_rx_received_bytes;
				dspi_dma_rx_process(dspi, buf, buf_len);
				dspi->dma_rx_received_bytes += buf_len;
			}
		} else {
			dspi->dma_rx_received_bytes = (dspi->dma_rx_received_bytes + buf_len) % dspi->dma_rx_len;
		}
		break;
	}
}

static irqreturn_t dspi_interrupt(int irq, void *dev_id)
{
	struct fsl_dspi *dspi = (struct fsl_dspi *)dev_id;
	u32 spi_sr, spi_mcr;
	int is_master;

	regmap_read(dspi->regmap, SPI_SR, &spi_sr);
	regmap_write(dspi->regmap, SPI_SR, spi_sr);
	regmap_read(dspi->regmap, SPI_MCR, &spi_mcr);

	is_master = (spi_mcr & SPI_MCR_MASTER);

	if (!is_master && (spi_sr & SPI_SR_TCFQF)) {
		queue_work(dspi->slave_wq, &dspi->slave_work);
	} else if (!is_master) {
		dev_err(&dspi->pdev->dev, "Interrupts are not supposed in Slave Mode. MCR 0x%x: %s%s%s%s%s%s\n",
			spi_mcr,
			(spi_sr & (1 << 31)) ? " TCF" : "",
			(spi_sr & (1 << 28)) ? " EOQ" : "",
			(spi_sr & (1 << 27)) ? " TFU" : "",
			(spi_sr & (1 << 25)) ? " TFF" : "",
			(spi_sr & (1 << 19)) ? " RFO" : "",
			(spi_sr & (1 << 17)) ? " RFD" : "");
	}

	if (is_master && (spi_sr & (SPI_SR_EOQF | SPI_SR_TCFQF))) {
		struct spi_message *msg = dspi->cur_msg;
		enum dspi_trans_mode trans_mode;

		msg->actual_length += dspi_read_update_recv_bytes(dspi);

		trans_mode = dspi->devtype_data->trans_mode;
		switch (trans_mode) {
		case DSPI_EOQ_MODE:
			dspi_eoq_read(dspi);
			break;
		case DSPI_TCFQ_MODE:
			dspi_tcfq_read(dspi);
			break;
		default:
			dev_err(&dspi->pdev->dev, "unsupported trans_mode %u\n",
				trans_mode);
				return IRQ_HANDLED;
		}

		if (!dspi->len) {
			if (dspi->dataflags & TRAN_STATE_WORD_ODD_NUM) {
				regmap_update_bits(dspi->regmap,
						   SPI_CTAR(dspi->cs),
						   SPI_FRAME_BITS_MASK,
						   SPI_FRAME_BITS(16));
				dspi->dataflags &= ~TRAN_STATE_WORD_ODD_NUM;
			}

			dspi->waitflags = 1;
			wake_up_interruptible(&dspi->waitq);
		} else {
			switch (trans_mode) {
			case DSPI_EOQ_MODE:
				dspi_eoq_write(dspi);
				break;
			case DSPI_TCFQ_MODE:
				dspi_tcfq_write(dspi);
				break;
			default:
				dev_err(&dspi->pdev->dev,
					"unsupported trans_mode %u\n",
					trans_mode);
			}
		}
	}

	return IRQ_HANDLED;
}

static const struct of_device_id fsl_dspi_dt_ids[] = {
	{ .compatible = "fsl,vf610-dspi", .data = (void *)&vf610_data, },
	{ .compatible = "fsl,ls1021a-v1.0-dspi",
		.data = (void *)&ls1021a_v1_data, },
	{ .compatible = "fsl,ls2085a-dspi", .data = (void *)&ls2085a_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_dspi_dt_ids);

#ifdef CONFIG_PM_SLEEP
static int dspi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct fsl_dspi *dspi = spi_master_get_devdata(master);

	if (device_may_wakeup(&dspi->pdev->dev)) {
		dspi_set_slave_mode(dspi, false);
		enable_irq_wake(dspi->irq);
	} else {
		spi_master_suspend(master);
		clk_disable_unprepare(dspi->clk);

		pinctrl_pm_select_sleep_state(dev);
	}

	return 0;
}

static int dspi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct fsl_dspi *dspi = spi_master_get_devdata(master);

	if (device_may_wakeup(&dspi->pdev->dev)) {
		dspi_set_master_mode(dspi);
		disable_irq_wake(dspi->irq);
	} else {
		pinctrl_pm_select_default_state(dev);

		clk_prepare_enable(dspi->clk);
		spi_master_resume(master);
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(dspi_pm, dspi_suspend, dspi_resume);

static const struct regmap_config dspi_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x88,
};

static int dspi_dma_init(struct fsl_dspi *dspi)
{
	struct dma_slave_config txconf, rxconf;
	int ret;

	dspi->dma_chan_rx = dma_request_slave_channel(&dspi->pdev->dev, "rx");
	if (!dspi->dma_chan_rx) {
		dev_warn(&dspi->pdev->dev, "Failed to requets RX DMA channel\n");
		goto fail_rx;
	}

	dspi->dma_chan_tx = dma_request_slave_channel(&dspi->pdev->dev, "tx");
	if (!dspi->dma_chan_tx) {
		dev_warn(&dspi->pdev->dev, "Failed to requets TX DMA channel\n");
		goto fail_tx;
	}

	txconf.direction = DMA_MEM_TO_DEV;
	txconf.dst_addr = dspi->phy_addr + SPI_PUSHR;
	txconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	txconf.dst_maxburst = DSPI_FIFO_SIZE / 2; /* Half of FIFO size */

	ret = dmaengine_slave_config(dspi->dma_chan_tx, &txconf);
	if (ret < 0) {
		dev_err(&dspi->pdev->dev, "Can't configure tx channel: %d\n", ret);
		goto fail_conf;
	}

	rxconf.direction = DMA_DEV_TO_MEM;
	rxconf.src_addr = dspi->phy_addr + SPI_POPR;
	rxconf.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	rxconf.src_maxburst = 1;

	ret = dmaengine_slave_config(dspi->dma_chan_rx, &rxconf);
	if (ret < 0) {
		dev_err(&dspi->pdev->dev, "Can't configure rx channel: %d\n", ret);
		goto fail_conf;
	}

	dspi->dma_def_rx_buf = devm_kmalloc(&dspi->pdev->dev, DSPI_SLAVE_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (!dspi->dma_def_rx_buf) {
		dev_err(&dspi->pdev->dev, "Can't allocate buffer for rx DMA\n");
		goto fail_conf;
	}

	dspi->dma_def_tx_buf = devm_kmalloc(&dspi->pdev->dev, DSPI_SLAVE_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (!dspi->dma_def_tx_buf) {
		dev_err(&dspi->pdev->dev, "Can't allocate buffer for tx DMA\n");
		goto fail_tx_alloc;
	}

	return 0;

fail_tx_alloc:
	devm_kfree(&dspi->pdev->dev, dspi->dma_def_rx_buf);
fail_conf:
	dma_release_channel(dspi->dma_chan_rx);
fail_tx:
	dma_release_channel(dspi->dma_chan_tx);
fail_rx:
	dev_info(&dspi->pdev->dev, "Can't use DMA\n");
	return -1;
}

static void dspi_disable_slave_mode(struct fsl_dspi *dspi)
{
	dmaengine_terminate_all(dspi->dma_chan_tx);
	dmaengine_terminate_all(dspi->dma_chan_rx);
	devm_kfree(&dspi->pdev->dev, dspi->dma_def_rx_buf);
	devm_kfree(&dspi->pdev->dev, dspi->dma_def_tx_buf);
	dspi_set_master_mode(dspi);
	device_set_wakeup_capable(&dspi->pdev->dev, 0);
	dspi->enable_slave = false;
	dspi->master->register_slave = NULL;
	dspi->master->unregister_slave = NULL;
	cancel_work_sync(&dspi->slave_work);
	destroy_workqueue(dspi->slave_wq);
}

void dspi_register_slave(struct spi_device *spi, spi_cmd_cb_t cmd_cb, spi_resp_cb_t resp_cb)
{
	struct fsl_dspi *dspi = spi_master_get_devdata(spi->master);

	dspi->slave_spi = spi;
	dspi->cmd_cb = cmd_cb;
	dspi->resp_cb = resp_cb;

	dspi->slave_wq = alloc_workqueue("dspi%d_slave", 0, 1, dspi->master->bus_num);
	init_completion(&dspi->slave_complete);
	BUG_ON(!dspi->slave_wq);
	INIT_WORK(&dspi->slave_work, dspi_work);
	dspi->enable_slave = true;

	device_set_wakeup_capable(&dspi->pdev->dev, 1);
	device_wakeup_disable(&dspi->pdev->dev);

	dspi_set_slave_mode(dspi, true);
}

void dspi_unregister_slave(struct spi_device *spi)
{
	struct fsl_dspi *dspi = spi_master_get_devdata(spi->master);

	dspi_disable_slave_mode(dspi);

	dspi->slave_spi = NULL;
	dspi->cmd_cb = NULL;
	dspi->resp_cb = NULL;
}

static int dspi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct spi_master *master;
	struct fsl_dspi *dspi;
	struct resource *res;
	void __iomem *base;
	int ret = 0, cs_num, bus_num;
	const struct of_device_id *of_id =
			of_match_device(fsl_dspi_dt_ids, &pdev->dev);

	master = spi_alloc_master(&pdev->dev, sizeof(struct fsl_dspi));
	if (!master)
		return -ENOMEM;

	dspi = spi_master_get_devdata(master);
	dspi->pdev = pdev;
	dspi->master = master;

	master->transfer = NULL;
	master->setup = dspi_setup;
	master->transfer_one_message = dspi_transfer_one_message;
	master->dev.of_node = pdev->dev.of_node;

	master->cleanup = dspi_cleanup;
	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->bits_per_word_mask = SPI_BPW_MASK(4) | SPI_BPW_MASK(8) | SPI_BPW_MASK(9) |
					SPI_BPW_MASK(16);

	ret = of_property_read_u32(np, "spi-num-chipselects", &cs_num);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't get spi-num-chipselects\n");
		goto out_master_put;
	}
	master->num_chipselect = cs_num;

	ret = of_property_read_u32(np, "bus-num", &bus_num);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't get bus-num\n");
		goto out_master_put;
	}
	master->bus_num = bus_num;

	dspi->devtype_data = (struct fsl_dspi_devtype_data *)of_id->data;
	if (!dspi->devtype_data) {
		dev_err(&pdev->dev, "can't get devtype_data\n");
		ret = -EFAULT;
		goto out_master_put;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		goto out_master_put;
	}
	dspi->phy_addr = res->start;

	dspi->is_response_prepared = false;

	dspi->regmap = devm_regmap_init_mmio_clk(&pdev->dev, NULL, base,
						&dspi_regmap_config);
	if (IS_ERR(dspi->regmap)) {
		dev_err(&pdev->dev, "failed to init regmap: %ld\n",
				PTR_ERR(dspi->regmap));
		return PTR_ERR(dspi->regmap);
	}

	dspi->irq = platform_get_irq(pdev, 0);
	if (dspi->irq < 0) {
		dev_err(&pdev->dev, "can't get platform irq\n");
		ret = dspi->irq;
		goto out_master_put;
	}

	ret = devm_request_irq(&pdev->dev, dspi->irq, dspi_interrupt, 0,
			pdev->name, dspi);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to attach DSPI interrupt\n");
		goto out_master_put;
	}

	dspi->clk = devm_clk_get(&pdev->dev, "dspi");
	if (IS_ERR(dspi->clk)) {
		ret = PTR_ERR(dspi->clk);
		dev_err(&pdev->dev, "unable to get clock\n");
		goto out_master_put;
	}
	clk_prepare_enable(dspi->clk);

	init_waitqueue_head(&dspi->waitq);
	platform_set_drvdata(pdev, master);

	if (!dspi_dma_init(dspi)) {
		master->register_slave = dspi_register_slave;
		master->unregister_slave = dspi_unregister_slave;
	}

	ret = spi_register_master(master);
	if (ret != 0) {
		dev_err(&pdev->dev, "Problem registering DSPI master\n");
		goto out_clk_put;
	}

	return ret;

out_clk_put:
	clk_disable_unprepare(dspi->clk);
out_master_put:
	spi_master_put(master);

	return ret;
}

static int dspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct fsl_dspi *dspi = spi_master_get_devdata(master);

	dspi_disable_slave_mode(dspi);

	/* Disconnect from the SPI framework */
	clk_disable_unprepare(dspi->clk);
	spi_unregister_master(dspi->master);
	spi_master_put(dspi->master);

	return 0;
}

static struct platform_driver fsl_dspi_driver = {
	.driver.name    = DRIVER_NAME,
	.driver.of_match_table = fsl_dspi_dt_ids,
	.driver.owner   = THIS_MODULE,
	.driver.pm	= &dspi_pm,
	.probe          = dspi_probe,
	.remove		= dspi_remove,
};
module_platform_driver(fsl_dspi_driver);

MODULE_DESCRIPTION("Freescale DSPI Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
