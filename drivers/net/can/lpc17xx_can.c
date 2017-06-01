/*
 * LPC17XX CAN controller driver
 *
 * Copyright (c) 2017 Emcraft Systems
 * Dmitry Konsyhev <probables@emraft.com>
 *
 * LICENCE:
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define DRV_NAME			"lcp17xx_can"

/* 8 for RX fifo and 2 error handling */
#define LPC17XX_CAN_NAPI_WEIGHT		(8 + 2)

#define LPC17XX_CAN_CF_COUNT		16
#define LPC17XX_CAN_CF_NEXT(val)	((val + 1) % LPC17XX_CAN_CF_COUNT)

struct lpc17xx_can_priv {
	struct can_priv can;
	struct napi_struct napi;

	void __iomem *base;

	u32 saved_icr;

	struct clk *clk;
	struct lpc17xx_can_platform_data *pdata;
	const struct lpc17xx_can_devtype_data *devtype_data;
	struct can_frame cf[LPC17XX_CAN_CF_COUNT];
	u32 cf_start;
	u32 cf_end;
	u32 can_state_changed;
};

struct lpc17xx_can_regs {
	u32 mod;
	u32 cmr;
	u32 gsr;
	u32 icr;
	u32 ier;
	u32 btr;
	u32 ewl;
	u32 sr;
	u32 rfs;
	u32 rid;
	u32 rda;
	u32 rdb;
	struct {
		u32 tfi;
		u32 tid;
		u32 tda;
		u32 tdb;
	} tx[3];
};

/* Mode Register bits */
#define CAN_MOD_RM		(1 << 0) /* Reset mode */
#define CAN_MOD_LOM		(1 << 1) /* Listen-only mode */
#define CAN_MOD_STM		(1 << 2) /* Self-test mode */
#define CAN_MOD_TPM		(1 << 3) /* Transmit priority mode */
#define CAN_MOD_SM		(1 << 4) /* Sleep mode */
#define CAN_MOD_RPM		(1 << 5) /* Receive polarity mode */
#define CAN_MOD_TM		(1 << 7) /* Test mode */

/* Command Register bits */
#define CAN_CMR_TR		(1 << 0) /* Transmission Request */
#define CAN_CMR_AT		(1 << 1) /* Abort Transmission */
#define CAN_CMR_RRB		(1 << 2) /* Release Receive Buffer */
#define CAN_CMR_CDO		(1 << 3) /* Clear Data Overrun */
#define CAN_CMR_SRR		(1 << 4) /* Self Reception Request */
#define CAN_CMR_STB1		(1 << 5) /* Select Tx Buffer 1 */
#define CAN_CMR_STB2		(1 << 6) /* Select Tx Buffer 2 */
#define CAN_CMR_STB3		(1 << 7) /* Select Tx Buffer 3 */

/* Global Status Register bits */
#define CAN_GSR_RBS		(1 << 0) /* Receive Buffer Status */
#define CAN_GSR_DOS		(1 << 1) /* Data Overrun Status */
#define CAN_GSR_TBS		(1 << 2) /* Transmit Buffer Status */
#define CAN_GSR_TCS		(1 << 3) /* Transmit Complete Status */
#define CAN_GSR_RS		(1 << 4) /* Receive Status */
#define CAN_GSR_TS		(1 << 5) /* Transmit Status */
#define CAN_GSR_ES		(1 << 6) /* Error Status */
#define CAN_GSR_BS		(1 << 7) /* Bus Status */
#define CAN_GSR_RXERR_MASK	0xff
#define CAN_GSR_RXERR_SHIFT	16
#define CAN_GSR_RXERR(val)	((val >> CAN_GSR_RXERR_SHIFT) & \
				 CAN_GSR_RXERR_MASK)
#define CAN_GSR_TXERR_MASK	0xff
#define CAN_GSR_TXERR_SHIFT	24
#define CAN_GSR_TXERR(val)	((val >> CAN_GSR_TXERR_SHIFT) & \
				 CAN_GSR_TXERR_MASK)

/* CAN Interrupt Enable Register bits */
#define CAN_IER_RIE		(1 << 0) /* Receiver Interrupt Enable */
#define CAN_IER_TIE1		(1 << 1) /* Transmit Interrupt Enable for Buffer1 */
#define CAN_IER_EIE		(1 << 2) /* Error Warning Interrupt Enable */
#define CAN_IER_DOIE		(1 << 3) /* Data Overrun Interrupt Enable */
#define CAN_IER_WUIE		(1 << 4) /* Wake-Up Interrupt Enable */
#define CAN_IER_EPIE		(1 << 5) /* Error Passive Interrupt Enable */
#define CAN_IER_ALIE		(1 << 6) /* Arbitration Lost Interrupt Enable */
#define CAN_IER_BEIE		(1 << 7) /* Bus Error Interrupt Enable */
#define CAN_IER_IDIE		(1 << 8) /* ID Ready Interrupt Enable */
#define CAN_IER_TIE2		(1 << 9) /* Transmit Interrupt Enable for Buffer2 */
#define CAN_IER_TIE3		(1 << 10) /* Transmit Interrupt Enable for Buffer3 */

/* CAN Interrupt and Capture Register bits */
#define CAN_ICR_RI		(1 << 0) /* Receive Interrupt */
#define CAN_ICR_TI1		(1 << 1) /* Transmit Interrupt 1 */
#define CAN_ICR_EI		(1 << 2) /* Error Warning Interrupt */
#define CAN_ICR_DOI		(1 << 3) /* Data Overrun Interrupt */
#define CAN_ICR_WUI		(1 << 4) /* Wake-Up Interrupt */
#define CAN_ICR_EPI		(1 << 5) /* Error Passive Interrupt */
#define CAN_ICR_ALI		(1 << 6) /* Arbitration Lost Interrupt */
#define CAN_ICR_BEI		(1 << 7) /* Bus Error Interrupt */
#define CAN_ICR_IDI		(1 << 8) /* ID Ready Interrupt */
#define CAN_ICR_TI2		(1 << 9) /* Transmit Interrupt 2 */
#define CAN_ICR_TI3		(1 << 10) /* Transmit Interrupt 3 */
#define CAN_ICR_ERRBIT_MASK	0x1f
#define CAN_ICR_ERRBIT_SHIFT	16
#define CAN_ICR_ERRBIT(val)	((val >> CAN_ICR_ERRBIT_SHIFT) & \
				 CAN_ICR_ERRBIT_MASK)
#define CAN_ICR_ERRRCV		(1 << 21) /* Error occurred during receiving */
#define CAN_ICR_ERRC_MASK	0x3
#define CAN_ICR_ERRC_SHIFT	22
#define CAN_ICR_ERRC(val)	((val >> CAN_ICR_ERRC_SHIFT) & \
				 CAN_ICR_ERRC_MASK)
#define CAN_ICR_ALCBIT_MASK	0xff
#define CAN_ICR_ALCBIT_SHIFT	24
#define CAN_ICR_ALCBIT(val)	((val >> CAN_ICR_ALCBIT_SHIFT) & \
				 CAN_ICR_ALCBIT_MASK)

enum errbit {
	ERRBIT_SOF = 3,
	ERRBIT_ID28_21 = 2,
	ERRBIT_ID20_18 = 6,
	ERRBIT_SRTR = 4,
	ERRBIT_IDE = 5,
	ERRBIT_ID17_13 = 7,
	ERRBIT_ID12_5 = 15,
	ERRBIT_ID4_0 = 14,
	ERRBIT_RTR = 12,
	ERRBIT_RBIT1 = 13,
	ERRBIT_RBIT0 = 9,
	ERRBIT_DLC = 11,
	ERRBIT_DATA = 10,
	ERRBIT_CRC_SEQ = 8,
	ERRBIT_CRC_DELIM = 24,
	ERRBIT_ACK_SLOT = 25,
	ERRBIT_ACK_DELIM = 27,
	ERRBIT_EOF = 26,
	ERRBIT_INTERMISSION = 18,
};

enum errc {
	ERRC_BIT = 0,
	ERRC_FORM = 1,
	ERRC_STUFF = 2,
	ERRC_OTHER = 3,
};

/* Bus Timing Register bits */
#define CAN_BTR_BRP_MASK	0x3ff
#define CAN_BTR_BRP_SHIFT	0
#define CAN_BTR_BRP		((val >> CAN_BTR_BRP_SHIFT) & CAN_BTR_BRP_MASK)
#define CAN_BTR_BRP_SET		((val & CAN_BTR_BRP_MASK) << CAN_BTR_BRP_SHIFT)
#define CAN_BTR_SJW_MASK	0x3
#define CAN_BTR_SJW_SHIFT	14
#define CAN_BTR_SJW		((val >> CAN_BTR_SJW_SHIFT) & CAN_BTR_SJW_MASK)
#define CAN_BTR_SJW_SET		((val & CAN_BTR_SJW_MASK) << CAN_BTR_SJW_SHIFT)
#define CAN_BTR_TSEG1_MASK	0xf
#define CAN_BTR_TSEG1_SHIFT	16
#define CAN_BTR_TSEG1		((val >> CAN_BTR_TSEG1_SHIFT) & \
				 CAN_BTR_TSEG1_MASK)
#define CAN_BTR_TSEG1_SET	((val & CAN_BTR_TSEG1_MASK) << \
				 CAN_BTR_TSEG1_SHIFT)
#define CAN_BTR_TSEG2_MASK	0x7
#define CAN_BTR_TSEG2_SHIFT	20
#define CAN_BTR_TSEG2		((val >> CAN_BTR_TSEG2_SHIFT) & \
				 CAN_BTR_TSEG2_MASK)
#define CAN_BTR_TSEG2_SET	((val & CAN_BTR_TSEG2_MASK) << \
				 CAN_BTR_TSEG2_SHIFT)
#define CAN_BTR_SAM		(1 << 23)

/* Status Register bits */
#define CAN_SR_RBS_1		(1 << 0)
#define CAN_SR_DOS_1		(1 << 1)
#define CAN_SR_TBS1_1		(1 << 2)
#define CAN_SR_TCS1_1		(1 << 3)
#define CAN_SR_RS_1		(1 << 4)
#define CAN_SR_TS1_1		(1 << 5)
#define CAN_SR_ES_1		(1 << 6)
#define CAN_SR_BS_1		(1 << 7)
#define CAN_SR_RBS_2		(1 << 8)
#define CAN_SR_DOS_2		(1 << 9)
#define CAN_SR_TBS2_2		(1 << 10)
#define CAN_SR_TCS2_2		(1 << 11)
#define CAN_SR_RS_2		(1 << 12)
#define CAN_SR_TS2_2		(1 << 13)
#define CAN_SR_ES_2		(1 << 14)
#define CAN_SR_BS_2		(1 << 15)
#define CAN_SR_RBS_3		(1 << 16)
#define CAN_SR_DOS_3		(1 << 17)
#define CAN_SR_TBS3_3		(1 << 18)
#define CAN_SR_TCS3_3		(1 << 19)
#define CAN_SR_RS_3		(1 << 20)
#define CAN_SR_TS3_3		(1 << 21)
#define CAN_SR_ES_3		(1 << 22)
#define CAN_SR_BS_3		(1 << 23)

/* Receive Frame Status Register bits */
#define CAN_RFS_IDINDEX_MASK	0x3ff
#define CAN_RFS_IDINDEX_SHIFT	0
#define CAN_RFS_IDINDEX(val)	((val >> CAN_RFS_IDINDEX_SHIFT) & \
				 CAN_RFS_IDINDEX_MASK)
#define CAN_RFS_IDINDEX_SET	((val & CAN_RFS_IDINDEX_MASK) << \
				 CAN_RFS_IDINDEX_SHIFT)
#define CAN_RFS_BP		(1 << 10)
#define CAN_RFS_DLC_MASK	0xf
#define CAN_RFS_DLC_SHIFT	16
#define CAN_RFS_DLC(val)	((val >> CAN_RFS_DLC_SHIFT) & CAN_RFS_DLC_MASK)
#define CAN_RFS_DLC_SET(val)	((val & CAN_RFS_DLC_MASK) << CAN_RFS_DLC_SHIFT)
#define CAN_RFS_RTR		(1 << 30)
#define CAN_RFS_FF		(1 << 31)

/* Transmit Frame Status Register bits */
#define CAN_TFI_PRIO_MASK	0xff
#define CAN_TFI_PRIO_SHIFT	0
#define CAN_TFI_PRIO(val)	((val >> CAN_TFI_PRIO_SHIFT) & \
				 CAN_TFI_PRIO_MASK)
#define CAN_TFI_PRIO_SET	((val & CAN_TFI_PRIO_MASK) << \
				 CAN_TFI_PRIO_SHIFT)
#define CAN_TFI_DLC_MASK	0xf
#define CAN_TFI_DLC_SHIFT	16
#define CAN_TFI_DLC(val)	((val >> CAN_TFI_DLC_SHIFT) & CAN_TFI_DLC_MASK)
#define CAN_TFI_DLC_SET(val)	((val & CAN_TFI_DLC_MASK) << CAN_TFI_DLC_SHIFT)
#define CAN_TFI_RTR		(1 << 30)
#define CAN_TFI_FF		(1 << 31)

struct lpc17xx_can_af_regs {
	u32 afmr;
	u32 sff_sa;
	u32 sff_grp_sa;
	u32 eff_sa;
	u32 eff_grp_sa;
	u32 endoftable;
	u32 luterrad;
	u32 luterr;
	u32 fcanie;
	u32 fcanic0;
	u32 fcanic1;
};

/* Acceptance Filter Mode Register bits */
#define CAN_AF_AFMR_RESET	(1 << 0)
#define CAN_AF_AFMR_BYPASS	(1 << 1)

static const struct can_bittiming_const lpc17xx_can_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static inline int lpc17xx_can_has_and_handle_berr(
	const struct lpc17xx_can_priv *priv, u32 icr)
{
	return (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) &&
		(icr & CAN_ICR_BEI);
}

static void lpc17xx_can_read_frame(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
        struct lpc17xx_can_priv *priv = netdev_priv(dev);
	struct lpc17xx_can_regs __iomem *regs = priv->base;
	struct can_frame *cf;

	if (LPC17XX_CAN_CF_NEXT(priv->cf_end) == priv->cf_start) {
		stats->rx_dropped++;
		regs->cmr = CAN_CMR_RRB;
		return;
	}

	cf = &priv->cf[priv->cf_end];

	cf->can_id = regs->rid;
	if (regs->rfs & CAN_RFS_FF)
		cf->can_id |= CAN_EFF_FLAG;
	if (regs->rfs & CAN_RFS_RTR)
		cf->can_id |= CAN_RTR_FLAG;
	cf->can_dlc = CAN_RFS_DLC(regs->rfs);
	cf->data[0] = (regs->rda >> 0) & 0xff;
	cf->data[1] = (regs->rda >> 8) & 0xff;
	cf->data[2] = (regs->rda >> 16) & 0xff;
	cf->data[3] = (regs->rda >> 24) & 0xff;
	cf->data[4] = (regs->rdb >> 0) & 0xff;
	cf->data[5] = (regs->rdb >> 8) & 0xff;
	cf->data[6] = (regs->rdb >> 16) & 0xff;
	cf->data[7] = (regs->rdb >> 24) & 0xff;

	regs->cmr = CAN_CMR_RRB;

	priv->cf_end = LPC17XX_CAN_CF_NEXT(priv->cf_end);
}

static void do_bus_err(struct net_device *dev, struct can_frame *cf)
{
	struct lpc17xx_can_priv *priv = netdev_priv(dev);
	struct lpc17xx_can_regs __iomem *regs = priv->base;

	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	switch (CAN_ICR_ERRC(regs->icr)) {
	case ERRC_BIT:
		cf->data[2] |= CAN_ERR_PROT_BIT;
		break;
	case ERRC_FORM:
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;
	case ERRC_STUFF:
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;
	}

	if ((regs->icr & CAN_ICR_ERRRCV) == 0)
		cf->data[2] |= CAN_ERR_PROT_TX;

	cf->data[3] = CAN_ICR_ERRBIT(regs->icr);

	priv->can.can_stats.bus_error++;
	if ((regs->icr & CAN_ICR_ERRRCV) == 0)
		dev->stats.tx_errors++;
	else
		dev->stats.rx_errors++;
}

static int lpc17xx_can_poll_bus_err(struct net_device *dev)
{
	struct sk_buff *skb;
	struct can_frame *cf;

	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	do_bus_err(dev, cf);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static int lpc17xx_can_poll_state(struct net_device *dev)
{
	struct lpc17xx_can_priv *priv = netdev_priv(dev);
	struct lpc17xx_can_regs __iomem *regs = priv->base;
	struct sk_buff *skb;
	struct can_frame *cf;
	enum can_state new_state = 0;

	/* state hasn't changed */
	if (likely(priv->can_state_changed))
		return 0;

	new_state = priv->can.state == CAN_STATE_ERROR_ACTIVE ?
		CAN_STATE_ERROR_PASSIVE : CAN_STATE_ERROR_ACTIVE;
	if (regs->gsr & CAN_GSR_BS)
		new_state = CAN_STATE_BUS_OFF;

	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	can_change_state(dev, cf, new_state, new_state);

	if (unlikely(new_state == CAN_STATE_BUS_OFF))
		can_bus_off(dev);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	priv->can_state_changed = 0;

	return 1;
}

static int lpc17xx_can_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct net_device_stats *stats = &dev->stats;
	struct lpc17xx_can_priv *priv = netdev_priv(dev);
	struct lpc17xx_can_regs __iomem *regs = priv->base;
	struct sk_buff *skb;
	struct can_frame *cf;
	int work_done = 0;

	/* handle state changes */
	work_done += lpc17xx_can_poll_state(dev);

	/* handle cached rx messages */
	while (priv->cf_start != priv->cf_end && work_done < quota) {
		skb = alloc_can_skb(dev, &cf);
		if (unlikely(!skb)) {
			stats->rx_dropped++;
			priv->cf_start = LPC17XX_CAN_CF_NEXT(priv->cf_start);
			break;
		}

		*cf = priv->cf[priv->cf_start];

		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_receive_skb(skb);

		can_led_event(dev, CAN_LED_EVENT_RX);

		priv->cf_start = LPC17XX_CAN_CF_NEXT(priv->cf_start);
		work_done++;
	}

	regs->ier |= CAN_IER_RIE;

	/* report bus errors */
	if (lpc17xx_can_has_and_handle_berr(priv, priv->saved_icr) &&
			work_done < quota)
		work_done += lpc17xx_can_poll_bus_err(dev);

	if (work_done < quota) {
		napi_complete(napi);
	}

	return work_done;
}

static irqreturn_t lpc17xx_can_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_device_stats *stats = &dev->stats;
	struct lpc17xx_can_priv *priv = netdev_priv(dev);
	struct lpc17xx_can_regs __iomem *regs = priv->base;
	u32 icr;

	icr = regs->icr;
	priv->saved_icr = icr;

	if (icr & CAN_ICR_RI) {
		while (regs->gsr & CAN_GSR_RBS)
			lpc17xx_can_read_frame(dev);
		napi_schedule(&priv->napi);
	}

	if ((icr & CAN_ICR_BEI) ||
			lpc17xx_can_has_and_handle_berr(priv, icr)) {
		regs->cmr = CAN_CMR_AT;
		napi_schedule(&priv->napi);
	}

	if (icr & CAN_ICR_EPI) {
		priv->can_state_changed = 1;
		napi_schedule(&priv->napi);
	}

	/* FIFO overflow */
	if (icr & CAN_ICR_DOI) {
		dev->stats.rx_over_errors++;
		dev->stats.rx_errors++;
		regs->cmr = CAN_CMR_CDO;
		regs->ier &= ~(CAN_IER_RIE | CAN_IER_DOIE);
	}

	/* transmission complete interrupt */
	if (icr & (CAN_ICR_TI1 | CAN_ICR_TI2 | CAN_ICR_TI3)) {
		stats->tx_bytes += can_get_echo_skb(dev, 0);
		stats->tx_packets++;
		can_led_event(dev, CAN_LED_EVENT_TX);
		netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static int __lpc17xx_can_get_berr_counter(const struct net_device *dev,
				      struct can_berr_counter *bec)
{
	const struct lpc17xx_can_priv *priv = netdev_priv(dev);
	struct lpc17xx_can_regs __iomem *regs = priv->base;

	bec->txerr = CAN_GSR_TXERR(regs->gsr);
	bec->rxerr = CAN_GSR_RXERR(regs->gsr);

	return 0;
}

static int lpc17xx_can_get_berr_counter(const struct net_device *dev,
				    struct can_berr_counter *bec)
{
	const struct lpc17xx_can_priv *priv = netdev_priv(dev);
	int err;

	err = clk_prepare_enable(priv->clk);
	if (err)
		return err;

	err = __lpc17xx_can_get_berr_counter(dev, bec);

	clk_disable_unprepare(priv->clk);

	return err;
}

static int lpc17xx_can_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	const struct lpc17xx_can_priv *priv = netdev_priv(dev);
	struct lpc17xx_can_regs __iomem *regs = priv->base;
	struct can_frame *cf = (struct can_frame *)skb->data;
	u32 idx, tfi;

	netdev_dbg(dev, "%s:%i\n", __func__, __LINE__);

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	if (regs->sr & CAN_SR_TBS1_1)
		idx = 0;
	else if (regs->sr & CAN_SR_TBS2_2)
		idx = 1;
	else if (regs->sr & CAN_SR_TBS3_3)
		idx = 2;
	else
		return -EBUSY;

	tfi = CAN_TFI_DLC_SET(cf->can_dlc);
	if (cf->can_id & CAN_EFF_FLAG)
		tfi |= CAN_TFI_FF;

	if (cf->can_id & CAN_RTR_FLAG)
		tfi |= CAN_TFI_RTR;

	if (cf->can_dlc > 0) {
		regs->tx[idx].tda = cf->data[0] |
			(cf->data[1] << 8) |
			(cf->data[2] << 16) |
			(cf->data[3] << 24);
	}
	if (cf->can_dlc > 3) {
		regs->tx[idx].tdb = cf->data[4] |
			(cf->data[5] << 8) |
			(cf->data[6] << 16) |
			(cf->data[7] << 24);
	}

	can_put_echo_skb(skb, dev, 0);

	regs->tx[idx].tfi = tfi;
	regs->tx[idx].tid = cf->can_id & CAN_EFF_MASK;

	regs->cmr = (CAN_CMR_STB1 << idx) | CAN_CMR_TR;

	return NETDEV_TX_OK;
}

static void lpc17xx_can_set_bittiming(struct net_device *dev)
{
	const struct lpc17xx_can_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	struct lpc17xx_can_regs __iomem *regs = priv->base;
	u8 nt, tseg1, tseg2, brfail;
	u32 div, brp;

	div = clk_get_rate(priv->clk) / bt->bitrate;

	brfail = 1;
	for (nt = 24; nt > 0; nt -= 2) {
		if (div % nt == 0) {
			brp = div / nt - 1;
			nt--;
			tseg2 = nt / 3 - 1;
			tseg1 = nt - nt / 3 - 1;
			brfail = 0;
			break;
		}
	}

	if (brfail) {
		netdev_err(dev, "can't set baudrate %i\n", bt->bitrate);
		return;
	}

	regs->btr = (tseg2 << CAN_BTR_TSEG2_SHIFT) |
		(tseg1 << CAN_BTR_TSEG1_SHIFT) |
		(3 << CAN_BTR_SJW_SHIFT) |
		(brp & CAN_BTR_BRP_MASK);

	/* print chip status */
	netdev_dbg(dev, "%s: rate %i btr=0x%08x\n", __func__, bt->bitrate, regs->btr);
}

/*
 * lpc17xx_can_chip_start
 *
 * this functions is entered with clocks enabled
 *
 */
static int lpc17xx_can_chip_start(struct net_device *dev)
{
	struct lpc17xx_can_priv *priv = netdev_priv(dev);
	volatile struct lpc17xx_can_regs __iomem *regs = priv->base;
	u32 tmp, mod;
	int rv = 0;

	if ((rv = clk_prepare_enable(priv->clk)) < 0) {
		netdev_err(dev, "failed to enable CAN clock, err %i\n", rv);
		return rv;
	}

	regs->mod = CAN_MOD_RM; /* enter reset mode */
	regs->ier = 0;          /* disable all CAN interrupts */
	regs->gsr = 0;           /* clear global status */
	regs->cmr = CAN_CMR_AT | CAN_CMR_RRB | CAN_CMR_CDO;
	tmp = regs->icr;         /* clear pending interrupts */

	lpc17xx_can_set_bittiming(dev);

	regs->ier = CAN_IER_RIE | CAN_IER_TIE1 | CAN_IER_TIE2 | CAN_IER_TIE3 |
		CAN_IER_DOIE | CAN_IER_EPIE | CAN_IER_BEIE;

	mod = 0;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		mod |= CAN_MOD_TM;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		mod |= CAN_MOD_LOM;
	regs->mod = mod;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	priv->can_state_changed = 0;
	priv->cf_start = priv->cf_end = 0;

	return rv;
}

/*
 * lpc17xx_can_chip_stop
 *
 * this functions is entered with clocks enabled
 *
 */
static void lpc17xx_can_chip_stop(struct net_device *dev)
{
	struct lpc17xx_can_priv *priv = netdev_priv(dev);

	priv->can.state = CAN_STATE_STOPPED;

	clk_disable_unprepare(priv->clk);

	return;
}

static int lpc17xx_can_open(struct net_device *dev)
{
	struct lpc17xx_can_priv *priv = netdev_priv(dev);
	int err;

	err = clk_prepare_enable(priv->clk);
	if (err)
		return err;

	err = open_candev(dev);
	if (err)
		goto out_disable_clk;

	err = request_irq(dev->irq, lpc17xx_can_irq, IRQF_SHARED, dev->name, dev);
	if (err)
		goto out_close;

	/* start chip and queuing */
	err = lpc17xx_can_chip_start(dev);
	if (err)
		goto out_free_irq;

	can_led_event(dev, CAN_LED_EVENT_OPEN);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;

 out_free_irq:
	free_irq(dev->irq, dev);
 out_close:
	close_candev(dev);
 out_disable_clk:
	clk_disable_unprepare(priv->clk);

	return err;
}

static int lpc17xx_can_close(struct net_device *dev)
{
	struct lpc17xx_can_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	lpc17xx_can_chip_stop(dev);

	free_irq(dev->irq, dev);
	clk_disable_unprepare(priv->clk);

	close_candev(dev);

	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static int lpc17xx_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	int err;

	switch (mode) {
	case CAN_MODE_START:
		err = lpc17xx_can_chip_start(dev);
		if (err)
			return err;

		netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops lpc17xx_can_netdev_ops = {
	.ndo_open	= lpc17xx_can_open,
	.ndo_stop	= lpc17xx_can_close,
	.ndo_start_xmit	= lpc17xx_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int register_lpc17xx_can_dev(struct net_device *dev)
{
	int err;

	err = register_candev(dev);

	return err;
}

static void unregister_lpc17xx_can_dev(struct net_device *dev)
{
	unregister_candev(dev);
}

static const struct of_device_id lpc17xx_can_of_match[] = {
	{ .compatible = "nxp,lpc17xx-can", .data = NULL, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, lpc17xx_can_of_match);

static int lpc17xx_can_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct net_device *dev;
	struct lpc17xx_can_priv *priv;
	struct resource *mem, *af_mem;
	struct clk *clk = NULL;
	void __iomem *base;
	struct lpc17xx_can_af_regs __iomem *af_regs;
	int err, irq;
	u32 clock_freq = 0;
	const struct lpc17xx_can_devtype_data *devtype_data;
	struct reset_control *rst;

	clk = devm_clk_get(&pdev->dev, "can");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no CAN clock defined\n");
		return PTR_ERR(clk);
	}
	clock_freq = clk_get_rate(clk);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	af_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENODEV;

	base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(base))
		return PTR_ERR(base);

	af_regs = devm_ioremap_resource(&pdev->dev, af_mem);
	if (IS_ERR(af_regs))
		return PTR_ERR(af_regs);

	/* enable clock to configure acceptance filter */
	if ((err = clk_prepare_enable(clk))) {
		dev_err(&pdev->dev, "can't enable CAN clock");
		return err;
	}

	/* bypass acceptance filter */
	af_regs->afmr = CAN_AF_AFMR_BYPASS;

	clk_disable_unprepare(clk);

	/* don't need acceptance filter mapping */
	devm_iounmap(&pdev->dev, af_regs);
	devm_release_mem_region(&pdev->dev, af_mem->start, resource_size(af_mem));

	of_id = of_match_device(lpc17xx_can_of_match, &pdev->dev);
	if (of_id) {
		devtype_data = of_id->data;
	} else {
		return -ENODEV;
	}

	rst = devm_reset_control_get(&pdev->dev, NULL);
	if (!IS_ERR_OR_NULL(rst))
		reset_control_reset(rst);

	dev = alloc_candev(sizeof(struct lpc17xx_can_priv), 1);
	if (!dev)
		return -ENOMEM;

	dev->netdev_ops = &lpc17xx_can_netdev_ops;
	dev->irq = irq;
	dev->flags |= IFF_ECHO;

	priv = netdev_priv(dev);
	priv->can.clock.freq = clock_freq;
	priv->can.bittiming_const = &lpc17xx_can_bittiming_const;
	priv->can.do_set_mode = lpc17xx_can_set_mode;
	priv->can.do_get_berr_counter = lpc17xx_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY	| CAN_CTRLMODE_BERR_REPORTING;
	priv->base = base;
	priv->clk = clk;
	priv->pdata = dev_get_platdata(&pdev->dev);
	priv->devtype_data = devtype_data;

	netif_napi_add(dev, &priv->napi, lpc17xx_can_poll, LPC17XX_CAN_NAPI_WEIGHT);

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = register_lpc17xx_can_dev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering netdev failed\n");
		goto failed_register;
	}

	devm_can_led_init(dev);

	dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%d)\n",
		 priv->base, dev->irq);

	return 0;

 failed_register:
	free_candev(dev);
	return err;
}

static int lpc17xx_can_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct lpc17xx_can_priv *priv = netdev_priv(dev);

	unregister_lpc17xx_can_dev(dev);
	netif_napi_del(&priv->napi);
	free_candev(dev);

	return 0;
}

static int __maybe_unused lpc17xx_can_suspend(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct lpc17xx_can_priv *priv = netdev_priv(dev);

	if (netif_running(dev)) {
		netif_stop_queue(dev);
		netif_device_detach(dev);
	}
	priv->can.state = CAN_STATE_SLEEPING;

	return 0;
}

static int __maybe_unused lpc17xx_can_resume(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct lpc17xx_can_priv *priv = netdev_priv(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	if (netif_running(dev)) {
		netif_device_attach(dev);
		netif_start_queue(dev);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(lpc17xx_can_pm_ops, lpc17xx_can_suspend, lpc17xx_can_resume);

static struct platform_driver lpc17xx_can_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &lpc17xx_can_pm_ops,
		.of_match_table = lpc17xx_can_of_match,
	},
	.probe = lpc17xx_can_probe,
	.remove = lpc17xx_can_remove,
};

module_platform_driver(lpc17xx_can_driver);
