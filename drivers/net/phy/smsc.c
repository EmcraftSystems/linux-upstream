/*
 * drivers/net/phy/smsc.c
 *
 * Driver for SMSC PHYs
 *
 * Author: Herbert Valerio Riedel
 *
 * Copyright (c) 2006 Herbert Valerio Riedel <hvr@gnu.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support added for SMSC LAN8187 and LAN8700 by steve.glendinning@shawell.net
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/smscphy.h>
#include <linux/mdio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

struct smsc_private {
	int wol_supported;
	int int_irq;
};

static const int smsc_match_cable_length_lookup[16] = {
	 0,   0,   0,   0,
	 6,  17,  27,  38,
	49,  59,  70,  81,
	91, 102, 113, 123
};

static const char smsc_gstrings[][ETH_GSTRING_LEN] = {
	"TX cable match\t",
	"TX cable opened\t",
	"TX cable shorted",
	"TX length (if match)",
	"TX distance to fault",
	"RX cable match\t",
	"RX cable opened\t",
	"RX cable shorted",
	"RX length (if match)",
	"RX distance to fault",
};

#define SMSC_TEST_LEN		ARRAY_SIZE(smsc_gstrings)
#define SMSC_STRINGS_LEN	(SMSC_TEST_LEN * ETH_GSTRING_LEN)

static int mmd_set_addr(struct phy_device *phydev, int dev, int reg)
{
	int rc = phy_write(phydev, MII_LAN83C185_MMD_CONTROL,
			   MII_LAN83C185_MMD_CONTROL_FUNC_ADDRESS | dev);
	if (rc < 0)
		return rc;

	rc = phy_write(phydev, MII_LAN83C185_MMD_ACCESS, reg);
	if (rc < 0)
		return rc;

	rc = phy_write(phydev, MII_LAN83C185_MMD_CONTROL,
		       MII_LAN83C185_MMD_CONTROL_FUNC_DATA | dev);

	return rc;
}

static int mmd_write(struct phy_device *phydev, int dev, int reg, int val)
{
	int rc = mmd_set_addr(phydev, dev, reg);
	if (rc < 0)
		return rc;

	rc = phy_write(phydev, MII_LAN83C185_MMD_ACCESS, val);

	return rc;
}

static int mmd_read(struct phy_device *phydev, int dev, int reg)
{
	int rc = mmd_set_addr(phydev, dev, reg);
	if (rc < 0)
		return rc;

	rc = phy_read(phydev, MII_LAN83C185_MMD_ACCESS);

	return rc;
}

static int smsc_phy_config_intr(struct phy_device *phydev)
{
	int rc = phy_write (phydev, MII_LAN83C185_IM,
			((PHY_INTERRUPT_ENABLED == phydev->interrupts)
			? MII_LAN83C185_ISF_INT_PHYLIB_EVENTS
			: 0));

	return rc < 0 ? rc : 0;
}

static int smsc_phy_ack_interrupt(struct phy_device *phydev)
{
	int rc = phy_read (phydev, MII_LAN83C185_ISF);

	return rc < 0 ? rc : 0;
}

static int smsc_phy_config_init(struct phy_device *phydev)
{
	int rc = phy_read(phydev, MII_LAN83C185_CTRL_STATUS);

	if (rc < 0)
		return rc;

	/* Enable energy detect mode for this SMSC Transceivers */
	rc = phy_write(phydev, MII_LAN83C185_CTRL_STATUS,
		       rc | MII_LAN83C185_EDPWRDOWN);
	if (rc < 0)
		return rc;

	return smsc_phy_ack_interrupt(phydev);
}

static int smsc_phy_reset(struct phy_device *phydev)
{
	int rc = phy_read(phydev, MII_LAN83C185_SPECIAL_MODES);
	if (rc < 0)
		return rc;

	/* If the SMSC PHY is in power down mode, then set it
	 * in all capable mode before using it.
	 */
	if ((rc & MII_LAN83C185_MODE_MASK) == MII_LAN83C185_MODE_POWERDOWN) {
		int timeout = 50000;

		/* set "all capable" mode and reset the phy */
		rc |= MII_LAN83C185_MODE_ALL;
		phy_write(phydev, MII_LAN83C185_SPECIAL_MODES, rc);
		phy_write(phydev, MII_BMCR, BMCR_RESET);

		/* wait end of reset (max 500 ms) */
		do {
			udelay(10);
			if (timeout-- == 0)
				return -1;
			rc = phy_read(phydev, MII_BMCR);
		} while (rc & BMCR_RESET);
	}
	return 0;
}

static int lan911x_config_init(struct phy_device *phydev)
{
	return smsc_phy_ack_interrupt(phydev);
}

/*
 * The LAN87xx suffers from rare absence of the ENERGYON-bit when Ethernet cable
 * plugs in while LAN87xx is in Energy Detect Power-Down mode. This leads to
 * unstable detection of plugging in Ethernet cable.
 * This workaround disables Energy Detect Power-Down mode and waiting for
 * response on link pulses to detect presence of plugged Ethernet cable.
 * The Energy Detect Power-Down mode is enabled again in the end of procedure to
 * save approximately 220 mW of power if cable is unplugged.
 */
static int lan87xx_read_status(struct phy_device *phydev)
{
	int err = genphy_read_status(phydev);
	int i;

	if (!phydev->link) {
		/* Disable EDPD to wake up PHY */
		int rc = phy_read(phydev, MII_LAN83C185_CTRL_STATUS);
		if (rc < 0)
			return rc;

		rc = phy_write(phydev, MII_LAN83C185_CTRL_STATUS,
			       rc & ~MII_LAN83C185_EDPWRDOWN);
		if (rc < 0)
			return rc;

		/* Wait max 640 ms to detect energy */
		for (i = 0; i < 64; i++) {
			/* Sleep to allow link test pulses to be sent */
			msleep(10);
			rc = phy_read(phydev, MII_LAN83C185_CTRL_STATUS);
			if (rc < 0)
				return rc;
			if (rc & MII_LAN83C185_ENERGYON)
				break;
		}

		/* Re-enable EDPD */
		rc = phy_read(phydev, MII_LAN83C185_CTRL_STATUS);
		if (rc < 0)
			return rc;

		rc = phy_write(phydev, MII_LAN83C185_CTRL_STATUS,
			       rc | MII_LAN83C185_EDPWRDOWN);
		if (rc < 0)
			return rc;
	}

	return err;
}

static irqreturn_t lan8742_irq(int irq, void *dev_id)
{
	/* Wake-on-LAN */
	return IRQ_HANDLED;
}

int lan8742_probe(struct phy_device *phydev)
{
	const struct device *dev = &phydev->dev;
	struct device_node *node = dev->of_node;
	int nint_gpio = of_get_named_gpio(node, "nint-gpios", 0);
	struct smsc_private *priv = kzalloc(sizeof(*priv), GFP_KERNEL);

	priv->wol_supported = 0;
	phydev->priv = priv;

	if (gpio_is_valid(nint_gpio)) {
		int err;

		priv->int_irq = gpio_to_irq(nint_gpio);

		err = request_irq(priv->int_irq, lan8742_irq,
				  IRQF_TRIGGER_FALLING, "smsc8742", phydev);
		if (err)
			dev_err(dev, "failed to request PHY irq %d from gpio %d: %d\n",
				gpio_to_irq(nint_gpio), nint_gpio, err);
		else
			priv->wol_supported = 1;
	}

	return 0;
}

static void lan8742_remove(struct phy_device *phydev)
{
	if (phydev->priv)
		kfree(phydev->priv);
}

static int lan8742_set_wol(struct phy_device *phydev,
			     struct ethtool_wolinfo *wol)
{
	u32 wucsr = mmd_read(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG);
	struct net_device *ndev = phydev->attached_dev;
	struct smsc_private *priv = phydev->priv;
	const u8 *mac;

	if (!ndev)
		return -ENODEV;

	if (!priv->wol_supported)
		return -EOPNOTSUPP;

	mac = (const u8*)ndev->dev_addr;
	mmd_write(phydev, MDIO_MMD_PCS, MMD_LAN8742_MAC_RECEIVE_ADDR_A, (mac[5] << 8) | mac[4]);
	mmd_write(phydev, MDIO_MMD_PCS, MMD_LAN8742_MAC_RECEIVE_ADDR_B, (mac[3] << 8) | mac[2]);
	mmd_write(phydev, MDIO_MMD_PCS, MMD_LAN8742_MAC_RECEIVE_ADDR_C, (mac[1] << 8) | mac[0]);

	if (wol->wolopts & WAKE_MAGIC)
		wucsr |= MMD_LAN8742_WUCS_MAGIC_ENABLED;
	else
		wucsr &= ~MMD_LAN8742_WUCS_MAGIC_ENABLED;

	mmd_write(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG, wucsr);

	return 0;
}

static void lan8742_get_wol(struct phy_device *phydev,
			   struct ethtool_wolinfo *wol)
{
	u32 wucsr = mmd_read(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG);
	struct smsc_private *priv = phydev->priv;

	wol->wolopts = 0;

	if (priv->wol_supported) {
		wol->supported = WAKE_MAGIC;

		if (wucsr & MMD_LAN8742_WUCS_MAGIC_ENABLED)
			wol->wolopts |= WAKE_MAGIC;
	} else {
		wol->supported = 0;
	}
}

static int lan8742_suspend(struct phy_device *phydev)
{
	u32 wucsr = mmd_read(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG);
	struct smsc_private *priv = phydev->priv;
	int err = 0;

	if (priv->wol_supported && (wucsr & MMD_LAN8742_WUCS_MAGIC_ENABLED)) {
		u32 im = phy_read(phydev, MII_LAN83C185_IM);
		im |= MII_LAN83C185_ISF_INT8;
		phy_write(phydev, MII_LAN83C185_IM, im);

		wucsr |= MMD_LAN8742_WUCS_INTERFACE_DISABLE;
		mmd_write(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG, wucsr);

		enable_irq_wake(priv->int_irq);
	} else {
		wucsr &= ~MMD_LAN8742_WUCS_MAGIC_ENABLED;
		mmd_write(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG, wucsr);
		err = genphy_suspend(phydev);
	}

	return err;
}

static int lan8742_resume(struct phy_device *phydev)
{
	u32 wucsr = mmd_read(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG);
	struct smsc_private *priv = phydev->priv;
	int err = 0;

	if (priv->wol_supported && (wucsr & MMD_LAN8742_WUCS_MAGIC_ENABLED) &&
	    (wucsr & MMD_LAN8742_WUCS_INTERFACE_DISABLE)) {
		u32 im = phy_read(phydev, MII_LAN83C185_IM);
		im &= ~MII_LAN83C185_ISF_INT8;
		phy_write(phydev, MII_LAN83C185_IM, im);

		disable_irq_wake(priv->int_irq);

		wucsr &= ~MMD_LAN8742_WUCS_INTERFACE_DISABLE;
		mmd_write(phydev, MDIO_MMD_PCS, MMD_LAN8742_WUCSR_REG, wucsr);
	} else {
		err = genphy_resume(phydev);
	}

	return err;
}

static int perform_cable_diag(struct phy_device *phydev, int mdix,
			      int *cable_type, int *cable_length)
{
	int i, max_wait = 10, rc;
	int ctrl_reg = phy_read(phydev, MII_LAN83C185_CTRL_STATUS);

	phy_write(phydev, MII_LAN83C185_CTRL_STATUS,
		  ctrl_reg & (~MII_LAN83C185_EDPWRDOWN));

	mmd_write(phydev, MDIO_MMD_VEND1,
		  MMD_LAN8742_TDR_MATCH_THR_REG,
		  MMD_LAN8742_TDR_MATCH_THR_VAL);

	mmd_write(phydev, MDIO_MMD_VEND1,
		  MMD_LAN8742_TDR_OPEN_THR_REG,
		  MMD_LAN8742_TDR_OPEN_THR_VAL);

	phy_write(phydev, MII_BMCR,
		  BMCR_SPEED100 | BMCR_FULLDPLX);

	if (mdix)
		phy_write(phydev, MII_LAN83C185_SCS,
			  MII_LAN83C185_SCS_DISABLE_AUTO_MDIX |
			  MII_LAN83C185_SCS_MDIX);
	else
		phy_write(phydev, MII_LAN83C185_SCS,
			  MII_LAN83C185_SCS_DISABLE_AUTO_MDIX);

	mdelay(500);

	rc = phy_write(phydev, MII_LAN83C185_TDR,
		       MII_LAN83C185_TDR_ENABLE);

	for (i = 0; i < max_wait; ++i) {
		rc = phy_read(phydev, MII_LAN83C185_TDR);
		if (rc >= 0) {
			if ((rc & MII_LAN83C185_TDR_STATUS) &&
			    !(rc & MII_LAN83C185_TDR_ENABLE))
				break;
		} else {
			dev_err(&phydev->dev, "Failed to read TDR PHY register: %d\n", rc);
			goto err;
		}

		mdelay(500);
		rc = -1;
	}

	phy_write(phydev, MII_LAN83C185_CTRL_STATUS, ctrl_reg);

	if (i >= max_wait) {
		dev_err(&phydev->dev, "TDR status read timeout\n");
		goto err;
	}

	*cable_type =
		(rc & MII_LAN83C185_TDR_CABLE_TYPE_MASK) >>
		MII_LAN83C185_TDR_CABLE_TYPE_SHIFT;

	if (SMSC_CABLE_TYPE__MATCH == *cable_type) {
		int raw_len;

		rc = phy_read(phydev, MII_LAN83C185_CBLN);
		raw_len = (rc & MII_LAN83C185_CBLN_MASK) >> MII_LAN83C185_CBLN_SHIFT;
		*cable_length = smsc_match_cable_length_lookup[raw_len];
	} else {
		*cable_length =
			(rc & MII_LAN83C185_TDR_CABLE_LENGTH_MASK) >>
			MII_LAN83C185_TDR_CABLE_LENGTH_SHIFT;
	}

	return 0;

err:
	*cable_type = -1;
	*cable_length = -1;
	return rc;
}

int lan8742_get_sset_count(struct phy_device *phydev, int sset)
{
	switch (sset) {
	case ETH_SS_TEST:
		return SMSC_TEST_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

void lan8742_get_strings(struct phy_device *phydev, u32 stringset, u8 *data)
{
	switch (stringset) {
	case ETH_SS_TEST:
		memcpy(data, smsc_gstrings, SMSC_STRINGS_LEN);
		break;
	}
}

void lan8742_self_test(struct phy_device *phydev,
		       struct ethtool_test *eth_test, u64 *data)
{
	int tx_state, tx_len, rx_state, rx_len;
	mutex_lock(&phydev->lock);

	perform_cable_diag(phydev, 0, &tx_state, &tx_len);

	perform_cable_diag(phydev, 1, &rx_state, &rx_len);

	data[0] = (tx_state == SMSC_CABLE_TYPE__MATCH);
	data[1] = (tx_state == SMSC_CABLE_TYPE__OPEN);
	data[2] = (tx_state == SMSC_CABLE_TYPE__SHORTED);
	data[3] = (tx_state == SMSC_CABLE_TYPE__MATCH) ? tx_len : 0;
	data[4] = (tx_state == SMSC_CABLE_TYPE__MATCH) ? 0 : tx_len;

	data[5] = (rx_state == SMSC_CABLE_TYPE__MATCH);
	data[6] = (rx_state == SMSC_CABLE_TYPE__OPEN);
	data[7] = (rx_state == SMSC_CABLE_TYPE__SHORTED);
	data[8] = (rx_state == SMSC_CABLE_TYPE__MATCH) ? rx_len : 0;
	data[9] = (rx_state == SMSC_CABLE_TYPE__MATCH) ? 0 : rx_len;

	/* Set Auto-functionality back */
	phy_write(phydev, MII_LAN83C185_SCS, 0);
	phy_write(phydev, MII_LAN83C185_TDR, 0);
	phy_write(phydev, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	mutex_unlock(&phydev->lock);
}

static struct phy_driver smsc_phy_driver[] = {
{
	.phy_id		= 0x0007c0a0, /* OUI=0x00800f, Model#=0x0a */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN83C185",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= { .owner = THIS_MODULE, }
}, {
	.phy_id		= 0x0007c0b0, /* OUI=0x00800f, Model#=0x0b */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN8187",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= { .owner = THIS_MODULE, }
}, {
	.phy_id		= 0x0007c0c0, /* OUI=0x00800f, Model#=0x0c */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN8700",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= lan87xx_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= { .owner = THIS_MODULE, }
}, {
	.phy_id		= 0x0007c0d0, /* OUI=0x00800f, Model#=0x0d */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN911x Internal PHY",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.config_init	= lan911x_config_init,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= { .owner = THIS_MODULE, }
}, {
	.phy_id		= 0x0007c0f0, /* OUI=0x00800f, Model#=0x0f */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN8710/LAN8720",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= lan87xx_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= { .owner = THIS_MODULE, }
}, {
	.phy_id		= MII_LAN8742_ID,
	.phy_id_mask	= MII_LAN8742_ID_MASK,
	.name		= "SMSC LAN8742",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= lan87xx_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,
	.probe		= lan8742_probe,
	.remove		= lan8742_remove,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= lan8742_suspend,
	.resume		= lan8742_resume,
	.set_wol	= lan8742_set_wol,
	.get_wol	= lan8742_get_wol,
	.get_sset_count	= lan8742_get_sset_count,
	.get_strings	= lan8742_get_strings,
	.self_test	= lan8742_self_test,

	.driver		= { .owner = THIS_MODULE, }
} };

module_phy_driver(smsc_phy_driver);

MODULE_DESCRIPTION("SMSC PHY driver");
MODULE_AUTHOR("Herbert Valerio Riedel");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused smsc_tbl[] = {
	{ 0x0007c0a0, 0xfffffff0 },
	{ 0x0007c0b0, 0xfffffff0 },
	{ 0x0007c0c0, 0xfffffff0 },
	{ 0x0007c0d0, 0xfffffff0 },
	{ 0x0007c0f0, 0xfffffff0 },
	{ MII_LAN8742_ID, MII_LAN8742_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, smsc_tbl);
