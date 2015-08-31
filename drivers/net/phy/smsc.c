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

static const char *smsc_cable_types_str[4] = {
	"Default",
	"Shorted cable",
	"Open cable",
	"Match cable"
};

static const int smsc_match_cable_length_lookup[16] = {
	 0,   0,   0,   0,
	 6,  17,  27,  38,
	49,  59,  70,  81,
	91, 102, 113, 123
};

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

static int perform_cable_diag(struct phy_device *phydev, int mdix)
{
	int i, max_wait = 10, rc;

	rc = phy_write(phydev, MII_BMCR,
		  BMCR_SPEED100 | BMCR_FULLDPLX);

	if (mdix)
		rc = phy_write(phydev, MII_LAN83C185_SCS,
			  MII_LAN83C185_SCS_DISABLE_AUTO_MDIX |
			  MII_LAN83C185_SCS_MDIX);
	else
		rc = phy_write(phydev, MII_LAN83C185_SCS,
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
			goto out;
		}

		mdelay(500);
		rc = -1;
	}

	if (i >= max_wait)
		dev_err(&phydev->dev, "TDR status read timeout\n");

out:
	return rc;
}

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

static ssize_t show_cable_diag(struct device *d,
			       struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(d);
	int rc, tx_cable_type, tx_cable_length, rx_cable_type, rx_cable_length;
	const char *tx_caption  = "estimated cable length (+/- 20m)";
	const char *rx_caption = "estimated cable length (+/- 20m)";
	const char *tx_help = "", *rx_help = "";
	const char *help_shorted = "\nMultiply this value by: 0.759 for CAT 6 cable, 0.788 for CAT 5E, 0.873 for CAT 5, 0.793 for unknown\n";
	const char *help_open = "\nMultiply this value by: 0.745 for CAT 6 cable, 0.76 for CAT 5E, 0.85 for CAT 5, 0.769 for unknown\n";

	mmd_write(phydev, MDIO_MMD_VEND1,
		  MMD_LAN8742A_TDR_MATCH_THR_REG,
		  MMD_LAN8742A_TDR_MATCH_THR_VAL);

	mmd_write(phydev, MDIO_MMD_VEND1,
		  MMD_LAN8742A_TDR_OPEN_THR_REG,
		  MMD_LAN8742A_TDR_OPEN_THR_VAL);

	rc = perform_cable_diag(phydev, 0);
	if (rc < 0)
		goto err;
	tx_cable_type = (rc & MII_LAN83C185_TDR_CABLE_TYPE_MASK) >>
		MII_LAN83C185_TDR_CABLE_TYPE_SHIFT;

	if (SMSC_CABLE_TYPE__MATCH == tx_cable_type) {
		rc = phy_read(phydev, MII_LAN83C185_CBLN);
		tx_cable_length = smsc_match_cable_length_lookup[
			(rc & MII_LAN83C185_CBLN_MASK) >>
			MII_LAN83C185_CBLN_SHIFT];
	} else {
		tx_caption = "Estimated physical distance to the fault";
		if (SMSC_CABLE_TYPE__SHORTED == tx_cable_type)
			tx_help = help_shorted;
		else if (SMSC_CABLE_TYPE__OPEN == tx_cable_type)
			tx_help = help_open;
		tx_cable_length = (rc & MII_LAN83C185_TDR_CABLE_LENGTH_MASK) >>
			MII_LAN83C185_TDR_CABLE_LENGTH_SHIFT;
	}

	rc = perform_cable_diag(phydev, 1);
	if (rc < 0)
		goto err;
	rx_cable_type = (rc & MII_LAN83C185_TDR_CABLE_TYPE_MASK) >>
		MII_LAN83C185_TDR_CABLE_TYPE_SHIFT;

	if (SMSC_CABLE_TYPE__MATCH == rx_cable_type) {
		rc = phy_read(phydev, MII_LAN83C185_CBLN);
		rx_cable_length = smsc_match_cable_length_lookup[
			(rc & MII_LAN83C185_CBLN_MASK) >>
			MII_LAN83C185_CBLN_SHIFT];
	} else {
		rx_caption = "Estimated physical distance to the fault";
		if (SMSC_CABLE_TYPE__SHORTED == rx_cable_type)
			rx_help = help_shorted;
		else if (SMSC_CABLE_TYPE__OPEN == rx_cable_type)
			rx_help = help_open;
		rx_cable_length = (rc & MII_LAN83C185_TDR_CABLE_LENGTH_MASK) >>
			MII_LAN83C185_TDR_CABLE_LENGTH_SHIFT;
	}

	rc = sprintf(buf, "TX: %s, %s: %d%s\nRX: %s, %s: %d%s\n",
		     smsc_cable_types_str[tx_cable_type],
		     tx_caption,
		     tx_cable_length,
		     tx_help,
		     smsc_cable_types_str[rx_cable_type],
		     rx_caption,
		     rx_cable_length,
		     rx_help);

err:
	/* Set Auto-functionality back */
	phy_write(phydev, MII_LAN83C185_SCS, 0);
	phy_write(phydev, MII_LAN83C185_TDR, 0);
	phy_write(phydev, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	return rc;
}

static DEVICE_ATTR(cable_diagnostics, S_IRUGO, show_cable_diag, NULL);

static ssize_t dump_registers(struct device *d,
			      struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(d);
	int rc, i, total_len = 0;

	for (i = 0; i < 32; ++i) {
		int len;
		rc = phy_read(phydev, i);
		if (rc >= 0)
			len = sprintf(buf, "reg %2d: 0x%x\n", i, rc);
		else
			len = sprintf(buf, "reg %2d: READ ERROR %d\n", i, rc);
		total_len += len;
		buf += len;
	}

	return total_len;
}

static DEVICE_ATTR(dump_registers, S_IRUGO, dump_registers, NULL);

static struct attribute *lan8742A_sysfs_entries[] = {
	&dev_attr_cable_diagnostics.attr,
	&dev_attr_dump_registers.attr,
	NULL
};

static struct attribute_group lan8742A_attribute_group = {
	.name = NULL,
	.attrs = lan8742A_sysfs_entries,
};

static int smsc8742Ax_fixup(struct phy_device *phydev)
{
	int rc;
	int phy_addr = phydev->addr;

	/*
	 * 8742A responds on PHY Addr 0 (which is broadcast)
	 * after reset, so reconfigure it to PHY Addr 1.
	 */
	phydev->addr = 0;
	rc = phy_read(phydev, MII_LAN83C185_SPECIAL_MODES);
	if (rc < 0)
		return rc;

	rc |= MII_LAN83C185_MODE_PHYADDR_1;
	rc = phy_write(phydev, MII_LAN83C185_SPECIAL_MODES, rc);
	phydev->addr = phy_addr;

	return rc;
}

static int lan8742Ax_config_init(struct phy_device *phydev)
{
	int rc = smsc8742Ax_fixup(phydev);

	if (rc < 0)
		return rc;

	rc = phy_read(phydev, MII_LAN83C185_SPECIAL_MODES);
	if (rc < 0)
		return rc;

	rc |= MII_LAN83C185_MODE_ALL;
	rc = phy_write(phydev, MII_LAN83C185_SPECIAL_MODES, rc);
	if (rc < 0)
		return rc;

	rc = phy_read(phydev, MII_BMCR);
	if (rc < 0)
		return rc;

	rc |= BMCR_ANRESTART;
	rc = phy_write(phydev, MII_BMCR, rc);
	if (rc < 0)
		return rc;

	return smsc_phy_ack_interrupt(phydev);
}

int lan8742Ax_probe(struct phy_device *phydev)
{
	return sysfs_create_group(&phydev->dev.kobj,
				  &lan8742A_attribute_group);
}

static void lan8742Ax_remove(struct phy_device *phydev)
{
	sysfs_remove_group(&phydev->dev.kobj,
			   &lan8742A_attribute_group);
}

static int lan8742Ax_read_status(struct phy_device *phydev)
{
	int rc = phy_read(phydev, MII_LAN83C185_SPECIAL_MODES);

	if (0xffff == rc) {
		/*
		 * PHY was on PHY Addr 1, but after unexpected
		 * reset switched to PHY Addr 0.
		 */
		lan8742Ax_config_init(phydev);
	}

	return genphy_read_status(phydev);
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
	.phy_id		= MII_LAN8742A_ID,
	.phy_id_mask	= MII_LAN8742A_ID_MASK,
	.name		= "SMSC LAN8742A/LAN8742Ai",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= lan8742Ax_read_status,
	.config_init	= lan8742Ax_config_init,
	.probe		= lan8742Ax_probe,
	.remove		= lan8742Ax_remove,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= { .owner = THIS_MODULE, }
} };

module_phy_driver(smsc_phy_driver);

static int __init smsc_init(void)
{
	return phy_register_fixup_for_uid(MII_LAN8742A_ID,
					  MII_LAN8742A_ID_MASK,
					  smsc8742Ax_fixup);
}

MODULE_DESCRIPTION("SMSC PHY driver");
MODULE_AUTHOR("Herbert Valerio Riedel");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused smsc_tbl[] = {
	{ 0x0007c0a0, 0xfffffff0 },
	{ 0x0007c0b0, 0xfffffff0 },
	{ 0x0007c0c0, 0xfffffff0 },
	{ 0x0007c0d0, 0xfffffff0 },
	{ 0x0007c0f0, 0xfffffff0 },
	{ MII_LAN8742A_ID, MII_LAN8742A_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, smsc_tbl);

module_init(smsc_init);
