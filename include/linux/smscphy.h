#ifndef __LINUX_SMSCPHY_H__
#define __LINUX_SMSCPHY_H__

#define MII_LAN8742_ID		0x0007c130
#define MII_LAN8742_ID_MASK	0xfffffff0

#define MII_LAN83C185_MMD_CONTROL	13	/* MMD Access Control Register */
#define MII_LAN83C185_MMD_ACCESS	14	/* MMD Access Address/Data Register */
#define MII_LAN83C185_CTRL_STATUS	17	/* Mode/Status Register */
#define MII_LAN83C185_SPECIAL_MODES	18	/* Special Modes Register */
#define MII_LAN83C185_TDR		25	/* TDR Control/Status Register */
#define MII_LAN83C185_SCS		27	/* Special Control/Status Register */
#define MII_LAN83C185_CBLN		28	/* Cable Length Register */
#define MII_LAN83C185_ISF		29	/* Interrupt Source Flags */
#define MII_LAN83C185_IM		30	/* Interrupt Mask */

#define MII_LAN83C185_ISF_INT1	(1<<1)		/* Auto-Negotiation Page Received */
#define MII_LAN83C185_ISF_INT2	(1<<2)		/* Parallel Detection Fault */
#define MII_LAN83C185_ISF_INT3	(1<<3)		/* Auto-Negotiation LP Ack */
#define MII_LAN83C185_ISF_INT4	(1<<4)		/* Link Down */
#define MII_LAN83C185_ISF_INT5	(1<<5)		/* Remote Fault Detected */
#define MII_LAN83C185_ISF_INT6	(1<<6)		/* Auto-Negotiation complete */
#define MII_LAN83C185_ISF_INT7	(1<<7)		/* ENERGYON */
#define MII_LAN83C185_ISF_INT8	(1<<8)		/* Wake on LAN */

#define MII_LAN83C185_ISF_INT_ALL (0x0e)

#define MII_LAN83C185_ISF_INT_PHYLIB_EVENTS \
	(MII_LAN83C185_ISF_INT6 | MII_LAN83C185_ISF_INT4 | \
	 MII_LAN83C185_ISF_INT7)

#define MII_LAN83C185_EDPWRDOWN			(1 << 13)	/* EDPWRDOWN */
#define MII_LAN83C185_ENERGYON			(1 << 1)	/* ENERGYON */

#define MII_LAN83C185_MODE_MASK			0xE0
#define MII_LAN83C185_MODE_POWERDOWN		0xC0		/* Power Down mode */
#define MII_LAN83C185_MODE_ALL			0xE0		/* All capable mode */
#define MII_LAN83C185_MODE_PHYADDR_1		0x1		/* Set to 1: Address 0 is a broadcast */

#define MII_LAN83C185_TDR_ENABLE		(0x8000)	/* TDR Enable */
#define MII_LAN83C185_TDR_STATUS		(0x0100)	/* TDR Channel Status */
#define MII_LAN83C185_TDR_CABLE_TYPE_MASK	(0x0600)	/* TDR Channel Cable Type */
#define MII_LAN83C185_TDR_CABLE_TYPE_SHIFT	(9)		/* TDR Channel Cable Type */
#define MII_LAN83C185_TDR_CABLE_LENGTH_MASK	(0x00ff)	/* TDR Channel Cable Length */
#define MII_LAN83C185_TDR_CABLE_LENGTH_SHIFT	(0)		/* TDR Channel Cable Length */

#define MII_LAN83C185_CBLN_MASK			(0xf000)	/* Cable Length */
#define MII_LAN83C185_CBLN_SHIFT		(12)		/* Cable Length */

#define MII_LAN83C185_MMD_CONTROL_FUNC_ADDRESS	(0 << 14)	/* MMD Function: Address */
#define MII_LAN83C185_MMD_CONTROL_FUNC_DATA	(1 << 14)	/* MMD Function: Data */

#define MII_LAN83C185_MMD_CONTROL_DEVAD_MASK	(0x1F)		/* MMD Device Address: field mask */

#define MII_LAN83C185_SCS_DISABLE_AUTO_MDIX	(0x8000)	/* Disable Auto-MDIX */
#define MII_LAN83C185_SCS_MDIX			(0x2000)	/* TX receives, RX transmits */

#define MMD_LAN8742_TDR_MATCH_THR_REG		11		/* TDR Match Threshold Register address*/
#define MMD_LAN8742_TDR_MATCH_THR_VAL	(0x09 | (0x12 << 5))	/* TDR Match Threshold Register value */

#define MMD_LAN8742_TDR_OPEN_THR_REG		12		/* TDR Short/Open Threshold Register address */
#define MMD_LAN8742_TDR_OPEN_THR_VAL	(0x12 | (0x09 << 5))	/* TDR Short/Open Threshold Register value*/

#define MMD_LAN8742_WUCSR_REG			32784		/* Wakeup Control and Status Register */
#define MMD_LAN8742_WUCS_INTERFACE_DISABLE	(1 << 15)	/* RMII Interface Disabled */
#define MMD_LAN8742_WUCS_WOL_CONFIGURED		(1 << 8)	/* WoL Configured */
#define MMD_LAN8742_WUCS_MAGIC_RECEIVED		(1 << 5)	/* Magic Packet Received */
#define MMD_LAN8742_WUCS_MAGIC_ENABLED		(1 << 1)	/* Wake On Magic Packet Enabled */

#define MMD_LAN8742_MAC_RECEIVE_ADDR_A		32865		/* MAC Receive Address A Register */
#define MMD_LAN8742_MAC_RECEIVE_ADDR_B		32866		/* MAC Receive Address A Register */
#define MMD_LAN8742_MAC_RECEIVE_ADDR_C		32867		/* MAC Receive Address A Register */

enum {
	SMSC_CABLE_TYPE__DEFAULT = 0,
	SMSC_CABLE_TYPE__SHORTED = 1,
	SMSC_CABLE_TYPE__OPEN = 2,
	SMSC_CABLE_TYPE__MATCH = 3
};

#endif /* __LINUX_SMSCPHY_H__ */
