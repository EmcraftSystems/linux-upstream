/*******************************************************************************

            Copyright (c) 2010 by Nanoradio AB

This software is copyrighted by and is the sole property of Nanoradio AB.
 All rights, title, ownership, or other interests in the
software remain the property of Nanoradio AB.  This software may
only be used in accordance with the corresponding license agreement.  Any
unauthorized use, duplication, transmission, distribution, or disclosure of
this software is expressly forbidden.

This Copyright notice may not be removed or modified without prior written
consent of Nanoradio AB.

Nanoradio AB reserves the right to modify this software without notice.

Nanoradio AB
Torshamnsgatan 39                  info@nanoradio.se
164 40 Kista                       http://www.nanoradio.com
SWEDEN
*******************************************************************************/
#ifndef MAC_API_BTHCI_H
#define MAC_API_BTHCI_H
#include "mac_api_defs.h"

/* E X P O R T E D  D E F I N E S ********************************************/

/******************************************************************************/
/* DRIVER TO MAC MESSAGE ID's                                                 */
/******************************************************************************/
#define HIC_MAC_BTHCI_REQ                 (0 | MAC_API_PRIMITIVE_TYPE_REQ)

/******************************************************************************/
/* MAC TO DRIVER MESSAGE ID's                                                 */
/******************************************************************************/
#define HIC_MAC_BTHCI_CFM                 (0 | MAC_API_PRIMITIVE_TYPE_CFM)
#define HIC_MAC_BTHCI_IND                 (0 | MAC_API_PRIMITIVE_TYPE_IND)

/* Muxing of HIC_MESSAGE_TYPE_BTHCI by setting "indicator" to these values */
#define BT_HCI_PACKET_CMD      1
#define BT_HCI_PACKET_ACL_DATA 2
#define BT_HCI_PACKET_SCO_DATA 3
#define BT_HCI_PACKET_EVENT    4
#define BT_HCI_PACKET_CONSOLE  5  /* Temporary clonsole for test of bt_host unit */

/* Layout of HIC_MAC_BTHCI_REQ. Transparently holds any downlink HCI */
typedef struct
{
  uint8_t indicator;
  uint8_t parameter[1];
}hic_mac_btchi_req_t;

/* Layout of HIC_MAC_BTHCI_IND. Transparently holds any uplink HCI */
typedef struct
{
  uint8_t indicator;
  uint8_t parameter[1];
}hic_mac_bthci_ind_t;

/*=================================================================*/

/* More detailed layout of HIC_MAC_BT_DATA_REQ when indicator==BT_HCI_PACKET_CMD */
typedef struct
{
  uint8_t indicator;
  uint8_t opcode[2];
  uint8_t length;
  uint8_t parameter[256]; /* Variable length, declared this way for static allocated variable */
}hic_mac_bt_hci_command_t;

/* More detailed layout of HIC_MAC_BT_DATA_IND when indicator==BT_HCI_PACKET_EVENT */
typedef struct
{
  uint8_t indicator;
  uint8_t evcode;
  uint8_t length;
  uint8_t parameter[1];
}hic_mac_bt_hci_event_t;

/* More detailed layout of HIC_MAC_BT_DATA_REQ or HIC_MAC_BT_DATA_IND when indicator==BT_HCI_PACKET_ACL_DATA */
typedef struct
{
  uint8_t indicator;
  uint8_t handle[2];
  uint8_t length[2];
  uint8_t data[1];
}hic_mac_bt_hci_acl_data_t;

/* More detailed layout of HIC_MAC_BT_DATA_REQ or HIC_MAC_BT_DATA_IND when indicator==BT_HCI_PACKET_SCO_DATA */
typedef struct {
  uint8_t indicator;
  uint8_t handle[2];
  uint8_t length;
  uint8_t data[1];
}hic_mac_bt_hci_sco_data_t;

/* More detailed layout of HIC_MAC_BT_DATA_REQ or HIC_MAC_BT_DATA_IND when indicator==BT_HCI_PACKET_CONSOLE */
/* Values for hic_mac_bt_hci_console_t.command */
#define CMD_L2CAP_REG      1
#define CMD_L2CAP_CC       2
#define CMD_L2CAP_SEND     3
#define CMD_L2CAP_DISC     4
#define CMD_L2CAP_CONN_CB  128
#define CMD_L2CAP_DISC_CB  129
#define CMD_L2CAP_DATA_CB  130
#define CMD_L2CAP_FLOW_CB  131

typedef struct {
  uint8_t indicator;
  uint8_t command;
  uint8_t length;
  uint8_t data[1];
}hic_mac_bt_hci_console_t;

#endif    /* MAC_API_BTHCI_H */
