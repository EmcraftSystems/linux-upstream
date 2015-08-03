/*******************************************************************************

            Copyright (c) 2004 by Nanoradio AB 

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
#ifndef MAC_API_MIB_H
#define MAC_API_MIB_H
#include "mac_api_defs.h"

/**
  @file mac_api_mib.h
  @defgroup mib
  This file defines the MAC API for MIB access.
 */

typedef uint32_t nrx_mib_id_t;

#define NRX_MIB_ID_DOT11_MAC_ADDRESS                             (0) /* 2.1.1.1 */
#define NRX_MIB_ID_DOT11_MAC_PRODUCT_ID                          (1) /* 2.1.1.9 */
#define NRX_MIB_ID_DOT11_MAC_ALL_COUNTERS                        (2) /* 2.2.1 */
#define NRX_MIB_ID_DOT11_MAC_TRANSMITTED_FRAGMENT_COUNT          (3) /* 2.2.1.1 */
#define NRX_MIB_ID_DOT11_MAC_MULTICAST_TRANSMITTED_FRAME_COUNT   (4) /* 2.2.1.2 */
#define NRX_MIB_ID_DOT11_MAC_FAILED_COUNT                        (5) /* 2.2.1.3 */
#define NRX_MIB_ID_DOT11_MAC_RETRY_COUNT                         (6) /* 2.2.1.4 */
#define NRX_MIB_ID_DOT11_MAC_MULTIPLE_RETRY_COUNT                (7) /* 2.2.1.5 */
#define NRX_MIB_ID_DOT11_MAC_FRAME_DUPLICATE_COUNT               (8) /* 2.2.1.6 */
#define NRX_MIB_ID_DOT11_MAC_RTS_SUCCESS_COUNT                   (9) /* 2.2.1.7 */
#define NRX_MIB_ID_DOT11_MAC_RTS_FAILURE_COUNT                   (10) /* 2.2.1.8 */
#define NRX_MIB_ID_DOT11_MAC_ACK_FAILURE_COUNT                   (11) /* 2.2.1.9 */
#define NRX_MIB_ID_DOT11_MAC_RECEIVED_FRAGMENT_COUNT             (12) /* 2.2.1.10 */
#define NRX_MIB_ID_DOT11_MAC_MULTICAST_RECEIVED_FRAME_COUNT      (13) /* 2.2.1.11 */
#define NRX_MIB_ID_DOT11_MAC_FCS_ERROR_COUNT                     (14) /* 2.2.1.12 */
#define NRX_MIB_ID_DOT11_MAC_TRANSMITTED_FRAME_COUNT             (15) /* 2.2.1.13 */
#define NRX_MIB_ID_DOT11_MAC_WEP_UNDECRYPTABLE_COUNT             (16) /* 2.2.1.14 */
#define NRX_MIB_ID_DOT11_RES_MANUFACTURER_PRODUCT_VERSION        (17) /* 3.1.2.1.4 */
#define NRX_MIB_ID_DOT11_PRIVATE_BEACON_LOSS_RATE                (18) /* 5.2.9 */
#define NRX_MIB_ID_DOT11_PRIVATE_PACKET_ERROR_RATE               (19) /* 5.2.10 */
#define NRX_MIB_ID_DOT11_PRIVATE_POWER_INDEX                     (20) /* 5.4. */
#define NRX_MIB_ID_DOT11_PRIVATE_TX_POWER_TX_GAIN_REDUCTION_HT          (21) /* 5.5.4 */
#define NRX_MIB_ID_DOT11_PRIVATE_TX_POWER_RF_PATH_LOSS_MODULE_INTERNAL  (22) /* 5.5.10 */
#define NRX_MIB_ID_DOT11_PRIVATE_TX_POWER_RF_PATH_LOSS_MODULE_EXTERNAL  (23) /* 5.5.11 */
#define NRX_MIB_ID_DOT11_PRIVATE_XCO_CRYSTAL_FREQUENCY_INDEX     (24) /* 5.7.1 */
#define NRX_MIB_ID_DOT11_PRIVATE_ENABLE_EXTERNAL_LFC             (25) /* 5.7.2 */
#define NRX_MIB_ID_DOT11_PRIVATE_HFC_BOOST_XCO_RTRIM             (26) /* 5.7.3 */
#define NRX_MIB_ID_DOT11_PRIVATE_HFC_STARTUP_STABILIZATION_TIME  (27) /* 5.7.4 */
#define NRX_MIB_ID_DOT11_PRIVATE_POWER_CORRECTION_QPSK           (28) /* 5.12.1 */
#define NRX_MIB_ID_DOT11_PRIVATE_POWER_CORRECTION_OFDM           (29) /* 5.12.2 */
#define NRX_MIB_ID_DOT11_PRIVATE_DCDC_THRESHOLD                  (30) /* 5.13 */
#define NRX_MIB_ID_DOT11_PRIVATE_BTCOEX_ENABLED                  (31) /* 5.16.1 */
#define NRX_MIB_ID_DOT11_PRIVATE_BTCOEX_PTA_CONFIG               (32) /* 5.16.2 */
#define NRX_MIB_ID_DOT11_PRIVATE_ANTENNA_CONFIG_MASK             (33) /* 5.18.1 */
#define NRX_MIB_ID_DOT11_PRIVATE_RX_FRAME_FILTER_GROUP_ADDRESS   (34) /* 5.22.1 */
#define NRX_MIB_ID_DOT11_PRIVATE_RX_FRAME_FILTER_UDP_BROADCAST   (35) /* 5.22.2 */
#define NRX_MIB_ID_DOT11_PRIVATE_FIRMWARE_CAPABILITIES           (36) /* 5.23 */
#define NRX_MIB_ID_DOT11_PRIVATE_ARP_FILTER                      (37) /* 5.24.1 */
#define NRX_MIB_ID_DOT11_PRIVATE_AGGREGATION_MAX_SIZE            (38) /* 5.25.1 */
#define NRX_MIB_ID_DOT11_PRIVATE_AGGREGATION_FILTER              (39) /* 5.25.2 */
#define NRX_MIB_ID_DOT11_PRIVATE_AP_COMPATIBILITY_MASK           (40) /* 5.30 */
#define NRX_MIB_ID_DOT11_PRIVATE_EXTERNAL_DCDC                   (41) /* 5.32 */
#define NRX_MIB_ID_DOT11_PRIVATE_TX_ROTATION_ANGLE               (42) /* 5.34 */
#define NRX_MIB_ID_DOT11_PRIVATE_CBSS_FLOW_CONTROL_RED_ZONE_OBJECT_MASK    (43) /* N/A */
#define NRX_MIB_ID_DOT11_PRIVATE_CBSS_FLOW_CONTROL_YELLOW_ZONE_OBJECT_MASK (44) /* N/A */
#define NRX_MIB_ID_DOT11_PRIVATE_BBRX_BUFFER_ZONE_TRANSITION_THRESHOLDS    (45) /* N/A */
#define NRX_MIB_ID_DOT11_PRIVATE_TX_POWER_LIMIT                            (46) /* N/A */
#define NRX_MIB_ID_REGULATION_REGION                                       (47) /* N/A */



#define MIB_MAX_GET_VAL_LENGTH             128

typedef uint32_t nrx_mib_result_t;
#define MIB_RESULT_OK                       (0)
#define MIB_RESULT_INVALID_PATH             (1)
#define MIB_RESULT_NO_SUCH_OBJECT           (2)
#define MIB_RESULT_SIZE_ERROR               (3)
#define MIB_RESULT_OBJECT_NOT_A_LEAF        (4)
#define MIB_RESULT_SET_FAILED               (5)
#define MIB_RESULT_GET_FAILED               (6)
#define MIB_RESULT_SET_NOT_ALLOWED          (7)
#define MIB_RESULT_INTERNAL_ERROR           (8)
#define MIB_RESULT_GET_NOT_ALLOWED          (9)
#define MIB_RESULT_MEM_REGION_INVALIDATED  (10)
#define MIB_RESULT_INVALID_NETIF           (11)
#define MIB_RESULT_TRIGGER_SETUP_FAILED    (12)
#define MIB_RESULT_TRIGGER_NOT_FOUND       (13)


#define MAC_API_SYSTEM_CAPABILITY_SCAN_COUNTRY_INFO_SET 0x00000001

/*---- mac broadcast groups filter "5.22.1" */

#define MAX_NUM_GROUP_ADDRESS_FILTER_ENTRIES 16
typedef struct
{
   uint32_t          discard;
   uint32_t          size;
   nrx_mac_addr_t    mac[MAX_NUM_GROUP_ADDRESS_FILTER_ENTRIES];
}group_addr_filter_t;


/*---- udp broadcast groups filter "5.22.2" */

typedef uint32_t udp_broadcast_filter_flag_t;
#define UDP_BROADCAST_FILTER_FLAG_BOOTP                    0x01
#define UDP_BROADCAST_FILTER_FLAG_SSDP                     0x02
#define UDP_BROADCAST_FILTER_FLAG_NETBIOS_NAME_SERVICE     0x04
#define UDP_BROADCAST_FILTER_FLAG_NETBIOS_DATAGRAM_SERVICE 0x08
#define UDP_BROADCAST_FILTER_FLAG_NETBIOS_SESSION_SERVICE  0x10


/*---- Arp filter/policy  "5.24.1-2" */

#define ARP_MAX_NUM_IP_ADDRESS_FILTER_ENTRIES 4
typedef uint32_t arp_policy_t;
#define ARP_HANDLE_NONE_FORWARD_ALL  0 /* Forward all arp requests to host.         */
#define ARP_HANDLE_NONE_FORWARD_MYIP 1 /* Only forward my ip to host.               */
#define ARP_HANDLE_MYIP_FORWARD_REST 2 /* FW handles my ip, forwards rest to host.  */
#define ARP_HANDLE_MYIP_FORWARD_NONE 3 /* Nothing is forwarded to host.             */

typedef struct
{
   arp_policy_t arp_policy;
   uint32_t     count;
   uint32_t     ip[ARP_MAX_NUM_IP_ADDRESS_FILTER_ENTRIES];
}arp_addr_filter_t;

/*---- CCX Statistics "5.37" */

typedef struct
{
   uint32_t    pkt_xmit_delay_total;
   uint32_t    pkt_xmitted_cnt;
}ccx_stats_t;

/**
  @defgroup NRX_MIB_GET_REQ
  @ingroup mib
  @{
 */

/** Message ID for requesting to get a specific MIB. */
#define NRX_MIB_GET_REQ                  (0  | MAC_API_PRIMITIVE_TYPE_REQ)

/** Message ID for the confirmation message of a NRX_MIB_GET_REQ. */
#define NRX_MIB_GET_CFM                  (0  | MAC_API_PRIMITIVE_TYPE_CFM)

/** Message structure for a NRX_MIB_GET_REQ */
typedef struct
{
   /** Identifier. The identifier of the MIB to get. */ 
   nrx_mib_id_t      id;
}nrx_mib_get_req_t;

typedef struct
{
   nrx_mib_result_t  result;

   /** Identifier. The identifier of the MIB that has been gotten. */ 
   nrx_mib_id_t      id;

   /** Size. The size of the MIB that follows immediately afterwards in the message. */
   uint32_t          size;
}nrx_mib_get_cfm_t;

#define NRX_MIB_GET_CFM_FIXED_PART (4 + 4 + 4)

/** @} */


/**
  @defgroup NRX_MIB_SET_REQ
  @ingroup mib
  @{
 */

/** Message ID for requesting to set a specific MIB. */
#define NRX_MIB_SET_REQ                  (1  | MAC_API_PRIMITIVE_TYPE_REQ)

/** Message ID for the confirmation message of a NRX_MIB_SET_REQ. */
#define NRX_MIB_SET_CFM                  (1  | MAC_API_PRIMITIVE_TYPE_CFM)

/** Message structure for a NRX_MIB_SET_REQ */
typedef struct
{
   /** Identifier. The identifier of the MIB to set. */ 
   nrx_mib_id_t      id;

   /** Size. The size of the MIB setting that follows immediately afterwards in the message. */
   uint32_t          size;
}nrx_mib_set_req_t;

typedef struct
{
   nrx_mib_result_t  result;
   nrx_mib_id_t      id;
}nrx_mib_set_cfm_t;

/** @} */


/**
  @defgroup NRX_MIB_SET_TRIGGER_REQ
  @ingroup mib
  @{
 */

#define NRX_MIB_SET_TRIGGER_REQ          (2  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MIB_SET_TRIGGER_CFM          (2  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef uint8_t mib_trigger_mode_t;
#define MIB_TRIG_MODE_ONESHOT  0
#define MIB_TRIG_MODE_CONT     1
#define MIB_TRIG_MODE_SILENT   2

/* bitmask */
typedef uint8_t mib_trigger_event_t;
#define MIB_TRIG_EVENT_RISING    1
#define MIB_TRIG_EVENT_FALLING   2
#define MIB_TRIG_EVENT_MATCHING  4

/* MIB trigger messages */
typedef struct
{
   nrx_mib_id_t         id;
   uint32_t             trigger_id;
   uint32_t             supv_interval;
   uint32_t             level;
   mib_trigger_event_t  event;
   mib_trigger_mode_t   trig_mode;
   uint16_t             event_count;
}nrx_mib_set_trigger_req_t;

typedef struct
{
   nrx_mib_result_t  result;
   uint32_t          trigger_id;
}nrx_mib_set_trigger_cfm_t;

/** @} */


/**
  @defgroup NRX_MIB_REMOVE_TRIGGER_REQ
  @ingroup mib
  @{
 */

#define NRX_MIB_REMOVE_TRIGGER_REQ       (3  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MIB_REMOVE_TRIGGER_CFM       (3  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t          trigger_id;
}nrx_mib_remove_trigger_req_t;

typedef struct
{
   nrx_mib_result_t  result;
   uint32_t          trigger_id;
}nrx_mib_remove_trigger_cfm_t;

/** @} */


/**
  @defgroup NRX_MIB_TRIGGER_IND
  @ingroup mib
  @{
 */

#define NRX_MIB_TRIGGER_IND              (0  | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   uint32_t          trigger_id;
   uint32_t          varsize;
   int32_t           value;
}nrx_mib_trigger_ind_t;

/** @} */


#endif /* MAC_API_MIB_H */
