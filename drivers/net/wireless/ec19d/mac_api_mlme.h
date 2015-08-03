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
#ifndef MAC_API_MLME_H
#define MAC_API_MLME_H
#include "mac_api_defs.h"

/**
  @file mac_api_mlme.h
  @defgroup mlme
  This file defines the MAC management API.
 */

/**
  @defgroup nrx_mlme_net_create_req
  @ingroup mlme
  @{
 */

#define NRX_MLME_NET_CREATE_REQ          (0  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_NET_CREATE_CFM          (0  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   nrx_mac_addr_t       mac_address;   /* Device mac address to be used in this specific network.  */
   nrx_net_type_t       net_type;      /* Type of network that will be initiated or connected to.  */
   uint8_t              reserved;
}nrx_mlme_net_create_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_net_create_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_net_destroy
  @ingroup mlme
  @{
 */

#define NRX_MLME_NET_DESTROY_REQ         (1  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_NET_DESTROY_CFM         (1  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint8_t              reserved[4];
}nrx_mlme_net_destroy_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_net_destroy_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_start
  @ingroup mlme
  @{
 */

#define NRX_MLME_START_REQ               (2  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_START_CFM               (2  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t             reserved;
}nrx_mlme_start_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_start_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_stop
  @ingroup mlme
  @{
 */

#define NRX_MLME_STOP_REQ                (3  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_STOP_CFM                (3  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t             reserved;
}nrx_mlme_stop_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_stop_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_power_mgmt
  @ingroup mlme
  @{
 */

#define NRX_MLME_POWER_MGMT_REQ          (4  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_POWER_MGMT_CFM          (4  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint8_t           power_mgmt_mode;           /* 0 = Active, !0 = Power save */
   uint8_t           receive_all_dtim;          /* 1 or 0 */
   uint8_t           periodic_service_interval; /* 1 or 0. If zero, only TIM element info will trig a service session */
   uint8_t           uapsd;                     /* Bit mask encoded for the different AC's */
   nrx_time_us32_t   traffic_timeout;           /* time to remain active upon observed traffic. If zero, PSPOLL is used for ucast traffic */
   nrx_time_us32_t   fixed_service_interval;    /* If zero, service interval will be adaptive (N.B This field is only valid when periodic_service_interval set to 1) */
   uint16_t          listen_interval;           /* beacon listen interval */
   uint8_t           reserved[2];
}nrx_mlme_power_mgmt_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_power_mgmt_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_set_key
  @ingroup mlme
  @{

  Several messages can be aggregated within the same primitive,
  i.e. several key setttings are possible to do in one chunk
 */


#define NRX_MLME_SET_KEY_REQ             (5  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_SET_KEY_CFM             (5  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   nrx_key_t               key;
   uint16_t                key_len;                      /* Length in number of bytes */
   uint8_t                 key_id;
   nrx_key_type_t          key_type;
   nrx_mac_addr_t          mac_addr;
   uint8_t                 receive_seq_cnt[16];          /* WPA uses first 8 */
   uint8_t                 config_by_authenticator;      /* boolean type */
   nrx_cipher_suite_t      cipher_suite;
   uint8_t                 reserved[4];
}nrx_mlme_key_descriptor_t;

typedef struct
{
   nrx_mlme_key_descriptor_t descr[1]; /* variable count */
}nrx_mlme_set_key_req_t;

typedef struct
{
   uint32_t          result;
}nrx_mlme_set_key_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_update_tim
  @ingroup mlme
  @{
 */


#define NRX_MLME_UPDATE_TIM_REQ          (6 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_UPDATE_TIM_CFM          (6 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint8_t bitmap_control;
   uint8_t bitmap[4];                     /* allows up to 31 stations per AP */
   uint8_t reserved[3];
}nrx_mlme_update_tim_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_update_tim_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_set_bss_ie
  @ingroup mlme
  @{
 */


#define NRX_MLME_SET_BSS_IE_REQ          (7 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_SET_BSS_IE_CFM          (7 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef uint32_t mac_api_bss_ie_type_t;
#define MAC_API_BSS_IE_TYPE_COMMON        1
#define MAC_API_BSS_IE_TYPE_BEACON        2
#define MAC_API_BSS_IE_TYPE_PROBE_RSP     3
#define MAC_API_BSS_IE_TYPE_P2P_PROBE_RSP 4
#define MAC_API_BSS_IE_TYPE_SSID          5

typedef struct
{
   mac_api_bss_ie_type_t ie_type;

   /* IEs follow */
   LIST_OF_INFORMATION_ELEMENTS(ie_list)
}nrx_mlme_set_bss_ie_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_set_bss_ie_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_set_sta_info
  @ingroup mlme
  @{
 */

#define NRX_MLME_SET_STA_INFO_REQ        (8 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_SET_STA_INFO_CFM        (8 | MAC_API_PRIMITIVE_TYPE_CFM)

#define NRX_BSS_DEFAULT_RETRANS_TABLE_NUM_ENTRIES 3

typedef struct
{
   mac_retrans_t retrans_table[NRX_BSS_DEFAULT_RETRANS_TABLE_NUM_ENTRIES];
   mac_retrans_t retrans_protect_table[NRX_BSS_DEFAULT_RETRANS_TABLE_NUM_ENTRIES];
}nrx_default_retrans_tables_t;

typedef struct
{
   nrx_mac_rssi_t retrans_rssi_thresholds[NRX_BSS_DEFAULT_RETRANS_TABLE_NUM_ENTRIES-1];
}nrx_default_retrans_thresholds_t;

typedef struct
{
   nrx_default_retrans_tables_t     tables;
   nrx_default_retrans_thresholds_t thresholds;
}nrx_default_retrans_t;


/* for use in tid_to_ac_map */
typedef uint8_t nrx_aci_t;
#define NRX_ACI_AC_BE  0
#define NRX_ACI_AC_BK  1
#define NRX_ACI_AC_VI  2
#define NRX_ACI_AC_VO  3

#define MAC_API_STA_INFO_INVALID_AID   0
typedef struct
{
   nrx_mac_addr_t       peer_sta;
   uint16_t             aid;
   uint16_t             ampdu_max_size;               /* In octets. Derived from Maximum A-MPDU Length Exponent (802.11 std section 7.3.2.56.3) */
   uint8_t              ampdu_min_mpdu_start_spacing; /* Encoded 0..7 according to 802.11 std section 7.3.2.56.3 */
   uint8_t              qos_flag;                     /* '!0' = QoS association, '0'= Not QoS association */
   uint8_t              reserved[1];
   uint8_t              uapsd_delivery_enabled_ac_mask;  /* Bitmask where bit[n] corresponds to ACI[n]. E.g AC_BE <==> bit[0] */
   uint8_t              uapsd_trigger_enabled_ac_mask;   /* See above */
   uint8_t              uapsd_max_sp_len_code;
   nrx_default_retrans_t oretrans;
   nrx_default_retrans_t bretrans;
   nrx_mac_rssi_t       rssi_trigger_rising;
   nrx_mac_rssi_t       rssi_trigger_falling;
   nrx_aci_t            tid_to_ac_map[16];
   /* TLV follow */
}nrx_mlme_set_sta_info_req_t;

typedef struct
{
   nrx_macapi_status_t result;
   nrx_mac_addr_t      peer_sta;
   uint8_t             reserved[2];
}nrx_mlme_set_sta_info_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_delete_sta
  @ingroup mlme
  @{
 */

#define NRX_MLME_DELETE_STA_REQ          (9 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_DELETE_STA_CFM          (9 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   nrx_mac_addr_t       peer_sta;
   uint8_t              reserved[2];
}nrx_mlme_delete_sta_req_t;

typedef struct
{
   nrx_macapi_status_t result;
}nrx_mlme_delete_sta_cfm_t;

/** @} */


/**
  @defgroup NRX_MLME_SET_BSSCONF_REQ
  @ingroup mlme
  @{
 */

#define NRX_MLME_SET_BSSCONF_REQ         (10 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_SET_BSSCONF_CFM         (10 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   mac_rate_t mod_qpsk[MAC_NO_OF_SUBRATES_QPSK];
   mac_rate_t mod_ofdm[MAC_NO_OF_SUBRATES_OFDM];
   mac_rate_t mod_ht[MAC_NO_OF_SUBRATES_HT];
   mac_rate_t mod_ht40[MAC_NO_OF_SUBRATES_HT40];
} nrx_mac_ctrlrsp_rate_set_t;

typedef uint8_t nrx_protection_flags_t;
#define NRX_PROTECTION_FORCE_LONG_PREAMBLE       (1<<0)
#define NRX_PROTECTION_FORCE_PROTECTION          (1<<1)

typedef struct
{
   nrx_mac_addr_t             bssid;
   uint16_t                   capability_info;
   mac_chn_prop_t             chn_prop;
   uint32_t                   p2p_flags;
#define NRX_P2P_FLAGS_ENABLED               (1<<0)
#define NRX_P2P_FLAGS_PS_TRACK_NOA          (1<<1)
#define NRX_P2P_FLAGS_PS_UPDATES_TO_DRV     (1<<2)
   nrx_mac_ctrlrsp_rate_set_t ctrlrsp_rate_set;
   mac_retrans_t              beacon_retrans; /* The tries field is don't care */
   nrx_time_tu16_t            beacon_period;
   /** IBSS only */
   uint16_t                   atim_window;
   /** Master only */
   uint8_t                    hidden_ssid;
   uint8_t                    dtim_period;
   nrx_protection_flags_t     protection_flags;
   uint8_t                    reserved;
} nrx_bss_conf_t;

typedef struct
{
   nrx_bss_conf_t    conf;
   /* TLV may follow */
} nrx_mlme_set_bssconf_req_t;

typedef struct
{
   nrx_macapi_status_t result;
} nrx_mlme_set_bssconf_cfm_t;

/** @} */


/**
  @defgroup NRX_MLME_GET_BSSCONF_REQ
  @ingroup mlme
  @{
 */

#define NRX_MLME_GET_BSSCONF_REQ         (11 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_GET_BSSCONF_CFM         (11 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t reserved;
} nrx_mlme_get_bssconf_req_t;

typedef struct
{
   nrx_bss_conf_t    conf;
} nrx_mlme_get_bssconf_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_config_tx
  @ingroup mlme
  @{
 */


#define NRX_MLME_CONFIG_TX_REQ           (12 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_CONFIG_TX_CFM           (12 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint16_t txop_limit;
   uint8_t  ecw_min;     /* 0..15 */
   uint8_t  ecw_max;     /* 0..15 */
   uint8_t  aifsn;       /* 0..15 */
   uint8_t  reserved[3];
} nrx_mlme_tx_config_t;

typedef uint8_t nrx_edca_queue_t;
#define NRX_EDCA_BACKGROUND_QUEUE  (0)  /* Lowest Priority */
#define NRX_EDCA_BEST_EFFORT_QUEUE (1)
#define NRX_EDCA_VIDEO_QUEUE       (2)
#define NRX_EDCA_VOICE_QUEUE       (3)  /* Highest Priority */

typedef struct
{
   uint32_t              num_queues;
   nrx_mlme_tx_config_t  queue[4]; /* index according to nrx_edca_queue_t */
} nrx_mlme_config_tx_req_t;

typedef struct
{
   nrx_macapi_status_t status;
} nrx_mlme_config_tx_cfm_t;

/** @} */


/**
  @defgroup NRX_MLME_CRITICAL_SESSION_REQ
  @ingroup mlme
  @{
 */

#define NRX_MLME_CRITICAL_SESSION_REQ    (13 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_CRITICAL_SESSION_CFM    (13 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef uint32_t nrx_critical_session_flag_t;
#define CRITICAL_SESSION_LISTEN         (1<<0)
#define CRITICAL_SESSION_AUTHENTICATING (1<<1)
#define CRITICAL_SESSION_ASSOCIATING    (1<<2)

typedef struct
{
   nrx_critical_session_flag_t flags;
} nrx_mlme_critical_session_req_t;

typedef struct
{
   nrx_macapi_status_t result;
} nrx_mlme_critical_session_cfm_t;

/** @} */


/**
  @defgroup NRX_MLME_ADD_BAA_REQ
  @ingroup mlme
  @{
 */

#define NRX_MLME_ADD_BAA_REQ             (14 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_ADD_BAA_CFM             (14 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   nrx_mac_addr_t    peer_sta;
   uint16_t          win_start;
   uint8_t           tid;
   uint8_t           reserved[3];
} nrx_mlme_add_baa_req_t;

typedef struct
{
   nrx_macapi_status_t result;
} nrx_mlme_add_baa_cfm_t;

/** @} */


/**
  @defgroup NRX_MLME_DEL_BAA_REQ
  @ingroup mlme
  @{
 */

#define NRX_MLME_DEL_BAA_REQ             (15 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_DEL_BAA_CFM             (15 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   nrx_mac_addr_t    peer_sta;
   uint8_t           tid;
   uint8_t           reserved;
} nrx_mlme_del_baa_req_t;

typedef struct
{
   nrx_macapi_status_t result;
} nrx_mlme_del_baa_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_p2p_ps_config family
  @ingroup mlme
  @{
 */

#define NRX_MLME_P2P_PS_SET_CONF_REQ     (16 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_P2P_PS_SET_CONF_CFM     (16 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   /** Bitfield indicating what will be updated. Is set to zero at nrx_mlme_p2p_ps_get_conf_cfm_t. */
   uint8_t              type;
#define NRX_MLME_P2P_PS_CONF_NOA_1        (1<<0)
#define NRX_MLME_P2P_PS_CONF_NOA_2        (1<<1)
#define NRX_MLME_P2P_PS_CONF_OPP          (1<<2)
#define NRX_MLME_P2P_PS_CONF_RESERVED_0   (1<<3)
#define NRX_MLME_P2P_PS_CONF_IND_GO_AWAKE (1<<4)
   uint8_t              reserved;
   nrx_p2p_ps_opp_t     opp;
   nrx_p2p_ps_noa_t     noa[2];
} nrx_mlme_p2p_ps_conf_t;

typedef struct
{
   nrx_mlme_p2p_ps_conf_t conf;
} nrx_mlme_p2p_ps_set_conf_req_t;

typedef struct
{
   uint8_t             status;
   uint8_t             type;   /* bitfield indicating what was updated */
   uint8_t             reserved[2];
   uint32_t            starttime[2];
} nrx_mlme_p2p_ps_set_conf_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_p2p_ps_config family
  @ingroup mlme
  @{
 */

#define NRX_MLME_P2P_PS_GET_CONF_REQ     (17 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_P2P_PS_GET_CONF_CFM     (17 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t reserved;
} nrx_mlme_p2p_ps_get_conf_req_t;

typedef struct
{
   nrx_mlme_p2p_ps_conf_t conf;
} nrx_mlme_p2p_ps_get_conf_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_mac_counters
  @ingroup mlme
  @{
 */

#define NRX_MLME_GET_COUNTERS_REQ     (18 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_GET_COUNTERS_CFM     (18 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t hic_uplink_data_discarded;
   uint32_t hic_uplink_low_prio_discarded;
   uint32_t macul_rx;
   uint32_t macul_rx_discarded;
   uint32_t macul_tx;
   uint32_t macul_tx_enqueued;
   uint32_t missed_beacons;
} nrx_mlme_mac_counters_t;

typedef struct
{
   uint32_t reserved; /* Must be zero */
} nrx_mlme_get_counters_req_t;

typedef struct
{
   dot11CountersEntry_t    dot11_counters;
   nrx_mlme_mac_counters_t mac_counters;
} nrx_mlme_get_counters_cfm_t;

/** @} */


/**
  @defgroup set_beacon_ie_filter
  @ingroup mlme
  @{
 */

#define NRX_MLME_SET_BEACON_IE_FILTER_REQ (19 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_SET_BEACON_IE_FILTER_CFM (19 | MAC_API_PRIMITIVE_TYPE_CFM)

#define IE_FILTER_INDEX(_id)     ((_id)>>5)     /* = id/32 */
#define IE_FILTER_BIT(_id)   (1<<((_id)&0x1f))  /* = 1<<(id%32) */

#define IE_BEACON_NUM_OF_STD_FILTERS      4
#define IE_BEACON_NUM_OF_VENDOR_FILTERS   16
#define IE_BEACON_SIZE_OF_VENDOR_PATTERN  7

typedef struct
{
   uint8_t  pattern_len;
   uint8_t  pattern[IE_BEACON_SIZE_OF_VENDOR_PATTERN];
}nrx_ie_vendor_filter_t;

typedef struct
{
   /** Each IE id is represented by one bit. If an IE (with its bit set) has been changed,
    *  bss synch will send the beacon up. Only IEs with ID 0 to 127 is represented. */
   uint32_t               ie_std[IE_BEACON_NUM_OF_STD_FILTERS];
   uint32_t               num_ie_vendor;
   /** Special handling of vendor specific IEs. If pattern matches, firmware will
    *  send the beacon up if this IE has changed. */
   nrx_ie_vendor_filter_t ie_vendor[IE_BEACON_NUM_OF_VENDOR_FILTERS];
} nrx_set_beacon_ie_vendor_filter_req_t;

typedef struct
{
   uint32_t   status;
}nrx_set_beacon_ie_vendor_filter_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_tx_statistics
  @ingroup mlme
  @{
 */

#define NRX_MLME_TX_STATISTICS_REQ (20 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_TX_STATISTICS_CFM (20 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t  trig_level;
   uint32_t  success;
   uint32_t  fails_per_rate[NRX_BSS_DEFAULT_RETRANS_TABLE_NUM_ENTRIES];
} nrx_tx_statistics_t;

typedef struct
{
   nrx_mac_addr_t       peer_sta;      /* Peer station to get statistics from.    */
   uint16_t             reserved;
   /** When the threshold is reached (>=) one NRX_MLME_TX_STATISTICS_IND will be sent with collected statistics. */
   uint32_t             trig_level;    /* Thresold for failures on original rate. */
}nrx_mlme_tx_statistics_req_t;


typedef struct
{
   uint32_t   status;
}nrx_mlme_tx_statistics_cfm_t;

/** @} */

/**
  @defgroup nrx_mlme_get_sta_statistics
  @ingroup mlme
  @{
 */

#define NRX_MLME_GET_STA_STATISTICS_REQ (21 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_MLME_GET_STA_STATISTICS_CFM (21 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   nrx_mac_addr_t       peer_sta;      /* Station to get statistics for.    */
   uint16_t             reserved;
}nrx_mlme_get_sta_statistics_req_t;


typedef struct
{
   nrx_macapi_status_t  status;
   /* TLV data follow */
}nrx_mlme_get_sta_statistics_cfm_t;

/** @} */


/**
  @defgroup nrx_mlme_p2p_ps_config family
  @ingroup mlme
  @{
 */
#define NRX_MLME_P2P_PS_NET_IND     (0 | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   uint8_t  reserved_1;
   uint8_t  go_asleep;
   uint16_t reserved_2;
} nrx_mlme_p2p_ps_net_ind_t;

/** @} */


/**
  @defgroup NRX_MLME_BAA_EXCEPTION_IND
  @ingroup mlme
  @{
 */

#define NRX_MLME_BAA_EXCEPTION_IND       (1 | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   uint32_t          reason; /* M80211_MGMT_REASON_MISSING_SETUP, M80211_MGMT_REASON_TIMEOUT */
   nrx_mac_addr_t    peer_sta;
   uint8_t           tid;
   uint8_t           reserved;
} nrx_mlme_baa_exception_ind_t;

/** @} */


/**
  @defgroup nrx_mlme_peer_status
  @ingroup mlme
  @{
 */

#define NRX_MLME_PEER_STATUS_IND         (2 | MAC_API_PRIMITIVE_TYPE_IND)

typedef uint32_t nrx_mlme_peer_status_t;
#define MLME_PEER_STATUS_CONNECTED                         (0x00)
#define MLME_PEER_STATUS_NOT_CONNECTED                     (0x01)
#define MLME_PEER_STATUS_TX_FAILED                         (0x02)
#define MLME_PEER_STATUS_RX_BEACON_FAILED                  (0x03)
#define MLME_PEER_STATUS_ROUNDTRIP_FAILED                  (0x04)
#define MLME_PEER_STATUS_RESTARTED                         (0x05)
#define MLME_PEER_STATUS_INCOMPATIBLE                      (0x06)
#define MLME_PEER_STATUS_RESERVED_07                       (0x07)
#define MLME_PEER_STATUS_RESERVED_08                       (0x08)
#define MLME_PEER_STATUS_BROADCAST_REQ                     (0x09)
#define MLME_PEER_STATUS_PS_FINISHED                       (0x0A)
#define MLME_PEER_STATUS_PS_POLL                           (0x0B)
#define MLME_PEER_STATUS_WMM_POLL                          (0x0C)
#define MLME_PEER_STATUS_TX_FAILED_WARNING                 (0x82)
#define MLME_PEER_STATUS_RX_BEACON_FAILED_WARNING          (0x83)

#define MLME_PEER_STATUS_RSSI_TRIGGER_RISING               (0x90)
#define MLME_PEER_STATUS_RSSI_TRIGGER_FALLING              (0x91)

typedef struct
{
   nrx_mlme_peer_status_t  status;
   nrx_mac_addr_t          peer_mac;
   nrx_mac_addr_t          bssid;
   /* TLV data may follow */
}nrx_mlme_peer_status_ind_t;

/** @} */

/**
  @defgroup nrx_mlme_tx_statistics_ind
  @ingroup mlme
  @{
 */

#define NRX_MLME_TX_STATISTICS_IND       (3 | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   nrx_mac_addr_t       peer_sta;
   uint16_t             reserved;
   nrx_tx_statistics_t  stats;
}nrx_mlme_tx_statistics_ind_t;

/** @} */


#endif    /* MAC_API_MLME_H */
