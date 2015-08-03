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
#ifndef MAC_API_DEFS_H
#define MAC_API_DEFS_H

/*!
  \file mac_api_defs.h

  \brief This header file contains common definitions for the MAC-API.

  \defgroup defs Common definitions
 */

/* E X P O R T E D  D E F I N E S ********************************************/

typedef uint32_t nrx_time_us32_t; /* 32 bit unsigned Time duration expressed in microseconds */
typedef uint32_t nrx_time_ms32_t; /* 32 bit unsigned Time duration expressed in milliseconds */
typedef uint16_t nrx_time_tu16_t; /* 16 bit unsigned Time duration expressed in "Time Units" as defined by 80211 std (1 TU = 1024 microseconds) */

typedef struct 
{
   char octet[6];
} nrx_mac_addr_t;

typedef struct
{
   /*
    * WPA:
    * WPA2:
    *   uint8_t tk[16];
    *   uint8_t mic_tx[8];
    *   uint8_t mic_rx[8];
    * WAPI:
    *   ?
    */ 
   uint8_t part[32];
} nrx_key_t;

typedef uint8_t nrx_key_type_t;
#define NRX_KEY_TYPE_GROUP    0
#define NRX_KEY_TYPE_PAIRWISE 1
#define NRX_KEY_TYPE_STA      2
#define NRX_KEY_TYPE_IGTK     3
#define NRX_KEY_TYPE_ALL      0xAA

typedef uint8_t nrx_cipher_suite_t;
#define NRX_CIPHER_SUITE_GROUP    0
#define NRX_CIPHER_SUITE_WEP40    1
#define NRX_CIPHER_SUITE_TKIP     2
#define NRX_CIPHER_SUITE_WPI      3
#define NRX_CIPHER_SUITE_CCMP     4
#define NRX_CIPHER_SUITE_WEP104   5
#define NRX_CIPHER_SUITE_BIP      6 /* IEEE 802.11w */
#define NRX_CIPHER_SUITE_WEP      7 /* Combining WEP40 and WEP104 to one */
#define NRX_CIPHER_SUITE_NONE     0xFF /* Pure accelerator, may not be used as index */

/* wifi direct NoA PS */
typedef struct
{
    uint32_t    duration;
    uint32_t    interval;
    uint32_t    starttime;
    uint8_t     count;
    uint8_t     reserved[3];
}nrx_p2p_ps_noa_t;

/* wifi direct Opp PS */
typedef struct
{
    uint8_t     enabled;
    uint8_t     ctwindow;
}nrx_p2p_ps_opp_t;

typedef struct
{
   uint32_t dot11TransmittedFragmentCount;
   uint32_t dot11MulticastTransmittedFrameCount;
   uint32_t dot11FailedCount;
   uint32_t dot11RetryCount;
   uint32_t dot11MultipleRetryCount;
   uint32_t dot11FrameDuplicateCount;
   uint32_t dot11RTSSuccessCount;
   uint32_t dot11RTSFailureCount;
   uint32_t dot11ACKFailureCount;
   uint32_t dot11ReceivedFragmentCount;
   uint32_t dot11MulticastReceivedFrameCount;
   uint32_t dot11FCSErrorCount;
   uint32_t dot11TransmittedFrameCount;
   uint32_t dot11WEPUndecryptableCount;
}dot11CountersEntry_t;

#ifndef LIST_OF_INFORMATION_ELEMENTS
#define LIST_OF_INFORMATION_ELEMENTS(_name)
#endif

#ifndef LIST_OF_TLV
#define LIST_OF_TLV(_name)
#endif


typedef enum
{
   MAC_API_PRIMITIVE_TYPE_REQ = 0x00,
   MAC_API_PRIMITIVE_TYPE_CFM = 0x80,
   MAC_API_PRIMITIVE_TYPE_IND = 0x40,
   MAC_API_PRIMITIVE_TYPE_BIT = 0xC0
} nrx_msg_id_flags_t;

#define MAC_API_PRIMITIVE_IS_REQ(_msg_id)\
   (((_msg_id) & MAC_API_PRIMITIVE_TYPE_BIT) == MAC_API_PRIMITIVE_TYPE_REQ)

#define MAC_API_PRIMITIVE_IS_CFM(_msg_id)\
   (((_msg_id) & MAC_API_PRIMITIVE_TYPE_BIT) == MAC_API_PRIMITIVE_TYPE_CFM)

#define MAC_API_PRIMITIVE_IS_IND(_msg_id)\
   (((_msg_id) & MAC_API_PRIMITIVE_TYPE_IND) == MAC_API_PRIMITIVE_TYPE_IND)

typedef uint8_t nrx_hic_message_type_t;
#define HIC_MESSAGE_TYPE_CTRL         0
#define HIC_MESSAGE_TYPE_FRAME        1
#define HIC_MESSAGE_TYPE_MLME         2
#define HIC_MESSAGE_TYPE_MIB          3
#define HIC_MESSAGE_TYPE_SCAN         4
#define HIC_MESSAGE_TYPE_DLM          5
#define HIC_MESSAGE_TYPE_BTHCI        6
#define HIC_MESSAGE_TYPE_DEBUG        7
#define HIC_MESSAGE_TYPE_NVM          8
#define HIC_MESSAGE_TYPE_AUDIO        9
#define HIC_MESSAGE_TYPE_STW_CTRL    10
#define HIC_MESSAGE_TYPE_STW_IND     11
#define HIC_MESSAGE_TYPE_STW_DEVCONF 12
#define HIC_MESSAGE_TYPE_STW_GPIO    13
#define HIC_MESSAGE_TYPE_STW_CONPROF 14
#define HIC_MESSAGE_TYPE_STW_SOCK    15
#define HIC_MESSAGE_TYPE_STW_NET     16
#define HIC_MESSAGE_TYPE_STW_TEST    17
#define HIC_MESSAGE_TYPE_STW_TEXT    18
#define HIC_MESSAGE_TYPE_EWTE        19
#define HIC_MESSAGE_TYPE_NUM_TYPES   20
#define HIC_MESSAGE_TYPE_AGGREGATION 63

typedef uint8_t nrx_hic_message_id_t;

/**
  Identification of the network when supporting multiple network types.
  
  Valid net id's must be in the range 1-127. Value 0 is reserved to represent
  no-network and bit 7 of the netword id is reserved for furure use.

  @defgroup netid nrx_net_id_t
  @{
 */
typedef uint8_t nrx_net_id_t;

#define NRX_NET_ID_UNKNOWN                 0x00
/** WIFI related message where net_id couldn't be determined */
#define NRX_NET_ID_WIFI_UNKNOWN            0x01
#define NRX_NET_ID_FIRST_DYN               0x08
#define NRX_NET_ID_RESERVED_BIT            0x80

/** @} */

/* MAC Data priority parameter */
typedef uint8_t  nrx_mac_prio_t;

/**
  Definition of the network type.
  
  The two most significant bits are flags indicating this the network is joined
  (NET_ROLE_CLIENT) or created (NET_ROLE_INITIATOR) by us.
 */
typedef uint8_t   nrx_net_type_t;

#define NRX_NET_ROLE_MASTER        ( 0x80 )

#define NRX_NET_TYPE_NO_NET        ( 0x00 )
#define NRX_NET_TYPE_BSS_STA       ( 0x01 )
#define NRX_NET_TYPE_BSS_AP        ( 0x01 | NRX_NET_ROLE_MASTER )
#define NRX_NET_TYPE_BT_AMP        ( 0x02 )
#define NRX_NET_TYPE_IBSS          ( 0x04 | NRX_NET_ROLE_MASTER )

typedef uint16_t nrx_hic_frame_size_t;
typedef uint16_t nrx_hic_message_length_t;

typedef uint32_t nrx_macapi_status_t;
#define NRX_SUCCESS        0
#define NRX_FAILED         1
#define NRX_NOT_SUPPORTED  2

/**
  Message control header
  @ingroup defs
 */

typedef uint8_t nrx_hic_flags_t;
#define NRX_HIC_FLAG_LOW_PRIO 0x01

typedef uint16_t nrx_hic_trans_id_t;
#define NRX_TRANS_ID_DEFAULT 0

typedef struct
{
   /** Length of message including this field */
   nrx_hic_message_length_t  length;
   /** Message type. */
   nrx_hic_message_type_t    type;
   /** Type specific message ID */
   nrx_hic_message_id_t      id;
   /** Flags. */
   nrx_hic_flags_t           flags;
   /** Identification of the network (or NRX_NET_ID_UNKNOWN if not applicable) */
   nrx_net_id_t              net_id;
   /** Transaction ID */
   nrx_hic_trans_id_t        trans_id;
   uint8_t                   tx_window_non_wifi_data;
   /**
    * Number of fragments in the tx window.
    *
    * This imposes a limit in on the number (and size) of the messages the host
    * may send to the chip. This field is only valid in messages originating
    * from the chip. In messages to the chip, this field must be zero.
    */
   uint8_t                   tx_window;
} nrx_hic_message_t;

/**
  Message transport header
  @ingroup defs
 */
typedef struct
{
   /** Transport size of the hic frame that follows, i.e. excluding the size field itself */
   nrx_hic_frame_size_t  size;    /* 2 */
   /** HIC message header */
   nrx_hic_message_t     control; /* 10 */
} nrx_hic_frame_t;
#define NRX_HIC_FRAME_HEADER_SIZE       12

/* Definitions common to MLME and SCAN api's */
typedef int32_t nrx_mac_rssi_t;
#define NRX_MAC_RSSI_UNKNOWN           -128
#define NRX_MAC_INVALID_RSSI_SNR_VALUE 0xFFFFFF81

/* ------------ Rates --------------------- */
typedef uint8_t nrx_mac_mod_t;
#define MAC_MOD_QPSK                0
#define MAC_MOD_OFDM                1
#define MAC_MOD_HT                  2
#define MAC_MOD_HT40                3
#define MAC_NO_OF_MODS              4

typedef uint8_t mac_subrate_t;
/* QPSK */
#define MAC_SUBRATE_QPSK_1MBIT      0
#define MAC_SUBRATE_QPSK_2MBIT      1
#define MAC_SUBRATE_QPSK_5_5MBIT    2
#define MAC_SUBRATE_QPSK_11MBIT     3

/* OFDM */
#define MAC_SUBRATE_OFDM_6MBIT      0
#define MAC_SUBRATE_OFDM_9MBIT      1
#define MAC_SUBRATE_OFDM_12MBIT     2
#define MAC_SUBRATE_OFDM_18MBIT     3
#define MAC_SUBRATE_OFDM_24MBIT     4
#define MAC_SUBRATE_OFDM_36MBIT     5
#define MAC_SUBRATE_OFDM_48MBIT     6
#define MAC_SUBRATE_OFDM_54MBIT     7

/* HT 20MHz */
#define NRX_MAC_SUBRATE_HT_MCS_0    0
#define NRX_MAC_SUBRATE_HT_MCS_1    1
#define NRX_MAC_SUBRATE_HT_MCS_2    2
#define NRX_MAC_SUBRATE_HT_MCS_3    3
#define NRX_MAC_SUBRATE_HT_MCS_4    4
#define NRX_MAC_SUBRATE_HT_MCS_5    5
#define NRX_MAC_SUBRATE_HT_MCS_6    6
#define NRX_MAC_SUBRATE_HT_MCS_7    7

/* HT 40MHz */
#define NRX_MAC_SUBRATE_HT40_MCS_0  0
#define NRX_MAC_SUBRATE_HT40_MCS_1  1
#define NRX_MAC_SUBRATE_HT40_MCS_2  2
#define NRX_MAC_SUBRATE_HT40_MCS_3  3
#define NRX_MAC_SUBRATE_HT40_MCS_4  4
#define NRX_MAC_SUBRATE_HT40_MCS_5  5
#define NRX_MAC_SUBRATE_HT40_MCS_6  6
#define NRX_MAC_SUBRATE_HT40_MCS_7  7

#define MAC_NO_OF_SUBRATES_QPSK     4
#define MAC_NO_OF_SUBRATES_OFDM     8
#define MAC_NO_OF_SUBRATES_HT       8
#define MAC_NO_OF_SUBRATES_HT40     8
#define MAC_MAX_NO_OF_SUBRATES      8
#define MAC_MAX_NO_OF_MODULATIONS   4

typedef uint16_t mac_rate_t;
/* QPSK */
#define MAC_RATE_QPSK_1MBIT      0x0000
#define MAC_RATE_QPSK_2MBIT      0x0001
#define MAC_RATE_QPSK_5_5MBIT    0x0002
#define MAC_RATE_QPSK_11MBIT     0x0003

/* OFDM */
#define MAC_RATE_OFDM_6MBIT      0x0100
#define MAC_RATE_OFDM_9MBIT      0x0101
#define MAC_RATE_OFDM_12MBIT     0x0102
#define MAC_RATE_OFDM_18MBIT     0x0103
#define MAC_RATE_OFDM_24MBIT     0x0104
#define MAC_RATE_OFDM_36MBIT     0x0105
#define MAC_RATE_OFDM_48MBIT     0x0106
#define MAC_RATE_OFDM_54MBIT     0x0107

/* HT 20MHz */
#define NRX_MAC_RATE_HT_MCS_0    0x0200
#define NRX_MAC_RATE_HT_MCS_1    0x0201
#define NRX_MAC_RATE_HT_MCS_2    0x0202
#define NRX_MAC_RATE_HT_MCS_3    0x0203
#define NRX_MAC_RATE_HT_MCS_4    0x0204
#define NRX_MAC_RATE_HT_MCS_5    0x0205
#define NRX_MAC_RATE_HT_MCS_6    0x0206
#define NRX_MAC_RATE_HT_MCS_7    0x0207

/* HT 40MHz */
#define NRX_MAC_RATE_HT40_MCS_0  0x0300
#define NRX_MAC_RATE_HT40_MCS_1  0x0301
#define NRX_MAC_RATE_HT40_MCS_2  0x0302
#define NRX_MAC_RATE_HT40_MCS_3  0x0303
#define NRX_MAC_RATE_HT40_MCS_4  0x0304
#define NRX_MAC_RATE_HT40_MCS_5  0x0305
#define NRX_MAC_RATE_HT40_MCS_6  0x0306
#define NRX_MAC_RATE_HT40_MCS_7  0x0307

#define MAC_RATE_INVALID         0xFFFF

#define MAC_SUBRATE_NOT_SUPPORTED ((mac_subrate_t)~0)

#define MAC_QPSK_RATE_BIT_MASK   ((1<<MAC_SUBRATE_QPSK_1MBIT)   |\
                                     (1<<MAC_SUBRATE_QPSK_2MBIT)   |\
                                     (1<<MAC_SUBRATE_QPSK_5_5MBIT) |\
                                     (1<<MAC_SUBRATE_QPSK_11MBIT))

#define MAC_OFDM_RATE_BIT_MASK   ((1<<MAC_SUBRATE_OFDM_6MBIT)   |\
                                     (1<<MAC_SUBRATE_OFDM_9MBIT)   |\
                                     (1<<MAC_SUBRATE_OFDM_12MBIT)  |\
                                     (1<<MAC_SUBRATE_OFDM_18MBIT)  |\
                                     (1<<MAC_SUBRATE_OFDM_24MBIT)  |\
                                     (1<<MAC_SUBRATE_OFDM_36MBIT)  |\
                                     (1<<MAC_SUBRATE_OFDM_48MBIT)  |\
                                     (1<<MAC_SUBRATE_OFDM_54MBIT))

#define MAC_HT_RATE_BIT_MASK     ((1<<NRX_MAC_SUBRATE_HT_MCS_0)   |\
                                     (1<<NRX_MAC_SUBRATE_HT_MCS_1)   |\
                                     (1<<NRX_MAC_SUBRATE_HT_MCS_2)  |\
                                     (1<<NRX_MAC_SUBRATE_HT_MCS_3)  |\
                                     (1<<NRX_MAC_SUBRATE_HT_MCS_4)  |\
                                     (1<<NRX_MAC_SUBRATE_HT_MCS_5)  |\
                                     (1<<NRX_MAC_SUBRATE_HT_MCS_6)  |\
                                     (1<<NRX_MAC_SUBRATE_HT_MCS_7))

#define MAC_HT40_RATE_BIT_MASK   ((1<<NRX_MAC_SUBRATE_HT40_MCS_0)   |\
                                     (1<<NRX_MAC_SUBRATE_HT40_MCS_1)   |\
                                     (1<<NRX_MAC_SUBRATE_HT40_MCS_2)  |\
                                     (1<<NRX_MAC_SUBRATE_HT40_MCS_3)  |\
                                     (1<<NRX_MAC_SUBRATE_HT40_MCS_4)  |\
                                     (1<<NRX_MAC_SUBRATE_HT40_MCS_5)  |\
                                     (1<<NRX_MAC_SUBRATE_HT40_MCS_6)  |\
                                     (1<<NRX_MAC_SUBRATE_HT40_MCS_7))

#define MAC_SUBRATE(_rate)      ((mac_subrate_t)_rate)
#define MAC_MOD(_rate)          ((nrx_mac_mod_t)((_rate)>>8))

typedef uint8_t nrx_mac_rate_flags_t;
#define NRX_MAC_RATE_FLAG_USE_RTS_CTS             (1<<0)
#define NRX_MAC_RATE_FLAG_USE_CTS_TO_SELF         (1<<1)
#define NRX_MAC_RATE_FLAG_SHORT_PREAMBLE          (1<<2)
#define NRX_MAC_RATE_FLAG_HT_SHORT_GI             (1<<3)
#define NRX_MAC_RATE_FLAG_HT_GREENFIELD           (1<<4)
#define NRX_MAC_RATE_FLAG_HT40_DUPL               (1<<5)
/** When using RTS-CTS there will be 2 CTSs, one normal and one with STBC */
#define NRX_MAC_RATE_FLAG_DUAL_CTS                (1<<6)


#define NRX_MAC_RETRANS_SUBRATE(_retrans)            (MAC_SUBRATE((_retrans).attrib.rate))
#define NRX_MAC_RETRANS_MOD(_retrans)                (MAC_MOD((_retrans).attrib.rate))
#define NRX_MAC_RETRANS_IS_RTS_CTS(_retrans)         ((_retrans).attrib.flags & NRX_MAC_RATE_FLAG_USE_RTS_CTS)
#define NRX_MAC_RETRANS_IS_CTS_TO_SELF(_retrans)     ((_retrans).attrib.flags & NRX_MAC_RATE_FLAG_USE_CTS_TO_SELF)
#define NRX_MAC_RETRANS_IS_SHORT_PREAMBLE(_retrans)  ((_retrans).attrib.flags & NRX_MAC_RATE_FLAG_SHORT_PREAMBLE)
#define NRX_MAC_RETRANS_IS_SHORT_GI(_retrans)        ((_retrans).attrib.flags & NRX_MAC_RATE_FLAG_HT_SHORT_GI)
#define NRX_MAC_RETRANS_IS_GREENFIELD(_retrans)      ((_retrans).attrib.flags & NRX_MAC_RATE_FLAG_HT_GREENFIELD)

/* The dummy union declaration below is needed for gcc to treat this as a 32-bit word
 * when returning this type from functions.
 */
typedef union
{
   struct
   {
      mac_rate_t           rate;
      uint8_t              tries;
      nrx_mac_rate_flags_t flags;
   } attrib;
   uint32_t dummy;
} mac_retrans_t;


typedef uint16_t nrx_mac_chn_freq_t;

typedef uint8_t nrx_mac_chn_flag_t;
#define NRX_MAC_CHN_FLAG_HT40_LOWER   0x01
#define NRX_MAC_CHN_FLAG_HT40_UPPER   0x02
#define NRX_MAC_CHN_FLAG_PASSIVE_SCAN 0x04

/** Channel properties */
/* The dummy union declaration below is needed for gcc to treat this as a 32-bit word
 * when returning this type from functions.
 */
typedef union
{
   struct
   {
      /** Freq in MHz */
      nrx_mac_chn_freq_t freq;
      /** Flags */ 
      nrx_mac_chn_flag_t flags; 
      /** Tx power in dBm */ 
      uint8_t            tx_dbm;
   } props;
   uint32_t all;
} mac_chn_prop_t;

/**
  @defgroup tlv_head_t
  @{

  Header (Type-Len) of a TLV (Type-Len-Value)

  The ID is 16bit to support all 802.11 IE's. The NR proprietary ID's start at
  0x0100.
 */
typedef struct 
{
   uint16_t type;
   uint16_t len;
} nrx_tlv_head_t;

/** @} */


/**
  @defgroup NRX_TLV_TYPE_RETRANS
  @{

 Specifies each rate with the number of retransmissions of each rate.
 Can include one or several mac_retrans_t.

 If RTS or CTS is enabled on at least one rate, there must be a second
 table with the protection rates. This table must have the same size
 as the first table.
 */
#define NRX_TLV_TYPE_RETRANS_TABLE              0x0101
#define NRX_TLV_TYPE_RETRANS_PROTECT_TABLE      0x0102
#define NRX_TLV_TYPE_CHN_PROP_LIST              0x0103
#define NRX_TLV_TYPE_SUPPORTED_RATE_SET         0x0104      /* vector of mac_rate_t */
#define NRX_TLV_TYPE_BASIC_RATE_SET             0x0105      /* vector of mac_rate_t */
#define NRX_TLV_TYPE_RSSI                       0x0200
#define NRX_TLV_TYPE_FRAME_COPY_REQ             0x0300      /* Format: see "nrx_tlv_frame_copy_req_t" */
#define NRX_TLV_TYPE_FLOW_CONTROL               0x0301      /* Format: see "nrx_tlv_flow_control_t" */
/** @} */

/**
  @defgroup nrx_tx_pwr_corr_mask_t
  @{

 Controls what type of tx power correction should be applied for Rf transmissions.

 */
typedef uint32_t nrx_tx_pwr_corr_mask_t;
#define NRX_TX_PWR_CORR_MASK_TX_PEAK (1<<0)
#define NRX_TX_PWR_CORR_MASK_VT      (1<<1)

/** @} */

/**
  @defgroup nrx_debug_tx_pwr_corr_req
  @{

 The maximun number of channels for the power correction table

 */
#define NRX_TX_PWR_CORR_MASK_RF_CHANNELS	14

/** @} */

/* G L O B A L  V A R I A B L E S ********************************************/

/* I N T E R F A C E  F U N C T I O N S **************************************/

#endif    /* MAC_API_DEFS_H */
/* END OF FILE ***************************************************************/
