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

Nanoradio AB reserves the right to modify this software without
notice.

Nanoradio AB
Torshamnsgatan 39                  info@nanoradio.se
164 40 Kista                       http://www.wep.com
SWEDEN
*******************************************************************************/
#ifndef MAC_API_FRAME_H
#define MAC_API_FRAME_H
#include "mac_api_defs.h"

/**
  @file mac_api_frame.h
  
  @defgroup frame

  This interface contains messages for transmitting and receiving 802.11
  Frames.
*/

/**
  @defgroup NRX_FRAME_TX_REQ
  @ingroup frame
  @{


  Attach dynamic parameters in IE style TLV
  ID       Size           Description
  0x0101   14 (.11b/g)    mac_rate_t  retransmission_rates[];
           21 (.11n)      

  Attach raw 802.11 frame (placed last to work with hosts that can only add extra header space befor the ethernet packet).
 */

#define NRX_FRAME_TX_REQ                 (0  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_FRAME_TX_CFM                 (0  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef uint32_t nrx_tx_flags_t;
#define NRX_TX_FLAG_UCAST                   (1<<0)
#define NRX_TX_FLAG_DO_ENCRYPT              (1<<1)
#define NRX_TX_FLAG_QOS                     (1<<2)
#define NRX_TX_FLAG_CHANNEL_SYNC            (1<<3)
#define NRX_TX_FLAG_FLUSH                   (1<<4)       /* Precede the tx with flushing all outsatnding tx.    */
#define NRX_TX_FLAG_AMPDU_ELIGIBLE          (1<<5)       /* Allow this frame to be transmitted as AMPDU.    */
#define NRX_TX_FLAG_UAPSD_TRIGGER           (1<<6)       /* This frame is UAPSD trigger-enable */
#define NRX_TX_FLAG_UAPSD_DELIVERY          (1<<7)       /* This frame is UAPSD delivery-enable */
#define NRX_TX_FLAG_REQUEST_DOT11_COUNTERS  (1<<16)      /* Attach dot11_counters mib as TLV to _cfm message.   */
#define NRX_TX_FLAG_REQUEST_RETRY_INFO      (1<<17)      /* Attach retries done by rate as TLV to _cfm message. */
#define NRX_TX_FLAG_REQUEST_PS_OVERRIDE     (1<<18)      /* Allow frame to be transmitted when peer is in Power Save. */
#define NRX_TX_FLAG_UPDATE_CCX_STATISTICS   (1<<19)      /* CCX statistics will be updated. */
#define NRX_TX_FLAG_TS_INSERT               (1<<20)       /* Timestamp is inserted in tx frame by baseband. */
#define NRX_TX_FLAG_STA_CLEAR_PS_FILT       (1<<21)
#define NRX_TX_FLAG_NET_CLEAR_PS_FILT       (1<<22)
#define NRX_TX_FLAG_IN_SP                   (1<<23)      /* This frame is a response to a PS-POLL or is a part of a WMM-PS service period */
#define NRX_TX_FLAG_LAST_IN_SP              (1<<24)      /* This frame is the last frame within the service period or ps-poll */
#define NRX_TX_FLAG_EOSP_INSERT             (1<<25)      /* Insert the EOSP (End of Service Period) flag at last frame */
#define NRX_TX_FLAG_BASIC_RATE              (1<<26)      /* Send with basic rate */

typedef struct
{
   /** Out of band parameters like AC, rate list, ... */
   uint16_t            tlv_size;
   /** Payload will start this number of bytes after the HIC header, that is, tlvs are within this size. */ 
   uint16_t            payload_offset;
   /** Controls what information the firmware will update before transmission. @sa nrx_tx_flags_t */
   nrx_tx_flags_t      flags;
   uint32_t            retrans_id;
   /** When the max_lifetime (in us) has passed the frame will be discarded and an tx_cfm will be sent up. */
   uint32_t            max_lifetime;  
   /** Set to zero to disable fragmentation */
   uint16_t            fragment_size;     
   /** Traffic ID */
   uint8_t             tid;     
   uint8_t             key_id;
   uint8_t             baa_buffer_size; /* See 80211 std section 7.3.1.14. Only valid for frames sent under a Block Ack Agreement (e.g currently only for AMPDU eligible frames).*/
   uint8_t             reserved[3];
} nrx_frame_tx_req_t;


/* M80211 unicast frame delivery status code definition */
typedef uint16_t   nrx_tx_status_code_t;
#define NRX_TX_STATUS_OK                   0
#define NRX_TX_STATUS_UNDEL                1
#define NRX_TX_STATUS_BO_TIMEOUT           2
#define NRX_TX_STATUS_QUEUE_FLUSHED        3
#define NRX_TX_STATUS_URGENT_PRIO_OVERRIDE 4
#define NRX_TX_STATUS_STA_PS               5     /* This flag means that the driver shall decrement tx_window 
                                             conuter but keep the packet for possible retransmission.  */
#define NRX_TX_STATUS_MAX_RETRANSMISSIONS  6
#define NRX_TX_STATUS_NET_PS               7
#define NRX_TX_STATUS_PS_TIMEOUT           8
#define NRX_TX_STATUS_NO_RESULT         ((nrx_tx_status_code_t)~0)

typedef uint16_t nrx_tx_cfm_flags_t;
#define NRX_TX_CFM_FLAG_AMPDU      (1 << 0)  /* frame was sent as A-MPDU */
#define NRX_TX_CFM_FLAG_ACK        (1 << 1)  /* frame was acked */
#define NRX_TX_CFM_FLAG_BACK       (1 << 2)  /* frame was block-acked */
#define NRX_TX_CFM_FLAG_EOSP       (1 << 3)  /* frame marks EOSP (End of Service Period) */

typedef struct
{
   nrx_tx_status_code_t status;
   nrx_tx_cfm_flags_t  tx_flags;
   uint32_t            retrans_id;
   uint32_t            tries;
   mac_rate_t          rate_used;
   /** Out of band parameters like AC, etc. */
   uint16_t            tlv_size;
   /** Priority */
   uint8_t             prio;
   /** Traffic ID sent in tx_req. */
   uint8_t             tid;
   uint16_t            seq_ctrl;
   int8_t              ack_rssi;
   int8_t              ack_noise;
   uint8_t             reserved[2];
   uint32_t            start_ts;
   uint32_t            end_ts;
} nrx_frame_tx_cfm_t;

/** @} */


/**
  @defgroup NRX_FRAME_CONFIG_REQ
  @ingroup frame
  @{

 */

#define NRX_FRAME_CONFIG_REQ             (1  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_FRAME_CONFIG_CFM             (1  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef uint32_t nrx_filter_flags_t;
        /** Pass all received frames regardless of destination address */
#define NRX_FILTER_PROMISC             (1<<0)
        /** Pass all frames in our BSS */
#define NRX_FILTER_PROMISC_BSS         (1<<1)
        /** Pass multicast/broadcast that does not belong to our BSS.
         *  For our bss, the multicast filter on the bss will be applied instead */
#define NRX_FILTER_PASS_MULTI_OTHER    (1<<2)
        /** Pass frames even if FCS check failed */
#define NRX_FILTER_BADFCS              (1<<3) 

typedef struct
{
   /** Flags that control frame filtering (i.e. which frames are passed to the
    * host via message NRX_FRAME_RX_IND). @sa nrx_filter_flags_t
    */
   nrx_filter_flags_t flags;

   /** Crop received frames to this size before passing them to host. Set to
    * zero to disable cropping
    */
   uint32_t  snap_len;    

   /** Bitmasks for frame subtypes to be sent up in raw format */
   uint32_t  mgmt;
   uint32_t  data;
   uint32_t  ctrl;

   /** Special bit mask for mgmt subtype action. Please note that action
    * subtype flag must be set in mgmt bit mask.
    */
   uint32_t  action;

   /** Reserved for future use. Must be zero */
   uint32_t  reserved;
} mac_frame_filter_t;

typedef struct
{
   mac_frame_filter_t    filter;
   uint32_t            frame_ind_header_padding;  /* Will move payload */
   uint32_t            reserved;
} nrx_frame_config_req_t;

typedef struct
{
   uint32_t            result;
} nrx_frame_config_cfm_t;


/** @} */


/**
  @defgroup NRX_FRAME_COPY_REQ
  @ingroup frame
  @{


  Format and content of NRX_FRAME_COPY_REQ is idential to NRX_FRAME_TX_REQ

 */

#define NRX_FRAME_COPY_REQ                 (2  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_FRAME_COPY_CFM                 (2  | MAC_API_PRIMITIVE_TYPE_CFM)

#define nrx_frame_copy_req_t nrx_frame_tx_req_t 

typedef struct
{
   nrx_tx_status_code_t status;
} nrx_frame_copy_cfm_t;

/** @} */


/**
  @defgroup NRX_FRAME_RX_IND
  @ingroup frame
  @{

 */

#define NRX_FRAME_RX_IND                 (0  | MAC_API_PRIMITIVE_TYPE_IND)


typedef uint32_t nrx_frame_rx_flags_t;
#define NRX_RX_FLAG_DECRYPTED                   (1<<0) /* A protected frame has been decrypted by FW */
#define NRX_RX_FLAG_SHORT_PREAMBLE              (1<<1)
#define NRX_RX_FLAG_GREENFIELD                  (1<<2)
#define NRX_RX_FLAG_SHORT_GI                    (1<<3)
#define NRX_RX_FLAG_STBC                        (1<<4)
#define NRX_RX_FLAG_FCS_FAIL                    (1<<5)
#define NRX_RX_FLAG_FCS_AT_END                  (1<<6)
#define NRX_RX_FLAG_AMPDU                       (1<<7)
#define NRX_RX_FLAG_PROTECTED                   (1<<8) /* Frame was protected when received by FW */
#define NRX_RX_FLAG_CRYPTO_REPLAY_FAILURE       (1<<9)
#define NRX_RX_FLAG_CRYPTO_GROUP_MIC_FAILURE    (1<<10)
#define NRX_RX_FLAG_CRYPTO_PAIRWISE_MIC_FAILURE (1<<11)
#define NRX_RX_FLAG_CRYPTO_MICHAEL_FAILURE      (1<<12)
#define NRX_RX_FLAG_PART_OF_BAA                 (1<<13)

typedef struct
{
   /** RSSI in dBm */
   int8_t               rssi;
   /** Noise in dBm */
   int8_t               noise;
   uint16_t             noise_count;
   /** @sa mac_rate_t */
   uint16_t             rate;
   /** Current channel in MHz */
   nrx_mac_chn_freq_t   freq;
   /** Out of band parameters like AC, etc. */
   uint16_t             tlv_size;
   /** Payload will start this number of bytes after the HIC header, that is tlvs are within this size. */ 
   uint16_t             payload_offset;
   /** payload size corresponds to the remaining part of the hic message. m80211_frame_size may be different from payload size if crop_to_len is not zero */
   uint16_t             m80211_frame_size;
   uint8_t              key_id;
   uint8_t              reserved;
   /** Timestamp when frame was received by PHY */
   uint32_t             start_ts;
   /** @sa nrx_frame_rx_flags_t */
   nrx_frame_rx_flags_t flags;
   uint32_t             end_ts;
   /* Attach dynamic parameters in IE style TLV */
   /* Attach raw 802.11 frame (placed last to work with hosts that can only add extra header space befor the ethernet packet). */
} nrx_frame_rx_ind_t;

/** @} */

/**
  @defgroup NRX_FRAME_STREAM_CTRL_IND
  @ingroup frame
  @{

 */

#define NRX_FRAME_STREAM_CTRL_IND               (1  | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   nrx_tlv_head_t     tlv_head;
   nrx_hic_trans_id_t transid;                  /* Host transid reference */
   uint8_t            reserved[2];
   uint32_t           retrans_id;               /* Mac retrans_id reference. To be used in retrans_id in the following NRX_FRAME_COPY_REQ message */
}nrx_tlv_frame_copy_req_t;

typedef struct
{
   uint16_t tlv_size;
   uint8_t  reserved[2];
   /* Attach dynamic parameters style TLV. Currently NRX_TLV_TYPE_FRAME_COPY_REQ and NRX_TLV_TYPE_FLOW_CONTROL may appear */

}nrx_frame_stream_ctrl_ind_t;

/** @} */
#endif /* MAC_API_FRAME_H */
