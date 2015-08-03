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
#ifndef MAC_API_SCAN_H
#define MAC_API_SCAN_H
#include "mac_api_defs.h"

/**

  @file mac_api_scan.h

  @brief Scan

  @defgroup scan Network scan

*/

/**
  @defgroup scan_start NRX_SCAN_START_REQ
  @ingroup scan
  @{

  Description.
 */

/** Message ID for request firmware to initiate scan. */
#define NRX_SCAN_START_REQ               (0 | MAC_API_PRIMITIVE_TYPE_REQ)

/** Message ID of confirmation message of a SCAN_REQ */
#define NRX_SCAN_START_CFM               (0 | MAC_API_PRIMITIVE_TYPE_CFM)

/**
  Message structure for a NRX_SCAN_START_REQ

  @note This message is followed by a chunk for information elements that is
  used to populate the probe request frame. The format of this variable sized
  data chunk is the same as IE's in the 802.11 frame (i.e. tag and length
  followed by the IE contents).
 */
typedef struct
{
   uint32_t                   tlv_size;
   /** Flags */
   uint32_t                   flags;
#define NRX_SCAN_FLAG_EXCLUDE_MY_BSS     (1 << 0)
#define NRX_SCAN_FLAG_UNUSED             (1 << 1) 
#define NRX_SCAN_FLAG_SEND_QUEUED        (1 << 2)   /** Don't send probe requests, instead send the queued frames */
#define NRX_SCAN_FLAG_SEND_READY_IND     (1 << 3)   /** Send NRX_SCAN_READY_IND on period completion */
#define NRX_SCAN_FLAG_REVERSE            (1 << 4)   /** Indicate reverse scan (answers to probe reqs) */
#define NRX_SCAN_FLAG_BLOCK_INDS         (1 << 5)   /** Scan will not send probe responses and beacons to host */

   /**
    * Minimum channel time. The minimum time spent on each channel (in TU)
    * listening for active stations
    */
   uint32_t                   min_ch_time;

   /**
    * Max channel time. The maximum time spent on each channel (in TU). If at least one
    * listening for active stations
    */
   uint32_t                   max_ch_time;

   /**
    * Time interval (in TU)
    */
   uint32_t                   channel_interval;

   /**
    * Time (in TU) between repetions of this scan request. Set this to zero to
    * do a one-shot scan
    */
   uint32_t                   scan_period;

   mac_retrans_t              retrans; /* num_probes_per_channel = tries */
   nrx_mac_addr_t             bssid;
   uint8_t                    reserved[2];

   /* TLVs follow (always 32bit aligned) */
   LIST_OF_TLV(chn_tlv)

   /* IEs follow (always 32bit aligned) */
   LIST_OF_INFORMATION_ELEMENTS(ie_list)
} nrx_scan_start_req_t;  

/** Message structure for a NRX_SCAN_START_CFM */
typedef struct
{
   nrx_macapi_status_t        status;
} nrx_scan_start_cfm_t;

/** @} */


/**
  @defgroup scan_stop NRX_SCAN_STOP_REQ
  @ingroup scan
  @{

  Description.
 */

#define NRX_SCAN_STOP_REQ                (1 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_SCAN_STOP_CFM                (1 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t                   reserved;
} nrx_scan_stop_req_t;

typedef struct
{
   uint32_t                   status;
} nrx_scan_stop_cfm_t;

/** @} */


/**
  @defgroup scan_ready_ind NRX_SCAN_READY_IND
  @ingroup scan
  @{

   scan_ready_ind are sent under the following conditions:
   
      (start_req->flags & NRX_SCAN_FLAG_SEND_READY_IND)
   && (   job_done
       || (scan_stop && job_in_process && (scan_period == 0)))
 */

#define NRX_SCAN_READY_IND               (0 | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   uint32_t                   reserved;
} nrx_scan_ready_ind_t;

/** @} */

#endif    /* MAC_API_SCAN_H */
