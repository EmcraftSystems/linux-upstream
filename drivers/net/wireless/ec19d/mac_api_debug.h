/*
  Copyright (c) 2004 by Nanoradio AB

  This software is copyrighted by and is the sole property of Nanoradio AB.

  All rights, title, ownership, or other interests in the software remain the
  property of Nanoradio AB.  This software may only be used in accordance with
  the corresponding license agreement.  Any unauthorized use, duplication,
  transmission, distribution, or disclosure of this software is expressly
  forbidden.

  This Copyright notice may not be removed or modified without prior written
  consent of Nanoradio AB.

  Nanoradio AB reserves the right to modify this software without notice.
 */
#ifndef MAC_API_DEBUG_H
#define MAC_API_DEBUG_H
#include "mac_api_defs.h"

/**

  @file mac_api_debug.h

  @defgroup debug Debug Interface

  Interface for test and debug of chip.

*/


/**
  @defgroup debug_setup NRX_DEBUG_SETUP_REQ
  @ingroup debug
  @{

  Setup test/debug mode of operation.

 */

/** Message ID for request message */
#define NRX_DEBUG_SETUP_REQ               (0 | MAC_API_PRIMITIVE_TYPE_REQ)
/** Message ID for confirm message */
#define NRX_DEBUG_SETUP_CFM               (0 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_setup_req
{
   uint32_t flags;
};

/** Set flag to enable debug/test mode */
#define NRX_DEBUG_FLAG_RF_ENABLE (1<<0)

struct nrx_debug_setup_cfm
{
   /** Status of the operation. Zero on success. */
   uint32_t status;
};

/** @} */


/**
  @defgroup debug_channel NRX_DEBUG_CHANNEL_REQ
  @ingroup debug
  @{

  Initiate collection of data for a specific channel.
 */

#define NRX_DEBUG_CHANNEL_REQ              (1 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_CHANNEL_CFM              (1 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_channel_req
{
   mac_chn_prop_t  chn_prop;
};

struct nrx_debug_channel_cfm
{
   /** Status of the operation. Zero on success. */
   uint32_t status;
};

/** @} */


/**
  @defgroup debug_tx NRX_DEBUG_TX_REQ
  @ingroup debug
  @{

  Transmit frames.
 */

#define NRX_DEBUG_TX_REQ                   (2 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_TX_CFM                   (2 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_tx_req
{
   mac_retrans_t  retrans;
   uint32_t       repeat;
   uint16_t       interval;
   uint16_t       flags;
#define NRX_TX_IND_LAST (1<<0)
};

struct nrx_debug_tx_cfm
{
   /** Status of the operation. Zero on success. */
   uint32_t status;
};

/** @} */


/**
  @defgroup debug_rx_stat NRX_DEBUG_RX_STATS_REQ
  @ingroup debug
  @{

  Read RX statistics.
 */

#define NRX_DEBUG_RX_STATS_REQ             (3 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_RX_STATS_CFM             (3 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_rx_stats_req
{
   /** Select which rate to return stats for. */
   mac_rate_t   rate;
   /** Reserved. Must be zero. */
   uint16_t     reserved;
};

typedef struct
{
   uint32_t num_frames;          /* total number of frames (crc good/bad) */ 
   uint32_t num_mcast;           /* good crc */  
   uint32_t num_ucast_to_us;     /* good crc */
   uint32_t num_ucast_to_other;  /* good crc */
   uint32_t num_bad_crc;         /* all frames with bad crc */
   int32_t  rssi_dbm;            /* accumulated value - divide with num_frames */
   int32_t  snr_db;              /* accumulated value - divide with num_frames */
   int32_t  dc_re;               /* accumulated value - divide with num_frames */
   int32_t  dc_im;               /* accumulated value - divide with num_frames */
   int32_t  freq_error;          /* in Hz, accumulated value - divide with num_frames */
   int32_t  delayspread;         /* accumulated value - divide with num_frames */
} nrx_debug_rx_stats_t;

struct nrx_debug_rx_stats_cfm
{
   uint32_t             status;
   mac_rate_t           rate;
   uint16_t             reserved;
   nrx_debug_rx_stats_t stats;
};

/** @} */


/**
  @defgroup debug_rx_stat NRX_DEBUG_RX_STATS_CLEAR_REQ
  @ingroup debug
  @{

  Clear RX statistics.
 */

#define NRX_DEBUG_RX_STATS_CLEAR_REQ       (4 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_RX_STATS_CLEAR_CFM       (4 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_rx_stats_clear_req
{
   /** Reserved. Must be zero. */
   uint32_t reserved;
};

struct nrx_debug_rx_stats_clear_cfm
{
   uint32_t status;
};

/** @} */

/**
  @defgroup debug_poke NRX_DEBUG_POKE_REQ
  @ingroup debug
  @{

 */

#define NRX_DEBUG_POKE_REQ                 (5 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_POKE_CFM                 (5 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_poke_req
{
   uint32_t op;
#define NRX_DEBUG_OP_WRITE  (1<<0) /** Write memory */
#define NRX_DEBUG_OP_READ   (1<<1) /** Read (after write) */
   uint32_t addr;
   uint32_t size;
   /* Variable size may follow */
};

struct nrx_debug_poke_cfm
{
   uint32_t status;
   uint32_t addr;
   uint32_t size;
   /* Variable size data may follow */
};


/**
  @defgroup tx_pwr_corr NRX_DEBUG_TX_PWR_CORR_REQ
  @ingroup debug
  @{

 */

#define NRX_DEBUG_TX_PWR_CORR_REQ          (6 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_TX_PWR_CORR_CFM          (6 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_tx_pwr_corr_req
{
    nrx_tx_pwr_corr_mask_t tx_pwr_corr_mask;
};

struct nrx_debug_tx_pwr_corr_cfm
{
    uint32_t status;
};

/** @} */

/**
  @defgroup power_cycle NRX_DEBUG_START_TX_POWER_CYCLE
  @ingroup debug
  @{

  Cycle Tx, Rx and Sleep continuously. Used for burn in tests.
 */

#define NRX_DEBUG_POWER_CYCLE_START_REQ          (7 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_POWER_CYCLE_START_CFM          (7 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_power_cycle_start_req
{
   mac_chn_prop_t  chn_prop;
   /** Tx time in us */
   uint32_t tx_time;
   /** Tx time in us */
   uint32_t rx_time;
   /** Sleep time in us */
   uint32_t sleep_time;
};

struct nrx_debug_power_cycle_start_cfm
{
   uint32_t status;
};

/** @} */

/**
  @defgroup power_cycle NRX_DEBUG_STOP_TX_POWER_CYCLE
  @ingroup debug
  @{

  Stop burn in cycling.
 */

#define NRX_DEBUG_POWER_CYCLE_STOP_REQ          (8 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_POWER_CYCLE_STOP_CFM          (8 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_power_cycle_stop_req
{
   uint32_t dummy;
};

struct nrx_debug_power_cycle_stop_cfm
{
   uint32_t status;
};

/** @} */


/**
  @defgroup debug_afc NRX_DEBUG_AFC_REQ
  @ingroup debug
  @{
 
  Enable/Disable AFC
 */

#define NRX_DEBUG_AFC_REQ          (9 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_AFC_CFM          (9 | MAC_API_PRIMITIVE_TYPE_CFM)

struct nrx_debug_afc_req
{
    uint32_t value;
};

struct nrx_debug_afc_cfm
{
    uint32_t status;
};

/** @} */

/**
  @defgroup rf_tx_pwr_corr NRX_DEBUG_RF_TX_PWR_CORR_REQ
  @ingroup debug
  @{
 
  Set RF Power Correction for TX
 */

#define NRX_DEBUG_RF_TX_PWR_CORR_REQ          (10 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_DEBUG_RF_TX_PWR_CORR_CFM          (10 | MAC_API_PRIMITIVE_TYPE_CFM)

#define NRX_DEBUG_RX_TX_PWR_CORR_MODE_WIFI      (1)
#define NRX_DEBUG_RX_TX_PWR_CORR_MODE_WIFI_QPSK (2)
#define NRX_DEBUG_RX_TX_PWR_CORR_MODE_BT        (3)

struct nrx_debug_rf_tx_pwr_corr_req
{
   uint16_t frequency;
   uint8_t  mode_select;
   uint8_t  filler;
   int32_t  delta_tx_gain;
};

struct nrx_debug_rf_tx_pwr_corr_cfm
{
    uint32_t status;
};

/** @} */

/**
  @defgroup debug_tx_ind NRX_DEBUG_TX_IND
  @ingroup debug
  @{

  Transmit indication.
 */

#define NRX_DEBUG_TX_IND                   (0 | MAC_API_PRIMITIVE_TYPE_IND)

struct nrx_debug_tx_ind
{
   uint32_t status;
   uint32_t frame_no;
};

/** @} */


/**
  @defgroup debug_log_ind NRX_DEBUG_LOG_IND
  @ingroup debug
  @{

  Transmit log indication.
 */

#define NRX_DEBUG_LOG_IND                   (1 | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct 
{
   uint32_t id;
   uint32_t value_1;
   uint32_t value_2;
}nrx_debug_log_ind_t;

/** @} */


#endif
