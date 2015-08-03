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
#ifndef MAC_API_IFCTRL_H
#define MAC_API_IFCTRL_H
#include "mac_api_defs.h"

#define NRX_API_VERSION  6

/**
  @file mac_api_ctrl.h

  @defgroup ifctrl Interface control API

  This file defines the MAC control API.

  The control API includes initialisation of firmware, power management and
  exception detection.
 */


/**
  @defgroup NRX_CTRL_SET_ALIGNMENT_REQ
  @ingroup ifctrl
  @{
 */
 
#define NRX_CTRL_SET_ALIGNMENT_REQ       (0  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_SET_ALIGNMENT_CFM       (0  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   /** Minimal size of uplink messages */
   uint16_t          min_sz;
   /** Pad uplink messages so that the total size is a multiple of this value. */
   uint16_t          sz_modulo;
   /** Reserved. Must be set to zero. */
   uint8_t           reserved_1;
   /** Swap bytes in all messages. When set to zero, byte swapping is disabled. */
   uint8_t           swap;
   /** GPIO pin ID to use as interrupt pin towards the host. See definitions below. */
   uint8_t           irq_gpio_id;
   /** GPIO pin ID to asserted when there are uplink messages to be tranmitted. Value 0xFF means disabled. */
   uint8_t           wakeup_gpio_id;
   /** Minimum interval between host interrupts in 1/10 ms. Set to 0xff to disable. */
   uint8_t           min_irq_interval;
   /** Reserved. Must be set to zero. */
   uint8_t           reserved_2;
   /** Requested number of bytes in tx buffers. This figure includes both commands and data. */
   uint16_t          tx_window_bytes;
   /** Must be set to the block size when using SDIO block mode transfers with NRX700 chipset. Set to zero to disable. */
   uint16_t          block_size;
   /** Reserved. Must be set to zero. */
   uint16_t          reserved_3;
} nrx_ctrl_set_alignment_req_t;

typedef struct
{
   uint32_t          result; /** @sa nrx_macapi_status_t */
   /** Number of bufs that the host can used in tx (both commands and data). */
   uint32_t          tx_num_bufs;
   /** Size of tx bufs. */
   uint32_t          tx_buf_size;
   uint32_t          supported_types;
   uint8_t           min_sz;
   uint8_t           initial_gpio_state;
   uint8_t           reserved[2];
} nrx_ctrl_set_alignment_cfm_t;

/** If this bit is set in field irq_gpio_id in nrx_ctrl_set_alignment_req_t,
 * interrupt signalling will done via according to SDIO interface standard. If
 * this bit is not set, the device will interrupt the host by toggling an GPIO
 * pin specified via the other bits of this field. */  
#define NRX_IRQ_SDIO_DAT1     (1<<0)

/** If this bit is set in irq_gpio_id, the GPIO ID encoded in the most
 * significant bits are interpreted as an extended GPIO. If this bit is
 * cleared, the GPIO ID encoded is a normal GPIO. */
#define NRX_IRQ_GPIO_EXT      (1<<2)

/** The 5 most significant bits are interpteted as an number in the range 0..31
 * and is used to determine which GPIO pin is used for interrupt signalling. */
#define NRX_IRQ_GPIO_ID(_id)  ((_id)<<3)

/** @} */


/**
  @defgroup NRX_CTRL_VERSION_REQ
  @ingroup ifctrl
  @{
 */
 
#define NRX_CTRL_VERSION_REQ             (1  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_VERSION_CFM             (1  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t          reserved;
} nrx_ctrl_version_req_t;

typedef struct
{
   uint32_t          version;
   uint32_t          chipid;
   char              fwbuild[4];
} nrx_ctrl_version_cfm_t;

/** @} */


/**
  @defgroup NRX_CTRL_INIT_COMPLETED_REQ
  @ingroup ifctrl
  @{
 */
#define NRX_CTRL_INIT_COMPLETED_REQ      (2  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_INIT_COMPLETED_CFM      (2  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t          reserved;
} nrx_ctrl_init_completed_req_t;

typedef struct
{
   uint32_t          result;
} nrx_ctrl_init_completed_cfm_t;

/** @} */


/**
  @defgroup NRX_CTRL_INTERFACE_DOWN_REQ
  @ingroup ifctrl
  @{
 */
 
#define NRX_CTRL_INTERFACE_DOWN_REQ      (3 | MAC_API_PRIMITIVE_TYPE_REQ) 

typedef struct
{
   uint32_t          reserved;
} nrx_ctrl_interface_down_t;

/** @} */


/**
  @defgroup NRX_CTRL_SCB_ERROR_REQ
  @ingroup ifctrl
  @{
 */

#define NRX_CTRL_SCB_ERROR_REQ           (4  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_SCB_ERROR_CFM           (4  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t magic1;
   uint32_t magic2;
   uint32_t magic3;
   uint32_t magic4;
} nrx_ctrl_scb_error_req_t;

#define NRX_CTRL_SCB_ERROR_REQ_MAGIC1 0x45716572 /* 'E', 'q', 'e', 'r' */
#define NRX_CTRL_SCB_ERROR_REQ_MAGIC2 0x65527272 /* 'e', 'R', 'r', 'r' */
#define NRX_CTRL_SCB_ERROR_REQ_MAGIC3 0x6e6f7361 /* 'n', 'o', 's', 'a' */
#define NRX_CTRL_SCB_ERROR_REQ_MAGIC4 0x00000000

typedef struct
{
   uint8_t           obj_id;
   uint8_t           error_code;
   uint8_t           reserved[2];
   uint32_t          tx_descriptor_address;
   uint32_t          signal_host_attention_address;
   uint32_t          fault_addr;
} nrx_ctrl_scb_error_cfm_t;

#define SCB_ERROR_KEY_STRING "reqErrReason"

/** @} */


/**
  @defgroup NRX_CTRL_SLEEP_FOREVER_REQ
  @ingroup ifctrl
  @{
 */

#define NRX_CTRL_SLEEP_FOREVER_REQ       (5  | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_SLEEP_FOREVER_CFM       (5  | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t          reserved;
} nrx_ctrl_sleep_forever_req_t;

typedef struct
{
   uint32_t          result;
} nrx_ctrl_sleep_forever_cfm_t;

/** @} */


/**
  @defgroup NRX_CTRL_COMMIT_SUICIDE_REQ
  @ingroup ifctrl
  @{
 */
#define NRX_CTRL_COMMIT_SUICIDE_REQ      (6 | MAC_API_PRIMITIVE_TYPE_REQ)

typedef struct
{
   uint32_t          reserved;
} nrx_ctrl_commit_suicide_req_t;

/** @} */


/**
  @defgroup NRX_CTRL_RESERVED_REQ
  @ingroup ifctrl
  @{
 */

#define NRX_CTRL_RESERVED_REQ            (7 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_RESERVED_CFM            (7 | MAC_API_PRIMITIVE_TYPE_CFM)


/**
  @defgroup NRX_CTRL_CLOCK_SYNC_REQ
  @ingroup ifctrl
  @{
 */

#define NRX_CTRL_CLOCK_SYNC_REQ          (8 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_CLOCK_SYNC_CFM          (8 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t          reserved;
} nrx_ctrl_clock_sync_req_t;

typedef struct
{
   uint32_t          result;
   uint32_t          hw_tsf_at_prev_bss_ext_tsf;
   uint64_t          prev_peer_extended_tsf;
   uint64_t          peer_extended_tsf;
   uint32_t          hw_tsf_at_bss_ext_tsf;
   uint8_t           padding_before_hic_tsf;
} nrx_ctrl_clock_sync_cfm_t;

/** @} */


/**
  @defgroup NRX_CTRL_ECHO_REQ
  @ingroup ifctrl
  @{
 */

#define NRX_CTRL_ECHO_REQ                (9 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_CTRL_ECHO_CFM                (9 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   uint32_t data;
} nrx_ctrl_echo_req_t;

typedef struct
{
   uint32_t data;
} nrx_ctrl_echo_cfm_t;

/** @} */


/**
  @defgroup NRX_CTRL_WAKEUP_IND
  @ingroup ifctrl
  @{
 */
 
#define NRX_CTRL_WAKEUP_IND              (0  | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   uint32_t          reason;
   uint32_t          wakeup_count;
} nrx_ctrl_wakeup_ind_t;

/** @} */


/**
  @defgroup NRX_CTRL_SCB_ERROR_IND
  @ingroup ifctrl
  @{
 */
#define NRX_CTRL_SCB_ERROR_IND           (1  | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   uint8_t           obj_id;
   uint8_t           error_code;
   uint8_t           reserved[2];
   uint32_t          tx_descriptor_address;
   uint32_t          signal_host_attention_address;
   uint32_t          fault_addr;
} nrx_ctrl_scb_error_ind_t;

/** @} */

/**
  @defgroup NRX_CTRL_RESERVED_REQ
  @ingroup ifctrl
  @{
 */
#define NRX_CTRL_RESERVED_IND             (2 | MAC_API_PRIMITIVE_TYPE_IND)



/**
  @defgroup NRX_CTRL_NULL_IND
  @ingroup ifctrl
  @{
 */
#define NRX_CTRL_NULL_IND                (3  | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct
{
   uint32_t          reserved[5];
} nrx_ctrl_null_ind_t;

/** @} */



#endif    /* MAC_API_IFCTRL_H */
