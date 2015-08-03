/* Copyright (c) 2012 by SNDC AB
 *
 * This software is copyrighted by and is the sole property of Samsung Nanoradio Design Centre (SNDC) AB.
 * All rights, title, ownership, or other interests in the software remain the property of SNDC AB.
 * This software may only be used in accordance with the corresponding license agreement.  Any unauthorized use, duplication, transmission, distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior written consent of SNDC AB.
 *
 * SNDC AB reserves the right to modify this software without notice.
 *
 * SNDC AB
 * Torshamnsgatan 39                       info@nanoradio.se
 * 164 40 Kista                            http://www.nanoradio.se
 * SWEDEN
 */

#ifndef _NANOCORE_H
#define _NANOCORE_H
#include "driverenv.h"
#include "mac_api.h"
#include "mac_api_debug.h"

/* Forward declaration */
struct nanoc_hw;

/**
 * @defgroup nanocore_driver_interface Nanocore driver interface.
 * @{
 */

/**
 * Forward declaration of client driver private data. This struct must be defined by the driver.
 */
struct nanoc_context;
struct nanoc_device_descr;

struct nanoc_ops {

   /**
    * Called by nanocore before recv_handler is called.
    *
    * The client must return the queue number that the request was sent on, in order
    * for nanocore to increase tx window on the correct queue.
    */
   unsigned (*get_q)(struct nanoc_context *context, unsigned net_id, de_pdu_t *pdu);

   /**
    * Called by nanocore when a message is reveived matching the net_id for
    * current client. The ownership of the buffer is transfered to the client
    * and the client should free the buffer (using de_pdu_free()) when finished
    * processing it.
    *
    * The hic message in the buffer will start with the nrx_hic_message_t
    * header and any trailing padding removed.
    */
   void (*recv_handler)(struct nanoc_context *context, unsigned net_id, de_pdu_t *pdu);

   /**
    * Called by nanocore when a new hardware has been detected. The handler
    * should return a pointer to some device context. This pointer is totally
    * opaque to nanocore. It will only be used to identify the client in
    * subsequent calls.
    */
   struct nanoc_context * (*new_device)(void *hwdev, const struct nanoc_device_descr*, unsigned char *mac_addr);

   /**
    * Called by nanocore when a previously announced hardware device was
    * removed. After this call, the device handle is no longer valid.
    */
   void                   (*del_device)(struct nanoc_context *context);

   /** Called by nanocore when the device is ready to receive HIC messages */
   void                   (*link_up)(struct nanoc_context *context, unsigned qmask);

   /** Called by nanocore when the communications link goes down or the tx window is full. */
   void                   (*link_down)(struct nanoc_context *context, unsigned qmask);

   /** Called by nanocore when the chip was reset (e.g. to restart after coredump). */
   void                   (*restart)(struct nanoc_context *context);

   /** Called by nanocore when a message passed to nanoc_send() has been transmitted. */
   void                   (*send_complete)(struct nanoc_context * context, de_pdu_t *pdu, int result);
};

#define NANOC_NUM_QS 2
enum nanoc_queues {
   NANOC_QCMD  = 0,
   NANOC_QDATA = 1,
   NANOC_QALL  = (1<<NANOC_QCMD) | (1<<NANOC_QDATA)
};

enum nanoc_feat {
   NANOC_FEAT_INIT =     (1<<0), /* Only used by Nanocore itself */
   NANOC_FEAT_WIFI_STA = (1<<1),
   NANOC_FEAT_WIFI_AP  = (1<<2),
   NANOC_FEAT_BT       = (1<<3),
   NANOC_FEAT_AUDIO    = (1<<4),
   NANOC_FEAT_TESTMODE = (1<<5)
};

/** Initialize new client. */
int nanoc_register(const struct nanoc_ops *ops);

int nanoc_chip_up(struct nanoc_context *handle, enum nanoc_feat feature_mask);

int nanoc_chip_down(struct nanoc_context *handle);

int nanoc_send_cmd(unsigned net_id, de_pdu_t *pdu);

int nanoc_send_data(unsigned net_id, de_pdu_t *pdu);

int nanoc_req_coredump(struct nanoc_context *handle);

int nanoc_alloc_net_id(struct nanoc_context *handle);

int nanoc_alloc_fixed_net_id(unsigned net_id, struct nanoc_context *handle);

void nanoc_free_net_id(struct nanoc_context *handle, unsigned net_id);

uint32_t nanoc_get_api_version(struct nanoc_context *handle);

/*
 *===========================================================================
 * Nanocore hw/transport interface.
 *===========================================================================
 */

/** Interface towards transport driver */
struct nanoc_hw_ops {
   /** Init transport */
   int (*hw_init)(struct nanoc_hw *hw, unsigned flags);
#define NANOC_INIT_4BIT (1<<0) /** Enable SDIO 4-bit mode in host controller */
#define NANOC_INIT_HS   (1<<1) /** Enable SDIO high-speed mode in host controller */
   /** Start transport receive */
   int (*hw_start_rx)(struct nanoc_hw *hw);
   /** Stop transport receive */
   int (*hw_stop_rx)(struct nanoc_hw *hw);
   /** Send data to chip */
   int (*hw_send)(struct nanoc_hw *hw, de_pdu_t *buf);
   /** Send data to chip (blocking). Not when rx is started. */
   int (*hw_download)(struct nanoc_hw *hw, de_pdu_t *buf);
   /** Read 8-bit configuration register (blocking) */
   int (*hw_getreg)(struct nanoc_hw *hw, unsigned addr, unsigned *value);
   /** Write 8-bit configuration register (blocking) */
   int (*hw_setreg)(struct nanoc_hw *hw, unsigned addr, unsigned value);
   /** Request wakeup/Enable chip sleep (non-blocking) */
   int (*hw_wakeup)(struct nanoc_hw *hw, unsigned wakeup);
   /** Drive chip shutdown pin */
   int (*hw_shutdown)(struct nanoc_hw *hw, unsigned asserted);
   /** Control chip power supply */
   int (*hw_power)(struct nanoc_hw *hw, unsigned on);
   /** Get interface clock speed */
   int (*hw_getspeed)(struct nanoc_hw *hw, unsigned *speed);
   /** Set interface clock speed */
   int (*hw_setspeed)(struct nanoc_hw *hw, unsigned speed);
};

/** Called from transport driver when hardware has been detected */
int nanoc_hw_register(struct nanoc_hw *hw);

void nanoc_hw_unregister(struct nanoc_hw *hw);

void nanoc_hw_restart(struct nanoc_hw *hw);

/** Called from transport driver when a buffer has been received */
void nanoc_hw_recv_handler(struct nanoc_hw *hw, de_pdu_t *buf);

/** Called from transport driver when a buffer has been successfully transmitted */
void nanoc_hw_send_complete(struct nanoc_hw *hw, de_pdu_t *buf);

/** Called from transport driver when a buffer failed to be transmitted */
void nanoc_hw_send_failed(struct nanoc_hw *hw, de_pdu_t *buf);

void nanoc_printbuf(const char *prefix, const void *ptr, size_t len);

void nanoc_hiclog(const char *tag, de_pdu_t *pdu);

/*
 *===========================================================================
 * Nanocore local definitions.
 *===========================================================================
 */

enum chip_variants
{
   NRXUNKNOWN,
   NRX701,
   NRX615,
   NRX900
};

struct nanoc_device_descr {
   enum chip_variants  ndd_variant;
   uint32_t            ndd_features;
   uint32_t            ndd_download_speed;
   uint32_t            ndd_coredump_speed;
   uint32_t            ndd_tx_window;
};

/** flags passed to new_devivce */
#define NANOC_DEVICE_FEAT_WIFI     (1 << 0)
#define NANOC_DEVICE_FEAT_BT       (1 << 1)
#define NANOC_DEVICE_FEAT_HT20     (1 << 2)
#define NANOC_DEVICE_FEAT_HT40     (1 << 3)
#define NANOC_DEVICE_FEAT_AMPDU_TX (1 << 4)
#define NANOC_DEVICE_FEAT_STBC_RX  (1 << 5)
#define NANOC_DEVICE_FEAT_BROKEN_TX_POWER  (1 << 6)
#define NANOC_DEVICE_FEAT_5GHZ     (1 << 7)

enum coredump_state
{
   COREDUMP_STATE_INIT,
   COREDUMP_STATE_WAIT_CFM,
   COREDUMP_STATE_UPLOAD,
   COREDUMP_STATE_COMPLETE,
   COREDUMP_STATE_FAIL
};

struct nanoc_hw {
   /** Initialisation values set by transport driver ---------- */
   const struct nanoc_hw_ops  *ops;
   struct de_obj               de;
   /** Minimal size of uplink messages */
   uint16_t                    min_sz_to_host;
   /** Pad downlink messages so that the total size is a multiple of this value. */
   uint16_t                    sz_modulo_to_host;
   /** Pad downlink messages so that the total size is a multiple of this value. */
   uint16_t                    sz_modulo_from_host;
   /** GPIO pin ID to use as interrupt pin towards the host. Value 0xFF means use SDIO DAT1 interrupt */
   uint8_t                     irq_gpio_id;
   /** GPIO pin ID to asserted when there are uplink messages to be tranmitted. Value 0xFF means disabled. */
   uint8_t                     wakeup_gpio_id;
   /** Minimum interval between host interrupts in 1/10 ms */
   uint8_t                     min_irq_interval;
   /* End of init values -------------------------------------- */

   /** Nanocore private variables, not to be used by transport driver */
   uint32_t                    api_version;
   unsigned                    flags;
#define NANOC_HW_IDLE            (1<<0)
#define NANOC_HW_WAKEUP          (1<<1)
#define NANOC_HW_COMMIT_SUICIDE  (1<<2)
#define NANOC_HW_COREDUMP        (1<<3)
#define NANOC_HW_RESTARTING      (1<<4)
   /** Bitmask where a '1' indicates that the corresponding queue (NANOC_QCMD
    * or NANOC_QDATA) is UP */
   unsigned                    queue_up;
   unsigned                    fw_id;
   unsigned                    num_clients;
   /** The size of the tx buffer used by the hic interface */
   unsigned                    tx_buf_size;
   /** Maximum available tx buffers on hic interface */
   struct {
      unsigned                 max;
      /** Number of currently used tx buffers on hic interface */
      unsigned                 used;
   } tx_win[2];
   void                       *dev; /** Pointer to device (used by driverenv) */
   uint16_t                    trans_id;
   const struct nanoc_device_descr   *device_descr;
   unsigned                    chip_ver;
   struct {
      unsigned                 state;
      struct chip_descr       *chip_descr;
      uint32_t                 descr_addr;
      uint32_t                 irq_addr;
      unsigned int             region_index;
      unsigned int             region_addr;
      unsigned int             objid;
      unsigned int             errcode;
      void                    *fh;
      de_pdu_t                *pdus[256];
      unsigned int             outstanding_pdus;
      de_timer_t               timer;
   } coredump;
   unsigned char               mac_addr[6];
   de_mutex_t                  mutex;
   de_timer_t                  cmd_timer;
   unsigned                    cmd_idx;
   unsigned                    cmd_trans_id;
   unsigned int                default_speed;
   unsigned int                download_speed;
   unsigned int                coredump_speed;
};

static inline int nanoc_tp_getspeed(struct nanoc_hw *hw, unsigned int *speed)
{
   if(hw->ops->hw_getspeed == NULL)
      return -EOPNOTSUPP;

   return (*hw->ops->hw_getspeed)(hw, speed);
}

static inline int nanoc_tp_setspeed(struct nanoc_hw *hw, unsigned int speed)
{
   if(hw->ops->hw_setspeed == NULL)
      return -EOPNOTSUPP;

   if(speed == 0)
      return 0; /* no change */

   return (*hw->ops->hw_setspeed)(hw, speed);

}

struct memory_region {
   uint32_t addr;
   uint32_t size;
};

struct chip_descr {
   const struct memory_region *regions;
   size_t                      nregions;
   uint32_t                    bm_reg_ctrl;
   uint32_t                    bm_reg_descr;
   uint32_t                    safe_addr;
   int                        (*synchronize)(struct nanoc_hw *);
   void                       (*setup_memory)(struct nanoc_hw *);
   void                       (*read_memory)(struct nanoc_hw *, uint32_t addr, uint32_t size);
};


#endif /* _NANOCORE_H */
