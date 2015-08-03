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

#include "driverenv.h"
#include "nanocore.h"
#include "nanoc.h"
#include "coredump.h"

/* Definitions ====================================== */

#define MAX_NUM_CLIENTS 4
#define MAX_NUM_HW_HANDLES (MAX_NUM_CLIENTS*MAX_NUM_HWS)
#define MAX_NUM_NET_IDS (NRX_NET_ID_FIRST_DYN+10)

struct nanoc_client
{
   const struct nanoc_ops *ops;  /* Set to NULL if record is unused */
};

struct hw_handle
{
   struct nanoc_context   *handle;  /* Set to NULL if record is unused */
   struct nanoc_hw        *hw;
   struct nanoc_client    *client;
   de_pdu_t               *queue[NANOC_NUM_QS];
   unsigned                chip_up;
};

static struct hw_handle *get_hw_handle(unsigned net_id);
static unsigned add_hw_handle(struct nanoc_context *handle,
                              struct nanoc_client * client,
                              struct nanoc_hw *hw);
static struct hw_handle *find_hw_handle(struct nanoc_context *handle);
static void   send_queued_pdus(struct nanoc_hw *hw);
static void   discard_queues(struct hw_handle *h);

/* Variables ======================================== */

static struct nanoc_client  client_array[MAX_NUM_CLIENTS];
static struct hw_handle     hw_handle_array[MAX_NUM_HW_HANDLES];
static struct hw_handle    *hw_handle_by_net_id[MAX_NUM_NET_IDS];
static de_mutex_t           nanoc_mutex = DE_MUTEX_INIT(nanoc_mutex);

/* API functions ==================================== */

int nanoc_register(const struct nanoc_ops *ops)
{
   unsigned i;
   for (i = 0; i < MAX_NUM_CLIENTS; i++) {
      if (client_array[i].ops == NULL) {
         memset(&client_array[i], 0, sizeof(client_array[0]));
         client_array[i].ops = ops;
         return 0;
      }
   }
   de_error(DE_TR_NANOC, "Max num clients reached");
   return -1;
}

static inline int
within_tx_window(int qidx, const struct nanoc_hw *hw, unsigned int num_bufs)
{
   return hw->tx_win[qidx].used + num_bufs <= hw->tx_win[qidx].max;
}

static void set_pdu_window_size(de_pdu_t *pdu, int qidx, unsigned num_bufs)
{
   nrx_hic_frame_t *fp = de_pdu_data(pdu);

   if(qidx == NANOC_QCMD) {
      fp->control.tx_window_non_wifi_data = num_bufs;
      fp->control.tx_window = 0;
   } else {
      fp->control.tx_window_non_wifi_data = 0;
      fp->control.tx_window = num_bufs;
   }
}

static int nanoc_send(int qidx, de_pdu_t *pdu, struct hw_handle *h)
{
   int ret = 0;
   unsigned num_bufs;
   struct nanoc_hw *hw = h->hw;

   nanoc_hw_add_framing_hdr(hw, pdu);
   num_bufs = nanoc_get_num_bufs(hw, de_pdu_size(pdu));

   de_mutex_lock(&hw->mutex);

   if (hw->flags & NANOC_HW_RESTARTING) {
      de_info(DE_TR_NANOC, "Chip is restarting");
      ret = -EBUSY;
   
   } else if (hw->flags & NANOC_HW_IDLE) {
      /*
       * Chip is sleeping. Queue this message and send wakeup
       */
      DE_BUG_ON(h->queue[qidx] != NULL);
      h->queue[qidx] = pdu;
      de_info(DE_TR_NANOC, "PDU queue@%d (sleep)", qidx);
      nanoc_link_down(hw, NANOC_QALL);
      if ((hw->flags & NANOC_HW_WAKEUP) == 0) {
         de_info(DE_TR_NANOC, "Wakeup");
         hw->flags |= NANOC_HW_WAKEUP;
         hw->ops->hw_wakeup(hw, 1);
      }
   
   } else if (!within_tx_window(qidx, hw, num_bufs)) {
      /*
       * There is not enough space in the tx window for this message. Queue
       * this message. The queue is only one entry per client and hw
       */
      DE_ASSERT(h->queue[qidx] == NULL);
      h->queue[qidx] = pdu;
      de_info(DE_TR_NANOC, "PDU queue@%d (txwindow)", qidx);
      nanoc_link_down(hw, (1<<qidx));
   
   } else {
      /*
       * Clear to send. Decrease the available tx window by the amount of
       * buffers used for this pdu.
       */
      set_pdu_window_size(pdu, qidx, num_bufs);
      hw->tx_win[qidx].used += num_bufs;
      de_debug(DE_TR_NANOC, "num_bufs=%d tx_win[0].used=%d tx_win[1].used=%d", num_bufs, hw->tx_win[NANOC_QCMD].used, hw->tx_win[NANOC_QDATA].used);
      ret = nanoc_hw_send(hw, pdu);
   }

   de_mutex_unlock(&hw->mutex);

   return ret;
}

int nanoc_send_cmd(unsigned net_id, de_pdu_t *pdu)
{
   struct hw_handle *h = get_hw_handle(net_id);
   DE_BUG_ON(!h);
   return nanoc_send(NANOC_QCMD, pdu, h);
}

int nanoc_send_data(unsigned net_id, de_pdu_t *pdu)
{
   struct hw_handle *h = get_hw_handle(net_id);
   DE_BUG_ON(!h);
   return nanoc_send(NANOC_QDATA, pdu, h);
}

int nanoc_chip_up(struct nanoc_context *handle, enum nanoc_feat feature_mask)
{
   struct hw_handle *h = find_hw_handle(handle);
   int ret;
   de_mutex_lock(&h->hw->mutex);
   h->chip_up = 1;
   ret = nanoc_hw_chip_up(h->hw, feature_mask);
   if (ret > 0) {
      /* Someone else has done chip up before this. If any queue is already up notify this. */
      if (h->hw->queue_up) {
         h->client->ops->link_up(h->handle, h->hw->queue_up);
      }
      ret = 0;
   }
   de_mutex_unlock(&h->hw->mutex);
   return ret;
}

int nanoc_chip_down(struct nanoc_context *handle)
{
   struct hw_handle *h = find_hw_handle(handle);
   int ret;
   de_mutex_lock(&h->hw->mutex);
   ret = nanoc_hw_chip_down(h->hw);
   if (ret > 0) {
      /* There are other clients holding the chip up. But this client needs a link_down event anyway. */
      h->client->ops->link_down(h->handle, NANOC_QALL);
      ret = 0;
   }
   h->chip_up = 0;
   de_mutex_unlock(&h->hw->mutex);
   discard_queues(h);
   return ret;
}

/* can be called after first link_up event */
uint32_t nanoc_get_api_version(struct nanoc_context *handle)
{
   struct hw_handle *h = find_hw_handle(handle);
   return h->hw->api_version;
}


int
nanoc_req_coredump(struct nanoc_context *handle)
{
   if(coredump_enable)
   {
      struct nanoc_hw *hw = find_hw_handle(handle)->hw;
      nanoc_commit_suicide(hw);
   }
   return 0;
}

/* Special handling for commit suicide. This message will have priority over all other messages */
void nanoc_commit_suicide(struct nanoc_hw *hw)
{
   de_mutex_lock(&hw->mutex);

   if ((hw->flags & NANOC_HW_RESTARTING) == 0) {

      hw->flags |= (NANOC_HW_RESTARTING | NANOC_HW_COMMIT_SUICIDE);

      coredump_start_timer(hw);
      
      if (hw->flags & NANOC_HW_IDLE) {
         /* Chip is sleeping. Must wait for a wakeup.  */
         nanoc_link_down(hw, NANOC_QALL);
         if ((hw->flags & NANOC_HW_WAKEUP) == 0) {
            de_info(DE_TR_NANOC, "Wakeup");
            hw->flags |= NANOC_HW_WAKEUP;
            hw->ops->hw_wakeup(hw, 1);
         }
      } else {
         unsigned all_queues_full = 1;
         unsigned i;
         for (i = 0; i < NANOC_NUM_QS; i++) {
            if (hw->tx_win[i].used < hw->tx_win[i].max) {
               all_queues_full = 0;
               break;
            }
         }
         if (all_queues_full) {
            /* Need to wait for a tx confirm. */
            de_info(DE_TR_NANOC, "Commit suicide queued");
            nanoc_link_down(hw, NANOC_QALL);
         } else {
            /* Clear to send. */
            coredump_commit_suicide(hw);
         }
      }
   }
   de_mutex_unlock(&hw->mutex);
}

int nanoc_alloc_net_id(struct nanoc_context * handle)
{
   struct hw_handle *hw_handle = find_hw_handle(handle);
   unsigned i;
   int ret = -1;

   de_mutex_lock(&nanoc_mutex);
   for (i = NRX_NET_ID_FIRST_DYN; i < MAX_NUM_NET_IDS; i++)
   {
      if (hw_handle_by_net_id[i] == NULL) {
         hw_handle_by_net_id[i] = hw_handle;
         ret = i;
         goto done;
      }
   }
   de_error(DE_TR_NANOC, "Max num of net_ids reached");
done:
   de_mutex_unlock(&nanoc_mutex);
   return ret;
}

int nanoc_alloc_fixed_net_id(unsigned net_id, struct nanoc_context *handle)
{
   struct hw_handle *hw_handle = find_hw_handle(handle);
   int ret = -1;

   de_mutex_lock(&nanoc_mutex);
   if (hw_handle_by_net_id[net_id] == NULL) {
      hw_handle_by_net_id[net_id] = hw_handle;
      ret = 0;
      goto done;
   }
   de_error(DE_TR_NANOC, "Trying to allocate fixed net_id %d which is already allocated.", net_id);
done:
   de_mutex_unlock(&nanoc_mutex);
   return ret;
}

void nanoc_free_net_id(struct nanoc_context *handle, unsigned net_id)
{
   struct hw_handle *hw_handle = find_hw_handle(handle);
   DE_BUG_ON(hw_handle == NULL);
   hw_handle_by_net_id[net_id] = NULL;
}

/* Global functions ================================= */

void
nanoc_event(struct nanoc_hw *hw, void (*event_func)(struct hw_handle *, unsigned), unsigned arg)
{
   struct hw_handle *h;
   unsigned i;

   for (h=hw_handle_array, i=0; i<MAX_NUM_HW_HANDLES; h++, i++) {
      if (h->handle && h->chip_up && h->hw == hw) {
         event_func(h, arg);
      }
   }
}

void
nanoc_ev_link_up(struct hw_handle *h, unsigned qmask)
{
   de_info(DE_TR_NANOC, "h=%p qmask=%#x", h, qmask);
   if (h->client->ops->link_up) {
      h->client->ops->link_up(h->handle, qmask);
   }
}

void
nanoc_ev_link_down(struct hw_handle *h, unsigned qmask)
{
   de_info(DE_TR_NANOC, "h=%p qmask=%#x", h, qmask);
   if (h->client->ops->link_down) {
      h->client->ops->link_down(h->handle, qmask);
   }
}

void
nanoc_ev_restart(struct hw_handle *h, unsigned arg)
{
   de_info(DE_TR_NANOC, "h=%p", h);
   discard_queues(h);
   if(h->client->ops->restart) {
      h->client->ops->restart(h->handle);
   }
}

void nanoc_recv_handler(struct nanoc_hw *hw, unsigned net_id, de_pdu_t *pdu)
{
   struct hw_handle  *h   = get_hw_handle(net_id);
   nrx_hic_message_t *hdr = de_pdu_data(pdu);

   uint8_t tx_win[NANOC_NUM_QS] = { 0, 0};

   if (h) {
      tx_win[NANOC_QDATA] = hdr->tx_window;
      tx_win[NANOC_QCMD]  = hdr->tx_window_non_wifi_data;
      de_mutex_lock(&hw->mutex);
      hw->tx_win[NANOC_QCMD].used  -= tx_win[NANOC_QCMD];
      hw->tx_win[NANOC_QDATA].used -= tx_win[NANOC_QDATA];
      de_mutex_unlock(&hw->mutex);

      de_debug(DE_TR_NANOC, "tx_window=%d tx_win[QDATA].used=%d",
               tx_win[NANOC_QDATA], hw->tx_win[NANOC_QDATA].used);
      de_debug(DE_TR_NANOC, "cmd_window=%d tx_win[QCMD].used=%d",
               tx_win[NANOC_QCMD], hw->tx_win[NANOC_QCMD].used);

      h->client->ops->recv_handler(h->handle, net_id, pdu);
      if ((hw->flags & (NANOC_HW_IDLE | NANOC_HW_WAKEUP | NANOC_HW_RESTARTING)) == 0) {
         send_queued_pdus(hw);
      }
   } else {
      de_warn(DE_TR_NANOC, "Discarding message due to invalid net_id (%u)", net_id);
   }

   if (MAC_API_PRIMITIVE_IS_CFM(hdr->id)) {
      de_mutex_lock(&hw->mutex);
      if (ifdown_enable && (hw->tx_win[NANOC_QCMD].used + hw->tx_win[NANOC_QDATA].used) == 0) {
         de_info(DE_TR_NANOC, "SET IDLE");
         hw->flags |= NANOC_HW_IDLE;
         nanoc_hw_if_down(hw);
      }
      de_mutex_unlock(&hw->mutex);
   }
}

void nanoc_wakeup_ind(struct nanoc_hw *hw)
{
   //DE_BUG_ON((hw->flags & (NANOC_HW_IDLE|NANOC_HW_WAKEUP)) == 0);
   de_info(DE_TR_NANOC, "hw->flags=%#x", hw->flags);
   hw->ops->hw_wakeup(hw, 0);
   hw->flags &= ~(NANOC_HW_IDLE|NANOC_HW_WAKEUP);
   send_queued_pdus(hw);
   de_info(DE_TR_NANOC, "CLEAR IDLE");
}

void nanoc_send_complete(struct nanoc_hw *hw, unsigned net_id, de_pdu_t *pdu)
{
   struct hw_handle *h= get_hw_handle(net_id);
   if (h) {
      h->client->ops->send_complete(h->handle, pdu, 0);
   } else {
      de_warn(DE_TR_NANOC, "Discarding message due to invalid net_id (%u)", net_id);
   }
}

void nanoc_new_device(struct nanoc_hw *hw)
{
   unsigned i;
   hw->queue_up = 0;
   de_mutex_init(&hw->mutex);
   de_mutex_lock(&nanoc_mutex);
   coredump_init(hw);
   for (i = 0; i < MAX_NUM_CLIENTS; i++)
   {
      if (client_array[i].ops != NULL) {
         struct nanoc_context * handle = client_array[i].ops->new_device(hw->dev,
                                                                         hw->device_descr,
                                                                         hw->mac_addr);
         add_hw_handle(handle, &client_array[i], hw);
      }
   }
   de_mutex_unlock(&nanoc_mutex);
}

void nanoc_del_device(struct nanoc_hw *hw)
{
   unsigned i;
   for (i=0; i<MAX_NUM_HW_HANDLES; i++) {
      if (hw_handle_array[i].handle != NULL) {
         if (hw_handle_array[i].hw == hw) {
            discard_queues(&hw_handle_array[i]);
            hw_handle_array[i].client->ops->del_device(hw_handle_array[i].handle);
            hw_handle_array[i].handle = NULL;
         }
      }
   }
}

void nanoc_link_down(struct nanoc_hw *hw, unsigned qmask)
{
   if ((hw->queue_up & qmask) != 0) {
      hw->queue_up &= ~qmask;
      nanoc_event(hw, nanoc_ev_link_down, qmask);
   }
}

void nanoc_link_up(struct nanoc_hw *hw, unsigned qmask)
{
   if ((hw->queue_up & qmask) != qmask) {
      hw->queue_up |= qmask;
      nanoc_event(hw, nanoc_ev_link_up, qmask);
   }
}

void nanoc_restart(struct nanoc_hw *hw)
{
   hw->tx_win[NANOC_QCMD].used  = 0;
   hw->tx_win[NANOC_QDATA].used = 0;
   nanoc_event(hw, nanoc_ev_restart, 0);
}

unsigned nanoc_get_num_bufs(struct nanoc_hw *hw, unsigned size)
{
   return (size+hw->tx_buf_size-1)/hw->tx_buf_size;
}

/* Local functions ================================= */

static struct hw_handle *get_hw_handle(unsigned net_id)
{
   if (net_id >= MAX_NUM_NET_IDS) {
      de_warn(DE_TR_NANOC, "Incorrect net_id received: %d", net_id);
      return NULL;
   }
   return hw_handle_by_net_id[net_id];
}

static struct hw_handle *find_hw_handle(struct nanoc_context *handle)
{
   unsigned i;

   for (i = 0; i < MAX_NUM_HW_HANDLES; i++)
   {
      if (hw_handle_array[i].handle == handle) {
         return &hw_handle_array[i];
      }
   }
   de_error(DE_TR_NANOC, "Context %p could not be found!", handle);
   DE_BUG_ON(1);
   return NULL;
}

static unsigned add_hw_handle(struct nanoc_context *handle,
                              struct nanoc_client * client,
                              struct nanoc_hw *hw)
{
   unsigned i;
   for (i = 0; i < MAX_NUM_HW_HANDLES; i++)
   {
      if (hw_handle_array[i].handle == NULL) {
         hw_handle_array[i].handle = handle;
         hw_handle_array[i].hw     = hw;
         hw_handle_array[i].client = client;
         return 0;
      }
   }
   de_error(DE_TR_NANOC, "Max num of hw_handles reached");
   return -1;
}

static void send_queued_pdus(struct nanoc_hw *hw)
{
   unsigned i, qidx;

   de_mutex_lock(&hw->mutex);

   if (hw->flags & NANOC_HW_COMMIT_SUICIDE) {
      unsigned all_queues_full = 1;
      unsigned i;
      for (i = 0; i < NANOC_NUM_QS; i++) {
         if (hw->tx_win[i].used < hw->tx_win[i].max) {
            all_queues_full = 0;
            break;
         }
      }
      if (!all_queues_full) {
         coredump_commit_suicide(hw); /* We might send this several time, but that doesn't matter */
      }
   } else {
      for (qidx = NANOC_QCMD; qidx <= NANOC_QDATA; qidx++) {
         unsigned data_queued = 0;

         for (i = 0; i < MAX_NUM_HW_HANDLES; i++) {
            if (hw_handle_array[i].handle != NULL
                && hw_handle_array[i].hw == hw)
            {
               struct hw_handle    *h      = &hw_handle_array[i];
               struct nanoc_client *client = h->client;

               if (client->ops != NULL && h->queue[qidx] != NULL) {
                  unsigned num_bufs = nanoc_get_num_bufs(hw, de_pdu_size(h->queue[qidx]));
                  if (within_tx_window(qidx, hw, num_bufs)) {
                     set_pdu_window_size(h->queue[qidx], qidx, num_bufs);
                     hw->tx_win[qidx].used += num_bufs;
                     de_debug(DE_TR_NANOC, "num_bufs=%d tx_win[0].used=%d tx_win[1].used=%d", num_bufs, hw->tx_win[NANOC_QCMD].used, hw->tx_win[NANOC_QDATA].used);
                     nanoc_hw_send(hw, h->queue[qidx]);
                     de_info(DE_TR_NANOC, "Send from queue@%d", qidx);
                     h->queue[qidx] = NULL;
                  } else {
                     de_info(DE_TR_NANOC, "No room queue@%d", qidx);
                     if (qidx == NANOC_QDATA) {
                        data_queued = 1;
                     }
                  }
               }
            }
         }
         if (!data_queued && hw->tx_win[qidx].used < hw->tx_win[qidx].max) {
            nanoc_link_up(hw, 1<<qidx);
         }
      }
   }
   de_mutex_unlock(&hw->mutex);
}

static void discard_queues(struct hw_handle *h)
{
   unsigned i;
   de_pdu_t *pdu[NANOC_NUM_QS];

   de_mutex_lock(&h->hw->mutex);
   for (i = 0; i < NANOC_NUM_QS; i++) {
      pdu[i] = h->queue[i];
      h->queue[i] = NULL;
   }
   de_mutex_unlock(&h->hw->mutex);
   for (i = 0; i < NANOC_NUM_QS; i++) {
      if (pdu[i] != NULL) {
         h->client->ops->send_complete(h->handle, pdu[i], 1);
      }
   }
}
