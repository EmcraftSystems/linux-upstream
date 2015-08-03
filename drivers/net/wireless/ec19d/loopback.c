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
#include "loopback.h"

// Definitions ================================

#define NUM_OUTSTANDING_ECHOS 3
#define LOOPBACK_TIMEOUT      100  /* ms */

static unsigned get_q(struct nanoc_context * context, unsigned net_id, de_pdu_t *pdu);
static void recv_handler(struct nanoc_context * context, unsigned net_id, de_pdu_t *pdu);
static void send_complete(struct nanoc_context * context, de_pdu_t *pdu, int result);
static struct nanoc_context * new_device(void *hwdev, const struct nanoc_device_descr*, unsigned char *mac_addr);
static void del_device(struct nanoc_context *context);
static void link_up(struct nanoc_context *context, unsigned qmask);
static void link_down(struct nanoc_context *context, unsigned qmask);
static void restart(struct nanoc_context *context);
static void lb_timer_cb(de_timer_t *timer);

// Variables   ================================

static struct nanoc_ops lb_ops =
{
   .get_q         = get_q,
   .recv_handler  = recv_handler,
   .send_complete = send_complete,
   .new_device    = new_device,
   .del_device    = del_device,
   .link_up       = link_up,
   .link_down     = link_down,
   .restart       = restart
};

struct nanoc_context {
   unsigned  allocated;
   unsigned  chip_up;
   unsigned  net_id;
   int       link_up;
   unsigned  trans_id;
   unsigned  start_time;
   unsigned  tx_bytes;
   unsigned  rx_bytes;
   unsigned  queue_was_full;
} lb_context;

unsigned    lb_running;
de_timer_t  lb_timer;
de_mutex_t  lb_mutex = DE_MUTEX_INIT(lb_mutex);

/* Module parameters ================================= */

int loopback_size = 0;
DE_MODULE_PARAM(loopback_size, int, "Size of loopback message. 0 disables loopback.");
int loopback_num_errors = 0;
DE_MODULE_PARAM_RO(loopback_num_errors, int, "Number of errors detected on received data");
int loopback_num_echoes = 0;
DE_MODULE_PARAM_RO(loopback_num_echoes, int, "Number of echo frames sent");
int loopback_tx_kbytes = 0;
DE_MODULE_PARAM_RO(loopback_tx_kbytes, int, "Number of kilobytes sent");
int loopback_rx_kbytes = 0;
DE_MODULE_PARAM_RO(loopback_rx_kbytes, int, "Number of kilobytes received");
int loopback_tx_kbitrate = 0;
DE_MODULE_PARAM_RO(loopback_tx_kbitrate, int, "Tx rate in kilobites per second");
int loopback_rx_kbitrate = 0;
DE_MODULE_PARAM_RO(loopback_rx_kbitrate, int, "Rx rate in kilobites per second");
int loopback_clear_stat;
DE_MODULE_PARAM_CB(loopback_clear_stat, loopback_get_clear_stat, loopback_set_clear_stat, "Write to this to reset all loopback statistics");

// Global functions ===========================

void loopback_init(void)
{
   nanoc_register(&lb_ops);
   de_timer_init(&lb_timer, lb_timer_cb);
}

void loopback_set_clear_stat(const char *val)
{
   de_info(DE_TR_NANOC, "Resetting loopback statistics");

   lb_context.start_time = de_now_msec();
   lb_context.rx_bytes = 0;
   lb_context.tx_bytes = 0;
   loopback_num_errors = 0;
   loopback_num_echoes = 0;
   loopback_tx_kbytes = 0;
   loopback_rx_kbytes = 0;
   loopback_tx_kbitrate = 0;
   loopback_rx_kbitrate = 0;
}

void loopback_get_clear_stat(char *buffer)
{
   de_warn(DE_TR_NANOC, "Reading clear stat does not have any effect");
}


// Local functions ============================

static void echo(void)
{
   int                   echo_size = loopback_size-NRX_HIC_FRAME_HEADER_SIZE;
   de_pdu_t              *pdu;
   nrx_ctrl_echo_req_t   *msg;
   nrx_hic_message_t     *hdr;

   de_mutex_lock(&lb_mutex);

   if (echo_size < 0) {
      echo_size = 0;
   }

   if (lb_context.link_up) {
      pdu = de_pdu_alloc(echo_size);
      msg = (nrx_ctrl_echo_req_t *)de_pdu_put(pdu, echo_size);

      hdr = (nrx_hic_message_t *) de_pdu_push(pdu, sizeof *hdr);
      hdr->length   = de_pdu_size(pdu);
      hdr->type     = HIC_MESSAGE_TYPE_CTRL;
      hdr->id       = NRX_CTRL_ECHO_REQ;
      hdr->flags    = 0;
      hdr->net_id   = lb_context.net_id;
      hdr->trans_id = lb_context.trans_id++;

      memset(((uint8_t *)&msg->data), (uint8_t)hdr->trans_id, echo_size);

      lb_context.tx_bytes += de_pdu_size(pdu)+2;  /* 2 for framing header */
      loopback_tx_kbytes += lb_context.tx_bytes / 1024;
      lb_context.tx_bytes %= 1024;
      loopback_num_echoes++;
      nanoc_send_data(lb_context.net_id, pdu);
   } else {
      lb_context.queue_was_full = 1;
   }
   de_mutex_unlock(&lb_mutex);
}

static unsigned get_q(struct nanoc_context * context, unsigned net_id, de_pdu_t *pdu)
{
   return NANOC_QDATA;
}

static void recv_handler(struct nanoc_context * context, unsigned net_id, de_pdu_t *pdu)
{
   nrx_hic_message_t   *hdr = (nrx_hic_message_t *)de_pdu_data(pdu);
   nrx_ctrl_echo_cfm_t *msg = (nrx_ctrl_echo_cfm_t *)&hdr[1];
   unsigned i;

   if (hdr->type == HIC_MESSAGE_TYPE_CTRL && hdr->id == NRX_CTRL_ECHO_CFM)
   {
      if (lb_running) {
         echo();
      }
      /* Verify content */
      for (i = 0; i < hdr->length - sizeof(nrx_hic_message_t); i++) {
         if (((uint8_t *)&msg->data)[i] != (uint8_t)hdr->trans_id) {
            loopback_num_errors++;
         }
      }
      lb_context.rx_bytes += de_pdu_size(pdu)+2;  /* 2 for framing header */
      loopback_rx_kbytes += lb_context.rx_bytes / 1024;
      lb_context.rx_bytes %= 1024;
   }
   de_pdu_free(pdu);
}

static void send_complete(struct nanoc_context * context, de_pdu_t *pdu, int result)
{
   DE_BUG_ON(context == NULL);
   de_pdu_free(pdu);
}

static struct nanoc_context * new_device(void *hwdev,
                                         const struct nanoc_device_descr *device_descr,
                                         unsigned char *mac_addr)
{
   if (lb_context.allocated) {
      de_warn(DE_TR_LOOPBACK, "Loopback only works on one device at a time");
      return NULL;
   }
   lb_context.allocated = 1;
   de_timer_start(&lb_timer, LOOPBACK_TIMEOUT);
   return &lb_context;
}

static void del_device(struct nanoc_context *context)
{
   if (context) {
      context->allocated = 0;
      lb_context.chip_up = 0;
      nanoc_free_net_id(&lb_context, lb_context.net_id);
      lb_running = 0;
      loopback_set_clear_stat(0);
      de_timer_stop(&lb_timer);
   }
}

static void link_up(struct nanoc_context *context, unsigned qmask)
{
   DE_BUG_ON(context == NULL);
   if (qmask & NANOC_QDATA) {
      context->link_up = 1;
      de_info(DE_TR_LOOPBACK, "Link is up");
   }
}

static void link_down(struct nanoc_context *context, unsigned qmask)
{
   DE_BUG_ON(context == NULL);
   if (qmask & NANOC_QDATA) {
      context->link_up = 0;
      de_warn(DE_TR_LOOPBACK, "Should never receive link_down when running loopback test");
   }
}

static void restart(struct nanoc_context *context)
{
   DE_BUG_ON(context == NULL);
}

static void lb_timer_cb(de_timer_t *timer)
{
   unsigned i;

   if (lb_context.allocated && !lb_context.chip_up) {
      nanoc_chip_up(&lb_context, NANOC_FEAT_WIFI_STA);
      lb_context.net_id = nanoc_alloc_net_id(&lb_context);
      lb_context.chip_up = 1;
   }
   /* Update measured values */
   if (lb_running) {
      loopback_tx_kbitrate = 8*loopback_tx_kbytes*1000
                                /(de_now_msec()-lb_context.start_time);
      loopback_rx_kbitrate = 8*loopback_rx_kbytes*1000
                                /(de_now_msec()-lb_context.start_time);
   }

   if (lb_context.queue_was_full) {
      /* Loopback was starved out (by other client). */
      lb_context.queue_was_full = 0;
      for (i = 0; i < NUM_OUTSTANDING_ECHOS; i++) {
         echo();
      }
   }
   if (!lb_running && loopback_size != 0) {
      /* Send echoes */
      lb_context.start_time = de_now_msec();
      lb_running = 1;
      for (i = 0; i < NUM_OUTSTANDING_ECHOS; i++) {
         echo();
      }
   }
   if (lb_running && loopback_size == 0) {
      lb_running = 0;
   }
   de_timer_start(&lb_timer, LOOPBACK_TIMEOUT);
}
