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
#include "hic_pcap.h"
#include "mac_api.h"
#include "mac_api_debug.h"

/* Definitions ====================================== */

#define PCAP_MAGIC 0xa1b2c3d4
#define PCAP_MAJOR 2
#define PCAP_MINOR 4
#define PCAP_LINKTYPE 148  /* USER1 */

#define DE_MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

#define SNAPLEN 0xffff
#if SNAPLEN < 16
#error pointlessly low SNAPLEN
#endif

struct pcap_file_header_t
{
   uint32_t magic;
   uint16_t major;
   uint16_t minor;
   uint32_t timezone;
   uint32_t time_accuracy;
   uint32_t snap_len;
   uint32_t linktype;
};

struct pcap_hdr_t
{
   uint32_t tv_sec;
   uint32_t tv_usec;
   uint32_t caplen;
   uint32_t len;
};

/* Variables ======================================== */

static unsigned hic_pcap_started;
static de_mutex_t mutex = DE_MUTEX_INIT(mutex);

/* Global functions ==================================== */

void hic_pcap_start(void)
{
   de_pdu_t *pdu = de_pdu_alloc(sizeof(struct pcap_file_header_t));
   struct pcap_file_header_t *hdr = de_pdu_put(pdu, sizeof(struct pcap_file_header_t));
   hdr->magic = PCAP_MAGIC;
   hdr->major = PCAP_MAJOR;
   hdr->minor = PCAP_MINOR;
   hdr->timezone = 0;
   hdr->time_accuracy = 0;
   hdr->snap_len = SNAPLEN;
   hdr->linktype = PCAP_LINKTYPE;

   de_info(DE_TR_NANOC, "Starting hic pcap logging");

   de_mutex_lock(&mutex);
   hic_pcap_started = 1;
   hic_pcap_hdlr_write(pdu);
   de_mutex_unlock(&mutex);
}

void hic_pcap_stop(void)
{
   de_info(DE_TR_NANOC, "Stopping hic pcap logging");
   hic_pcap_started = 0;
}

void hic_pcap_add_pdu(de_pdu_t *pdu)
{
   de_pdu_t *pcap_data;
   struct pcap_hdr_t *hdr;
   struct de_time now;

   size_t cap_len;

   if (!hic_pcap_started)
      return;

   de_now(&now);

   cap_len = DE_MIN(de_pdu_size(pdu), SNAPLEN);

   pcap_data = de_pdu_alloc(sizeof(*hdr) + cap_len);
   if(pcap_data == NULL)
      return;

   hdr = de_pdu_put(pcap_data, sizeof(*hdr));

   memcpy(de_pdu_put(pcap_data, cap_len), de_pdu_data(pdu), cap_len);

   /* PCAP header */
   hdr->tv_sec  = now.sec;
   hdr->tv_usec = now.usec;
   hdr->caplen = cap_len;
   hdr->len = de_pdu_size(pdu);

   de_mutex_lock(&mutex);
   hic_pcap_hdlr_write(pcap_data);
   de_mutex_unlock(&mutex);
}

struct debug_log_header {
   nrx_hic_frame_t      hdr;
   nrx_debug_log_ind_t  log;
   uint32_t             size;
};

static void hic_pcap_debug_log_int(de_pdu_t *pdu,
                                   unsigned int id,
                                   unsigned int value_1,
                                   unsigned int value_2)
{
   struct debug_log_header *dlh;
   dlh = (struct debug_log_header*)de_pdu_data(pdu);

   dlh->hdr.size = dlh->hdr.control.length = de_pdu_size(pdu) - 2;
   dlh->hdr.control.type = HIC_MESSAGE_TYPE_DEBUG;
   dlh->hdr.control.id = NRX_DEBUG_LOG_IND;
   dlh->hdr.control.flags = 0;
   dlh->hdr.control.net_id = 0;
   dlh->hdr.control.trans_id = 0;
   dlh->hdr.control.tx_window_non_wifi_data = 0;
   dlh->hdr.control.tx_window = 0;
   dlh->log.id = 0xfb000000 | (id & 0xffffff);
   dlh->log.value_1 = value_1;
   dlh->log.value_2 = value_2;
   dlh->size = de_pdu_size(pdu) - sizeof(*dlh);

   hic_pcap_add_pdu(pdu);
}

void hic_pcap_debug_log_pdu(unsigned int id,
                            unsigned int value_1,
                            unsigned int value_2,
                            de_pdu_t *pdu,
                            unsigned int snaplen)
{
   size_t orig_len;
   de_pdu_t *tmp;

   orig_len = de_pdu_size(pdu);
   if(snaplen == 0)
      snaplen = orig_len;

   if(de_pdu_headroom(pdu) < sizeof(struct debug_log_header)) {
      tmp = de_pdu_alloc(sizeof(struct debug_log_header) + snaplen);
      if(tmp == NULL)
         return;
      de_pdu_put(pdu, sizeof(struct debug_log_header));
      memcpy(de_pdu_put(tmp, snaplen), de_pdu_data(pdu), snaplen);
   } else {
      if(de_pdu_size(pdu) > snaplen)
         de_pdu_trim(pdu, snaplen);
      de_pdu_push(pdu, sizeof(struct debug_log_header));
      tmp = pdu;
   }

   hic_pcap_debug_log_int(tmp, id, value_1, value_2);

   if(tmp == pdu) {
      de_pdu_pull(pdu, sizeof(struct debug_log_header));
      de_pdu_trim(pdu, orig_len);
   } else {
      de_pdu_free(tmp);
   }
}

void hic_pcap_debug_log_data(unsigned int id,
                             unsigned int value_1,
                             unsigned int value_2,
                             const void *data,
                             size_t len)
{
   de_pdu_t *pdu;

   pdu = de_pdu_alloc(sizeof(struct debug_log_header) + len);
   if(pdu == NULL)
      return;
   de_pdu_put(pdu, sizeof(struct debug_log_header));
   memcpy(de_pdu_put(pdu, len), data, len);

   hic_pcap_debug_log_int(pdu, id, value_1, value_2);

   de_pdu_free(pdu);
}
