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

#include "nanocore.h"
#include "driverenv.h"
#include "m80211-stddefs.h"

#define PACKET_MAX 4000  /* An AMSDU can be 3939 bytes */

#define DE_MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

struct type_id_def {
   unsigned int table_size;
   const char *const table[];
};

/* BEGIN GENERATED CODE */

static const struct type_id_def AUDIO_cfm_table = {
   6,
   {
      "NRX_AUDIO_SETUP_CFM",
      "NRX_AUDIO_SHUTDOWN_CFM",
      "NRX_AUDIO_REC_START_CFM",
      "NRX_AUDIO_REC_STOP_CFM",
      "NRX_AUDIO_CTRL_CFM",
      "NRX_AUDIO_PLAY_CFM"
   }
};

static const struct type_id_def AUDIO_ind_table = {
   1,
   {
      "NRX_AUDIO_REC_IND"
   }
};

static const struct type_id_def AUDIO_req_table = {
   6,
   {
      "NRX_AUDIO_SETUP_REQ",
      "NRX_AUDIO_SHUTDOWN_REQ",
      "NRX_AUDIO_REC_START_REQ",
      "NRX_AUDIO_REC_STOP_REQ",
      "NRX_AUDIO_CTRL_REQ",
      "NRX_AUDIO_PLAY_REQ"
   }
};

static const struct type_id_def BTHCI_cfm_table = {
   1,
   {
      "HIC_MAC_BTHCI_CFM"
   }
};

static const struct type_id_def BTHCI_ind_table = {
   1,
   {
      "HIC_MAC_BTHCI_IND"
   }
};

static const struct type_id_def BTHCI_req_table = {
   1,
   {
      "HIC_MAC_BTHCI_REQ"
   }
};

static const struct type_id_def CTRL_cfm_table = {
   10,
   {
      "NRX_CTRL_SET_ALIGNMENT_CFM",
      "NRX_CTRL_VERSION_CFM",
      "NRX_CTRL_INIT_COMPLETED_CFM",
      "UNKNOWN_3",
      "NRX_CTRL_SCB_ERROR_CFM",
      "NRX_CTRL_SLEEP_FOREVER_CFM",
      "UNKNOWN_6",
      "NRX_CTRL_RESERVED_CFM",
      "NRX_CTRL_CLOCK_SYNC_CFM",
      "NRX_CTRL_ECHO_CFM"
   }
};

static const struct type_id_def CTRL_ind_table = {
   4,
   {
      "NRX_CTRL_WAKEUP_IND",
      "NRX_CTRL_SCB_ERROR_IND",
      "NRX_CTRL_RESERVED_IND",
      "NRX_CTRL_NULL_IND"
   }
};

static const struct type_id_def CTRL_req_table = {
   10,
   {
      "NRX_CTRL_SET_ALIGNMENT_REQ",
      "NRX_CTRL_VERSION_REQ",
      "NRX_CTRL_INIT_COMPLETED_REQ",
      "NRX_CTRL_INTERFACE_DOWN_REQ",
      "NRX_CTRL_SCB_ERROR_REQ",
      "NRX_CTRL_SLEEP_FOREVER_REQ",
      "NRX_CTRL_COMMIT_SUICIDE_REQ",
      "NRX_CTRL_RESERVED_REQ",
      "NRX_CTRL_CLOCK_SYNC_REQ",
      "NRX_CTRL_ECHO_REQ"
   }
};

static const struct type_id_def DEBUG_cfm_table = {
   11,
   {
      "NRX_DEBUG_SETUP_CFM",
      "NRX_DEBUG_CHANNEL_CFM",
      "NRX_DEBUG_TX_CFM",
      "NRX_DEBUG_RX_STATS_CFM",
      "NRX_DEBUG_RX_STATS_CLEAR_CFM",
      "NRX_DEBUG_POKE_CFM",
      "NRX_DEBUG_TX_PWR_CORR_CFM",
      "NRX_DEBUG_POWER_CYCLE_START_CFM",
      "NRX_DEBUG_POWER_CYCLE_STOP_CFM",
      "NRX_DEBUG_AFC_CFM",
      "NRX_DEBUG_RF_TX_PWR_CORR_CFM"
   }
};

static const struct type_id_def DEBUG_ind_table = {
   2,
   {
      "NRX_DEBUG_TX_IND",
      "NRX_DEBUG_LOG_IND"
   }
};

static const struct type_id_def DEBUG_req_table = {
   11,
   {
      "NRX_DEBUG_SETUP_REQ",
      "NRX_DEBUG_CHANNEL_REQ",
      "NRX_DEBUG_TX_REQ",
      "NRX_DEBUG_RX_STATS_REQ",
      "NRX_DEBUG_RX_STATS_CLEAR_REQ",
      "NRX_DEBUG_POKE_REQ",
      "NRX_DEBUG_TX_PWR_CORR_REQ",
      "NRX_DEBUG_POWER_CYCLE_START_REQ",
      "NRX_DEBUG_POWER_CYCLE_STOP_REQ",
      "NRX_DEBUG_AFC_REQ",
      "NRX_DEBUG_RF_TX_PWR_CORR_REQ"
   }
};

static const struct type_id_def DLM_cfm_table = {
   2,
   {
      "NRX_DLM_LOAD_CFM",
      "NRX_DLM_LOAD_FAILED_CFM"
   }
};

static const struct type_id_def DLM_ind_table = {
   1,
   {
      "NRX_DLM_SWAP_IND"
   }
};

static const struct type_id_def DLM_req_table = {
   2,
   {
      "NRX_DLM_LOAD_REQ",
      "NRX_DLM_LOAD_FAILED_REQ"
   }
};

static const struct type_id_def FRAME_cfm_table = {
   3,
   {
      "NRX_FRAME_TX_CFM",
      "NRX_FRAME_CONFIG_CFM",
      "NRX_FRAME_COPY_CFM"
   }
};

static const struct type_id_def FRAME_ind_table = {
   2,
   {
      "NRX_FRAME_RX_IND",
      "NRX_FRAME_STREAM_CTRL_IND"
   }
};

static const struct type_id_def FRAME_req_table = {
   3,
   {
      "NRX_FRAME_TX_REQ",
      "NRX_FRAME_CONFIG_REQ",
      "NRX_FRAME_COPY_REQ"
   }
};

static const struct type_id_def MIB_cfm_table = {
   4,
   {
      "NRX_MIB_GET_CFM",
      "NRX_MIB_SET_CFM",
      "NRX_MIB_SET_TRIGGER_CFM",
      "NRX_MIB_REMOVE_TRIGGER_CFM"
   }
};

static const struct type_id_def MIB_ind_table = {
   1,
   {
      "NRX_MIB_TRIGGER_IND"
   }
};

static const struct type_id_def MIB_req_table = {
   4,
   {
      "NRX_MIB_GET_REQ",
      "NRX_MIB_SET_REQ",
      "NRX_MIB_SET_TRIGGER_REQ",
      "NRX_MIB_REMOVE_TRIGGER_REQ"
   }
};

static const struct type_id_def MLME_cfm_table = {
   22,
   {
      "NRX_MLME_NET_CREATE_CFM",
      "NRX_MLME_NET_DESTROY_CFM",
      "NRX_MLME_START_CFM",
      "NRX_MLME_STOP_CFM",
      "NRX_MLME_POWER_MGMT_CFM",
      "NRX_MLME_SET_KEY_CFM",
      "NRX_MLME_UPDATE_TIM_CFM",
      "NRX_MLME_SET_BSS_IE_CFM",
      "NRX_MLME_SET_STA_INFO_CFM",
      "NRX_MLME_DELETE_STA_CFM",
      "NRX_MLME_SET_BSSCONF_CFM",
      "NRX_MLME_GET_BSSCONF_CFM",
      "NRX_MLME_CONFIG_TX_CFM",
      "NRX_MLME_CRITICAL_SESSION_CFM",
      "NRX_MLME_ADD_BAA_CFM",
      "NRX_MLME_DEL_BAA_CFM",
      "NRX_MLME_P2P_PS_SET_CONF_CFM",
      "NRX_MLME_P2P_PS_GET_CONF_CFM",
      "NRX_MLME_GET_COUNTERS_CFM",
      "NRX_MLME_SET_BEACON_IE_FILTER_CFM",
      "NRX_MLME_TX_STATISTICS_CFM",
      "NRX_MLME_GET_STA_STATISTICS_CFM"
   }
};

static const struct type_id_def MLME_ind_table = {
   4,
   {
      "NRX_MLME_P2P_PS_NET_IND",
      "NRX_MLME_BAA_EXCEPTION_IND",
      "NRX_MLME_PEER_STATUS_IND",
      "NRX_MLME_TX_STATISTICS_IND"
   }
};

static const struct type_id_def MLME_req_table = {
   22,
   {
      "NRX_MLME_NET_CREATE_REQ",
      "NRX_MLME_NET_DESTROY_REQ",
      "NRX_MLME_START_REQ",
      "NRX_MLME_STOP_REQ",
      "NRX_MLME_POWER_MGMT_REQ",
      "NRX_MLME_SET_KEY_REQ",
      "NRX_MLME_UPDATE_TIM_REQ",
      "NRX_MLME_SET_BSS_IE_REQ",
      "NRX_MLME_SET_STA_INFO_REQ",
      "NRX_MLME_DELETE_STA_REQ",
      "NRX_MLME_SET_BSSCONF_REQ",
      "NRX_MLME_GET_BSSCONF_REQ",
      "NRX_MLME_CONFIG_TX_REQ",
      "NRX_MLME_CRITICAL_SESSION_REQ",
      "NRX_MLME_ADD_BAA_REQ",
      "NRX_MLME_DEL_BAA_REQ",
      "NRX_MLME_P2P_PS_SET_CONF_REQ",
      "NRX_MLME_P2P_PS_GET_CONF_REQ",
      "NRX_MLME_GET_COUNTERS_REQ",
      "NRX_MLME_SET_BEACON_IE_FILTER_REQ",
      "NRX_MLME_TX_STATISTICS_REQ",
      "NRX_MLME_GET_STA_STATISTICS_REQ"
   }
};

static const struct type_id_def NVM_cfm_table = {
   4,
   {
      "NRX_NVMEM_IDENTIFY_CFM",
      "NRX_NVMEM_ERASE_CFM",
      "NRX_NVMEM_WRITE_CFM",
      "NRX_NVMEM_READ_CFM"
   }
};

static const struct type_id_def NVM_req_table = {
   4,
   {
      "NRX_NVMEM_IDENTIFY_REQ",
      "NRX_NVMEM_ERASE_REQ",
      "NRX_NVMEM_WRITE_REQ",
      "NRX_NVMEM_READ_REQ"
   }
};

static const struct type_id_def SCAN_cfm_table = {
   2,
   {
      "NRX_SCAN_START_CFM",
      "NRX_SCAN_STOP_CFM"
   }
};

static const struct type_id_def SCAN_ind_table = {
   1,
   {
      "NRX_SCAN_READY_IND"
   }
};

static const struct type_id_def SCAN_req_table = {
   2,
   {
      "NRX_SCAN_START_REQ",
      "NRX_SCAN_STOP_REQ"
   }
};

/* END GENERATED CODE */


static struct {
   const struct type_id_def *req_table;
   const struct type_id_def *cfm_table;
   const struct type_id_def *ind_table;
} type_table[] = {
   [HIC_MESSAGE_TYPE_CTRL]    = { &CTRL_req_table,
                                  &CTRL_cfm_table,
                                  &CTRL_ind_table },
   [HIC_MESSAGE_TYPE_FRAME]   = { &FRAME_req_table,
                                  &FRAME_cfm_table,
                                  &FRAME_ind_table },
   [HIC_MESSAGE_TYPE_MLME]    = { &MLME_req_table,
                                  &MLME_cfm_table,
                                  &MLME_ind_table },
   [HIC_MESSAGE_TYPE_MIB]     = { &MIB_req_table,
                                  &MIB_cfm_table,
                                  &MIB_ind_table},
   [HIC_MESSAGE_TYPE_SCAN]    = { &SCAN_req_table,
                                  &SCAN_cfm_table,
                                  &SCAN_ind_table},
   [HIC_MESSAGE_TYPE_DLM]     = { &DLM_req_table,
                                  &DLM_cfm_table,
                                  &DLM_ind_table},
   [HIC_MESSAGE_TYPE_BTHCI]   = { &BTHCI_req_table,
                                  &BTHCI_cfm_table,
                                  &BTHCI_ind_table},
   [HIC_MESSAGE_TYPE_DEBUG]   = { &DEBUG_req_table,
                                  &DEBUG_cfm_table,
                                  &DEBUG_ind_table },
   [HIC_MESSAGE_TYPE_NVM]     = { &NVM_req_table,
                                  &NVM_cfm_table,
                                  NULL },
   [HIC_MESSAGE_TYPE_AUDIO]   = { &AUDIO_req_table,
                                  &AUDIO_cfm_table,
                                  &AUDIO_ind_table }
};


static const char *frame_types_mgmt[] = {
   /* 00 */ "ASSOCIATE-REQUEST",
   /* 10 */ "ASSOCIATE-RESPONSE",
   /* 20 */ "REASSOCIATE-REQUEST",
   /* 30 */ "REASSOCIATE-RESPONSE",
   /* 40 */ "PROBE-REQUEST",
   /* 50 */ "PROBE-RESPONSE",
   /* 60 */ NULL,
   /* 70 */ NULL,
   /* 80 */ "BEACON",
   /* 90 */ "ATIM",
   /* a0 */ "DISASSOCIATE",
   /* b0 */ "AUTHENTICATE",
   /* c0 */ "DEAUTHENTICATE",
   /* d0 */ "ACTION",
   /* e0 */ NULL,
   /* f0 */ NULL,
};

static const char *frame_types_ctrl[] = {
   /* 04 */ NULL,
   /* 14 */ NULL,
   /* 24 */ NULL,
   /* 34 */ NULL,
   /* 44 */ NULL,
   /* 54 */ NULL,
   /* 64 */ NULL,
   /* 74 */ NULL,
   /* 84 */ "BLOCK-ACK-REQ",
   /* 94 */ "BLOCK-ACK",
   /* a4 */ "PS-POLL",
   /* b4 */ "RTS",
   /* c4 */ "CTS",
   /* d4 */ "ACK",
   /* e4 */ "CF-END",
   /* f4 */ "CF-END+CF-POLL",
};

static const char *frame_types_data[] = {
   /* 08 */ "DATA",
   /* 18 */ "DATA+CF-ACK",
   /* 28 */ "DATA+CF-POLL",
   /* 38 */ "DATA+CF-ACK+CF-POLL",
   /* 48 */ "NULL",
   /* 58 */ "CF-ACK",
   /* 68 */ "CF-POLL",
   /* 78 */ "CF-ACK+CF-POLL",
   /* 88 */ "QOS DATA",
   /* 98 */ "QOS+CF-ACK",
   /* a8 */ "QOS+CF-POLL",
   /* b8 */ "QOS+CF-ACK+CF-POLL",
   /* c8 */ "QOS-NULL",
   /* d8 */ NULL,
   /* e8 */ "CF-POLL",
   /* f8 */ "CF-ACK+CF-POLL",
};

static size_t
print_frame_type(uint16_t frame_control, char *str, size_t size)
{
   unsigned   frametype = M80211_GET_FTYPE(de_le16_to_cpu(frame_control));
   unsigned   subtype   = M80211_GET_STYPE(de_le16_to_cpu(frame_control));
   const char *n = NULL;

   switch (frametype) {
      case M80211_FRAMEGROUPTYPE_MGMT:
         n = frame_types_mgmt[subtype];
         break;
      case M80211_FRAMEGROUPTYPE_CTRL:
         n = frame_types_ctrl[subtype];
         break;
      case M80211_FRAMEGROUPTYPE_DATA:
         n = frame_types_data[subtype];
      default:
         break;
   }

   if (n != NULL) {
      return snprintf(str + strlen(str), size - strlen(str), ": %s", n);
   }

   return snprintf(str + strlen(str), size - strlen(str),
                   ": FC-%04x",
                   de_le16_to_cpu(frame_control));
}

#define x_isprintascii(c) ((c) >= 32 && (c) <= 126)

void
nanoc_printbuf(const char *prefix, const void *ptr, size_t len)
{
   size_t i, j;
   const unsigned char *p = ptr;
   char line[48+2+16+1];

   if (len > 128)
      len = 128;

   for (i = 0; i < len; i += 16) {
      line[0] = '\0';

      for(j = 0; j < 16; j++) {
         if(i + j < len)
            snprintf(line + strlen(line), sizeof(line) - strlen(line),
                     "%02x ", p[i+j]);
         else
            snprintf(line + strlen(line), sizeof(line) - strlen(line), "   ");
      }

      snprintf(line + strlen(line), sizeof(line) - strlen(line), ": ");

      for (j = 0; j < 16; j++) {
         if (i + j < len) {
            if (x_isprintascii(p[i+j])) {
               snprintf(line + strlen(line), sizeof(line) - strlen(line), "%c", p[i+j]);
            } else {
               snprintf(line + strlen(line), sizeof(line) - strlen(line), ".");
            }
         }
      }
      de_print("%s %04zx: %s", prefix, i, line);
   }
}

void
nanoc_hiclog(const char *tag, de_pdu_t *pdu)
{
   char str[128];
   uint8_t id_index;
   size_t c = 0;
   const struct type_id_def *def = NULL;
   char *pkt = (char*)de_pdu_data(pdu)+2;
   size_t len = de_pdu_size(pdu);
   nrx_hic_message_t *hdr = (nrx_hic_message_t *)pkt;
   const size_t header_size = sizeof *hdr;

   if (len < header_size) {
      de_error(DE_TR_HIC, "Short packet (%zu bytes)", len);
      return;
   }

   c += snprintf(str + c, sizeof(str) - c, "%d,%x.%x [%u/%x]",
                 hdr->length,
                 hdr->type,
                 hdr->id,
                 hdr->net_id,
                 hdr->trans_id);
   c = DE_MIN(sizeof(str), c);

   if (hdr->length < header_size || hdr->length > PACKET_MAX) {
      de_error(DE_TR_HIC, "Bad sized packet %s", str);
      return;
   }

   switch (hdr->type) {
      case HIC_MESSAGE_TYPE_AGGREGATION:
         de_error(DE_TR_HIC, "Aggregated packet %s", str);
         return;
   }

   if (hdr->type >= ARRAY_SIZE(type_table)) {
      de_error(DE_TR_HIC, "Unknown message type %s", str);
      nanoc_printbuf(tag, pkt, len);
      return;
   }

   switch (hdr->id & MAC_API_PRIMITIVE_TYPE_BIT) {
      case MAC_API_PRIMITIVE_TYPE_REQ:
         def = type_table[hdr->type].req_table;
         break;
      case MAC_API_PRIMITIVE_TYPE_CFM:
         def = type_table[hdr->type].cfm_table;
         break;
      case MAC_API_PRIMITIVE_TYPE_IND:
         def = type_table[hdr->type].ind_table;
         break;
   }

   id_index = hdr->id & ~MAC_API_PRIMITIVE_TYPE_BIT;

   if (def == NULL || id_index >= def->table_size) {
      c += snprintf(str + c, sizeof(str) - c, ": UNKNOWN%d", id_index);
      c = DE_MIN(sizeof(str), c);
   } else {
      c += snprintf(str + c, sizeof(str) - c, ": %s", def->table[id_index]);
      c = DE_MIN(sizeof(str), c);
   }

   if (hdr->type == HIC_MESSAGE_TYPE_FRAME
       && (hdr->id == NRX_FRAME_TX_REQ || hdr->id == NRX_FRAME_COPY_REQ)
       && hdr->length >= (header_size + sizeof(nrx_frame_tx_req_t)))
   {
      nrx_frame_tx_req_t *req = (void*)(pkt + header_size);
      if (hdr->length >= header_size + req->payload_offset + sizeof(uint16_t)) {
         uint16_t frame_control = *(uint16_t*)(pkt + header_size + req->payload_offset);
         c += print_frame_type(frame_control, str, sizeof(str));
         c = DE_MIN(sizeof(str), c);
      }
   }

   if (hdr->type == HIC_MESSAGE_TYPE_FRAME
      && hdr->id == NRX_FRAME_RX_IND
      && hdr->length >= (header_size + sizeof(nrx_frame_rx_ind_t)))
   {
      nrx_frame_rx_ind_t *ind = (void*)(pkt + header_size);
      if(hdr->length >= header_size + ind->payload_offset + sizeof(uint16_t)) {
         uint16_t frame_control = *(uint16_t*)(pkt + header_size + ind->payload_offset);
         c += print_frame_type(frame_control, str, sizeof(str));
         c = DE_MIN(sizeof(str), c);
      }
   }

   if (hdr->type == HIC_MESSAGE_TYPE_FRAME
       && hdr->id == NRX_FRAME_TX_CFM
       && hdr->length >= sizeof(nrx_hic_message_t) + sizeof(nrx_frame_tx_cfm_t))
   {
      nrx_frame_tx_cfm_t *cfm = (void*)(pkt + header_size);
      c += snprintf(str + c, sizeof(str) - c, " s%ur%xp%x",
                    cfm->status, cfm->rate_used, cfm->prio);
      c = DE_MIN(sizeof(str), c);
   }

   if (de_trace_enabled(DE_INFO, DE_TR_HIC)) {
      de_print("%s: %s", tag, str);
   }
   if (de_trace_enabled(DE_DEBUG, DE_TR_HIC)) {
      nanoc_printbuf(tag, pkt, len);
   }
}
