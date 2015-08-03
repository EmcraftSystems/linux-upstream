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

#include "smd_common.h"

static int smd_monitor_update_scanjob(struct smd_hw_priv *sh);

int
resource_request_release_scan(struct smd_hw_priv *sh,
                              unsigned int expected,
                              unsigned int requested)
{
   int ret;

   if(sh == NULL)
      return -EINVAL;

   de_debug(DE_TR_WIFI, "current=%u, expected=%u, requested=%u",
            sh->sh_scan_type, expected, requested);

   spin_lock_bh(&sh->sh_scan_lock);
   if(sh->sh_scan_type == expected) {
      sh->sh_scan_type = requested;
      ret = 0;
   } else {
      ret = -EBUSY;
   }
   spin_unlock_bh(&sh->sh_scan_lock);
   return ret;
}

int
smd_scan_single_passive_channel(struct smd_hw_priv *sh,
                                const mac_chn_prop_t *channel,
                                uint32_t scan_flags)
{
   struct smd_transaction *stx;
   nrx_scan_start_req_t *req;
   nrx_scan_start_cfm_t *cfm;
   nrx_tlv_head_t *tlv;
   size_t tlv_size = ALIGN(sizeof(*tlv) + sizeof(*channel), 4);
   int status;

   req = smd_alloc_req(sh, sizeof(*req) + tlv_size,
                       HIC_MESSAGE_TYPE_SCAN, NRX_SCAN_START_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   memset(req, 0, sizeof(*req));

   req->flags = scan_flags;
   req->tlv_size = tlv_size;

   /* Add TLV header */
   tlv = (nrx_tlv_head_t *) &req[1];
   tlv->type = NRX_TLV_TYPE_CHN_PROP_LIST;
   tlv->len  = sizeof(*channel);

   /* populate TLV */
   memcpy(&tlv[1], channel, sizeof(*channel));

   /* Issue SCAN_START request */
   status = smd_wait_transaction(stx);
   if(status == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->status != NRX_SUCCESS) {
         de_error(DE_TR_WIFI, "FAILED: nrx_scan_start_req: %d", cfm->status);
         status = -EIO;
      }
   }
   smd_txn_decref(stx);

   return status;
}

int
smd_stop_scan_job(struct smd_hw_priv *sh)
{
   struct smd_transaction *stx;
   nrx_scan_stop_req_t *req;
   nrx_scan_stop_cfm_t *cfm;
   int status;

   req = smd_alloc_req(sh, sizeof(*req),
                       HIC_MESSAGE_TYPE_SCAN, NRX_SCAN_STOP_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   memset(req, 0, sizeof(*req));

   status = smd_wait_transaction(stx);
   if(status == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->status != NRX_SUCCESS) {
         de_error(DE_TR_WIFI, "FAILED: nrx_scan_stop_req: %d", cfm->status);
         status = -EIO;
      }
   }
   smd_txn_decref(stx);
   return status;
}


int smd_monitor_mode_enable(struct smd_hw_priv *sh)
{
   int status;

   if(smd_monitor_mode_enabled(sh))
      return 0;

   status = resource_request_scan(sh, SMD_FLAG_SCAN_MONITOR);
   if(status != 0)
      return status;

   return smd_monitor_update_scanjob(sh);
}

bool smd_monitor_mode_enabled(struct smd_hw_priv *sh)
{
   return resource_test_scan(sh, SMD_FLAG_SCAN_MONITOR) == 0;
}

int smd_monitor_mode_disable(struct smd_hw_priv *sh)
{
   if(!smd_monitor_mode_enabled(sh))
      return 0;

   smd_monitor_stop_scanjob(sh);

   return resource_release_scan(sh, SMD_FLAG_SCAN_MONITOR) == 0;
}

static int smd_monitor_update_scanjob(struct smd_hw_priv *sh)
{
   int status;

   if(!smd_monitor_mode_enabled(sh))
      return -EINVAL;

   smd_monitor_stop_scanjob(sh);

   if(sh->sh_monitor_channel.props.freq == 0) {
      de_info(DE_TR_WIFI, "no channel");
      return -EINVAL;
   }

   sh->sh_monitor_channel.props.tx_dbm = 20; /* XXX does this matter in any way */
   sh->sh_monitor_channel.props.flags |= NRX_MAC_CHN_FLAG_PASSIVE_SCAN;

   status = smd_scan_single_passive_channel(sh,
                                            &sh->sh_monitor_channel,
                                            NRX_SCAN_FLAG_SEND_QUEUED);

   if(status == 0) {
      de_info(DE_TR_WIFI, "monitor on %u MHz", sh->sh_monitor_channel.props.freq);
   } else {
      de_error(DE_TR_WIFI, "failed to start monitor scan job: %d", status);
   }

   return status;
}

int smd_monitor_stop_scanjob(struct smd_hw_priv *sh)
{
   if(smd_monitor_mode_enabled(sh)) {
      de_info(DE_TR_WIFI, "monitor stopped");
      return smd_stop_scan_job(sh);
   }

   de_error(DE_TR_WIFI, "try do disable monitor scan job in non-monitor mode");
   dump_stack();
   return -EINVAL;
}

void
smd_monitor_update_channel(struct smd_hw_priv *sh,
                           struct ieee80211_channel *channel,
                           enum nl80211_channel_type channel_type)
{
   if(channel == NULL)
      return;

   smd_update_chn_prop_from_channel(&sh->sh_monitor_channel,
                                    channel,
                                    channel_type);

   smd_monitor_update_scanjob(sh);
}

#ifdef NRX_USE_CHANNEL_CONTEXT
void
smd_monitor_update_chan_def(struct smd_hw_priv *sh,
                            const struct cfg80211_chan_def *chandef)
{
   smd_update_chn_prop_from_chan_def(&sh->sh_monitor_channel, chandef);

   smd_monitor_update_scanjob(sh);
}
#endif /* NRX_USE_CHANNEL_CONTEXT */

/* if dst == NULL, only count size */
static size_t
scanreq_fill_ies(struct cfg80211_scan_request *sreq, uint8_t *dst)
{
   unsigned i;
   size_t   len = 0;

   if (sreq->n_ssids == 0) {
      if(dst != NULL) {
         dst[0] = WLAN_EID_SSID;
         dst[1] = 0;
         dst += 2;
      }
      len += 2;
   } else {
      for(i=0; i < sreq->n_ssids; i++) {
         if (sreq->ssids[i].ssid_len > 0 || i == 0) {
            if(dst != NULL) {
               dst[0] = WLAN_EID_SSID;
               dst[1] = sreq->ssids[i].ssid_len;
               memcpy(&dst[2], sreq->ssids[i].ssid, sreq->ssids[i].ssid_len);
               dst += 2 + sreq->ssids[i].ssid_len;
            }
            len += 2 + sreq->ssids[i].ssid_len;
         }
      }
   }
   if(dst != NULL)
      memcpy(dst, sreq->ie, sreq->ie_len);
   len += sreq->ie_len;

   return len;
}


int
smd_scan_request(struct smd_hw_priv *sh,
                 struct cfg80211_scan_request *sreq)
{
   struct smd_transaction *stx;
   nrx_scan_start_req_t *req;
   nrx_scan_start_cfm_t *cfm;
   nrx_tlv_head_t *tlv;
   mac_chn_prop_t *cprop;
   size_t tlv_size = sizeof(*tlv) + sreq->n_channels * sizeof(*cprop);
   bool passive_scan;
   unsigned i;
   int status;

   req = smd_alloc_req(sh, sizeof(*req) + tlv_size + scanreq_fill_ies(sreq, NULL),
                       HIC_MESSAGE_TYPE_SCAN, NRX_SCAN_START_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   /* this is really weird, what does passive scan have to do
    * with not having any ssids? anyway this is what the "iw"
    * tool does */
   passive_scan = (sreq->n_ssids == 0);

   req->flags = NRX_SCAN_FLAG_SEND_READY_IND;
   req->tlv_size = tlv_size;
   if(passive_scan) {
      req->min_ch_time = 20;
      req->max_ch_time = 160;
      req->channel_interval = 100;
   } else {
      req->min_ch_time = 10;
      req->max_ch_time = 50;
      req->channel_interval = 0;
   }
   req->scan_period = 0;

   if(smd_ie_find_vendor(sreq->ie,
                         sreq->ie_len,
                         cpu_to_be32(P2P_OUI),
                         NULL) != NULL) {
      req->retrans.attrib.rate  = MAC_RATE_OFDM_6MBIT;
   } else {
      req->retrans.attrib.rate  = MAC_RATE_QPSK_5_5MBIT;
   }
   req->retrans.attrib.tries = 2;
   req->retrans.attrib.flags = 0;
   smd_broadcast_addr(&req->bssid);

   /* Add TLV header */
   tlv = (nrx_tlv_head_t *) &req[1];
   tlv->type = NRX_TLV_TYPE_CHN_PROP_LIST;
   tlv->len  = sreq->n_channels * sizeof(*cprop);

   /* Add list of channels */
   cprop = (mac_chn_prop_t *) &tlv[1];
   for(i=0; i < sreq->n_channels; i++) {
      cprop[i].props.freq = sreq->channels[i]->center_freq;
      if(passive_scan) {
         cprop[i].props.flags = NRX_MAC_CHN_FLAG_PASSIVE_SCAN;
      } else {
         cprop[i].props.flags = 0;
      }
      cprop[i].props.tx_dbm = sreq->channels[i]->max_power;
   }

   scanreq_fill_ies(sreq, (uint8_t *) &cprop[i]);

   /* Issue SCAN_START request */
   /* Issue SCAN_START request */
   status = smd_wait_transaction(stx);
   if(status == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->status != NRX_SUCCESS) {
         de_error(DE_TR_WIFI, "FAILED: nrx_scan_start_req: %d", cfm->status);
         status = -EIO;
      }
   }
   smd_txn_decref(stx);

   return status;
}
