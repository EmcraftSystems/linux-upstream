#include "smd_common.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
#define IEEE80211_TX_CTL_NO_PS_BUFFER IEEE80211_TX_CTL_PSPOLL_RESPONSE
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#define IEEE80211_TX_CTL_NO_PS_BUFFER IEEE80211_TX_CTL_POLL_RESPONSE
#endif

static bool
smd_check_discard_time(struct smd_hw_priv *sh, struct sk_buff *skb)
{
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   struct smd_sta_priv      *ss  = SMD_TX_INFO_SS(txi);
   if(time_is_after_eq_jiffies(SMD_TX_INFO_DISCARD_TIME(txi)))
      return true;
   smd_tx_failure(sh->sh_hw, skb, true);
   SS_PUT(ss);
   de_info(DE_TR_WIFI, "Discarding frame due to time to live expired, skb %p", skb);
   return false;
}

/**
 * Called for FRAME_TX transaction when _REQ message was sent.
 */
static void
smd_tx_complete(struct smd_transaction *stx)
{
   smd_txn_decref(stx);
}

static void remove_transport_header(struct sk_buff *skb)
{
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   nrx_frame_tx_req_t *req = (nrx_frame_tx_req_t*)skb->data;

   /* remove HIC header */
   DE_ASSERT(req->payload_offset < skb->len);
   skb_pull(skb, req->payload_offset);

   ieee80211_tx_info_clear_status(txi);
}

static void smd_indicate_tx_status(struct smd_transaction *stx,
                                   struct ieee80211_hw *hw,
                                   struct smd_sta_priv *ss,
                                   nrx_frame_tx_cfm_t *cfm)
{
   unsigned int i;
   uint32_t tries = cfm->tries;
   struct sk_buff *skb = stx->st_txskborg;
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   struct ieee80211_tx_rate  rates[IEEE80211_TX_MAX_RATES];

   /*
    * Save tx rate list, since clear_status for some reason zapps
    * the count field, which we need
    */
   memcpy(rates, txi->control.rates, sizeof(rates));

   remove_transport_header(skb);

   SS_PUT(ss);

   if(SMD_TX_INFO_FLAGS(txi) & SMD_TX_INFO_FLAG_INTERNAL)
      /* nothing further to do */
      return;

   if (cfm->status == NRX_TX_STATUS_OK)
      txi->flags |= IEEE80211_TX_STAT_ACK;

   /*
    * Update the rate information with the number of transmission
    * attempts for each rate.
    */
   for (i=0; i < IEEE80211_TX_MAX_RATES && rates[i].idx >= 0; i++) {
      if (tries == 0) {
         /* We succeeded on previous rate. Therefore terminate the list. */
         txi->status.rates[i].idx = -1;
         break;
      }
      if (tries <= rates[i].count) {
         txi->status.rates[i].count = tries;
      } else {
         txi->status.rates[i].count = rates[i].count;
      }
      tries -= txi->status.rates[i].count;

      de_debug(DE_TR_WIFI, "rate = %d, %u, %x",
               txi->status.rates[i].idx,
               txi->status.rates[i].count,
               txi->status.rates[i].flags);
   }

#ifdef NRX_CONFIG_ENABLE_STATISTICS
   smd_stat_update_tx_cfm_statistics(hw_to_sh(hw), cfm);
#endif

   if(txi->flags & IEEE80211_TX_CTL_AMPDU
      && !(cfm->tx_flags & NRX_TX_CFM_FLAG_AMPDU)) {
      cfm->tx_flags |= NRX_TX_CFM_FLAG_AMPDU;
   }
   if(txi->flags & IEEE80211_TX_CTL_AMPDU
      && cfm->tx_flags & NRX_TX_CFM_FLAG_AMPDU) {
      txi->flags |= IEEE80211_TX_STAT_AMPDU;
      if(txi->flags & IEEE80211_TX_STAT_ACK)
         txi->status.ampdu_ack_len = 1;
      else
         txi->status.ampdu_ack_len = 0;
      txi->status.ampdu_len = 1;
   }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   if (cfm->tx_flags & NRX_TX_CFM_FLAG_EOSP) {
      txi->flags |= IEEE80211_TX_STATUS_EOSP;
   }
#endif

   if(cfm->ack_rssi > -100)
      txi->status.ack_signal = cfm->ack_rssi;
   else
      txi->status.ack_signal = 0;

   /* consume original skb */
   stx->st_txskborg = NULL;

   /* Report Tx status to mac80211 */
   ieee80211_tx_status_ni(hw, skb);
}

/**
 * Called for FRAME_TX transaction when _CFM message was received.
 */
static void
smd_tx_confirm(struct smd_transaction *stx)
{
   struct smd_hw_priv       *sh  = stx->st_sh;
   struct ieee80211_hw      *hw  = sh->sh_hw;
   struct sk_buff           *skb = stx->st_txskborg;
   nrx_frame_tx_req_t       *req = (nrx_frame_tx_req_t*)skb->data;
   nrx_frame_tx_cfm_t       *cfm;
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   struct smd_sta_priv      *ss = SMD_TX_INFO_SS(txi);

   if(stx->st_restart) {
      SS_PUT(ss);
      smd_txn_decref(stx);
      return;
   }
   cfm = (nrx_frame_tx_cfm_t*)stx->st_rxskb->data;

   de_info(DE_TR_WIFI, "TX_CFM: %u/%x, %u, %u, %x, %u %u s=%x",
           cfm->status,
           cfm->tx_flags,
           cfm->retrans_id,
           cfm->tries,
           cfm->rate_used,
           cfm->prio,
           cfm->tid,
           cfm->seq_ctrl);

   if (req->flags & NRX_TX_FLAG_QOS) {
      uint16_t              seq_no = (cfm->seq_ctrl>>4);

      ss->ss_tid[req->tid].st_last_seq_no = seq_no;
   }

   smd_indicate_tx_status(stx, hw, ss, cfm);

   complete(&stx->st_completion);
   QUEUE_TXWORK(sh);
   smd_txn_decref(stx);
}

/**
 * Called for FRAME_COPY_REQ transaction when _CFM message was received. API 6+ only.
 */
static void
smd_tx_copy_confirm(struct smd_transaction *stx)
{
   struct sk_buff           *skb = stx->st_txskb;
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   struct smd_sta_priv      *ss  = SMD_TX_INFO_SS(txi);
   
   SS_PUT(ss);
   if(!stx->st_restart) {
      complete(&stx->st_completion);
      QUEUE_TXWORK(stx->st_sh);
   }
   smd_txn_decref(stx);
}


/**
 * Called from STREAM_CTRL_IND to create a FRAME_COPY_REQ transaction. API 6+ only.
 */
static void
retransmit_frame(struct smd_hw_priv *sh,
                 nrx_hic_trans_id_t  transid,
                 uint32_t            retransid)
{
   struct smd_transaction   *stx_orig;
   struct smd_transaction   *stx;
   struct ieee80211_tx_info *txi;
   struct smd_sta_priv      *ss;
   nrx_frame_tx_req_t       *req;

   stx_orig = smd_txn_getref(sh, transid);

   if(stx_orig == NULL) {
      de_error(DE_TR_WIFI, "failed to find copy of tid=%u", transid);
      return;
   }

   //if (!smd_check_discard_time(sh, skb)) {
   //   return;
   //}
   //if(stx_orig->st_status != STX_TX_COMPLETE) {
   //   /* XXX check if in transit */
   //}

   if(stx_orig->st_txskborg == NULL)
   {
      de_error(DE_TR_WIFI, "frame to be retransmitted never reached fw, or tid=%u corrupted", transid);
      BUG();
      return;
   }

   stx = smd_clone_transaction(stx_orig);
   smd_txn_decref(stx_orig);

   if(stx == NULL) {
      de_error(DE_TR_WIFI, "failed to clone transaction for tid=%u", transid);
      return;
   }

   de_info(DE_TR_WIFI, "retransmit original stx tid=%d with retransid=%u", transid, retransid);
   smd_txn_incref(stx); //was 1, now 2

   /* different confirm callback; frame_copy_cfm_t is "simple cfm" (status only) */
   stx->st_tx_confirm = smd_tx_copy_confirm;

   /* assign retrans id */
   req = (nrx_frame_tx_req_t*)stx->st_txskb->data;
   req->retrans_id = retransid;

   /* find and get ss */
   txi = IEEE80211_SKB_CB(stx->st_txskb);
   ss  = SMD_TX_INFO_SS(txi);
   SS_GET(ss);

   smd_add_hic_header(stx, HIC_MESSAGE_TYPE_FRAME, NRX_FRAME_COPY_REQ);

   if (nanoc_send_data(stx->st_netid, stx->st_txskb) != 0) {
      de_error(DE_TR_WIFI, "nanoc_send_data() failed");
      smd_txn_decref(stx); /* one for tx complete... */
      smd_txn_decref(stx); /* ... and one for rx confirm */
   }
}

void
smd_retransmit_frames(struct smd_hw_priv *sh,
                      nrx_tlv_frame_copy_req_t *tlv)
{
   retransmit_frame(sh, tlv->transid, tlv->retrans_id);
}

static void
print_tx_info(struct ieee80211_tx_info *txi)
{
   unsigned int i;

   de_debug(DE_TR_WIFI, "flags = %x", txi->flags);
#define Z(I, F) if((I) & (F)) de_debug(DE_TR_WIFI, "  " #F);
   Z(txi->flags, IEEE80211_TX_CTL_REQ_TX_STATUS);
   Z(txi->flags, IEEE80211_TX_CTL_ASSIGN_SEQ);
   Z(txi->flags, IEEE80211_TX_CTL_NO_ACK);
   Z(txi->flags, IEEE80211_TX_CTL_CLEAR_PS_FILT);
   Z(txi->flags, IEEE80211_TX_CTL_FIRST_FRAGMENT);
   Z(txi->flags, IEEE80211_TX_CTL_SEND_AFTER_DTIM);
   Z(txi->flags, IEEE80211_TX_CTL_AMPDU);
   Z(txi->flags, IEEE80211_TX_CTL_INJECTED);
   Z(txi->flags, IEEE80211_TX_STAT_TX_FILTERED);
   Z(txi->flags, IEEE80211_TX_STAT_ACK);
   Z(txi->flags, IEEE80211_TX_STAT_AMPDU);
   Z(txi->flags, IEEE80211_TX_STAT_AMPDU_NO_BACK);
   Z(txi->flags, IEEE80211_TX_CTL_RATE_CTRL_PROBE);
   Z(txi->flags, IEEE80211_TX_CTL_MORE_FRAMES);
   Z(txi->flags, IEEE80211_TX_CTL_NO_PS_BUFFER);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   Z(txi->flags, IEEE80211_TX_STATUS_EOSP);
#endif
   Z(txi->flags, IEEE80211_TX_CTL_TX_OFFCHAN);
#undef ZZ
   de_debug(DE_TR_WIFI, "band = %u", txi->band);

   for (i = 0;
        i < ARRAY_SIZE(txi->control.rates) && txi->control.rates[i].idx >= 0;
        i++)
   {
      de_debug(DE_TR_WIFI, "rate = %d, %u, %x",
            txi->control.rates[i].idx,
            txi->control.rates[i].count,
            txi->control.rates[i].flags);
   }

   de_debug(DE_TR_WIFI, "rts_cts_rate_idx = %d", txi->control.rts_cts_rate_idx);
   de_debug(DE_TR_WIFI, "vif = %p", txi->control.vif);
   de_debug(DE_TR_WIFI, "hw_key = %p", txi->control.hw_key);
   if(txi->control.hw_key != NULL) {
      de_debug(DE_TR_WIFI, "  hw_key.cipher = %08x", txi->control.hw_key->cipher);
      de_debug(DE_TR_WIFI, "  hw_key.icv_len = %u", txi->control.hw_key->icv_len);
      de_debug(DE_TR_WIFI, "  hw_key.iv_len = %u", txi->control.hw_key->iv_len);
      de_debug(DE_TR_WIFI, "  hw_key.hw_key_idx = %u", txi->control.hw_key->hw_key_idx);
      de_debug(DE_TR_WIFI, "  hw_key.flags = %x", txi->control.hw_key->flags);
      de_debug(DE_TR_WIFI, "  hw_key.keyidx = %d", txi->control.hw_key->keyidx);
      de_debug(DE_TR_WIFI, "  hw_key.keylen = %u", txi->control.hw_key->keylen);
   }
   de_debug(DE_TR_WIFI, "sta = %p", SMD_TX_INFO_SS(txi));
}

#define USE_PROTECT(R) (((R)->flags & (IEEE80211_TX_RC_USE_RTS_CTS      \
                                       |IEEE80211_TX_RC_USE_CTS_PROTECT)) != 0)

void
make_rate_entry(struct ieee80211_tx_rate *rate,
                struct ieee80211_supported_band *band,
                mac_retrans_t *rp,
                mac_retrans_t *rr)
{
   /* The following code must work even when rp or rr is 16-bit aligned. */
   if(rate->flags & IEEE80211_TX_RC_MCS) {
      /* rate is MCS index */
      if(rate->flags & IEEE80211_TX_RC_40_MHZ_WIDTH) {
         rp->attrib.rate = (MAC_MOD_HT40 << 8) | rate->idx;
      } else {
         rp->attrib.rate = (MAC_MOD_HT << 8) | rate->idx;
      }
   } else {
      rp->attrib.rate = band->bitrates[rate->idx].hw_value;
   }
   rp->attrib.tries = rate->count;
   rp->attrib.flags = 0;
   if(rate->flags & IEEE80211_TX_RC_USE_RTS_CTS)
      rp->attrib.flags |= NRX_MAC_RATE_FLAG_USE_RTS_CTS;
   if(rate->flags & IEEE80211_TX_RC_USE_CTS_PROTECT)
      rp->attrib.flags |= NRX_MAC_RATE_FLAG_USE_CTS_TO_SELF;
   if(rate->flags & IEEE80211_TX_RC_USE_SHORT_PREAMBLE)
      rp->attrib.flags |= NRX_MAC_RATE_FLAG_SHORT_PREAMBLE;
   if(rate->flags & IEEE80211_TX_RC_GREEN_FIELD)
      rp->attrib.flags |= NRX_MAC_RATE_FLAG_HT_GREENFIELD;
   if(rate->flags & IEEE80211_TX_RC_SHORT_GI)
      rp->attrib.flags |= NRX_MAC_RATE_FLAG_HT_SHORT_GI;
   if(rate->flags & IEEE80211_TX_RC_DUP_DATA) {
      de_info(DE_TR_WIFI, "IEEE80211_TX_RC_DUP_DATA unhandled");
   }
   if(rr != NULL) {
      rr->attrib.tries = 1;
      rr->attrib.flags = 0;
      if(USE_PROTECT(rate)) {
         /* XXX we're handed a pre-computed rate, which is the fastest
          * basic rate that's lower than the first rate in the list,
          * but we really want this for each rate */
         rr->attrib.rate = MAC_RATE_QPSK_1MBIT;
      } else {
         rr->attrib.rate = MAC_RATE_QPSK_1MBIT;
      }
   }
}

static size_t
add_rate_lists(struct sk_buff *skb,
               struct ieee80211_tx_rate *rates,
               struct ieee80211_supported_band *band)
{
   mac_retrans_t *rp;
   mac_retrans_t *rr = NULL;
   size_t rate_size;
   nrx_tlv_head_t *tlv;
   unsigned int i;
   unsigned int nrates = 0;
   bool use_rts_cts = false;
   size_t tlv_size = 0;

   for(i = 0; i < IEEE80211_TX_MAX_RATES; i++) {
      if(rates[i].idx < 0)
         break;
      if(USE_PROTECT(&rates[i])) {
         use_rts_cts = true;
      }
      nrates++;
   }
   rate_size = nrates * sizeof(*rp);

   if(use_rts_cts) {
      rr = (mac_retrans_t*)skb_push(skb, rate_size);
      tlv_size += rate_size;

      tlv = (nrx_tlv_head_t*)skb_push(skb, sizeof(*tlv));
      tlv->len = rate_size;
      tlv->type = NRX_TLV_TYPE_RETRANS_PROTECT_TABLE;
      tlv_size += sizeof(*tlv);
   }
   rp = (mac_retrans_t*)skb_push(skb, rate_size);
   tlv_size += rate_size;

   tlv = (nrx_tlv_head_t*)skb_push(skb, sizeof(*tlv));
   tlv->len = rate_size;
   tlv->type = NRX_TLV_TYPE_RETRANS_TABLE;
   tlv_size += sizeof(*tlv);

   for(i = 0; i < nrates; i++) {
      make_rate_entry(&rates[i],
                      band,
                      &rp[i],
                      use_rts_cts ? &rr[i] : NULL);
   }
   return tlv_size;
}

static void
maybe_start_ba_session(struct smd_sta_priv *ss,
                       __be16 protocol,
                       uint16_t tid)
{
   /* this needs improvement */

   if(!ss_to_sta(ss)->ht_cap.ht_supported)
      return;

   if(protocol == cpu_to_be16(ETH_P_PAE))
      /* XXX should look at control_port_protocol, but this is private
       * data */
      return;

   if(test_bit(SMD_STA_TID_FLAG_AMPDU_ENABLED, &ss->ss_tid[tid].st_flags))
      return;

   if(test_bit(SMD_STA_TID_FLAG_AMPDU_START_REQ, &ss->ss_tid[tid].st_flags)
      && time_is_after_jiffies(ss->ss_tid[tid].st_ampdu_start_req_time))
      return;

   de_debug(DE_TR_WIFI, "request BA session start for STA %pM, TID %u",
            ss_to_sta(ss)->addr, tid);

   ieee80211_start_tx_ba_session(ss_to_sta(ss), tid, 5000);
   set_bit(SMD_STA_TID_FLAG_AMPDU_START_REQ, &ss->ss_tid[tid].st_flags);
   ss->ss_tid[tid].st_ampdu_start_req_time = jiffies + HZ;
}

void tx_prepare(struct smd_hw_priv *sh,
                struct smd_vif_priv *sv,
                struct smd_sta_priv *ss,
                struct sk_buff *skb)
{
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   struct ieee80211_hdr *mac_hdr = (struct ieee80211_hdr*)skb->data;
   nrx_frame_tx_req_t *req;
   bool fragment_frame = (sh->sh_frag_threshold != 0
                          && skb->len > sh->sh_frag_threshold);
   size_t pad_size;
   size_t tlv_size = 0;

   BUILD_BUG_ON(__alignof__(nrx_frame_tx_req_t) > 4);

   pad_size = (uintptr_t)skb->data & 3;
   DE_ASSERT((pad_size & 1) == 0); /* handled in smd_op_tx */
   skb_push(skb, pad_size);

   if(ieee80211_is_probe_resp(mac_hdr->frame_control)
      && pad_size == 2) {
      /* XXX firmware currently requires the timestamp (see below) to
       * be 4-byte aligned relative to tx_req header (we get a 134.41
       * coredump if it's not), so move the header two bytes */
      memmove(skb->data, skb->data + 2, skb->len - 2);
      skb_trim(skb, skb->len - 2);
      mac_hdr = (struct ieee80211_hdr*)skb->data;
      pad_size = 0;
   }

   
   //if(!(sh->sh_hw->flags & IEEE80211_HW_HAS_RATE_CONTROL))
   if(!ieee80211_hw_check(sh->sh_hw, HAS_RATE_CONTROL))
      tlv_size += add_rate_lists(skb,
                                 txi->control.rates,
                                 sh->sh_hw->wiphy->bands[txi->band]);

   DE_ASSERT(IS_ALIGNED((uintptr_t)skb->data, __alignof__(*req)));
   req = (nrx_frame_tx_req_t*)skb_push(skb, sizeof(*req));
   memset(req, 0, sizeof(*req));
   req->tlv_size = tlv_size;
   req->payload_offset = sizeof(*req) + tlv_size + pad_size;
   req->max_lifetime = 500000;

   if (!is_multicast_ether_addr(mac_hdr->addr1))
      req->flags |= NRX_TX_FLAG_UCAST;
   else
      fragment_frame = false;

   if (ieee80211_is_data_qos(mac_hdr->frame_control)) {
      u8 *qos_control;

      req->flags |= NRX_TX_FLAG_QOS;
      qos_control = ieee80211_get_qos_ctl(mac_hdr);
      req->tid = qos_control[0] & IEEE80211_QOS_CTL_TID_MASK;
      if (ss) {
         unsigned ac = ss->ss_sta_info.tid_to_ac_map[req->tid];
         if (((1<<ac) & ss->ss_sta_info.uapsd_trigger_enabled_ac_mask) != 0) {
            req->flags |= NRX_TX_FLAG_UAPSD_TRIGGER;
         }
         if (((1<<ac) & ss->ss_sta_info.uapsd_delivery_enabled_ac_mask) != 0) {
            req->flags |= NRX_TX_FLAG_UAPSD_DELIVERY;
         }
         set_bit(SMD_STA_TID_FLAG_GOT_FRAME, &ss->ss_tid[req->tid].st_flags);
         //if(sh->sh_hw->flags & IEEE80211_HW_HAS_RATE_CONTROL)
         if(ieee80211_hw_check(sh->sh_hw, HAS_RATE_CONTROL))
            maybe_start_ba_session(ss, skb->protocol, req->tid);
      }
   } else if(ieee80211_is_mgmt(mac_hdr->frame_control)
             && ss != NULL && ss->ss_sta_info.qos_flag) {
#define TRAFFIC_TYPE_VOICE 6 /* as per 802.1d */
      req->tid = TRAFFIC_TYPE_VOICE;
   } else {
#define TRAFFIC_TYPE_BEST_EFFORT 0 /* as per 802.1d */
      req->tid = TRAFFIC_TYPE_BEST_EFFORT;
   }

   if(ieee80211_is_probe_resp(mac_hdr->frame_control)) {
      /* fill in timestamp */
      req->flags |= NRX_TX_FLAG_TS_INSERT;
   }
#define UNDER_BAA(SS, TID) ((SS) != NULL                                \
                            && (TID) < ARRAY_SIZE((SS)->ss_tid)         \
                            && test_bit(SMD_STA_TID_FLAG_AMPDU_ENABLED, \
                                        &(SS)->ss_tid[(TID)].st_flags))
   if(UNDER_BAA(ss, req->tid)) {
      /* an msdu transmitted under HT BAA shall not be fragmented */
      fragment_frame = false;
      if(txi->flags & IEEE80211_TX_CTL_AMPDU) {
         req->baa_buffer_size = ss->ss_tid[req->tid].st_ampdu_buf_size;
         req->flags |= NRX_TX_FLAG_AMPDU_ELIGIBLE;
      }
   }
   if(txi->flags & IEEE80211_TX_CTL_TX_OFFCHAN) {
      req->flags |= NRX_TX_FLAG_CHANNEL_SYNC;
      de_info(DE_TR_WIFI, "channel sync TX frame");
   }
   if (txi->control.hw_key != NULL) {
      req->key_id = txi->control.hw_key->keyidx;
      req->flags |= NRX_TX_FLAG_DO_ENCRYPT;
   }
   if (resource_test_scan(sh, SMD_FLAG_SCAN_MONITOR) == 0) {
      req->flags |= NRX_TX_FLAG_CHANNEL_SYNC;
      de_info(DE_TR_WIFI, "Monitor TX frame");
   }
   if(fragment_frame)
      req->fragment_size = sh->sh_frag_threshold;

   print_tx_info(txi);

   BUILD_BUG_ON(sizeof(struct smd_tx_info) > IEEE80211_TX_INFO_RATE_DRIVER_DATA_SIZE);

   /* txi->{vif,hw_key,sta} are gone after this point */
   SMD_TX_INFO_DISCARD_TIME(txi) = jiffies + msecs_to_jiffies(req->max_lifetime/1000);
   SMD_TX_INFO_SS(txi) = ss;
   SMD_TX_INFO_FLAGS(txi) = 0;
   if(sv != NULL)
      SMD_TX_INFO_NETID(txi) = sv->sv_net_id;
   else
      SMD_TX_INFO_NETID(txi) = sh->sh_dummy_netid;
}

static void
send_single_skb(struct smd_hw_priv *sh, struct sk_buff *skb)
{
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   nrx_net_id_t              net_id = SMD_TX_INFO_NETID(txi);
   struct smd_sta_priv      *ss = SMD_TX_INFO_SS(txi);
   struct smd_transaction   *stx;
#ifdef NRX_CONFIG_ENABLE_STATISTICS
   unsigned int              frame_len = skb->len;
#endif

   if (!smd_check_discard_time(sh, skb)) {
      return;
   }

   stx = smd_create_transaction(sh, net_id);
   if (stx == NULL) {
      SS_PUT(ss);
      FREE_TXSKB(sh, skb);
      return;
   }

   stx->st_txskborg = skb;
   if ((skb = skb_clone(skb, GFP_KERNEL)) == NULL) {
      de_error(DE_TR_WIFI, "Failed to clone skb");
      SS_PUT(ss);
      smd_txn_decref(stx);
      return;
   }

   stx->st_queue       = NANOC_QDATA;
   stx->st_txskb       = skb;
   stx->st_tx_complete = smd_tx_complete;
   stx->st_tx_confirm  = smd_tx_confirm;
   smd_txn_incref(stx);

#ifdef NRX_CONFIG_ENABLE_STATISTICS
   smd_stat_update_tx_req_statistics(sh, (nrx_frame_tx_req_t*)skb->data, frame_len);
#endif

   smd_add_hic_header(stx, HIC_MESSAGE_TYPE_FRAME, NRX_FRAME_TX_REQ);

   if (nanoc_send_data(net_id, skb) != 0) {
      de_error(DE_TR_WIFI, "nanoc_send_data() failed");
      SS_PUT(ss);
      smd_txn_decref(stx); /* one for tx complete... */
      smd_txn_decref(stx); /* ... and one for rx confirm */
      /* Releasing the transaction will free the st_txskborg */
#ifdef NRX_CONFIG_ENABLE_STATISTICS
      LOCK_STATS(sh);
      sh->sh_statistics.stat_tx_status[7]++;
      UNLOCK_STATS(sh);
#endif
   }
}

struct smd_queue_barrier {
   unsigned int       qb_magic;
#define QB_MAGIC 0x45674567 /* for frames this is two sizes, which
                             * can't be the same */
   struct completion  qb_completion;
};

/* inject a dummy packet into the TX queue, and wait for it to trickle
 * through, return true if queue was processed */
bool smd_tx_queue_barrier(struct smd_hw_priv *sh)
{
   struct sk_buff *skb;
   struct smd_queue_barrier *qb;
   unsigned long timeout;

   skb = dev_alloc_skb(sizeof(*qb));
   if(skb == NULL)
      return false;

   qb = (struct smd_queue_barrier*)skb_put(skb, sizeof(*qb));

   qb->qb_magic = QB_MAGIC;
   init_completion(&qb->qb_completion);

   skb_get(skb); /* get extra refrerence */

   de_debug(DE_TR_WIFI, "Pre-barrier");
   skb_queue_tail(&sh->sh_txqueue, skb);
   QUEUE_TXWORK(sh);
   timeout = wait_for_completion_timeout(&qb->qb_completion, 2 * HZ);
   de_debug(DE_TR_WIFI, "Post-barrier, timeout = %lu", timeout);

   dev_kfree_skb(skb);

   return timeout != 0;
}

static bool smd_tx_queue_barrier_complete(struct sk_buff *skb)
{
   struct smd_queue_barrier *qb = (struct smd_queue_barrier*)skb->data;

   if(skb->len == sizeof(*qb) && qb->qb_magic == QB_MAGIC) {
      complete(&qb->qb_completion);
      dev_kfree_skb(skb);
      return true;
   }
   return false;
}

void flush_single_queue(struct smd_hw_priv *sh,
                        struct sk_buff_head *queue,
                        bool drop)
{
   struct ieee80211_tx_info *txi;
   struct smd_sta_priv *ss;
   struct sk_buff *skb;

   while((skb = skb_dequeue(queue)) != NULL) {
      if(unlikely(smd_tx_queue_barrier_complete(skb)))
         continue;
      txi = IEEE80211_SKB_CB(skb);
      ss = SMD_TX_INFO_SS(txi);
      SS_PUT(ss);
      if (!drop) {
         smd_tx_failure(sh->sh_hw, skb, true);
      } else {
         FREE_TXSKB(sh, skb);
      }
   }
}

void flush_tx_queues(struct smd_hw_priv *sh, bool drop)
{
   HWLOCK(sh);
   flush_single_queue(sh, &sh->sh_txqueue, drop);
   HWUNLOCK(sh);
}

NRX_WORKER_DECLARE(smd_process_tx_queue)
{
   NRX_WORKER_SETUP(struct smd_hw_priv, sh, sh_txwork);
   struct sk_buff *skb;

   if (test_bit(SMD_FLAG_SH_LINKUP_DATA, &sh->sh_flags))
   {
      if (test_and_clear_bit(SMD_FLAG_SH_FRAME_CONFIG, &sh->sh_flags)) {
         frame_config(sh, sh->sh_filter_flags); /* Will make sure that we get beacons and probe responses */
      }
      if (test_and_clear_bit(SMD_FLAG_SH_UPDATED_SV, &sh->sh_flags))
         FOREACH_VIF(sh, smd_update_sv);
   }

   while (test_bit(SMD_FLAG_SH_LINKUP_DATA, &sh->sh_flags) &&
          (skb = skb_dequeue(&sh->sh_txqueue)) != NULL)
   {
      if(unlikely(smd_tx_queue_barrier_complete(skb)))
         continue;
      send_single_skb(sh, skb);
   }

   if (test_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags))
   {
      de_info(DE_TR_WIFI, "Purge tx queue");
      flush_tx_queues(sh, false);
   }
   else if (test_bit(SMD_FLAG_SH_LINKUP_DATA, &sh->sh_flags) &&
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
            !test_bit(SMD_FLAG_SH_FLUSHING, &sh->sh_flags) &&
#endif
       skb_queue_len(&sh->sh_txqueue) == 0)
   {
      ieee80211_wake_queues(sh->sh_hw);
   }
}

void
smd_tx_failure(struct ieee80211_hw *hw,
               struct sk_buff *skb,
               bool remove_header)
{
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);

   if(remove_header)
      remove_transport_header(skb);

   ieee80211_tx_info_clear_status(txi);

   ieee80211_tx_status_ni(hw, skb);
}

/* Handler that 802.11 module calls for each transmitted frame. skb
 * contains the buffer starting from the IEEE 802.11 header. The
 * low-level driver should send the frame out based on configuration
 * in the TX control data. This handler should, preferably, never fail
 * and stop queues appropriately, more importantly, however, it must
 * never fail for A-MPDU-queues. This function should return
 * NETDEV_TX_OK except in very limited cases. Must be implemented and
 * atomic. */
void
smd_tx_frame(struct ieee80211_hw *hw,
             struct ieee80211_sta *sta,
             struct sk_buff *skb)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);
   /* Need to insert noa attrib on probe responses */
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   struct ieee80211_hdr *mac_hdr = (struct ieee80211_hdr*)skb->data;
   struct smd_vif_priv *sv = NULL;
   struct smd_sta_priv *ss = NULL;

   if(test_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
      smd_tx_failure(hw, skb, false);
      de_debug(DE_TR_API, "driver is RESTARTING, dropping packet %p", skb);
      return;
   }

   /* We must add padding to make the request header 32-bit
    * aligned. But we can only handle 16-bit alignments. */
   if ((uintptr_t)skb->data & 1) {
      de_warn(DE_TR_WIFI, "Warning! mac header is not 16-bit aligned!");
      FREE_TXSKB(sh, skb);
      return;
   }

   if(txi->flags & IEEE80211_TX_CTL_TX_OFFCHAN) {
      if(resource_test_scan(sh, SMD_FLAG_SCAN_ROC) < 0) {
         de_warn(DE_TR_WIFI, "Tx REJECTED: OFFCHAN frame while remain-on-channel not ongoing");
         smd_tx_failure(hw, skb, false);
         return;
      }
   }
   if(txi->control.vif != NULL) {
      sv = vif_to_sv(txi->control.vif);
      if(!SV_IS_VALID(sv)) {
         de_info(DE_TR_WIFI, "dropping frame for unconfigured interface");
         FREE_TXSKB(sh, skb);
         return;
      }
   }

   if(sta != NULL)
      ss = sta_to_ss(sta);

   if(ss == NULL && ieee80211_is_data_qos(mac_hdr->frame_control)) {
      /* we should not get these frames if there is no peer STA, but
       * we do, so until we figure out why this happens, we just drop
       * these frames */
      de_info(DE_TR_WIFI, "dropping QoS-frame without peer station");
      FREE_TXSKB(sh, skb);
      return;
   }
   SS_GET(ss);

   if ( sv != NULL && SV_IS_P2P(sv) )
   {
      if(ieee80211_is_probe_resp(mac_hdr->frame_control)) {
         p2p_insert_noa_attrib(sv, skb);
      }
      else if(ieee80211_is_deauth(mac_hdr->frame_control)) {
         /* reset P2P PS only if client */
         if( (sv->sv_net_type & NRX_NET_ROLE_MASTER) == 0 ) {
            p2p_ps_finished(txi->control.vif);
         }
      }
   }
   de_debug(DE_TR_API, "skb=%p, skb->len=%u headroom=%u tailroom=%u",
            skb, skb->len,
            skb_headroom(skb),
            skb_tailroom(skb));

   tx_prepare(sh, sv, ss, skb);

   HWLOCK(sh);
   skb_queue_tail(&sh->sh_txqueue, skb);
   HWUNLOCK(sh);

   QUEUE_TXWORK(sh);
}
