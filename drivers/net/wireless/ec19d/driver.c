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
#include "m80211-stddefs.h"
#include "nanocore.h"
#include "driver_nrx_nl_ext.h"

static int remove_beacon_loss_trigger(struct smd_vif_priv *sv);
static int set_beacon_loss_trigger(struct smd_vif_priv *sv);
static void scan_timeout_cb(de_timer_t *t);

static bool
smd_update_chn_prop(mac_chn_prop_t*,
                    nrx_mac_chn_freq_t,
                    nrx_mac_chn_flag_t);

static unsigned int cfg_service_interval = 20000;
static int          wmm_ps_enable = 0;
static bool disable_short_gi = false;
static bool disable_concurrent_net = false;
#ifdef NRX_CONFIG_FCC
static bool fcc = true;
#else
static bool fcc = false;
#endif
static bool fw_rate_adaptation = false;
static bool disable_tx_ampdu = false;
static unsigned int traffic_timeout = 100000;

module_param_named(service_interval, cfg_service_interval, uint, 0644);
MODULE_PARM_DESC(service_interval, "UAPSD service interval (us)");
module_param(wmm_ps_enable, int, 0644);
MODULE_PARM_DESC(wmm_ps_enable, "Enable wmm ps");
module_param(disable_short_gi, bool, 0644);
MODULE_PARM_DESC(disable_short_gi, "Disable Short Guard Interval");
module_param(disable_concurrent_net, bool, 0644);
MODULE_PARM_DESC(disable_concurrent_net, "Disable concurrent support.");
module_param(fcc, bool, 0644);
MODULE_PARM_DESC(fcc, "Enable FCC mode");
module_param(fw_rate_adaptation, bool, 0644);
MODULE_PARM_DESC(fw_rate_adaptation, "Disable host rate adaptation");
module_param(disable_tx_ampdu, bool, 0644);
MODULE_PARM_DESC(disable_tx_ampdu, "Disable TX A-MPDU");
module_param(traffic_timeout, uint, 0644);
MODULE_PARM_DESC(traffic_timeout, "Power save traffic timeout (us)");
MODULE_LICENSE("GPL");

#define MIB_TRIGGER_BEACON_LOSS  1

static void print_bssconf(nrx_bss_conf_t *c);
static void print_sta_info(struct ieee80211_sta *sta);

void smd_ss_release(struct kref *ref)
{
   struct smd_sta_priv *ss = container_of(ref, struct smd_sta_priv, ss_refcnt);
   wake_up_interruptible(&ss->ss_waitqueue);
}

static bool
start_net(struct smd_vif_priv *sv)
{
   if (!test_and_set_bit(SMD_FLAG_SV_STARTED, &sv->sv_flags)) {
      struct smd_transaction *stx;
      nrx_mlme_start_req_t   *req;
      req = smd_alloc_req_vif(sv, sizeof *req, HIC_MESSAGE_TYPE_MLME, NRX_MLME_START_REQ, &stx);
      if (req != NULL) {
         req->reserved = 0;
         if (smd_wait_transaction(stx) == 0) {
            nrx_mlme_start_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
            if (cfm->result != NRX_SUCCESS) {
               de_error(DE_TR_WIFI, "FAILED: nrx_mlme_start_req");
               clear_bit(SMD_FLAG_SV_STARTED, &sv->sv_flags);
            }
         }
         smd_txn_decref(stx);
      }
      return true;
   }
   return false;
}

static int
stop_net(struct smd_vif_priv *sv)
{
   if (test_and_clear_bit(SMD_FLAG_SV_STARTED, &sv->sv_flags)) {
      struct smd_transaction *stx;
      nrx_mlme_stop_req_t    *req;

      remove_beacon_loss_trigger(sv);

      req = smd_alloc_req_vif(sv, sizeof *req, HIC_MESSAGE_TYPE_MLME, NRX_MLME_STOP_REQ, &stx);
      if (req) {
         req->reserved = 0;
         if (smd_wait_transaction(stx) == 0) {
            nrx_mlme_stop_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
            if (cfm->result != NRX_SUCCESS) {
               de_error(DE_TR_WIFI, "FAILED: nrx_mlme_stop_req");
            }
         }
         smd_txn_decref(stx);
      }
   }
   return 0;
}

static int
destroy_net(struct smd_vif_priv *sv)
{
   struct smd_transaction     *stx;
   nrx_mlme_net_destroy_req_t *req;

   req = smd_alloc_req_vif(sv, sizeof *req,
                           HIC_MESSAGE_TYPE_MLME, NRX_MLME_NET_DESTROY_REQ, &stx);
   if (req) {
      memset(req->reserved, 0, sizeof req->reserved);
      if (smd_wait_transaction(stx) == 0) {
         nrx_mlme_net_destroy_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
         if (cfm->result != NRX_SUCCESS) {
            de_error(DE_TR_WIFI, "FAILED: nrx_mlme_stop_req");
         }
      }
      smd_txn_decref(stx);
   }
   return 0;
}

int
frame_config(struct smd_hw_priv *sh, unsigned int filter_flags)
{
   struct smd_transaction *stx;
   nrx_frame_config_req_t *req;
   int ret;

   req = smd_alloc_req(sh, sizeof(*req), HIC_MESSAGE_TYPE_FRAME, NRX_FRAME_CONFIG_REQ, &stx);
   if (!req) {
      return -ENOMEM;
   }

   req->filter.flags    = 0;
   req->filter.snap_len = 0;
   req->filter.mgmt     = ~0;
   req->filter.action   = ~0;
   req->filter.data     = ~0;
   req->filter.ctrl     = 0;
   req->filter.reserved = 0;
   req->frame_ind_header_padding = 0;
   req->reserved        = 0;

   if (filter_flags & FIF_FCSFAIL) {
      req->filter.flags |= NRX_FILTER_BADFCS;
   }

   if (filter_flags & FIF_CONTROL) {
      req->filter.ctrl = ~0;
   } else if (filter_flags & FIF_PSPOLL) {
      req->filter.ctrl = (1<<M80211_FRAMESUBTYPE_CTRL_PSPOLL);
   }

   if (filter_flags & (FIF_OTHER_BSS)) {
      req->filter.flags |= NRX_FILTER_PROMISC;
   } else {
      /* Clear frame types not needed */
      if (!test_bit(SMD_FLAG_SH_WAIT_FOR_CTW, &sh->sh_flags)
          && (filter_flags & FIF_BCN_PRBRESP_PROMISC) == 0) {
         req->filter.mgmt &= ~(1 << M80211_FRAMESUBTYPE_MGMT_BEACON);
      }
      if ((filter_flags & FIF_PROBE_REQ) == 0) {
         /* This is most likely client mode only, since probe requests are needed in AP mode */
         req->filter.mgmt &= ~(1 << M80211_FRAMESUBTYPE_MGMT_PROBEREQ);
      }

      /* only fw deals with NULL frames */
      req->filter.data &= ~(1 << M80211_FRAMESUBTYPE_DATA_NULL);
      req->filter.data &= ~(1 << M80211_FRAMESUBTYPE_QOS_NULL);
   }

   ret = smd_wait_transaction(stx);
   if(ret != 0) {
      de_info(DE_TR_WIFI, "smd_wait_transaction = %d", ret);
   }
   smd_txn_decref(stx);

   return ret;
}

static int
set_power_mgmt(struct smd_vif_priv *sv, struct smd_hw_priv *sh)
{
   struct smd_transaction          *stx;
   nrx_mlme_power_mgmt_req_t       *req;
   int retval;

   if (!test_bit(SMD_FLAG_SV_STARTED, &sv->sv_flags))
      return 0;

   if (sv->sv_net_type != NRX_NET_TYPE_BSS_STA)
      return 0;

   req = smd_alloc_req_vif(sv, sizeof *req, HIC_MESSAGE_TYPE_MLME, NRX_MLME_POWER_MGMT_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
   *req = sv->sv_psconf;
#else
   *req = sh->sh_psconf;
#endif
   req->traffic_timeout = traffic_timeout; /* Set by module variable */
   req->listen_interval = sh->sh_listen_interval;  /* Set per hw */
   if (test_bit(SMD_FLAG_SV_HIBERNATING, &sv->sv_flags)) {
      /* If the net is hibernating, we always allow power save */
      req->power_mgmt_mode = 1;
   }

   de_info(DE_TR_WIFI,
           "power_mgmt_req: ps=%u"
           " traffic_tmo=%u ms"
           " listen_interval=%u"
           " uapsd=%#x",
           req->power_mgmt_mode,
           req->traffic_timeout/1000,
           req->listen_interval,
           req->uapsd);
   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      nrx_mlme_power_mgmt_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
      if (cfm->result != NRX_SUCCESS) {
         de_error(DE_TR_WIFI, "FAILED: nrx_mlme_power_mgmt");
         retval = -EIO;
      }
   }
   smd_txn_decref(stx);

   return retval;
}

static unsigned int count_bits(unsigned int value)
{
   unsigned int count;
   for(count = 0; value != 0; count++)
      value &= value - 1;
   return count;
}

static unsigned int
smd_bss_count_basic_rates(struct smd_vif_priv *sv)
{
   struct ieee80211_vif *vif = sv_to_vif(sv);
   unsigned int nrates;

   /* BG rates */
   nrates = count_bits(vif->bss_conf.basic_rates);

   /* XXX HT rates? */

   return nrates;
}

static void
smd_bss_set_basic_rates(struct smd_vif_priv *sv,
                        mac_rate_t *rates,
                        unsigned int nrates)
{
   struct smd_hw_priv *sh = sv_to_sh(sv);
   struct ieee80211_vif *vif = sv_to_vif(sv);
   unsigned int i, n;
   struct ieee80211_supported_band *band;

   band = sh->sh_hw->wiphy->bands[IEEE80211_BAND_2GHZ];

   for(n = 0, i = band->n_bitrates - 1; i < band->n_bitrates; i--) {
      if(vif->bss_conf.basic_rates & BIT(i)) {
         rates[n++] = band->bitrates[i].hw_value;
      }
   }
}

static int
update_bssconf(struct smd_vif_priv *sv)
{
   struct smd_transaction     *stx;
   nrx_mlme_set_bssconf_req_t *req;
   int retval;
   size_t tlv_size = 0;
   nrx_tlv_head_t *tlv;
   unsigned int nrates;

   if (!test_and_clear_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags))
      return 0;

   nrates = smd_bss_count_basic_rates(sv);

   if(nrates > 0)
      tlv_size += sizeof(*tlv) + nrates * sizeof(mac_rate_t);

   req = smd_alloc_req_vif(sv,
                           sv->sv_bssconf_size + tlv_size,
                           HIC_MESSAGE_TYPE_MLME,
                           NRX_MLME_SET_BSSCONF_REQ,
                           &stx);
   if(req == NULL)
      return -ENOMEM;

   /* a struct copy would be nice here, but there could be fields we
    * don't know about, so use memcpy */
   memcpy(&req->conf, sv->sv_bssconf, sv->sv_bssconf_size);

   if (sv->sv_net_type == NRX_NET_TYPE_BSS_AP) {
      if (!sv->sv_enable_beacon)
         req->conf.beacon_period = 0;
   } else if (sv->sv_bssconf->beacon_period == 0) {
      req->conf.beacon_period = 100;
   }

   print_bssconf(&req->conf);

   if(nrates > 0) {
      tlv = (nrx_tlv_head_t*)((char*)&req->conf + sv->sv_bssconf_size);
      tlv->type = NRX_TLV_TYPE_BASIC_RATE_SET;
      tlv->len = nrates * sizeof(mac_rate_t);
      smd_bss_set_basic_rates(sv, (mac_rate_t*)&tlv[1], nrates);
   }

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      nrx_mlme_set_bssconf_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
      if (cfm->result != NRX_SUCCESS) {
         de_error(DE_TR_WIFI, "FAILED: nrx_mlme_set_bssconf");
         retval = -EIO;
      }
   }
   smd_txn_decref(stx);

   return retval;
}

static unsigned int
smd_sta_count_supported_rates(struct smd_sta_priv *ss)
{
   struct ieee80211_sta *sta = ss_to_sta(ss);
   unsigned int nrates;

   print_sta_info(sta);

   /* BG rates */
   nrates = count_bits(sta->supp_rates[IEEE80211_BAND_2GHZ]);

   /* N rates */
   if(sta->ht_cap.ht_supported && sta->ht_cap.mcs.rx_mask[0] != 0) {
      nrates += count_bits(sta->ht_cap.mcs.rx_mask[0]);
      if(sta->ht_cap.cap & IEEE80211_HT_CAP_SUP_WIDTH_20_40)
         nrates += count_bits(sta->ht_cap.mcs.rx_mask[0]);
   }
   return nrates;
}

static void
smd_sta_set_supp_rates(struct smd_vif_priv *sv,
                       struct smd_sta_priv *ss,
                       mac_rate_t *rates,
                       unsigned int nrates)
{
   struct smd_hw_priv *sh = sv_to_sh(sv);
   struct ieee80211_sta *sta = ss_to_sta(ss);
   unsigned int i, n;
   struct ieee80211_supported_band *band;

   band = sh->sh_hw->wiphy->bands[IEEE80211_BAND_2GHZ];

   for(n = 0, i = band->n_bitrates - 1; i < band->n_bitrates; i--) {
      if(rate_supported(sta, IEEE80211_BAND_2GHZ, i)) {
         rates[n++] = band->bitrates[i].hw_value;
      }
   }
   if(sta->ht_cap.ht_supported && sta->ht_cap.mcs.rx_mask[0] != 0) {
      for(i = 0; i <= 7; i++) {
         if(sta->ht_cap.mcs.rx_mask[0] & BIT(i))
            rates[n++] = (MAC_MOD_HT << 8) | i;
      }
      if(sta->ht_cap.cap & IEEE80211_HT_CAP_SUP_WIDTH_20_40) {
         for(i = 0; i <= 7; i++) {
            if(sta->ht_cap.mcs.rx_mask[0] & BIT(i))
               rates[n++] = (MAC_MOD_HT40 << 8) | i;
         }
      }
   }
}

static int
smd_sta_update(struct smd_vif_priv *sv,
               struct smd_sta_priv *ss)
{
   struct smd_transaction *stx;
   nrx_mlme_set_sta_info_req_t *req;
   int retval = 0;
   size_t tlv_size = 0;
   nrx_tlv_head_t *tlv;
   unsigned int nrates;

   nrates = smd_sta_count_supported_rates(ss);

   if(nrates > 0)
      tlv_size += sizeof(*tlv) + nrates * sizeof(mac_rate_t);

   req = smd_alloc_req_vif(sv, sizeof(*req) + tlv_size,
                           HIC_MESSAGE_TYPE_MLME,
                           NRX_MLME_SET_STA_INFO_REQ,
                           &stx);
   if(req == NULL)
      return -ENOMEM;

   *req = ss->ss_sta_info;

   if(nrates > 0) {
      tlv = (nrx_tlv_head_t*)&req[1];
      tlv->type = NRX_TLV_TYPE_SUPPORTED_RATE_SET;
      tlv->len = nrates * sizeof(mac_rate_t);
      smd_sta_set_supp_rates(sv, ss, (mac_rate_t*)&tlv[1], nrates);
   }

   if (smd_wait_transaction(stx) == 0) {
      nrx_mlme_set_sta_info_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
      if (cfm->result != NRX_SUCCESS) {
         retval = -EINVAL;
         de_error(DE_TR_WIFI, "FAILED: nrx_mlme_set_sta_info(peer=%pM)", &cfm->peer_sta);
      }
      smd_txn_decref(stx);
   }

   return retval;
}

static void
set_ap_sta_qos_mode(struct ieee80211_sta *sta,
                    nrx_mlme_set_sta_info_req_t *sta_info)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   if(sta->wme) {
      uint8_t uapsd_mask = 0;
      sta_info->qos_flag = true;

      /* convert to aci-indexed bitmask from qos_info format, which
       * for some reason is bit-reversed */

      if(sta->uapsd_queues & IEEE80211_WMM_IE_STA_QOSINFO_AC_VO)
         uapsd_mask |= BIT(NRX_ACI_AC_VO);
      if(sta->uapsd_queues & IEEE80211_WMM_IE_STA_QOSINFO_AC_VI)
         uapsd_mask |= BIT(NRX_ACI_AC_VI);
      if(sta->uapsd_queues & IEEE80211_WMM_IE_STA_QOSINFO_AC_BE)
         uapsd_mask |= BIT(NRX_ACI_AC_BE);
      if(sta->uapsd_queues & IEEE80211_WMM_IE_STA_QOSINFO_AC_BK)
         uapsd_mask |= BIT(NRX_ACI_AC_BK);

      sta_info->uapsd_delivery_enabled_ac_mask = uapsd_mask;
      sta_info->uapsd_trigger_enabled_ac_mask = uapsd_mask;
      sta_info->uapsd_max_sp_len_code = sta->max_sp;
   } else {
#endif
      sta_info->qos_flag = false;
      sta_info->uapsd_delivery_enabled_ac_mask = 0;
      sta_info->uapsd_trigger_enabled_ac_mask = 0;
      sta_info->uapsd_max_sp_len_code = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   }
#endif
}

static void
set_client_sta_qos_mode(struct ieee80211_sta *sta,
                        nrx_mlme_set_sta_info_req_t *sta_info,
                        uint8_t uapsd_mask)
{
   /* we're client, qos_flag is set via bss_info_changed */
   if(sta_info->qos_flag) {
      sta_info->uapsd_delivery_enabled_ac_mask = uapsd_mask;
      sta_info->uapsd_trigger_enabled_ac_mask = uapsd_mask;
      sta_info->uapsd_max_sp_len_code = 0;
   } else {
      sta_info->uapsd_delivery_enabled_ac_mask = 0;
      sta_info->uapsd_trigger_enabled_ac_mask = 0;
      sta_info->uapsd_max_sp_len_code = 0;
   }
}

static void
set_sta_info_from_sta(struct smd_vif_priv *sv, struct ieee80211_sta *sta)
{
   struct smd_sta_priv *ss = sta_to_ss(sta);

   if (sta->ht_cap.ht_supported) {
      switch (sta->ht_cap.ampdu_factor) {
         case IEEE80211_HT_MAX_AMPDU_8K:
            ss->ss_sta_info.ampdu_max_size = 8191;
            break;
         case IEEE80211_HT_MAX_AMPDU_16K:
            ss->ss_sta_info.ampdu_max_size = 16383;
            break;
         case IEEE80211_HT_MAX_AMPDU_32K:
            ss->ss_sta_info.ampdu_max_size = 32767;
            break;
         case IEEE80211_HT_MAX_AMPDU_64K:
            ss->ss_sta_info.ampdu_max_size = 65535;
            break;
      }
      ss->ss_sta_info.ampdu_min_mpdu_start_spacing = sta->ht_cap.ampdu_density;
   }

   if(sta->aid != 0) {
      set_ap_sta_qos_mode(sta, &ss->ss_sta_info);
   } else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
      set_client_sta_qos_mode(sta, &ss->ss_sta_info, sv->sv_psconf.uapsd);
#else
      struct smd_hw_priv *sh = sv_to_sh(sv);
      set_client_sta_qos_mode(sta, &ss->ss_sta_info, sh->sh_psconf.uapsd);
#endif
   }

   smd_sta_update(sv, ss);
}

static int
set_beacon_ie_filter(struct smd_vif_priv *sv)
{
   struct smd_transaction                *stx;
   nrx_set_beacon_ie_vendor_filter_req_t *req;
   int retval;

   req = smd_alloc_req_vif(sv,
                           sizeof(*req),
                           HIC_MESSAGE_TYPE_MLME,
                           NRX_MLME_SET_BEACON_IE_FILTER_REQ,
                           &stx);
   if(req == NULL)
      return -ENOMEM;

   memset(req, 0, sizeof(*req));
#define ADD_BEACON_IE_FILTER(_id)                               \
   req->ie_std[IE_FILTER_INDEX(_id)] |= IE_FILTER_BIT(_id)

   ADD_BEACON_IE_FILTER(WLAN_EID_SSID);
   ADD_BEACON_IE_FILTER(WLAN_EID_SUPP_RATES);
   ADD_BEACON_IE_FILTER(WLAN_EID_ERP_INFO);
   ADD_BEACON_IE_FILTER(WLAN_EID_EXT_SUPP_RATES);

#undef ADD_BEACON_IE_FILTER

   if(SV_IS_P2P(sv)) {
      req->num_ie_vendor = 1;
      req->ie_vendor[0].pattern_len = 4;
      put_unaligned_be32(P2P_OUI, req->ie_vendor[0].pattern);
   }

   de_info(DE_TR_WIFI, "set_beacon_ie_filter p2p=%u", SV_IS_P2P(sv));
   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      nrx_set_beacon_ie_vendor_filter_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
      if (cfm->status != NRX_SUCCESS) {
         de_error(DE_TR_WIFI, "FAILED: nrx_set_beacon_ie");
         retval = -EIO;
      }
   }
   smd_txn_decref(stx);

   return retval;
}

static int
set_beacon_loss_trigger(struct smd_vif_priv *sv)
{
   struct ieee80211_vif *vif = sv_to_vif(sv);
   uint32_t interval;
   int ret;

   if(vif->bss_conf.beacon_int != 0)
      interval = vif->bss_conf.beacon_int * 5;
   else
      interval = 2000; /* ~2 seconds */

   de_info(DE_TR_WIFI, "setting beacon loss interval to %u TU",
           interval);

   interval *= 1024; /* convert to us */

   ret = smd_mib_trigger_set(sv,
                             NRX_MIB_ID_DOT11_PRIVATE_BEACON_LOSS_RATE,
                             MIB_TRIGGER_BEACON_LOSS,
                             true,
                             interval,
                             80); /* 80% loss during interval */
   if(ret == 0) {
      set_bit(SMD_FLAG_SV_BEACON_TRIGGER, &sv->sv_flags);
   } else {
      de_warn(DE_TR_WIFI, "failed to set beacon loss trigger");
   }

   return ret;
}

static int
remove_beacon_loss_trigger(struct smd_vif_priv *sv)
{
   int ret;

   if(!test_and_clear_bit(SMD_FLAG_SV_BEACON_TRIGGER, &sv->sv_flags))
      return 0;

   ret = smd_mib_trigger_remove(sv, MIB_TRIGGER_BEACON_LOSS);

   if(ret != 0)
      de_warn(DE_TR_WIFI, "failed to remove beacon loss trigger");

   return ret;
}

/**
 * Called for transaction when the _REQ message has been sent.
 */
void
smd_command_complete(struct smd_transaction *stx)
{
   de_debug(DE_TR_WIFI, "tid=%d", stx->st_transid);
   smd_txn_decref(stx);
}

/**
 * Called for transaction when the _CFM message was received.
 */
void
smd_command_confirm(struct smd_transaction *stx)
{
   de_debug(DE_TR_WIFI, "tid=%d", stx->st_transid);
   complete(&stx->st_completion);
}

/**
 * Called by nanocore when a message has been sent to chip
 */
static void
smd_transport_complete(struct nanoc_context *context, struct sk_buff *skb, int result)
{
   struct ieee80211_hw    *hw  = (struct ieee80211_hw *) context;
   struct smd_hw_priv     *sh  = hw_to_sh(hw);
   nrx_hic_message_t      *hdr = de_pdu_data(skb);
   struct smd_transaction *stx = smd_txn_getref(sh, hdr->trans_id);

   if (stx == NULL) {
      de_info(DE_TR_WIFI, "No transaction for completed tx (tid=%d)", hdr->trans_id);
   } else {
      BUG_ON(stx->st_tx_complete == NULL);
      stx->st_tx_complete(stx);
      smd_txn_decref(stx); //of smd_txn_getref
   }
}

static void
print_bssconf(nrx_bss_conf_t *c)
{
   de_info(DE_TR_WIFI, "bssid           = %pM", c->bssid.octet);
   de_info(DE_TR_WIFI, "capability_info = %#x", c->capability_info);
   de_info(DE_TR_WIFI, "channel freq    = %u",  c->chn_prop.props.freq);
   de_info(DE_TR_WIFI, "channel flags   = %#x", c->chn_prop.props.flags);
   de_info(DE_TR_WIFI, "tx power        = %d",  c->chn_prop.props.tx_dbm);
   de_info(DE_TR_WIFI, "beacon_period   = %u",  c->beacon_period);
   de_info(DE_TR_WIFI, "p2p_flags       = %u",  c->p2p_flags);
}


static unsigned
smd_get_q(struct nanoc_context *context, unsigned net_id, struct sk_buff *skb)
{
   struct ieee80211_hw    *hw  = (struct ieee80211_hw *) context;
   struct smd_hw_priv     *sh  = hw_to_sh(hw);
   nrx_hic_message_t      *hdr = de_pdu_data(skb);
   struct smd_transaction *stx = smd_txn_getref(hw_to_sh(hw), hdr->trans_id);
   unsigned                q   = 0;

   if (stx == NULL) {
      de_error(DE_TR_WIFI, "Invalid packet received, tid=%d", hdr->trans_id);
      if (!test_and_set_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
         nanoc_req_coredump(context);
      }
   } else {
      q = stx->st_queue;
      smd_txn_decref(stx); //of smd_txn_getref
   }
   return q;
}

static void smd_handle_frame_stream_ctrl_ind(struct smd_hw_priv *sh,
                                             struct ieee80211_vif *vif,
                                             struct sk_buff *skb)
{
   nrx_frame_stream_ctrl_ind_t *ind = (nrx_frame_stream_ctrl_ind_t*)skb->data;
   nrx_tlv_head_t *tlv;

   FOREACH_TLV(tlv, &ind[1], skb->len - sizeof(nrx_frame_stream_ctrl_ind_t)) {
      switch(tlv->type) {
         case NRX_TLV_TYPE_FRAME_COPY_REQ:
            smd_retransmit_frames(sh, (nrx_tlv_frame_copy_req_t*)tlv);
            break;

         default:
            break;
      }
   }
   dev_kfree_skb(skb);
}

static void
smd_rx_packet(struct nanoc_context *context, unsigned net_id, struct sk_buff *skb)
{
   struct ieee80211_hw    *hw = (struct ieee80211_hw *) context;
   struct smd_hw_priv     *sh = hw_to_sh(hw);
   struct ieee80211_vif   *vif = smd_lookup_vif(sh, net_id);
   nrx_hic_message_t      *hdr = (nrx_hic_message_t *)skb->data;
   struct smd_transaction *stx;

   if (skb->len < sizeof *hdr) {
      de_error(DE_TR_WIFI, "Dropping runt frame (len=%u)", skb->len);
      if (de_trace_enabled(DE_ERR, DE_TR_WIFI)) {
         nanoc_printbuf("RUNT", skb->data, skb->len);
      }
      dev_kfree_skb(skb);
      return;
   }

   skb_pull(skb, sizeof *hdr);

   switch (hdr->id & MAC_API_PRIMITIVE_TYPE_BIT) {
      /*
       * Handling _CFM messages. Lookup transaction and call confirm handler.
       */
      case MAC_API_PRIMITIVE_TYPE_CFM:
         stx = smd_txn_getref(sh, hdr->trans_id);
         if (stx == NULL) {
            de_warn(DE_TR_WIFI, "No transaction for trans_id=%u", hdr->trans_id);
            de_pdu_free(skb);
         } else {
            smd_completion_func_t tx_confirm;
            HWLOCK(sh);
            tx_confirm = stx->st_tx_confirm;
            if(tx_confirm != NULL) {
               stx->st_tx_confirm = NULL;
               stx->st_rxskb      = skb;
               HWUNLOCK(sh);
               tx_confirm(stx);
            } else {
               HWUNLOCK(sh);
            }
            smd_txn_decref(stx); //of smd_txn_getref
         }
         break;

      /*
       * Handling of _IND messages
       */
      case MAC_API_PRIMITIVE_TYPE_IND:

         if (hdr->type == HIC_MESSAGE_TYPE_FRAME && hdr->id == NRX_FRAME_RX_IND) {
            /*
             * Received frame
             */
            smd_rx_generic(sh, vif, skb);

         } else if(hdr->type == HIC_MESSAGE_TYPE_FRAME && hdr->id == NRX_FRAME_STREAM_CTRL_IND) {
            smd_handle_frame_stream_ctrl_ind(sh, vif, skb);
         } else if (hdr->type == HIC_MESSAGE_TYPE_SCAN && hdr->id == NRX_SCAN_READY_IND) {
            /*
             * Scan job completed.
             */
            if(resource_release_scan(sh, SMD_FLAG_SCAN_SINGLE) == 0) {
               ieee80211_scan_completed(sh->sh_hw, false);
               de_info(DE_TR_WIFI, "single scan complete");
            }
            de_pdu_free(skb);

         } else if (hdr->type == HIC_MESSAGE_TYPE_MLME && hdr->id == NRX_MLME_PEER_STATUS_IND) {
            /*
             * Peer status indication
             */
            nrx_mlme_peer_status_ind_t *ind = (nrx_mlme_peer_status_ind_t *) de_pdu_data(skb);

            switch (ind->status)
            {
               case MLME_PEER_STATUS_PS_FINISHED:
                  p2p_ps_finished(vif);
                  break;

               case MLME_PEER_STATUS_RSSI_TRIGGER_RISING:
                  ieee80211_cqm_rssi_notify(vif,
                                            NL80211_CQM_RSSI_THRESHOLD_EVENT_HIGH,
                                            GFP_KERNEL);
                  break;

               case MLME_PEER_STATUS_RSSI_TRIGGER_FALLING:
                  ieee80211_cqm_rssi_notify(vif,
                                            NL80211_CQM_RSSI_THRESHOLD_EVENT_LOW,
                                            GFP_KERNEL);
                  break;

               case MLME_PEER_STATUS_RESTARTED:
               case MLME_PEER_STATUS_PS_POLL:
               case MLME_PEER_STATUS_WMM_POLL:
                  break;

               default:
                  de_warn(DE_TR_WIFI, "Peer status %#x (peer=%pM bssid=%pM)", ind->status, &ind->peer_mac, &ind->bssid);
            }
            de_pdu_free(skb);

         } else if (hdr->type == HIC_MESSAGE_TYPE_MIB && hdr->id == NRX_MIB_TRIGGER_IND) {
            /*
             * MIB trigger
             */
            nrx_mib_trigger_ind_t *ind = de_pdu_data(skb);
            if (ind->trigger_id == MIB_TRIGGER_BEACON_LOSS) {
               /* Tell mac80211 that we haven't received any beacons from the AP */
               de_info(DE_TR_WIFI, "MIB trigger beacon loss");
               ieee80211_beacon_loss(vif);
            } else {
               de_warn(DE_TR_WIFI, "Unknown MIB trigger received %d", ind->trigger_id);
            }
            de_pdu_free(skb);
         } else {
            /*
             * Unhandled IND message
             */
            de_warn(DE_TR_WIFI, "Unhandled message: type=%#x id=%#x", hdr->type, hdr->id);
            de_pdu_free(skb);
         }
         break;

      default:
         de_error(DE_TR_WIFI, "Unexpected message: type=%#x id=%#x", hdr->type, hdr->id);
         de_pdu_free(skb);
         break;
   }
}

int smd_update_sv(struct smd_vif_priv *sv)
{
   if (test_and_clear_bit(SMD_FLAG_SV_P2P_PS_CHANGED, &sv->sv_flags))
      p2p_send_ps_req(sv);

   return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)

static void
smd_op_tx(struct ieee80211_hw *hw,
          struct ieee80211_tx_control *control,
          struct sk_buff *skb)
{
   smd_tx_frame(hw, control->sta, skb);
}

#else

static void
smd_op_tx(struct ieee80211_hw *hw,
          struct sk_buff *skb)
{
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   smd_tx_frame(hw, txi->control.sta, skb);
}

#endif

static int update_bss_ie(struct smd_vif_priv *sv,
                         uint32_t ie_type,
                         const unsigned char *ie_ptr,
                         size_t ie_len)
{
   struct smd_transaction    *stx;
   nrx_mlme_set_bss_ie_req_t *req;
   nrx_mlme_set_bss_ie_cfm_t *cfm;
   int retval;

   req = smd_alloc_req_vif(sv, sizeof(*req) + ie_len,
                           HIC_MESSAGE_TYPE_MLME, NRX_MLME_SET_BSS_IE_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   req->ie_type = ie_type;
   memcpy(req->ie_list, ie_ptr, ie_len);

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if (cfm->result != NRX_SUCCESS) {
         de_error(DE_TR_WIFI, "FAILED: nrx_mlme_set_bss_ie_req: %d", cfm->result);
         retval = -EIO;
      }
   }
   smd_txn_decref(stx);

   return retval;
}

int smd_update_beacon(struct smd_vif_priv *sv)
{
   struct smd_hw_priv *sh  = hw_to_sh(sv->sv_hw);
   struct sk_buff *skb;
   struct ieee80211_mgmt *hdr;
   struct ieee80211_tim_ie *tim;
   size_t ie_totlen;
   const u8 *ie;
   unsigned char *ie_list;
   unsigned char *ie_dest;
   struct ieee80211_tx_info *txi;
   mac_retrans_t beacon_rate;

   if((sv->sv_net_type & NRX_NET_ROLE_MASTER) == 0) {
      /* not an AP */
      return 0;
   }

   de_info(DE_TR_API, "sv=%p", sv);

   skb = ieee80211_beacon_get(sv->sv_hw, sv_to_vif(sv));
   if (skb == NULL) {
      de_info(DE_TR_WIFI, "failed to get beacon");
      return -EINVAL;
   }
   txi = IEEE80211_SKB_CB(skb);

   make_rate_entry(&txi->control.rates[0],
                   sh->sh_hw->wiphy->bands[txi->band],
                   &beacon_rate,
                   NULL);

   if(sv->sv_bssconf->beacon_retrans.attrib.rate != beacon_rate.attrib.rate
      || sv->sv_bssconf->beacon_retrans.attrib.flags != beacon_rate.attrib.flags) {
      sv->sv_bssconf->beacon_retrans = beacon_rate;
      sv->sv_bssconf->beacon_retrans.attrib.tries = 1;
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }

   /* check if a NoA attribute needs to be inserted */
   p2p_insert_noa_attrib(sv, skb);

   if (de_trace_enabled(DE_DEBUG, DE_TR_WIFI)) {
      nanoc_printbuf("BEACON", skb->data, skb->len);
   }

   hdr = (struct ieee80211_mgmt*)skb->data;
   if(sv->sv_bssconf->capability_info != hdr->u.beacon.capab_info) {
      sv->sv_bssconf->capability_info = hdr->u.beacon.capab_info;
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   ie_totlen = 0;
   /* walk through ie list, counting ie size, ignoring TIM vector */
   ie = NULL;
   while((ie = smd_ie_find(hdr->u.beacon.variable,
                           skb->data + skb->len - hdr->u.beacon.variable,
                           255,
                           ie)) != NULL) {
      uint8_t ie_id = ie[0];
      uint8_t ie_len = ie[1];

      switch (ie_id) {
         case WLAN_EID_SSID:
         case WLAN_EID_TIM:
            break;
         default:
            ie_totlen += 2 + ie_len;
            break;
      }
   }

   if (ie_totlen == 0) {
      de_info(DE_TR_WIFI, "malformed beacon frame (no IE:s?)");
      dev_kfree_skb(skb);
      return -EINVAL;
   }

   ie_list = kmalloc(ie_totlen, GFP_KERNEL);
   if(ie_list == NULL) {
      de_warn(DE_TR_WIFI, "out of memory");
      dev_kfree_skb(skb);
      return -ENOMEM;
   }

   ie_dest = ie_list;
   /*
    * Now, walk the list again and update SSID and TIM vector, along with
    * common IE blob
    */
   ie = NULL;
   while((ie = smd_ie_find(hdr->u.beacon.variable,
                           skb->data + skb->len - hdr->u.beacon.variable,
                           255,
                           ie)) != NULL) {
      uint8_t ie_id = ie[0];
      uint8_t ie_len = ie[1];

      switch (ie_id) {
         case WLAN_EID_TIM:
            /* don't include TIM in common beacon data */
            tim = (struct ieee80211_tim_ie*)&ie[2];
            if (ie_len >= sizeof(*tim) && sv->sv_bssconf->dtim_period != tim->dtim_period) {
               de_info(DE_TR_WIFI, "Update DTIM period %u", tim->dtim_period);
               sv->sv_bssconf->dtim_period = tim->dtim_period;
               set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
            }
            break;
         case WLAN_EID_SSID:
            update_bss_ie(sv, MAC_API_BSS_IE_TYPE_SSID, ie, 2 + ie_len);
            break;
         default:
            memcpy(ie_dest, &ie[0], 2 + ie_len);
            ie_dest += 2 + ie_len;
            break;
      }
   }
   BUG_ON(ie_dest - ie_list != (ptrdiff_t)ie_totlen);

   update_bss_ie(sv, MAC_API_BSS_IE_TYPE_COMMON, ie_list, ie_totlen);

   kfree(ie_list);

   /* Free beacon template */
   dev_kfree_skb(skb);

   return 0;
}

static bool
smd_update_chn_prop(mac_chn_prop_t *chan_prop,
                    nrx_mac_chn_freq_t freq,
                    nrx_mac_chn_flag_t flags)
{
   bool changed = false;

   if(chan_prop->props.freq != freq) {
      chan_prop->props.freq = freq;
      changed = true;
   }
   if(chan_prop->props.flags != flags) {
      chan_prop->props.flags = flags;
      changed = true;
   }

   return changed;
}

#ifdef NRX_USE_CHANNEL_CONTEXT
bool
smd_update_chn_prop_from_chan_def(mac_chn_prop_t *chan_prop,
                                  const struct cfg80211_chan_def *chandef)
{
   nrx_mac_chn_freq_t freq = 2412;
   nrx_mac_chn_flag_t flags = 0;

   if (chandef->chan != NULL) {
      freq = chandef->chan->center_freq;

      switch(chandef->width) {
         default:
            de_warn(DE_TR_WIFI, "unexpected channel width: %u", chandef->width);
            /* FALLTHROUGH */
         case NL80211_CHAN_WIDTH_20_NOHT:
         case NL80211_CHAN_WIDTH_20:
            flags = 0;
            break;
         case NL80211_CHAN_WIDTH_40:
            if (chandef->center_freq1 > chandef->chan->center_freq)
               flags = NRX_MAC_CHN_FLAG_HT40_LOWER;
            else
               flags = NRX_MAC_CHN_FLAG_HT40_UPPER;
            break;
      }
   }

   return smd_update_chn_prop(chan_prop, freq, flags);
}
#endif

bool
smd_update_chn_prop_from_channel(mac_chn_prop_t *chan_prop,
                                 const struct ieee80211_channel *channel,
                                 enum nl80211_channel_type channel_type)
{
   nrx_mac_chn_freq_t freq;
   nrx_mac_chn_flag_t flags;

   freq = channel->center_freq;

   switch(channel_type) {
      case NL80211_CHAN_HT40PLUS:
         flags = NRX_MAC_CHN_FLAG_HT40_LOWER;
         break;
      case NL80211_CHAN_HT40MINUS:
         flags = NRX_MAC_CHN_FLAG_HT40_UPPER;
         break;
      default:
         de_warn(DE_TR_WIFI, "unexpected channel type: %u", channel_type);
         /* FALLTHROUGH */
      case NL80211_CHAN_NO_HT:
      case NL80211_CHAN_HT20:
         flags = 0;
         break;
   }

   return smd_update_chn_prop(chan_prop, freq, flags);
}


#ifdef NRX_USE_CHANNEL_CONTEXT
static void smd_sv_set_channel_def(struct smd_vif_priv *sv,
                                   const struct cfg80211_chan_def *chandef)
{
   if(smd_update_chn_prop_from_chan_def(&sv->sv_bssconf->chn_prop, chandef))
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static void smd_sv_set_channel(struct smd_vif_priv *sv,
                               const struct ieee80211_channel *channel,
                               enum nl80211_channel_type channel_type)
{
   if(smd_update_chn_prop_from_channel(&sv->sv_bssconf->chn_prop, channel, channel_type))
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */

static void smd_sv_set_txpower(struct smd_vif_priv *sv,
                               uint8_t tx_dbm)
{
   if(sv->sv_bssconf->chn_prop.props.tx_dbm != tx_dbm) {
      sv->sv_bssconf->chn_prop.props.tx_dbm = tx_dbm;
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
}

static void
change_channel_vif(struct smd_vif_priv *sv)
{
   struct smd_hw_priv *sh = sv_to_sh(sv);

   smd_sv_set_txpower(sv, sh_txpower(sh));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
   smd_sv_set_channel_def(sv, &sh->sh_hw->conf.chandef);
#else
   smd_sv_set_channel(sv, sh_current_channel(sh), sh->sh_hw->conf.channel_type);
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */

   update_bssconf(sv);
}

static void
change_channel_all_vifs(struct smd_hw_priv *sh)
{
   FOREACH_VIF(sh, SV_CHECK_BSSCONF, change_channel_vif);
}


/*
 * smd_op_config
 *
 * Handler for configuration requests. IEEE 802.11 code calls this function to
 * change hardware configuration, e.g., channel. This function should never
 * fail but returns a negative error code if it does. The callback can sleep.
 * */
static int smd_op_config(struct ieee80211_hw *hw, u32 changed)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);
   bool update_power_mgmt = false;

   de_info(DE_TR_API, "changed = %#x", changed);

   if(changed & IEEE80211_CONF_CHANGE_LISTEN_INTERVAL) {
      de_info(DE_TR_WIFI, "  listen interval = %u", hw->conf.listen_interval);
      sh->sh_listen_interval = hw->conf.listen_interval;
      update_power_mgmt = true;
   }
   if(changed & IEEE80211_CONF_CHANGE_SMPS) {
      de_info(DE_TR_WIFI, "  smps = %u", hw->conf.smps_mode);
   }
   if(changed & IEEE80211_CONF_CHANGE_MONITOR) {
      de_info(DE_TR_WIFI, "  monitor = %u", !!(hw->conf.flags & IEEE80211_CONF_MONITOR));
      if(hw->conf.flags & IEEE80211_CONF_MONITOR) {
         smd_monitor_mode_enable(sh);
#ifdef NRX_USE_CHANNEL_CONTEXT
         smd_monitor_update_chan_def(sh, &sh->sh_hw->conf.chandef);
#else
         smd_monitor_update_channel(sh,
                                    sh_current_channel(sh),
                                    sh->sh_hw->conf.channel_type);
#endif
      } else {
         smd_monitor_mode_disable(sh);
      }
   }
   if(changed & IEEE80211_CONF_CHANGE_PS) {
      de_info(DE_TR_WIFI, "  PS = %u", (hw->conf.flags & IEEE80211_CONF_PS) != 0);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
      /* From kernel 3.7, this is set per vif using BSS_CHANGED_PS */
      sh->sh_psconf.power_mgmt_mode = (hw->conf.flags & IEEE80211_CONF_PS) != 0;
#endif
      update_power_mgmt = true;
   }
   if(changed & IEEE80211_CONF_CHANGE_POWER) {
      de_info(DE_TR_WIFI, "  power = %d", hw->conf.power_level);
      if(test_bit(SMD_FLAG_SH_BROKEN_TX_POWER, &sh->sh_flags)) {
         /* XXX this is a workaround for fw failure to look at tx_dbm on
          * nrx701 */
         smd_set_tx_power(sh, sh_txpower(sh));
      }
      change_channel_all_vifs(sh);
   }
   if(changed & IEEE80211_CONF_CHANGE_CHANNEL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#define hw_chan_value(HW) ((HW)->conf.chandef.chan->hw_value)
#define hw_chan_width(HW) ((HW)->conf.chandef.width)
#else
#define hw_chan_value(HW) ((HW)->conf.channel->hw_value)
#define hw_chan_width(HW) ((HW)->conf.channel_type)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */
      de_info(DE_TR_WIFI, "  channel = %u, width = %u, offchannel = %u",
              hw_chan_value(hw),
              hw_chan_width(hw),
              !!(hw->conf.flags & IEEE80211_CONF_OFFCHANNEL));
      change_channel_all_vifs(sh);
#ifdef NRX_USE_CHANNEL_CONTEXT
      smd_monitor_update_chan_def(sh, &sh->sh_hw->conf.chandef);
#else
      smd_monitor_update_channel(sh,
                                 sh_current_channel(sh),
                                 sh->sh_hw->conf.channel_type);
#endif
   }
   if(changed & IEEE80211_CONF_CHANGE_RETRY_LIMITS) {
      de_info(DE_TR_WIFI, "  retry long = %u, short = %u",
            hw->conf.long_frame_max_tx_count,
            hw->conf.short_frame_max_tx_count);
   }
   if(changed & IEEE80211_CONF_CHANGE_IDLE) {
      de_info(DE_TR_WIFI, "  hw idle = %u", !!(hw->conf.flags & IEEE80211_CONF_IDLE));
   }

   if (update_power_mgmt) {
      FOREACH_VIF(sh, SV_CHECK_IFTYPE_STATION, set_power_mgmt, sh);
   }

   return 0;
}

static void
smd_op_flush(struct ieee80211_hw *hw,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
	     struct ieee80211_vif *vif,
             u32 queues,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */
             bool drop)
{
   struct smd_hw_priv     *sh = hw_to_sh(hw);

   de_info(DE_TR_WIFI, "drop = %d", drop);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
   /* mac80211 should stop all queues before calling flush, but
    * this is not done in one case when sending null frames to the AP.
    * (This bug seems to be corrected in kernel 3.10)
    */
   set_bit(SMD_FLAG_SH_FLUSHING, &sh->sh_flags);
   ieee80211_stop_queues(hw);
#endif
   if(drop || !smd_tx_queue_barrier(sh)) {
      flush_tx_queues(sh, drop);
   }

   if (smd_wait_all_transactions(sh) < 0) {
      smd_fail_transactions(sh);
   }
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
   clear_bit(SMD_FLAG_SH_FLUSHING, &sh->sh_flags);
   if (test_bit(SMD_FLAG_SH_LINKUP_DATA, &sh->sh_flags))
   {
      ieee80211_wake_queues(hw);
   }
#endif
}

#define IS_IPV4_MCAST(A) ((A).octet[0] == 0x01         \
                          && (A).octet[1] == 0x00      \
                          && (A).octet[2] == 0x5e)

#define IS_IPV6_MCAST(A) ((A).octet[0] == 0x33         \
                          && (A).octet[1] == 0x33)

static void
smd_rxfiltering_update(struct smd_vif_priv *sv)
{
   struct smd_hw_priv *sh  = hw_to_sh(sv->sv_hw);
   group_addr_filter_t mc_filter;
   unsigned int i;

   if(sh->sh_rxfilter == 0) {
      smd_sv_set_group_filter(sv, &sh->sh_mc_filter);
      return;
   }
   if(sh->sh_mc_filter.discard) {
      mc_filter = sh->sh_mc_filter;

      if(sh->sh_mc_filter.size == 0) {
         /* we're in ALLMULTI land */

         if(sh->sh_rxfilter & NRX_RXFILTER_BROADCAST) {
            smd_broadcast_addr(&mc_filter.mac[0]);
            mc_filter.size++;
         }
         /* not much to do with IPv{4,6} multicast */
      }
   } else {
      mc_filter.discard = 0;
      mc_filter.size = 0;

      for(i = 0; i < sh->sh_mc_filter.size; i++) {
         if((sh->sh_rxfilter & NRX_RXFILTER_BROADCAST)
            && is_broadcast_ether_addr(sh->sh_mc_filter.mac[i].octet))
            continue;

         if((sh->sh_rxfilter & NRX_RXFILTER_MULTICAST4)
            && IS_IPV4_MCAST(sh->sh_mc_filter.mac[i]))
            continue;

         if((sh->sh_rxfilter & NRX_RXFILTER_MULTICAST6)
            && IS_IPV6_MCAST(sh->sh_mc_filter.mac[i]))
            continue;

         mc_filter.mac[mc_filter.size] = sh->sh_mc_filter.mac[i];
         mc_filter.size++;
      }
   }
   smd_sv_set_group_filter(sv, &mc_filter);
}



#ifdef CONFIG_NL80211_TESTMODE

#include <net/netlink.h>

static void
smd_restart_peers(struct smd_hw_priv *sh)
{	
   struct sk_buff   *skb;
   nrx_buf_restart_t nrx_buf;

   if ( sh==NULL || sh->sh_hw==NULL || sh->sh_hw->wiphy==NULL ) {
      de_debug(DE_TR_WIFI, "no environment");
      return;
   }
   
   /* TODO: consider doing this per-vif */
   
   if( (skb = cfg80211_testmode_alloc_event_skb(sh->sh_hw->wiphy, 20, GFP_KERNEL)) == NULL ) {   
      de_debug(DE_TR_WIFI, "failed to allocate event skb");
      return;
   }

   nrx_buf.id = NRX_CMD_RESTART_PEERS;
   nla_put(skb, NL80211_ATTR_TESTDATA, sizeof(nrx_buf), (const u8*)&nrx_buf);

   de_debug(DE_TR_WIFI, "sending testmode event");
   cfg80211_testmode_event(skb, GFP_KERNEL);
}

static void
smd_rxfiltering_cmd(struct smd_vif_priv *sv, nrx_buf_rxfilter_t *rxfil)
{
   struct smd_hw_priv             *sh  = hw_to_sh(sv->sv_hw);

   switch(rxfil->type){
      case NRX_RXFILTER_START:
         /* start the rx filtering for the bits that we have recv */
         sh->sh_rxfilter = rxfil->filter;
         break;

      case NRX_RXFILTER_STOP:
         /* stop the rx filtering */
         sh->sh_rxfilter = 0;
         break;
   }

   smd_rxfiltering_update(sv);
}


/* smd_op_testmode_cmd
 *
 * @testmode_cmd: run a test mode command
 */
static int
smd_op_testmode_cmd(struct ieee80211_hw *hw, void *data, int len)
{
   struct smd_hw_priv * sh = hw_to_sh(hw);
   uint8_t              cmd_id;

   if(data==NULL || len==0)
      return 0;

   nanoc_printbuf("TEST_CMD", data, len);

   cmd_id = ((uint8_t*)data)[0];

   switch(cmd_id) {
      case NRX_CMD_SET_NOA:
         FOREACH_VIF(sh, p2p_update_noa_from_nl, data);
         break;
      case NRX_CMD_SET_OPP:
         FOREACH_VIF(sh, p2p_update_opp_from_nl, data);
         break;
      case NRX_CMD_RXFILTER:
          FOREACH_VIF(sh, smd_rxfiltering_cmd, data);
         break;
      default:
         break;
   }
   return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
/* smd_op_testmode_dump
 *
 * @testmode_dump: Implement a test mode dump. The cb->args[2] and up may be
 *	used by the function, but 0 and 1 must not be touched. Additionally,
 *	return error codes other than -ENOBUFS and -ENOENT will terminate the
 *	dump and return to userspace with an error, so be careful. If any data
 *	was passed in from userspace then the data/len arguments will be present
 *	and point to the data contained in %NL80211_ATTR_TESTDATA.
 */
static int
smd_op_testmode_dump(struct ieee80211_hw *hw, struct sk_buff *skb,
                     struct netlink_callback *cb,
                     void *data, int len)
{
   nanoc_printbuf("TEST_DUMP", data, len);
   return 0;
}
#endif  /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0) */

#endif  /* CONFIG_NL80211_TESTMODE */

/*
 * smd_op_remain_on_channel
 *
 * @remain_on_channel: Starts an off-channel period on the given channel, must
 *      call back to ieee80211_ready_on_channel() when on that channel. Note
 *      that normal channel traffic is not stopped as this is intended for hw
 *      offload. Frames to transmit on the off-channel channel are transmitted
 *      normally except for the %IEEE80211_TX_CTL_TX_OFFCHAN flag. When the
 *      duration (which will always be non-zero) expires, the driver must call
 *      ieee80211_remain_on_channel_expired(). This callback may sleep.
 */
static int
smd_op_remain_on_channel(struct ieee80211_hw *hw,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
                         struct ieee80211_vif *vif,
                         struct ieee80211_channel *chan,
#else
                         struct ieee80211_channel *chan,
                         enum nl80211_channel_type channel_type,
#endif
                         int duration
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
                         , enum ieee80211_roc_type type
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */
   )
{
   struct smd_hw_priv *sh = hw_to_sh(hw);
   mac_chn_prop_t channel;
   int retval;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
   enum nl80211_channel_type channel_type = NL80211_CHAN_HT20;
#endif

   de_info(DE_TR_WIFI, "remain-on-channel freq=%u duration=%u", chan->center_freq, duration);

   if(resource_request_scan(sh, SMD_FLAG_SCAN_ROC) < 0) {
      de_info(DE_TR_WIFI, "Scan BUSY");
      return -EAGAIN;
   }

   smd_update_chn_prop_from_channel(&channel, chan, channel_type);
   channel.props.flags  |= NRX_MAC_CHN_FLAG_PASSIVE_SCAN;
   channel.props.tx_dbm = chan->max_power;

   retval = smd_scan_single_passive_channel(sh,
                                            &channel,
                                            NRX_SCAN_FLAG_SEND_QUEUED|NRX_SCAN_FLAG_BLOCK_INDS);

   if(retval == 0) {
      /* normally this should be called when the firmware actually
       * starts the passive scan. But f/w only indicates for each scan
       * result and the end of scan, so call here
       */
      ieee80211_ready_on_channel(hw);
      de_timer_start(&sh->sh_scan_timer, duration);
   } else {
      de_error(DE_TR_WIFI, "failed to start ROC scan job: %d", retval);
      resource_release_scan(sh, SMD_FLAG_SCAN_ROC);
   }

   return retval;
}

/*
 * smd_op_cancel_remain_on_channel
 *
 * @cancel_remain_on_channel: Requests that an ongoing off-channel period is
 *      aborted before it expires. This callback may sleep.
 */
static int
smd_op_cancel_remain_on_channel(struct ieee80211_hw *hw)
{
   struct smd_hw_priv     *sh = hw_to_sh(hw);

   if(resource_release_scan(sh, SMD_FLAG_SCAN_ROC) == 0) {
      de_timer_stop(&sh->sh_scan_timer);
      smd_stop_scan_job(sh);
      de_info(DE_TR_WIFI, "remain-on-channel cancelled");
   }
   return 0;
}

static void
scan_timeout_cb(de_timer_t *t)
{
   struct smd_hw_priv *sh = container_of(t, struct smd_hw_priv, sh_scan_timer);

   if(resource_release_scan(sh, SMD_FLAG_SCAN_ROC) == 0) {
      smd_stop_scan_job(sh);
      ieee80211_remain_on_channel_expired(sh->sh_hw);
      de_info(DE_TR_WIFI, "remain-on-channel complete (scan timeout for sh=%p)", sh);
   }
}

static void
remove_vif(struct smd_vif_priv *sv)
{
   if (sv->sv_hw != NULL) {
      struct smd_hw_priv *sh = sv_to_sh(sv);

      rcu_assign_pointer(sh->sh_vif[sv->sv_index], NULL);
      synchronize_rcu();
      if (sv->sv_net_id) {
         nanoc_free_net_id((struct nanoc_context*)sv->sv_hw, sv->sv_net_id);
      }
      HWLOCK(sh);
      kfree(sv->sv_bssconf);
      memset(sv, 0, sizeof(*sv));
      HWUNLOCK(sh);
   }
}

static void
remove_vifs(struct smd_hw_priv *sh)
{
   FOREACH_VIF(sh, remove_vif);
}

static void
smd_link_up(struct nanoc_context *context, unsigned qmask)
{
   struct ieee80211_hw *hw = (struct ieee80211_hw *)context;
   struct smd_hw_priv *sh = hw_to_sh(hw);
   bool wakeup = false;

   if(sh->sh_api_version == 0) {
      sh->sh_api_version = nanoc_get_api_version(context);
   }

   if (qmask & (1<<NANOC_QDATA)) {
      set_bit(SMD_FLAG_SH_LINKUP_DATA, &sh->sh_flags);
      wakeup = true;
   }
   if (qmask & (1<<NANOC_QCMD)) {
      set_bit(SMD_FLAG_SH_LINKUP_CMD, &sh->sh_flags);
      wakeup = true;
   }
   if(wakeup) {
      wake_up_interruptible(&sh->sh_link_wait);
      de_debug(DE_TR_WIFI, "flags=%lu", sh->sh_flags);
      QUEUE_TXWORK(sh);
   }
}

static void
smd_link_down(struct nanoc_context *context, unsigned qmask)
{
   struct ieee80211_hw *hw = (struct ieee80211_hw *)context;
   struct smd_hw_priv *sh = hw_to_sh(hw);

   if (qmask & (1<<NANOC_QDATA)) {
      clear_bit(SMD_FLAG_SH_LINKUP_DATA, &sh->sh_flags);
      ieee80211_stop_queues(hw);
   }
   if (qmask & (1<<NANOC_QCMD))
      clear_bit(SMD_FLAG_SH_LINKUP_CMD, &sh->sh_flags);
}

static void
smd_driver_reset(struct nanoc_context *context)
{
   struct ieee80211_hw *hw = (struct ieee80211_hw *)context;
   struct smd_hw_priv *sh = hw_to_sh(hw);

   set_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags);

   de_info(DE_TR_WIFI, "hw=%p", hw);

   ieee80211_stop_queues(hw);
   flush_tx_queues(sh, true);

   wake_up(&sh->sh_link_wait);
   if(resource_release_scan(sh, SMD_FLAG_SCAN_SINGLE) == 0) {
      smd_stop_scan_job(sh);
      ieee80211_scan_completed(sh->sh_hw, true);
      de_info(DE_TR_WIFI, "Aborted single scan");
   } else if(resource_release_scan(sh, SMD_FLAG_SCAN_ROC) == 0) {
      smd_stop_scan_job(sh);
      de_timer_stop(&sh->sh_scan_timer);
      ieee80211_remain_on_channel_expired(sh->sh_hw);
      de_info(DE_TR_WIFI, "Aborted remain-on-channel");
   } else if(resource_release_scan(sh, SMD_FLAG_SCAN_MONITOR) == 0) {
      smd_stop_scan_job(sh);
      de_info(DE_TR_WIFI, "Aborted monitor mode");
   }
   smd_fail_transactions(sh);

   ieee80211_restart_hw(sh->sh_hw);
   
#ifdef CONFIG_NL80211_TESTMODE
   smd_restart_peers(sh);
#endif
}

static int set_key(struct smd_vif_priv *sv,
                   struct ieee80211_sta *sta,
                   struct ieee80211_key_conf *key)
{
   struct smd_transaction *stx;
   nrx_mlme_set_key_req_t *req;
   nrx_mlme_set_key_cfm_t *cfm;
   nrx_cipher_suite_t      cipher_suite;
   nrx_key_type_t          key_type;
   int retval;

   if (key->keylen > 32) {
      de_info(DE_TR_WIFI, "key too long: %u", key->keylen);
      return -ENOSPC;
   }

   switch (key->cipher) {
      case WLAN_CIPHER_SUITE_WEP40:
      case WLAN_CIPHER_SUITE_WEP104:
         cipher_suite = NRX_CIPHER_SUITE_WEP;
         break;
      case WLAN_CIPHER_SUITE_TKIP:
         cipher_suite = NRX_CIPHER_SUITE_TKIP;
         break;
      case WLAN_CIPHER_SUITE_CCMP:
         cipher_suite = NRX_CIPHER_SUITE_CCMP;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
         key->flags |= IEEE80211_KEY_FLAG_SW_MGMT_TX;
#else
         key->flags |= IEEE80211_KEY_FLAG_SW_MGMT;
#endif
         break;
      case WLAN_CIPHER_SUITE_AES_CMAC:
      default:
         de_info(DE_TR_WIFI, "unsupported cipher suite %x", key->cipher);
         return -EOPNOTSUPP;
   }

   if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE)
      key_type = NRX_KEY_TYPE_PAIRWISE;
   else
      key_type = NRX_KEY_TYPE_GROUP;

   req = smd_alloc_req_vif(sv, sizeof *req,
                           HIC_MESSAGE_TYPE_MLME, NRX_MLME_SET_KEY_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   memcpy(&req->descr[0].key, key->key, key->keylen);
   req->descr[0].key_len  = key->keylen;
   req->descr[0].key_id   = key->keyidx;
   req->descr[0].key_type = key_type;
   if (sta)
      memcpy(&req->descr[0].mac_addr, sta->addr, ETH_ALEN);
   else
      smd_broadcast_addr(&req->descr[0].mac_addr);
   memset(&req->descr[0].receive_seq_cnt[0], 0,
          sizeof(req->descr[0].receive_seq_cnt));
   req->descr[0].config_by_authenticator = true;
   req->descr[0].cipher_suite = cipher_suite;
   memset(&req->descr[0].reserved, 0, sizeof req->descr[0].reserved);

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if (cfm->result != NRX_SUCCESS) {
         de_info(DE_TR_WIFI, "Set key FAILED");
         retval = -EIO;
      }
   }
   smd_txn_decref(stx);

   key->hw_key_idx = 0;

   return retval;
}

static int
smd_alloc_vif(struct smd_hw_priv *sh, struct smd_vif_priv *sv)
{
   unsigned i;

   for (i=0; i<ARRAY_SIZE(sh->sh_vif); i++) {
      if (sh->sh_vif[i] == NULL) {
         rcu_assign_pointer(sh->sh_vif[i], sv);
         sv->sv_index  = i;
         sv->sv_hw     = sh->sh_hw;
         sv->sv_net_id = nanoc_alloc_net_id((struct nanoc_context *)sh->sh_hw);
         return 0;
      }
   }

   return -ENOMEM;
}

struct ieee80211_vif *
smd_lookup_vif(struct smd_hw_priv *sh, unsigned net_id)
{
   struct ieee80211_vif *vif = NULL;
   struct smd_vif_priv *sv;
   unsigned i;

   rcu_read_lock();
   for (i=0; i<ARRAY_SIZE(sh->sh_vif); i++) {
      sv = rcu_dereference(sh->sh_vif[i]);
      if (sv != NULL && sv->sv_net_id == net_id) {
         vif = sv_to_vif(sv);
         break;
      }
   }
   rcu_read_unlock();

   return vif;
}

/*
 *---------------------------------------------------------------------------
 *                      DRIVER CALLBACKS
 *---------------------------------------------------------------------------
 */


/*
 * smd_op_start
 *
 * Called before the first netdevice attached to the hardware is
 * enabled. This should turn on the hardware and must turn on frame
 * reception (for possibly enabled monitor interfaces.) Returns
 * negative error codes, these may be seen in userspace, or zero. When
 * the device is started it should not have a MAC address to avoid
 * acknowledging frames before a non-monitor device is added. Must be
 * implemented and can sleep. */
static int smd_op_start(struct ieee80211_hw *hw)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);
   int ret;

   de_info(DE_TR_WIFI, "hw = %p", hw);
   if (sh->sh_dummy_netid == 0) {
      sh->sh_dummy_netid = NRX_NET_ID_WIFI_UNKNOWN;
      nanoc_alloc_fixed_net_id(sh->sh_dummy_netid, (struct nanoc_context *)hw);
   }

   flush_single_queue(sh, &sh->sh_txqueue, false);
   smd_fail_transactions(sh);
   remove_vifs(sh);

   if (!test_and_set_bit(SMD_FLAG_SH_CHIP_UP, &sh->sh_flags)) {
      if (nanoc_chip_up((struct nanoc_context *)hw, NANOC_FEAT_WIFI_STA) < 0) {
         de_info(DE_TR_WIFI, "nanoc_chip_up() failed. Missing fw file?");
         clear_bit(SMD_FLAG_SH_CHIP_UP, &sh->sh_flags);
         return -ENOENT;
      }
   }

   ret = wait_event_interruptible_timeout(sh->sh_link_wait,
                                          test_bit(SMD_FLAG_SH_LINKUP_CMD, &sh->sh_flags),
                                          msecs_to_jiffies(1000));
   de_info(DE_TR_WIFI, "wait_event ret = %d", ret);
   if (ret <= 0) {
      return -ETIMEDOUT;
   }

   clear_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags);

   if(0 != frame_config(sh, 0)) {
      return -EAGAIN;
   }

   if (smd_set_fcc(sh, fcc) != 0) {
      de_warn(DE_TR_WIFI, "Failed to set fcc mode");
   }

   ieee80211_wake_queues(sh->sh_hw);
   return 0;
}

/*
 * smd_op_stop
 *
 * Called after last netdevice attached to the hardware is
 * disabled. This should turn off the hardware (at least it must turn
 * off frame reception.) May be called right after add_interface if
 * that rejects an interface. If you added any work onto the mac80211
 * workqueue you should ensure to cancel it on this callback. Must be
 * implemented and can sleep.
 */
static void smd_op_stop(struct ieee80211_hw *hw)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);

   de_info(DE_TR_WIFI, "hw = %p", hw);

   if(resource_release_scan(sh, SMD_FLAG_SCAN_SINGLE) == 0) {
      smd_stop_scan_job(sh);
      ieee80211_scan_completed(sh->sh_hw, true);
      de_info(DE_TR_WIFI, "Aborted single scan");
   } else if(resource_release_scan(sh, SMD_FLAG_SCAN_ROC) == 0) {
      smd_stop_scan_job(sh);
      de_timer_stop(&sh->sh_scan_timer);
      ieee80211_remain_on_channel_expired(sh->sh_hw);
      de_info(DE_TR_WIFI, "Aborted remain-on-channel");
   }
   smd_monitor_mode_disable(sh);

   if (test_and_clear_bit(SMD_FLAG_SH_CHIP_UP, &sh->sh_flags)) {
      if (!test_and_clear_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
         nanoc_chip_down((struct nanoc_context *)hw);
      }
   }
}

static int create_net(struct smd_vif_priv *sv)
{
   nrx_mlme_net_create_req_t *req;
   nrx_mlme_net_create_cfm_t *cfm;
   struct smd_transaction *stx;
   int retval;

   req = smd_alloc_req_vif(sv, sizeof *req,
                           HIC_MESSAGE_TYPE_MLME,
                           NRX_MLME_NET_CREATE_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   req->net_type = sv->sv_net_type;
   req->reserved = 0;
   memcpy(req->mac_address.octet, sv->sv_address, ETH_ALEN);

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->result != NRX_SUCCESS) {
         de_warn(DE_TR_WIFI, "FAILED: mlme_net_create_req: %d", cfm->result);
         retval = -EIO;
      }
   }

   smd_txn_decref(stx);

   return retval;
}

/* this updates the bssconf with what firmware has, overwriting any
 * uncommitted changed */
static int get_bssconf(struct smd_vif_priv *sv)
{
   struct smd_transaction *stx;
   nrx_mlme_get_bssconf_req_t *req;
   int retval;
   nrx_bss_conf_t *bssconf;

   req = smd_alloc_req_vif(sv, sizeof *req,
                           HIC_MESSAGE_TYPE_MLME,
                           NRX_MLME_GET_BSSCONF_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   req->reserved = 0;

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      bssconf = kmalloc(stx->st_rxskb->len, GFP_KERNEL);
      if(unlikely(bssconf == NULL)) {
         retval = -ENOMEM;
      } else {
         memcpy(bssconf, stx->st_rxskb->data, stx->st_rxskb->len);
         if(sv->sv_bssconf != NULL)
            kfree(sv->sv_bssconf);
         sv->sv_bssconf = bssconf;
         sv->sv_bssconf_size = stx->st_rxskb->len;
      }
   }

   smd_txn_decref(stx);
   return retval;
}

/*
 * smd_op_add_interface
 *
 * Called when a netdevice attached to the hardware is
 * enabled. Because it is not called for monitor mode devices, start
 * and stop must be implemented. The driver should perform any
 * initialization it needs before the device can be enabled. The
 * initial configuration for the interface is given in the conf
 * parameter. The callback may refuse to add an interface by returning
 * a negative error code (which will be seen in userspace.) Must be
 * implemented and can sleep. */
static int smd_op_add_interface(struct ieee80211_hw *hw,
                                struct ieee80211_vif *vif)
{
   void *mac_addr = vif->addr;
   struct smd_hw_priv *sh = hw_to_sh(hw);
   struct smd_vif_priv *sv = vif_to_sv(vif);
   int retval;

   de_info(DE_TR_WIFI, "hw=%p, vif=%p, iftype=%u, p2p=%d, mac=%pM, bssid=%pM",
           hw,
           vif,
           vif->type,
           vif->p2p,
           mac_addr,
           vif->bss_conf.bssid);

   memset(sv, 0, sizeof(*sv));

   switch (vif->type) {
      case NL80211_IFTYPE_UNSPECIFIED:
      case NL80211_IFTYPE_STATION:
         vif->type       = NL80211_IFTYPE_STATION;
         sv->sv_net_type = NRX_NET_TYPE_BSS_STA;
         break;
      case NL80211_IFTYPE_ADHOC:
         sv->sv_net_type = NRX_NET_TYPE_IBSS;
         break;
      case NL80211_IFTYPE_AP:
         sv->sv_net_type = NRX_NET_TYPE_BSS_AP;
         break;
      case NL80211_IFTYPE_MONITOR:
#ifdef NRX_USE_CHANNEL_CONTEXT
         return 0;
         break;
#endif
      case NL80211_IFTYPE_AP_VLAN:
      case NL80211_IFTYPE_WDS:
      case NL80211_IFTYPE_MESH_POINT:
      default:
         de_info(DE_TR_WIFI, "Unsupported interface type %u", vif->type);
         return -EINVAL;
   }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
   vif->driver_flags |= IEEE80211_VIF_BEACON_FILTER;
   vif->driver_flags |= IEEE80211_VIF_SUPPORTS_CQM_RSSI;
#endif
   if(vif->p2p)
      set_bit(SMD_FLAG_SV_P2P, &sv->sv_flags);
   sv->sv_p2p_noa_index = -1;  /* -1 (in 32-bit) will never be set by GO therefore we can detect if this has changed */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
   sv->sv_psconf.receive_all_dtim          = 1;
   sv->sv_psconf.uapsd                     = 0;
   sv->sv_psconf.traffic_timeout           = 0;
   sv->sv_psconf.periodic_service_interval = 0;
   sv->sv_psconf.fixed_service_interval    = 0;
   sv->sv_psconf.listen_interval           = 0;
#endif
   de_timer_init(&sv->sv_p2p_timer, p2p_timeout_cb);

   if (smd_alloc_vif(sh, sv) < 0) {
      de_info(DE_TR_WIFI, "too many interfaces");
      return -ENOMEM;
   }

   memcpy(sv->sv_address, mac_addr, ETH_ALEN);

   smd_monitor_mode_disable(sh); /* XXX */

   retval = create_net(sv);
   if(retval != 0) {
      remove_vif(sv);
      return retval;
   }

   retval = get_bssconf(sv);
   if(retval != 0) {
      destroy_net(sv);
      remove_vif(sv);
      return retval;
   }

   if (sv->sv_net_type == NRX_NET_TYPE_BSS_AP) {
      /* XXX why is this necessary? */

      static const unsigned char dummy_ie_list[] = {
         WLAN_EID_DS_PARAMS, 1, 1,
      };

      update_bss_ie(sv, MAC_API_BSS_IE_TYPE_COMMON, dummy_ie_list, sizeof(dummy_ie_list));
   }

   if (sv->sv_net_type == NRX_NET_TYPE_BSS_AP) {
      memcpy(sv->sv_bssconf->bssid.octet, sv->sv_address, ETH_ALEN);
   } else {
      memset(sv->sv_bssconf->bssid.octet, 0x00, sizeof(sv->sv_bssconf->bssid.octet));
      set_bit(SMD_FLAG_SV_HIBERNATING, &sv->sv_flags);
   }

   sv->sv_bssconf->beacon_period = 100;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
   smd_sv_set_channel_def(sv, &sh->sh_hw->conf.chandef);
#else
   if (sh_current_channel(sh)) {
      smd_sv_set_channel(sv, sh_current_channel(sh),
                         sh->sh_hw->conf.channel_type);
   } else {
      smd_update_chn_prop(&sv->sv_bssconf->chn_prop, 2412, 0);
   }
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */
   smd_sv_set_txpower(sv, sh_txpower(sh));

   if(SV_IS_P2P(sv)) {
      sv->sv_bssconf->p2p_flags = NRX_P2P_FLAGS_ENABLED;
      sv->sv_bssconf->p2p_flags |= NRX_P2P_FLAGS_PS_TRACK_NOA;
   }
   print_bssconf(sv->sv_bssconf);

   set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   update_bssconf(sv);

   if ((sv->sv_net_type & NRX_NET_ROLE_MASTER) == 0) {
      set_beacon_ie_filter(sv);
   }

   if(!vif->bss_conf.idle
      || sv->sv_net_type != NRX_NET_TYPE_BSS_STA)
   {
      start_net(sv);
      set_power_mgmt(sv, sh);
   }

   set_bit(SMD_FLAG_SV_ENABLED, &(sv)->sv_flags);

   if ( sv->sv_net_type == NRX_NET_TYPE_BSS_AP && SV_IS_P2P(sv) )
   {
      p2p_go_initial_ps(sv);
   }

   return 0;
}


/*
 * smd_op_remove_interface
 *
 * Notifies a driver that an interface is going down. The stop
 * callback is called after this if it is the last interface and no
 * monitor interfaces are present. When all interfaces are removed,
 * the MAC address in the hardware must be cleared so the device no
 * longer acknowledges packets, the mac_addr member of the conf
 * structure is, however, set to the MAC address of the device going
 * away. Hence, this callback must be implemented. It can sleep. */
static void smd_op_remove_interface(struct ieee80211_hw *hw,
                                    struct ieee80211_vif *vif)
{
   void *mac_addr = vif->addr;
   struct smd_vif_priv *sv = vif_to_sv(vif);

   de_info(DE_TR_WIFI, "hw = %p, vif = %p, iftype = %u, mac = %pM",
           hw,
           vif,
           vif->type,
           mac_addr);

#ifdef NRX_USE_CHANNEL_CONTEXT
   if(vif->type == NL80211_IFTYPE_MONITOR) {
      smd_monitor_mode_disable(hw_to_sh(hw));
      return;
   }
#endif

   clear_bit(SMD_FLAG_SV_ENABLED, &(sv)->sv_flags);
   p2p_ps_finished(vif);
   stop_net(sv);
   destroy_net(sv);

   remove_vif(sv);
}

static int
smd_op_set_key(struct ieee80211_hw *hw,
               enum set_key_cmd cmd,
               struct ieee80211_vif *vif,
               struct ieee80211_sta *sta,
               struct ieee80211_key_conf *key)
{
   struct smd_vif_priv *sv = vif_to_sv(vif);
   int ret = -EOPNOTSUPP;

   BUG_ON(vif == NULL);

   if(!SV_IS_VALID(sv)) {
      de_info(DE_TR_WIFI, "called with uninitialised vif");
      return -EINVAL;
   }

   switch (cmd) {
      case SET_KEY:
         /* May fail */
         de_info(DE_TR_WIFI, "SET_KEY %p %u %u %x %u", sta, key->keyidx, key->cipher, key->flags, key->keylen);
         ret = set_key(sv, sta, key);
         break;

      case DISABLE_KEY:
         /* Must succeed */
         de_info(DE_TR_WIFI, "DISABLE_KEY %p %u %x", sta, key->keyidx, key->flags);
         ret = 0;
         break;
   }

   return ret;
}

static u64 smd_set_allmulti(struct smd_hw_priv *sh)
{
   sh->sh_mc_filter.discard = true;
   sh->sh_mc_filter.size = 0;

   return (uintptr_t)&sh->sh_mc_filter;
}

/* smd_op_prepare_multicast
 *
 * Prepare for multicast filter configuration.  This callback is
 * optional, and its return value is passed to configure_filter().
 * This callback must be atomic.
 */
static u64 smd_op_prepare_multicast(struct ieee80211_hw *hw,
                                    struct netdev_hw_addr_list *mc_list)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);
   struct netdev_hw_addr *mc;
   nrx_mac_addr_t *dst;

   sh->sh_mc_filter.discard = false;
   sh->sh_mc_filter.size = 0;
   dst = &sh->sh_mc_filter.mac[0];

   smd_broadcast_addr(dst);
   dst++;
   sh->sh_mc_filter.size++;
   netdev_hw_addr_list_for_each(mc, mc_list) {
      if(mc->type != NETDEV_HW_ADDR_T_MULTICAST)
         continue;
      if(sh->sh_mc_filter.size == MAX_NUM_GROUP_ADDRESS_FILTER_ENTRIES)
         return smd_set_allmulti(sh);
      memcpy(dst->octet, mc->addr, sizeof(dst->octet));
      dst++;
      sh->sh_mc_filter.size++;
   }
   return (uintptr_t)&sh->sh_mc_filter;
}


/*
 * smd_op_configure_filter
 *
 * Configure the device's RX filter. This callback must be implemented and can
 * sleep.
 */
static void
smd_op_configure_filter(struct ieee80211_hw *hw,
                        unsigned int changed_flags,
                        unsigned int *total_flags,
                        u64 multicast)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);

   de_info(DE_TR_WIFI, "changed=%#x total=%#x", changed_flags, *total_flags);

#define XX(F)   if(changed_flags & FIF_##F) de_info(DE_TR_WIFI, "  " #F)
   XX(ALLMULTI);
   XX(FCSFAIL);
   XX(PLCPFAIL);
   XX(BCN_PRBRESP_PROMISC);
   XX(CONTROL);
   XX(OTHER_BSS);
   XX(PSPOLL);
#undef XX

   /* this is our supported flag set */
   (*total_flags) &=
      (FIF_ALLMULTI
       | FIF_BCN_PRBRESP_PROMISC
       | FIF_CONTROL
       | FIF_OTHER_BSS
       | FIF_PROBE_REQ
       | FIF_PSPOLL
      );
   sh->sh_filter_flags = *total_flags;

   frame_config(sh, sh->sh_filter_flags);

   if(multicast == (uintptr_t)&sh->sh_mc_filter) {
      if(sh->sh_filter_flags & FIF_ALLMULTI)
         smd_set_allmulti(sh);
      FOREACH_VIF(sh, smd_rxfiltering_update);
   }
}

static int
smd_sv_update_arp_filter(struct smd_vif_priv *sv,
                         struct ieee80211_bss_conf *info,
                         bool enable)
{
   arp_addr_filter_t filter;
   unsigned int i;

   memset(&filter, 0, sizeof(filter));
   if(enable
      && info->arp_addr_cnt <= ARP_MAX_NUM_IP_ADDRESS_FILTER_ENTRIES) {
      filter.arp_policy = ARP_HANDLE_NONE_FORWARD_MYIP;
      filter.count = info->arp_addr_cnt;
      for(i = 0; i < info->arp_addr_cnt; i++) {
         filter.ip[i] = info->arp_addr_list[i];
      }
   } else {
      filter.arp_policy = ARP_HANDLE_NONE_FORWARD_ALL;
   }

   return smd_sv_set_arp_filter(sv, &filter);
}

static void
smd_sta_update_rssi_levels(struct smd_vif_priv *sv,
                           struct smd_sta_priv *ss,
                           s32 threshold,
                           u32 hysteresis)
{
   if(threshold == 0) {
      ss->ss_sta_info.rssi_trigger_rising = NRX_MAC_INVALID_RSSI_SNR_VALUE;
      ss->ss_sta_info.rssi_trigger_falling = NRX_MAC_INVALID_RSSI_SNR_VALUE;
   } else {
      /* do it this awkward way to get the level spread equal to
       * hysteresis */
      threshold += hysteresis / 2;
      ss->ss_sta_info.rssi_trigger_rising = threshold;
      threshold -= hysteresis;
      ss->ss_sta_info.rssi_trigger_falling = threshold;
   }
   smd_sta_update(sv, ss);
}

/* Handler for configuration requests related to BSS
 * parameters that may vary during BSS's lifespan, and may affect low
 * level driver (e.g. assoc/disassoc status, erp parameters).
 * This function should not be used if no BSS has been set, unless
 * for association indication. The @changed parameter indicates which
 * of the bss parameters has changed when a call is made. The callback
 * can sleep.
 */
static void smd_op_bss_info_changed(struct ieee80211_hw *hw,
                                    struct ieee80211_vif *vif,
                                    struct ieee80211_bss_conf *info,
                                    u32 changed)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);
   struct smd_vif_priv *sv = vif_to_sv(vif);
   struct ieee80211_sta *sta;
   struct smd_sta_priv *ss;
   bool   changed_ps = false;

   de_info(DE_TR_WIFI, "changed = %#x", changed);

   if(!SV_IS_VALID(sv) || sv->sv_bssconf == NULL) {
      de_info(DE_TR_WIFI, "called with uninitialised vif");
      return;
   }

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
   /* In older kernels, the idle flag was not cleared correctly in AP mode */
   if(vif->type != NL80211_IFTYPE_STATION) {
      info->idle = false;
   }
#endif

   if (changed & BSS_CHANGED_ASSOC) {
      de_info(DE_TR_WIFI, "  assoc = %u, aid = %u, assoc_capability = %x, dtim_period = %u",
            info->assoc,
            info->aid,
            info->assoc_capability,
            info->dtim_period);
      changed &= ~BSS_CHANGED_ASSOC;

      if(info->assoc && (sv->sv_net_type & NRX_NET_ROLE_MASTER) == 0)
         set_beacon_loss_trigger(sv);
      else
         remove_beacon_loss_trigger(sv);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
      sv->sv_bssconf->dtim_period         = info->dtim_period;
#endif
      sv->sv_bssconf->capability_info     = info->assoc_capability;
#ifndef NRX_USE_CHANNEL_CONTEXT
      smd_sv_set_channel(sv,
                         sh_current_channel(sh),
                         sh->sh_hw->conf.channel_type);
#endif

      /* Update association ID in firmware */
      if (test_bit(SMD_FLAG_SV_STARTED, &sv->sv_flags)) {
         rcu_read_lock();
         sta = ieee80211_find_sta(vif, info->bssid);
         if (sta != NULL) {
            ss = sta_to_ss(sta);
            ss->ss_sta_info.aid      = info->assoc ? info->aid : 0;
            ss->ss_sta_info.qos_flag = info->qos;
            set_sta_info_from_sta(sv, sta);
         } else {
            de_warn(DE_TR_WIFI, "Unknown STA %pM", info->bssid);
         }
         rcu_read_unlock();
      }
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   if(changed & BSS_CHANGED_ERP_CTS_PROT) {
      de_info(DE_TR_WIFI, "  use_cts_prot = %u", info->use_cts_prot);
      changed &= ~BSS_CHANGED_ERP_CTS_PROT;
      if(info->use_cts_prot)
         sv->sv_bssconf->protection_flags |= NRX_PROTECTION_FORCE_PROTECTION;
      else
         sv->sv_bssconf->protection_flags &= ~NRX_PROTECTION_FORCE_PROTECTION;
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   if(changed & BSS_CHANGED_ERP_PREAMBLE) {
      de_info(DE_TR_WIFI, "  use_short_preamble = %u", info->use_short_preamble);
      changed &= ~BSS_CHANGED_ERP_PREAMBLE;
      
      /* info->assoc_capability is only set at (re)assoc_rsp, not on rcvd beacons,
       * even if we send them up as "updated" */
      if(info->use_short_preamble) {
         sv->sv_bssconf->protection_flags &= ~NRX_PROTECTION_FORCE_LONG_PREAMBLE;
         sv->sv_bssconf->capability_info |=  WLAN_CAPABILITY_SHORT_PREAMBLE;
      } else {
         sv->sv_bssconf->protection_flags |= NRX_PROTECTION_FORCE_LONG_PREAMBLE;
         sv->sv_bssconf->capability_info &= ~WLAN_CAPABILITY_SHORT_PREAMBLE;
      }
      
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   if(changed & BSS_CHANGED_ERP_SLOT) {
      de_info(DE_TR_WIFI, "  use_short_slot = %u", info->use_short_slot);
      changed &= ~BSS_CHANGED_ERP_SLOT;
      
      /* info->assoc_capability is only set at (re)assoc_rsp, not on rcvd beacons,
       * even if we send them up as "updated" */
      if(info->use_short_slot)
         sv->sv_bssconf->capability_info |=  WLAN_CAPABILITY_SHORT_SLOT_TIME;
      else
         sv->sv_bssconf->capability_info &= ~WLAN_CAPABILITY_SHORT_SLOT_TIME;
      
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   if(changed & BSS_CHANGED_HT) {
      de_info(DE_TR_WIFI, "  ht_operation_mode = %x", info->ht_operation_mode);
      changed &= ~BSS_CHANGED_HT;
   }
   if(changed & BSS_CHANGED_BASIC_RATES) {
      de_info(DE_TR_WIFI, "  basic_rates = %x", info->basic_rates);
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
      changed &= ~BSS_CHANGED_BASIC_RATES;
   }
   if(changed & BSS_CHANGED_BEACON_INT) {
      de_info(DE_TR_WIFI, "  beacon_int = %u", info->beacon_int);
      changed &= ~BSS_CHANGED_BEACON_INT;
      sv->sv_bssconf->beacon_period = info->beacon_int;
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   if(changed & BSS_CHANGED_BSSID) {
      nrx_mac_addr_t empty_bssid = {{0,0,0,0,0,0}};
      de_info(DE_TR_WIFI, "  bssid = %pM", info->bssid);
      changed &= ~BSS_CHANGED_BSSID;
      memcpy(sv->sv_bssconf->bssid.octet, info->bssid, ETH_ALEN);
      /* The net shall only be started if it has a valid bssid */
      if (memcmp(sv->sv_bssconf->bssid.octet,
                 empty_bssid.octet,
                 sizeof(empty_bssid.octet)) == 0)
      {
         set_bit(SMD_FLAG_SV_HIBERNATING, &sv->sv_flags);
      } else {
         clear_bit(SMD_FLAG_SV_HIBERNATING, &sv->sv_flags);
      }
      /* Setting BSS ID affects power save state, since mac80211 seems to consider a net with
       * no valid BSS ID as not active.
       */
      changed_ps = true;

      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   if(changed & BSS_CHANGED_BEACON) {
      de_info(DE_TR_WIFI, "  BSS_CHANGED_BEACON");
      changed &= ~BSS_CHANGED_BEACON;
      smd_update_beacon(sv);
   }
   if(changed & BSS_CHANGED_BEACON_ENABLED) {
      de_info(DE_TR_WIFI, "  enable_beacon = %u", info->enable_beacon);
      changed &= ~BSS_CHANGED_BEACON_ENABLED;
      sv->sv_enable_beacon = info->enable_beacon;
      set_bit(SMD_FLAG_SV_BSSCONF_CHANGED, &sv->sv_flags);
   }
   if(changed & BSS_CHANGED_CQM) {
      de_info(DE_TR_WIFI, "  cqm_rssi_thold = %d, cqm_rssi_hyst = %u",
              info->cqm_rssi_thold, info->cqm_rssi_hyst);
      changed &= ~BSS_CHANGED_CQM;
      rcu_read_lock();
      sta = ieee80211_find_sta(vif, info->bssid);
      ss = sta_to_ss(sta);
      if (sta != NULL) {
         smd_sta_update_rssi_levels(sv, ss,
                                    info->cqm_rssi_thold,
                                    info->cqm_rssi_hyst);
      } else {
         de_warn(DE_TR_WIFI, "Unknown STA %pM", info->bssid);
      }
      rcu_read_unlock();
   }
   if(changed & BSS_CHANGED_IBSS) {
      de_info(DE_TR_WIFI, "  ibss_joined = %u", info->ibss_joined);
      changed &= ~BSS_CHANGED_IBSS;
   }
   if(changed & BSS_CHANGED_ARP_FILTER) {
      bool enable;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
      enable = info->arp_addr_cnt > 0;
#else
      enable = info->arp_filter_enabled;
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0) */
      de_info(DE_TR_WIFI, "  arp_filter_enabled = %u", enable);
      smd_sv_update_arp_filter(sv, info, enable);
      changed &= ~BSS_CHANGED_ARP_FILTER;
   }
   if(changed & BSS_CHANGED_QOS) {
      de_info(DE_TR_WIFI, "  qos = %u", info->qos);
      changed &= ~BSS_CHANGED_QOS;
   }
   if(changed & BSS_CHANGED_IDLE) {
      de_info(DE_TR_WIFI, "  bss idle = %u", info->idle);
      changed &= ~BSS_CHANGED_IDLE;
   }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   if (changed & BSS_CHANGED_SSID) {
      de_info(DE_TR_WIFI, "  ssid_len = %zu, hidden = %u",
              info->ssid_len, info->hidden_ssid);
      changed &= ~BSS_CHANGED_SSID;
   }
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0)
   if (changed & BSS_CHANGED_AP_PROBE_RESP) {
      de_info(DE_TR_WIFI, "  BSS_CHANGED_AP_PROBE_RESP");
      changed &= ~BSS_CHANGED_AP_PROBE_RESP;
   }
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
   if (changed & BSS_CHANGED_PS) {
      de_info(DE_TR_WIFI, "  PS = %u",
              info->ps);
      sv->sv_psconf.power_mgmt_mode = info->ps;
      changed_ps = true;
      changed &= ~BSS_CHANGED_PS;
   }
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
   if (changed & BSS_CHANGED_TXPOWER) {
      de_info(DE_TR_WIFI, "  TX_POWER = %d dBm", info->txpower);
      smd_sv_set_txpower(sv, info->txpower);
      /* XXX nrx701 */
      changed &= ~BSS_CHANGED_TXPOWER;
   }
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
#define BSS_CHANGED_BEACON_INFO BSS_CHANGED_DTIM_PERIOD
#endif
   if (changed & BSS_CHANGED_BEACON_INFO) 
   {
      de_info(DE_TR_WIFI, "  dtim_period = %d", info->dtim_period);
      sv->sv_bssconf->dtim_period         = info->dtim_period;
      changed &= ~BSS_CHANGED_BEACON_INFO;
   }
#endif

   if(changed) {
      de_info(DE_TR_WIFI, "unknown changed mask %x", changed);
   }

   update_bssconf(sv);

   if(info->idle && (sv->sv_net_type & NRX_NET_ROLE_MASTER) == 0) {
      stop_net(sv);
   } else {
#ifdef DONT_START_NETS_WITH_BSSID_ZEROES
      /* In a future near you, it might be possible to activate this function.
       * From kernel 3.4, mac80211 usually doesn't send frames before a valid bssid is set.
       * But after a coredump that case happens.
       */
      nrx_mac_addr_t empty_bssid = {{0,0,0,0,0,0}};
      /* The net shall only be started if it has a valid bssid */
      if (memcmp(sv->sv_bssconf->bssid.octet,
                 empty_bssid.octet,
                 sizeof(empty_bssid.octet)) != 0)
#endif
      {
         if (start_net(sv) || changed_ps) {
            set_power_mgmt(sv, sh);
         }
      }
   }
}

/*
 * set_frag_threshold
 *
 * Configuration of fragmentation threshold. Assign this if the device
 * does fragmentation by itself; if this callback is implemented then
 * the stack will not do fragmentation.
 *
 * The callback can sleep.
 */
static int smd_op_set_frag_threshold(struct ieee80211_hw *hw,
                                 u32 value)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);

   if(value < 256)
      return -EINVAL;

   if(value >= 2346)
      value = 0;

   if(value & 1)
      return -EINVAL;

   sh->sh_frag_threshold = value;

   return 0;
}

static void
set_default_retrans(nrx_default_retrans_t *retrans, int idx, int rate, int tries, s8 rssi)
{
   de_info(DE_TR_WIFI, "[%d] rate=%#x tries=%d", idx, rate, tries);
   if (idx < NRX_BSS_DEFAULT_RETRANS_TABLE_NUM_ENTRIES) {
      retrans->tables.retrans_table[idx].attrib.rate = rate;
      retrans->tables.retrans_table[idx].attrib.tries = tries;
      retrans->tables.retrans_table[idx].attrib.flags = 0;
      retrans->tables.retrans_protect_table[idx].attrib.rate = rate;
      retrans->tables.retrans_protect_table[idx].attrib.tries = tries;
      retrans->tables.retrans_protect_table[idx].attrib.flags = 0;
      if (idx < NRX_BSS_DEFAULT_RETRANS_TABLE_NUM_ENTRIES-1) {
         retrans->thresholds.retrans_rssi_thresholds[idx] = rssi;
      }
   }
}

static void print_sta_info(struct ieee80211_sta *sta)
{
   de_info(DE_TR_WIFI, "addr:       %pM", sta->addr);
   de_info(DE_TR_WIFI, "rates 2GHz: %#x", sta->supp_rates[IEEE80211_BAND_2GHZ]);
   de_info(DE_TR_WIFI, "rates 5GHz: %#x", sta->supp_rates[IEEE80211_BAND_5GHZ]);
   de_info(DE_TR_WIFI, "aid:        %u", sta->aid);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   de_info(DE_TR_WIFI, "wme:        %u", sta->wme);
   de_info(DE_TR_WIFI, "uapsd_qs:   %u", sta->uapsd_queues);
   de_info(DE_TR_WIFI, "max_sp:     %u", sta->max_sp);
#endif

   if (sta->ht_cap.ht_supported) {
      de_info(DE_TR_WIFI, "ht_cap = %x, apmdu_factor = %u, ampdu_density = %u",
              sta->ht_cap.cap,
              sta->ht_cap.ampdu_factor,
              sta->ht_cap.ampdu_density);
      de_info(DE_TR_WIFI, "rx_mask = %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
              sta->ht_cap.mcs.rx_mask[9],
              sta->ht_cap.mcs.rx_mask[8],
              sta->ht_cap.mcs.rx_mask[7],
              sta->ht_cap.mcs.rx_mask[6],
              sta->ht_cap.mcs.rx_mask[5],
              sta->ht_cap.mcs.rx_mask[4],
              sta->ht_cap.mcs.rx_mask[3],
              sta->ht_cap.mcs.rx_mask[2],
              sta->ht_cap.mcs.rx_mask[1],
              sta->ht_cap.mcs.rx_mask[0]);
      de_info(DE_TR_WIFI, "rx_highest = %u", sta->ht_cap.mcs.rx_highest);
      de_info(DE_TR_WIFI, "tx_params = %x", sta->ht_cap.mcs.tx_params);
   }
}

static const nrx_aci_t tid_to_aci_map[16] = {
   NRX_ACI_AC_BE, NRX_ACI_AC_BK,
   NRX_ACI_AC_BK, NRX_ACI_AC_BE,
   NRX_ACI_AC_VI, NRX_ACI_AC_VI,
   NRX_ACI_AC_VO, NRX_ACI_AC_VO,
   NRX_ACI_AC_BE, NRX_ACI_AC_BE,
   NRX_ACI_AC_BE, NRX_ACI_AC_BE,
   NRX_ACI_AC_BE, NRX_ACI_AC_BE,
   NRX_ACI_AC_BE, NRX_ACI_AC_BE,
};

static void smd_ss_setup(struct ieee80211_vif *vif,
                         struct ieee80211_sta *sta)
{
   struct smd_vif_priv *sv = vif_to_sv(vif);
   struct smd_sta_priv *ss = sta_to_ss(sta);

   memset(ss, 0, sizeof *ss);
   kref_init(&ss->ss_refcnt);
   init_waitqueue_head(&ss->ss_waitqueue);
   ss->ss_sv = sv;
   ss->ss_sta_info.aid = sta->aid;
   memcpy(ss->ss_sta_info.peer_sta.octet, sta->addr, ETH_ALEN);
   ss->ss_sta_info.rssi_trigger_rising = NRX_MAC_INVALID_RSSI_SNR_VALUE;
   ss->ss_sta_info.rssi_trigger_falling = NRX_MAC_INVALID_RSSI_SNR_VALUE;
   BUILD_BUG_ON(sizeof(ss->ss_sta_info.tid_to_ac_map) != sizeof(tid_to_aci_map));
   memcpy(ss->ss_sta_info.tid_to_ac_map,
          tid_to_aci_map,
          sizeof(ss->ss_sta_info.tid_to_ac_map));
}

static void
smd_ss_update_rates(struct ieee80211_hw *hw,
                    struct ieee80211_sta *sta)
{
   struct smd_sta_priv *ss = sta_to_ss(sta);
   int rate, idx = 0;
   struct ieee80211_supported_band *band = hw->wiphy->bands[IEEE80211_BAND_2GHZ];

   for (rate=band->n_bitrates-1; rate>=0; rate--) {
      if (rate_supported(sta, IEEE80211_BAND_2GHZ, rate)) {
         uint16_t hw_rate = band->bitrates[rate].hw_value;
         if (idx == 0) {
            set_default_retrans(&ss->ss_sta_info.oretrans, idx, hw_rate, 2, -73);
            idx++;
         } else if (idx == 1 && band->bitrates[rate].bitrate <= 240) {
            set_default_retrans(&ss->ss_sta_info.oretrans, idx, hw_rate, 2, -82);
            idx++;
         } else if (idx == 2) {
            set_default_retrans(&ss->ss_sta_info.oretrans, idx, hw_rate, 3, NRX_MAC_RSSI_UNKNOWN);
         }
      }
   }
   ss->ss_sta_info.bretrans = ss->ss_sta_info.oretrans;
}

/* Configuration of RTS threshold
 *	The callback can sleep.
 */
static int
smd_op_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
   return 0;
}

/* Notifies low level driver about addition of an associated station,
 * AP, IBSS/WDS/mesh peer etc. This callback can sleep.
 */
static int smd_op_sta_add(struct ieee80211_hw *hw,
                          struct ieee80211_vif *vif,
                          struct ieee80211_sta *sta)
{
   struct smd_vif_priv *sv = vif_to_sv(vif);

   if(!SV_IS_VALID(sv)) {
      de_info(DE_TR_WIFI, "called with uninitialised vif");
      return -EINVAL;
   }

   print_sta_info(sta);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
   smd_ss_setup(vif, sta);
#endif
   smd_ss_update_rates(hw, sta);

   set_sta_info_from_sta(sv, sta);

   return 0;
}

/*
 * smd_op_sta_remove
 *
 * Notifies low level driver about removal of an associated station, AP,
 * IBSS/WDS/mesh peer etc. This callback can sleep.
 */
static int
smd_op_sta_remove(struct ieee80211_hw *hw,
                  struct ieee80211_vif *vif,
                  struct ieee80211_sta *sta)
{
   struct smd_sta_priv       *ss = sta_to_ss(sta);
   struct smd_vif_priv       *sv = vif_to_sv(vif);
   struct smd_hw_priv        *sh = hw_to_sh(hw);
   struct smd_transaction    *stx;
   nrx_mlme_delete_sta_req_t *req;
   nrx_mlme_delete_sta_cfm_t *cfm;
   int retval;

   de_info(DE_TR_WIFI, "addr=%pM", sta->addr);

   if(!SV_IS_VALID(sv)) {
      de_info(DE_TR_WIFI, "called with uninitialised vif");
      return -EINVAL;
   }
   set_bit(SMD_FLAG_SS_REMOVING, &ss->ss_flags);
   SS_PUT(ss);
   if(wait_event_interruptible_timeout(ss->ss_waitqueue,
                                       atomic_read(&ss->ss_refcnt.refcount) == 0,
                                       HZ) == 0) {
      de_error(DE_TR_WIFI, "WAIT FOR STA REFERENCE TIMED OUT, THIS MAY BE A DRIVER BUG");
      if (!test_and_set_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
         nanoc_req_coredump((struct nanoc_context*)hw);
      }
      return -EIO;
   }

   req = smd_alloc_req_vif(sv, sizeof *req,
                           HIC_MESSAGE_TYPE_MLME, NRX_MLME_DELETE_STA_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   memcpy(req->peer_sta.octet, sta->addr, ETH_ALEN);
   memset(&req->reserved, 0, sizeof(req->reserved));

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->result != NRX_SUCCESS) {
         de_warn(DE_TR_WIFI, "FAILURE: mlme_delete_sta: %d", cfm->result);
         retval = -EIO;
      }
   }
   smd_txn_decref(stx);

   return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
/* Notifies low level driver about state transition of a station
 * (which can be the AP, a client, IBSS/WDS/mesh peer etc.)  This
 * callback is mutually exclusive with @sta_add/@sta_remove.  It must
 * not fail for down transitions but may fail for transitions up the
 * list of states.
 *
 * The callback can sleep.
 */
static int smd_op_sta_state(struct ieee80211_hw *hw,
                            struct ieee80211_vif *vif,
                            struct ieee80211_sta *sta,
                            enum ieee80211_sta_state old_state,
                            enum ieee80211_sta_state new_state)
{
   const char *s[] = {
      "IEEE80211_STA_NOTEXIST",
      "IEEE80211_STA_NONE",
      "IEEE80211_STA_AUTH",
      "IEEE80211_STA_ASSOC",
      "IEEE80211_STA_AUTHORIZED"
   };

   de_debug(DE_TR_WIFI, "STA %s -> %s", s[old_state], s[new_state]);

   if(old_state == IEEE80211_STA_NOTEXIST
      && new_state == IEEE80211_STA_NONE) {
      smd_ss_setup(vif, sta);
      return 0;
   }

   if(old_state == IEEE80211_STA_AUTH
      && new_state == IEEE80211_STA_ASSOC)
      return smd_op_sta_add(hw, vif, sta);

   if(old_state == IEEE80211_STA_ASSOC
      && new_state == IEEE80211_STA_AUTH)
      return smd_op_sta_remove(hw, vif, sta);

   return 0;
}
#endif

/*
 * smd_op_hw_scan
 *
 * Ask the hardware to service the scan request, no need to start the scan
 * state machine in stack. The scan must honour the channel configuration done
 * by the regulatory agent in the wiphy's registered bands. The hardware (or
 * the driver) needs to make sure that power save is disabled.  The @req
 * ie/ie_len members are rewritten by mac80211 to contain the entire IEs after
 * the SSID, so that drivers need not look at these at all but just send them
 * after the SSID -- mac80211 includes the (extended) supported rates and HT
 * information (where applicable).  When the scan finishes,
 * ieee80211_scan_completed() must be called; note that it also must be called
 * when the scan cannot finish due to any error unless this callback returned a
 * negative error code.  The callback can sleep.
 */
static int
smd_op_hw_scan(struct ieee80211_hw *hw,
               struct ieee80211_vif *vif,
               struct ieee80211_scan_request *req)
{
   struct cfg80211_scan_request *sreq = &req->req;
   struct smd_hw_priv *sh = hw_to_sh(hw);
   int status;

   de_info(DE_TR_WIFI, "n_ssids=%u ie_len=%zu", sreq->n_ssids, sreq->ie_len);

   if(resource_request_scan(sh, SMD_FLAG_SCAN_SINGLE) < 0) {
      de_info(DE_TR_WIFI, "Scan BUSY");
      return -EAGAIN;
   }

   status = smd_scan_request(sh, sreq);

   if(status != 0) {
      de_error(DE_TR_WIFI, "failed to start scan job: %d", status);
      resource_release_scan(sh, SMD_FLAG_SCAN_SINGLE);
   }

   return status;
}

static int smd_update_queue_param(struct smd_hw_priv *sh)
{
   nrx_mlme_config_tx_req_t *req;
   nrx_mlme_config_tx_cfm_t *cfm;
   struct smd_transaction *stx;
   int retval;

   req = smd_alloc_req(sh, sizeof(*req),
                       HIC_MESSAGE_TYPE_MLME,
                       NRX_MLME_CONFIG_TX_REQ,
                       &stx);
   if(req == NULL)
      return -ENOMEM;

   *req = sh->sh_tx_conf;

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->status != NRX_SUCCESS) {
         de_warn(DE_TR_WIFI, "FAILURE: mlme_config_tx_req: %d", cfm->status);
         retval = -EIO;
      }
   }
   smd_txn_decref(stx);

   return retval;
}

#define IS_POWER_OF_2(N) ((N) != 0 && (((N) & ((N) - 1)) == 0))

static uint8_t cw2ecw(uint16_t cw)
{
   return fls(cw);
}

/*
 * smd_op_conf_tx
 *
 * Configure TX queue parameters (EDCF (aifs, cw_min, cw_max), bursting) for a
 * hardware TX queue.  Returns a negative error code on failure.  The callback
 * can sleep.
 */
static int
smd_op_conf_tx(struct ieee80211_hw *hw,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
                struct ieee80211_vif *vif,
#endif
               u16 queue,
               const struct ieee80211_tx_queue_params *params)
{
   struct smd_hw_priv *sh = hw_to_sh(hw);
   nrx_mlme_tx_config_t *q;
   uint8_t uapsd_mask;
   uint8_t ecw_min, ecw_max;
   bool changed = false;

   de_info(DE_TR_WIFI, "Q%u, txop = %u, cw_min = %u, cw_max = %u, "
           "aifs = %u, uapsd = %u",
           queue, params->txop, params->cw_min, params->cw_max,
           params->aifs, params->uapsd);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
   if(params->acm)
      de_info(DE_TR_WIFI, "Q%u: acm flag ignored", queue);
#endif

   if(params->cw_min > params->cw_max
      || !IS_POWER_OF_2(params->cw_min + 1)
      || !IS_POWER_OF_2(params->cw_max + 1))
      return -EINVAL;

   switch(queue) {
      case IEEE80211_AC_VO:
         q = &sh->sh_tx_conf.queue[NRX_EDCA_VOICE_QUEUE];
         uapsd_mask = BIT(NRX_ACI_AC_VO);
         break;
      case IEEE80211_AC_VI:
         q = &sh->sh_tx_conf.queue[NRX_EDCA_VIDEO_QUEUE];
         uapsd_mask = BIT(NRX_ACI_AC_VI);
         break;
      case IEEE80211_AC_BE:
         q = &sh->sh_tx_conf.queue[NRX_EDCA_BEST_EFFORT_QUEUE];
         uapsd_mask = BIT(NRX_ACI_AC_BE);
         break;
      case IEEE80211_AC_BK:
         q = &sh->sh_tx_conf.queue[NRX_EDCA_BACKGROUND_QUEUE];
         uapsd_mask = BIT(NRX_ACI_AC_BK);
         break;
      default:
         return -EINVAL;
   }

   /* for some obscure reason we're passed the computed CW and not
    * just the exponent, convert back to ECW */
   ecw_min = cw2ecw(params->cw_min);
   ecw_max = cw2ecw(params->cw_max);

   if(q->txop_limit != params->txop) {
      q->txop_limit = params->txop;
      changed = true;
   }
   if(q->ecw_min != ecw_min) {
      q->ecw_min = ecw_min;
      changed = true;
   }
   if(q->ecw_max != ecw_max) {
      q->ecw_max = ecw_max;
      changed = true;
   }
   if(q->aifsn != params->aifs) {
      q->aifsn = params->aifs;
      changed = true;
   }
   {
      nrx_mlme_power_mgmt_req_t * psconf;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
      struct smd_vif_priv *sv = vif_to_sv(vif);
      psconf = &sv->sv_psconf;
#else
      psconf = &sh->sh_psconf;
#endif
      if (params->uapsd)
         psconf->uapsd |= uapsd_mask;
      else
         psconf->uapsd &= ~uapsd_mask;

      if (psconf->uapsd) {
         psconf->periodic_service_interval = 1;
         psconf->fixed_service_interval = cfg_service_interval;
      } else {
         psconf->periodic_service_interval = 0;
         psconf->fixed_service_interval = 0;
      }
   }

   if(changed)
      return smd_update_queue_param(sh);

   return 0;
}

static int
smd_op_get_stats(struct ieee80211_hw *hw, struct ieee80211_low_level_stats *stats)
{
   struct smd_hw_priv          *sh = hw_to_sh(hw);
   struct smd_transaction      *stx;
   nrx_mlme_get_counters_req_t *req;
   int ret;

   de_info(DE_TR_WIFI, "hw=%p", hw);

   req = smd_alloc_req(sh, sizeof *req, HIC_MESSAGE_TYPE_MLME, NRX_MLME_GET_COUNTERS_REQ, &stx);
   if (req == NULL ) {
      ret = -ENOMEM;
   } else {
      req->reserved = 0;
      /* Send transaction and wait for completion */
      if ((ret = smd_wait_transaction(stx)) == 0) {
         /* Update counters */
         nrx_mlme_get_counters_cfm_t *cfm = de_pdu_data(stx->st_rxskb);
         stats->dot11ACKFailureCount = cfm->dot11_counters.dot11ACKFailureCount;
         stats->dot11RTSFailureCount = cfm->dot11_counters.dot11RTSFailureCount;
         stats->dot11FCSErrorCount   = cfm->dot11_counters.dot11FCSErrorCount;
         stats->dot11RTSSuccessCount = cfm->dot11_counters.dot11RTSSuccessCount;

         de_info(DE_TR_WIFI, "hic_uplink_data_discarded     = %u", cfm->mac_counters.hic_uplink_data_discarded);
         de_info(DE_TR_WIFI, "hic_uplink_low_prio_discarded = %u", cfm->mac_counters.hic_uplink_low_prio_discarded);
         de_info(DE_TR_WIFI, "macul_rx                      = %u", cfm->mac_counters.macul_rx);
         de_info(DE_TR_WIFI, "macul_rx_discarded            = %u", cfm->mac_counters.macul_rx_discarded);
         de_info(DE_TR_WIFI, "macul_tx                      = %u", cfm->mac_counters.macul_tx);
         de_info(DE_TR_WIFI, "macul_tx_enqueued             = %u", cfm->mac_counters.macul_tx_enqueued);
         de_info(DE_TR_WIFI, "missed_beacons                = %u", cfm->mac_counters.missed_beacons);
      }
      smd_txn_decref(stx);
   }

   return ret;
}

static int
smd_add_baa(struct smd_sta_priv *ss, uint16_t tid, uint16_t ssn)
{
   nrx_mlme_add_baa_req_t *req;
   nrx_mlme_add_baa_cfm_t *cfm;
   struct smd_transaction *stx;
   int retval = 0;

   if(tid >= 0x10) {
      de_warn(DE_TR_WIFI, "got bad tid %x", tid);
      return -EINVAL;
   }

   if(ss->ss_rx_baa_mask & BIT(tid)) {
      de_warn(DE_TR_WIFI, "tried to re-add baa for tid %x", tid);
      return -EINVAL;
   }
   ss->ss_rx_baa_mask |= BIT(tid);

   req = smd_alloc_req_vif(ss->ss_sv,
                           sizeof(*req),
                           HIC_MESSAGE_TYPE_MLME, NRX_MLME_ADD_BAA_REQ,
                           &stx);
   if(req == NULL) {
      de_warn(DE_TR_WIFI, "failed to allocate memory");
      return -ENOMEM;
   }

   memcpy(req->peer_sta.octet, ss_to_sta(ss)->addr, ETH_ALEN);
   req->tid = tid;
   req->win_start = ssn;
   memset(req->reserved, 0, sizeof(req->reserved));

   retval = smd_wait_transaction(stx);
   if(retval == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->result != 0) {
         retval = -EINVAL;
      }
   }
   smd_txn_decref(stx);
   return retval;
}

static int
smd_del_baa(struct smd_sta_priv *ss, uint16_t tid)
{
   nrx_mlme_del_baa_req_t *req;
   nrx_mlme_del_baa_cfm_t *cfm;
   struct smd_transaction *stx;
   int retval = 0;

   if((ss->ss_rx_baa_mask & BIT(tid)) == 0) {
      de_warn(DE_TR_WIFI, "tried to remove unused tid %x", tid);
      return -EINVAL;
   }
   ss->ss_rx_baa_mask &= ~BIT(tid);

   req = smd_alloc_req_vif(ss->ss_sv,
                           sizeof(*req),
                           HIC_MESSAGE_TYPE_MLME, NRX_MLME_DEL_BAA_REQ,
                           &stx);
   if(req == NULL) {
      de_warn(DE_TR_WIFI, "failed to allocate memory");
      return -ENOMEM;
   }

   memcpy(req->peer_sta.octet, ss_to_sta(ss)->addr, ETH_ALEN);
   req->tid = tid;
   req->reserved = 0;

   if(smd_wait_transaction(stx) == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      if(cfm->result != 0) {
         retval = -EINVAL;
      }
   } else {
      retval = -EIO;
   }
   smd_txn_decref(stx);
   return retval;
}

/*
 * @ampdu_action: Perform a certain A-MPDU action
 *      The RA/TID combination determines the destination and TID we want
 *      the ampdu action to be performed for. The action is defined through
 *      ieee80211_ampdu_mlme_action. Starting sequence number (@ssn)
 *      is the first frame we expect to perform the action on. Notice
 *      that TX/RX_STOP can pass NULL for this parameter.
 *      The @buf_size parameter is only valid when the action is set to
 *      %IEEE80211_AMPDU_TX_OPERATIONAL and indicates the peer's reorder
 *      buffer size (number of subframes) for this session -- the driver
 *      may neither send aggregates containing more subframes than this
 *      nor send aggregates in a way that lost frames would exceed the
 *      buffer size. If just limiting the aggregate size, this would be
 *      possible with a buf_size of 8:
 *      - TX: 1.....7
 *      - RX:  2....7 (lost frame #1)
 *      - TX:        8..1...
 *      which is invalid since #1 was now re-transmitted well past the
 *      buffer size of 8. Correct ways to retransmit #1 would be:
 *      - TX:       1 or 18 or 81
 *      Even "189" would be wrong since 1 could be lost again.
 *
 *      Returns a negative error code on failure.
 *      The callback can sleep.
*/
static int
smd_op_ampdu_action(struct ieee80211_hw *hw,
                    struct ieee80211_vif *vif,
                    enum ieee80211_ampdu_mlme_action action,
                    struct ieee80211_sta *sta,
                    u16 tid,
                    u16 *ssn,
                    u8 buf_size)
{
   struct smd_sta_priv *ss = sta_to_ss(sta);

   switch (action) {
      case IEEE80211_AMPDU_RX_START:
         de_info(DE_TR_WIFI, "IEEE80211_AMPDU_RX_START, tid = %u", tid);
         return smd_add_baa(ss, tid, *ssn);

      case IEEE80211_AMPDU_RX_STOP:
         de_info(DE_TR_WIFI, "IEEE80211_AMPDU_RX_STOP, tid = %u", tid);
         ss->ss_tid[tid].st_delba_time = jiffies + HZ / 10;
         return smd_del_baa(ss, tid);

      case IEEE80211_AMPDU_TX_START:
         de_info(DE_TR_WIFI, "IEEE80211_AMPDU_TX_START, tid = %u", tid);
         if(disable_tx_ampdu)
            return -EOPNOTSUPP;
         if(tid >= ARRAY_SIZE(ss->ss_tid)) {
            de_info(DE_TR_WIFI, "tid out of range");
            return -EINVAL;
         }
         if (test_bit(SMD_STA_TID_FLAG_AMPDU_ENABLED, &ss->ss_tid[tid].st_flags)) {
            de_info(DE_TR_WIFI, "already enabled");
            return -EINVAL;
         }
         if (!test_bit(SMD_STA_TID_FLAG_GOT_FRAME, &ss->ss_tid[tid].st_flags)) {
            /* No frame has yet been sent. It is safe to immediately send an add_block_ack_req */
            de_info(DE_TR_WIFI, "First frame");
            ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
         } else {
            /* We must make sure that we have sent the previous sequence number
             * before we can proceed and tell mac80211 that it is OK to send the add_block_ack_req.
             * mac80211 have stopped the queue on that TID but there might be frames in fw and in other
             * queues that we must wait for before the add_block_ack_req can be sent. The add_baa_req
             * is sent as soon as we return from this function. */
            uint16_t previous_seq = ((*ssn-1)&0xfff); /* Sequence number is 12 bits */
            unsigned timeout = 200;

            /* If we haven't got all confirms within 200ms we'll send the add_baa_req anyway.
             * Some frames might be discarded at the other end due to this. */
            while (previous_seq != ss->ss_tid[tid].st_last_seq_no
                   && timeout-- != 0)
            {
               msleep(1);
            }
            ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
         }
         break;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
      case IEEE80211_AMPDU_TX_STOP_CONT:
      case IEEE80211_AMPDU_TX_STOP_FLUSH:
      case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
#else
      case IEEE80211_AMPDU_TX_STOP:
#endif
         de_info(DE_TR_WIFI, "IEEE80211_AMPDU_TX_STOP, tid = %u", tid);
         if(tid >= ARRAY_SIZE(ss->ss_tid)) {
            de_info(DE_TR_WIFI, "tid out of range");
            return -EINVAL;
         }
         clear_bit(SMD_STA_TID_FLAG_AMPDU_ENABLED, &ss->ss_tid[tid].st_flags);
         ss->ss_tid[tid].st_ampdu_buf_size = 0;
         ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
         break;

      case IEEE80211_AMPDU_TX_OPERATIONAL:
         de_info(DE_TR_WIFI, "IEEE80211_AMPDU_TX_OPERATIONAL, tid = %u, buf_size = %u", tid, buf_size);
         if(tid >= ARRAY_SIZE(ss->ss_tid)) {
            de_info(DE_TR_WIFI, "tid out of range");
            return -EINVAL;
         }
         set_bit(SMD_STA_TID_FLAG_AMPDU_ENABLED, &ss->ss_tid[tid].st_flags);
         ss->ss_tid[tid].st_ampdu_buf_size = buf_size;
         break;
      default:
         de_info(DE_TR_WIFI, "unknown action = %u, sta = %pMF,"
                 " tid = %u, ssn = %u, buf_size = %u",
                 action,
                 sta->addr,
                 tid,
                 ssn ? *ssn : 0xdeadbeef,
                 buf_size);
         return -EOPNOTSUPP;
   }

   return 0;
}

static struct ieee80211_channel smd_2ghz_channels[] = {
#define CH2G(N, F) {                            \
      .band = IEEE80211_BAND_2GHZ,              \
      .center_freq = (F),                       \
      .hw_value = (N),                          \
      .flags = 0,                               \
      .max_antenna_gain = 0,                    \
      .max_power = 20,                          \
   }
   CH2G(1, 2412),
   CH2G(2, 2417),
   CH2G(3, 2422),
   CH2G(4, 2427),
   CH2G(5, 2432),
   CH2G(6, 2437),
   CH2G(7, 2442),
   CH2G(8, 2447),
   CH2G(9, 2452),
   CH2G(10, 2457),
   CH2G(11, 2462),
   CH2G(12, 2467),
   CH2G(13, 2472),
   CH2G(14, 2484)
};

static struct ieee80211_channel smd_5ghz_channels[] = {
#define CH5G(N, F) {                            \
      .band = IEEE80211_BAND_5GHZ,              \
      .center_freq = (F),                       \
      .hw_value = (N),                          \
      .flags = 0,                               \
      .max_antenna_gain = 0,                    \
      .max_power = 20,                          \
   }
   CH5G(15, 5035),
   CH5G(16, 5040),
   CH5G(17, 5045),
   CH5G(18, 5055),
   CH5G(19, 5060),
   CH5G(20, 5080),
   CH5G(21, 5170),
   CH5G(22, 5180),
   CH5G(23, 5190),
   CH5G(24, 5200),
   CH5G(25, 5210),
   CH5G(26, 5220),
   CH5G(27, 5230),
   CH5G(28, 5240),
   CH5G(29, 5260),
   CH5G(30, 5280),
   CH5G(31, 5300),
   CH5G(32, 5320),
   CH5G(33, 5500),
   CH5G(34, 5520),
   CH5G(35, 5540),
   CH5G(36, 5560),
   CH5G(37, 5580),
   CH5G(38, 5600),
   CH5G(39, 5620),
   CH5G(40, 5640),
   CH5G(41, 5660),
   CH5G(42, 5680),
   CH5G(43, 5700),
   CH5G(44, 5745),
   CH5G(45, 5765),
   CH5G(46, 5785),
   CH5G(47, 5805),
   CH5G(48, 5825)
};

static int
smd_op_get_survey(struct ieee80211_hw *hw, int idx, struct survey_info *survey)
{
   de_info(DE_TR_WIFI, "Channel survey requested %d", idx);
   if (idx < ARRAY_SIZE(smd_2ghz_channels)) {
      memset(survey, 0, sizeof(*survey));
      survey->channel = &smd_2ghz_channels[idx];
      return 0;
   } else {
      return -ENOENT;
   }
}

/*
 * @release_buffered_frames: Release buffered frames according to the given
 *      parameters. In the case where the driver buffers some frames for
 *      sleeping stations mac80211 will use this callback to tell the driver
 *      to release some frames, either for PS-poll or uAPSD.
 *      Note that if the @more_data paramter is %false the driver must check
 *      if there are more frames on the given TIDs, and if there are more than
 *      the frames being released then it must still set the more-data bit in
 *      the frame. If the @more_data parameter is %true, then of course the
 *      more-data bit must always be set.
 *      The @tids parameter tells the driver which TIDs to release frames
 *      from, for PS-poll it will always have only a single bit set.
 *      In the case this is used for a PS-poll initiated release, the
 *      @num_frames parameter will always be 1 so code can be shared. In
 *      this case the driver must also set %IEEE80211_TX_STATUS_EOSP flag
 *      on the TX status (and must report TX status) so that the PS-poll
 *      period is properly ended. This is used to avoid sending multiple
 *      responses for a retried PS-poll frame.
 *      In the case this is used for uAPSD, the @num_frames parameter may be
 *      bigger than one, but the driver may send fewer frames (it must send
 *      at least one, however). In this case it is also responsible for
 *      setting the EOSP flag in the QoS header of the frames. Also, when the
 *      service period ends, the driver must set %IEEE80211_TX_STATUS_EOSP
 *      on the last frame in the SP. Alternatively, it may call the function
 *      ieee80211_sta_eosp_irqsafe() to inform mac80211 of the end of the SP.
 *      This callback must be atomic.
 */
void
smd_op_release_buffered_frames(struct ieee80211_hw *hw,
                               struct ieee80211_sta *sta,
                               u16 tids, int num_frames,
                               enum ieee80211_frame_release_type reason,
                               bool more_data)
{
   de_info(DE_TR_WIFI, "tids %X, num_frames %d, reason %d", tids, num_frames, reason);
}

/*
 * @allow_buffered_frames: Prepare device to allow the given number of frames
 *      to go out to the given station. The frames will be sent by mac80211
 *      via the usual TX path after this call. The TX information for frames
 *      released will also have the %IEEE80211_TX_CTL_NO_PS_BUFFER flag set
 *      and the last one will also have %IEEE80211_TX_STATUS_EOSP set. In case
 *      frames from multiple TIDs are released and the driver might reorder
 *      them between the TIDs, it must set the %IEEE80211_TX_STATUS_EOSP flag
 *      on the last frame and clear it on all others and also handle the EOSP
 *      bit in the QoS header correctly. Alternatively, it can also call the
 *      ieee80211_sta_eosp_irqsafe() function.
 *      The @tids parameter is a bitmap and tells the driver which TIDs the
 *      frames will be on; it will at most have two bits set.
 *      This callback must be atomic.
 */
void
smd_op_allow_buffered_frames(struct ieee80211_hw *hw,
                             struct ieee80211_sta *sta,
                             u16 tids, int num_frames,
                             enum ieee80211_frame_release_type reason,
                             bool more_data)
{
   de_info(DE_TR_WIFI, "tids %X, num_frames %d, reason %d", tids, num_frames, reason);
}


#ifdef NRX_USE_CHANNEL_CONTEXT

#define TRACE_CHANNEL_CONTEXT(CTX)                                      \
   de_info(DE_TR_WIFI, "chan = %u/%u, width = %u, f1 = %u, f2 = %u,"    \
           " chains=%uS/%uD",                                           \
           (CTX)->def.chan->hw_value,                                   \
           (CTX)->def.chan->center_freq,                                \
           (CTX)->def.width,                                            \
           (CTX)->def.center_freq1,                                     \
           (CTX)->def.center_freq2,                                     \
           (CTX)->rx_chains_static,                                     \
           (CTX)->rx_chains_dynamic)


/* Notifies device driver about new channel context creation. */
static int smd_op_add_chanctx(struct ieee80211_hw *hw,
                              struct ieee80211_chanctx_conf *ctx)
{
   TRACE_CHANNEL_CONTEXT(ctx);

   return 0;
}

/* Notifies device driver about channel context destruction. */
static void smd_op_remove_chanctx(struct ieee80211_hw *hw,
                                  struct ieee80211_chanctx_conf *ctx)
{
   TRACE_CHANNEL_CONTEXT(ctx);
}

/* Notifies device driver about channel context changes that may
 * happen when combining different virtual interfaces on the same
 * channel context with different settings
 */
static void smd_op_change_chanctx(struct ieee80211_hw *hw,
                                  struct ieee80211_chanctx_conf *ctx,
                                  u32 changed)
{
   TRACE_CHANNEL_CONTEXT(ctx);

   if(changed & IEEE80211_CHANCTX_CHANGE_WIDTH) {
      de_info(DE_TR_WIFI, "  IEEE80211_CHANCTX_CHANGE_WIDTH");
      /* XXX are we supposed to update all vifs assigned to this
       * context here? */
      changed &= ~IEEE80211_CHANCTX_CHANGE_WIDTH;
   }
   if(changed & IEEE80211_CHANCTX_CHANGE_RX_CHAINS) {
      de_info(DE_TR_WIFI, "  IEEE80211_CHANCTX_CHANGE_RX_CHAINS");
      changed &= ~IEEE80211_CHANCTX_CHANGE_RX_CHAINS;
   }
   if(changed != 0) {
      de_info(DE_TR_WIFI, "  %x", changed);
   }
}

/* Notifies device driver about channel context being bound to
 * vif. Possible use is for hw queue remapping.
 */
static int smd_op_assign_vif_chanctx(struct ieee80211_hw *hw,
                                     struct ieee80211_vif *vif,
                                     struct ieee80211_chanctx_conf *ctx)
{
   struct smd_vif_priv *sv = vif_to_sv(vif);

   de_info(DE_TR_WIFI, "vif = %p", vif);
   TRACE_CHANNEL_CONTEXT(ctx);

   if(vif->type == NL80211_IFTYPE_MONITOR) {
      smd_monitor_update_chan_def(hw_to_sh(hw), &ctx->def);
      return 0;
   }
   smd_sv_set_channel_def(sv, &ctx->def);

   update_bssconf(sv);

   return 0;
}


/* Notifies device driver about channel context being unbound from
 * vif.
 */
static void smd_op_unassign_vif_chanctx(struct ieee80211_hw *hw,
                                        struct ieee80211_vif *vif,
                                        struct ieee80211_chanctx_conf *ctx)
{
   de_info(DE_TR_WIFI, "vif = %p", vif);
   TRACE_CHANNEL_CONTEXT(ctx);

   if(vif->type == NL80211_IFTYPE_MONITOR) {
      struct smd_hw_priv *sh = hw_to_sh(hw);
      sh->sh_monitor_channel.props.freq = 0;
      smd_monitor_stop_scanjob(sh);
   }
}
#endif /* NRX_USE_CHANNEL_CONTEXT */

static const struct ieee80211_ops smd_ieee80211_ops = {
   .tx               = smd_op_tx,               /* MANDATORY */
   .start            = smd_op_start,            /* MANDATORY */
   .stop             = smd_op_stop,             /* MANDATORY */
   .config           = smd_op_config,           /* MANDATORY */
   .add_interface    = smd_op_add_interface,    /* MANDATORY */
   .remove_interface = smd_op_remove_interface, /* MANDATORY */
   .set_key          = smd_op_set_key,
   .prepare_multicast= smd_op_prepare_multicast,
   .configure_filter = smd_op_configure_filter, /* MANDATORY */
   .bss_info_changed = smd_op_bss_info_changed,
   .get_stats        = smd_op_get_stats,
   .set_frag_threshold = smd_op_set_frag_threshold,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
   .sta_state        = smd_op_sta_state,
#else
   .sta_add          = smd_op_sta_add,
   .sta_remove       = smd_op_sta_remove,
#endif
   .set_rts_threshold = smd_op_set_rts_threshold,
   .hw_scan          = smd_op_hw_scan,
   .conf_tx          = smd_op_conf_tx,
   .ampdu_action     = smd_op_ampdu_action,
   .get_survey       = smd_op_get_survey,
   .flush            = smd_op_flush,
#ifdef CONFIG_NL80211_TESTMODE
   .testmode_cmd     = smd_op_testmode_cmd,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
   .testmode_dump    = smd_op_testmode_dump,
#endif
#endif
   .remain_on_channel= smd_op_remain_on_channel,
   .cancel_remain_on_channel = smd_op_cancel_remain_on_channel,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   .allow_buffered_frames    = smd_op_allow_buffered_frames,
   .release_buffered_frames  = smd_op_release_buffered_frames,
#endif
#ifdef NRX_USE_CHANNEL_CONTEXT
   .add_chanctx          = smd_op_add_chanctx,
   .remove_chanctx       = smd_op_remove_chanctx,
   .change_chanctx       = smd_op_change_chanctx,
   .assign_vif_chanctx   = smd_op_assign_vif_chanctx,
   .unassign_vif_chanctx = smd_op_unassign_vif_chanctx,
#endif
};

static struct ieee80211_rate smd_bg_rates[] = {
#define RATE(N, R, F) {                         \
      .flags = (F),                             \
      .bitrate = (R),                           \
      .hw_value = (N),                          \
      .hw_value_short = (N)                     \
   }
   RATE(MAC_RATE_QPSK_1MBIT,   10, 0),
   RATE(MAC_RATE_QPSK_2MBIT,   20, IEEE80211_RATE_SHORT_PREAMBLE),
   RATE(MAC_RATE_QPSK_5_5MBIT, 55, IEEE80211_RATE_SHORT_PREAMBLE),
   RATE(MAC_RATE_OFDM_6MBIT,   60, 0),
   RATE(MAC_RATE_OFDM_9MBIT,   90, 0),
   RATE(MAC_RATE_QPSK_11MBIT, 110, IEEE80211_RATE_SHORT_PREAMBLE),
   RATE(MAC_RATE_OFDM_12MBIT, 120, 0),
   RATE(MAC_RATE_OFDM_18MBIT, 180, 0),
   RATE(MAC_RATE_OFDM_24MBIT, 240, 0),
   RATE(MAC_RATE_OFDM_36MBIT, 360, 0),
   RATE(MAC_RATE_OFDM_48MBIT, 480, 0),
   RATE(MAC_RATE_OFDM_54MBIT, 540, 0),
};

static struct ieee80211_rate smd_a_rates[] = {
#define RATE(N, R, F) {                         \
      .flags = (F),                             \
      .bitrate = (R),                           \
      .hw_value = (N),                          \
      .hw_value_short = (N)                     \
   }
   RATE(MAC_RATE_OFDM_6MBIT,   60, 0),
   RATE(MAC_RATE_OFDM_9MBIT,   90, 0),
   RATE(MAC_RATE_OFDM_12MBIT, 120, 0),
   RATE(MAC_RATE_OFDM_18MBIT, 180, 0),
   RATE(MAC_RATE_OFDM_24MBIT, 240, 0),
   RATE(MAC_RATE_OFDM_36MBIT, 360, 0),
   RATE(MAC_RATE_OFDM_48MBIT, 480, 0),
   RATE(MAC_RATE_OFDM_54MBIT, 540, 0),
};

static struct ieee80211_supported_band smd_2ghz_band = {
   .channels   = smd_2ghz_channels,
   .bitrates   = smd_bg_rates,
   .band       = IEEE80211_BAND_2GHZ,
   .n_channels = ARRAY_SIZE(smd_2ghz_channels),
   .n_bitrates = ARRAY_SIZE(smd_bg_rates),
};

static struct ieee80211_supported_band smd_5ghz_band = {
   .channels   = smd_5ghz_channels,
   .bitrates   = smd_a_rates,
   .band       = IEEE80211_BAND_5GHZ,
   .n_channels = ARRAY_SIZE(smd_5ghz_channels),
   .n_bitrates = ARRAY_SIZE(smd_a_rates),
};

static void
ht_setup(struct ieee80211_hw *hw,
         struct ieee80211_supported_band *band,
         const struct nanoc_device_descr *device_descr)
{
   memset(&band->ht_cap, 0, sizeof(band->ht_cap));
   if (device_descr->ndd_features & NANOC_DEVICE_FEAT_HT20) {
      /* XXX fill in correct values*/
      band->ht_cap.cap |= IEEE80211_HT_CAP_GRN_FLD;
      band->ht_cap.cap |= IEEE80211_HT_CAP_SGI_20;

      if(device_descr->ndd_features & NANOC_DEVICE_FEAT_STBC_RX) {
         band->ht_cap.cap |= (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
      }
#ifdef NRX_CONFIG_AMPDU_16K
      band->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_16K;
#else
      /* AMPDU_8K */
      band->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_8K;
#endif
      band->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;
      band->ht_cap.mcs.rx_mask[0] = 0xff;
      band->ht_cap.mcs.rx_highest = 72;
      band->ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
      if(device_descr->ndd_features & NANOC_DEVICE_FEAT_HT40) {
         band->ht_cap.cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;
         band->ht_cap.cap |= IEEE80211_HT_CAP_SGI_40;
         band->ht_cap.mcs.rx_highest = 150;
      }
      band->ht_cap.ht_supported = true;
      if(device_descr->ndd_features & NANOC_DEVICE_FEAT_AMPDU_TX) {
         ieee80211_hw_set(hw, AMPDU_AGGREGATION);
         hw->max_tx_aggregation_subframes = 32;
      }
   }
   if(disable_short_gi)
      band->ht_cap.cap &= ~(IEEE80211_HT_CAP_SGI_20|IEEE80211_HT_CAP_SGI_40);
}

static const struct ieee80211_iface_limit smd_iface_limits[] = {
   {
      .types = BIT(NL80211_IFTYPE_P2P_GO)
      | BIT(NL80211_IFTYPE_AP),
      .max = 1
   },
   {
      .types = BIT(NL80211_IFTYPE_STATION)
      | BIT(NL80211_IFTYPE_P2P_CLIENT),
      .max = 2
   }
};

static struct ieee80211_iface_combination smd_iface_combinations[] = {
   {
      .limits = smd_iface_limits,
      .n_limits = ARRAY_SIZE(smd_iface_limits),
      .num_different_channels = 1,
      .max_interfaces = 2,
      .beacon_int_infra_match = false
   }
};


static struct ieee80211_hw *
smd_alloc_hw(struct device *dev,
             const struct nanoc_device_descr *device_descr,
             unsigned char *mac_addr)
{
   struct ieee80211_hw *hw;
   struct smd_hw_priv *sh;
   char wqname[16];

   if ((hw = ieee80211_alloc_hw(sizeof(*sh), &smd_ieee80211_ops)) == NULL) {
      return NULL;
   }
   sh = hw_to_sh(hw);

   sh->sh_hw = hw;
   atomic_set(&sh->sh_transid, 0);
   sh->sh_flags = 0;
   sh->sh_frag_threshold = 0; /* disabled */
   spin_lock_init(&sh->sh_lock);
   spin_lock_init(&sh->sh_scan_lock);
   de_timer_init(&sh->sh_scan_timer, scan_timeout_cb);
   sema_init(&sh->sh_cmd_sem, 1);
   init_waitqueue_head(&sh->sh_link_wait);
   snprintf(wqname, sizeof(wqname), "txwq/%s", wiphy_name(hw->wiphy));
   sh->sh_txwq = alloc_workqueue(wqname,
                                 WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
   skb_queue_head_init(&sh->sh_txqueue);
   NRX_WORKER_INIT(sh, sh_txwork, smd_process_tx_queue);
   INIT_LIST_HEAD(&sh->sh_transactions);

   sh->sh_rxfilter = 0;

   SET_IEEE80211_DEV(hw, dev);
   SET_IEEE80211_PERM_ADDR(hw, mac_addr);

   /* XXX this is a quick workaround to get multiple addresses,
      required for concurrent network, just take the assigned address,
      and make the second one local, with a simple bit change */
   memcpy(sh->sh_addresses[0].addr, mac_addr, ETH_ALEN);
   memcpy(sh->sh_addresses[1].addr, mac_addr, ETH_ALEN);
   sh->sh_addresses[1].addr[0] |= 2; /* link local */
   sh->sh_addresses[1].addr[5] ^= 0x10;  /* toggle a bit */

   hw->wiphy->n_addresses = ARRAY_SIZE(sh->sh_addresses);
   hw->wiphy->addresses = sh->sh_addresses;

   hw->rate_control_algorithm = NULL;

   hw->max_rates = 4;
   hw->max_rate_tries = 7;

   ieee80211_hw_set(hw, SIGNAL_DBM);
   ieee80211_hw_set(hw, SUPPORTS_PS);
   ieee80211_hw_set(hw, SUPPORTS_DYNAMIC_PS);
   ieee80211_hw_set(hw, REPORTS_TX_ACK_STATUS);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
   hw->flags |= IEEE80211_HW_BEACON_FILTER;
   hw->flags |= IEEE80211_HW_SUPPORTS_CQM_RSSI;
#endif

   if(fw_rate_adaptation)
      ieee80211_hw_set(hw, HAS_RATE_CONTROL);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
   /* don't know how this is supposed to work otherwise */
   ieee80211_hw_set(hw, WANT_MONITOR_VIF);
#endif

   hw->max_rx_aggregation_subframes = 32;

   hw->wiphy->interface_modes = (BIT(NL80211_IFTYPE_STATION) |
                                 BIT(NL80211_IFTYPE_AP) |
                                 BIT(NL80211_IFTYPE_MONITOR) |
                                 BIT(NL80211_IFTYPE_P2P_GO) |
                                 BIT(NL80211_IFTYPE_P2P_CLIENT));
   hw->wiphy->max_scan_ssids = 3;
   hw->wiphy->max_scan_ie_len = IEEE80211_MAX_DATA_LEN;
   hw->wiphy->max_remain_on_channel_duration = 30000;
   // hw->wiphy->flags |= WIPHY_FLAG_PS_ON_BY_DEFAULT; /* disable for now */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
   hw->wiphy->flags |= WIPHY_FLAG_AP_UAPSD;
#endif

   if(device_descr->ndd_features & NANOC_DEVICE_FEAT_BROKEN_TX_POWER)
      set_bit(SMD_FLAG_SH_BROKEN_TX_POWER, &sh->sh_flags);

   if(!disable_concurrent_net){
      hw->wiphy->iface_combinations = smd_iface_combinations;
      hw->wiphy->n_iface_combinations = ARRAY_SIZE(smd_iface_combinations);
   }

   sh->sh_tx_conf.num_queues = 4;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
   sh->sh_psconf.receive_all_dtim          = 1;
   sh->sh_psconf.uapsd                     = 0;
   sh->sh_psconf.traffic_timeout           = 0;
   sh->sh_psconf.periodic_service_interval = 0;
   sh->sh_psconf.fixed_service_interval    = 0;
   sh->sh_psconf.listen_interval           = 0;
#endif

   hw->extra_tx_headroom = (sizeof(nrx_hic_message_t) +
                            sizeof(nrx_frame_tx_req_t) +
                            2 * (sizeof(mac_retrans_t) * IEEE80211_TX_MAX_RATES + sizeof(nrx_tlv_head_t))) +
                            /* Add room for P2P PS IEs. Workaround until supplicant inserts P2P PS IEs itself */
                            ((sizeof(struct ieee80211_vendor_ie)+sizeof(struct m80211_p2p_attr_noa) +
                            sizeof(struct m80211_noa_desc) + 3) & (~3)) /* 32-bit aligned */;
   hw->extra_tx_headroom = ALIGN(hw->extra_tx_headroom, 4);
   hw->vif_data_size = sizeof(struct smd_vif_priv);
   hw->sta_data_size = sizeof(struct smd_sta_priv);
   hw->queues = 4;
   hw->max_listen_interval = 5; /* what's this used for? */

   /* 2.4GHz */
   ht_setup(hw, &smd_2ghz_band, device_descr);
   hw->wiphy->bands[IEEE80211_BAND_2GHZ] = &smd_2ghz_band;

   /* 5GHz */
   if(device_descr->ndd_features & NANOC_DEVICE_FEAT_5GHZ) {
      ht_setup(hw, &smd_5ghz_band, device_descr);
      hw->wiphy->bands[IEEE80211_BAND_5GHZ] = &smd_5ghz_band;
   }

   return hw;
}

static
NRX_WORKER_DECLARE(smd_register_hw_worker)
{
   NRX_WORKER_SETUP(struct smd_hw_priv, sh, sh_regwork.work);

   if (ieee80211_register_hw(sh->sh_hw) != 0) {
      de_info(DE_TR_WIFI, "Failed to register ieee80211 device");
   } else {
      set_bit(SMD_FLAG_SH_REGISTERED, &sh->sh_flags);
      smd_debugfs_init(sh); /* must be done after wiphy_register */
#ifdef NRX_CONFIG_ENABLE_STATISTICS
      smd_stat_init(sh);
#endif
   }
}

static struct nanoc_context *
smd_new_device(void *hwdev,
               const struct nanoc_device_descr *device_descr,
               unsigned char *mac_addr)
{
   if (device_descr->ndd_features & NANOC_DEVICE_FEAT_WIFI) {
      struct device       *dev = hwdev;
      struct ieee80211_hw *hw  = smd_alloc_hw(dev, device_descr, mac_addr);
      struct smd_hw_priv  *sh  = hw_to_sh(hw);

      /* Workaround for a shortcoming in nanocore:
       *
       * Upon detection of a new device, the new_device op (this
       * function) is called, which returns a context used as a reference
       * to nanocore internal state, this context is later used in any
       * nanocore related actions.
       *
       * However, there is no guarantee that no calls (such as op_start),
       * which can happen directly after ieee80211_register_hw, happens
       * before this context is setup.  In this case the reference is
       * still NULL (in nanocore) and an assert is triggered.
       *
       * We solve this by deferring the register a short while, hoping
       * that the return from this function and assignment is done.
       *
       * A better solution to this is to explicitly registering the
       * context at this point (before ieee80211_register_hw), but this
       * is left as a future excercise.
       */

      NRX_DELAYED_WORKER_INIT(sh, sh_regwork, smd_register_hw_worker);

      /* can't use ieee80211_queue_delayed_work before
       * ieee80211_register_hw */
      schedule_delayed_work(&sh->sh_regwork, 1);

      return (struct nanoc_context *)hw;
   } else {
      return NULL;
   }
}

static void
smd_del_device(struct nanoc_context *context)
{
   struct ieee80211_hw *hw = (struct ieee80211_hw *)context;
   struct smd_hw_priv *sh = hw_to_sh(hw);

   smd_debugfs_remove(sh);

   de_info(DE_TR_WIFI, "hw=%p", hw);
   clear_bit(SMD_FLAG_SH_LINKUP_DATA, &sh->sh_flags);
   clear_bit(SMD_FLAG_SH_LINKUP_CMD, &sh->sh_flags);

   set_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags);
   wake_up(&sh->sh_link_wait);
   ieee80211_stop_queues(hw);

   if (test_and_clear_bit(SMD_FLAG_SH_REGISTERED, &sh->sh_flags)) {
      ieee80211_unregister_hw(hw);
   }
   flush_tx_queues(sh, true);
   if (sh->sh_dummy_netid != 0) {
      nanoc_free_net_id((struct nanoc_context *)hw, sh->sh_dummy_netid);
   }
   destroy_workqueue(sh->sh_txwq);
   ieee80211_free_hw(hw);
}

static struct nanoc_ops smd_nanoc_ops = {
   .get_q         = smd_get_q,
   .recv_handler  = smd_rx_packet,
   .new_device    = smd_new_device,
   .del_device    = smd_del_device,
   .link_up       = smd_link_up,
   .link_down     = smd_link_down,
   .restart       = smd_driver_reset,
   .send_complete = smd_transport_complete
};

int __init
smd_wifi_init(void)
{
   return nanoc_register(&smd_nanoc_ops);

}

void __exit
smd_wifi_exit(void)
{
}
