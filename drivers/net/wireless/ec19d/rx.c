#include "smd_common.h"
#include <linux/ratelimit.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
#define RX_FLAG_MACTIME_START RX_FLAG_MACTIME_MPDU
#endif

static void
print_rx_status(struct ieee80211_rx_status *rxs)
{
#if 0
   de_debug(DE_TR_WIFI, "mactime = %llx", rxs->mactime);
   de_debug(DE_TR_WIFI, "band = %u", rxs->band);
   de_debug(DE_TR_WIFI, "freq = %u", rxs->freq);
   de_debug(DE_TR_WIFI, "signal = %d", rxs->signal);
   de_debug(DE_TR_WIFI, "antenna = %u", rxs->antenna);
   de_debug(DE_TR_WIFI, "rate_idx = %u", rxs->rate_idx);
   de_debug(DE_TR_WIFI, "flag = %x", rxs->flag);

#define Z(I, F) if((I) & (F)) de_debug(DE_TR_WIFI, "  " #F);
   Z(rxs->flag, RX_FLAG_MMIC_ERROR);
   Z(rxs->flag, RX_FLAG_DECRYPTED);
   Z(rxs->flag, RX_FLAG_MMIC_STRIPPED);
   Z(rxs->flag, RX_FLAG_IV_STRIPPED);
   Z(rxs->flag, RX_FLAG_FAILED_FCS_CRC);
   Z(rxs->flag, RX_FLAG_FAILED_PLCP_CRC);
   Z(rxs->flag, RX_FLAG_MACTIME_START);
   Z(rxs->flag, RX_FLAG_SHORTPRE);
   Z(rxs->flag, RX_FLAG_HT);
   Z(rxs->flag, RX_FLAG_40MHZ);
   Z(rxs->flag, RX_FLAG_SHORT_GI);
#undef ZZ
#endif
}

static int
smd_get_rate_index(struct ieee80211_supported_band *band, mac_rate_t rate)
{
   int i;
   for(i = 0; i < band->n_bitrates; i++) {
      if(band->bitrates[i].hw_value == rate) {
         return i;
      }
   }
   de_warn(DE_TR_WIFI, "rate = %u, not found", rate);
   return 0;
}

void
smd_rx_generic(struct smd_hw_priv *sh,
               struct ieee80211_vif * vif,
               struct sk_buff *skb)
{
   struct ieee80211_rx_status *rxi = IEEE80211_SKB_RXCB(skb);
   nrx_frame_rx_ind_t         *ind = (nrx_frame_rx_ind_t *)skb->data;
   struct ieee80211_hdr       *mac_hdr;
   struct smd_vif_priv        *sv = NULL;
   struct ieee80211_sta       *sta;

   if(vif != NULL)
      sv = vif_to_sv(vif);

   /* Position skb data pointer to beginning of MAC frame */
   skb_pull(skb, ind->payload_offset);
   mac_hdr = (struct ieee80211_hdr*)skb->data;

#ifdef NRX_CONFIG_ENABLE_STATISTICS
   smd_stat_update_rx_ind_statistics(sh, ind, skb->len);
#endif

   /* Convert rx status info */
   memset(rxi, 0, sizeof(*rxi));
   rxi->flag   |= RX_FLAG_MACTIME_START; /* XXX time of preamble */
   rxi->mactime = ind->start_ts;
   rxi->band    = IEEE80211_BAND_2GHZ;
   rxi->freq = ind->freq;
   rxi->signal  = (int8_t)ind->rssi;
   rxi->antenna = 0;
   if(MAC_MOD(ind->rate) == MAC_MOD_HT
      || MAC_MOD(ind->rate) == MAC_MOD_HT40) {
      rxi->rate_idx = MAC_SUBRATE(ind->rate);
      rxi->flag |= RX_FLAG_HT;
      if (MAC_MOD(ind->rate) == MAC_MOD_HT40)
         rxi->flag |= RX_FLAG_40MHZ;
   } else {
      rxi->rate_idx = smd_get_rate_index(sh->sh_hw->wiphy->bands[rxi->band],
                                         ind->rate);
   }
   if(ind->flags & NRX_RX_FLAG_DECRYPTED) {
      rxi->flag |= RX_FLAG_DECRYPTED;
      rxi->flag |= RX_FLAG_MMIC_STRIPPED;
      rxi->flag |= RX_FLAG_IV_STRIPPED;
   }

   if(ind->flags & NRX_RX_FLAG_SHORT_PREAMBLE)
      rxi->flag |= RX_FLAG_SHORTPRE;

   if(ind->flags & NRX_RX_FLAG_CRYPTO_MICHAEL_FAILURE)
      rxi->flag |= RX_FLAG_MMIC_ERROR;

   if(ind->flags & NRX_RX_FLAG_CRYPTO_FAILURE) {
      de_error(DE_TR_WIFI, "Dropping undecryptable frame from %pM", mac_hdr->addr2);
      dev_kfree_skb_any(skb);
      return;
   }
   if(ind->flags & NRX_RX_FLAG_AMPDU) {
      if(!(ind->flags & NRX_RX_FLAG_PART_OF_BAA)) {
         static DEFINE_RATELIMIT_STATE(_rs, HZ / 10, 5);
         if(__ratelimit(&_rs))
            de_info(DE_TR_WIFI, "Dropping A-MPDU subframe without BAA");
         /* XXX it's not our responsibility to send a DELBA here, but
          * nobody else has stepped up */
         rcu_read_lock();
         sta = ieee80211_find_sta(vif, mac_hdr->addr2);
         if(sta != NULL)
            send_delba(sh, sv, sta_to_ss(sta), ind->rate, mac_hdr);
         else
            de_info(DE_TR_WIFI, "vif = %p, sta not found: %pMF", vif, mac_hdr->addr2);
         rcu_read_unlock();

         dev_kfree_skb_any(skb);
         return;
      }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
      rxi->flag |= RX_FLAG_AMPDU_DETAILS;
      rxi->ampdu_reference = ind->start_ts;
#endif
   }

   if(ind->flags & NRX_RX_FLAG_FCS_FAIL)
      rxi->flag |= RX_FLAG_FAILED_FCS_CRC;

   if(ind->flags & NRX_RX_FLAG_GREENFIELD)
      rxi->flag |= RX_FLAG_SHORT_GI;

   if(ind->flags & NRX_RX_FLAG_SHORT_GI)
      rxi->flag |= RX_FLAG_SHORT_GI;

   if (sv != NULL
       && SV_IS_P2P(sv)
       && (ieee80211_is_probe_resp(mac_hdr->frame_control)
           || ieee80211_is_beacon(mac_hdr->frame_control))
       && memcmp(mac_hdr->addr2, sv->sv_bssconf->bssid.octet, ETH_ALEN) == 0)
      p2p_check_beacon_probersp(sv, skb);

   print_rx_status(rxi);

   ieee80211_rx_ni(sh->sh_hw, skb);
}
