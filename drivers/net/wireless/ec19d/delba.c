#include "smd_common.h"

static int set_bitrate_idx(struct ieee80211_supported_band *band,
                           mac_rate_t rate,
                           struct ieee80211_tx_rate *rc)
{
   unsigned int i;

   if(MAC_MOD(rate) == MAC_MOD_HT
      || MAC_MOD(rate) == MAC_MOD_HT40) {
      rc->idx = MAC_SUBRATE(rate);
      rc->flags = IEEE80211_TX_RC_MCS;
      if (MAC_MOD(rate) == MAC_MOD_HT40)
         rc->flags |= IEEE80211_TX_RC_40_MHZ_WIDTH;
      return 0;
   }

   for(i = 0; i < band->n_bitrates; i++) {
      if(band->bitrates[i].hw_value == rate) {
         rc->idx = i;
         rc->flags = 0;
         return 0;
      }
   }
   return -ENOENT;
}

static mac_rate_t
lowest_supported_rate_same_modulation(mac_rate_t rate)
{
   /* this is a simplistic approach */
   return rate & 0xff00;
}

static int
internal_send_data_frame(struct smd_hw_priv *sh,
                         struct smd_vif_priv *sv,
                         struct smd_sta_priv *ss,
                         mac_rate_t rate,
                         struct sk_buff *skb)
{
   struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
   int retval;

   memset(txi, 0, sizeof(*txi));

   txi->flags = 0;
   txi->band = IEEE80211_BAND_2GHZ;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
   txi->hw_queue = 0;
#endif

   retval = set_bitrate_idx(sh->sh_hw->wiphy->bands[txi->band],
                            rate,
                            &txi->control.rates[0]);
   if(retval != 0)
      return retval;

   txi->control.rates[0].count = 7;
   txi->control.rates[1].idx = -1;
   txi->control.rts_cts_rate_idx = -1;
   txi->control.vif = sv_to_vif(sv);
   txi->control.hw_key = NULL;

   SS_GET(ss);
   tx_prepare(sh, sv, ss, skb);
   SMD_TX_INFO_FLAGS(txi) |= SMD_TX_INFO_FLAG_INTERNAL;

   HWLOCK(sh);
   /* XXX is it always safe to queue @head? */
   skb_queue_head(&sh->sh_txqueue, skb);
   HWUNLOCK(sh);
   QUEUE_TXWORK(sh);

   return 0;
}

struct delba_frame {
   __le16  frame_control;
   __le16  duration;
   u8      da[6];
   u8      sa[6];
   u8      bssid[6];
   __le16  seq_ctrl;
   u8      category;
   u8      action_code;
   __le16  params;
   __le16  reason_code;
} __attribute__ ((packed));

int
send_delba(struct smd_hw_priv *sh,
           struct smd_vif_priv *sv,
           struct smd_sta_priv *ss,
           mac_rate_t rate,
           struct ieee80211_hdr *hdr)
{
   struct sk_buff *skb;
   struct delba_frame *delba;
   u8 tid;
   int retval;

   tid = *ieee80211_get_qos_ctl(hdr) & IEEE80211_QOS_CTL_TID_MASK;

   if(time_is_after_jiffies(ss->ss_tid[tid].st_delba_time))
      return 0;

   ss->ss_tid[tid].st_delba_time = jiffies + HZ / 10;

   skb = dev_alloc_skb(sizeof(*delba) + sh->sh_hw->extra_tx_headroom);
   if(skb == NULL)
      return -ENOMEM;

   skb_reserve(skb, sh->sh_hw->extra_tx_headroom);

   delba = (struct delba_frame*)skb_put(skb, sizeof(*delba));
   delba->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
                                      IEEE80211_STYPE_ACTION);
   delba->duration = 0;
   memcpy(delba->da, hdr->addr2, ETH_ALEN);
   memcpy(delba->sa, sv->sv_address, ETH_ALEN);
   memcpy(delba->bssid, sv->sv_bssconf->bssid.octet, ETH_ALEN);
   delba->seq_ctrl = 0;
   delba->category = WLAN_CATEGORY_BACK;
   delba->action_code = WLAN_ACTION_DELBA;
   delba->params = cpu_to_le16(tid << 12);
   delba->reason_code = cpu_to_le16(WLAN_REASON_QSTA_NOT_USE);

   rate = lowest_supported_rate_same_modulation(rate);

   retval = internal_send_data_frame(sh, sv, ss, rate, skb);
   if(retval != 0)
      dev_kfree_skb(skb);

   return retval;
}
