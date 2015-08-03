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

#ifndef __smd_common_h__
#define __smd_common_h__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/etherdevice.h>
#include <asm/atomic.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <net/mac80211.h>
#define NRX_VARDATA(_name) unsigned char _name[0];
#define LIST_OF_INFORMATION_ELEMENTS(_name) unsigned char _name[0];
#define LIST_OF_TLV(_name) unsigned char _name[0];
#include "nanocore.h"

#define NRX_RX_FLAG_CRYPTO_FAILURE (NRX_RX_FLAG_CRYPTO_REPLAY_FAILURE       | \
                                    NRX_RX_FLAG_CRYPTO_GROUP_MIC_FAILURE    | \
                                    NRX_RX_FLAG_CRYPTO_PAIRWISE_MIC_FAILURE)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
/**
 * enum ieee80211_frame_release_type - frame release reason
 * @IEEE80211_FRAME_RELEASE_PSPOLL: frame released for PS-Poll
 * @IEEE80211_FRAME_RELEASE_UAPSD: frame(s) released due to
 *	frame received on trigger-enabled AC
 */
enum ieee80211_frame_release_type {
	IEEE80211_FRAME_RELEASE_PSPOLL,
	IEEE80211_FRAME_RELEASE_UAPSD,
};
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#define NRX_USE_CHANNEL_CONTEXT 1
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0) && defined(NRX_ENABLE_CHANNEL_CONTEXT)
#define NRX_USE_CHANNEL_CONTEXT 1
#else
#undef NRX_USE_CHANNEL_CONTEXT
#endif

/************************************************************/
/************************************************************/
struct smd_vif_priv;
struct smd_transaction;

//TODO: consider replacing these with spin_lock_bh (bottom half locking only)
#define HWLOCK(SH) spin_lock_irqsave(&(SH)->sh_lock, (SH)->sh_irqflags)
#define HWUNLOCK(SH) spin_unlock_irqrestore(&(SH)->sh_lock, (SH)->sh_irqflags)

#ifdef NRX_CONFIG_ENABLE_STATISTICS
struct smd_statistics {
   spinlock_t      stat_lock;

   struct timeval  stat_reset_time;

   uint64_t        stat_tx_bytes;
   uint32_t        stat_tx_frames;
   uint32_t        stat_tx_tries[4];
   uint32_t        stat_tx_status[8];
   uint32_t        stat_tx_rates[28];
   uint32_t        stat_tx_ampdu_req;
   uint32_t        stat_tx_ampdu_back;

   uint64_t        stat_rx_bytes;
   uint32_t        stat_rx_frames;
   uint32_t        stat_rx_rates[28];
};

#define LOCK_STATS(SH)   spin_lock(&(SH)->sh_statistics.stat_lock)
#define UNLOCK_STATS(SH) spin_unlock(&(SH)->sh_statistics.stat_lock)

#endif

struct smd_hw_priv {
   spinlock_t                  sh_lock;
   unsigned long               sh_irqflags;
   struct ieee80211_hw        *sh_hw;
   struct delayed_work         sh_regwork;
   const struct smd_transport *sh_transport;
   struct semaphore            sh_cmd_sem;
   struct sk_buff_head         sh_txqueue;
   struct work_struct          sh_txwork;
   struct workqueue_struct    *sh_txwq;
   nrx_mlme_config_tx_req_t    sh_tx_conf;
   nrx_net_id_t                sh_dummy_netid;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
   nrx_mlme_power_mgmt_req_t   sh_psconf;
#endif
   uint16_t                    sh_listen_interval;
   group_addr_filter_t         sh_mc_filter;
   atomic_t                    sh_transid;
   struct smd_vif_priv __rcu  *sh_vif[4];
   struct list_head            sh_transactions;
   size_t                      sh_num_stx;
   wait_queue_head_t           sh_link_wait;
   unsigned long               sh_flags;
#define SMD_FLAG_SH_REGISTERED    0
#define SMD_FLAG_SH_CHIP_UP       1
#define SMD_FLAG_SH_RESTARTING    2
#define SMD_FLAG_SH_LINKUP_CMD    3
#define SMD_FLAG_SH_LINKUP_DATA   4
#define SMD_FLAG_SH_UPDATED_SV    6
#define SMD_FLAG_SH_WAIT_FOR_CTW  7
#define SMD_FLAG_SH_FRAME_CONFIG  8
#define SMD_FLAG_SH_BROKEN_TX_POWER  9
#define SMD_FLAG_SH_BROKEN_FRAGMENTATION 10
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#define SMD_FLAG_SH_FLUSHING      11
#endif
   struct list_head            sh_attrs;
   uint32_t                    sh_frag_threshold;
   unsigned int                sh_filter_flags;
   spinlock_t                  sh_scan_lock; //protects sharing of the SCAN resource and its subtype flags
   de_timer_t                  sh_scan_timer;
   unsigned int                sh_scan_type;
#define SMD_FLAG_SCAN_NONE        0
#define SMD_FLAG_SCAN_SINGLE      1
#define SMD_FLAG_SCAN_PERIODIC    2
#define SMD_FLAG_SCAN_ROC         3
#define SMD_FLAG_SCAN_MONITOR     4
   struct dentry              *sh_debugfs_root;
#ifdef NRX_CONFIG_ENABLE_STATISTICS
   struct dentry              *sh_debugfs_stats;
   struct smd_statistics       sh_statistics;
#endif
   struct mac_address          sh_addresses[2];
   uint8_t                     sh_rxfilter; /* cf. nrx_cmd_rxfilter_filters_t */
   uint32_t                    sh_api_version;

   mac_chn_prop_t              sh_monitor_channel;
};

#define QUEUE_TXWORK(SH)				\
  queue_work((SH)->sh_txwq, &(SH)->sh_txwork)


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0)
#define FREE_TXSKB(SH, SKB)                     \
   ieee80211_free_txskb((SH)->sh_hw, (SKB))
#else
#define FREE_TXSKB(SH, SKB)                     \
   dev_kfree_skb_any(SKB)
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static inline struct ieee80211_channel *
sh_current_channel(const struct smd_hw_priv *sh)
{
   return sh->sh_hw->conf.channel;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */

static inline int
sh_txpower(const struct smd_hw_priv *sh)
{
   return sh->sh_hw->conf.power_level;
}


typedef void (*smd_completion_func_t)(struct smd_transaction*);

struct smd_transaction {
   struct smd_hw_priv     *st_sh;
   struct list_head        st_list;
   uint16_t                st_transid;
   uint8_t                 st_netid;
   uint8_t                 st_queue:1;
   uint8_t                 st_restart:1;
   struct sk_buff         *st_txskborg; /* as received from mac8011 */
   struct sk_buff         *st_txskb;    /* cloned copy */
   struct sk_buff         *st_rxskb;
   struct kref             st_ref;
   struct completion       st_completion;
   smd_completion_func_t   st_tx_complete;
   smd_completion_func_t   st_tx_confirm;
};

struct smd_vif_priv {
   struct ieee80211_hw           *sv_hw;
   nrx_net_id_t                   sv_net_id;
   nrx_net_type_t                 sv_net_type;
   unsigned int                   sv_index;
   uint16_t                       sv_seqno;
   uint8_t                        sv_address[ETH_ALEN];
   bool                           sv_enable_beacon;
   nrx_bss_conf_t                *sv_bssconf;
   size_t                         sv_bssconf_size;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
   nrx_mlme_power_mgmt_req_t      sv_psconf;
#endif
   de_timer_t                     sv_p2p_timer;
   uint32_t                       sv_p2p_noa_index; /* 32-bit to be able to set an invalid init value. */
   uint32_t                       sv_p2p_noa_starttime[2]; /* 32-bit to be able to set an invalid init value. */
   nrx_mac_addr_t                 sv_ctw_bssid;
   nrx_mlme_p2p_ps_set_conf_req_t sv_p2p_ps_req;
   unsigned long                  sv_flags;
#define SMD_FLAG_SV_STARTED         0
#define SMD_FLAG_SV_BSSCONF_CHANGED 1
#define SMD_FLAG_SV_P2P             3
#define SMD_FLAG_SV_ENABLED         6
#define SMD_FLAG_SV_P2P_PS_CHANGED  8
#define SMD_FLAG_SV_P2P_PS_ENABLED  9
#define SMD_FLAG_SV_BEACON_TRIGGER 10
#define SMD_FLAG_SV_HIBERNATING    11
};

#define SV_IS_VALID(SV)         test_bit(SMD_FLAG_SV_ENABLED,        &(SV)->sv_flags)
#define SV_IS_P2P(SV)           test_bit(SMD_FLAG_SV_P2P,            &(SV)->sv_flags)
#define SV_IS_P2P_PS_ACTIVE(SV) test_bit(SMD_FLAG_SV_P2P_PS_ENABLED, &(SV)->sv_flags)

#define SV_P2P_PS_FW_TRACKS_NOA(SV)    ((SV)->sv_bssconf->p2p_flags & NRX_P2P_FLAGS_PS_TRACK_NOA)
#define SV_P2P_PS_DRV_UPTODATE(SV)     ((SV)->sv_bssconf->p2p_flags & NRX_P2P_FLAGS_PS_UPDATES_TO_DRV)

#define vif_to_sv(VIF) ((struct smd_vif_priv*)&(VIF)->drv_priv)
#define sv_to_vif(SV) container_of((void*)(SV), struct ieee80211_vif, drv_priv)
#define hw_to_sh(HW) ((struct smd_hw_priv*)(HW)->priv)
#define sv_to_sh(SV) hw_to_sh((SV)->sv_hw)

#define FOREACH_VIF(SH, F, ...) ({                              \
         unsigned int __i;                                      \
         struct smd_vif_priv *__sv;                             \
         for(__i = 0; __i < ARRAY_SIZE((SH)->sh_vif); __i++) {  \
            rcu_read_lock();                                    \
            __sv = rcu_dereference((SH)->sh_vif[__i]);          \
            if(__sv != NULL                                     \
               && test_bit(SMD_FLAG_SV_ENABLED,                 \
                           &__sv->sv_flags))                    \
               F(__sv , ## __VA_ARGS__);                        \
            rcu_read_unlock();                                  \
         }                                                      \
      })

#define SV_CHECK_IFTYPE_STATION(SV, F, ...) ({                  \
         if(sv_to_vif(SV)->type == NL80211_IFTYPE_STATION)      \
            F((SV) , ## __VA_ARGS__);                           \
      })

#define SV_CHECK_BSSCONF(SV, F, ...) ({                         \
         if((SV)->sv_bssconf != NULL)                           \
            F((SV) , ## __VA_ARGS__);                           \
      })

struct smd_sta_tid {
   unsigned long  st_flags;
#define SMD_STA_TID_FLAG_GOT_FRAME         0    /* At least one frame has been received on this TID */
#define SMD_STA_TID_FLAG_AMPDU_ENABLED     1    /* Block ack agreement is operational. AMPDUs can be sent. */
#define SMD_STA_TID_FLAG_AMPDU_START_REQ   2
   uint16_t       st_ampdu_buf_size;
   uint16_t       st_last_seq_no;
   unsigned long  st_delba_time;
   unsigned long  st_ampdu_start_req_time;
};

struct smd_sta_priv {
   struct kref                  ss_refcnt;
   wait_queue_head_t            ss_waitqueue;
   struct smd_vif_priv         *ss_sv;
   unsigned long                ss_flags;
#define SMD_FLAG_SS_REMOVING       3    /* STA is about to be removed. No frames can be queued in ps queues */
   struct smd_sta_tid           ss_tid[16];
   uint16_t                     ss_rx_baa_mask;
   nrx_mlme_set_sta_info_req_t  ss_sta_info;
   uint32_t                     ss_txmask;
};
#define sta_to_ss(STA) ((struct smd_sta_priv*)&(STA)->drv_priv)
#define ss_to_sta(SS) container_of((void*)(SS), struct ieee80211_sta, drv_priv)

#define SS_GET(SS) while((SS) != NULL) {                        \
      BUG_ON(atomic_read(&(SS)->ss_refcnt.refcount) == 0);      \
      kref_get(&(SS)->ss_refcnt);                               \
      break;                                                    \
   }
#define SS_PUT(SS) while((SS) != NULL) {                \
      kref_put(&(SS)->ss_refcnt, smd_ss_release);       \
      break;                                            \
   }


struct smd_tx_info {
   struct smd_sta_priv *sti_ss;
   unsigned long        sti_discard_time;   /* jiffies */
   uint32_t             sti_flags;
   nrx_net_id_t         sti_netid;
#define SMD_TX_INFO_FLAG_INTERNAL BIT(0)
};

#define SMD_TX_INFO_SS(TXI)           (((struct smd_tx_info *)(TXI)->rate_driver_data)->sti_ss)
#define SMD_TX_INFO_DISCARD_TIME(TXI) (((struct smd_tx_info *)(TXI)->rate_driver_data)->sti_discard_time)
#define SMD_TX_INFO_FLAGS(TXI)        (((struct smd_tx_info *)(TXI)->rate_driver_data)->sti_flags)
#define SMD_TX_INFO_NETID(TXI)        (((struct smd_tx_info *)(TXI)->rate_driver_data)->sti_netid)

/************************************************************/

#define CTWINDOW         20

#define OPP_PS           0x80
#define CTWINDOW_MASK    0x7F

#define PRES_REQ_NOA_TYPE_PREFERRED     1
#define PRES_REQ_NOA_TYPE_ACCEPTABLE    2

#define NOA_ENDLESS         0xff

#define M80211_P2P_NOA_ATTR_ID 12

struct m80211_p2p_attr_noa
{
   u8     id;
   __le16 length;
   u8     index;
   u8     ctwindow_opp_params;
   /* None or several m80211_noa_desc may follow */
} __packed;

struct m80211_noa_desc
{
   u8     count_type;
   __le32 duration;
   __le32 interval;
   __le32 start_time;
} __packed;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
struct ieee80211_vendor_ie {
	u8 element_id;
	u8 len;
	u8 oui[3];
	u8 oui_type;
} __packed;
#endif


/************************************************************/
/************************************************************/

#include "smd_proto.h"

/************************************************************/
/************************************************************/

static inline int
smd_txn_decref(struct smd_transaction *stx)
{
   return kref_put(&stx->st_ref, smd_release_transaction);
}

static inline void
smd_txn_incref(struct smd_transaction *stx)
{
   kref_get(&stx->st_ref);
}

/************************************************************/
/************************************************************/

static inline int
resource_request_scan(struct smd_hw_priv *sh, unsigned int scan_type)
{
   return resource_request_release_scan(sh, SMD_FLAG_SCAN_NONE, scan_type);
}

static inline int
resource_test_scan(struct smd_hw_priv *sh, unsigned int scan_type)
{
   return resource_request_release_scan(sh, scan_type, scan_type);
}

static inline int
resource_release_scan(struct smd_hw_priv *sh, unsigned int scan_type)
{
   return resource_request_release_scan(sh, scan_type, SMD_FLAG_SCAN_NONE);
}

static inline void smd_broadcast_addr(nrx_mac_addr_t *addr)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
   eth_broadcast_addr(addr->octet);
#else
   memset(addr->octet, 0xff, sizeof(addr->octet));
#endif
}

#define P2P_OUI 0x506f9a09

#define TRACE_STX(STX, PREFIX) \
   de_debug(DE_TR_WIFI, PREFIX ":%p, %u, %u, %p, %p", (STX), (STX)->st_transid, (STX)->st_queue, (STX)->st_txskb, (STX)->st_rxskb)

#define PRINTBUF(PREFIX, BUF, LEN) \
   nanoc_printbuf((PREFIX), (BUF), (LEN))

#define FOREACH_TLV(TLV, START, SIZE)                                   \
   for((TLV) = (nrx_tlv_head_t*)(START);                                \
       ((const u8*)(TLV)) - ((const u8*)(START))                        \
          + sizeof(nrx_tlv_head_t) <= (SIZE)                            \
          && ((const u8*)(TLV)) - ((const u8*)(START))                  \
          + sizeof(nrx_tlv_head_t) + (TLV)->len <= (SIZE);              \
       (TLV) += 1 + DIV_ROUND_UP((TLV)->len, sizeof(nrx_tlv_head_t)))

#endif /* __smd_common_h__ */
