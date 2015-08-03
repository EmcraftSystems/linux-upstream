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

/* Copyright (C) 2011 Nanoradio AB */
/* $Id$ */
/* This is a generated file. */
#ifndef __smd_proto_h__
#define __smd_proto_h__

#include <stdarg.h>
#include "driver_nrx_nl_ext.h"

struct smd_hw_priv;

/* BEGIN GENERATED PROTOS */

/* debugfs.c */

void smd_debugfs_init(struct smd_hw_priv *sh);

void smd_debugfs_remove(struct smd_hw_priv *sh);

#if defined(NRX_CONFIG_ENABLE_STATISTICS)
void smd_stat_init(struct smd_hw_priv *sh);
#endif

#if defined(NRX_CONFIG_ENABLE_STATISTICS)
void
smd_stat_update_rx_ind_statistics(struct smd_hw_priv *sh,
                                  nrx_frame_rx_ind_t *ind,
                                  unsigned int frame_len);
#endif

#if defined(NRX_CONFIG_ENABLE_STATISTICS)
void
smd_stat_update_tx_cfm_statistics(struct smd_hw_priv *sh,
                                  nrx_frame_tx_cfm_t *cfm);
#endif

#if defined(NRX_CONFIG_ENABLE_STATISTICS)
void
smd_stat_update_tx_req_statistics(struct smd_hw_priv *sh,
                                  nrx_frame_tx_req_t *req,
                                  unsigned int frame_len);
#endif

/* delba.c */

int
send_delba(struct smd_hw_priv *sh,
           struct smd_vif_priv *sv,
           struct smd_sta_priv *ss,
           mac_rate_t rate,
           struct ieee80211_hdr *hdr);

/* driver.c */

int frame_config(struct smd_hw_priv *sh, unsigned int filter_flags);

void smd_command_complete(struct smd_transaction *stx);

void smd_command_confirm(struct smd_transaction *stx);

struct ieee80211_vif *
smd_lookup_vif(struct smd_hw_priv *sh,
               unsigned net_id);

void
smd_op_allow_buffered_frames(struct ieee80211_hw *hw,
                             struct ieee80211_sta *sta,
                             u16 tids,
                             int num_frames,
                             enum ieee80211_frame_release_type reason,
                             bool more_data);

void
smd_op_release_buffered_frames(struct ieee80211_hw *hw,
                               struct ieee80211_sta *sta,
                               u16 tids,
                               int num_frames,
                               enum ieee80211_frame_release_type reason,
                               bool more_data);

void smd_ss_release(struct kref *ref);

int smd_update_beacon(struct smd_vif_priv *sv);

#if defined(NRX_USE_CHANNEL_CONTEXT)
bool
smd_update_chn_prop_from_chan_def(mac_chn_prop_t *chan_prop,
                                  const struct cfg80211_chan_def *chandef);
#endif

bool
smd_update_chn_prop_from_channel(mac_chn_prop_t *chan_prop,
                                 const struct ieee80211_channel *channel,
                                 enum nl80211_channel_type channel_type);

int smd_update_sv(struct smd_vif_priv *sv);

void __exit smd_wifi_exit(void);

int __init smd_wifi_init(void);

/* ie.c */

const u8*
smd_ie_find(const u8 *ies,
            size_t ie_len,
            u8 ie_id,
            const u8 *ref);

const u8*
smd_ie_find_vendor(const u8 *ies,
                   size_t ie_len,
                   __be32 oui,
                   const u8 *ref);

/* mib.c */

int
smd_mib_set(struct smd_hw_priv *sh,
            struct smd_vif_priv *sv,
            nrx_mib_id_t id,
            const void *data,
            size_t size);

int
smd_mib_trigger_remove(struct smd_vif_priv *sv,
                       uint32_t trigger_id);

int
smd_mib_trigger_set(struct smd_vif_priv *sv,
                    nrx_mib_id_t mib_id,
                    uint32_t trigger_id,
                    bool rising_trigger,
                    uint32_t interval_us,
                    uint32_t level);

int smd_set_fcc(struct smd_hw_priv *sh, bool fcc);

int smd_set_tx_power(struct smd_hw_priv *sh, int32_t dbm);

int
smd_sv_set_arp_filter(struct smd_vif_priv *sv,
                      const arp_addr_filter_t *filter);

int
smd_sv_set_group_filter(struct smd_vif_priv *sv,
                        const group_addr_filter_t *filter);

/* p2p.c */

void
p2p_check_beacon_probersp(struct smd_vif_priv *sv,
                          struct sk_buff *skb);

void p2p_go_initial_ps(struct smd_vif_priv *sv);

int
p2p_insert_noa_attrib(struct smd_vif_priv *sv,
                      struct sk_buff *skb);

void p2p_ps_finished(struct ieee80211_vif *vif);

void p2p_send_ps_req(struct smd_vif_priv *sv);

void p2p_timeout_cb(de_timer_t *t);

#if defined(CONFIG_NL80211_TESTMODE) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
void
p2p_update_noa_from_nl(struct smd_vif_priv *sv,
                       nrx_buf_noa_t *noa);
#endif

#if defined(CONFIG_NL80211_TESTMODE) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
void
p2p_update_opp_from_nl(struct smd_vif_priv *sv,
                       nrx_buf_opp_t *opp);
#endif

/* rx.c */

void
smd_rx_generic(struct smd_hw_priv *sh,
               struct ieee80211_vif * vif,
               struct sk_buff *skb);

/* scan.c */

int
resource_request_release_scan(struct smd_hw_priv *sh,
                              unsigned int expected,
                              unsigned int requested);

int smd_monitor_mode_disable(struct smd_hw_priv *sh);

int smd_monitor_mode_enable(struct smd_hw_priv *sh);

bool smd_monitor_mode_enabled(struct smd_hw_priv *sh);

int smd_monitor_stop_scanjob(struct smd_hw_priv *sh);

#if defined(NRX_USE_CHANNEL_CONTEXT)
void
smd_monitor_update_chan_def(struct smd_hw_priv *sh,
                            const struct cfg80211_chan_def *chandef);
#endif

void
smd_monitor_update_channel(struct smd_hw_priv *sh,
                           struct ieee80211_channel *channel,
                           enum nl80211_channel_type channel_type);

int
smd_scan_request(struct smd_hw_priv *sh,
                 struct cfg80211_scan_request *sreq);

int
smd_scan_single_passive_channel(struct smd_hw_priv *sh,
                                const mac_chn_prop_t *channel,
                                uint32_t scan_flags);

int smd_stop_scan_job(struct smd_hw_priv *sh);

/* transaction.c */

void
smd_add_hic_header(struct smd_transaction *stx,
                   uint8_t type,
                   uint8_t id);

void *
smd_alloc_req(struct smd_hw_priv *sh,
              size_t size,
              nrx_hic_message_type_t hictype,
              nrx_hic_message_id_t hicid,
              struct smd_transaction **p_stx);

void *
smd_alloc_req_vif(struct smd_vif_priv *sv,
                  size_t size,
                  nrx_hic_message_type_t hictype,
                  nrx_hic_message_id_t hicid,
                  struct smd_transaction **p_stx);

struct smd_transaction*
smd_clone_transaction(struct smd_transaction *stx_orig);

struct smd_transaction *
smd_create_transaction(struct smd_hw_priv *sh,
                       unsigned netid);

void smd_fail_transactions(struct smd_hw_priv *sh);

void smd_release_transaction(struct kref *ref);

struct smd_transaction*
smd_txn_getref(struct smd_hw_priv *sh,
               uint16_t transid);

int smd_wait_all_transactions(struct smd_hw_priv *sh);

int smd_wait_transaction(struct smd_transaction *stx);

/* tx.c */

NRX_WORKER_DECLARE(smd_process_tx_queue);

void
flush_single_queue(struct smd_hw_priv *sh,
                   struct sk_buff_head *queue,
                   bool drop);

void flush_tx_queues(struct smd_hw_priv *sh, bool drop);

void
make_rate_entry(struct ieee80211_tx_rate *rate,
                struct ieee80211_supported_band *band,
                mac_retrans_t *rp,
                mac_retrans_t *rr);

void
smd_retransmit_frames(struct smd_hw_priv *sh,
                      nrx_tlv_frame_copy_req_t *tlv);

void
smd_tx_failure(struct ieee80211_hw *hw,
               struct sk_buff *skb,
               bool remove_header);

void
smd_tx_frame(struct ieee80211_hw *hw,
             struct ieee80211_sta *sta,
             struct sk_buff *skb);

bool smd_tx_queue_barrier(struct smd_hw_priv *sh);

void
tx_prepare(struct smd_hw_priv *sh,
           struct smd_vif_priv *sv,
           struct smd_sta_priv *ss,
           struct sk_buff *skb);

/* END GENERATED PROTOS */

#endif /* __smd_proto_h__ */
