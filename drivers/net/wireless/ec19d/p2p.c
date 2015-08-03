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

/* Definitions ================================ */

#define P2P_PS_OFF  0
#define P2P_PS_OPP  (1<<0)
#define P2P_PS_NOA  (1<<1)

#define NOA_STARTTIME_REFRESH (30*60*1000)

/* Static declarations ============================== */

static void p2p_ps_stop(struct smd_vif_priv *sv);

/* Variables ======================================== */

static const struct ieee80211_vendor_ie p2p_vendor_ie =
{
   .element_id = 0xDD,
   .len = 4,
   .oui = {0x50, 0x6F, 0x9A},
   .oui_type = 9
};

static int p2p_ps = P2P_PS_OFF;
module_param(p2p_ps, int, 0644);
MODULE_PARM_DESC(p2p_ps, "p2p GO power save: 0 - off, 1 - opp, 2 - noa. Must be set prior to start of net.");

static int p2p_noa_duration = 20000;
module_param(p2p_noa_duration, int, 0644);
MODULE_PARM_DESC(p2p_noa_duration, "Absence duration in us");

static int p2p_noa_interval = 40000;
module_param(p2p_noa_interval, int, 0644);
MODULE_PARM_DESC(p2p_noa_interval, "Absence interval in us");

/* Global functions ================================= */

/* Configure fw to use a power save scheme in p2p group owner mode
 * NOTE: this is only used when P2P PS settings come as module parameters
 *       at initialization
 */
void
p2p_go_initial_ps(struct smd_vif_priv *sv)
{
   if (p2p_ps != P2P_PS_OFF)
   {
      struct smd_hw_priv             *sh  = hw_to_sh(sv->sv_hw);
      nrx_mlme_p2p_ps_set_conf_req_t *req = &sv->sv_p2p_ps_req;

      memset(req, 0, sizeof *req);
      req->conf.type = 0;
      if (sv->sv_enable_beacon) {
         req->conf.opp.ctwindow = CTWINDOW; /* According to the standard CTWINDOW must be at least 10 */
         if (p2p_ps & P2P_PS_OPP) {
                req->conf.type |= NRX_MLME_P2P_PS_CONF_OPP;
                req->conf.opp.enabled = 1;
         }
         if (p2p_ps & P2P_PS_NOA) {
            req->conf.type |= NRX_MLME_P2P_PS_CONF_NOA_1;
            req->conf.noa[0].duration = p2p_noa_duration;
            req->conf.noa[0].interval = p2p_noa_interval;
            req->conf.noa[0].count = 255; /* 255 = Forever */

            /* start timer for noa starttime refreshing */
            de_timer_start(&sv->sv_p2p_timer, NOA_STARTTIME_REFRESH);
         }
      }
      de_info(DE_TR_WIFI, "p2p_ps_conf set at init; type=%d", req->conf.type);
      set_bit(SMD_FLAG_SV_P2P_PS_CHANGED, &sv->sv_flags);
      set_bit(SMD_FLAG_SH_UPDATED_SV, &sh->sh_flags);
      sv->sv_p2p_noa_index++; /* Will result in 0 first time */
      QUEUE_TXWORK(sh);
   }
}

#ifdef CONFIG_NL80211_TESTMODE

void
p2p_update_noa_from_nl(struct smd_vif_priv *sv, nrx_buf_noa_t *noa)
{
   if (SV_IS_P2P(sv)) {
      struct smd_hw_priv             *sh  = hw_to_sh(sv->sv_hw);
      nrx_mlme_p2p_ps_set_conf_req_t *req = &sv->sv_p2p_ps_req;

      if (sv->sv_enable_beacon) {
         memset(&req->conf.noa[0], 0, sizeof(nrx_p2p_ps_noa_t));
         req->conf.type = NRX_MLME_P2P_PS_CONF_NOA_1;
         req->conf.noa[0].duration  = noa->duration;
         req->conf.noa[0].interval  = noa->interval ? noa->interval : sv->sv_bssconf->beacon_period*1024;
         req->conf.noa[0].starttime = noa->start    ? noa->start    : 0;  /* 0 means no offset from next DTIM */
         req->conf.noa[0].count     = noa->count;

         /* restart starttime timer */
         de_timer_stop(&sv->sv_p2p_timer);
         if(req->conf.noa[0].duration != 0)
            de_timer_start(&sv->sv_p2p_timer, NOA_STARTTIME_REFRESH);

         set_bit(SMD_FLAG_SV_P2P_PS_CHANGED, &sv->sv_flags);
         set_bit(SMD_FLAG_SH_UPDATED_SV, &sh->sh_flags);
         sv->sv_p2p_noa_index++;
         QUEUE_TXWORK(sh);
      }
      de_info(DE_TR_WIFI, "p2p_update_noa_from_nl duration=%d,interval=%d,start=%d,count=%d",
              req->conf.noa[0].duration,
              req->conf.noa[0].interval,
              req->conf.noa[0].starttime,
              req->conf.noa[0].count);
   }
}

void
p2p_update_opp_from_nl(struct smd_vif_priv *sv, nrx_buf_opp_t *opp)
{
   if (SV_IS_P2P(sv)) {
      struct smd_hw_priv             *sh  = hw_to_sh(sv->sv_hw);
      nrx_mlme_p2p_ps_set_conf_req_t *req = &sv->sv_p2p_ps_req;

      if (sv->sv_enable_beacon) {
         bool update = false;

         if( (opp->opp_enabled != -1)
          && (opp->opp_enabled != req->conf.opp.enabled) ) {
            req->conf.opp.enabled = opp->opp_enabled;
            update = true;
         }
         if(opp->ctwindow != -1) {
            req->conf.opp.ctwindow = opp->ctwindow;
            update = true;
         }

         if(update) {
            req->conf.type = NRX_MLME_P2P_PS_CONF_OPP;
            sv->sv_p2p_noa_index++;
            set_bit(SMD_FLAG_SV_P2P_PS_CHANGED, &sv->sv_flags);
            set_bit(SMD_FLAG_SH_UPDATED_SV, &sh->sh_flags);
            QUEUE_TXWORK(sh);
            de_info(DE_TR_WIFI, "p2p_update_opp_from_nl new opp values enabled=%d, ctwindow=%d",
                    req->conf.opp.enabled,
                    req->conf.opp.ctwindow);
         }
      }
   }
}

#endif  /* CONFIG_NL80211_TESTMODE */

void
p2p_send_ps_req(struct smd_vif_priv *sv)
{
   struct smd_transaction         *stx;
   nrx_mlme_p2p_ps_set_conf_req_t *req;

   req = smd_alloc_req_vif(sv, sizeof *req, HIC_MESSAGE_TYPE_MLME, NRX_MLME_P2P_PS_SET_CONF_REQ, &stx);
   *req = sv->sv_p2p_ps_req;
   smd_wait_transaction(stx);
   smd_txn_decref(stx);

   if (stx->st_rxskb) {
      if (stx->st_rxskb->len == sizeof(nrx_mlme_p2p_ps_set_conf_cfm_t)) {
         nrx_mlme_p2p_ps_set_conf_cfm_t *cfm = (nrx_mlme_p2p_ps_set_conf_cfm_t*)stx->st_rxskb->data;
         nrx_mlme_p2p_ps_conf_t * saved_conf = &sv->sv_p2p_ps_req.conf;

         /* This part is only relevant in GO mode */
         if(saved_conf->type & NRX_MLME_P2P_PS_CONF_NOA_1)
            sv->sv_p2p_noa_starttime[0] = cfm->starttime[0];
         if(saved_conf->type & NRX_MLME_P2P_PS_CONF_NOA_2)
            sv->sv_p2p_noa_starttime[1] = cfm->starttime[1];

         if ( (saved_conf->opp.enabled     != 0)
           || (saved_conf->noa[0].duration != 0)
           || (saved_conf->noa[1].duration != 0) ) {
            set_bit(SMD_FLAG_SV_P2P_PS_ENABLED, &sv->sv_flags);
         } else {
            clear_bit(SMD_FLAG_SV_P2P_PS_ENABLED, &sv->sv_flags);
         }
         smd_update_beacon(sv);
      } else {
         /* Incompatible firmware */
         de_error(DE_TR_WIFI, "Incompatible firmware, %u %zu", stx->st_rxskb->len, sizeof(nrx_mlme_p2p_ps_set_conf_cfm_t));
      }
   }
}

int
p2p_insert_noa_attrib(struct smd_vif_priv *sv, struct sk_buff *skb)
{
   int p2p_ps_found = 0;

   if (SV_IS_P2P(sv)
       && sv->sv_net_type == NRX_NET_TYPE_BSS_AP
       && test_bit(SMD_FLAG_SV_P2P_PS_ENABLED, &sv->sv_flags)
       ) {
      /* This applies only to GOs */
      struct ieee80211_mgmt *hdr = (struct ieee80211_mgmt*)skb->data;
      unsigned char *ie;
      unsigned char *first_p2p_ie = NULL;
      unsigned char *first_ie_after_p2p_ie = NULL;

      /* Find position to insert noa attrib */
      for (ie = hdr->u.beacon.variable; ie < (skb->data+skb->len); ie += 2+ie[1]) {
         if (skb->data + skb->len - ie < 2 + ie[1]) {
            de_info(DE_TR_WIFI, "malformed IE @%zd", ie - skb->data);
            break;
         }

         if (ie[0] == p2p_vendor_ie.element_id
             && memcmp(&ie[2], p2p_vendor_ie.oui, 4) == 0)
         {
            if (!first_p2p_ie) {
               first_p2p_ie = ie;
            }
         } else if (first_p2p_ie && !first_ie_after_p2p_ie) {
            first_ie_after_p2p_ie = ie;
            break;
         }
      }

      /* Insert noa attrib */
      if (first_p2p_ie) {
         nrx_mlme_p2p_ps_set_conf_req_t *req = &sv->sv_p2p_ps_req;

         size_t         size = sizeof(p2p_vendor_ie)+sizeof(struct m80211_p2p_attr_noa)
                               + ((req->conf.noa[0].duration != 0) ? sizeof(struct m80211_noa_desc) : 0);
         unsigned char              *new_data;
         struct m80211_p2p_attr_noa *noa_attrib;

         if (skb_tailroom(skb) < size) {
            /* Not enough tail room. Use head room instead. hw->extra_tx_headroom should be
             * configured so that there is space for this IE.
             */
            /* Alignment must be preserved */
            size_t aligned_size = ((size + 3) & (~3));
            skb_push(skb, aligned_size);
            memmove(skb->data, skb->data + aligned_size, skb->len - aligned_size);
            skb_trim(skb, skb->len - (aligned_size-size));
            new_data = skb_tail_pointer(skb) - size;
         } else {
            new_data = skb_put(skb, size);
         }

         if (!first_ie_after_p2p_ie) {
            /* There were no ies after the p2p ies. Just add noa attrib last */
         } else {
            /* There are other ies after the p2p ies. Move the data after the p2p ies */
            memmove(first_ie_after_p2p_ie + size, first_ie_after_p2p_ie, skb_tail_pointer(skb) - first_ie_after_p2p_ie - size);
            new_data = first_ie_after_p2p_ie;
         }

         *(struct ieee80211_vendor_ie *)new_data = p2p_vendor_ie;
         ((struct ieee80211_vendor_ie *)new_data)->len = size-2 /* 2 = 8-bit tag + 8-bit len */;
         noa_attrib = (struct m80211_p2p_attr_noa *)(new_data + sizeof(struct ieee80211_vendor_ie));
         noa_attrib->id = M80211_P2P_NOA_ATTR_ID;
         noa_attrib->length = 2;
         noa_attrib->index = sv->sv_p2p_noa_index;
         noa_attrib->ctwindow_opp_params = req->conf.opp.ctwindow;
         if (req->conf.opp.enabled)
         {
            noa_attrib->ctwindow_opp_params |= OPP_PS;
            p2p_ps_found |= NRX_MLME_P2P_PS_CONF_OPP;
         }
         if (req->conf.noa[0].duration != 0) {
            struct m80211_noa_desc *noa_desc = (struct m80211_noa_desc *)&noa_attrib[1];
            noa_desc->count_type = req->conf.noa[0].count;
            noa_desc->duration   = req->conf.noa[0].duration;
            noa_desc->interval   = req->conf.noa[0].interval;
            noa_desc->start_time = sv->sv_p2p_noa_starttime[0];
            noa_attrib->length += sizeof(struct m80211_noa_desc);
            p2p_ps_found |= NRX_MLME_P2P_PS_CONF_NOA_1;
         }
         de_debug(DE_TR_WIFI, "Inserted p2p noa attrib");
      } else {
         de_info(DE_TR_WIFI, "No p2p IEs in beacon/probe resp");
      }
   }
   return p2p_ps_found;
}

void
p2p_check_beacon_probersp(struct smd_vif_priv *sv, struct sk_buff *skb)
{
   struct ieee80211_mgmt *hdr = (struct ieee80211_mgmt*)skb->data;
   struct smd_hw_priv    *sh  = hw_to_sh(sv->sv_hw);
   unsigned char *ie;
   bool noa_attr_found = false;

   /* Loop through the IEs to find P2P IEs */
   for (ie = hdr->u.beacon.variable; ie < (skb->data+skb->len); ie += 2+ie[1]) {

      if (skb->data + skb->len - ie < 2 + ie[1]) {
         de_info(DE_TR_WIFI, "malformed IE @%zd", ie - skb->data);
         break;
      }
      if (ie[0] == p2p_vendor_ie.element_id
            && memcmp(&ie[2], p2p_vendor_ie.oui, 4) == 0)
      {
         /* P2P IE found. Loop through the attributes to find a NOA attribute */
         /* An attribute is like an IE, but the length field is 16 bit instead of 8 bit. */
         unsigned char *attr;
         for (attr = &ie[sizeof(struct ieee80211_vendor_ie)]; attr < &ie[ie[1]]; attr += 2+((struct m80211_p2p_attr_noa*)attr)->length) {
            struct m80211_p2p_attr_noa *noa_attrib = (struct m80211_p2p_attr_noa *)attr;
            if (noa_attrib->id == M80211_P2P_NOA_ATTR_ID) {
               if (noa_attrib->length < sizeof(struct m80211_p2p_attr_noa)-sizeof(u8)-sizeof(__le16)) {
                  de_info(DE_TR_WIFI, "malformed p2p attrib");
                  break;
               }

               noa_attr_found = true;

               if (sv->sv_p2p_noa_index != noa_attrib->index) {
                  /* NOA settings have changed */
                  nrx_mlme_p2p_ps_set_conf_req_t *req = &sv->sv_p2p_ps_req;
                  bool     opp      = !!(noa_attrib->ctwindow_opp_params & OPP_PS);
                  uint8_t  ctwindow =   (noa_attrib->ctwindow_opp_params & CTWINDOW_MASK);
                  unsigned num_noa = 0;
                  struct m80211_noa_desc *noa_desc;

                  sv->sv_p2p_noa_index = noa_attrib->index;

                  memset(req, 0, sizeof *req);

                  /* all p2p ps types must be updated, so that those that existed but
                     now do not, can be cancelled */
                  req->conf.type = NRX_MLME_P2P_PS_CONF_OPP
                                   | NRX_MLME_P2P_PS_CONF_NOA_1
                                   | NRX_MLME_P2P_PS_CONF_NOA_2;
                  /* update Opp PS */
                  req->conf.opp.ctwindow = ctwindow;
                  req->conf.opp.enabled = opp;
                  de_info(DE_TR_WIFI, "p2p_ps_conf opp=%d ctwin=%d", opp, ctwindow);

                  /* update NoA PS */
                  for (noa_desc = (struct m80211_noa_desc*)&noa_attrib[1];
                       noa_desc < (struct m80211_noa_desc*)(attr+sizeof(u8)+sizeof(__le16) + noa_attrib->length);
                       noa_desc += sizeof(struct m80211_noa_desc))
                  {
                     req->conf.noa[num_noa].duration  = noa_desc->duration;
                     req->conf.noa[num_noa].interval  = noa_desc->interval;
                     req->conf.noa[num_noa].starttime = noa_desc->start_time;
                     req->conf.noa[num_noa].count     = noa_desc->count_type;

                     de_info(DE_TR_WIFI, "p2p_ps_conf noa_num=%d dur=%d int=%d starttime=%d count=%d",
                                         num_noa, noa_desc->duration, noa_desc->interval, noa_desc->start_time, noa_desc->count_type);
                     if (++num_noa >= 2)
                        break;
                  }
                  /* clear the remaining (if any) NoA schedules */
                  for (; num_noa < 2; num_noa++) {
                     memset(&req->conf.noa[num_noa], 0, sizeof(nrx_p2p_ps_noa_t));
                  }
                  set_bit(SMD_FLAG_SV_P2P_PS_CHANGED, &sv->sv_flags);
                  set_bit(SMD_FLAG_SH_UPDATED_SV, &sh->sh_flags);
                  QUEUE_TXWORK(sh);
                  break;
               }
            }
         }
      }
   }

   if(!noa_attr_found && test_bit(SMD_FLAG_SV_P2P_PS_ENABLED, &sv->sv_flags)) {
      /* no NoA Attribute found while P2P PS was enabled. This means that the GO
       * cancelled its P2P PS schedules and we need to cancel it locally
       */
      p2p_ps_stop(sv);
   }
}

void p2p_ps_finished(struct ieee80211_vif *vif)
{
   struct smd_vif_priv *sv = NULL;

   if (vif) {
      sv = vif_to_sv(vif);

      p2p_ps_stop(sv);
   }
}

void
p2p_timeout_cb(de_timer_t *t)
{
   struct smd_vif_priv *sv = container_of(t, struct smd_vif_priv, sv_p2p_timer);

   nrx_mlme_p2p_ps_set_conf_req_t *req = &sv->sv_p2p_ps_req;
   struct smd_hw_priv             *sh  = hw_to_sh(sv->sv_hw);

   de_debug(DE_TR_WIFI, "NoA StartTime about to expire. Refreshing");

   /* restart p2p noa ps timer if noa is still active */
   de_timer_stop(&sv->sv_p2p_timer);

   if(req->conf.noa[0].duration != 0)
   {
      de_timer_start(&sv->sv_p2p_timer, NOA_STARTTIME_REFRESH);

      /* increment index to force update with saved values */
      sv->sv_p2p_noa_index++;
      req->conf.type = NRX_MLME_P2P_PS_CONF_NOA_1|NRX_MLME_P2P_PS_CONF_NOA_2;

      set_bit(SMD_FLAG_SV_P2P_PS_CHANGED, &sv->sv_flags);
      set_bit(SMD_FLAG_SH_UPDATED_SV, &sh->sh_flags);
      QUEUE_TXWORK(sh);
   }
}

static void p2p_ps_stop(struct smd_vif_priv *sv)
{
   if(test_bit(SMD_FLAG_SV_P2P_PS_ENABLED, &sv->sv_flags)
         && sv->sv_hw != NULL)
   {
      nrx_mlme_p2p_ps_set_conf_req_t *req = &sv->sv_p2p_ps_req;
      struct smd_hw_priv             *sh  = hw_to_sh(sv->sv_hw);

      de_info(DE_TR_WIFI, "stopping p2p ps");

      sv->sv_p2p_noa_index = -1;
      de_timer_stop(&sv->sv_p2p_timer);

      memset(req, 0, sizeof(nrx_mlme_p2p_ps_set_conf_req_t));
      req->conf.type = NRX_MLME_P2P_PS_CONF_OPP
         | NRX_MLME_P2P_PS_CONF_NOA_1
         | NRX_MLME_P2P_PS_CONF_NOA_2;

      set_bit(SMD_FLAG_SV_P2P_PS_CHANGED, &sv->sv_flags);
      set_bit(SMD_FLAG_SH_UPDATED_SV, &sh->sh_flags);
      QUEUE_TXWORK(sh);
   }
}
