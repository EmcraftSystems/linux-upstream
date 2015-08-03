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

static int mib_result_to_sys_error(nrx_mib_result_t result)
{
   int ret = 0;

   switch(result) {
      case MIB_RESULT_OK:
         break;
      case MIB_RESULT_NO_SUCH_OBJECT:
         ret = -ENOENT;
         break;
      case MIB_RESULT_SET_NOT_ALLOWED:
      case MIB_RESULT_GET_NOT_ALLOWED:
         ret = -EPERM;
         break;
      case MIB_RESULT_SIZE_ERROR:
      case MIB_RESULT_OBJECT_NOT_A_LEAF:
      case MIB_RESULT_SET_FAILED:
      case MIB_RESULT_GET_FAILED:
      case MIB_RESULT_INTERNAL_ERROR:
      case MIB_RESULT_MEM_REGION_INVALIDATED:
      case MIB_RESULT_INVALID_NETIF:
      case MIB_RESULT_TRIGGER_SETUP_FAILED:
      case MIB_RESULT_TRIGGER_NOT_FOUND:
      default:
         ret = -EINVAL;
         break;
   }

   return ret;
}

int smd_mib_set(struct smd_hw_priv *sh,
                struct smd_vif_priv *sv,
                nrx_mib_id_t id,
                const void *data,
                size_t size)
{
   nrx_mib_set_req_t *req;
   nrx_mib_set_cfm_t *cfm;
   struct smd_transaction *stx;
   int ret;

   if(sv != NULL) {
      req = smd_alloc_req_vif(sv, sizeof(*req) + size,
                              HIC_MESSAGE_TYPE_MIB,
                              NRX_MIB_SET_REQ,
                              &stx);
   } else {
      req = smd_alloc_req(sh, sizeof(*req) + size,
                          HIC_MESSAGE_TYPE_MIB,
                          NRX_MIB_SET_REQ,
                          &stx);
   }
   if(req == NULL)
      return -ENOMEM;

   req->id = id;
   req->size = size;
   memcpy(&req[1], data, size);

   ret = smd_wait_transaction(stx);
   if(ret == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      ret = mib_result_to_sys_error(cfm->result);
   }
   smd_txn_decref(stx);

   return ret;
}


int
smd_sv_set_group_filter(struct smd_vif_priv *sv,
                        const group_addr_filter_t *filter)
{
   return smd_mib_set(sv_to_sh(sv), sv,
                      NRX_MIB_ID_DOT11_PRIVATE_RX_FRAME_FILTER_GROUP_ADDRESS,
                      filter,
                      sizeof(*filter));
}

int
smd_sv_set_arp_filter(struct smd_vif_priv *sv,
                      const arp_addr_filter_t *filter)
{
   return smd_mib_set(sv_to_sh(sv), sv,
                      NRX_MIB_ID_DOT11_PRIVATE_ARP_FILTER,
                      filter,
                      sizeof(*filter));
}

int
smd_set_tx_power(struct smd_hw_priv *sh, int32_t dbm)
{
   struct {
      uint8_t    qpsk_index;
      uint8_t    ofdm_index;
   } value;
   int32_t tmp_index;

   tmp_index = 21 - dbm;
   if(tmp_index < 0)
      tmp_index = 0;
   if(tmp_index > 19)
      tmp_index = 19;
   value.qpsk_index = tmp_index;

   tmp_index = 17 - dbm;
   if(tmp_index < 0)
      tmp_index = 0;
   if(tmp_index > 19)
      tmp_index = 19;
   value.ofdm_index = tmp_index;

   return smd_mib_set(sh, NULL,
                      NRX_MIB_ID_DOT11_PRIVATE_POWER_INDEX,
                      &value,
                      sizeof(value));

}

int
smd_set_fcc(struct smd_hw_priv *sh, bool fcc)
{
   uint32_t value = (fcc) ? 2 : 0;
   return smd_mib_set(sh, NULL,
                      NRX_MIB_ID_REGULATION_REGION,
                      &value,
                      sizeof(value));
}

int
smd_mib_trigger_set(struct smd_vif_priv *sv,
                    nrx_mib_id_t mib_id,
                    uint32_t trigger_id,
                    bool rising_trigger,
                    uint32_t interval_us,
                    uint32_t level)
{
   struct smd_transaction *stx;
   nrx_mib_set_trigger_req_t *req;
   nrx_mib_set_trigger_cfm_t *cfm;
   int ret;

   req = smd_alloc_req_vif(sv, sizeof *req, HIC_MESSAGE_TYPE_MIB, NRX_MIB_SET_TRIGGER_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   memset(req, 0, sizeof(*req));
   req->id = mib_id;
   req->trigger_id = trigger_id;
   req->supv_interval = interval_us;
   req->level = level;
   if(rising_trigger)
      req->event = MIB_TRIG_EVENT_RISING;
   else
      req->event = MIB_TRIG_EVENT_FALLING;
   req->trig_mode = MIB_TRIG_MODE_CONT; /** @sa mib_trigger_mode_t */
   req->event_count = 1;

   de_debug(DE_TR_WIFI, "Sending nrx_mib_set_trigger_req");
   ret = smd_wait_transaction(stx);
   if(ret == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      ret = mib_result_to_sys_error(cfm->result);
      if(ret == 0 && cfm->trigger_id != trigger_id) {
         de_error(DE_TR_WIFI, "got trigger cfm with bad trigger id");
         ret = -EIO;
      }
   }
   smd_txn_decref(stx);

   return ret;
}

int
smd_mib_trigger_remove(struct smd_vif_priv *sv,
                       uint32_t trigger_id)
{
   struct smd_transaction *stx;
   nrx_mib_remove_trigger_req_t *req;
   nrx_mib_remove_trigger_cfm_t *cfm;
   int ret;

   req = smd_alloc_req_vif(sv, sizeof *req, HIC_MESSAGE_TYPE_MIB, NRX_MIB_REMOVE_TRIGGER_REQ, &stx);
   if(req == NULL)
      return -ENOMEM;

   memset(req, 0, sizeof(*req));
   req->trigger_id = trigger_id;

   ret = smd_wait_transaction(stx);
   if(ret == 0) {
      cfm = de_pdu_data(stx->st_rxskb);
      ret = mib_result_to_sys_error(cfm->result);
      if(ret == 0 && cfm->trigger_id != trigger_id) {
         de_error(DE_TR_WIFI, "got trigger cfm with bad trigger id");
         ret = -EIO;
      }
   }
   smd_txn_decref(stx);

   return ret;
}
