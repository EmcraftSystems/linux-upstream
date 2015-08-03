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

static uint16_t
smd_get_transid(struct smd_hw_priv *sh)
{
   int ret;

   do {
      ret = atomic_inc_return(&sh->sh_transid);
      ret = (ret << 1) & 0xFFFF;
      /* Don't use trans ID zero */
   } while(ret == 0);

   return ret;
}

void
smd_add_hic_header(struct smd_transaction *stx, uint8_t type, uint8_t id)
{
   struct sk_buff    *skb = stx->st_txskb;
   nrx_hic_message_t *hdr = (nrx_hic_message_t *)skb_push(skb, sizeof *hdr);

   hdr->length    = skb->len;
   hdr->type      = type;
   hdr->id        = id;
   hdr->flags     = 0;
   hdr->net_id    = stx->st_netid;
   hdr->trans_id  = stx->st_transid;
   hdr->tx_window_non_wifi_data  = 0;
   hdr->tx_window = 0;
}

static void smd_txn_link(struct smd_transaction *stx)
{
   unsigned int pending_trans;

   HWLOCK(stx->st_sh);
   list_add_tail(&stx->st_list, &stx->st_sh->sh_transactions);
   pending_trans = ++stx->st_sh->sh_num_stx;
   HWUNLOCK(stx->st_sh);

   de_debug(DE_TR_WIFI, "pending transactions=%u, tid=%u",
            pending_trans, stx->st_transid);
}

static void smd_txn_unlink(struct smd_transaction *stx)
{
   unsigned int pending_trans;

   HWLOCK(stx->st_sh);
   list_del(&stx->st_list);
   BUG_ON(stx->st_sh->sh_num_stx == 0);
   pending_trans = --stx->st_sh->sh_num_stx;
   HWUNLOCK(stx->st_sh);

   de_debug(DE_TR_WIFI, "pending transactions=%u, tid=%u",
            pending_trans, stx->st_transid);
}

/* clones a transaction for purposes of resending a packet to
 * firmware */
struct smd_transaction*
smd_clone_transaction(struct smd_transaction *stx_orig)
{
   struct smd_transaction *stx;
   struct sk_buff *skb;

   if((stx_orig->st_transid & 1) != 0)
   {
      de_error(DE_TR_OS, "cloning of a cloned transaction is not currently supported");
      return NULL;
   }

   skb = skb_clone(stx_orig->st_txskborg, GFP_KERNEL);
   if(skb == NULL)
      return NULL;

   if ((stx = kzalloc(sizeof(*stx), GFP_KERNEL)) == NULL) {
      de_error(DE_TR_OS, "Out of memory");
      dev_kfree_skb(skb);
      return NULL;
   }

   INIT_LIST_HEAD(&stx->st_list);
   kref_init(&stx->st_ref); /* reference counter is 1 */
   init_completion(&stx->st_completion);

   stx->st_txskb       = skb;
   stx->st_sh          = stx_orig->st_sh;
   stx->st_transid     = stx_orig->st_transid | 1;
   stx->st_netid       = stx_orig->st_netid;
   stx->st_queue       = stx_orig->st_queue;
   stx->st_tx_complete = stx_orig->st_tx_complete;
   stx->st_tx_confirm  = stx_orig->st_tx_confirm;

   smd_txn_link(stx);

   TRACE_STX(stx, "ALLOC");

   return stx;
}

struct smd_transaction *
smd_create_transaction(struct smd_hw_priv *sh, unsigned netid)
{
   struct smd_transaction *stx;


   if(!test_bit(SMD_FLAG_SH_CHIP_UP, &sh->sh_flags))
      return NULL;

   if ((stx = kzalloc(sizeof(*stx), GFP_KERNEL)) == NULL) {
      de_error(DE_TR_OS, "Out of memory");
      return NULL;
   }
   stx->st_sh          = sh;
   stx->st_transid     = smd_get_transid(sh);
   stx->st_netid       = netid;
   stx->st_queue       = NANOC_QCMD;
   INIT_LIST_HEAD(&stx->st_list);
   kref_init(&stx->st_ref); /* reference counter is 1 */
   init_completion(&stx->st_completion);

   TRACE_STX(stx, "ALLOC");

   smd_txn_link(stx);

   return stx;
}

void
smd_release_transaction(struct kref *ref)
{
   struct smd_transaction *stx = container_of(ref, struct smd_transaction, st_ref);

   TRACE_STX(stx, "RELEASE");

   smd_txn_unlink(stx);

   if (stx->st_txskb != NULL)
      dev_kfree_skb(stx->st_txskb);
   if (stx->st_txskborg != NULL)
      FREE_TXSKB(stx->st_sh, stx->st_txskborg);
   if (stx->st_rxskb != NULL)
      dev_kfree_skb(stx->st_rxskb);

   kfree(stx);
}

void
smd_fail_transactions(struct smd_hw_priv *sh)
{
   struct smd_transaction *stx, *next;
   int pending;

   /* this competes with tx_confirm called by smd_rx_packet. Whoever gets here first
      will nullify stx->st_tx_confirm and call the saved ptr
    */
   HWLOCK(sh);
   list_for_each_entry_safe(stx, next, &sh->sh_transactions, st_list) {
      smd_completion_func_t tx_confirm = stx->st_tx_confirm;
      smd_txn_incref(stx);
      stx->st_restart = true;
      if (tx_confirm != NULL) {
         stx->st_tx_confirm = NULL;
         HWUNLOCK(sh);  //unlock for callback
         tx_confirm(stx);

         de_info(DE_TR_WIFI, "cleanup of stx=%p queue=%d tid=%d",
                 stx,
                 stx->st_queue,
                 stx->st_transid);
      } else {
         HWUNLOCK(sh);
      }
      smd_txn_decref(stx);
      HWLOCK(sh);    //relock for next iteration
   }
   pending = sh->sh_num_stx;
   HWUNLOCK(sh);

   de_info(DE_TR_WIFI, "after cleanup of sh=%p: pending transactions=%d", sh, pending);
}

int
smd_wait_all_transactions(struct smd_hw_priv *sh)
{
   unsigned i;

   de_info(DE_TR_WIFI, "sh=%p, pending=%zu", sh, sh->sh_num_stx);
   for (i = 0; i < 100; i++) {
      HWLOCK(sh);
      if (list_empty(&sh->sh_transactions)) {
         HWUNLOCK(sh);
         return 0;
      } else {
         HWUNLOCK(sh);
      }
      msleep(10);
   }
   return -1;
}

struct smd_transaction*
smd_txn_getref(struct smd_hw_priv *sh, uint16_t transid)
{
   struct smd_transaction *stx, *result = NULL;

   HWLOCK(sh);
   list_for_each_entry(stx, &sh->sh_transactions, st_list) {
      if (stx->st_transid == transid) {
         result = stx;
         /* incref so that caller may safely play with transaction */
         smd_txn_incref(stx);
         break;
      }
   }
   HWUNLOCK(sh);

   if(result)
      TRACE_STX(result, "LOOKUP");
   return result;
}

int
smd_wait_transaction(struct smd_transaction *stx)
{
   struct smd_hw_priv *sh = stx->st_sh;
   int ret = -1;

   if (test_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
      de_warn(DE_TR_WIFI, "RESTARTING stx=%p", stx);
      return -EAGAIN;
   }

   /* Aquire sh_cmd_sem to guarantee that transactions are serialized */
   if ((ret = down_interruptible(&sh->sh_cmd_sem)) < 0) {
      de_warn(DE_TR_WIFI, "down_interruptible = %d", ret);
      return ret;
   }

   /*
    * If link is down, block and wait for it to come up
    */
   if (!test_bit(SMD_FLAG_SH_LINKUP_CMD, &sh->sh_flags)) {
      de_info(DE_TR_WIFI, "Wait for LINKUP stx=%p", stx);
      ret = wait_event_interruptible_timeout(sh->sh_link_wait,
                                             (test_bit(SMD_FLAG_SH_LINKUP_CMD, &sh->sh_flags) ||
                                              test_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)),
                                             msecs_to_jiffies(500));
      if (ret == 0) {
         de_warn(DE_TR_WIFI, "LINK_UP TIMEOUT");
         if (!test_and_set_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
            nanoc_req_coredump((struct nanoc_context *)sh->sh_hw);
         }
         ret = -EAGAIN;
         goto leave;
      } else if (ret < 0) {
         de_warn(DE_TR_WIFI, "wait_event_interruptible_timeout = %d", ret);
         goto leave;
      }

      if (test_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
         de_warn(DE_TR_WIFI, "RESTARTING stx=%p", stx);
         ret = -EAGAIN;
         goto leave;
      }
   }

   smd_txn_incref(stx); /* Keep transaction until send complete */
   if ((ret = nanoc_send_cmd(stx->st_netid, stx->st_txskb)) == 0) {
      if (wait_for_completion_timeout(&stx->st_completion, msecs_to_jiffies(100)) == 0) {
         de_warn(DE_TR_WIFI, "COMMAND TIMEOUT tid=%d (restart=%d), flags=%lx", stx->st_transid, stx->st_restart, sh->sh_flags);

         if (!test_and_set_bit(SMD_FLAG_SH_RESTARTING, &sh->sh_flags)) {
            nanoc_req_coredump((struct nanoc_context *)sh->sh_hw);
         }
         ret = -ETIMEDOUT;
         goto leave;
      } else if(stx->st_restart) {
         ret = -EIO;
      }
   } else {
      de_warn(DE_TR_WIFI, "nanoc_send_cmd = %d", ret);
      /* never sent; stx_tx_complete never ran.
         return the ref taken before nanoc_send_cmd
       */
      smd_txn_decref(stx);
   }

leave:
   up(&sh->sh_cmd_sem);

   return ret;
}

static inline void *
alloc_req(struct smd_transaction *stx, size_t size,
          nrx_hic_message_type_t hictype,
          nrx_hic_message_id_t hicid)
{
   void *req = NULL;

   if ((stx->st_txskb = de_pdu_alloc(size)) == NULL) {
      smd_txn_decref(stx); /* Free transaction */
   } else {
      stx->st_tx_complete = smd_command_complete;
      stx->st_tx_confirm  = smd_command_confirm;
      /* Reserve space for message payload */
      req = skb_put(stx->st_txskb, size);
      /* Prepend HIC header */
      smd_add_hic_header(stx, hictype, hicid);
   }
   return req;
}

void *
smd_alloc_req_vif(struct smd_vif_priv *sv,
                  size_t size,
                  nrx_hic_message_type_t hictype,
                  nrx_hic_message_id_t hicid,
                  struct smd_transaction **p_stx)
{
   void                   *req = NULL;
   struct smd_transaction *stx;

   if (sv->sv_hw
         && (stx = smd_create_transaction(sv->sv_hw->priv, sv->sv_net_id)) != NULL) {
      if ((req = alloc_req(stx, size, hictype, hicid)) != NULL) {
         *p_stx = stx;
      }
   }

   return req;
}

void *
smd_alloc_req(struct smd_hw_priv *sh,
              size_t size,
              nrx_hic_message_type_t hictype,
              nrx_hic_message_id_t hicid,
              struct smd_transaction **p_stx)
{
   void                   *req = NULL;
   struct smd_transaction *stx;

   if ((stx = smd_create_transaction(sh, sh->sh_dummy_netid)) != NULL) {
      if ((req = alloc_req(stx, size, hictype, hicid)) != NULL) {
         *p_stx = stx;
      }
   }

   return req;
}
