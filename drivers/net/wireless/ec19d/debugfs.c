#include "smd_common.h"

void
smd_debugfs_init(struct smd_hw_priv *sh)
{
   sh->sh_debugfs_root = debugfs_create_dir("nanoradio",
                                            sh->sh_hw->wiphy->debugfsdir);
   if(IS_ERR(sh->sh_debugfs_root))
      sh->sh_debugfs_root = NULL;
}

void
smd_debugfs_remove(struct smd_hw_priv *sh)
{
   if(sh->sh_debugfs_root != NULL)
      debugfs_remove_recursive(sh->sh_debugfs_root);
   sh->sh_debugfs_root = NULL;
}

#ifdef NRX_CONFIG_ENABLE_STATISTICS

#define NRX_TX_STATUS_UNKNOWN 7

struct smd_stat_private {
   struct smd_hw_priv    *sp_sh;
   struct smd_statistics  sp_stat;
   size_t                 sp_len;
   char                   sp_buf[4000];
};

static void smd_stat_clear(struct smd_hw_priv *sh)
{
   LOCK_STATS(sh);
   memset(&sh->sh_statistics.stat_reset_time,
          0,
          sizeof(sh->sh_statistics)
          - offsetof(struct smd_statistics, stat_reset_time));
   do_gettimeofday(&sh->sh_statistics.stat_reset_time);
   UNLOCK_STATS(sh);
}

static unsigned int smd_stat_mac_rate_to_index(mac_rate_t rate)
{
   switch(MAC_MOD(rate)) {
      case MAC_MOD_QPSK:
         return 0 + MAC_SUBRATE(rate);
      case MAC_MOD_OFDM:
         return 4 + MAC_SUBRATE(rate);
      case MAC_MOD_HT:
         return 12 + MAC_SUBRATE(rate);
      case MAC_MOD_HT40:
         return 20 + MAC_SUBRATE(rate);
   }
   return 0; /* XXX */
}

#define S(FMT, ...)\
   snprintf(sp->sp_buf + strlen(sp->sp_buf),                  \
            sizeof(sp->sp_buf) - strlen(sp->sp_buf),          \
            FMT , ## __VA_ARGS__)

static void smd_stat_format_rates(struct smd_stat_private *sp,
                                  const char *prefix,
                                  uint32_t *count)
{
#define P(R) S("    %s " #R ": %u\n",                 \
               prefix,                                \
               count[smd_stat_mac_rate_to_index(R)])
   P(MAC_RATE_QPSK_1MBIT);
   P(MAC_RATE_QPSK_2MBIT);
   P(MAC_RATE_QPSK_5_5MBIT);
   P(MAC_RATE_QPSK_11MBIT);

   P(MAC_RATE_OFDM_6MBIT);
   P(MAC_RATE_OFDM_9MBIT);
   P(MAC_RATE_OFDM_12MBIT);
   P(MAC_RATE_OFDM_18MBIT);
   P(MAC_RATE_OFDM_24MBIT);
   P(MAC_RATE_OFDM_36MBIT);
   P(MAC_RATE_OFDM_48MBIT);
   P(MAC_RATE_OFDM_54MBIT);

   P(NRX_MAC_RATE_HT_MCS_0);
   P(NRX_MAC_RATE_HT_MCS_1);
   P(NRX_MAC_RATE_HT_MCS_2);
   P(NRX_MAC_RATE_HT_MCS_3);
   P(NRX_MAC_RATE_HT_MCS_4);
   P(NRX_MAC_RATE_HT_MCS_5);
   P(NRX_MAC_RATE_HT_MCS_6);
   P(NRX_MAC_RATE_HT_MCS_7);

#if 0
   P(NRX_MAC_RATE_HT40_MCS_0);
   P(NRX_MAC_RATE_HT40_MCS_1);
   P(NRX_MAC_RATE_HT40_MCS_2);
   P(NRX_MAC_RATE_HT40_MCS_3);
   P(NRX_MAC_RATE_HT40_MCS_4);
   P(NRX_MAC_RATE_HT40_MCS_5);
   P(NRX_MAC_RATE_HT40_MCS_6);
   P(NRX_MAC_RATE_HT40_MCS_7);
#endif
#undef P
}

static void smd_stat_format_statistics(struct smd_stat_private *sp)
{
   sp->sp_buf[0] = '\0';

   S("Statistics reset time: %ld.%06ld\n",
     sp->sp_stat.stat_reset_time.tv_sec,
     sp->sp_stat.stat_reset_time.tv_usec);
   S("TX Frames: %u\n",
     sp->sp_stat.stat_tx_frames);
   S("TX Bytes: %llu\n",
     sp->sp_stat.stat_tx_bytes);

   S("TX Success: %u\n",
     sp->sp_stat.stat_tx_status[NRX_TX_STATUS_OK]);
   S("TX Failure: %u\n",
     sp->sp_stat.stat_tx_status[NRX_TX_STATUS_UNDEL]
     + sp->sp_stat.stat_tx_status[NRX_TX_STATUS_BO_TIMEOUT]
     + sp->sp_stat.stat_tx_status[NRX_TX_STATUS_QUEUE_FLUSHED]
     + sp->sp_stat.stat_tx_status[NRX_TX_STATUS_URGENT_PRIO_OVERRIDE]
     + sp->sp_stat.stat_tx_status[NRX_TX_STATUS_STA_PS]
     + sp->sp_stat.stat_tx_status[NRX_TX_STATUS_MAX_RETRANSMISSIONS]
     + sp->sp_stat.stat_tx_status[NRX_TX_STATUS_UNKNOWN]);
#define P(X) S("    " #X ": %u\n",              \
               sp->sp_stat.stat_tx_status[(X)])
   P(NRX_TX_STATUS_UNDEL);
   P(NRX_TX_STATUS_BO_TIMEOUT);
   P(NRX_TX_STATUS_QUEUE_FLUSHED);
   P(NRX_TX_STATUS_URGENT_PRIO_OVERRIDE);
   P(NRX_TX_STATUS_STA_PS);
   P(NRX_TX_STATUS_MAX_RETRANSMISSIONS);
   P(NRX_TX_STATUS_UNKNOWN);
#undef P

   S("TX Tries 1: %u 2: %u 3: %u 4+: %u\n",
     sp->sp_stat.stat_tx_tries[0],
     sp->sp_stat.stat_tx_tries[1],
     sp->sp_stat.stat_tx_tries[2],
     sp->sp_stat.stat_tx_tries[3]);

   smd_stat_format_rates(sp, "TX", sp->sp_stat.stat_tx_rates);

   S("TX A-MPDU Requested: %u\n",
     sp->sp_stat.stat_tx_ampdu_req);
   S("TX A-MPDU BlockAcked: %u\n",
     sp->sp_stat.stat_tx_ampdu_back);

   S("RX Frames: %u\n",
     sp->sp_stat.stat_rx_frames);
   S("RX Bytes: %llu\n",
     sp->sp_stat.stat_rx_bytes);

   smd_stat_format_rates(sp, "RX", sp->sp_stat.stat_rx_rates);

   sp->sp_len = strlen(sp->sp_buf);
}

static int smd_stat_fop_open(struct inode *inode, struct file *file)
{
   struct smd_hw_priv *sh = (struct smd_hw_priv*)inode->i_private;
   struct smd_stat_private *sp;
   sp = kmalloc(sizeof(*sp), GFP_KERNEL);
   if(sp == NULL)
      return -ENOMEM;

   sp->sp_sh = sh;

   LOCK_STATS(sh);
   sp->sp_stat = sh->sh_statistics;
   UNLOCK_STATS(sh);

   smd_stat_format_statistics(sp);

   file->private_data = sp;

   return 0;
}

static int smd_stat_fop_release(struct inode *inode, struct file *filp)
{
   struct smd_stat_private *sp = (struct smd_stat_private*)filp->private_data;

   filp->private_data = NULL;
   kfree(sp);

   return 0;
}


static ssize_t
smd_stat_fop_read(struct file *filp,
                  char __user *buf,
                  size_t len,
                  loff_t *ppos)
{
   struct smd_stat_private *sp = (struct smd_stat_private*)filp->private_data;
   size_t bytes;

   if(*ppos >= 0 && *ppos <= sp->sp_len) {
      bytes = min(len, (size_t)(sp->sp_len - *ppos));
      if(copy_to_user(buf, &sp->sp_buf[*ppos], bytes))
         return -EFAULT;
      *ppos += bytes;
      return bytes;
   }
   return 0;
}

static ssize_t
smd_stat_fop_write(struct file *filp,
                   const char __user *buf,
                   size_t len,
                   loff_t *ppos)
{
   struct smd_stat_private *sp = (struct smd_stat_private*)filp->private_data;

   smd_stat_clear(sp->sp_sh);

   return len;
}

static const struct file_operations smd_stat_fops = {
   .open = smd_stat_fop_open,
   .read = smd_stat_fop_read,
   .write = smd_stat_fop_write,
   .release = smd_stat_fop_release,
   .llseek = default_llseek
};


void smd_stat_init(struct smd_hw_priv *sh)
{
   spin_lock_init(&sh->sh_statistics.stat_lock);
   smd_stat_clear(sh);

   if(sh->sh_debugfs_root != NULL)
      sh->sh_debugfs_stats = debugfs_create_file("packet_stat", 0666,
                                                 sh->sh_debugfs_root,
                                                 sh,
                                                 &smd_stat_fops);
}

static void smd_stat_update_rates(uint32_t *rate_table, mac_rate_t rate)
{
   unsigned int rate_index;

   rate_index = smd_stat_mac_rate_to_index(rate);

   rate_table[rate_index]++;
}

void smd_stat_update_tx_req_statistics(struct smd_hw_priv *sh,
                                       nrx_frame_tx_req_t *req,
                                       unsigned int frame_len)
{
   LOCK_STATS(sh);
   if(req->flags & NRX_TX_FLAG_AMPDU_ELIGIBLE)
      sh->sh_statistics.stat_tx_ampdu_req++;

   sh->sh_statistics.stat_tx_frames++;
   sh->sh_statistics.stat_tx_bytes += frame_len;
   UNLOCK_STATS(sh);
}

void smd_stat_update_tx_cfm_statistics(struct smd_hw_priv *sh,
                                       nrx_frame_tx_cfm_t *cfm)
{
   unsigned int tries;
   LOCK_STATS(sh);
   if(cfm->status <= NRX_TX_STATUS_MAX_RETRANSMISSIONS)
      sh->sh_statistics.stat_tx_status[cfm->status]++;
   else
      sh->sh_statistics.stat_tx_status[NRX_TX_STATUS_UNKNOWN]++;

   if(cfm->tries > 0) {
      if(cfm->tries >= ARRAY_SIZE(sh->sh_statistics.stat_tx_tries))
         tries = ARRAY_SIZE(sh->sh_statistics.stat_tx_tries) - 1;
      else
         tries = cfm->tries - 1;
      sh->sh_statistics.stat_tx_tries[tries]++;
   }

   if(cfm->status == NRX_TX_STATUS_OK) {
      if(cfm->tx_flags & NRX_TX_CFM_FLAG_BACK)
         sh->sh_statistics.stat_tx_ampdu_back++;
      smd_stat_update_rates(sh->sh_statistics.stat_tx_rates,
                            cfm->rate_used);
   }
   UNLOCK_STATS(sh);
}

void smd_stat_update_rx_ind_statistics(struct smd_hw_priv *sh,
                                       nrx_frame_rx_ind_t *ind,
                                       unsigned int frame_len)
{
   LOCK_STATS(sh);
   sh->sh_statistics.stat_rx_frames++;
   sh->sh_statistics.stat_rx_bytes += frame_len;
   smd_stat_update_rates(sh->sh_statistics.stat_rx_rates, ind->rate);
   UNLOCK_STATS(sh);
}

#endif /* NRX_CONFIG_ENABLE_STATISTICS */
