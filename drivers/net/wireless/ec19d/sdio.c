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

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#if defined(NRX_SDIO_THREAD_RX) || defined(NRX_SDIO_THREAD_TX)
#include <linux/kthread.h>
#endif
#include "nanocore.h"
#include "sdio_reg.h"
#include "transport_sdio.h"

#ifdef NRX_CONFIG_SDIO_BLOCK_MODE
#ifndef NRX_CONFIG_SDIO_BLOCK_SIZE
#define NRX_CONFIG_SDIO_BLOCK_SIZE 64
#endif

#if NRX_CONFIG_SDIO_BLOCK_SIZE != 16            \
   && NRX_CONFIG_SDIO_BLOCK_SIZE != 32          \
   && NRX_CONFIG_SDIO_BLOCK_SIZE != 64          \
   && NRX_CONFIG_SDIO_BLOCK_SIZE != 128         \
   && NRX_CONFIG_SDIO_BLOCK_SIZE != 256         \
   && NRX_CONFIG_SDIO_BLOCK_SIZE != 512
#error NRX_CONFIG_SDIO_BLOCK_SIZE must be power of two in range [16,512]
#endif
#if NRX_CONFIG_SDIO_BLOCK_SIZE > 64
#warning there are currently issues with NRX_CONFIG_SDIO_BLOCK_SIZE > 64
#endif

#define MINSIZE             ALIGN(1650, NRX_CONFIG_SDIO_BLOCK_SIZE)
#else /* NRX_CONFIG_SDIO_BLOCK_MODE */
#define MINSIZE             32
#endif /* NRX_CONFIG_SDIO_BLOCK_MODE */
#define MAXSIZE             4000  /* An AMSDU can be 3839 bytes */

#define MMC_CMD_RETRIES        3

struct smd_sdio_softc {
   struct sdio_func        *ss_func;
   struct nanoc_hw          ss_hw;
   unsigned long            ss_flags;
#define FLAG_SHUTDOWN    0
#define FLAG_IRQ_CLAIMED 1
   unsigned int             ss_irqcount;
   unsigned int             ss_rxcount;
#if defined(NRX_SDIO_THREAD_RX) || defined(NRX_SDIO_THREAD_TX)
   struct task_struct      *ss_thread;
   wait_queue_head_t        ss_waitq;
#endif
#ifdef NRX_SDIO_THREAD_TX
   struct sk_buff_head      ss_tx_queue;
#endif
#ifndef NRX_SDIO_THREAD_RX
   struct work_struct       ss_rxwork;
   struct workqueue_struct *ss_rxwq;
#endif
   bool                     ss_irq_allowed;
};

#define hw_to_ss(_hw) container_of(_hw, struct smd_sdio_softc, ss_hw)
static int tp_sdio_setreg(struct nanoc_hw *hw, unsigned addr, unsigned value);

static void
mmc_set_ios(struct mmc_host *host)
{
   struct mmc_ios *ios = &host->ios;

   de_debug(DE_TR_TP, "%s: "
            "clock %uHz "
            "busmode %u "
            "powermode %u "
            "cs %u "
            "Vdd %u "
            "width %u "
            "timing %u",
            mmc_hostname(host), ios->clock, ios->bus_mode,
            ios->power_mode, ios->chip_select, ios->vdd,
            ios->bus_width, ios->timing);

   host->ops->set_ios(host, ios);
}

static void
mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
   de_info(DE_TR_TP, "hz = %d", hz);
#ifdef CONFIG_MMC_CLKGATE
   {
      unsigned long flags;
      /* This is a bit ugly. If kernel implementation in drivers/mmc/core/core.c changes, this code may not work.
       * When clock gating is used ios.clock is set to zero
       * and clk_old contains the actual clock value. When clock is ungated
       * clk_old is copied to ios.clock.
       */
      spin_lock_irqsave(&host->clk_lock, flags);
      if (host->clk_gated) {
         host->clk_old = hz;
      } else {
         host->ios.clock = hz;
         mmc_set_ios(host);
      }
      spin_unlock_irqrestore(&host->clk_lock, flags);
   }
#else
   host->ios.clock = hz;
   mmc_set_ios(host);
#endif
}

static void
mmc_set_timing(struct mmc_host *host, unsigned int timing)
{
   host->ios.timing = timing;
   mmc_set_ios(host);
}

static void
mmc_set_bus_width(struct mmc_host *host, unsigned width)
{
   host->ios.bus_width = width;
   mmc_set_ios(host);
}

static int
wait_for_cmd(struct mmc_host *host, u32 opcode, u32 arg, unsigned flags, u32 *resp)
{
   struct mmc_command cmd;
   int ret;

   cmd.opcode = opcode;
   cmd.arg    = arg;
   cmd.flags  = flags;

   if ((ret = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES)) != 0) {
      return ret;
   }

   if (resp != NULL) {
      *resp = cmd.resp[0];
   }

   return 0;
}

static int
mmc_send_relative_addr(struct mmc_host *host, unsigned int *rca)
{
#if __GNUC__ == 4 && __GNUC_MINOR__ == 7
   /* bug in gcc 4.7.3 with -Os -Wuninitialized */
   u32 resp = 0;
#else
   u32 resp;
#endif
   int ret;

   ret = wait_for_cmd(host, SD_SEND_RELATIVE_ADDR, 0,
                      MMC_RSP_R6 | MMC_CMD_BCR, &resp);
   if (ret == 0) {
      *rca = resp >> 16;
   }

   return ret;
}

static int
mmc_select_card(struct mmc_card *card)
{
   return wait_for_cmd(card->host, MMC_SELECT_CARD, card->rca << 16,
                       MMC_RSP_R1 | MMC_CMD_AC, NULL);
}

static struct sk_buff *
recv_one_skb(struct smd_sdio_softc *ss)
{
   struct sk_buff *skb;
   size_t          len;
   int             ret;

   if (test_bit(FLAG_SHUTDOWN, &ss->ss_flags)) {
      return NULL;
   }

   if ((skb = dev_alloc_skb(MAXSIZE)) == NULL) {
      return NULL;
   }

   ret = sdio_readsb(ss->ss_func, skb_put(skb, MINSIZE), 0, MINSIZE);
   if (ret < 0) {
      de_error(DE_TR_TP, "sdio_readsb(%d) = %d", MINSIZE, ret);
      de_pdu_free(skb);
      return NULL;
   }

#define INVALID_PACKET(SKB)                     \
   ((SKB)->data[0] == (SKB)->data[1]                \
    && (SKB)->data[0] == (SKB)->data[2]             \
    && (SKB)->data[0] == (SKB)->data[3]             \
    && (SKB)->data[0] == (SKB)->data[4]             \
    && (SKB)->data[0] == (SKB)->data[5]             \
    && (SKB)->data[0] == (SKB)->data[6]             \
    && (SKB)->data[0] == (SKB)->data[7])
   //BUG_ON(INVALID_PACKET(skb));
   if(INVALID_PACKET(skb)) {
      nanoc_printbuf("INVALID", skb->data, skb->len);
      de_pdu_free(skb);
      return NULL;
   }
   len = *(uint16_t *)de_pdu_data(skb) + 2;

   if (len < 12 || len > MAXSIZE) {
      de_error(DE_TR_TP, "Invalid length %zu", len);
      nanoc_printbuf("INVALID", skb->data, skb->len);
      de_pdu_free(skb);
      return NULL;
   }

   if (len > MINSIZE) {
      len -= MINSIZE;
      ret = sdio_readsb(ss->ss_func, skb_put(skb, len), 0, len);
      if (ret < 0) {
         de_error(DE_TR_TP, "sdio_readsb(%zu) = %d", len, ret);
         de_pdu_free(skb);
         return NULL;
      }
   }

   return skb;
}

static int tx_one_packet(struct smd_sdio_softc *ss, struct sk_buff *skb)
{
   int ret;

   sdio_claim_host(ss->ss_func);
   ret = sdio_writesb(ss->ss_func, 0, skb->data, skb->len);
   sdio_release_host(ss->ss_func);
   nanoc_hw_send_complete(&ss->ss_hw, skb);
   return ret;
}

static void rx_one_packet(struct smd_sdio_softc *ss)
{
   struct sk_buff *skb;

   sdio_claim_host(ss->ss_func);
   skb = recv_one_skb(ss);
   sdio_release_host(ss->ss_func);
   ++ss->ss_rxcount;
   if (skb) {
      nanoc_hw_recv_handler(&ss->ss_hw, skb);
   }
}

#define RX_PENDING(SS) ((int)(SS)->ss_irqcount - (int)(SS)->ss_rxcount > 0)
#ifdef NRX_SDIO_THREAD_TX
#define TX_PENDING(SS) (!skb_queue_empty(&(SS)->ss_tx_queue))
#endif

#if defined(NRX_SDIO_THREAD_RX) || defined(NRX_SDIO_THREAD_TX)
static int ss_worker_thread(void *data)
{
   struct smd_sdio_softc *ss = data;

#ifdef NRX_SDIO_THREAD_PRIO
   set_user_nice(current, NRX_SDIO_THREAD_PRIO);
#endif

   while(true) {
      wait_event(ss->ss_waitq,
                 kthread_should_stop()
#ifdef NRX_SDIO_THREAD_RX
                 || RX_PENDING(ss)
#endif
#ifdef NRX_SDIO_THREAD_TX
                 || TX_PENDING(ss)
#endif
         );

      if(kthread_should_stop())
         break;

#ifdef NRX_SDIO_THREAD_RX
      if(RX_PENDING(ss)) {
         rx_one_packet(ss);
         continue;
      }
#endif

#ifdef NRX_SDIO_THREAD_TX
      if(TX_PENDING(ss)) {
         struct sk_buff *skb;
         if((skb = skb_dequeue(&ss->ss_tx_queue)) != NULL) {
            tx_one_packet(ss, skb);
         }
         continue;
      }
#endif
   }
   return 0;
}
#endif /* defined(NRX_SDIO_THREAD_RX) || defined(NRX_SDIO_THREAD_TX) */

#ifndef NRX_SDIO_THREAD_RX

static
NRX_WORKER_DECLARE(smd_sdio_rx_worker)
{
   NRX_WORKER_SETUP(struct smd_sdio_softc, ss, ss_rxwork);

   while (RX_PENDING(ss)) {
      rx_one_packet(ss);
   }
}

#endif /* !NRX_SDIO_THREAD_RX */

#ifdef NRX_CONFIG_HOST_GPIO_IRQ
static irqreturn_t
smd_gpio_irq(int irq_no, void* data)
{
   struct smd_sdio_softc *ss = data;

   if (ss->ss_irq_allowed) {
      ++ss->ss_irqcount;
#ifdef NRX_SDIO_THREAD_RX
      wake_up(&ss->ss_waitq);
#else
      queue_work(ss->ss_rxwq, &ss->ss_rxwork);
#endif
   }
   else {
      de_info(DE_TR_TP, "blocked irq");
   }

   return IRQ_HANDLED;
}
#else
static void
smd_sdio_irq(struct sdio_func *func)
{
   struct smd_sdio_softc *ss = dev_get_drvdata(&func->dev);
   if (ss->ss_irq_allowed) {
      int ret;

      de_info(DE_TR_TP, "NRX_CCCR_INTACK");
      sdio_f0_writeb(func, 2, NRX_CCCR_INTACK, &ret);
      if (ret != 0) {
         de_error(DE_TR_TP, "failed to ack interrupt\n");
      }

      ++ss->ss_irqcount;
#ifdef NRX_SDIO_THREAD_RX
      wake_up(&ss->ss_waitq);
#else
      queue_work(ss->ss_rxwq, &ss->ss_rxwork);
#endif
   }
   else {
      de_info(DE_TR_TP, "blocked irq");
   }
}
#endif

static int
tp_sdio_init(struct nanoc_hw *hw, unsigned flags)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   unsigned rca;
   int ret;

   de_info(DE_TR_TP, "flags=%#x", flags);

   sdio_claim_host(ss->ss_func);

   ret = mmc_send_relative_addr(ss->ss_func->card->host, &rca);
   if (ret != 0) {
      de_error(DE_TR_TP, "Failed to set relative address (%d)", ret);
      goto leave;
   }

   ss->ss_func->card->rca = rca;
   de_info(DE_TR_TP, "RCA=%u", rca);

   ret = mmc_select_card(ss->ss_func->card);
   if (ret != 0) {
      de_error(DE_TR_TP, "Failed to select card (%d)", ret);
      goto leave;
   }

   ret = sdio_enable_func(ss->ss_func);
   if (ret != 0) {
      de_error(DE_TR_TP, "Failed to enable function (%d)", ret);
      goto leave;
   }

   if (flags & NANOC_INIT_4BIT) {
      mmc_set_bus_width(ss->ss_func->card->host, MMC_BUS_WIDTH_4);
      tp_sdio_setreg(hw, NRX_CCCR_IF, 0x02);
      de_info(DE_TR_TP, "4-bit mode");
   } else {
      mmc_set_bus_width(ss->ss_func->card->host, MMC_BUS_WIDTH_1);
      tp_sdio_setreg(hw, NRX_CCCR_IF, 0x00);
      de_info(DE_TR_TP, "1-bit mode");
   }

   if (flags & NANOC_INIT_HS) {
      //mmc_card_set_highspeed(ss->ss_func->card);
      mmc_set_timing(ss->ss_func->card->host, MMC_TIMING_SD_HS);
      tp_sdio_setreg(hw, NRX_CCCR_HISPEED_EN, 0x01);
      de_info(DE_TR_TP, "HighSpeed");
   } else {
      tp_sdio_setreg(hw, NRX_CCCR_HISPEED_EN, 0x00);
      de_info(DE_TR_TP, "NormalSpeed");
   }

leave:
   sdio_release_host(ss->ss_func);
   return ret;
}

#ifdef NRX_CONFIG_HOST_GPIO_IRQ

static int
tp_sdio_start_rx(struct nanoc_hw *hw)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   int err;

   if (!test_and_set_bit(FLAG_IRQ_CLAIMED, &ss->ss_flags)) {
      int gpio_irq_num;

      de_info(DE_TR_TP, "Claim IRQ");

#ifdef CONFIG_GENERIC_GPIO
      err = gpio_request(NRX_CONFIG_HOST_GPIO_IRQ, "nrx GPIO");
      if (err) {
         de_error(DE_TR_TP, "gpio_request failed, err %d", err);
         return err;
      }
      err = gpio_direction_input(NRX_CONFIG_HOST_GPIO_IRQ);
      if (err) {
         de_error(DE_TR_TP, "gpio_direction_input failed, err %d", err);
         return err;
      }
#endif

      /* When turning on irq on Exynos board we got a "ghost" irq.
       * The ss_irq_allowed=false will mask that irq out.
       * The code is made generic for all platforms since it wont hurt
       */
      ss->ss_irq_allowed = false;
      gpio_irq_num = gpio_to_irq(NRX_CONFIG_HOST_GPIO_IRQ);
      de_info(DE_TR_TP, "gpio irq num %d", gpio_irq_num);
      err = request_irq(gpio_irq_num, smd_gpio_irq, 0x00000000, "nrx IRQ", ss);
      if (err) {
         de_error(DE_TR_TP, "request_irq failed, err %d", err);
         return err;
      }
      err = irq_set_irq_type(gpio_irq_num, IRQ_TYPE_EDGE_BOTH);
      if (err) {
         de_error(DE_TR_TP, "set_irq_type failed, err %d", err);
         return err;
      }
      msleep(50);
      ss->ss_irq_allowed = true;
   }

   return 0;
}

static int
tp_sdio_stop_rx(struct nanoc_hw *hw)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   int ret = 0;

   de_info(DE_TR_TP, "hw=%p", hw);
   if(test_and_clear_bit(FLAG_IRQ_CLAIMED, &ss->ss_flags)) {
      free_irq(gpio_to_irq(NRX_CONFIG_HOST_GPIO_IRQ), ss);
#ifdef CONFIG_GENERIC_GPIO
      gpio_free(NRX_CONFIG_HOST_GPIO_IRQ);
#endif
   }
   return ret;
}
#else
static int
tp_sdio_start_rx(struct nanoc_hw *hw)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   int ret = 0;

   sdio_claim_host(ss->ss_func);

   if (!test_and_set_bit(FLAG_IRQ_CLAIMED, &ss->ss_flags)) {
      de_info(DE_TR_TP, "Claim IRQ");

      /* When sdio_claim_irq is called, smd_sdio_irq will be called once
       * even though no irq has been issued from the chip.
       * Therefore we need to block that "ghost" irq.
       */
      ss->ss_irq_allowed = false;
      ret = sdio_claim_irq(ss->ss_func, smd_sdio_irq);
      if (ret != 0) {
         de_error(DE_TR_TP, "Failed to claim irq (%d)\n", ret);
      }
   }

   sdio_release_host(ss->ss_func);
   msleep(100);
   ss->ss_irq_allowed = true;

   return ret;
}

static int
tp_sdio_stop_rx(struct nanoc_hw *hw)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   int ret = 0;

   de_info(DE_TR_TP, "hw=%p", hw);
   sdio_claim_host(ss->ss_func);
   if(test_and_clear_bit(FLAG_IRQ_CLAIMED, &ss->ss_flags)) {
      ret = sdio_release_irq(ss->ss_func);
   }
   sdio_release_host(ss->ss_func);
   return ret;
}
#endif

static int
tp_sdio_getreg(struct nanoc_hw *hw, unsigned addr, unsigned *value)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   int ret;

   sdio_claim_host(ss->ss_func);
   *value = sdio_f0_readb(ss->ss_func, addr, &ret);
   sdio_release_host(ss->ss_func);
   /*de_info(DE_TR_TP, "[%#x] -> %#x", addr, *value);*/

   return ret;
}

static int
tp_sdio_setreg(struct nanoc_hw *hw, unsigned addr, unsigned value)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   int ret;

   /*de_info(DE_TR_TP, "[%#x] <- %#x", addr, value);*/
   sdio_claim_host(ss->ss_func);
   sdio_f0_writeb(ss->ss_func, value, addr, &ret);
   sdio_release_host(ss->ss_func);
   de_info(DE_TR_TP, "setreg(%#x, %#x) = %d", addr, value, ret);

   return ret;
}

#ifdef NRX_CONFIG_SDIO_BLOCK_MODE
static void set_block_mode(struct smd_sdio_softc *ss, bool enable)
{
   if(enable) {
      sdio_claim_host(ss->ss_func);
      ss->ss_func->card->cccr.multi_block = 1;
      sdio_set_block_size(ss->ss_func, NRX_CONFIG_SDIO_BLOCK_SIZE);
      sdio_release_host(ss->ss_func);
   } else {
      ss->ss_func->card->cccr.multi_block = 0;
   }
}
#endif


static int
tp_sdio_send(struct nanoc_hw *hw, de_pdu_t *skb)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   struct sdio_func *func = ss->ss_func;
   unsigned aligned_size;
#ifdef NRX_CONFIG_SDIO_BLOCK_MODE
   if (!ss->ss_func->card->cccr.multi_block) {
      set_block_mode(ss, true);
   }
#endif

   de_info(DE_TR_TP, "len=%d", skb->len);

   aligned_size = sdio_align_size(func, de_pdu_size(skb));
   if (aligned_size != de_pdu_size(skb)) {
      de_error(DE_TR_TP, "Bad size alignment (%zu, need %u)", de_pdu_size(skb), aligned_size);
      return -EINVAL;
   }

#ifdef NRX_SDIO_THREAD_TX
   skb_queue_tail(&ss->ss_tx_queue, skb);
   wake_up(&ss->ss_waitq);
   return 0;
#else
   return tx_one_packet(ss, skb);
#endif
}

static int
tp_sdio_download(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   struct sdio_func *func = ss->ss_func;
   unsigned aligned_size;
   int ret;

   de_info(DE_TR_TP, "len=%zu", de_pdu_size(pdu));

#ifdef NRX_CONFIG_SDIO_BLOCK_MODE
   if (ss->ss_func->card->cccr.multi_block) {
      set_block_mode(ss, false);
   }
#endif
   aligned_size = sdio_align_size(func, de_pdu_size(pdu));
   if (aligned_size != de_pdu_size(pdu)) {
      de_error(DE_TR_TP, "Bad size alignment (%zu, need %u)", de_pdu_size(pdu), aligned_size);
   }

   if ((de_pdu_size(pdu) & 0x3) != 0) {
      de_warn(DE_TR_TP, "Skip bad aligned pdu (%zu)", de_pdu_size(pdu));
   }

   sdio_claim_host(func);
   ret = sdio_writesb(ss->ss_func, 0, de_pdu_data(pdu), de_pdu_size(pdu));
   sdio_release_host(func);

   return ret;
}

static int
tp_sdio_wakeup(struct nanoc_hw *hw, unsigned wakeup)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   struct sdio_func *func = ss->ss_func;
   int ret;

   sdio_claim_host(func);
   sdio_f0_writeb(func, wakeup & 1, NRX_CCCR_WAKEUP, &ret);
   sdio_release_host(func);
   
   de_debug(DE_TR_TP, "sent wakeup to chip");

   return ret;
}

static int
tp_sdio_shutdown(struct nanoc_hw *hw, unsigned asserted)
{
   return 0;
}

static int
tp_sdio_power(struct nanoc_hw *hw, unsigned on)
{
   return 0;
}


static unsigned int mmc_get_clock(struct mmc_host *host)
{
   return host->ios.clock;
}

static int
tp_sdio_getspeed(struct nanoc_hw *hw, unsigned int *speed)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   struct sdio_func *func = ss->ss_func;

   sdio_claim_host(func);
   *speed = mmc_get_clock(func->card->host);
   sdio_release_host(func);

   return 0;
}

static int
tp_sdio_setspeed(struct nanoc_hw *hw, unsigned int speed)
{
   struct smd_sdio_softc *ss = hw_to_ss(hw);
   struct sdio_func *func = ss->ss_func;

   sdio_claim_host(func);
   mmc_set_clock(func->card->host, speed);
   sdio_release_host(func);

   return 0;
}

static const struct nanoc_hw_ops sdio_transport = {
   .hw_init      = tp_sdio_init,
   .hw_start_rx  = tp_sdio_start_rx,
   .hw_stop_rx   = tp_sdio_stop_rx,
   .hw_send      = tp_sdio_send,
   .hw_download  = tp_sdio_download,
   .hw_getreg    = tp_sdio_getreg,
   .hw_setreg    = tp_sdio_setreg,
   .hw_wakeup    = tp_sdio_wakeup,
   .hw_shutdown  = tp_sdio_shutdown,
   .hw_power     = tp_sdio_power,
   .hw_getspeed  = tp_sdio_getspeed,
   .hw_setspeed  = tp_sdio_setspeed
};


static int tp_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
   struct smd_sdio_softc *ss;

   de_info(DE_TR_TP, "class=%#x vendor=%#x device=%#x", func->class, func->vendor, func->device);

   if (func->vendor != SDIO_VENDOR_ID_NANORADIO)
      return -ENODEV;

   if((func->device & DEVICE_ID_CHIP_MASK) == SDIO_DEVICE_ID_NANORADIO_NRX615
      && func->num > 1) {
      /* function 2 is not used on nrx615 */
      return -ENODEV;
   }

   /* NRX900 has several functions and therefore tp_sdio_probe will be called several times */
   if (func->num > 1) {
      return 0;
   }

#ifdef MMC_QUIRK_BROKEN_CLK_GATING
   /* we don't need clock running at all times */
   func->card->quirks &= ~MMC_QUIRK_BROKEN_CLK_GATING;
#endif

   /* allows us to write to non-vendor CCCR registers */
   func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

   if (func->card->cccr.multi_block) {
      de_info(DE_TR_TP, "Disable block mode cap");
      func->card->cccr.multi_block = 0;
   }
   func->max_blksize = 512;

   ss = kzalloc(sizeof(*ss), GFP_KERNEL);
   if (ss == NULL) {
      de_error(DE_TR_TP, "Failed to allocate memory");
      return -ENOMEM;
   }

   ss->ss_func = func;

#ifdef NRX_SDIO_THREAD_TX
   skb_queue_head_init(&ss->ss_tx_queue);
#endif
#if defined(NRX_SDIO_THREAD_RX) || defined(NRX_SDIO_THREAD_TX)
   init_waitqueue_head(&ss->ss_waitq);

   ss->ss_thread = kthread_run(ss_worker_thread, ss, "nrx_sdio_worker");
#endif
#ifndef NRX_SDIO_THREAD_RX
   ss->ss_rxwq = alloc_workqueue("nano_wifi_sdio",
                                 WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
   NRX_WORKER_INIT(ss, ss_rxwork, smd_sdio_rx_worker);
#endif
   sdio_set_drvdata(func, ss);
   ss->ss_hw.ops                 = &sdio_transport;
   ss->ss_hw.dev                 = &func->dev;
   ss->ss_hw.min_sz_to_host      = MINSIZE;
   ss->ss_hw.sz_modulo_to_host   = 4;
#ifdef NRX_CONFIG_SDIO_BLOCK_MODE
   ss->ss_hw.sz_modulo_from_host = NRX_CONFIG_SDIO_BLOCK_SIZE;
#else
   ss->ss_hw.sz_modulo_from_host = 4;
#endif
#ifdef NRX_CONFIG_TARGET_GPIO_IRQ
   ss->ss_hw.irq_gpio_id         = NRX_IRQ_GPIO_ID(NRX_CONFIG_TARGET_GPIO_IRQ);
#else
   ss->ss_hw.irq_gpio_id         = NRX_IRQ_SDIO_DAT1;
#endif
   ss->ss_hw.wakeup_gpio_id      = 0xFF;
   ss->ss_hw.min_irq_interval    = 0;

   nanoc_hw_register(&ss->ss_hw);

   return 0;
}

static void tp_sdio_remove(struct sdio_func *func)
{
   struct smd_sdio_softc *ss = sdio_get_drvdata(func);

   if (ss == NULL) {
      de_warn(DE_TR_TP, "Weird error, drvdata is NULL.");
      return;
   }
   set_bit(FLAG_SHUTDOWN, &ss->ss_flags);
   tp_sdio_stop_rx(&ss->ss_hw);
#if defined(NRX_SDIO_THREAD_RX) || defined(NRX_SDIO_THREAD_TX)
   kthread_stop(ss->ss_thread);
#endif
#ifndef NRX_SDIO_THREAD_RX
   destroy_workqueue(ss->ss_rxwq);
#endif
   sdio_claim_host(func);
   sdio_disable_func(func);
   sdio_release_host(func);
   nanoc_hw_unregister(&ss->ss_hw);
   sdio_set_drvdata(func, NULL);
   kfree(ss);
}

static const struct sdio_device_id tp_sdio_id_table[] = {
   { SDIO_DEVICE(SDIO_VENDOR_ID_NANORADIO, SDIO_ANY_ID) },
   { .class = 0 }
};

MODULE_DEVICE_TABLE(sdio, tp_sdio_id_table);

static struct sdio_driver tp_sdio_driver = {
   .probe    = tp_sdio_probe,
   .remove   = tp_sdio_remove,
   .name     = "smd_sdio",
   .id_table = tp_sdio_id_table,
};

int __init
smd_sdio_init(void)
{
   return sdio_register_driver(&tp_sdio_driver);
}

void __exit
smd_sdio_exit(void)
{
   sdio_unregister_driver(&tp_sdio_driver);
}
