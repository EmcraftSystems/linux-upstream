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

/*
 * Memory dump sequence on nrx701 and nrx600.
 *
 * There are hw registers (bm_reg) that controls DMA to and from the chip,
 * these registers have a pointer (xmit_descr) to a DMA chain, and a control
 * word. When the start bit is set in the control register, the hardware will
 * traverse the DMA chain and transfer any data it points to.
 *
 * The DMA chain is linked list (of bm descriptors) with pointers and sizes to
 * the data to transmit.
 *
 * When firmware crashes (or is requested to crash), it sends an
 * SCB_ERROR_IND message to the host. In order to synchronise firmware with the
 * host (there may be outstanding transfers scheduled), firmware enters a
 * special loop that only waits for a special magic pattern. This is sent with
 * SCB_ERROR_REQ, and a response with SCB_ERROR_CFM is produced by firmware.
 *
 * At this point firmware has setup the RX (from the chip point ot view) bm
 * register to accept writes to any memory location with a special message
 * format. The driver is responsible for setting up the registers for TX.
 *
 * Received from firmware in the SCB_ERROR_IND are two memory locations, one
 * (txDescriptorAddress) that is used hold descriptors for memory reads, the
 * other (signalHostAttentionAddress) is used to trigger interrupts.
 *
 * We use this so that the first descriptor transmits two bytes from the length
 * fields of the seconds descriptor, and the second descriptor transmits that
 * amount of bytes from a specified address.
 *
 * To start the memory dump, the host writes txDescriptorAddress to the
 * xmit_descr register, it also writes the two stage DMA chain to
 * txDescriptorAddress.
 *
 * During the course of the memory dump, we update the address pointer in the
 * second descriptor, walking through memory.
 *
 * To read the memory, a transfer is initiated by writing the BM control
 * register, and an interrupt it triggered by writing to
 * signalHostAttentionAddress.
 *
 * This scheme employs a fixed size read, so at end of regions too much memory
 * is read, that is discarded. It would be possible to change the size
 * dynamically, but this would add very little.
 */
#include "nanocore.h"
#include "nanoc.h"
#include "mac_api.h"
#include "coredump.h"

/* Definitions ====================================== */

#define PACK_ULE16(_ptr, _val) \
   do {                                                     \
      ((unsigned char*)(_ptr))[0] = (_val) & 0xff;          \
      ((unsigned char*)(_ptr))[1] = ((_val) >> 8) & 0xff;   \
   } while(0)

#define PACK_ULE32(_ptr, _val) \
   do {                                                     \
      ((unsigned char*)(_ptr))[0] = (_val) & 0xff;          \
      ((unsigned char*)(_ptr))[1] = ((_val) >> 8) & 0xff;   \
      ((unsigned char*)(_ptr))[2] = ((_val) >> 16) & 0xff;  \
      ((unsigned char*)(_ptr))[3] = ((_val) >> 24) & 0xff;  \
   } while(0)

#define COREDUMP_TIMEOUT 1000
#define XFER_SIZE 512 /* this gets aligned with pdu_size_alignment etc */
#define SIZEOF_BM_DESCRIPTOR 16

void add_hic_header(struct nanoc_hw *hw, unsigned msgid, de_pdu_t *pdu);
static int  coredump_state_init(struct nanoc_hw *hw, de_pdu_t *pdu);
static int  coredump_state_wait_cfm(struct nanoc_hw *hw, de_pdu_t *pdu);
static int  coredump_state_upload(struct nanoc_hw *hw, de_pdu_t *pdu);
static int  send_scb_error_req(struct nanoc_hw *hw);
static void prepare_bm(struct nanoc_hw *hw);
static void upload_bm(struct nanoc_hw *hw, uint32_t addr, uint32_t size);
static int  synchronize_seq(struct nanoc_hw *hw);
static void read_seq(struct nanoc_hw *hw, uint32_t addr, uint32_t size);
static void coredump_timeout(de_timer_t *t);
static const struct memory_region * current_region(const struct nanoc_hw *hw);
static int  get_next_block(struct nanoc_hw *hw);
static void check_coredump_complete(struct nanoc_hw *hw);

/* Variables ======================================== */

static const struct memory_region regions_nrx701[] = {
   /* NRX701 core registers */
   { 0x00000000, 0x00020000 }, /* RAM */
   { 0x00040000, 0x00008000 }, /* RAM */
   { 0x00070000, 0x00000100 }, /* reset,hpi,gpio,iomux,dcu,clocks,rfif,dcu2,adcdac,hfc2,clock gate */
   { 0x00070100, 0x0000000c }, /* tsf timer */
   { 0x00070200, 0x00000020 }, /* icu context, clock divider */
   { 0x00070300, 0x00000010 }, /* watchdog */
   { 0x00071000, 0x00000060 }, /* icu */
   { 0x00072000, 0x00000050 }, /* rand+spi+soft irq */
   { 0x00073000, 0x00000080 }, /* bm reg */
   { 0x00074000, 0x00000014 }, /* bb events */
   { 0x00075000, 0x00000038 }, /* backoff */
   { 0x00076000, 0x00000050 }, /* tsf */
   { 0x00078000, 0x000000c8 }, /* bb */
   { 0x0007a000, 0x00000084 }  /* bma */
};

static struct chip_descr chipdescr_nrx701 = {
   regions_nrx701,
   ARRAY_SIZE(regions_nrx701),
   0x73020,
   0x73024,
   0x40000,
   send_scb_error_req,
   prepare_bm,
   upload_bm,
};

static const struct memory_region regions_nrx600[] = {
   /* NRX600 core registers */
   { 0x00040000, 0x00010000 }, /* RAM */
   { 0x00050000, 0x00006000 }, /* RAM */
   { 0xfffc0000, 0x000000a0 }, /* .hw_dcu_reg    */
   { 0xfffc0200, 0x00000010 }, /* .hw_tsf_tmr_cntr_reg  */
   { 0xfffc0300, 0x00000030 }, /* .hw_gpio_reg   */
   { 0xfffc0400, 0x00000010 }, /* .hw_wdg_ctrl_reg  */
   { 0xfffc0500, 0x00000018 }, /* .hw_hpi_hw_reg  */
   { 0xfffc0e00, 0x00000010 }, /* .hw_mac_clk_div_reg  */
   { 0xfffc0f00, 0x00000010 }, /* .hw_mac_misc_reg  */
   { 0xfffc1000, 0x00000034 }, /* .hw_icu_hw_reg  */
   { 0xfffc2000, 0x0000004c }, /* .hw_tsf_tmr_reg  */
   { 0xfffc3000, 0x00000060 }, /* .hw_bm_reg     */
   { 0xfffc4000, 0x00000010 }, /* .hw_fmu_reg    */
   { 0xfffc4100, 0x00000010 }, /* .hw_hw_build_info_reg   */
   { 0xfffc4200, 0x00000004 }, /* .hw_rand_reg   */
   { 0xfffc4300, 0x0000000c }, /* .hw_spi_reg    */
   { 0xfffc5000, 0x000000c0 }, /* .hw_backoff_reg  */
   { 0xfffc6000, 0x00000080 }, /* .hw_seq_reg    */
   { 0xfffc6100, 0x0000001c }, /* .hw_hw_events_reg  */
   { 0xfffc7000, 0x0000008c }, /* .hw_bbrx_ctrl_reg  */
   { 0xfffc7100, 0x000000d0 }, /* .hw_rf_control_reg  */
   { 0xfffc7200, 0x00000050 }, /* .hw_bb_misc_reg  */
   { 0xfffc7300, 0x00000020 }, /* .hw_bbtx_reg   */
   { 0xfffc7320, 0x0000001c }, /* .hw_bbtx_predist_reg  */
   { 0xfffc7400, 0x00000044 }, /* .hw_bbrx_status_reg  */
   { 0xfffc7500, 0x00000014 }, /* .hw_reset_reg_bb  */
   { 0xfffc7600, 0x00000010 }, /* .hw_bbrx_ctrl_reg2  */
   { 0xfffc7700, 0x0000003c }, /* .hw_adc_dac_reg  */
   { 0xfffc7800, 0x0000001c }, /* .hw_bb_debug_channel_reg  */
   { 0xfffc8000, 0x00000100 }, /* .hw_mmu_reg  */
   { 0xfffca000, 0x0000004c }  /* .hw_coexist_reg  */
};

static struct chip_descr chipdescr_nrx600 = {
   regions_nrx600,
   ARRAY_SIZE(regions_nrx600),
   0xfffc3000,
   0xfffc3004,
   0x60000,
   send_scb_error_req,
   prepare_bm,
   upload_bm
};

static const struct memory_region regions_nrx900[]= {
   { 0x00040000, 256 * 1024 },  /* RAM */
   { 0xfffc0000,  64 * 1024 }  /* registers */
};

static struct chip_descr chipdescr_nrx900 = {
   regions_nrx900,
   ARRAY_SIZE(regions_nrx900),
   0,  /* N/A */
   0,  /* N/A */
   0x00000000,
   synchronize_seq,
   NULL,
   read_seq
};


#if 0
static const char *state2str[] = {
   "INIT", "WAIT_CFM", "UPLOAD", "FAIL"
};
#endif

/* Global functions ======================================== */

void
coredump_init(struct nanoc_hw *hw)
{
   de_timer_init(&hw->coredump.timer, coredump_timeout);
}

void
coredump_start_timer(struct nanoc_hw *hw)
{
   de_timer_stop(&hw->coredump.timer);
   de_timer_start(&hw->coredump.timer, COREDUMP_TIMEOUT);
}

int
coredump_commit_suicide(struct nanoc_hw *hw)
{
   nrx_ctrl_commit_suicide_req_t *msg;
   de_pdu_t *pdu = de_pdu_alloc(sizeof *msg);
   de_info(DE_TR_NANOC, "Sending commit suicide");

   msg = de_pdu_put(pdu, sizeof *msg);
   msg->reserved = 0;
   nanoc_hw_add_hic_header(hw, HIC_MESSAGE_TYPE_CTRL, NRX_CTRL_COMMIT_SUICIDE_REQ, pdu);
   coredump_start_timer(hw);
   return nanoc_hw_send(hw, pdu);
}

int
coredump_confirm_pdu(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   unsigned i;
   for (i = 0; i < sizeof(hw->coredump.pdus)/sizeof(hw->coredump.pdus[0]); i++) {
      if (hw->coredump.pdus[i] == pdu) {
         hw->coredump.pdus[i] = NULL;
         hw->coredump.outstanding_pdus--;
         check_coredump_complete(hw);
         return 1;
      }
   }
   return 0;
}

/**
 * Since we are in coredump state, the HIC framing length was not removed on
 * this pdu. We have to do it ourselves.
 */
void
coredump_sm(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   de_timer_stop(&hw->coredump.timer);
   if (coredump_enable) {
      int next_state;

      switch (hw->coredump.state)
      {
         case COREDUMP_STATE_INIT:
            next_state = coredump_state_init(hw, pdu);
            break;

         case COREDUMP_STATE_WAIT_CFM:
            /* Remove framing length field */
            de_pdu_pull(pdu, sizeof(uint16_t));
            de_pdu_trim(pdu, *(uint16_t *)de_pdu_data(pdu));
            next_state = coredump_state_wait_cfm(hw, pdu);
            break;

         case COREDUMP_STATE_UPLOAD:
            {
               uint16_t length = *(uint16_t *)de_pdu_data(pdu);
               de_pdu_pull(pdu, sizeof(uint16_t));
               de_pdu_trim(pdu, length);
               if (de_pdu_size(pdu) >= 2) {
                  next_state = coredump_state_upload(hw, pdu);
               } else {
                  de_error(DE_TR_NANOC, "Chunk too small (%zu)", de_pdu_size(pdu));
                  next_state = COREDUMP_STATE_FAIL;
               }
            }
            break;

         case COREDUMP_STATE_FAIL:
            next_state = COREDUMP_STATE_FAIL;
            break;

         default:
            next_state = COREDUMP_STATE_FAIL;
            de_error(DE_TR_NANOC, "Error! Coredump state machine confusion, state: %u",
                     hw->coredump.state);
      }

      hw->coredump.state = next_state;

      if (next_state >= COREDUMP_STATE_COMPLETE) {
         if (next_state == COREDUMP_STATE_FAIL) {
            de_error(DE_TR_NANOC, "Coredump failed");
         }
         if (!hw->coredump.fh) {
            /* Create an empty coredump file if the coredump failed */
            hw->coredump.fh = de_coredump_open(&hw->de, 0, 0, 0);
         }
         de_coredump_close(hw->coredump.fh);
         hw->coredump.fh = NULL;
         check_coredump_complete(hw);
      } else {
         de_timer_start(&hw->coredump.timer, COREDUMP_TIMEOUT);
      }
   }
}


/* Local functions ======================================== */

static void
coredump_timeout(de_timer_t *t)
{
   struct nanoc_hw *hw = container_of(t, struct nanoc_hw, coredump.timer);
   de_warn(DE_TR_NANOC, "Timeout hw=%p, hw->flags=%#x, hw->coredump.state=%d ", hw, hw->flags, hw->coredump.state);
   hw->coredump.state = COREDUMP_STATE_FAIL;
   coredump_sm(hw, NULL);
}

static size_t
calc_padding(struct nanoc_hw *hw, uint32_t size, uint32_t min_pad)
{
   size_t padding = 0;

   if(size < hw->min_sz_to_host) {
      padding = hw->min_sz_to_host - size;
   }
   if(padding != 0 && padding < min_pad) {
      padding = min_pad;
   }

#undef A
#define A(N, S) ((N) & ((S) - 1))

   if (A(size + padding, hw->sz_modulo_from_host) != 0) {
      padding += hw->sz_modulo_from_host - A(size + padding, hw->sz_modulo_from_host);
   }
   if (padding != 0 && padding < min_pad) {
      padding = min_pad;
   }
   if (A(size + padding, hw->sz_modulo_from_host) != 0) {
      padding += hw->sz_modulo_from_host - A(size + padding, hw->sz_modulo_from_host);
   }

   return padding;
}

/* We need to remember which pdus that belong to coredump so that these wont be sent up to the clients */
static de_pdu_t *
coredump_alloc_pdu(struct nanoc_hw *hw, size_t size)
{
   unsigned i;
   de_pdu_t *pdu = de_pdu_alloc(size);
   for (i = 0; i < sizeof(hw->coredump.pdus)/sizeof(hw->coredump.pdus[0]); i++) {
      if (hw->coredump.pdus[i] == NULL) {
         hw->coredump.pdus[i] = pdu;
         hw->coredump.outstanding_pdus++;
         break;
      }
   }
   DE_BUG_ON(i == sizeof(hw->coredump.pdus)/sizeof(hw->coredump.pdus[0]));
   return pdu;
}

static int
send_scb_error_req(struct nanoc_hw *hw)
{
   de_pdu_t *req_pdu = coredump_alloc_pdu(hw, sizeof(nrx_ctrl_scb_error_req_t));
   nrx_ctrl_scb_error_req_t *req = de_pdu_put(req_pdu, sizeof(*req));
   req->magic1 = NRX_CTRL_SCB_ERROR_REQ_MAGIC1;
   req->magic2 = NRX_CTRL_SCB_ERROR_REQ_MAGIC2;
   req->magic3 = NRX_CTRL_SCB_ERROR_REQ_MAGIC3;
   req->magic4 = NRX_CTRL_SCB_ERROR_REQ_MAGIC4;
   nanoc_hw_add_hic_header(hw, HIC_MESSAGE_TYPE_CTRL, NRX_CTRL_SCB_ERROR_REQ, req_pdu);
   nanoc_hw_send(hw, req_pdu);
   return COREDUMP_STATE_WAIT_CFM;
}

static unsigned char *
write_u32_bm(unsigned char *buf, uint32_t address, uint32_t value)
{
   PACK_ULE16(buf + 0, 4);
   PACK_ULE32(buf + 2, address);
   PACK_ULE32(buf + 6, value);
   return buf + 10;
}

static unsigned char *
write_descr_bm(unsigned char *buf, uint32_t buf_ptr, uint32_t size, uint32_t next)
{
   PACK_ULE32(buf + 0, buf_ptr);        /* buf_ptr */
   PACK_ULE32(buf + 4, 0);              /* ctrl */
   PACK_ULE16(buf + 8, size);           /* size */
   PACK_ULE16(buf + 10, 0);             /* expedited_size */
   PACK_ULE32(buf + 12, next);          /* next */
   return buf+16;
}

static void
prepare_bm(struct nanoc_hw *hw)
{
   size_t         len, padding;
   de_pdu_t      *pdu;
   unsigned char *buf;

   len = 6 + 4 + 6 + 2 * SIZEOF_BM_DESCRIPTOR;
   padding = calc_padding(hw, len, 6);
   len += padding;

   pdu = coredump_alloc_pdu(hw, len);
   de_pdu_put(pdu, len);
   buf = de_pdu_data(pdu);

   /* Write descriptor address to register */
   buf = write_u32_bm(buf, hw->coredump.chip_descr->bm_reg_descr, hw->coredump.descr_addr);

   PACK_ULE16(buf + 0, 2 * SIZEOF_BM_DESCRIPTOR);
   PACK_ULE32(buf + 2, hw->coredump.descr_addr);
   buf += 6;

   /* Descriptor that transmits the length (2 bytes) from the 'size' field of
    * the next descriptor.
    */
   buf = write_descr_bm(buf,
                        hw->coredump.descr_addr + SIZEOF_BM_DESCRIPTOR + 8,
                        2,
                        hw->coredump.descr_addr + SIZEOF_BM_DESCRIPTOR);

   /* Descriptor that transmits data from specified address, length is fiexed
    * to XFER_SIZE
    */
   buf = write_descr_bm(buf, 0, XFER_SIZE-2, 0);

   DE_ASSERT(padding == 0 || padding >= 6);

   if (padding > 0) {
      /* Padding is at least 6 */
      PACK_ULE16(buf + 0, padding - 6);
      PACK_ULE32(buf + 2, hw->coredump.chip_descr->safe_addr);
      memset(buf + 6, 0, padding - 6);
   }

   hw->ops->hw_send(hw, pdu);
}

static void
upload_bm(struct nanoc_hw *hw, uint32_t addr, uint32_t size)
{
   size_t len = 0;
   size_t padding;
   de_pdu_t *pdu;
   unsigned char *buf;

   len += 3 * 10; /* we're writing 3 32-bit words */

   padding = calc_padding(hw, len, 6);
   DE_ASSERT(padding == 0 || padding >= 6);
   len += padding;

   pdu = coredump_alloc_pdu(hw, len);
   de_pdu_put(pdu, len);
   buf = de_pdu_data(pdu);

   /* write address to descriptor */
   buf = write_u32_bm(buf, hw->coredump.descr_addr + SIZEOF_BM_DESCRIPTOR, addr);

   /* start the transmission */
   buf = write_u32_bm(buf, hw->coredump.chip_descr->bm_reg_ctrl, 1);

   /* trigger irq */
   buf = write_u32_bm(buf, hw->coredump.irq_addr, 1);

   DE_ASSERT(padding == 0 || padding >= 6);

   if(padding > 0) {
      /* padding is at least 6 */
      PACK_ULE16(buf + 0, padding - 6);
      PACK_ULE32(buf + 2, hw->coredump.chip_descr->safe_addr);
      memset(buf + 6, 0, padding - 6);
   }

   hw->ops->hw_send(hw, pdu);
}

static int
synchronize_seq(struct nanoc_hw *hw)
{
   /* No sync needed for NRX900. Just start to upload */
   hw->coredump.region_index = 0;
   hw->coredump.region_addr  = current_region(hw)->addr;
   return get_next_block(hw);
}

static void
read_seq(struct nanoc_hw *hw, uint32_t addr, uint32_t size)
{
   size_t len = 0;
   size_t padding;
   de_pdu_t *pdu;
   unsigned char *buf;

   len += 3 * 4;

   de_debug(DE_TR_NANOC, "Reading coredump on addr 0x%X size %d", addr, size);
   padding = calc_padding(hw, len, 12);
   DE_ASSERT(padding == 0 || padding >= 12);
   len += padding;

   pdu = coredump_alloc_pdu(hw, len);
   de_pdu_put(pdu, len);
   buf = de_pdu_data(pdu);

   /* first pad to even address, by writing 1 to 3 bytes to memory */
   if((padding & 3) != 0) {
      PACK_ULE32(buf + 0, 0xfeed0001);                   /* WRITE */
      PACK_ULE32(buf + 4, 4 - (padding & 3));            /* LENGTH */
      PACK_ULE32(buf + 8, hw->coredump.chip_descr->safe_addr); /* ADDRESS */
      buf += 12 + 4 - (padding & 3);                        /* DATA */
      padding -= 12 + 4 - (padding & 3);
   }
   DE_ASSERT((padding & 3) == 0);
   /* pad remaining buffer with nops */
   while(padding) {
      PACK_ULE32(buf, 0xfeed0000); /* NOP */
      buf += 4;
      padding -= 4;
   }
   /* done with padding */

   if(size > XFER_SIZE) {
      size = XFER_SIZE;
   }
   size += 2;
   size += calc_padding(hw, size, 0);

   PACK_ULE32(buf + 0, 0xfeed0002); /* READ */
   PACK_ULE32(buf + 4, size - 2);   /* LENGTH */
   PACK_ULE32(buf + 8, addr);       /* ADDRESS */

   hw->ops->hw_send(hw, pdu);
}

static const struct memory_region *
current_region(const struct nanoc_hw *hw)
{
   return &hw->coredump.chip_descr->regions[hw->coredump.region_index];
}

static int
get_next_block(struct nanoc_hw *hw)
{
   const struct memory_region *cur_region = current_region(hw);
   uint32_t size;

   if (hw->coredump.region_addr == cur_region->addr + cur_region->size) {
      if (++hw->coredump.region_index == hw->coredump.chip_descr->nregions) {
         de_debug(DE_TR_NANOC, "COREDUMP_STATE_COMPLETE");
         return COREDUMP_STATE_COMPLETE;
      }
      cur_region = current_region(hw);
      hw->coredump.region_addr = cur_region->addr;
   }
   size = cur_region->addr + cur_region->size - hw->coredump.region_addr;
   hw->coredump.chip_descr->read_memory(hw, hw->coredump.region_addr, size);
   return COREDUMP_STATE_UPLOAD;
}

static size_t calc_coredump_size(struct chip_descr *descr)
{
   size_t size = 0;
   unsigned int i;

   for(i = 0; i < descr->nregions; i++)
      size += sizeof(descr->regions[i]) + descr->regions[i].size;

   return size;
}

static int
coredump_state_init(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_hic_message_t    *hdr = (nrx_hic_message_t *)de_pdu_data(pdu);
   nrx_ctrl_scb_error_ind_t *ind = de_pdu_pull(pdu, sizeof *hdr);

   DE_ASSERT(hdr->type == HIC_MESSAGE_TYPE_CTRL);
   DE_ASSERT(hdr->id == NRX_CTRL_SCB_ERROR_IND);

   de_mutex_lock(&hw->mutex);
   hw->flags |= (NANOC_HW_RESTARTING | NANOC_HW_COREDUMP);
   hw->flags &= ~NANOC_HW_COMMIT_SUICIDE;
   de_mutex_unlock(&hw->mutex);
   
   de_error(DE_TR_NANOC, "SCB_ERROR_IND %u.%u %#x",
           ind->obj_id,
           ind->error_code,
           ind->fault_addr);

   switch (hw->device_descr->ndd_variant) {
      case NRX701:
         hw->coredump.chip_descr = &chipdescr_nrx701;
         break;
      case NRX615:
         hw->coredump.chip_descr = &chipdescr_nrx600;
         break;
      case NRX900:
         hw->coredump.chip_descr = &chipdescr_nrx900;
         break;
      default:
         DE_ASSERT(0);
   }
   hw->coredump.objid        = ind->obj_id;
   hw->coredump.errcode      = ind->error_code;
   hw->coredump.irq_addr     = ind->signal_host_attention_address;
   hw->coredump.descr_addr   = ind->tx_descriptor_address;
   hw->coredump.region_index = 0;
   hw->coredump.region_addr  = hw->coredump.chip_descr->regions[0].addr;
   hw->coredump.outstanding_pdus = 0;
   memset(hw->coredump.pdus, 0, sizeof(hw->coredump.pdus));

   hw->coredump.fh = de_coredump_open(&hw->de, ind->obj_id, ind->error_code,
                                      calc_coredump_size(hw->coredump.chip_descr));
   DE_ASSERT(hw->coredump.fh != NULL);

   nanoc_tp_setspeed(hw, hw->coredump_speed);
   
   return hw->coredump.chip_descr->synchronize(hw);
}

static int
coredump_state_wait_cfm(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_hic_message_t    *hdr = (nrx_hic_message_t *)de_pdu_data(pdu);
   nrx_ctrl_scb_error_cfm_t *cfm = de_pdu_pull(pdu, sizeof *hdr);

   if (hdr->type != HIC_MESSAGE_TYPE_CTRL || hdr->id != NRX_CTRL_SCB_ERROR_CFM)
   {
      de_warn(DE_TR_NANOC, "Frame dropped (expecting SCB_ERROR_CFM)");
      de_pdu_free(pdu);
      return COREDUMP_STATE_WAIT_CFM;
   }

   de_info(DE_TR_NANOC, "SCB_ERROR_CFM %u.%u %#x %#x",
           cfm->obj_id,
           cfm->error_code,
           cfm->tx_descriptor_address,
           cfm->signal_host_attention_address);

   hw->coredump.chip_descr->setup_memory(hw);
   hw->coredump.region_index = 0;
   hw->coredump.region_addr  = current_region(hw)->addr;
   return get_next_block(hw);
}

static int
coredump_state_upload(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   const struct memory_region *cur_region = current_region(hw);
   uint32_t cur_addr = hw->coredump.region_addr;
   size_t   bytes;

   bytes = de_pdu_size(pdu);

   /* We may have received memory outside current region */
   if (cur_addr + bytes > cur_region->addr + cur_region->size) {
      bytes = cur_region->addr + cur_region->size - cur_addr;
      de_pdu_trim(pdu, bytes);
   }

   /* Write coredump region header */
   if (cur_addr == cur_region->addr) {
      memcpy(de_pdu_push(pdu, sizeof(*cur_region)),
             cur_region,
             sizeof(*cur_region));
   }

   if (hw->coredump.fh) {
      de_coredump_write(hw->coredump.fh, pdu);
   }

   hw->coredump.region_addr += bytes;
   return get_next_block(hw);
}

/* The Tx complete signal can come after the data therefore we need to check outstanding pdus
 * after COREDUMP_STATE_COMPLETE. */
static void
check_coredump_complete(struct nanoc_hw *hw)
{
   unsigned done = 0;
   de_mutex_lock(&hw->mutex);
   if (hw->coredump.state >= COREDUMP_STATE_COMPLETE) {
      if (hw->coredump.outstanding_pdus == 0) {
         hw->flags &= ~(NANOC_HW_COMMIT_SUICIDE|NANOC_HW_COREDUMP);
         hw->coredump.state = COREDUMP_STATE_INIT;
         done = 1;
      }
   }
   de_mutex_unlock(&hw->mutex);
   if (done) {
      de_error(DE_TR_NANOC, "Coredump completed!");
      if (restart_after_coredump) {
         nanoc_hw_restart(hw);
      }
   }
}
