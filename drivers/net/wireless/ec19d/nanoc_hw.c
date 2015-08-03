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

#include "driverenv.h"
#include "nanocore.h"
#include "nanoc.h"
#include "coredump.h"
#include "sdio_reg.h"
#ifdef NRX_COMPONENT_HIC_PCAP_BASE
   #include "hic_pcap.h"
#endif


/* Definitions ====================================== */

#define _STR(x) #x
#define STR(x) _STR(x)

static int        chip_identify(struct nanoc_hw *hw);
static int        chip_init(struct nanoc_hw *hw);
static void       chip_start_rx(struct nanoc_hw *hw);
static void       chip_stop_rx(struct nanoc_hw *hw);
static int        chip_download_fw(struct nanoc_hw *hw, unsigned fw_id);
static de_pdu_t * create_set_alignment(struct nanoc_hw *hw);
static int        recv_set_alignment(struct nanoc_hw *hw, de_pdu_t *pdu);
static de_pdu_t * create_set_mac_addr(struct nanoc_hw *hw);
static de_pdu_t * create_init_completed(struct nanoc_hw *hw);
static int        recv_init_completed(struct nanoc_hw *hw, de_pdu_t *pdu);
static de_pdu_t * create_get_fw_version(struct nanoc_hw *hw);
static int        recv_fw_version(struct nanoc_hw *hw, de_pdu_t *pdu);
#ifdef NRX_CONFIG_HIC_AGGREGATION
static de_pdu_t*  create_mib_set_hic_aggr_filter(struct nanoc_hw *hw);
static de_pdu_t*  create_mib_set_hic_aggr_size(struct nanoc_hw *hw);
#endif /* NRX_CONFIG_HIC_AGGREGATION */
static de_pdu_t* create_mib_set_req(struct nanoc_hw *hw, nrx_mib_id_t id,
                                    const void *data, size_t size);
static int        recv_mib_set_cfm(struct nanoc_hw *hw, de_pdu_t *pdu);
static void       nanoc_hw_sleep_forever(struct nanoc_hw *hw);
static int        chip_download_blob(struct nanoc_hw *hw, const void *blob, size_t size);
static int        get_fw_id(struct nanoc_hw *hw, enum nanoc_feat feature_mask);
static int        chip_reset(struct nanoc_hw *hw);
static void       handle_command_timeout(de_timer_t *t);
static void       send_cmds(struct nanoc_hw *hw);
static unsigned   recv_cmds(struct nanoc_hw *hw, de_pdu_t *pdu);

/* Variables ======================================== */

static struct nanoc_hw *hw_array[MAX_NUM_HWS];

static uint8_t recv_interval;
/* Time for firmware to startup in millseconds */
/* Temporary fix (was 100) due to BT firmware delays startup with some crypto initialization that takes looong time */
static const int fw_startup_time_ms = 100;

static const struct fw_feature
{
   enum chip_variants chip_variant;
   enum nanoc_feat    feature_mask;
   char               filename[32];
} fw_table[] =
{
   {
      NRX701,
      NANOC_FEAT_INIT,
      "nrx701_init.fw"
   },

   {
      NRX701,
      NANOC_FEAT_WIFI_STA | NANOC_FEAT_WIFI_AP,
      "nrx701_wifi.fw"
   },

   {
      NRX701,
      NANOC_FEAT_TESTMODE,
      "nrx701_test.fw"
   },

   {
      NRX615,
      NANOC_FEAT_INIT,
      "nrx615_init.fw"
   },

   {
      NRX615,
      NANOC_FEAT_WIFI_STA | NANOC_FEAT_WIFI_AP,
      "nrx615_wifi.fw"
   },

   {
      NRX615,
      NANOC_FEAT_TESTMODE,
      "nrx615_test.fw"
   },

   {
      NRX900,
      NANOC_FEAT_INIT,
      "nrx900_init.fw"
   },

   {
      NRX900,
      NANOC_FEAT_WIFI_STA | NANOC_FEAT_WIFI_AP | NANOC_FEAT_BT | NANOC_FEAT_AUDIO,
      "nrx900_wifi.fw"
   },

   {
      NRX900,
      NANOC_FEAT_TESTMODE,
      "nrx900_test.fw"
   },
};

static struct {
   de_pdu_t * (*create_cmd)(struct nanoc_hw *hw);
   int        (*recv_cmd)(struct nanoc_hw *hw, de_pdu_t *pdu);
} cmd_array[] =
{
   {create_set_alignment , recv_set_alignment},
   {create_set_mac_addr  , recv_mib_set_cfm},
   {create_get_fw_version, recv_fw_version},
   {create_init_completed, recv_init_completed},
#ifdef NRX_CONFIG_HIC_AGGREGATION
   {create_mib_set_hic_aggr_filter, recv_mib_set_cfm},
   {create_mib_set_hic_aggr_size, recv_mib_set_cfm},
#endif /* NRX_CONFIG_HIC_AGGREGATION */
};

static const struct bm_descr
{
       uint32_t buf_ptr;           /* ptr to data buffer */
       uint32_t ctrl;
       uint16_t size;              /* Request buffer size in octets */
       uint16_t expedited_size;    /* Expedited size in octets */
       uint32_t next;              /* ptr to next descriptor, '0' terminates */
} bm_descr_nrx701[] =
{
   { 0x46110, 0, 0x30, 0, 0x46110 },
   { 0x46138, 0, 0x02, 0, 0x46120 },
   { 0x46130, 0, 0x04, 0, 0x46130 },
   { 0x00000, 0, 0x00, 0, 0x46110 }
};

/* Generated with:
   arm-none-eabi-gcc -c shutdown_nrx701.S
   arm-none-eabi-ld -Ttext=0 -o shutdown_nrx701.elf shutdown_nrx701.o
   fw_path/tools/bin/elf2boot.py --fmt=ccode --hw=nrx1 --exclude-bm shutdown_nrx701.elf
 */
static const uint8_t sleep_forever_700[] =
{
   0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xa0, 0xe1, 0x28, 0x00, 0x9f, 0xe5, 0x46, 0x10,
   0xa0, 0xe3, 0x18, 0x10, 0xc0, 0xe5, 0x05, 0x10,
   0xa0, 0xe3, 0xb8, 0x10, 0xc0, 0xe1, 0x1f, 0x10,
   0xa0, 0xe3, 0x1a, 0x10, 0xc0, 0xe5, 0xff, 0x14,
   0xe0, 0xe3, 0x0c, 0x10, 0x80, 0xe5, 0x02, 0x10,
   0xa0, 0xe3, 0x00, 0x10, 0xc0, 0xe5, 0xfe, 0xff,
   0xff, 0xea, 0x30, 0x00, 0x07, 0x00, 0x08, 0x00,
   0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x60,
   0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x40, 0x20, 0x07, 0x00, 0xff, 0xff, 0xff, 0xff
};

/* Generated with:
 * fw_path/tools/bin/make_init_blob.py --hw=nrx2a -c sleep_forever_nrx600.txt out.txt
 * cat out.txt
 */
static const uint8_t sleep_forever_600[] =
{
   0x04, 0x00, 0x2C, 0x10, 0xFC, 0xFF, 0x00, 0x00,
   0x00, 0x00, 0x01, 0x00, 0x19, 0x00, 0xFC, 0xFF,
   0x07, 0x04, 0x00, 0x08, 0x00, 0xFC, 0xFF, 0x01,
   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xFC,
   0xFF, 0x01, 0x18, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Generated with:
 * fw_path/tools/bin/make_init_blob.py --hw=nrx2b -c sleep_forever_nrx900.txt out.txt
 * cat out.txt
 */
static const uint8_t sleep_forever_900[] =
{
   0x01, 0x00, 0xED, 0xFE, 0x04, 0x00, 0x00, 0x00,
   0x2C, 0x10, 0xFC, 0xFF, 0x00, 0x00, 0x00, 0x00,
   0x01, 0x00, 0xED, 0xFE, 0x04, 0x00, 0x00, 0x00,
   0x68, 0x00, 0xFC, 0xFF, 0x00, 0x00, 0x00, 0x00,
   0x01, 0x00, 0xED, 0xFE, 0x02, 0x00, 0x00, 0x00,
   0x32, 0x00, 0xFC, 0xFF, 0x10, 0x01, 0x01, 0x00,
   0xED, 0xFE, 0x01, 0x00, 0x00, 0x00, 0x19, 0x00,
   0xFC, 0xFF, 0x07, 0x01, 0x00, 0xED, 0xFE, 0x04,
   0x00, 0x00, 0x00, 0x08, 0x00, 0xFC, 0xFF, 0x01,
   0x00, 0x00, 0x00, 0x01, 0x00, 0xED, 0xFE, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x01,
   0x01, 0x00, 0xED, 0xFE, 0x1C, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Generated with:
 * fw_path/tools/bin/make_init_blob.py --hw=nrx1 -c gpio_init_700.txt out.txt
 * cat out.txt
 */
/* FIXME: Must move padding to beginning of array to make it work on USB box. No clue why. */
static const uint8_t gpio_init_700[] =
{
   0x12, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x04, 0x00, 0x10, 0x00, 0x07, 0x00, 0x00, 0x40,
   0x83, 0x00, 0x04, 0x00, 0x18, 0x00, 0x07, 0x00,
   0xDF, 0x40, 0xCF, 0xFD, 0x04, 0x00, 0x14, 0x00,
   0x07, 0x00, 0xDF, 0xC0, 0xCF, 0xFD, 0x04, 0x00,
   0x20, 0x00, 0x07, 0x00, 0x00, 0x03, 0x02, 0x00,
};

/* Generated with:
 * fw_path/tools/bin/make_init_blob.py --hw=nrx2a -c gpio_init_600_900.txt out.txt
 * cat out.txt
 */
static const uint8_t gpio_init_600[] =
{
   0x04, 0x00, 0x08, 0x03, 0xFC, 0xFF, 0x00, 0x00,
   0x80, 0x00, 0x04, 0x00, 0x04, 0x03, 0xFC, 0xFF,
   0x40, 0x00, 0x80, 0x81, 0x04, 0x00, 0x00, 0x03,
   0xFC, 0xFF, 0x40, 0x00, 0x80, 0x81, 0x1C, 0x00,
   0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* Generated with:
 * fw_path/tools/bin/make_init_blob.py --hw=nrx2b -c gpio_init_600_900.txt out.txt
 * cat out.txt
 */
static const uint8_t gpio_init_900[] =
{
   0x01, 0x00, 0xED, 0xFE, 0x04, 0x00, 0x00, 0x00,
   0x08, 0x03, 0xFC, 0xFF, 0x00, 0x00, 0x80, 0x00,
   0x01, 0x00, 0xED, 0xFE, 0x04, 0x00, 0x00, 0x00,
   0x04, 0x03, 0xFC, 0xFF, 0x40, 0x00, 0x80, 0x81,
   0x01, 0x00, 0xED, 0xFE, 0x04, 0x00, 0x00, 0x00,
   0x00, 0x03, 0xFC, 0xFF, 0x40, 0x00, 0x80, 0x81,
   0x01, 0x00, 0xED, 0xFE, 0x04, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#ifdef NRX_CONFIG_EMBED_FW
/* If a fw file cannot be stored on the host platform, the fw can be built into the driver.
 * Note! Only one fw image can be handled, which means that only one type of chip can be
 * handled.
 *
 * Use the following command to create a fw.c file
 * -----------------------------------------------
 *
 * nrx700:
 * fw_path/tools/bin/elf2boot.py --fmt=ccode --hw=nrx1 --exclude-bm x_mac.axf fw.c
 *
 * nrx600:
 * fw_path/tools/bin/elf2boot.py --fmt=ccode --hw=nrx2a patch.elf fw.c
 *
 * nrx900:
 * fw_path/tools/bin/elf2boot.py --fmt=ccode --hw=nrx2b patch.elf fw.c
 */
#include "fw.c"
#endif

/* Module parameters ================================= */

#ifndef NRX_DEFAULT_TRACE_LEVEL
#define NRX_DEFAULT_TRACE_LEVEL DE_WARN
//#define NRX_DEFAULT_TRACE_LEVEL DE_DEBUG
#endif
#ifndef NRX_DEFAULT_TRACE_MASK
#define NRX_DEFAULT_TRACE_MASK DE_TR_ALL
#endif

int command_timeout = 5000;
DE_MODULE_PARAM(command_timeout, int, "Command timeout");
int fw_load_enable = 1;
DE_MODULE_PARAM(fw_load_enable, int, "No reset, dont load fw");
int restart_after_coredump = 0;
DE_MODULE_PARAM(restart_after_coredump, int, "Restart chip after coredump");
int coredump_enable = 0;
DE_MODULE_PARAM(coredump_enable, int, "Request coredump");
int ifdown_enable = 0;
DE_MODULE_PARAM(ifdown_enable, int, "Enable interface down");
int trace_level = NRX_DEFAULT_TRACE_LEVEL;
DE_MODULE_PARAM(trace_level, int, "Debug level selection");
unsigned int trace_mask = NRX_DEFAULT_TRACE_MASK;
DE_MODULE_PARAM(trace_mask, int, "Debug mask selection");
static char chip_version[12];
DE_MODULE_PARAM_STR_RO(chip_version, sizeof(chip_version), "Chip version");
static char fw_version[256];
DE_MODULE_PARAM_STR_RO(fw_version, sizeof(fw_version), "Firmware version in chip");
static char mac_addr_override[32] = "00-AB-11-CD-22-EF";
DE_MODULE_PARAM_STR(mac_addr_override, sizeof(mac_addr_override), "This mac address will be used if it is set prior to power up of chip");

static void nanoc_hw_set_req_coredump(const char *val);
static void nanoc_hw_get_req_coredump(char *buffer);
static int req_coredump;
DE_MODULE_PARAM_CB(req_coredump, nanoc_hw_get_req_coredump, nanoc_hw_set_req_coredump, "Write this to request coredump");

/* Global functions ================================= */

int
nanoc_hw_register(struct nanoc_hw *hw)
{
   unsigned i;
   de_init(&hw->de, hw->dev);
   de_timer_init(&hw->cmd_timer, handle_command_timeout);
   hw->flags = 0;
   hw->fw_id = 0; /* This points to init fw */
   hw->num_clients = 0;
#ifdef NRX_CONFIG_SDIO_CLK
   hw->default_speed = NRX_CONFIG_SDIO_CLK;
#else
   nanoc_tp_getspeed(hw, &hw->default_speed);
#endif

   for (i = 0; i < MAX_NUM_HWS; i++)
   {
      if (hw_array[i] == NULL) {
         hw_array[i] = hw;
         if (chip_init(hw) != 0) {
            return -2;
         }
         if (hw->ops->hw_shutdown) {
            hw->ops->hw_shutdown(hw, 1);
         } else {
            nanoc_hw_sleep_forever(hw);
         }
         if (strlen(mac_addr_override) >=17) {
            char dummy;
            sscanf(mac_addr_override, "%hhx%c%hhx%c%hhx%c%hhx%c%hhx%c%hhx",
                   &hw->mac_addr[0], &dummy,
                   &hw->mac_addr[1], &dummy,
                   &hw->mac_addr[2], &dummy,
                   &hw->mac_addr[3], &dummy,
                   &hw->mac_addr[4], &dummy,
                   &hw->mac_addr[5]);
         } else {
            de_assign_mac_addr(hw->dev, hw->mac_addr);
         }
         nanoc_new_device(hw);
         return 0;
      }
   }
   de_error(DE_TR_NANOC, "Too many hws!");
   return -1;
}

void
nanoc_hw_unregister(struct nanoc_hw *hw)
{
   unsigned i;
   de_info(DE_TR_NANOC, "hw=%p", hw);
   de_timer_stop(&hw->cmd_timer);
   nanoc_del_device(hw);
   for (i = 0; i < MAX_NUM_HWS; i++)
   {
      if (hw_array[i] == hw) {
         hw_array[i] = NULL;
         de_release(&hw->de);
         return;
      }
   }
   de_error(DE_TR_NANOC, "Failed to deregister hw!");
   DE_BUG_ON(1);
}

void
nanoc_hw_restart(struct nanoc_hw *hw)
{
   de_info(DE_TR_NANOC, "flags=%#x", hw->flags);
   chip_stop_rx(hw);
   if (chip_init(hw) != 0) {
      return;
   }
   chip_download_fw(hw, hw->fw_id);
   de_msleep(fw_startup_time_ms);
   chip_start_rx(hw);
   send_cmds(hw);
}

/**
 * Called from transport driver when a buffer has been received.
 *
 * Context: input-thread
 */
void nanoc_hw_recv_handler(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_hic_message_t *hdr;
   unsigned           net_id;
   unsigned char     *msg;

   if (hw->flags & NANOC_HW_COREDUMP)
   {
      coredump_sm(hw, pdu);
      return;
   }

   nanoc_hiclog("RX", pdu);
#ifdef NRX_COMPONENT_HIC_PCAP_BASE
   hic_pcap_add_pdu(pdu);
#endif
   /* Remove framing length field */
   hdr    = de_pdu_pull(pdu, sizeof(uint16_t));
   net_id = hdr->net_id;
   msg    = (unsigned char *)&((nrx_hic_message_t*)de_pdu_data(pdu))[1];

   de_pdu_trim(pdu, *(uint16_t *)de_pdu_data(pdu));

#ifdef NRX_CONFIG_HIC_AGGREGATION
   if(hdr->type == HIC_MESSAGE_TYPE_AGGREGATION) {
      nrx_hic_frame_t *frame;
      de_pdu_t *new_pdu;
      size_t frame_size;

      /* remove aggregation hic header */
      de_pdu_pull(pdu, sizeof(*hdr));

      while(de_pdu_size(pdu) > 0) {
         frame = (nrx_hic_frame_t*)de_pdu_data(pdu);
         frame_size = sizeof(frame->size) + frame->size;

         DE_BUG_ON(frame_size > de_pdu_size(pdu));
         if(frame_size == de_pdu_size(pdu))
            break;

#ifdef __linux__
         /* XXX */
         new_pdu = skb_clone(pdu, GFP_KERNEL);
         de_pdu_trim(new_pdu, frame_size);
#else
         new_pdu = de_pdu_alloc(frame_size);
         de_pdu_put(new_pdu, frame_size);
         memcpy(de_pdu_data(new_pdu), de_pdu_data(pdu), frame_size);
#endif
         nanoc_hw_recv_handler(hw, new_pdu);

         de_pdu_pull(pdu, frame_size);
      }
      nanoc_hw_recv_handler(hw, pdu);
      return;
   }
#endif /* NRX_CONFIG_HIC_AGGREGATION */
   if (net_id == NRX_NET_ID_UNKNOWN)
   {
      if (!recv_cmds(hw, pdu)) {
         if (hdr->type == HIC_MESSAGE_TYPE_CTRL) {
            switch (hdr->id)
            {
               case NRX_CTRL_WAKEUP_IND:
               {
                  nrx_ctrl_wakeup_ind_t *wake_ind = (nrx_ctrl_wakeup_ind_t *)msg;
                  de_debug(DE_TR_NANOC, "wakeup reason %#x, count = %u",
                        wake_ind->reason,
                        wake_ind->wakeup_count);
                  nanoc_wakeup_ind(hw);
               }
               break;

               case NRX_CTRL_SCB_ERROR_IND:
                  nanoc_link_down(hw, NANOC_QALL);
                  coredump_sm(hw, pdu);
                  break;

               case NRX_CTRL_SCB_ERROR_CFM:
                  /* this shouldn't be needed; by the time we ask for scb_error_req we have 
                     set NANOC_HW_COREDUMP. However in case of a race it's not a bad idea to
                     handle the CFM and let the coredump sm deal with it. Otherwise we lose
                     the core.
                  */
                  coredump_sm(hw, pdu);
                  break;
                  
               default:
                  de_error(DE_TR_NANOC, "WARNING! Unhandled message (type %d, id %d)", hdr->type, hdr->id);
            }
            de_pdu_free(pdu);
         } else if (hdr->type == HIC_MESSAGE_TYPE_DEBUG && hdr->id == NRX_DEBUG_LOG_IND) {
            nrx_debug_log_ind_t *ind = de_pdu_pull(pdu, sizeof *hdr);
            de_info(DE_TR_NANOC, "FW: [%08x] 0x%08x 0x%08x", ind->id, ind->value_1, ind->value_2);
            de_pdu_free(pdu);
         } else {
            de_error(DE_TR_NANOC, "WARNING! Unhandled message (type %d, id %d)", hdr->type, hdr->id);
            de_pdu_free(pdu);
         }
      }
   } else {
      de_debug(DE_TR_NANOC, "Routing (net=%d type=%d id=%d)", net_id, hdr->type, hdr->id);
      nanoc_recv_handler(hw, net_id, pdu);
   }
}

/**
 * Called from transport driver when a buffer has been successfully transmitted
 *
 * Context: any-thread
 */
void nanoc_hw_send_complete(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_hic_frame_t *hdr = (nrx_hic_frame_t *)de_pdu_data(pdu);

   if ((hw->flags & NANOC_HW_COREDUMP) && coredump_confirm_pdu(hw, pdu)) {
      de_pdu_free(pdu);
   } else if (hdr->control.net_id == NRX_NET_ID_UNKNOWN) {
      de_pdu_free(pdu);
   } else {
      de_pdu_pull(pdu, sizeof(uint16_t));
      nanoc_send_complete(hw, hdr->control.net_id, pdu);
   }
}

int nanoc_hw_chip_up(struct nanoc_hw *hw, enum nanoc_feat feature_mask)
{
   int fw_id;

   de_info(DE_TR_NANOC, "feature_mask = %#x", feature_mask);

   if (hw->num_clients != 0)
   {
      if ((fw_table[hw->fw_id].feature_mask & feature_mask) == feature_mask) {
         /* The loaded fw does already support the features requested */
         de_info(DE_TR_NANOC, "Matching firmware already loaded");
         hw->num_clients++;
         return 1;
      }
      /* The loaded fw does not support all the requested features */
      return -1;
   }

   fw_id = get_fw_id(hw, feature_mask);
   if (!fw_load_enable || fw_id >= 0) {
      if (hw->ops->hw_shutdown) {
         hw->ops->hw_shutdown(hw, 0);
      }
      if (chip_init(hw) != 0) {
         return -3;
      }
      hw->fw_id = fw_id;
      /* Download the "real" firmware */
      if (chip_download_fw(hw, hw->fw_id) != 0) {
         return -2;
      }
      hw->num_clients++;
      de_msleep(fw_startup_time_ms);
      chip_start_rx(hw);
      send_cmds(hw);
      return 0;
   }
   /* No fw available with the requested features */
   return -2;
}

int nanoc_hw_chip_down(struct nanoc_hw *hw)
{
   if (--hw->num_clients == 0) {
      hw->fw_id = 0;
      nanoc_link_down(hw, NANOC_QALL);
      chip_stop_rx(hw);
      if (chip_init(hw) != 0) {
         return -1;
      }
      if (hw->ops->hw_shutdown) {
         hw->ops->hw_shutdown(hw, 1);
      } else {
         nanoc_hw_sleep_forever(hw);
      }
      return 0;
   }
   return 1;
}

int nanoc_hw_if_down(struct nanoc_hw *hw)
{
   de_pdu_t *pdu = de_pdu_alloc(sizeof(nrx_ctrl_interface_down_t));
   ((nrx_ctrl_interface_down_t *)de_pdu_put(pdu, sizeof(nrx_ctrl_interface_down_t)))->reserved = 0;
   nanoc_hw_add_hic_header(hw, HIC_MESSAGE_TYPE_CTRL, NRX_CTRL_INTERFACE_DOWN_REQ, pdu);
   return nanoc_hw_send(hw, pdu);
}

/* Local functions ========================================== */

static void
handle_command_timeout(de_timer_t *t)
{
   struct nanoc_hw *hw = container_of(t, struct nanoc_hw, cmd_timer);
   de_error(DE_TR_NANOC, "Command timeout hw=%p, cmd_idx=%d", hw, hw->cmd_idx);
}

static int read_cis_bytes(struct nanoc_hw *hw, unsigned *value, unsigned addr, size_t n)
{
   unsigned result = 0;
   int      err = 0;
   unsigned tmp;

   while (n-- > 0) {
      if ((err = hw->ops->hw_getreg(hw, addr+n, &tmp)) < 0) {
         break;
      }
      result <<= 8;
      result |= tmp;
   }

   *value = result;

   return err;
}

#ifndef NRX_CONFIG_SDIO_CLK_FW_DOWNLOAD
#define NRX_CONFIG_SDIO_CLK_FW_DOWNLOAD 12000000
#endif

static const struct nanoc_device_descr ndd_nrx701 = {
   .ndd_variant = NRX701,
   .ndd_features =  NANOC_DEVICE_FEAT_WIFI
                   |NANOC_DEVICE_FEAT_BROKEN_TX_POWER,
   .ndd_download_speed = NRX_CONFIG_SDIO_CLK_FW_DOWNLOAD,
   .ndd_coredump_speed = 0,
   .ndd_tx_window      = 8*1688
};

static const struct nanoc_device_descr ndd_nrx600 = {
   .ndd_variant = NRX615,
   .ndd_features =  NANOC_DEVICE_FEAT_WIFI
                   |NANOC_DEVICE_FEAT_HT20
                   /*|NANOC_DEVICE_FEAT_STBC_RX*/
                   |NANOC_DEVICE_FEAT_AMPDU_TX,
   .ndd_download_speed = NRX_CONFIG_SDIO_CLK_FW_DOWNLOAD,
   .ndd_coredump_speed = 0,
   .ndd_tx_window      = 8*1676
};

static const struct nanoc_device_descr ndd_nrx900 = {
   .ndd_variant = NRX900,
   .ndd_features =  NANOC_DEVICE_FEAT_WIFI
                   |NANOC_DEVICE_FEAT_HT20
                   |NANOC_DEVICE_FEAT_STBC_RX
                   |NANOC_DEVICE_FEAT_AMPDU_TX
                   |NANOC_DEVICE_FEAT_BT
                   /*|NANOC_DEVICE_FEAT_5GHZ*/,
   .ndd_download_speed = NRX_CONFIG_SDIO_CLK_FW_DOWNLOAD,
   .ndd_coredump_speed = NRX_CONFIG_SDIO_CLK_FW_DOWNLOAD,
#ifdef NRX_CONFIG_AMPDU_16K
   .ndd_tx_window      = 28*1024
#else
   /* AMPDU_8K */
   .ndd_tx_window      = 14*1024
#endif
};

static int chip_identify(struct nanoc_hw *hw)
{
   unsigned   cis_base, cis_ptr;
   unsigned   vendor = 0;
   unsigned   mask_id = 0;
   unsigned   tpcode, tplen;
   int        err;

   err = read_cis_bytes(hw, &cis_base, 0x09, 3);
   if (err < 0)
      return err;

   de_debug(DE_TR_NANOC, "cis_base = %04x", cis_base);

   cis_ptr = cis_base;
   do {
      err = hw->ops->hw_getreg(hw, cis_ptr+0, &tpcode);
      err = hw->ops->hw_getreg(hw, cis_ptr+1, &tplen);
      de_debug(DE_TR_NANOC, "cis_tpl[%04x] = 0x%02x:%u", cis_ptr, tpcode, tplen);
      if (tpcode == 0x20) {
         if ((err = read_cis_bytes(hw, &vendor, cis_ptr+2, 2)) < 0)
            break;
         if ((err = read_cis_bytes(hw, &mask_id, cis_ptr+4, 2)) < 0)
            break;
      }
      cis_ptr += 2 + tplen;
   } while (cis_ptr-cis_base < 0x100 && tpcode != 0xFF);

   if (err < 0)
      return err;

   if (vendor == SDIO_VENDOR_ID_NANORADIO) {
      switch (mask_id & DEVICE_ID_CHIP_MASK) {
         case SDIO_DEVICE_ID_NANORADIO_NRX701:
            hw->device_descr = &ndd_nrx701;
            hw->chip_ver     = 0;
            de_info(DE_TR_NANOC, "Nanoradio NRX701");
         break;
         case SDIO_DEVICE_ID_NANORADIO_NRX615:
            hw->device_descr = &ndd_nrx600;
            hw->chip_ver = mask_id & DEVICE_ID_REV_MASK;
            de_info(DE_TR_NANOC, "Nanoradio NRX615 rev %02x", mask_id & DEVICE_ID_REV_MASK);
         break;
         case SDIO_DEVICE_ID_NANORADIO_NRX900C:
         case SDIO_DEVICE_ID_NANORADIO_NRX900D:
         case SDIO_DEVICE_ID_NANORADIO_NRX900EF:
         case SDIO_DEVICE_ID_NANORADIO_NRX900G:
            hw->device_descr = &ndd_nrx900;
            hw->chip_ver = mask_id & DEVICE_ID_REV_MASK;
            de_info(DE_TR_NANOC, "Nanoradio NRX900 rev %02x", mask_id & DEVICE_ID_REV_MASK);
         break;
         default:
            de_warn(DE_TR_NANOC, "Unrecognized device (vendor=%#x mask_id=%#x)", vendor, mask_id);
            err = -1;
         break;
      }
   } else {
      de_warn(DE_TR_NANOC, "Unrecognized device (vendor=%#x mask_id=%#x)", vendor, mask_id);
      err = -1;
   }
   if(hw->device_descr != NULL) {
      hw->download_speed = hw->device_descr->ndd_download_speed;
      hw->coredump_speed = hw->device_descr->ndd_coredump_speed;
   }

   return err;
}

static int chip_reset(struct nanoc_hw *hw)
{
   int err = 0;
#ifdef RESET_POWER_CYCLE
   de_info(DE_TR_NANOC, "Reset via POWER");
   hw->ops->hw_power(hw, 0);
   de_msleep(100);
   hw->ops->hw_power(hw, 1);
#else
   /*
    * NRX701: Reset bits in NRX_CCCR_RESET are active low.
    * NRX600/900: Reset bits in NRX_CCCR_RESET are active high.
    */
   de_info(DE_TR_NANOC, "Reset via CCCR. Trying different reset mechanisms. A SDIO error may follow.");
   hw->ops->hw_setreg(hw, NRX_CCCR_RESET, 0x00);
   hw->ops->hw_setreg(hw, NRX_CCCR_RESET, 0x03);
#endif

   /* Await reset of chip - measured to approx 8 ms */
   de_msleep(200);

   return err;
}

static int chip_init(struct nanoc_hw *hw)
{
   const unsigned sdio_cfg = 0
#ifndef NRX_CONFIG_SDIO_1BIT
   | NANOC_INIT_4BIT
#endif
#ifdef NRX_CONFIG_SDIO_HS
   | NANOC_INIT_HS
#endif
   ;

   hw->flags &= ~(NANOC_HW_IDLE|NANOC_HW_WAKEUP);

   de_info(DE_TR_NANOC, "flags=%#x", hw->flags);

   if (fw_load_enable) {
      chip_reset(hw);
   }

   nanoc_tp_setspeed(hw, hw->download_speed);

   hw->ops->hw_init(hw, sdio_cfg);

   if (hw->device_descr == NULL) {
      if (chip_identify(hw) < 0) {
         de_error(DE_TR_NANOC, "Failed to read SDIO CIS");
         /* This is ok in the uart/tty case */
         return -1;
      }
   }

   nanoc_tp_setspeed(hw, hw->default_speed);

   return 0;
}

static void chip_start_rx(struct nanoc_hw *hw)
{
   /* Ack pending interrupt */
   hw->ops->hw_setreg(hw, NRX_CCCR_INTACK, 0x06);
   /* Call transport driver to start RX */
   hw->ops->hw_start_rx(hw);
   /* Enable interrupt generation without clock */
   hw->ops->hw_setreg(hw, NRX_CCCR_NOCLK_INT, 0x01);
   /* Clear wakeup bit */
   hw->ops->hw_wakeup(hw, 0);
}

static void chip_stop_rx(struct nanoc_hw *hw)
{
   /* Disable SDIO interrupt */
   hw->ops->hw_stop_rx(hw);
   hw->ops->hw_setreg(hw, NRX_CCCR_IENx, 0x00);
   /* Ack interrupts that may have come in after we did stop_rx but before we disabled interrupt */
   hw->ops->hw_setreg(hw, NRX_CCCR_INTACK, 0x06);
}

#ifdef NRX_CONFIG_EMBED_FW
/* Fw is stored in an array: const uint8_t fw[] */
static int chip_download_fw(struct nanoc_hw *hw, unsigned fw_id)
{
   int ret = 0;

   if ((fw_table[fw_id].feature_mask & NANOC_FEAT_INIT) == 0) {
      if (fw_load_enable) {
         size_t i;
         de_info(DE_TR_NANOC, "Downloading embedded fw");
         nanoc_tp_setspeed(hw, hw->download_speed);
         for (i = 0; i < sizeof(fw); i+=DE_FW_CHUNK_SIZE)
         {
            size_t size = sizeof(fw)-i;
            if (size > DE_FW_CHUNK_SIZE) size = DE_FW_CHUNK_SIZE;
            chip_download_blob(hw, &fw[i], size);
         }
         nanoc_tp_setspeed(hw, hw->default_speed);
      } else {
         de_warn(DE_TR_NANOC, "Firmware download disabled by configuration.");
      }
   }

   return ret;
}
#else
/* Read FW from file */
static int chip_download_fw(struct nanoc_hw *hw, unsigned fw_id)
{
   int ret = 0;

   if (fw_load_enable) {
      de_pdu_t * buf;
      de_fw_ref ref = de_fw_open(hw->dev, fw_table[fw_id].filename);
      if (ref) {
         de_info(DE_TR_NANOC, "Downloading fw %s", fw_table[fw_id].filename);
         nanoc_tp_setspeed(hw, hw->download_speed);
         while ((buf = de_fw_get_chunk(ref)))
         {
            if (hw->ops->hw_download(hw, buf) == 0) {
               de_fw_free_chunk(ref, buf);
            } else {
               de_info(DE_TR_NANOC, "Aborting fw download");
               break;
            }
         }
         de_fw_close(ref);
         nanoc_tp_setspeed(hw, hw->default_speed);
      } else {
         ret = -1;
      }
   } else {
      de_warn(DE_TR_NANOC, "Firmware download disabled by configuration.");
   }

   return ret;
}
#endif

void
nanoc_hw_add_hic_header(struct nanoc_hw *hw, unsigned type, unsigned msgid, de_pdu_t *pdu)
{
   nrx_hic_message_t *hdr = de_pdu_push(pdu, sizeof *hdr);

   hdr->length   = de_pdu_size(pdu);
   hdr->type     = type;
   hdr->id       = msgid;
   hdr->flags    = 0;
   hdr->net_id   = NRX_NET_ID_UNKNOWN;
   hdr->trans_id = hw->trans_id++;
   hdr->tx_window_non_wifi_data = 0;
   hdr->tx_window = 0;
   nanoc_hw_add_framing_hdr(hw, pdu);
}

void
nanoc_hw_add_framing_hdr(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   size_t           frame_size  = sizeof(uint16_t) + de_pdu_size(pdu);
   size_t           padded_size = frame_size;
   nrx_hic_frame_t *hdr;

   if (hw->sz_modulo_from_host) {
      padded_size = (frame_size + hw->sz_modulo_from_host - 1) & ~(hw->sz_modulo_from_host-1);
      if (frame_size != padded_size) {
         de_pdu_pad(pdu, padded_size - frame_size);
      }
   }
   hdr = de_pdu_push(pdu, sizeof(uint16_t));
   hdr->size = padded_size - sizeof(hdr->size);
   de_debug(DE_TR_WIFI, "frame_size=%zu padding=%zu", padded_size, padded_size - frame_size);
}

int nanoc_hw_send(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nanoc_hiclog("TX", pdu);
#ifdef NRX_COMPONENT_HIC_PCAP_BASE
   hic_pcap_add_pdu(pdu);
#endif
   return hw->ops->hw_send(hw, pdu);
}

/* Override min_irq_interval from transport driver. Mainly used for test. Don't use 0xFF. */
void nanoc_hw_min_recv_interval(uint8_t tenths_ms)
{
   recv_interval = tenths_ms;
}

static void nanoc_hw_set_req_coredump(const char *val)
{
   unsigned i;
   de_info(DE_TR_NANOC, "Requesting coredump by module parameter");
   /* Request coredump on all hws */
   for (i = 0; i < MAX_NUM_HWS; i++) {
      if (hw_array[i] != NULL) {
         nanoc_commit_suicide(hw_array[i]);
      }
   }
}

static void nanoc_hw_get_req_coredump(char *buffer)
{
   de_warn(DE_TR_NANOC, "Reading req_coredump has no effect");
}

/* Local functions ================================= */

static void send_next_cmd(struct nanoc_hw *hw)
{
   de_pdu_t        *pdu = cmd_array[hw->cmd_idx].create_cmd(hw);
   nrx_hic_frame_t *hdr = (nrx_hic_frame_t *)de_pdu_data(pdu);
   hw->cmd_trans_id = hdr->control.trans_id;
   if (command_timeout != 0) {
      de_timer_start(&hw->cmd_timer, command_timeout);
   }
   nanoc_hw_send(hw, pdu);
}

static void send_cmds(struct nanoc_hw *hw)
{
   hw->cmd_idx = 0;
   hw->trans_id = 1;
   send_next_cmd(hw);
}

static unsigned recv_cmds(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_hic_message_t *hdr = (nrx_hic_message_t *)de_pdu_data(pdu);

   if (hw->cmd_trans_id == hdr->trans_id) {
      /* Call handler function */
      de_timer_stop(&hw->cmd_timer);
      if (cmd_array[hw->cmd_idx].recv_cmd(hw, pdu) == 0) {
         if (++hw->cmd_idx < sizeof(cmd_array)/sizeof(cmd_array[0])) {
            send_next_cmd(hw);
         } else {
            /* We're done */
            int restart = !!(hw->flags & NANOC_HW_RESTARTING);
            
            de_mutex_lock(&hw->mutex);
            hw->flags &= ~NANOC_HW_RESTARTING;
            hw->cmd_trans_id = ~0; /* XXX, we don't want to match this again */
            de_mutex_unlock(&hw->mutex);
            
            /* if this is a restart, tell all attached drivers */
            if(restart)
               nanoc_restart(hw);
            
            nanoc_link_up(hw, NANOC_QALL);
         }
      } else {
         de_warn(DE_TR_NANOC, "Chip initialization failed, aborting!");
      }
      de_pdu_free(pdu);
      return 1;
   }
   return 0;
}

static de_pdu_t * create_set_alignment(struct nanoc_hw *hw)
{
   de_pdu_t                     *pdu = de_pdu_alloc(sizeof(nrx_ctrl_set_alignment_req_t));
   nrx_ctrl_set_alignment_req_t *msg = (nrx_ctrl_set_alignment_req_t *) de_pdu_put(pdu, sizeof *msg);

   msg->min_sz           = hw->min_sz_to_host;
   msg->sz_modulo        = hw->sz_modulo_to_host;
   msg->reserved_1       = 0;
   msg->swap             = 0;
   msg->irq_gpio_id      = hw->irq_gpio_id;
   msg->wakeup_gpio_id   = hw->wakeup_gpio_id;
   msg->min_irq_interval = (recv_interval != 0) ? recv_interval : hw->min_irq_interval;
   msg->tx_window_bytes  = hw->device_descr->ndd_tx_window;
   msg->block_size       = 0;
   msg->reserved_2       = 0;

   nanoc_hw_add_hic_header(hw, HIC_MESSAGE_TYPE_CTRL, NRX_CTRL_SET_ALIGNMENT_REQ, pdu);
   return pdu;
}

static int recv_set_alignment(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_ctrl_set_alignment_cfm_t *cfm;

   de_pdu_pull(pdu, sizeof(nrx_hic_message_t)); /* Remove HIC header */
   cfm = (nrx_ctrl_set_alignment_cfm_t *)de_pdu_data(pdu);

   if (cfm->result == NRX_SUCCESS && cfm->tx_buf_size != 0 && cfm->tx_num_bufs != 0) {
      size_t cmd_max;
      de_debug(DE_TR_NANOC, "Set alignment confirm received");
      hw->tx_buf_size = cfm->tx_buf_size;
      cmd_max = nanoc_get_num_bufs(hw, HIC_CMD_MAX_SIZE);
      hw->tx_win[NANOC_QCMD].max  = cmd_max;
      hw->tx_win[NANOC_QDATA].max = cfm->tx_num_bufs - cmd_max;
      return 0;
   } else {
      de_warn(DE_TR_NANOC, "Set alignment confirm result is no good");
      return -1;
   }
}

static de_pdu_t * create_set_mac_addr(struct nanoc_hw *hw)
{
   de_pdu_t *pdu = create_mib_set_req(hw,
                                      NRX_MIB_ID_DOT11_MAC_ADDRESS,
                                      hw->mac_addr,
                                      sizeof(hw->mac_addr));
   de_info(DE_TR_NANOC, "Setting mac addr %02x-%02x-%02x-%02x-%02x-%02x",
           hw->mac_addr[0], hw->mac_addr[1], hw->mac_addr[2],
           hw->mac_addr[3], hw->mac_addr[4], hw->mac_addr[5]);

   return pdu;
}

static de_pdu_t * create_get_fw_version(struct nanoc_hw *hw)
{
   de_pdu_t               *pdu = de_pdu_alloc(sizeof(nrx_ctrl_version_req_t));
   nrx_ctrl_version_req_t *msg = de_pdu_put(pdu, sizeof *msg);

   msg->reserved = 0;
   nanoc_hw_add_hic_header(hw, HIC_MESSAGE_TYPE_CTRL, NRX_CTRL_VERSION_REQ, pdu);
   return pdu;
}

static int recv_fw_version(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_ctrl_version_cfm_t *cfm;

   de_pdu_pull(pdu, sizeof(nrx_hic_message_t)); /* Remove HIC header */
   cfm = de_pdu_data(pdu);

   strncpy(fw_version, cfm->fwbuild, sizeof fw_version);
   snprintf(chip_version, sizeof chip_version, "0x%08x", cfm->chipid);
   de_info(DE_TR_NANOC, "Firmware: %s", cfm->fwbuild);
   de_info(DE_TR_NANOC, "Chip: 0x%08x", cfm->chipid);
   hw->api_version = cfm->version;
   if (cfm->version != NRX_API_VERSION) {
      de_warn(DE_TR_WIFI, "Warning! Different mac_api versions between fw (ver %d) and driver (ver %d).", cfm->version, NRX_API_VERSION);
      return -1;
   }
   return 0;
}

static de_pdu_t * create_init_completed(struct nanoc_hw *hw)
{
   de_pdu_t                      *pdu = de_pdu_alloc(sizeof(nrx_ctrl_init_completed_req_t));
   nrx_ctrl_init_completed_req_t *msg = (nrx_ctrl_init_completed_req_t *) de_pdu_put(pdu, sizeof *msg);

   msg->reserved = 0;
   nanoc_hw_add_hic_header(hw, HIC_MESSAGE_TYPE_CTRL, NRX_CTRL_INIT_COMPLETED_REQ, pdu);
   return pdu;
}

static int recv_init_completed(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_ctrl_init_completed_cfm_t *cfm;

   de_pdu_pull(pdu, sizeof(nrx_hic_message_t)); /* Remove HIC header */
   cfm = (nrx_ctrl_init_completed_cfm_t *)de_pdu_data(pdu);

   if (cfm->result == NRX_SUCCESS) {
      de_debug(DE_TR_NANOC, "Init completed confirm received");
      return 0;
   } else {
      de_warn(DE_TR_NANOC, "Init completed result is no good");
      return -1;
   }
}

static de_pdu_t * create_mib_set_req(struct nanoc_hw *hw,
                                     nrx_mib_id_t id,
                                     const void *data,
                                     size_t size)
{
   nrx_mib_set_req_t *req;

   de_pdu_t *pdu = de_pdu_alloc(sizeof(*req) + size);
   req = (nrx_mib_set_req_t*)de_pdu_put(pdu, sizeof(*req) + size);

   req->id = id;
   req->size = size;
   memcpy(&req[1], data, size);

   nanoc_hw_add_hic_header(hw,
                           HIC_MESSAGE_TYPE_MIB,
                           NRX_MIB_SET_REQ,
                           pdu);
   return pdu;
}

static int recv_mib_set_cfm(struct nanoc_hw *hw, de_pdu_t *pdu)
{
   nrx_mib_set_cfm_t *cfm;

   de_pdu_pull(pdu, sizeof(nrx_hic_message_t)); /* Remove HIC header */
   cfm = (nrx_mib_set_cfm_t*)de_pdu_data(pdu);

   if (cfm->result == MIB_RESULT_OK) {
      de_debug(DE_TR_NANOC, "MIB (%u) set complete", cfm->id);
      return 0;
   } else {
      de_warn(DE_TR_NANOC, "Failed to set MIB (%u), result = %u",
              cfm->id,
              cfm->result);
      return -1;
   }
}

#ifdef NRX_CONFIG_HIC_AGGREGATION
static de_pdu_t * create_mib_set_hic_aggr_filter(struct nanoc_hw *hw)
{
   uint8_t filter[HIC_MESSAGE_TYPE_NUM_TYPES * 128 / (8*sizeof(uint8_t))];
   size_t len = sizeof(filter);

   /* The filter is a bitmask with HIC_MESSAGE_TYPE_NUM_TYPES by 128
    * bits, one bit per nrx_hic_message_id_t & 0x7f, sequences of
    * messages that are in the set of asserted bits will be aggregated
    * into one packet, given size constraint of max size below.
    *
    * For now, let's aggregate everything.
    */
   memset(filter, 0xff, sizeof(filter));

   return create_mib_set_req(hw,
                             NRX_MIB_ID_DOT11_PRIVATE_AGGREGATION_FILTER,
                             filter,
                             len);
}

static de_pdu_t * create_mib_set_hic_aggr_size(struct nanoc_hw *hw)
{
   uint32_t size = 1600;

   return create_mib_set_req(hw,
                             NRX_MIB_ID_DOT11_PRIVATE_AGGREGATION_MAX_SIZE,
                             &size,
                             sizeof(size));
}
#endif /* NRX_CONFIG_HIC_AGGREGATION */

static void nanoc_hw_sleep_forever(struct nanoc_hw *hw)
{
   nanoc_tp_setspeed(hw, hw->download_speed);
   switch (hw->device_descr->ndd_variant) {
      case NRX701:
         chip_download_blob(hw, sleep_forever_700, sizeof(sleep_forever_700));
         break;
      case NRX615:
         chip_download_blob(hw, sleep_forever_600, sizeof(sleep_forever_600));
         break;
      case NRX900:
         chip_download_blob(hw, sleep_forever_900, sizeof(sleep_forever_900));
         break;
      default:
         de_warn(DE_TR_NANOC, "Unknown chip variant");
   }
   nanoc_tp_setspeed(hw, hw->default_speed);
}

static int get_fw_id(struct nanoc_hw *hw, enum nanoc_feat feature_mask)
{
   unsigned i;
   for (i = 0; i < sizeof(fw_table)/sizeof(fw_table[0]); i++) {
      if (fw_table[i].chip_variant == hw->device_descr->ndd_variant
          && (fw_table[i].feature_mask & feature_mask) == feature_mask) {
          return i;
       }
   }
   return -1;
}

static int chip_download_blob(struct nanoc_hw *hw, const void *blob, size_t size)
{
   int ret = 0;

   if (fw_load_enable) {
      de_pdu_t *pdu = de_pdu_alloc(size);
      memcpy(de_pdu_put(pdu, size), blob, size);
      ret = hw->ops->hw_download(hw, pdu);
      if (ret == 0) {
         de_pdu_free(pdu);
      }
   }

   return ret;
}
