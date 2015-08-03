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

#ifndef __NANOC__
#define __NANOC__

#define MAX_NUM_HWS 5
#define HIC_CMD_MAX_SIZE (1600-sizeof(nrx_hic_frame_size_t))

struct hw_handle;

void nanoc_recv_handler(struct nanoc_hw *hw, unsigned net_id, de_pdu_t *pdu);
void nanoc_new_device(struct nanoc_hw *hw);
void nanoc_del_device(struct nanoc_hw *hw);
void nanoc_send_complete(struct nanoc_hw *hw, unsigned net_id, de_pdu_t *pdu);
void nanoc_event(struct nanoc_hw *hw, void (*event_func)(struct hw_handle *, unsigned), unsigned arg);
void nanoc_ev_link_up(struct hw_handle *h, unsigned qmask);
void nanoc_ev_link_down(struct hw_handle *h, unsigned qmask);
void nanoc_ev_restart(struct hw_handle *h, unsigned arg);
void nanoc_link_down(struct nanoc_hw *hw, unsigned qmask);
void nanoc_link_up(struct nanoc_hw *hw, unsigned qmask);
void nanoc_restart(struct nanoc_hw *hw);
int  nanoc_hw_chip_up(struct nanoc_hw *hw, enum nanoc_feat feature_mask);
int  nanoc_hw_chip_down(struct nanoc_hw *hw);
unsigned nanoc_get_num_bufs(struct nanoc_hw *hw, unsigned size);
void nanoc_wakeup_ind(struct nanoc_hw *hw);
void nanoc_commit_suicide(struct nanoc_hw *hw);

/* Internal functions from nanoc_hw.c */
int  nanoc_hw_send(struct nanoc_hw *hw, de_pdu_t *pdu);
int  nanoc_hw_if_down(struct nanoc_hw *hw);
void nanoc_hw_add_hic_header(struct nanoc_hw *hw, unsigned type, unsigned msgid, de_pdu_t *pdu);
void nanoc_hw_add_framing_hdr(struct nanoc_hw *hw, de_pdu_t *pdu);


/* Module parameters */
extern int command_timeout;
extern int fw_load_enable;
extern int restart_after_coredump;
extern int coredump_enable;
extern int ifdown_enable;

/* To support testing */
void nanoc_hw_min_recv_interval(uint8_t tenths_ms);

#endif
