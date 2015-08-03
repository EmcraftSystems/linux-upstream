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

#ifndef __COREDUMP__
#define __COREDUMP__

void coredump_init(struct nanoc_hw *hw);
void coredump_start_timer(struct nanoc_hw *hw);
int  coredump_confirm_pdu(struct nanoc_hw *hw, de_pdu_t *pdu);
int  coredump_commit_suicide(struct nanoc_hw *hw);
void coredump_sm(struct nanoc_hw *hw, de_pdu_t *pdu);

#endif
