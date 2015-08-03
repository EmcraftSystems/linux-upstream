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

#ifndef __HIC_PCAP_H__
#define __HIC_PCAP_H__

void hic_pcap_start(void);
void hic_pcap_stop(void);
void hic_pcap_add_pdu(de_pdu_t *pdu);

/* Must be defined in the target/OS specific code */
void hic_pcap_hdlr_write(de_pdu_t *pdu);

void hic_pcap_debug_log_pdu(unsigned int id,
                            unsigned int value_1,
                            unsigned int value_2,
                            de_pdu_t *pdu,
                            unsigned int snaplen);

void hic_pcap_debug_log_data(unsigned int id,
                             unsigned int value_1,
                             unsigned int value_2,
                             const void *data,
                             size_t len);
#endif
