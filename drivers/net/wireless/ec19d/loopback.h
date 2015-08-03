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

#ifndef __LOOPBACK_H__
#define __LOOPBACK_H__

/* Module parameters ================================= */
extern int loopback_size;
extern int loopback_num_errors;
extern int loopback_num_echoes;
extern int loopback_tx_kbytes;
extern int loopback_rx_kbytes;
extern int loopback_tx_kbitrate;
extern int loopback_rx_kbitrate ;

void loopback_init(void);
void loopback_set_clear_stat(const char *val);
void loopback_get_clear_stat(char *buffer);

#endif /* __LOOPBACK_H__ */
