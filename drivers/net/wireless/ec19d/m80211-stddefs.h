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

#ifndef IEEE80211_STDDEFS_H
#define IEEE80211_STDDEFS_H

#define M80211_GET_FTYPE(_fc)          (((_fc) >> 2) & 0x3)
#define M80211_GET_STYPE(_fc)          (((_fc) >> 4) & 0xF)

#define M80211_FRAMEGROUPTYPE_MGMT     0x00
#define M80211_FRAMEGROUPTYPE_CTRL     0x01
#define M80211_FRAMEGROUPTYPE_DATA     0x02
#define M80211_FRAMEGROUPTYPE_RESERVED 0x03

/* Management subtype field values */
#define M80211_FRAMESUBTYPE_MGMT_ASSOCREQ        0
#define M80211_FRAMESUBTYPE_MGMT_ASSOCRSP        1
#define M80211_FRAMESUBTYPE_MGMT_REASSOCREQ      2
#define M80211_FRAMESUBTYPE_MGMT_REASSOCRSP      3
#define M80211_FRAMESUBTYPE_MGMT_PROBEREQ        4
#define M80211_FRAMESUBTYPE_MGMT_PROBERSP        5
#define M80211_FRAMESUBTYPE_MGMT_MEASUREMENT_PILOT 6 /* 802.11k */
#define M80211_FRAMESUBTYPE_MGMT_BEACON          8
#define M80211_FRAMESUBTYPE_MGMT_ATIM            9
#define M80211_FRAMESUBTYPE_MGMT_DISASSOCIATE   10
#define M80211_FRAMESUBTYPE_MGMT_AUTHENTICATE   11
#define M80211_FRAMESUBTYPE_MGMT_DEAUTHENTICATE 12
#define M80211_FRAMESUBTYPE_MGMT_ACTION         13
#define M80211_FRAMESUBTYPE_MGMT_ACTION_NO_ACK  14 /* 802.11n */
#define M80211_FRAMESUBTYPE_MAX_VAL 0x0F

/* Control subtype field values */
#define M80211_FRAMESUBTYPE_CTRL_BLOCK_ACK_REQ          0x08
#define M80211_FRAMESUBTYPE_CTRL_BLOCK_ACK              0x09
#define M80211_FRAMESUBTYPE_CTRL_PSPOLL                 0x0A
#define M80211_FRAMESUBTYPE_CTRL_RTS                    0x0B
#define M80211_FRAMESUBTYPE_CTRL_CTS                    0x0C
#define M80211_FRAMESUBTYPE_CTRL_ACK                    0x0D
#define M80211_FRAMESUBTYPE_CTRL_CFEND                  0x0E
#define M80211_FRAMESUBTYPE_CTRL_CFEND_CFACK            0x0F

/* Data subtype field values */
#define M80211_FRAMESUBTYPE_DATA_DATA      0x00
#define M80211_FRAMESUBTYPE_DATA_NULL      0x04
#define M80211_FRAMESUBTYPE_QOS_DATA       0x08
#define M80211_FRAMESUBTYPE_QOS_NULL       0x0C
#define M80211_FRAMESUBTYPE_QOSMGMT_ACTION 0x0D

#endif
