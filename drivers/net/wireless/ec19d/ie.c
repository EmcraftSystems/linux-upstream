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

static bool smd_ie_is_valid(const u8 *ies, size_t ie_len, const u8 *ie)
{
   BUG_ON(!(ie >= ies && ie <= &ies[ie_len]));

   return &ies[ie_len] - ie >= 2 && &ies[ie_len] - ie >= 2 + ie[1];
}

/*!
 * @brief find an IE with a given ID in a buffer containing multiple IE:s
 *
 * @param [in] ies    the blob of IEs to search
 * @param [in] ie_len the size of blob
 * @param [in] ie_id  the IE ID to look for, or 255 for any IE
 * @param [in] ref    should point to a preceeding IE, or NULL to get the
 *                    first IE
 *
 * @return a pointer to an information element or NULL if none was found
 *
 * It's possible to loop over all information elements in the blob
 * by passing a returned IE pointer as ref.
 */
const u8* smd_ie_find(const u8 *ies,
                      size_t ie_len,
                      u8 ie_id,
                      const u8 *ref)
{
   if(ref == NULL) {
      ref = ies;
   } else {
      if(smd_ie_is_valid(ies, ie_len, ref)) {
         ref += 2 + ref[1];
      }
   }
   while(smd_ie_is_valid(ies, ie_len, ref)) {
      if(ie_id == 255 || ie_id == ref[0])
         return ref;
      ref += 2 + ref[1];
   }
   return NULL;
}

/*!
 * @brief find an vendor specific IE with a given OUI+ID byte in a
 * buffer containing multiple IE:s
 *
 * @param [in] ies    the blob of IEs to search
 * @param [in] ie_len the size of blob
 * @param [in] oui    the vendor OUI to look for
 * @param [in] ref    should point to a preceeding IE, or NULL to get the
 *                    first IE
 *
 * @return a pointer to an information element or NULL if none was found
 *
 * The OUI is really a three byte OUI plus one ID byte (which is the
 * common format used). It should be passed in network byteorder with
 * something like cpu_to_be32(0x0050f201) for WPA.
 */
const u8* smd_ie_find_vendor(const u8 *ies,
                             size_t ie_len,
                             __be32 oui,
                             const u8 *ref)
{
   while((ref = smd_ie_find(ies,
                            ie_len,
                            WLAN_EID_VENDOR_SPECIFIC,
                            ref)) != NULL) {
      if(ref[1] >= 4 && memcmp(&oui, &ref[2], 4) == 0)
         return ref;
   }
   return NULL;
}
