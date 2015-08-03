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
#include <linux/ctype.h>

/* we know that CH isxdigit, so this can be stupid */
#define HEXVALUE(CH) (((CH) & 0x10) ? ((CH) & 0x0f) : ((CH) & 0x0f) + 9)

/* This function parses a string as a mac-address. It accepts single
 * or double hex digits separated with either colon, dash or period,
 * and double hex digits without separators, leading and trailing
 * white space is ignored.
 *
 * Return zero on success, or -EINVAL on malformed strings.
 *
 * Example input:
 *   a-b-c-d-e-f
 *   aa-bb-cc-dd-ee-ff
 *   aa:b:c:d:ee:ff
 *   aabbccddeeff
 *   aabb.ccdd.eeff
 *
 * Degenerate forms are also accepted:
 *   aabb-cc:dd.eef
 */
static int smd_parse_macaddr(const char *string, u8 result[ETH_ALEN])
{
   u8 addr[ETH_ALEN];
   unsigned int index = 0;

   /* skip leading white space */
   while(isspace(*string))
      string++;

   while(index < ETH_ALEN) {
      unsigned int value;

      /* we expect at least one hex digit ... */
      if(!isxdigit(*string))
         return -EINVAL;

      value = HEXVALUE(*string);
      string++;

      /* ... but there can be one more */
      if(isxdigit(*string)) {
         value = value * 16 + HEXVALUE(*string);
         string++;
      }
      addr[index++] = value;
      if(index < ETH_ALEN) {
         /* skip optional separator */
         if(*string == ':' || *string == '-' || *string == '.') {
            string++;
         }
      }
   }
   /* skip trailing white space */
   while(isspace(*string))
      string++;

   /* we should be at end of string */
   if(*string != '\0')
      return -EINVAL;

   memcpy(result, addr, ETH_ALEN);

   return 0;
}
#undef HEXVALUE

static int param_set_macaddr(const char *val, const struct kernel_param *kp)
{
   return smd_parse_macaddr(val, kp->arg);
}

static int param_get_macaddr(char *buffer, const struct kernel_param *kp)
{
   return scnprintf(buffer, 4096, "%pMF", kp->arg);
}

const struct kernel_param_ops smd_param_macaddr_ops = {
   .flags = 0,
   .set = param_set_macaddr,
   .get = param_get_macaddr,
   .free = NULL,
};
