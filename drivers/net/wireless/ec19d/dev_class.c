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

#include <linux/device.h>
#include "driverenv.h"
#include "dev_class.h"

/* Variables ======================================== */

static struct class *nanocore_class;
static unsigned      users;

/* Global functions ==================================== */

struct class * dev_class_create(void)
{
   if (nanocore_class == NULL) {
      if ((nanocore_class = class_create(THIS_MODULE, "nanocore")) == NULL) {
         de_error(DE_TR_NANOC, "Failed to register device class 'nanocore'");
         return NULL;
      }
      de_info(DE_TR_NANOC, "Created nanocore device class");
   }
   users++;
   return nanocore_class;
}

void dev_class_remove(void)
{
   if (nanocore_class && --users == 0) {
      class_destroy(nanocore_class);
   }
}
