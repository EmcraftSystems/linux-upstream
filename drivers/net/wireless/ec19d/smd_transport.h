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

#ifndef __smd_transport_h__
#define __smd_transport_h__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>

struct smd_transport {
   int (*tp_reset)(struct device *dev);
   int (*tp_download)(struct device *dev, const void *buf, size_t size);
   int (*tp_send)(struct device *dev, struct sk_buff *buf);
   int (*tp_sleep)(struct device *dev);
   int (*tp_wakeup)(struct device *dev);
   int (*tp_debug)(struct device*, const char*, const void*, size_t);
};

#endif
