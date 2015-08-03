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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/etherdevice.h>
#include "driverenv.h"

#define attr_to_kobj(SA) (&(SA)->sa_de->de_dev->kobj)

#define SAINCREF(SA)   kref_get(&(SA)->sa_kref)
#define SADECREF(SA)   kref_put(&(SA)->sa_kref, de_attr_release)

#define DE_LOCK(_de)   spin_lock_irqsave(&(_de)->de_lock, (_de)->de_irqflags)
#define DE_UNLOCK(_de) spin_unlock_irqrestore(&(_de)->de_lock, (_de)->de_irqflags)

static unsigned core_idx;

struct de_attribute {
   struct de_obj        *sa_de;
   struct list_head      sa_list;
   struct kref           sa_kref;
   char                  sa_name[32];
   struct bin_attribute  sa_attr;
   size_t                sa_totalsize;
   unsigned int          sa_pageslots; /* length of pagearray */
   unsigned char        *sa_pagearray[0];
};

/*
 *===========================================================================
 * Attributes in sysfs
 *===========================================================================
 */

static void de_attr_release(struct kref *kref)
{
   struct de_attribute *sa = container_of(kref, struct de_attribute, sa_kref);
   unsigned int page_slot;

   might_sleep();

   DE_LOCK(sa->sa_de);
   list_del(&sa->sa_list);
   DE_UNLOCK(sa->sa_de);

   if(sa->sa_attr.read != NULL) {
      /* XXX should have flag */
      sysfs_remove_bin_file(attr_to_kobj(sa), &sa->sa_attr);
   }

   for(page_slot = 0;
       page_slot < sa->sa_pageslots && sa->sa_pagearray[page_slot] != NULL;
       page_slot++) {
      free_page((unsigned long)sa->sa_pagearray[page_slot]);
      sa->sa_pagearray[page_slot] = NULL;
   }

   kfree(sa);
}

static void
de_attr_free(struct de_attribute *sa)
{
   SADECREF(sa);
}

static int
de_attr_read(struct de_attribute *sa,
              void *buffer,
              loff_t offset,
              size_t count)
{
   unsigned int page_slot;
   unsigned char *page;
   loff_t page_offset;
   loff_t page_start;
   size_t copy_len;
   size_t copied = 0;

   if(offset < 0) {
      return 0;
   }

   while(offset < sa->sa_totalsize && count > 0) {
      page_slot = offset / PAGE_SIZE;
      page_start = page_slot * PAGE_SIZE;
      page_offset = offset - page_start;
      if(page_slot >= sa->sa_pageslots) {
         de_error(DE_TR_OS, "bad page slot %u", page_slot);
         break;
      }
      page = sa->sa_pagearray[page_slot];
      if(page == NULL) {
         de_error(DE_TR_OS, "bad page slot %u", page_slot);
         break;
      }
      copy_len = PAGE_SIZE - page_offset;
      if(copy_len > count)
         copy_len = count;
      if(offset + copy_len > sa->sa_totalsize)
         copy_len = sa->sa_totalsize - offset;

      memcpy(buffer, page + page_offset, copy_len);
      copied += copy_len;
      offset += copy_len;
      buffer = (unsigned char*)buffer + copy_len;
      count -= copy_len;
   }

   return copied;
}

static ssize_t
de_attr_sysfs_read(struct file *filep,
                   struct kobject *kobj,
                   struct bin_attribute *attr,
                   char *buffer,
                   loff_t offset,
                   size_t count)
{
   struct de_attribute *sa = container_of(attr, struct de_attribute, sa_attr);
   return de_attr_read(sa, buffer, offset, count);
}

static int
de_attr_write(struct de_attribute *sa,
               const void *buffer,
               loff_t offset,
               size_t count)
{
   unsigned char *page;
   unsigned int page_slot;
   loff_t page_offset;
   loff_t page_start;
   size_t copy_len;

   might_sleep();

   while(count > 0) {
      page_slot = offset / PAGE_SIZE;
      page_start = page_slot * PAGE_SIZE;
      page_offset = offset - page_start;

      if(page_slot >= sa->sa_pageslots)
         break;
      page = sa->sa_pagearray[page_slot];
      if(page == NULL) {
         page = (unsigned char*)__get_free_page(GFP_KERNEL);
         if(page == NULL) {
            /* XXX */
         }
         sa->sa_pagearray[page_slot] = page;
      }
      copy_len = PAGE_SIZE - page_offset;
      if(copy_len > count)
         copy_len = count;

      memcpy(page + page_offset, buffer, copy_len);
      offset += copy_len;
      buffer = (const unsigned char*)buffer + copy_len;
      count -= copy_len;
   }
   if(offset > sa->sa_totalsize)
      sa->sa_totalsize = offset;

   sa->sa_attr.size = sa->sa_totalsize;

   return 0;
}

static ssize_t
de_attr_sysfs_write(struct file *filep,
                    struct kobject *kobj,
                    struct bin_attribute *attr,
                    char *buffer,
                    loff_t offset,
                    size_t count)
{
   struct de_attribute *sa = container_of(attr, struct de_attribute, sa_attr);
   return de_attr_write(sa, buffer, offset, count);
}


static void *
de_attr_alloc(struct de_obj *de, size_t maxsize)
{
   const unsigned int slots = (maxsize + PAGE_SIZE - 1) / PAGE_SIZE;
   struct de_attribute *sa;

   sa = kzalloc(sizeof(*sa) + slots * sizeof(sa->sa_pagearray[0]), GFP_KERNEL);
   sa->sa_de = de;
   kref_init(&sa->sa_kref);
   INIT_LIST_HEAD(&sa->sa_list);

   DE_LOCK(de);
   list_add_tail(&sa->sa_list, &de->de_attrs);
   DE_UNLOCK(de);

   sa->sa_totalsize = 0;
   sa->sa_pageslots = slots;

   return sa;
}

/* set name attribute name, call before de_attr_complete */
static int
de_attr_setname(struct de_attribute *sa, const char *fmt, ...)
{
   int size;
   va_list ap;
   va_start(ap, fmt);
   size = vsnprintf(sa->sa_name, sizeof(sa->sa_name), fmt, ap);
   va_end(ap);

   return size >= 0 && size < sizeof(sa->sa_name);
}

/* finish creating the attribute, after this call, it will be
 * available in sysfs */
static int
de_attr_create(struct de_attribute *sa)
{
   sysfs_bin_attr_init(&sa->sa_attr);
   sa->sa_attr.attr.name = sa->sa_name;
   sa->sa_attr.attr.mode = (S_IRUSR|S_IRGRP|S_IROTH);
   sa->sa_attr.read = de_attr_sysfs_read;
   sa->sa_attr.write = (sa->sa_attr.attr.mode & S_IWUGO) != 0 ? de_attr_sysfs_write : NULL;
   sa->sa_attr.size = sa->sa_totalsize;
   sa->sa_attr.private = NULL;
   return sysfs_create_bin_file(attr_to_kobj(sa), &sa->sa_attr);
}


/* send change uevent with additional environment */
static int
de_attr_uevent_change(struct de_attribute *sa, char *envp[])
{
   return kobject_uevent_env(attr_to_kobj(sa), KOBJ_CHANGE, envp);
}

/*
 *===========================================================================
 *                       DRIVERENV FUNCTIONS
 *===========================================================================
 */

void
de_init(struct de_obj *de, void *dev)
{
   spin_lock_init(&de->de_lock);
   INIT_LIST_HEAD(&de->de_attrs);
   de->de_dev = dev;
}

void
de_release(struct de_obj *de)
{
   struct list_head *p, *q;
   struct de_attribute *sa;

   list_for_each_safe(p, q, &de->de_attrs) {
      sa = container_of(p, struct de_attribute, sa_list);
      de_attr_free(sa);
   }
}

void de_assign_mac_addr(void *_dev, unsigned char *addr)
{
   struct device *dev = (struct device *)_dev;
   const struct firmware *fw;

   if(request_firmware(&fw, "nanoradio/settings.bin", dev) != 0)
      goto randomise_addr;

   if(fw->size != ETH_ALEN || (fw->data[0] & 1) == 1) {
      release_firmware(fw);
      goto randomise_addr;
   }
   memcpy(addr, fw->data, ETH_ALEN);
   release_firmware(fw);
   return;

  randomise_addr:
   random_ether_addr(addr);
   addr[0] = 0x5e;
   addr[1] = 0x01;
   addr[2] = 0x01;
}

void *
de_coredump_open(struct de_obj *de, unsigned obj_id, unsigned err_code, size_t size)
{
   struct de_attribute *sa = de_attr_alloc(de, size);
   de_attr_setname(sa, "core-%u-%u.%u", core_idx++, obj_id, err_code);
   return sa;
}

void
de_coredump_write(void *ref, de_pdu_t *pdu)
{
   struct de_attribute *sa = ref;
   de_attr_write(sa, de_pdu_data(pdu), sa->sa_totalsize, de_pdu_size(pdu));
}

void
de_coredump_close(void *ref)
{
   struct de_attribute *sa = ref;

   if (de_attr_create(sa) == 0) {
      char core_name[64];
      char *uevent_env[] = { "NRX_EVENT=core", core_name, NULL };
      snprintf(core_name, sizeof(core_name), "NRX_CORENAME=%s", sa->sa_name);
      de_attr_uevent_change(sa, uevent_env);
   }
}


de_fw_ref
de_fw_open(void *_dev, const char *filename)
{
   struct device *dev = (struct device *)_dev;
   de_fw_ref ref = NULL;
   const struct firmware *fw;
   char path[64];

   snprintf(path, sizeof path, "nanoradio/%s", filename);

   if (request_firmware(&fw, path, dev) == 0) {
      ref = kmalloc(sizeof *ref, GFP_KERNEL);
      ref->fw  = fw;
      ref->pos = 0;
   }

   return ref;
}

de_pdu_t *
de_fw_get_chunk(de_fw_ref ref)
{
   de_pdu_t *pdu = NULL;
   size_t chunk = min((size_t)DE_FW_CHUNK_SIZE, (size_t)ref->fw->size - ref->pos);

   if (chunk > 0) {
      pdu = de_pdu_alloc(DE_FW_CHUNK_SIZE);
      memcpy(de_pdu_put(pdu, chunk), &ref->fw->data[ref->pos], chunk);
      ref->pos += chunk;
   }

   return pdu;
}

void
de_fw_free_chunk(de_fw_ref ref, de_pdu_t *pdu)
{
   de_pdu_free(pdu);
}

void
de_fw_close(de_fw_ref ref)
{
   release_firmware(ref->fw);
   kfree(ref);
}

static void
de_timeout_handler(unsigned long data)
{
   de_timer_t *t = (de_timer_t *)data;
   schedule_work(&t->dt_work);
}

static void
de_timeout_work(struct work_struct *work)
{
   de_timer_t *t = container_of(work, de_timer_t, dt_work);
   de_debug(DE_TR_OS, "t=%p callback=%p", t, t->dt_callback);
   t->dt_callback(t);
}

void
de_timer_init(de_timer_t *t, de_timer_callback_t cb)
{
   init_timer(&t->dt_timer);
   INIT_WORK(&t->dt_work, de_timeout_work);
   t->dt_timer.function = de_timeout_handler;
   t->dt_timer.data = (unsigned long)t;
   t->dt_callback = cb;
}
