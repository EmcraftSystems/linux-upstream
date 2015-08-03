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

#define TRACE(...)

#include "smd_common.h"

struct smd_attribute {
   struct smd_hw_priv   *sa_sh;
   struct list_head      sa_list;
   struct kref           sa_kref;
   char                  sa_name[32];
   struct bin_attribute  sa_attr;
   size_t                sa_totalsize;
   unsigned int          sa_pageslots; /* length of pagearray */
   unsigned char        *sa_pagearray[0];
};

#define attr_to_kobj(SA) (&wiphy_dev((SA)->sa_sh->sh_hw->wiphy)->kobj)

#define SAINCREF(SA)   kref_get(&(SA)->sa_kref)
#define SADECREF(SA)   kref_put(&(SA)->sa_kref, smd_attr_release)


struct smd_attribute*
smd_attr_alloc(struct smd_hw_priv *sh,
               size_t maxsize)
{
   struct smd_attribute *sa;
   unsigned int slots;

   slots = (maxsize + PAGE_SIZE - 1) / PAGE_SIZE;

   sa = kzalloc(sizeof(*sa)
                + slots * sizeof(sa->sa_pagearray[0]),
                GFP_KERNEL);
   TRACE("ALLOC %p, %u slots", sa, slots);

   sa->sa_sh = sh;

   kref_init(&sa->sa_kref);

   INIT_LIST_HEAD(&sa->sa_list);

   HWLOCK(sa->sa_sh);
   list_add_tail(&sa->sa_list, &sa->sa_sh->sh_attrs);
   HWUNLOCK(sa->sa_sh);

   sa->sa_totalsize = 0;
   sa->sa_pageslots = slots;

   return sa;
}

static void smd_attr_release(struct kref *kref)
{
   struct smd_attribute *sa = container_of(kref, struct smd_attribute, sa_kref);
   unsigned int page_slot;

   TRACE("HERE");

   might_sleep();

   HWLOCK(sa->sa_sh);
   list_del(&sa->sa_list);
   HWUNLOCK(sa->sa_sh);

   if(sa->sa_attr.read != NULL) {
      /* XXX should have flag */
      sysfs_remove_bin_file(attr_to_kobj(sa), &sa->sa_attr);
   }

   //del_timer_sync(&sa->sa_timer);

   for(page_slot = 0;
       page_slot < sa->sa_pageslots && sa->sa_pagearray[page_slot] != NULL;
       page_slot++) {
      free_page((unsigned long)sa->sa_pagearray[page_slot]);
      sa->sa_pagearray[page_slot] = NULL;
   }

   kfree(sa);
}

void smd_attr_free(struct smd_attribute *sa)
{
   SADECREF(sa);
}

static int
smd_attr_read(struct smd_attribute *sa,
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

   TRACE("HERE");

   if(offset < 0) {
      return 0;
   }

   while(offset < sa->sa_totalsize && count > 0) {
      page_slot = offset / PAGE_SIZE;
      page_start = page_slot * PAGE_SIZE;
      page_offset = offset - page_start;
      //ASSERT(page_offset >= 0);
      if(page_slot >= sa->sa_pageslots) {
         /* XXX */
         TRACE("bad page slot %u", page_slot);
         break;
      }
      page = sa->sa_pagearray[page_slot];
      if(page == NULL) {
         /* XXX */
         TRACE("bad page slot %u", page_slot);
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
smd_attr_sysfs_read(struct file *filep,
                    struct kobject *kobj,
                    struct bin_attribute *attr,
                    char *buffer,
                    loff_t offset,
                    size_t count)
{
   struct smd_attribute *sa = container_of(attr,
                                           struct smd_attribute, sa_attr);
   return smd_attr_read(sa, buffer, offset, count);
}

static int
smd_attr_write(struct smd_attribute *sa,
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
smd_attr_sysfs_write(struct file *filep,
                     struct kobject *kobj,
                     struct bin_attribute *attr,
                     char *buffer,
                     loff_t offset,
                     size_t count)
{
   struct smd_attribute *sa = container_of(attr,
                                           struct smd_attribute, sa_attr);
   return smd_attr_write(sa, buffer, offset, count);
}


int
smd_attr_append(struct smd_attribute *sa,
                const void *buffer, size_t count)
{
   return smd_attr_write(sa, buffer, sa->sa_totalsize, count);
}

/* set name attribute name, call before smd_attr_complete */
int
smd_attr_setname(struct smd_attribute *sa, const char *fmt, ...)
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
int
smd_attr_create(struct smd_attribute *sa)
{

   sysfs_bin_attr_init(&sa->sa_attr);
   sa->sa_attr.attr.name = sa->sa_name;
   sa->sa_attr.attr.mode = S_IRUSR | S_IRGRP;
   sa->sa_attr.read = smd_attr_sysfs_read;
   if((sa->sa_attr.attr.mode & S_IWUGO) != 0)
      sa->sa_attr.write = smd_attr_sysfs_write;
   sa->sa_attr.size = sa->sa_totalsize;
   sa->sa_attr.private = NULL;

   return sysfs_create_bin_file(attr_to_kobj(sa), &sa->sa_attr);
}


/* send change uevent with additional environment */
int
smd_attr_uevent_change(struct smd_attribute *sa, char *envp[])
{
   return kobject_uevent_env(attr_to_kobj(sa), KOBJ_CHANGE, envp);
}

/* called from smd_alloc_hw */
void
smd_attr_alloc_hw(struct smd_hw_priv *sh)
{
   INIT_LIST_HEAD(&sh->sh_attrs);
}

/* called from smd_free_hw */
void
smd_attr_free_hw(struct smd_hw_priv *sh)
{
   struct list_head *p, *q;
   struct smd_attribute *sa;

   list_for_each_safe(p, q, &sh->sh_attrs) {
      sa = container_of(p, struct smd_attribute, sa_list);
      smd_attr_free(sa);
   }
}
