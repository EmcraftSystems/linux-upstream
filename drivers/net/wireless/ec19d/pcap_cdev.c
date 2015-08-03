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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/moduleparam.h>
#include <linux/skbuff.h>
#include <linux/cdev.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include "driverenv.h"
#include "hic_pcap.h"
#include "dev_class.h"
#include "pcap_cdev.h"

/* Variables ======================================== */

static dev_t               devno;
static struct              cdev cdev;
static wait_queue_head_t   waitqueue;
static struct class       *dev_class;
static struct sk_buff_head queue;
static struct device      *device;

static struct file_operations pcap_cdev_fileops;

/* Global functions ==================================== */

void pcap_cdev_init(void)
{
   int ret;
   const char dev_name[] = "ttyPCAP";

   dev_class = dev_class_create();
   if ((ret = alloc_chrdev_region(&devno, 0, 1, dev_name)) != 0) {
      de_error(DE_TR_PCAP, "Failed to allocate chrdev region: %d", ret);
      return;
   }
   de_info(DE_TR_PCAP, "Allocated chrdev region @ %u.%u", MAJOR(devno), MINOR(devno));

   cdev_init(&cdev, &pcap_cdev_fileops);
   cdev.owner = THIS_MODULE;

   if ((ret = cdev_add(&cdev, devno, 1)) != 0) {
      de_error(DE_TR_PCAP, "Failed to add device: %d", ret);
      goto error1;
   }

   device = device_create(dev_class, NULL, devno, NULL, "%s%d", dev_name, MINOR(devno));
   if (device == NULL) {
      goto error2;
   }
   return;

error2:
   cdev_del(&cdev);
error1:
   unregister_chrdev_region(devno, 1);
}

void pcap_cdev_exit(void)
{
   device_destroy(dev_class, devno);
   cdev_del(&cdev);
   unregister_chrdev_region(devno, 1);
   dev_class_remove();
}

void hic_pcap_hdlr_write(de_pdu_t *pdu)
{
   skb_queue_tail(&queue, pdu);
   wake_up_interruptible(&waitqueue);
}

/* Local functions ==================================== */

static int
pcap_cdev_open(struct inode *inode, struct file *filp)
{
   init_waitqueue_head(&waitqueue);
   skb_queue_head_init(&queue);
   hic_pcap_start();
   return 0;
}

static int
pcap_cdev_release(struct inode *inode,
                  struct file *filp)
{
   hic_pcap_stop();
   skb_queue_purge(&queue);
   return 0;
}

static ssize_t
pcap_cdev_read(struct file *filp,
               char __user *buf,
               size_t count,
               loff_t *off)
{
   struct sk_buff   *skb = NULL;
   size_t            n;

   while ((skb = skb_peek(&queue)) == NULL) {
      if (filp->f_flags & O_NONBLOCK) {
         return -EAGAIN;
      }
      if (wait_event_interruptible(waitqueue, !skb_queue_empty(&queue))) {
         return -EINTR;
      }
   }

   n = min(count, (size_t)skb->len);

   if (copy_to_user(buf, de_pdu_data(skb), n)) {
      return -EFAULT;
   }
   skb_pull(skb, n);
   if (skb->len == 0) {
      skb_unlink(skb, &queue);
      de_pdu_free(skb);
   }

   return n;
}

static ssize_t
pcap_cdev_write(struct file *filp,
                const char __user *buf,
                size_t count,
                loff_t *off)
{
   return 0;
}

static unsigned int
pcap_cdev_poll(struct file *filp,
               poll_table *wait)
{
   unsigned int mask = POLLOUT | POLLWRNORM;

   poll_wait(filp, &waitqueue, wait);

   if (!skb_queue_empty(&queue)) {
      mask |= POLLIN | POLLRDNORM;
   }

   return mask;
}

static struct file_operations pcap_cdev_fileops = {
   .owner   = THIS_MODULE,
   .open    = pcap_cdev_open,
   .release = pcap_cdev_release,
   .read    = pcap_cdev_read,
   .write   = pcap_cdev_write,
   .poll    = pcap_cdev_poll,
};
