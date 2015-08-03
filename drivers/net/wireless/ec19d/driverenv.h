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

#ifndef DRIVERENV_H
#define DRIVERENV_H
#include <linux/kernel.h>
#include <linux/elf.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/time.h>

extern int trace_level;
extern unsigned int trace_mask;

#ifdef DE_CFG_TRACE_TASKNAME
#define current_comm() (current->comm)
#else
#define current_comm() ""
#endif

#define DE_ERR                0
#define DE_WARN               1
#define DE_INFO               2
#define DE_DEBUG              3

#define DE_TR_HIC             (1<<0)
#define DE_TR_OS              (1<<1)
#define DE_TR_TP              (1<<2)
#define DE_TR_NANOC           (1<<3)
#define DE_TR_API             (1<<4)
#define DE_TR_WIFI            (1<<5)
#define DE_TR_BT              (1<<6)
#define DE_TR_AUDIO           (1<<7)
#define DE_TR_LOOPBACK        (1<<8)
#define DE_TR_PCAP            (1<<9)
#define DE_TR_ALL             (~DE_TR_HIC)

#ifdef DE_CFG_NOTRACE

#define de_print(_cat, _fmt, ...) do {} while(0)
#define de_trace_enabled(_sel)         (0)

#else

#define de_trace_enabled(_level, _mask) \
   (trace_level >= (_level) && (trace_mask & (_mask)) != 0)

#define de_print(_fmt, ...) \
   printk(KERN_INFO "" _fmt "\n", ## __VA_ARGS__)

#endif

#define __de_trace(_level, _sel, _tag, _fmt, ...)       \
   while(de_trace_enabled(_level, _sel)) {              \
      printk(_tag "%s[%s:%u] " _fmt "\n",               \
             current_comm(), __func__,                  \
             __LINE__ , ## __VA_ARGS__);                \
      break;                                            \
   }
#define de_error(_sel, _fmt, ...) __de_trace(DE_ERR,   _sel, KERN_ERR,     _fmt , ## __VA_ARGS__)
#define de_warn(_sel, _fmt, ...)  __de_trace(DE_WARN,  _sel, KERN_WARNING, _fmt , ## __VA_ARGS__)
#define de_info(_sel, _fmt, ...)  __de_trace(DE_INFO,  _sel, KERN_INFO,    _fmt , ## __VA_ARGS__)
#define de_debug(_sel, _fmt, ...) __de_trace(DE_DEBUG, _sel, KERN_DEBUG,   _fmt , ## __VA_ARGS__)

#define DE_EXPORT_SYMBOL(_name) /*EXPORT_SYMBOL(_name)*/

#define DE_FW_CHUNK_SIZE   480
#define DE_PDU_HEADROOM     64
#define DE_PDU_MAX_PADDING  32

#define DE_BUG_ON(_exp) BUG_ON(_exp)
#define DE_ASSERT(_exp) WARN_ON(!(_exp))

#define de_cpu_to_le16(x) cpu_to_le16(x)
#define de_cpu_to_le32(x) cpu_to_le32(x)
#define de_le16_to_cpu(x) le16_to_cpu(x)
#define de_le32_to_cpu(x) le32_to_cpu(x)

struct de_obj {
   spinlock_t         de_lock;
   unsigned long      de_irqflags;
   struct device     *de_dev;
   struct list_head   de_attrs;
};

/* Init functions */
void de_init(struct de_obj *de, void *dev);
void de_release(struct de_obj *de);
void de_assign_mac_addr(void*, unsigned char *addr);

/* Module parameters */
#define DE_MODULE_PARAM(name, type, desc) \
   module_param(name, type, 0644); \
   MODULE_PARM_DESC(name, desc)

#define DE_MODULE_PARAM_RO(name, type, desc) \
   module_param(name, type, 0444); \
   MODULE_PARM_DESC(name, desc)

#define DE_MODULE_PARAM_STR(name, len ,desc) \
   module_param_string(name, name, len, 0644); \
   MODULE_PARM_DESC(name, desc)

#define DE_MODULE_PARAM_STR_RO(name, len ,desc) \
   module_param_string(name, name, len, 0444); \
   MODULE_PARM_DESC(name, desc)

typedef void(*de_set_param_cb)(const char *val);
typedef void(*de_get_param_cb)(char *buffer);
#define DE_MODULE_PARAM_CB(name, _get, _set, desc) \
   static int param_set_##name(const char *val, const struct kernel_param *kp) \
   { _set(val); return 0; }\
   static int param_get_##name(char *buffer, const struct kernel_param *kp) \
   { _get(buffer); return 0; }\
   static struct kernel_param_ops param_ops_##name = \
   {.set = param_set_##name, .get = param_get_##name }; \
   module_param_cb(name, &param_ops_##name, &name, 0644); \
   MODULE_PARM_DESC(name, desc)

/* Timer functions */

static inline unsigned
de_now_msec(void)
{
   struct timeval tv;
   do_gettimeofday(&tv);
   return (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ; // convert tv_sec & tv_usec to millisecond
}

struct de_time {
   unsigned sec;
   unsigned usec;
};
static inline void
de_now(struct de_time* time)
{
   struct timeval tv;
   do_gettimeofday(&tv);
   time->sec = tv.tv_sec;
   time->usec = tv.tv_usec;
}

struct de_timer;

typedef struct de_timer de_timer_t;

typedef void (*de_timer_callback_t)(de_timer_t *);

struct de_timer {
   struct timer_list   dt_timer;
   struct work_struct  dt_work;
   de_timer_callback_t dt_callback;
};

void de_timer_init(de_timer_t *t, de_timer_callback_t cb);

static inline void
de_timer_start(de_timer_t *t, unsigned int timeout_ms)
{
   mod_timer(&t->dt_timer, jiffies + msecs_to_jiffies(timeout_ms));
}

static inline void
de_timer_stop(de_timer_t *t)
{
   del_timer(&t->dt_timer);
}

static inline void
de_msleep(unsigned time_ms)
{
   msleep(time_ms);
}

/* Locking */

typedef struct mutex de_mutex_t;
/* Static init of mutex */
#define DE_MUTEX_INIT(_mutex) __MUTEX_INITIALIZER(_mutex)

/* Dynamic init of mutex */
static inline void de_mutex_init(de_mutex_t *mutex)
{
   mutex_init(mutex);
}

static inline void de_mutex_lock(de_mutex_t *mutex)
{
   mutex_lock(mutex);
}

static inline void de_mutex_unlock(de_mutex_t *mutex)
{
   mutex_unlock(mutex);
}


/* PDU functions */

typedef struct sk_buff de_pdu_t;

static inline de_pdu_t *
de_pdu_alloc(size_t size)
{
   de_pdu_t *pdu = alloc_skb(DE_PDU_HEADROOM+size+DE_PDU_MAX_PADDING, GFP_KERNEL | GFP_DMA);
   if (!pdu) {
      de_error(DE_TR_OS, "Out of memory");
   } else {
      skb_reserve(pdu, DE_PDU_HEADROOM);
   }
   return pdu;
}

static inline size_t
de_pdu_headroom(de_pdu_t *pdu)
{
   return skb_headroom(pdu);
}

static inline void *
de_pdu_put(de_pdu_t *pdu, size_t size)
{
   return skb_put(pdu, size);
}

static inline void *
de_pdu_push(de_pdu_t *pdu, size_t size)
{
   return skb_push(pdu, size);
}

static inline void *
de_pdu_pull(de_pdu_t *pdu, size_t size)
{
   return skb_pull(pdu, size);
}

static inline void
de_pdu_trim(de_pdu_t *pdu, size_t size)
{
   skb_trim(pdu, size);
}

static inline void
de_pdu_free(de_pdu_t *pdu)
{
   dev_kfree_skb(pdu);
}

static inline void *
de_pdu_data(const de_pdu_t *pdu)
{
   return pdu->data;
}

static inline size_t
de_pdu_size(const de_pdu_t *pdu)
{
   return pdu->len;
}

static inline de_pdu_t *
de_pdu_copy(de_pdu_t *pdu)
{
   return skb_copy(pdu, GFP_KERNEL);
}

static inline void
de_pdu_pad(de_pdu_t *pdu, size_t pad)
{
   if (unlikely(skb_pad(pdu, pad) != 0)) {
      DE_ASSERT(0);
   }
   pdu->len += pad;
}

/* Firmware functions */

typedef struct {
   const struct firmware *fw;
   size_t pos;
} *de_fw_ref;

de_fw_ref de_fw_open(void *_dev, const char *filename);
de_pdu_t *de_fw_get_chunk(de_fw_ref ref);
void      de_fw_free_chunk(de_fw_ref ref, de_pdu_t *pdu);
void      de_fw_close(de_fw_ref ref);

/* Coredump functions */
void *de_coredump_open(struct de_obj *de, unsigned obj_id, unsigned err_code, size_t size);
void  de_coredump_write(void *ref, de_pdu_t *pdu);
void  de_coredump_close(void *ref);

/*
 * These macros wraps kernel version differences
 */

/* Work queues */
#define NRX_WORKER_INIT(VARNAME, MEMNAME, FUNCNAME) \
   INIT_WORK(&VARNAME->MEMNAME, FUNCNAME)
#define NRX_WORKER_DECLARE(FUNCNAME) \
   void FUNCNAME(struct work_struct *__work)
#define NRX_WORKER_SETUP(TYPENAME, VARNAME, MEMNAME) \
   TYPENAME *VARNAME = container_of(__work, TYPENAME, MEMNAME)
#define NRX_DELAYED_WORKER_INIT(VARNAME, MEMNAME, FUNCNAME) \
   INIT_DELAYED_WORK(&VARNAME->MEMNAME, FUNCNAME)

#endif
