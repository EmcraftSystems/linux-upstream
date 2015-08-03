#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>

#define NRX_COMPONENT_SDIO
#define NRX_COMPONENT_WIFI

/* this is the module entry file for the nrx driver, feel free to come
 * up with a less kludgy solution to multiple component
 * initialisation */

#ifdef NRX_COMPONENT_WIFI
#include "smd_common.h"
MODULE_INFO(nr_component, "wifi");
#endif
#ifdef NRX_COMPONENT_RFTEST
#include "testing.h"
MODULE_INFO(nr_component, "rftest");
#define isRFTest rtest_mode
#else
#define isRFTest false
#endif
#ifdef NRX_COMPONENT_BT
#include "nanobt.h"
MODULE_INFO(nr_component, "bt");
#endif
#ifdef NRX_COMPONENT_LOOPBACK
#include "loopback.h"
MODULE_INFO(nr_component, "loopback");
#endif
#ifdef NRX_COMPONENT_HIC_PCAP_CDEV
#include "pcap_cdev.h"
MODULE_INFO(nr_component, "hic_pcap_cdev");
#endif
#ifdef NRX_COMPONENT_HIC_PCAP_IFACE
#include "pcap_iface.h"
MODULE_INFO(nr_component, "hic_pcap_iface");
#endif
#ifdef NRX_COMPONENT_USB
#include "transport_usb.h"
MODULE_INFO(nr_component, "usb");
#endif
#ifdef NRX_COMPONENT_SDIO
#include "transport_sdio.h"
MODULE_INFO(nr_component, "sdio");
#endif
#ifdef NRX_COMPONENT_PCI
/* XXX */
int smd_pci_init(void);
void smd_pci_exit(void);
MODULE_INFO(nr_component, "pci");
#endif
#ifdef NRX_COMPONENT_CDEV
/* XXX */
int smd_cdev_init(void);
void smd_cdev_exit(void);
MODULE_INFO(nr_component, "cdev");
#endif

#ifdef NR_DRIVER_VERSION
MODULE_INFO(nr_driver_version, NR_DRIVER_VERSION);
#endif

static int __init
nano_init(void)
{
   int ret;

#ifdef NRX_COMPONENT_WIFI
   if (!isRFTest)
   {
       ret = smd_wifi_init();
       if (ret != 0) {
          return ret;
       }
   }
#endif

#ifdef NRX_COMPONENT_RFTEST
   if (isRFTest)
   {
       ret = smd_rftest_init();
       if (ret != 0) {
           return ret;
       }
   }
#endif
   
#ifdef NRX_COMPONENT_BT
   nanobt_init();
#endif

#ifdef NRX_COMPONENT_LOOPBACK
   loopback_init();
#endif
#ifdef NRX_COMPONENT_HIC_PCAP_CDEV
   pcap_cdev_init();
#endif
#ifdef NRX_COMPONENT_HIC_PCAP_IFACE
   pcap_iface_init();
#endif

#ifdef NRX_COMPONENT_USB
   ret = smd_usb_init();
   if(ret != 0) {
      return ret;
   }
#endif
#ifdef NRX_COMPONENT_SDIO
   ret = smd_sdio_init();
   if(ret != 0) {
      return ret;
   }
#endif
#ifdef NRX_COMPONENT_PCI
   ret = smd_pci_init();
   if(ret != 0) {
      return ret;
   }
#endif
#ifdef NRX_COMPONENT_CDEV
   ret = smd_cdev_init();
   if(ret != 0) {
      return ret;
   }
#endif

   return 0;
}

static void __exit
nano_exit(void)
{
#ifdef NRX_COMPONENT_CDEV
   smd_cdev_exit();
#endif
#ifdef NRX_COMPONENT_PCI
   smd_pci_exit();
#endif
#ifdef NRX_COMPONENT_SDIO
   smd_sdio_exit();
#endif
#ifdef NRX_COMPONENT_USB
   smd_usb_exit();
#endif
#ifdef NRX_COMPONENT_HIC_PCAP_CDEV
   pcap_cdev_exit();
#endif
#ifdef NRX_COMPONENT_HIC_PCAP_IFACE
   pcap_iface_exit();
#endif
#ifdef NRX_COMPONENT_BT
   nanobt_exit();
#endif
#ifdef NRX_COMPONENT_WIFI
   if (!isRFTest)
      smd_wifi_exit();
#endif
#ifdef NRX_COMPONENT_RFTEST
   if (isRFTest)
      smd_rftest_exit();
#endif
}

module_init(nano_init);
module_exit(nano_exit);
MODULE_LICENSE("GPL");
