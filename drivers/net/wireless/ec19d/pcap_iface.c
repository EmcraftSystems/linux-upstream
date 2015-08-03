#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <net/sch_generic.h>

#include "driverenv.h"
#include "hic_pcap.h"
#include "pcap_iface.h"

struct pcap_iface_handle {
   struct net_device *pih_iface;
   bool               pih_seen_header;
};

static netdev_tx_t
pcap_iface_start_xmit(struct sk_buff *skb,
                     struct net_device *dev)
{
   dev_kfree_skb(skb);

   return NETDEV_TX_OK;
}

static int pcap_iface_open(struct net_device *dev)
{
   hic_pcap_start();

   return 0;
}

static int pcap_iface_stop(struct net_device *dev)
{
   hic_pcap_stop();

   return 0;
}

static const struct net_device_ops pcap_iface_device_ops = {
   .ndo_open = pcap_iface_open,
   .ndo_stop = pcap_iface_stop,
   .ndo_start_xmit = pcap_iface_start_xmit,
   .ndo_change_mtu = eth_change_mtu,
   .ndo_set_mac_address = eth_mac_addr,
   .ndo_validate_addr = eth_validate_addr
};

static void pcap_iface_setup(struct net_device *dev)
{
   struct pcap_iface_handle *pih;

   dev->netdev_ops = &pcap_iface_device_ops;
   dev->destructor = free_netdev;
   ether_setup(dev);
   dev->tx_queue_len = 0;
   dev->flags = IFF_NOARP;
   memset(dev->dev_addr, 0x22, ETH_ALEN);

   pih = netdev_priv(dev);
   pih->pih_iface = dev;
   pih->pih_seen_header = false;
}

static struct pcap_iface_handle*
pcap_iface_create(void)
{
   struct net_device *dev;
   int retval;

   dev = alloc_netdev(sizeof(struct pcap_iface_handle), "hic-monitor", pcap_iface_setup);
   if (dev == NULL)
      return NULL;

   retval = register_netdev(dev);
   if(retval < 0) {
      free_netdev(dev);
      return NULL;
   }

   return (struct pcap_iface_handle*)netdev_priv(dev);
}

static void pcap_iface_destroy(struct pcap_iface_handle *pih)
{
   unregister_netdev(pih->pih_iface);
}

static void pcap_iface_write(struct pcap_iface_handle *pih, struct sk_buff *skb)
{
   u8 *hdr;

   if (!netif_running(pih->pih_iface)) {
      dev_kfree_skb(skb);
      return;
   }

   /* add dummy ethernet header */
   hdr = skb_push(skb, 14);
   memset(hdr, 0, ETH_ALEN * 2);
   memset(hdr + 12, 0xee, 2);

   skb->dev = pih->pih_iface;
   skb_set_mac_header(skb, 0);
   skb->ip_summed = CHECKSUM_UNNECESSARY;
   skb->pkt_type = PACKET_OTHERHOST;
   skb->protocol = htons(ETH_P_802_2);

   netif_rx_ni(skb);
}

static struct pcap_iface_handle *__pih;

void hic_pcap_hdlr_write(de_pdu_t *pdu)
{
   if(__pih == NULL) {
      de_pdu_free(pdu);
      return;
   }
   if(!__pih->pih_seen_header && de_pdu_size(pdu) == 24) {
      /* drop header */
      __pih->pih_seen_header = true;
      de_pdu_free(pdu);
      return;
   }
   de_pdu_pull(pdu, 16); /* remove extra pcap header */
   pcap_iface_write(__pih, pdu);
}


int __init
pcap_iface_init(void)
{
   __pih = pcap_iface_create();
   if(__pih == NULL)
      return -ENOMEM;

   rtnl_lock();
   dev_open(__pih->pih_iface);

   /* attempt to get rid of the useless router solicitations */
   dev_deactivate(__pih->pih_iface);
   rtnl_unlock();

   return 0;
}

void __exit
pcap_iface_exit(void)
{
   struct pcap_iface_handle *tmp = __pih;
   __pih = NULL;
   pcap_iface_destroy(tmp);
}
