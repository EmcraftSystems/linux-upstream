obj-$(CONFIG_EC19D) = ec19d.o
ccflags-y += -g -ggdb
#ccflags-y += -DNRX_CONFIG_SDIO_CLK=20000
ccflags-y += -DNRX_CONFIG_SDIO_CLK=400000
#ccflags-y += -DNRX_SDIO_THREAD_RX -DNRX_SDIO_THREAD_TX
ec19d-objs=attribute.o debugfs.o delba.o driver.o ie.o loopback.o mib.o p2p.o rx.o scan.o transaction.o  tx.o  util.o \
	coredump.o  hic_pcap.o  loopback.o  nanobt_debug.o  nanoc.o  nanoc_hw.o  pkt_debug.o driverenv.o pcap_cdev.o \
	dev_class.o module.o sdio.o
