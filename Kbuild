ccflags-$(CONFIG_ARP_CAN) += -DARP_ENABLE_CAN
ccflags-$(CONFIG_ARP_CAMERA) += -DARP_ENABLE_CAMERA

obj-m := arp.o
arp-y := arp-core.o

ifeq ($(CONFIG_ARP_CAMERA), y)
CONFIG_ARP_CAMERAm := m
endif

obj-$(CONFIG_ARP_CAMERAm) += arp-camera.o
arp-camera-y := arp-cam.o

