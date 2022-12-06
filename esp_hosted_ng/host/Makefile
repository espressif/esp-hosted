CONFIG_TEST_RAW_TP := n
CONFIG_ENABLE_MONITOR_PROCESS = n

# Toolchain Path
CROSS_COMPILE := /usr/bin/arm-linux-gnueabihf-
# Linux Kernel header
KERNEL := /lib/modules/$(shell uname -r)/build
# Architecture
ARCH := arm

MODULE_NAME=esp32

ifeq ($(CONFIG_ENABLE_MONITOR_PROCESS), y)
	EXTRA_CFLAGS += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

ifeq ($(CONFIG_TEST_RAW_TP), y)
	EXTRA_CFLAGS += -DCONFIG_TEST_RAW_TP
endif

EXTRA_CFLAGS += -I$(PWD)/include -I$(PWD) -I$(PWD)/sdio  -I$(PWD)/spi -Wall

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := esp_bt.o main.o esp_cmd.o esp_wpa_utils.o esp_cfg80211.o esp_stats.o sdio/esp_sdio.o sdio/esp_sdio_api.o spi/esp_spi.o

all: clean
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o sdio/*.o spi/*.o *.ko
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean
