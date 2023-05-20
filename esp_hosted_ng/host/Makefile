CONFIG_TEST_RAW_TP := n
CONFIG_ENABLE_MONITOR_PROCESS = n

# Toolchain Path
CROSS_COMPILE := /usr/bin/arm-linux-gnueabihf-
# Linux Kernel header
KERNEL := /lib/modules/$(shell uname -r)/build
# Architecture
ARCH := arm

#Default interface is sdio
MODULE_NAME=esp32_sdio

#enable logging by default
CONFIG_DEBUG_LOGS := y

#Targets passed overrrides default value
ifeq ($(target), sdio)
	MODULE_NAME=esp32_sdio
endif

ifeq ($(target), spi)
	MODULE_NAME=esp32_spi
endif

ifeq ($(CONFIG_ENABLE_MONITOR_PROCESS), y)
	EXTRA_CFLAGS += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

ifeq ($(CONFIG_INFO_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_INFO_LOGS
endif

ifeq ($(CONFIG_DEBUG_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_INFO_LOGS
	EXTRA_CFLAGS += -DCONFIG_DEBUG_LOGS
endif

ifeq ($(CONFIG_TEST_RAW_TP), y)
	EXTRA_CFLAGS += -DCONFIG_TEST_RAW_TP
endif

EXTRA_CFLAGS += -I$(PWD)/include -I$(PWD)

ifeq ($(MODULE_NAME), esp32_sdio)
	EXTRA_CFLAGS += -I$(PWD)/sdio
	module_objects += sdio/esp_sdio.o sdio/esp_sdio_api.o
endif

ifeq ($(MODULE_NAME), esp32_spi)
	EXTRA_CFLAGS += -I$(PWD)/spi
	module_objects += spi/esp_spi.o
endif

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := esp_bt.o main.o esp_cmd.o esp_wpa_utils.o esp_cfg80211.o esp_stats.o $(module_objects)

all: clean
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o sdio/*.o spi/*.o *.ko
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean
