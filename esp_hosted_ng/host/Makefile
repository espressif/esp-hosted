CONFIG_ENABLE_MONITOR_PROCESS = n

# Toolchain Path
CROSS_COMPILE :=
# Linux Kernel header
KERNEL := /lib/modules/$(shell uname -r)/build
# Architecture
ARCH := arm64

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
	debug_flags := -DDEBUG
endif

ifeq ($(CONFIG_VERBOSE_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_INFO_LOGS
	EXTRA_CFLAGS += -DCONFIG_DEBUG_LOGS
	EXTRA_CFLAGS += -DCONFIG_VERBOSE_LOGS
	debug_flags := -DDEBUG
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

module_objects += esp_bt.o main.o esp_cmd.o esp_utils.o esp_cfg80211.o esp_stats.o
$(foreach obj,$(module_objects),$(eval CFLAGS_$(obj:.o=).o := $(debug_flags)))

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := $(module_objects)

all: clean
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o sdio/*.o spi/*.o *.ko
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean
