CONFIG_ENABLE_MONITOR_PROCESS = n

# Toolchain Path
CROSS_COMPILE :=
# Linux Kernel header
KERNEL := /lib/modules/$(shell uname -r)/build
# Architecture
ARCH := $(shell uname -m)

# Check and adjust ARCH for specific cases
ifeq ($(ARCH), x86_64)
    ARCH := x86
endif

ifeq ($(ARCH), aarch64)
    ARCH := arm64
endif

#Default interface is sdio
MODULE_NAME=esp32_sdio

#enable logging by default
CONFIG_DEBUG_LOGS := y

ifeq ($(CONFIG_AP_SUPPORT), y)
	EXTRA_CFLAGS += -DCONFIG_AP_MODE
endif

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

#debug_flags := -DDEBUG

EXTRA_CFLAGS += -I$(PWD)/include -I$(PWD)

ifeq ($(MODULE_NAME), esp32_sdio)
	EXTRA_CFLAGS += -I$(PWD)/sdio
	module_objects += sdio/esp_sdio.o sdio/esp_sdio_api.o
endif

ifeq ($(MODULE_NAME), esp32_spi)
	EXTRA_CFLAGS += -I$(PWD)/spi
	module_objects += spi/esp_spi.o
endif

module_objects += esp_bt.o main.o esp_cmd.o esp_utils.o esp_cfg80211.o esp_stats.o esp_debugfs.o esp_log.o
#$(foreach obj,$(module_objects),$(eval CFLAGS_$(obj:.o=).o := $(debug_flags)))
CFLAGS_esp_log.o = -DDEBUG

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := $(module_objects)

all: clean
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o sdio/*.o spi/*.o *.ko
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean

check:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) $(module_objects)
