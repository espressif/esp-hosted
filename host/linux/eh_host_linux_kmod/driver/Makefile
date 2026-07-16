CONFIG_BT_ENABLED := y
CONFIG_TEST_RAW_TP := y
CONFIG_INFO_LOGS=y
CONFIG_DEBUG_LOGS=n
CONFIG_VERBOSE_LOGS=n
CONFIG_DEBUG_STATS=n

# Choose one:
CONFIG_ESP_HOSTED_USE_WORKQUEUE=n    # For thread-based solution
# OR
# CONFIG_ESP_HOSTED_USE_WORKQUEUE=y  # For workqueue solution

# SDIO target chip override. Set on the make command line, e.g.
#   make target=sdio ESP_SLAVE=CONFIG_TARGET_ESP32C6
# Valid values: CONFIG_TARGET_ESP32, CONFIG_TARGET_ESP32C6.
ESP_SLAVE := ""

# Imx8mm
# CROSS_COMPILE := "/opt/fsl-imx-wayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-"
# KERNEL := "/home/u64/imx8_dev_board/imx-yocto-bsp/build/tmp/work/imx8mm_lpddr4_evk-poky-linux/linux-imx/5.15.52+gitAUTOINC+36363d8623-r0/build"

CROSS_COMPILE :=
KERNEL := /lib/modules/$(shell uname -r)/build
ARCH := arm64

#Default interface is sdio
MODULE_NAME=esp32_sdio

#Targets passed overrides default value
ifeq ($(target), sdio)
	MODULE_NAME=esp32_sdio
endif

ifeq ($(target), spi)
	MODULE_NAME=esp32_spi
endif

ifeq ($(CONFIG_TEST_RAW_TP), y)
	EXTRA_CFLAGS += -DCONFIG_TEST_RAW_TP
endif

ifeq ($(CONFIG_DEBUG_STATS), y)
	EXTRA_CFLAGS += -DESP_DEBUG_STATS
endif

ifeq ($(CONFIG_BT_ENABLED), y)
	EXTRA_CFLAGS += -DCONFIG_BT_ENABLED
	module_objects += src/esp_bt.o
endif

EXTRA_CFLAGS += -I$(M)/include -I$(M)/src/priv_include -I$(M)/../../../../common/eh_common/include

ifeq ($(MODULE_NAME), esp32_sdio)
	module_objects += src/sdio/esp_sdio.o src/sdio/esp_sdio_api.o
endif

ifeq ($(MODULE_NAME), esp32_spi)
	module_objects += src/spi/esp_spi.o
endif

ifneq ($(ESP_SLAVE), "")
EXTRA_CFLAGS += -D$(ESP_SLAVE)
endif

ifeq ($(CONFIG_INFO_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_INFO_LOGS
endif

ifeq ($(CONFIG_DEBUG_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_INFO_LOGS
	EXTRA_CFLAGS += -DCONFIG_DEBUG_LOGS
endif

ifeq ($(CONFIG_VERBOSE_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_INFO_LOGS
	EXTRA_CFLAGS += -DCONFIG_DEBUG_LOGS
	EXTRA_CFLAGS += -DCONFIG_VERBOSE_LOGS
endif

$(info DEBUG: CONFIG_ESP_HOSTED_USE_WORKQUEUE='$(CONFIG_ESP_HOSTED_USE_WORKQUEUE)')
$(info DEBUG: EXTRA_CFLAGS before workqueue='$(EXTRA_CFLAGS)')

ifeq ($(CONFIG_ESP_HOSTED_USE_WORKQUEUE), y)
	EXTRA_CFLAGS += -DCONFIG_ESP_HOSTED_USE_WORKQUEUE=1
else
	EXTRA_CFLAGS += -DCONFIG_ESP_HOSTED_USE_THREAD=1
endif

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := src/main.o src/esp_stats.o $(module_objects)
$(MODULE_NAME)-y += src/esp_serial.o src/esp_rb.o src/esp_fw_verify.o

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf src/*.o src/sdio/*.o src/spi/*.o *.ko
	rm -rf src/.*.cmd src/sdio/.*.cmd src/spi/.*.cmd
	rm -rf .*.cmd .tmp_versions *.mod.c *.mod *.order *.symvers *.markers
	-@make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean 2>/dev/null || true
