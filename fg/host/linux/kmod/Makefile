# -----------------------------
# Kernel module build settings
# -----------------------------
CONFIG_BT_ENABLED := y
CONFIG_TEST_RAW_TP := n
CONFIG_INFO_LOGS := y
CONFIG_DEBUG_LOGS := n
CONFIG_VERBOSE_LOGS := n

BUILD_DIR := $(PWD)/build

# Target slave type (esp32 or esp32c6)
ESP_SLAVE := ""

# Compiler and kernel
CROSS_COMPILE :=
ARCH := arm64
#KERNEL := /lib/modules/$(shell uname -r)/build
KERNEL := /lib/modules/5.15.0-161-generic/build

# Default interface is SDIO
MODULE_NAME := esp32_sdio

ifeq ($(target), sdio)
	MODULE_NAME := esp32_sdio
endif

ifeq ($(target), spi)
	MODULE_NAME := esp32_spi
endif

# -----------------------------
# Compute portable paths
# -----------------------------
# Directory of this Makefile
MAKEFILE_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

# Root of esp_hosted_fg
TRANSPORT_HEADERS := $(realpath $(MAKEFILE_DIR)/../../../coprocessor/components/esp_hosted_transport_cp)

# Host transport component (new architecture)
TRANSPORT_HOST_HEADERS := $(realpath $(MAKEFILE_DIR)/../../components/esp_hosted_transport_host)

# Main common directory (project-wide common headers)
COMMON_DIR := $(realpath $(MAKEFILE_DIR)/../../../common)

# Canonical common headers (components/common/esp_hosted_common/include/)
CANONICAL_COMMON_INC := $(realpath $(MAKEFILE_DIR)/../../../../components/common/esp_hosted_common/include)

$(info TRANSPORT_HEADERS = $(TRANSPORT_HEADERS))
$(info TRANSPORT_HOST_HEADERS = $(TRANSPORT_HOST_HEADERS))
$(info COMMON_DIR = $(COMMON_DIR))

# Common include folders
COMMON_TRANSPORT_INC := $(TRANSPORT_HEADERS)/include/common
TRANSPORT_HOST_INC := $(TRANSPORT_HOST_HEADERS)/include
TRANSPORT_HOST_KMOD_INC := $(TRANSPORT_HOST_HEADERS)/private_include/kmod
#COMMON_INC := $(MAKEFILE_DIR)/include/common/

# Source directories
TRANSPORT_HOST_SRC := $(TRANSPORT_HOST_HEADERS)/src/kmod

# -----------------------------
# Compiler flags
# -----------------------------
EXTRA_CFLAGS += -I$(COMMON_TRANSPORT_INC) -I$(TRANSPORT_HOST_INC) -I$(TRANSPORT_HOST_KMOD_INC) -I$(COMMON_DIR) -I$(CANONICAL_COMMON_INC) -I$(MAKEFILE_DIR)/include -I$(MAKEFILE_DIR)/include/common
#EXTRA_CFLAGS += -I$(COMMON_TRANSPORT_INC) -I$(MAKEFILE_DIR)/include -I$(COMMON_INC)

ifeq ($(CONFIG_TEST_RAW_TP), y)
	EXTRA_CFLAGS += -DCONFIG_TEST_RAW_TP
endif

ifeq ($(CONFIG_BT_ENABLED), y)
	EXTRA_CFLAGS += -DCONFIG_BT_ENABLED
	module_objects += src/esp_bt.o
endif

ifeq ($(MODULE_NAME), esp32_sdio)
	EXTRA_CFLAGS += -I$(MAKEFILE_DIR)/include/sdio -I$(TRANSPORT_HOST_KMOD_INC)/sdio
	module_objects += $(TRANSPORT_HOST_SRC)/sdio/esp_sdio.o $(TRANSPORT_HOST_SRC)/sdio/esp_sdio_api.o
endif

ifeq ($(MODULE_NAME), esp32_spi)
	EXTRA_CFLAGS += -I$(MAKEFILE_DIR)/include/spi -I$(TRANSPORT_HOST_KMOD_INC)/spi
	module_objects += $(TRANSPORT_HOST_SRC)/spi/esp_spi.o
endif

ifneq ($(ESP_SLAVE), "")
	EXTRA_CFLAGS += -D$(ESP_SLAVE)
endif

ifeq ($(CONFIG_INFO_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_INFO_LOGS
endif
ifeq ($(CONFIG_DEBUG_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_DEBUG_LOGS
endif
ifeq ($(CONFIG_VERBOSE_LOGS), y)
	EXTRA_CFLAGS += -DCONFIG_VERBOSE_LOGS
endif

# -----------------------------
# Module objects
# -----------------------------
obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := src/main.o src/esp_stats.o $(module_objects)
$(MODULE_NAME)-y += src/esp_serial.o src/esp_rb.o src/esp_fw_verify.o src/esp_hosted_transport_linux_impl.o

# -----------------------------
# Build rules
# -----------------------------
all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(MAKEFILE_DIR) modules

clean:
	rm -rf *.o src/*.o src/sdio/*.o src/spi/*.o *.ko $(TRANSPORT_HOST_SRC)/sdio/*.o $(TRANSPORT_HOST_SRC)/spi/*.o
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(MAKEFILE_DIR) clean

