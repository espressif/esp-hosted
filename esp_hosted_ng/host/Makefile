ifneq ($(filter $(CONFIG_ESPRESSIF_ESP_HOSTED_NG),y m),)

mod_name_sdio = esp32_sdio
mod_name_spi = esp32_spi

ccflags-y += -I$(src)/include -I$(src)

mod_objects = \
	esp_bt.o \
	esp_cmd.o \
	esp_utils.o \
	esp_cfg80211.o \
	esp_stats.o \
	esp_debugfs.o \
	esp_log.o \
	main.o \

$(mod_name_sdio)-objs := $(mod_objects)
$(mod_name_spi)-objs := $(mod_objects)

ifneq ($(filter $(CONFIG_ESP_HOSTED_NG_SDIO),y m),)
	ccflags-y += -I$(src)/sdio
	$(mod_name_sdio)-objs += sdio/esp_sdio.o sdio/esp_sdio_api.o
endif

ifneq ($(filter $(CONFIG_ESP_HOSTED_NG_SPI),y m),)
	ccflags-y += -I$(src)/spi
	$(mod_name_spi)-objs += spi/esp_spi.o
endif

log_level := $(CONFIG_ESP_HOSTED_NG_LOG_LEVEL)
log_debug := 3
is_debug := $(shell echo ${log_level}\>=${log_debug} | bc)
ifeq ($(is_debug),1)
	ccflags-y += -DDEBUG
endif

obj-$(CONFIG_ESP_HOSTED_NG_SDIO) += $(mod_name_sdio).o
obj-$(CONFIG_ESP_HOSTED_NG_SPI) += $(mod_name_spi).o

endif
