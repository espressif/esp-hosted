CONFIG_SUPPORT_ESP_SERIAL = y
CONFIG_ENABLE_MONITOR_PROCESS = n

ifeq ($(CONFIG_SUPPORT_ESP_SERIAL), y)
	EXTRA_CFLAGS += -DCONFIG_SUPPORT_ESP_SERIAL
endif

ifeq ($(CONFIG_ENABLE_MONITOR_PROCESS), y)
	EXTRA_CFLAGS += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

EXTRA_CFLAGS += -I$(PWD)/../../common/include

CR_C := arm-linux-gnueabihf-
PWD := $(shell pwd)
obj-m := esp32.o

esp32-y := esp_sdio.o esp_sdio_api.o esp_bt.o main.o
ifeq ($(CONFIG_SUPPORT_ESP_SERIAL), y)
esp32-y += esp_serial.o esp_rb.o
endif

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean 

