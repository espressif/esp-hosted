CR_C := arm-linux-gnueabihf-
PWD := $(shell pwd)
obj-m := esp32.o

esp32-y := esp32_sdio.o main.o

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean 

