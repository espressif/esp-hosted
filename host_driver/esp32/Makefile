CR_C := arm-linux-gnueabihf-
PWD := $(shell pwd)
obj-m := esp32.o

esp32-y := main.o

all:
	@echo ${PWD}
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /home/mangesh/Work/raspberry/linux/ M=$(PWD) modules

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /home/mangesh/Work/raspberry/linux/ M=$(PWD) clean

