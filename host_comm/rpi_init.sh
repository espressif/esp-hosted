#!/bin/sh

wlan_init()
{
	cd ../host_driver/esp32/
	make -j8
	sudo insmod esp32.ko
	sudo mknod /dev/esps0 c 221 0
	sudo chmod 666 /dev/esps0
}

bt_init()
{
	sudo raspi-gpio set 16 a3 pu
	sudo raspi-gpio set 17 a3 pn
}

if [ $# -eq 0 ]; then
	wlan_init
	bt_init
fi

if [ "$1" = "wlan" ]; then
	wlan_init
fi

if [ "$1" = "bt" ]; then
	bt_init
fi

if [ "$1" = "-h" ]; then
	echo "Usage: ./rpi_init.sh [arguments]"
	echo "\nArguments are optional and are as below"
	echo "	wlan:	Insert and setup esp32 network driver"
	echo "	bt:	Set GPIO pins on RPi for bluetooth operation"
	echo "\nExample:"
	echo "	- Prepare RPi for WLAN operation: 		./rpi_init.sh wlan"
	echo "	- Prepare RPi for Bluetooth operation: 		./rpi_init.sh bt"
	echo "	- Prepare RPi for both WLAN and Bluetoorh: 	./rpi_init.sh"
fi
