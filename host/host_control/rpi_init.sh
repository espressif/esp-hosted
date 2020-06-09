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

if [ "$1" = "-h" ]; then
	echo "This script prepares RPI for wlan and bt/ble operation over esp32 device"
	echo "\nUsage: ./rpi_init.sh [arguments]"
	echo "\nArguments are optional and are as below"
	echo "	btuart:	Set GPIO pins on RPi for HCI UART operations"
	echo "\nExample:"
	echo "  - Prepare RPi for WLAN and bt/ble operation on SDIO"
	echo "	 # ./rpi_init.sh"
	echo "\n  - Prepare RPi for bt/ble operation over UART and WLAN over SDIO"
	echo "	 # ./rpi_init.sh btuart"
	exit 0
fi

sudo modprobe bluetooth
wlan_init

if [ "$1" = "btuart" ]; then
	bt_init
fi

