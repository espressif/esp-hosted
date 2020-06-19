#!/bin/sh

# Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

wlan_init()
{
	cd ../host_driver/esp32/
	make -j8
	if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
		sudo rmmod esp32
		sudo rm /dev/esps0
	fi
	sudo insmod esp32.ko
	if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
		echo "esp32 module inserted "
		sudo mknod /dev/esps0 c 221 0
		sudo chmod 666 /dev/esps0
		echo "/dev/esps0 device created"
		echo "RPi init successfully completed"
	fi
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

if [ `lsmod | grep bluetooth | wc -l` = "0" ]; then
	echo "bluetooth module inserted"
	sudo modprobe bluetooth
fi

if [ `lsmod | grep bluetooth | wc -l` != "0" ]; then
	wlan_init
fi

if [ "$1" = "btuart" ]; then
	bt_init
fi
