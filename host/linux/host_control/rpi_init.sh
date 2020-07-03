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

RESETPIN=""
BT_INIT_SET="0"
IF_TYPE="sdio"
MODULE_NAME="esp32_${IF_TYPE}.ko"

wlan_init()
{
	cd ../host_driver/esp32/
	make -j8 target=$IF_TYPE
	if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
		sudo rmmod esp32_sdio &> /dev/null
		sudo rm /dev/esps0
	fi
	if [ "$RESETPIN" = "" ] ; then
		#By Default, BCM6 is GPIO on host. use resetpin=6
		sudo insmod $MODULE_NAME resetpin=6
	else
		#Use resetpin value from argument
		sudo insmod $MODULE_NAME $RESETPIN
	fi
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

usage()
{
	echo "This script prepares RPI for wlan and bt/ble operation over esp32 device"
	echo "\nUsage: ./rpi_init.sh [arguments]"
	echo "\nArguments are optional and are as below"
	echo "	btuart:	Set GPIO pins on RPi for HCI UART operations"
	echo "	resetpin=6:	Set GPIO pins on RPi connected to EN pin of ESP, used to reset ESP (default:6 for BCM6)"
	echo "\nExample:"
	echo "  - Prepare RPi for WLAN and bt/ble operation on SDIO"
	echo "	 # ./rpi_init.sh"
	echo "\n  - Prepare RPi for bt/ble operation over UART and WLAN over SDIO"
	echo "	 # ./rpi_init.sh btuart"
	echo "\n  - use GPIO pin BCM5 (GPIO29) for reset"
	echo "	 # ./rpi_init.sh resetpin=5"
	echo "\n  - do btuart, use GPIO pin BCM5 (GPIO29) for reset"
	echo "	 # ./rpi_init.sh btuart resetpin=5"
	echo "\n  - Use sdio for host<->esp32 communication. sdio is default if no interface mentioned"
	echo "	 # ./rpi_init.sh sdio"
}

parse_arguments()
{
	while [ "$1" != "" ] ; do
		case $1 in
			--help | -h )
				usage
				exit 0
				;;

			btuart)
				echo "Recvd Option: $1"
				BT_INIT_SET="1"
				;;

			resetpin=*)
				echo "Recvd Option: $1"
				RESETPIN=$1
				;;

			sdio)
				IF_TYPE=$1
				;;

			*)
				echo "$1 : unknown option"
				usage
				exit 1
				;;
		esac
		shift
	done
}


parse_arguments $*
if [ "$IF_TYPE" = "" ] ; then
	echo "Error: No protocol selected"
	usage
	exit 1
else
	echo "Building for $IF_TYPE protocol"
	MODULE_NAME=esp32_${IF_TYPE}.ko
fi

if [ `lsmod | grep bluetooth | wc -l` = "0" ]; then
	echo "bluetooth module inserted"
	sudo modprobe bluetooth
fi

if [ `lsmod | grep bluetooth | wc -l` != "0" ]; then
	wlan_init
fi

if [ "$BT_INIT_SET" = "1" ] ; then
	bt_init
fi
