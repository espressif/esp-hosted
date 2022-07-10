#!/bin/sh

# Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
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

RESETPIN="resetpin=60"
BT_INIT_SET="0"
IF_TYPE="spi"
MODULE_NAME="esp32_${IF_TYPE}.ko"

echo "setup default HANDSHAKE_PIN (GPIO_49) and SPI_DATA_READY_PIN (GPIO_48)\n"
sed -i 's/#define SPI_DATA_READY_PIN.*/#define SPI_DATA_READY_PIN 48/g' ../host_driver/esp32/spi/esp_spi.h
sed -i 's/#define HANDSHAKE_PIN.*/#define HANDSHAKE_PIN 49/g' ../host_driver/esp32/spi/esp_spi.h

build_c_demo_app()
{
    cd c_support/
    make clean
    make -j8
    make -j8 stress
    cd ..
}

build_python_demo_app()
{
    cd python_support/
    make clean
    make -j8
    cd ..
}

wlan_init()
{
    build_c_demo_app
    build_python_demo_app

    cd ../host_driver/esp32/
    if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
        sudo rm /dev/esps0
        if [ `lsmod | grep esp32_sdio | wc -l` != "0" ]; then
            sudo rmmod esp32_sdio &> /dev/null
            else
            sudo rmmod esp32_spi &> /dev/null
        fi
    fi

    make -j8 target=$IF_TYPE CROSS_COMPILE=/usr/bin/arm-linux-gnueabihf- KERNEL="/lib/modules/$(uname -r)/build" ARCH=arm

    if [ "$RESETPIN" = "" ] ; then
        echo "By Default, use resetpin=60"
        sudo insmod $MODULE_NAME resetpin=60
    else
        echo "Use resetpin value from argument"
        sudo insmod $MODULE_NAME $RESETPIN
    fi
    if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
        echo "esp32 module inserted "
        sudo mknod /dev/esps0 c 221 0
        sudo chmod 666 /dev/esps0
        echo "/dev/esps0 device created"
        echo "BBB init successfully completed"
    fi
}

load_overlay()
{
	echo "build .dtbo"

	dtc -O dtb -o BB-SPI0-01-00A0.dtbo -b 0 -@ BBB_spidev_disabler.dts

	echo "cp BB-SPI0-01-00A0.dtbo /lib/firmware/"
	sudo mv BB-SPI0-01-00A0.dtbo /lib/firmware/

	echo "updating /boot/uEnv.txt for active overlay. default I'll update overlay entry 0\n"

	sudo sed -Ei 's/#?uboot_overlay_addr0.*/uboot_overlay_addr0=\/lib\/firmware\/BB-SPI0-01-00A0.dtbo/g' /boot/uEnv.txt
}


usage()
{
    echo "This script prepares BBB for wlan and bt/ble operation over esp32 device"
    echo "\nUsage: ./bbb_init.sh [arguments]"
    echo "\nArguments are optional and are as below"
    echo "  load_dtoverlay: to disable SPI0 which is used by default linux device driver (spidev)"
    echo "  spi:    sets ESP32<->BBB communication over SPI"
    echo "  resetpin=60:     Set GPIO pins on BBB connected to EN pin of ESP32, used to reset ESP32 (default:60 for P9_12 )"
    echo "\nExample:"
    echo "\n  - Use spi for host<->ESP32 communication"
    echo "   # ./bbb.sh spi"
    echo "\n  - use GPIO pin P9_12 (GPIO_60) for reset"
    echo "   # ./bbb.sh resetpin=60"
}

parse_arguments()
{
    while [ "$1" != "" ] ; do
        case $1 in
            --help | -h )
                usage
                exit 0
                ;;
            spi)
                IF_TYPE=$1
                ;;
            load_dtoverlay)
	        load_overlay
		echo "NEED TO REBOOT to take effect. I'll reboot in 10s. This only need only one time."
		sleep 10; reboot
		;;
            resetpin=*)
                echo "Recvd Option: $1"
                RESETPIN=$1
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
