#!/usr/bin/env bash

# SPDX-License-Identifier: Apache-2.0
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

RESETPIN=""
BT_UART_INIT="0"
TEST_RAW_TP="0"
IF_TYPE="sdio"
MODULE_NAME="esp32_${IF_TYPE}.ko"
RPI_RESETPIN=6

bringup_network_interface()
{
	if [ "$1" != "" ] ; then
		if [ `ifconfig -a | grep $1 | wc -l` != "0" ]; then
			sudo ifconfig $1 up
		fi
	fi
}

wlan_init()
{
    if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
        if [ `lsmod | grep esp32_sdio | wc -l` != "0" ]; then
            sudo rmmod esp32_sdio &> /dev/null
            else
            sudo rmmod esp32_spi &> /dev/null
        fi
    fi

    if [ "$TEST_RAW_TP" = "0" ] ; then
        VAL_CONFIG_TEST_RAW_TP=n
    else
        VAL_CONFIG_TEST_RAW_TP=y
    fi

    # For Linux other than Raspberry Pi, Please point
    # CROSS_COMPILE -> <Toolchain-Path>/bin/arm-linux-gnueabihf-
    # KERNEL        -> Place where kernel is checked out and built
    # ARCH          -> Architecture
    make -j8 target=$IF_TYPE CROSS_COMPILE=/usr/bin/arm-linux-gnueabihf- KERNEL="/lib/modules/$(uname -r)/build" \
    CONFIG_TEST_RAW_TP="$VAL_CONFIG_TEST_RAW_TP" ARCH=arm

    if [ "$RESETPIN" = "" ] ; then
        #By Default, BCM6 is GPIO on host. use resetpin=6
        sudo insmod $MODULE_NAME resetpin=$RPI_RESETPIN
    else
        #Use resetpin value from argument
        sudo insmod $MODULE_NAME $RESETPIN
    fi

    if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
        echo "esp32 module inserted "
		sleep 4
		bringup_network_interface "espsta0"

        echo "ESP32 host init successfully completed"
    fi
}

bt_uart_init()
{
    sudo raspi-gpio set 15 a0 pu
    sudo raspi-gpio set 14 a0 pu
    sudo raspi-gpio set 16 a3 pu
    sudo raspi-gpio set 17 a3 pu
}

usage()
{
    echo "This script prepares RPI for wlan and bt/ble operation over esp32 device"
    echo "\nUsage: ./rpi_init.sh [arguments]"
    echo "\nArguments are optional and are as below"
    echo "  spi:    sets ESP32<->RPi communication over SPI"
    echo "  sdio:   sets ESP32<->RPi communication over SDIO"
    echo "  btuart: Set GPIO pins on RPi for HCI UART operations"
    echo "  resetpin=6:     Set GPIO pins on RPi connected to EN pin of ESP32, used to reset ESP32 (default:6 for BCM6)"
    echo "\nExample:"
    echo "  - Prepare RPi for WLAN operation on SDIO. sdio is default if no interface mentioned"
    echo "   # ./rpi_init.sh or ./rpi_init.sh sdio"
    echo "\n  - Use spi for host<->ESP32 communication. sdio is default if no interface mentioned"
    echo "   # ./rpi_init.sh spi"
    echo "\n  - Prepare RPi for bt/ble operation over UART and WLAN over SDIO/SPI"
    echo "   # ./rpi_init.sh sdio btuart or ./rpi_init.sh spi btuart"
    echo "\n  - use GPIO pin BCM5 (GPIO29) for reset"
    echo "   # ./rpi_init.sh resetpin=5"
    echo "\n  - do btuart, use GPIO pin BCM5 (GPIO29) for reset over SDIO/SPI"
    echo "   # ./rpi_init.sh sdio btuart resetpin=5 or ./rpi_init.sh spi btuart resetpin=5"
}

parse_arguments()
{
    while [ "$1" != "" ] ; do
        case $1 in
            --help | -h )
                usage
                exit 0
                ;;
            sdio)
                IF_TYPE=$1
                ;;
            spi)
                IF_TYPE=$1
                ;;
            resetpin=*)
                echo "Recvd Option: $1"
                RESETPIN=$1
                ;;
            btuart)
                echo "Recvd Option: $1"
                BT_UART_INIT="1"
                ;;
            rawtp)
                echo "Test RAW TP"
                TEST_RAW_TP="1"
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

if [ "$IF_TYPE" = "spi" ] ; then
    rm spidev_disabler.dtbo
    # Disable default spidev driver
    dtc spidev_disabler.dts -O dtb > spidev_disabler.dtbo
    sudo dtoverlay -d . spidev_disabler
fi

if [ `lsmod | grep bluetooth | wc -l` = "0" ]; then
    echo "bluetooth module inserted"
    sudo modprobe bluetooth
fi

if [ `lsmod | grep cfg80211 | wc -l` = "0" ]; then
    echo "cfg80211 module inserted"
    sudo modprobe cfg80211
fi

if [ `lsmod | grep bluetooth | wc -l` != "0" ]; then
    wlan_init
fi

if [ "$BT_UART_INIT" = "1" ] ; then
    bt_uart_init
fi


#alias load_module_sdio='sudo modprobe bluetooth; sudo modprobe cfg80211; sudo insmod ./esp32_sdio.ko resetpin=6; sleep 4;sudo ifconfig espsta0 up'
#alias load_module_spi='sudo dtoverlay spidev_disabler; sudo modprobe bluetooth; sudo modprobe cfg80211; sudo insmod ./esp32_spi.ko resetpin=6; sleep 4;sudo ifconfig espsta0 up'
