#!/bin/bash

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
BT_INIT_SET="0"
TEST_RAW_TP="0"
IF_TYPE="sdio"
MODULE_NAME="esp32_${IF_TYPE}.ko"
ESP_SLAVE_CHIPSET=""
#ESP_SLAVE_CHIPSET could be one of esp32, esp32c2, esp32c3, esp32c6, esp32s2, esp32s3
#For now Slave chipset is needed to distinguish between ESP32 and ESP32-C6 chipset for SDIO protocol

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

demo_app_building()
{
    build_c_demo_app
    build_python_demo_app
}

wlan_init()
{
    cd ../host_driver/esp32/
    if [ `lsmod | grep esp32 | wc -l` != "0" ]; then
        sudo rm /dev/esps0
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

    if [ "$ESP_SLAVE" != "" ] ; then
	    CUSTOM_OPTS=${CUSTOM_OPTS}" ESP_SLAVE=\"$ESP_SLAVE"\"
    fi
    if [ "$CUSTOM_OPTS" != "" ] ; then
	    echo "Adding $CUSTOM_OPTS"
    fi

    # For Linux other than Raspberry Pi, Please uncomment below 'make' line and provide:
    # <CROSS_COMPILE> -> <Toolchain-Path>/bin/arm-linux-gnueabihf-
    # <KERNEL>        -> Place where kernel is checked out and built. For Example, "/lib/modules/$(uname -r)/build"
    # <ARCH>          -> Architecture. for example, arm64

    #make -j8 target=$IF_TYPE CROSS_COMPILE=<CROSS_COMPILE> KERNEL=<KERNEL> ARCH=<ARCH> CONFIG_TEST_RAW_TP="$VAL_CONFIG_TEST_RAW_TP" $CUSTOM_OPTS

    # Also, Check detailed doc, esp_hosted_fg/docs/Linux_based_host/porting_guide.md for more details.

    # Populate your arch if not populated correctly.
    arch_num_bits=$(getconf LONG_BIT)
    if [ "$arch_num_bits" = "32" ] ; then arch_found="arm"; else arch_found="arm64"; fi

    make -j8 target=$IF_TYPE KERNEL="/lib/modules/$(uname -r)/build" ARCH=$arch_found CONFIG_TEST_RAW_TP="$VAL_CONFIG_TEST_RAW_TP" $CUSTOM_OPTS


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
    sudo raspi-gpio set 15 a0 pu
    sudo raspi-gpio set 14 a0 pu
    if [ "$BT_INIT_SET" = "4" ] ; then
        sudo raspi-gpio set 16 a3 pu
        sudo raspi-gpio set 17 a3 pu
    fi
}

usage()
{
    echo "This script prepares RPI for wlan and bt/ble operation over esp32 device"
    echo "\nUsage: ./rpi_init.sh [arguments]"
    echo "\nArguments are optional and are as below"
    echo "  spi:    sets ESP32<->RPi communication over SPI"
    echo "  sdio:   sets ESP32<->RPi communication over SDIO"
    echo "  btuart: Set GPIO pins on RPi for HCI UART operations with TX, RX, CTS, RTS (defaulted to option btuart_4pins)"
    echo "  btuart_2pins: Set GPIO pins on RPi for HCI UART operations with only TX & RX pins configured (only for ESP32-C2/C6)"
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
            btuart | btuart_4pins | btuart_4pin)
                echo "Configure Host BT UART with 4 pins, RX, TX, CTS, RTS"
                BT_INIT_SET="4"
                ;;
            btuart_2pins | btuart_2pin)
                echo "Configure Host BT UART with 2 pins, RX & TX"
                BT_INIT_SET="2"
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

select_esp_slave()
{
    case $ESP_SLAVE_CHIPSET in
        [Ee][Ss][Pp]32)
            echo "Building for esp32"
			ESP_SLAVE='CONFIG_TARGET_ESP32=y'
            ;;
        [Ee][Ss][Pp]32[Cc]2)
            echo "Building for esp32c2"
			ESP_SLAVE='CONFIG_TARGET_ESP32C2=y'
            ;;
        [Ee][Ss][Pp]32-[Cc]2)
            echo "Building for esp32c2"
			ESP_SLAVE='CONFIG_TARGET_ESP32C2=y'
            ;;
        [Ee][Ss][Pp]32[Cc]3)
            echo "Building for esp32c3"
			ESP_SLAVE='CONFIG_TARGET_ESP32C3=y'
            ;;
        [Ee][Ss][Pp]32-[Cc]3)
            echo "Building for esp32c3"
			ESP_SLAVE='CONFIG_TARGET_ESP32C3=y'
            ;;
        [Ee][Ss][Pp]32[Cc]5)
            echo "esp32c5 Not yet supported"
			ESP_SLAVE='CONFIG_TARGET_ESP32C5=y'
            ;;
        [Ee][Ss][Pp]32-[Cc]5)
            echo "esp32c5 Not yet supported"
			ESP_SLAVE='CONFIG_TARGET_ESP32C5=y'
            ;;
        [Ee][Ss][Pp]32[Cc]6)
            echo "Building for esp32c6"
			ESP_SLAVE='CONFIG_TARGET_ESP32C6=y'
            ;;
        [Ee][Ss][Pp]32-[Cc]6)
            echo "Building for esp32c6"
			ESP_SLAVE='CONFIG_TARGET_ESP32C6=y'
            ;;
        [Ee][Ss][Pp]32[Ss]2)
            echo "Building for esp32s2"
			ESP_SLAVE='CONFIG_TARGET_ESP32S2=y'
            ;;
        [Ee][Ss][Pp]32-[Ss]2)
            echo "Building for esp32s2"
			ESP_SLAVE='CONFIG_TARGET_ESP32S2=y'
            ;;
        [Ee][Ss][Pp]32[Ss]3)
            echo "Building for esp32s3"
			ESP_SLAVE='CONFIG_TARGET_ESP32S3=y'
            ;;
        [Ee][Ss][Pp]32-[Ss]3)
            echo "Building for esp32s3"
			ESP_SLAVE='CONFIG_TARGET_ESP32S3=y'
            ;;
        [Ee][Ss][Pp]32[Pp]4)
            echo "Not yet supported esp32p4"
			ESP_SLAVE='CONFIG_TARGET_ESP32P4=y'
            ;;
        [Ee][Ss][Pp]32-[Pp]4)
            echo "Not yet supported esp32p4"
			ESP_SLAVE='CONFIG_TARGET_ESP32P4=y'
            ;;
        *)
            echo "***** Err: Please set expected ESP slave chipset ****"
			exit 1
            ;;
    esac

}


parse_arguments $*
if [ "$IF_TYPE" = "" ] ; then
    echo "Error: No protocol selected"
    usage
    exit 1
else
	if [ "$IF_TYPE" = "sdio" ] ; then
		# SDIO Kernel driver registration varies for ESP32 and ESP32-C6 slave chipsets
		select_esp_slave
	fi
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

demo_app_building

if [ `lsmod | grep bluetooth | wc -l` != "0" ]; then
    wlan_init
fi

if [ "$BT_INIT_SET" != "0" ] ; then
    bt_init
fi
