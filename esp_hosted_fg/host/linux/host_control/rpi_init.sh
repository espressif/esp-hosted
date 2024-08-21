#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Espressif Systems Wireless LAN device driver
#
# SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
#
# SPDX-License-Identifier: GPL-2.0-only
#

##################  USER config (porting) ######################
# Module param, resetpin: Mandatory for SPI
resetpin="518"

# Module param, clockspeed : clock freq of SPI or SDIO - Optional
# Defaults: SPI: 10MHz SDIO: As per device tree, typically 25MHz or 50MHz
clockspeed="10"

# Module param, spi_bus: spi bus instance to use
spi_bus="0"
# Module param, spi_cs: spi cs instance to use
spi_cs="0"
# Module param, spi_mode: spi mode to use from (1/2/3)
spi_mode="2"
# Module param, spi_handshake: spi handshake GPIO to use
spi_handshake="534"
# Module param, spi_dataready: spi dataready GPIO to use
spi_dataready="539"

##################  Local params for host ######################
cpu_perf="on"

# Old Raspberry Pi config
#resetpin="6"
#spi_handshake="22"
#spi_dataready="27"

XTRA_MODULE_PARAMS=""

##################  Bluetooth Config ###########################
# 1) Bluetooth over HCI : Bluetooth over spi/sdio
#    No extra pins or config needed. Reuse SPI/SDIO pins for Wi-Fi+Bluetooth
#    Use: BT_CONFIG="bt_using_hci"
# 2) Bluetooth over UART : Wi-Fi over spi/sdio. Bluetooth over dedicated UART
#    Use: BT_CONFIG="bt_using_uart_2pins" OR
#         BT_CONFIG="bt_using_uart_4pins"
#    2 pins UART: Tx and Rx only
#    4 pins UART: Tx, Rx, RTS, CTS.
#    Extra configuration needed!
#    - Connect extra pins
#    - Configure these 2/4 UART pins in your device tree as UART pins
# 3) Disable Bluetooth (Only Wi-Fi)
#    Use: BT_CONFIG=""
BT_CONFIG=""
#BT_CONFIG="bt_using_hci"
#BT_CONFIG="bt_using_uart_2pins"
#BT_CONFIG="bt_using_uart_4pins"


##################  Wi-Fi config  ##############################
#  1) wifi enabled : 'wifi_on' (default)
#  2) wifi disabled : 'wifi_off'
WIFI_CONFIG="wifi_on"
#WIFI_CONFIG="wifi_off"

############  Cross compilation options ########################
CROSS_COMPILE="/bin/arm-linux-gnueabihf-"
KERNEL_BUILD_DIR="/lib/modules/$(uname -r)/build"
ARCH=""

TEST_RAW_TP="0"
IF_TYPE=""
MODULE_NAME="esp32_${IF_TYPE}.ko"


log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

warn() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] **Warn** $1"
}

log_enter() {
    log "Entering ${FUNCNAME[1]}"
}

log_exit() {
    log "Exiting ${FUNCNAME[1]}"
}

device_tree_dependency_spi() {
    log_enter
	cd $SCRIPT_DIR
	log "Current dir: $PWD"
    rm spidev_disabler.dtbo > /dev/null
    dtc spidev_disabler.dts -O dtb > spidev_disabler.dtbo
    sudo dtoverlay -d . spidev_disabler
	cd $MAKE_DIR
    log_exit
}

device_tree_dependency_uart_2pins() {
    log_enter
	gpio_pin_ctl=pinctrl
	$gpio_pin_ctl 2> /dev/null
	if [ $? -ne 0 ]; then
		gpio_pin_ctl=raspi-gpio
	fi
    sudo $gpio_pin_ctl set 15 a0 pu
    sudo $gpio_pin_ctl set 14 a0 pu
    log_exit
}

device_tree_dependency_uart_4pins() {
    log_enter
	gpio_pin_ctl=pinctrl
	$gpio_pin_ctl 2> /dev/null
	if [ $? -ne 0 ]; then
		gpio_pin_ctl=raspi-gpio
	fi
    sudo $gpio_pin_ctl set 15 a0 pu
    sudo $gpio_pin_ctl set 14 a0 pu
    sudo $gpio_pin_ctl set 16 a3 pu
    sudo $gpio_pin_ctl set 17 a3 pu
    log_exit
}

build_c_demo_app() {
    log_enter
    cd $SCRIPT_DIR/c_support/
    make clean
    make -j8 test
    if [ $? -ne 0 ]; then
        log "Failed to build test app"
        exit 1
    fi

    make -j8 stress
    if [ $? -ne 0 ]; then
        log "Failed to build stress app"
        exit 1
    fi
    cd ..
    log_exit
}

build_python_demo_app() {
    log_enter
    cd $SCRIPT_DIR/python_support/
    make clean
    make -j8
    if [ $? -ne 0 ]; then
        log "Failed to build python demo app"
        exit 1
    fi
    cd ..
    log_exit
}

build_user_space_apps() {
    log_enter
    build_c_demo_app
    build_python_demo_app
    log_exit
}

remove_module() {
    log_enter
    if [ "$(lsmod | grep esp32 | wc -l)" != "0" ]; then
        if [ "$(lsmod | grep esp32_sdio | wc -l)" != "0" ]; then
            sudo rmmod esp32_sdio &> /dev/null
            else
            sudo rmmod esp32_spi &> /dev/null
        fi
        if [ $? -ne 0 ]; then
            log "Failed to remove esp kernel module"
            exit 1
        fi
        log "esp module removed using script"
    fi
    log_exit
}

build_module()
{
    if [ "$TEST_RAW_TP" = "0" ] ; then
        VAL_CONFIG_TEST_RAW_TP=n
    else
        VAL_CONFIG_TEST_RAW_TP=y
    fi

    if [ "$BT_CONFIG" != "" ] ; then
        VAL_BT_ENABLED=y
    else
        VAL_BT_ENABLED=n
    fi

    # For Linux other than Raspberry Pi, Please uncomment below 'make' line and provide:
    # <CROSS_COMPILE> -> <Toolchain-Path>/bin/arm-linux-gnueabihf-
    # <KERNEL>        -> Place where kernel is checked out and built. For Example, "/lib/modules/$(uname -r)/build"
    # <ARCH>          -> Architecture. for example, arm64

    #make -j8 target=$IF_TYPE CROSS_COMPILE=<CROSS_COMPILE> KERNEL=<KERNEL> ARCH=<ARCH> CONFIG_TEST_RAW_TP="$VAL_CONFIG_TEST_RAW_TP"

    # Also, Check detailed doc, esp_hosted_fg/docs/Linux_based_host/porting_guide.md for more details.

    # Populate your arch if not populated correctly.
	if [ "$ARCH" = "" ]; then
		arch_num_bits=$(getconf LONG_BIT)
		if [ "$arch_num_bits" = "32" ] ; then arch_found="arm"; else arch_found="arm64"; fi
		ARCH="$arch_found"
	fi

	cd $MAKE_DIR
	log "Current dir: $PWD"
	log "Building for target $IF_TYPE"
	log "Using ARCH as : $ARCH"
	log "Using KERNEL as $KERNEL_BUILD_DIR"
	log "Using CONFIG_TEST_RAW_TP as $CONFIG_TEST_RAW_TP"

    make -j8 target="$IF_TYPE" KERNEL="$KERNEL_BUILD_DIR" ARCH="$ARCH" CONFIG_TEST_RAW_TP="$VAL_CONFIG_TEST_RAW_TP" CONFIG_BT_ENABLED="$VAL_BT_ENABLED"

    # Check the exit status of make
    if [ $? -ne 0 ] ; then
        log "Failed to build the esp kernel module"
        exit -1
    fi
}

load_bluetooth_module()
{
	sudo modprobe bluetooth

	if [ `lsmod | grep bluetooth | wc -l` = "0" ]; then
		log "Failed to insert bluetooth module, comment if bluetooth is configured directly in kernel"
		exit -1
	fi
}

insert_module() {
    log_enter
    # Check and apply bluetooth config
    if [ "$BT_CONFIG" != "" ]; then
        load_bluetooth_module

        if [ "$BT_CONFIG" = "bt_using_uart_2pins" ]; then
			log "Wi-Fi ($IF_TYPE) + Bluetooth (UART-2pins)"
            device_tree_dependency_uart_2pins
        elif [ "$BT_CONFIG" = "bt_using_uart_4pins" ]; then
			log "Wi-Fi ($IF_TYPE) + Bluetooth (UART-4pins)"
            device_tree_dependency_uart_4pins
        elif [ "$BT_CONFIG" = "bt_using_hci" ]; then
			log "$IF_TYPE only setup: Wi-Fi+Bluetooth both over $IF_TYPE"
		else
            log "Incorrect bluetooth config"
            exit 1
        fi
    fi

    # Additional dependency for SPI
    if [ "$IF_TYPE" = "spi" ]; then
        device_tree_dependency_spi
    fi

	# Insert module with parameters
	sudo insmod $MODULE_NAME $XTRA_MODULE_PARAMS

    if [ $? -ne 0 ]; then
        log "Failed to insert module"
        exit 1
    fi

    if [ "$(lsmod | grep esp32 | wc -l)" != "0" ]; then
        log "esp32 module inserted using script"
    else
        log "Failed to insert esp32 module"
        exit 1
    fi
    log_exit
}

parse_arguments() {
    log_enter
    while [ "$1" != "" ]; do
        case $1 in
            --help | -h )
                usage
                exit 0
                ;;
			wifi=*)
                WIFI_TP=${1#*=}
				if [ "$WIFI_TP" = "spi" ]; then
					log "Wi-Fi on SPI"
				elif [ "$WIFI_TP" = "sdio" ]; then
					log "Wi-Fi on SDIO"
				elif [ "$WIFI_TP" = "-" ]; then
					log "****** Disable Wi-Fi ******"
					WIFI_TP=""
				else
					log "****** Unsupported Wi-Fi transport[$WIFI_TP] ******"
					log "Possible transport values: 'spi' / 'sdio' / '-'"
					exit 1
				fi
                ;;
            resetpin=*)
                log "Recvd Option: $1"
                resetpin=${1#*=}
                ;;
			bt=*)
                log "Recvd Option: $1"
				BT_TP=${1#*=}
				if [ "$BT_TP" = "spi" ]; then
					BT_CONFIG="bt_using_hci"
					log "BT on SPI"
				elif [ "$BT_TP" = "sdio" ]; then
					BT_CONFIG="bt_using_hci"
					log "BT on SDIO"
				elif [ "$BT_TP" = "uart_2pins" ]; then
					BT_CONFIG="bt_using_uart_2pins"
					log "BT on UART (Tx, RX)"
				elif [ "$BT_TP" = "uart_4pins" ]; then
					BT_CONFIG="bt_using_uart_4pins"
					log "BT on UART (Tx, RX, CTS, RTS)"
				elif [ "$BT_TP" = "-" ]; then
					BT_CONFIG=""
					log "****** Disable BT ******"
					BT_TP=""
				else
					log "****** Unsupported BT transport[$BT_TP] ******"
					log "Possible transport values: 'spi' / 'sdio' / 'uart_2pins' / 'uart_4pins' / '-'"
					exit 1
				fi
                ;;
            rawtp)
                log "Test RAW TP"
                TEST_RAW_TP="1"
                ;;
            clockspeed=*)
                clockspeed=${1#*=}
                log "Clock freq: $clockspeed MHz"
                ;;
			spi_bus=*)
				spi_bus=${1#*=}
				log "SPI bus: $spi_bus"
				;;
			spi_cs=*)
				spi_cs=${1#*=}
				log "SPI CS: $spi_cs"
				;;
			spi_mode=*)
				spi_mode=${1#*=}
				log "SPI Mode: $spi_mode"
				;;
			spi_handshake=*)
				spi_handshake=${1#*=}
				log "SPI handshake gpio: $spi_handshake"
				;;
			spi_dataready=*)
				spi_dataready=${1#*=}
				log "SPI dataready gpio: $spi_dataready"
				;;
			cpu_perf=*)
				cpu_perf=${1#*=}
				log "Set CPU performance: $cpu_perf"
				;;
            *)
                log "$1 : unknown option"
                usage
                exit 1
                ;;
        esac
        shift
    done

	verify_transport_combination
	verify_clock_freq
    log_exit
}

verify_transport_combination()
{
	if [ "$WIFI_TP" != "" ]; then
		IF_TYPE=$WIFI_TP
	fi

	if [ "$BT_TP" != "" ]; then
		if [ "$BT_TP" != "uart_2pins" ] && [ "$BT_TP" != "uart_4pins" ]; then
			if [ "$WIFI_TP" != "" ] && [ "$BT_TP" != "$WIFI_TP" ]; then
				log "Transport combination unsupported: Wifi[$WIFI_TP] BT[$BT_TP]"
				exit 1
			fi
			IF_TYPE=$BT_TP
		fi
	fi

	if [ "$IF_TYPE" = "" ] ; then
		echo ""
		log "transport combination unsupported. At least, Wi-Fi or BT need to run over SPI/SDIO"
		echo ""
		usage
		exit 1
	fi
}

verify_clock_freq()
{
	if [ "$clockspeed" != "" ]; then
		if [ "$IF_TYPE" = "spi" ] && [ $clockspeed -gt 40 ]; then
			log "SPI slave clock freq [$clockspeed]  not supported"
			exit 1
		elif [ "$IF_TYPE" = "sdio" ] && [ $clockspeed -gt 50 ]; then
			log "SDIO slave clock [$clockspeed] not supported"
			exit 1
		fi
	fi
}

add_module_param()
{
	local param_name=$1
	if [ "${!param_name}" = "" ]; then
		warn "Param, $param_name not configured, ignoring"
	else
		XTRA_MODULE_PARAMS="$XTRA_MODULE_PARAMS $param_name=${!param_name}"
		log "Adding module_param '$param_name=${!param_name}'"
	fi
}

# Example usage function
usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --help, -h                   Show this help message"
    echo "  wifi=<value>                 Set bluetooth transport"
    echo "     > 'sdio'                       <Use Wi-Fi over SDIO>"
    echo "     > 'spi'                        <Use Wi-Fi over SPI>"
    echo "     > '-'                          <Disable Wi-Fi>"
    echo "  resetpin=<gpio>              Set the reset pin GPIO"
    echo "  bt=<value>                   Set bluetooth transport"
    echo "     > 'spi'                        <Use bluetooth over SPI>"
    echo "     > 'sdio'                       <Use bluetooth over SDIO>"
    echo "     > 'uart_2pins'                 <Use bluetooth over UART Tx,Rx>"
    echo "     > 'uart_4pins'                 <Use bluetooth over UART Tx,Rx,CTS,RTS>"
    echo "     > '-'                          <Disable bluetooth>"
    echo "  clockspeed=<freq_in_mhz>     Set SPI/SDIO clock frequency to be used"
    echo "                                     SPI Default: 10MHz"
    echo "                                     SDIO Default: As per Device Tree (25 or 50MHz)"
    echo "  spi_bus=<num>                Use this SPI bus instance"
    echo "  spi_cs=<num>                 Use this ChipSelect instance"
    echo "  spi_mode=<num>               Use this SPI mode"
    echo "  spi_handshake=<gpio_num>     SPI Handshake GPIO"
    echo "  spi_dataready=<gpio_num>     SPI DataReady GPIO"
    echo "  rawtp                        Test RAW TP"
    echo "  cpu_perf=<on/off>            Change cpu performance level(may need porting)"
    echo ""
}


populate_module_params()
{
	# Populate module params
	add_module_param "resetpin"
	add_module_param "clockspeed"

    if [ "$IF_TYPE" = "spi" ]; then
		add_module_param "spi_bus"
		add_module_param "spi_cs"
		add_module_param "spi_mode"
		add_module_param "spi_handshake"
		add_module_param "spi_dataready"
	fi
}

port_populate_local_params()
{
	if [ "$cpu_perf" = "on" ] ; then
		for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
			sudo sh -c "echo performance > $cpu"
		done
	elif [ "$cpu_perf" = "off" ] ; then
		for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
			sudo sh -c "echo ondemand > $cpu"
		done
	fi

	echo "Current CPU governor settings:"
	for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
		echo "$cpu: $(cat $cpu)"
	done
}


######## Script ##########
SCRIPT_DIR=$PWD
MAKE_DIR=$SCRIPT_DIR/../host_driver/esp32/

cd $MAKE_DIR
parse_arguments "$@"

log "Building for $IF_TYPE protocol"
MODULE_NAME=esp32_${IF_TYPE}.ko

populate_module_params
port_populate_local_params

build_user_space_apps

remove_module
build_module
insert_module


log "--- finished ---"
