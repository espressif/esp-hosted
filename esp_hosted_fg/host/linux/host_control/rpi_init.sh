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
CROSS_COMPILE=""
KERNEL_BUILD_DIR="/lib/modules/$(uname -r)/build"
ARCH=""

############  Script control options ###########################
TEST_RAW_TP="0"
DRY_RUN="0"
SKIP_BUILD_APPS="0"
SKIP_MODULE_BUILD="0"
USE_COLOR="1"

IF_TYPE=""
MODULE_NAME="esp32_${IF_TYPE}.ko"

############  Color definitions ################################
if [ "$USE_COLOR" = "1" ] && [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    CYAN='\033[0;36m'
    BOLD='\033[1m'
    NC='\033[0m' # No Color
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    CYAN=''
    BOLD=''
    NC=''
fi


log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

info() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

warn() {
    echo -e "${YELLOW}[$(date +'%Y-%m-%d %H:%M:%S')] Warning:${NC} $1"
}

error() {
    echo -e "${RED}[$(date +'%Y-%m-%d %H:%M:%S')] Error:${NC} $1"
}

success() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1"
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
        error "Failed to build C test app"
        exit 1
    fi

    make -j8 stress
    if [ $? -ne 0 ]; then
        error "Failed to build C stress app"
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
        error "Failed to build Python demo app"
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
            error "Failed to remove esp kernel module"
            exit 1
        fi
        success "Removed existing esp32 kernel module"
    else
        info "No existing esp32 module loaded"
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
        error "Failed to build the esp kernel module"
        exit 1
    fi
    success "Kernel module built successfully"
}

load_bluetooth_module()
{
	sudo modprobe bluetooth

	if [ `lsmod | grep bluetooth | wc -l` = "0" ]; then
		error "Failed to load bluetooth kernel module"
		info "If bluetooth is built directly into kernel, this can be ignored"
		exit 1
	fi
	success "Bluetooth module loaded"
}

insert_module() {
    log_enter
    # Check and apply bluetooth config
    if [ "$BT_CONFIG" != "" ]; then
        load_bluetooth_module

        if [ "$BT_CONFIG" = "bt_using_uart_2pins" ]; then
			info "Wi-Fi ($IF_TYPE) + Bluetooth (UART-2pins)"
            device_tree_dependency_uart_2pins
        elif [ "$BT_CONFIG" = "bt_using_uart_4pins" ]; then
			info "Wi-Fi ($IF_TYPE) + Bluetooth (UART-4pins)"
            device_tree_dependency_uart_4pins
        elif [ "$BT_CONFIG" = "bt_using_hci" ]; then
			info "$IF_TYPE setup: Wi-Fi+Bluetooth both over $IF_TYPE"
		else
            error "Incorrect bluetooth config: $BT_CONFIG"
            exit 1
        fi
    fi

    # Additional dependency for SPI
    if [ "$IF_TYPE" = "spi" ]; then
        device_tree_dependency_spi
    fi

	# Insert module with parameters
	info "Inserting module: $MODULE_NAME $XTRA_MODULE_PARAMS"
	sudo insmod $MODULE_NAME $XTRA_MODULE_PARAMS

    if [ $? -ne 0 ]; then
        error "Failed to insert module $MODULE_NAME"
        info "Check dmesg for details"
        exit 1
    fi

    if [ "$(lsmod | grep esp32 | wc -l)" != "0" ]; then
        success "ESP32 kernel module inserted successfully"
    else
        error "Module insertion reported success but not found in lsmod"
        exit 1
    fi
    log_exit
}

parse_arguments() {
    log_enter
    while [ "$1" != "" ]; do
		# Strip leading -- to make it optional and normalize _ to - for compatibility
		arg="$1"
		arg="${arg#--}"
		# Support both dashes and underscores for backward compatibility
		arg="${arg//_/-}"

        case $arg in
            help | -h )
                usage
                exit 0
                ;;
			wifi=* | wifi-transport=*)
                WIFI_TP=${arg#*=}
				if [ "$WIFI_TP" = "spi" ]; then
					log "Wi-Fi on SPI"
				elif [ "$WIFI_TP" = "sdio" ]; then
					log "Wi-Fi on SDIO"
				elif [ "$WIFI_TP" = "-" ]; then
					log "****** Disable Wi-Fi ******"
					WIFI_TP=""
				else
					error "Unsupported Wi-Fi transport[$WIFI_TP]"
					info "Possible transport values: 'spi' / 'sdio' / '-'"
					exit 1
				fi
                ;;
            resetpin=*)
                log "Recvd Option: $1"
                resetpin=${arg#*=}
                ;;
			bt=* | bt-transport=*)
                log "Recvd Option: $1"
				BT_TP=${arg#*=}
				if [ "$BT_TP" = "spi" ]; then
					BT_CONFIG="bt_using_hci"
					log "BT on SPI"
				elif [ "$BT_TP" = "sdio" ]; then
					BT_CONFIG="bt_using_hci"
					log "BT on SDIO"
				elif [ "$BT_TP" = "uart-2pins" ] || [ "$BT_TP" = "uart_2pins" ]; then
					BT_CONFIG="bt_using_uart_2pins"
					log "BT on UART (Tx, RX)"
				elif [ "$BT_TP" = "uart-4pins" ] || [ "$BT_TP" = "uart_4pins" ]; then
					BT_CONFIG="bt_using_uart_4pins"
					log "BT on UART (Tx, RX, CTS, RTS)"
				elif [ "$BT_TP" = "-" ]; then
					BT_CONFIG=""
					log "****** Disable BT ******"
					BT_TP=""
				else
					error "Unsupported BT transport[$BT_TP]"
					info "Possible values: 'spi' / 'sdio' / 'uart-2pins' / 'uart-4pins' / '-'"
					info "Note: Both 'uart-2pins' and 'uart_2pins' formats are accepted"
					exit 1
				fi
                ;;
            rawtp)
                log "Test RAW TP"
                TEST_RAW_TP="1"
                ;;
            clockspeed=*)
                clockspeed=${arg#*=}
                log "Clock freq: $clockspeed MHz"
                ;;
			spi-bus=*)
				spi_bus=${arg#*=}
				log "SPI bus: $spi_bus"
				;;
			spi-cs=*)
				spi_cs=${arg#*=}
				log "SPI CS: $spi_cs"
				;;
			spi-mode=*)
				spi_mode=${arg#*=}
				log "SPI Mode: $spi_mode"
				;;
			spi-handshake=*)
				spi_handshake=${arg#*=}
				log "SPI handshake gpio: $spi_handshake"
				;;
			spi-dataready=*)
				spi_dataready=${arg#*=}
				log "SPI dataready gpio: $spi_dataready"
				;;
			cpu-perf=*)
				cpu_perf=${arg#*=}
				log "Set CPU performance: $cpu_perf"
				;;
			cross-compile=*)
				CROSS_COMPILE=${arg#*=}
				log "Cross compile toolchain: $CROSS_COMPILE"
				;;
			kernel=*)
				KERNEL_BUILD_DIR=${arg#*=}
				log "Kernel build directory: $KERNEL_BUILD_DIR"
				;;
			arch=*)
				ARCH=${arg#*=}
				log "Architecture: $ARCH"
				;;
			dry-run)
				DRY_RUN="1"
				info "Dry-run mode enabled (no actual changes)"
				;;
			skip-build-apps)
				SKIP_BUILD_APPS="1"
				log "Skipping user-space app builds"
				;;
			skip-module-build)
				SKIP_MODULE_BUILD="1"
				log "Skipping kernel module build"
				;;
			no-color)
				USE_COLOR="0"
				RED=''
				GREEN=''
				YELLOW=''
				NC=''
				;;
            *)
                error "$1 : unknown option"
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
				error "Transport combination unsupported: Wi-Fi[$WIFI_TP] BT[$BT_TP]"
				info "BT and Wi-Fi must use same transport (spi/sdio) unless BT uses UART"
				exit 1
			fi
			IF_TYPE=$BT_TP
		fi
	fi

	if [ "$IF_TYPE" = "" ] ; then
		echo ""
		error "No valid transport configured"
		info "At least Wi-Fi or BT must run over SPI/SDIO"
		echo ""
		usage
		exit 1
	fi
}

verify_clock_freq()
{
	if [ "$clockspeed" != "" ]; then
		if [ "$IF_TYPE" = "spi" ] && [ $clockspeed -gt 40 ]; then
			error "SPI clock frequency [$clockspeed MHz] exceeds maximum (40 MHz)"
			exit 1
		elif [ "$IF_TYPE" = "sdio" ] && [ $clockspeed -gt 50 ]; then
			error "SDIO clock frequency [$clockspeed MHz] exceeds maximum (50 MHz)"
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
    echo ""
    echo "ESP-Hosted Driver Initialization Script"
    echo "========================================"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Note: All options can be used with or without '--' prefix"
    echo "      Both dashes and underscores are supported (e.g., 'spi-bus' or 'spi_bus')"
    echo ""
    echo "Transport Configuration:"
    echo "  wifi=<value>                 Set Wi-Fi transport"
    echo "  wifi-transport=<value>       (alias for wifi=)"
    echo "     'sdio'                      Use Wi-Fi over SDIO"
    echo "     'spi'                       Use Wi-Fi over SPI"
    echo "     '-'                         Disable Wi-Fi"
    echo ""
    echo "  bt=<value>                   Set Bluetooth transport"
    echo "  bt-transport=<value>         (alias for bt=)"
    echo "     'spi'                       Use Bluetooth over SPI"
    echo "     'sdio'                      Use Bluetooth over SDIO"
    echo "     'uart-2pins'                Use Bluetooth over UART (Tx,Rx)"
    echo "     'uart-4pins'                Use Bluetooth over UART (Tx,Rx,CTS,RTS)"
    echo "     '-'                         Disable Bluetooth"
    echo ""
    echo "GPIO Configuration:"
    echo "  resetpin=<gpio>              Reset pin GPIO number"
    echo "  spi-handshake=<gpio>         SPI Handshake GPIO"
    echo "  spi-dataready=<gpio>         SPI DataReady GPIO"
    echo ""
    echo "SPI Configuration:"
    echo "  spi-bus=<num>                SPI bus instance (default: 0)"
    echo "  spi-cs=<num>                 SPI ChipSelect instance (default: 0)"
    echo "  spi-mode=<num>               SPI mode 1/2/3 (default: 2)"
    echo "  clockspeed=<freq_mhz>        Clock frequency in MHz"
    echo "                                 SPI default: 10MHz"
    echo "                                 SDIO default: As per Device Tree (25/50MHz)"
    echo ""
    echo "Build Configuration:"
    echo "  cross-compile=<path>         Cross-compilation toolchain path"
    echo "  kernel=<path>                Kernel build directory"
    echo "                                 Default: /lib/modules/\$(uname -r)/build"
    echo "  arch=<value>                 Architecture (arm/arm64)"
    echo "                                 Default: auto-detected"
    echo ""
    echo "Test & Performance:"
    echo "  rawtp                        Enable RAW throughput test mode"
    echo "  cpu-perf=<on/off>            Set CPU performance mode (default: on)"
    echo ""
    echo "Script Control:"
    echo "  dry-run                      Show what would be done without executing"
    echo "  skip-build-apps              Skip building user-space applications"
    echo "  skip-module-build            Skip building kernel module (use existing)"
    echo "  no-color                     Disable colored output"
    echo "  help, -h                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  # Basic SPI setup with Wi-Fi and Bluetooth"
    echo "  $0 wifi=spi bt=spi resetpin=6"
    echo ""
    echo "  # Using alternative argument names (all equivalent)"
    echo "  $0 wifi-transport=sdio bt-transport=sdio"
    echo "  $0 --wifi-transport=sdio --bt-transport=sdio"
    echo "  $0 wifi_transport=sdio bt_transport=sdio"
    echo ""
    echo "  # SDIO with custom clock speed"
    echo "  $0 wifi=sdio bt=sdio clockspeed=40"
    echo ""
    echo "  # Wi-Fi over SPI, Bluetooth over UART"
    echo "  $0 wifi=spi bt=uart-4pins resetpin=6"
    echo ""
    echo "  # Cross-compilation for different platform"
    echo "  $0 wifi=spi cross-compile=/opt/toolchain/bin/arm-linux-gnueabihf- \\"
    echo "     kernel=/lib/modules/5.15.0-161-generic/build/ arch=arm64"
    echo ""
    echo "  # Quick test without rebuilding (dashes preferred but underscores work too)"
    echo "  $0 wifi=spi skip-build-apps skip-module-build"
    echo "  $0 wifi=spi --skip-build-apps --skip-module-build"
    echo ""
    echo "  # Preview changes without executing"
    echo "  $0 wifi=spi bt=spi dry-run"
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

display_configuration_summary()
{
	echo ""
	echo "========================================"
	echo "Configuration Summary"
	echo "========================================"
	echo "Transport:"
	echo "  Interface Type       : ${IF_TYPE:-Not Set}"
	[ -n "$WIFI_TP" ] && echo "  Wi-Fi Transport      : $WIFI_TP"
	[ -n "$BT_TP" ] && echo "  Bluetooth Transport  : $BT_TP"
	[ -n "$BT_CONFIG" ] && echo "  BT Config Mode       : $BT_CONFIG"

	echo ""
	echo "Build Configuration:"
	echo "  Module Name          : $MODULE_NAME"
	echo "  Architecture         : ${ARCH:-auto-detect}"
	echo "  Kernel Build Dir     : $KERNEL_BUILD_DIR"
	echo "  Cross Compile        : ${CROSS_COMPILE:-Not Set}"
	echo "  RAW TP Test          : $([ "$TEST_RAW_TP" = "1" ] && echo "Enabled" || echo "Disabled")"

	# Display transport-specific configuration
	if [ "$IF_TYPE" = "spi" ]; then
		echo ""
		echo "SPI Configuration:"
		[ -n "$resetpin" ] && echo "  Reset Pin            : $resetpin"
		echo "  Clock Speed          : ${clockspeed:-10} MHz"
		echo "  SPI Bus              : ${spi_bus:-0}"
		echo "  SPI CS               : ${spi_cs:-0}"
		echo "  SPI Mode             : ${spi_mode:-2}"
		echo "  SPI Handshake GPIO   : ${spi_handshake}"
		echo "  SPI DataReady GPIO   : ${spi_dataready}"
	elif [ "$IF_TYPE" = "sdio" ]; then
		echo ""
		echo "SDIO Configuration:"
		[ -n "$resetpin" ] && echo "  Reset Pin            : $resetpin"
		echo "  Clock Speed          : ${clockspeed:-As per Device Tree} MHz"
	fi

	# Show UART config if bluetooth uses UART
	if [ "$BT_CONFIG" = "bt_using_uart_2pins" ] || [ "$BT_CONFIG" = "bt_using_uart_4pins" ]; then
		echo ""
		echo "UART Configuration:"
		echo "  BT UART Mode         : $BT_CONFIG"
	fi

	echo ""
	echo "Performance:"
	echo "  CPU Performance      : ${cpu_perf:-default}"

	echo ""
	echo "Script Options:"
	echo "  Dry Run              : $([ "$DRY_RUN" = "1" ] && echo "Yes (no changes will be made)" || echo "No")"
	echo "  Skip Build Apps      : $([ "$SKIP_BUILD_APPS" = "1" ] && echo "Yes" || echo "No")"
	echo "  Skip Module Build    : $([ "$SKIP_MODULE_BUILD" = "1" ] && echo "Yes" || echo "No")"
	echo "  Module Parameters    : ${XTRA_MODULE_PARAMS:-None}"
	echo "========================================"
	echo ""
}


######## Script ##########
SCRIPT_DIR=$PWD
MAKE_DIR=$SCRIPT_DIR/../host_driver/esp32/

cd $MAKE_DIR
parse_arguments "$@"

log "Building for $IF_TYPE protocol"
MODULE_NAME=esp32_${IF_TYPE}.ko

populate_module_params
display_configuration_summary

if [ "$DRY_RUN" = "1" ]; then
	warn "Dry-run mode: No actual changes will be made"
	info "Would execute the following steps:"
	info "  1. Set CPU performance mode: $cpu_perf"
	[ "$SKIP_BUILD_APPS" != "1" ] && info "  2. Build user-space applications (C & Python)"
	info "  3. Remove existing module (if loaded)"
	[ "$SKIP_MODULE_BUILD" != "1" ] && info "  4. Build kernel module: $MODULE_NAME"
	info "  5. Insert module with params: $XTRA_MODULE_PARAMS"
	info ""
	info "To execute for real, run without --dry-run"
	exit 0
fi

port_populate_local_params

if [ "$SKIP_BUILD_APPS" != "1" ]; then
	build_user_space_apps
else
	info "Skipping user-space app builds (--skip-build-apps)"
fi

remove_module

if [ "$SKIP_MODULE_BUILD" != "1" ]; then
	build_module
else
	info "Skipping module build (--skip-module-build)"
	if [ ! -f "$MODULE_NAME" ]; then
		error "Module file $MODULE_NAME not found!"
		error "Cannot skip build when module doesn't exist"
		exit 1
	fi
	info "Using existing module: $MODULE_NAME"
fi

insert_module

success "ESP-Hosted driver initialization completed successfully!"
log "--- finished ---"
