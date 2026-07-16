#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Build esp32_{sdio,spi}.ko.

set -eu

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAKE_DIR="$SCRIPT_DIR/../driver"
LAST_CFG_FILE="$SCRIPT_DIR/.last_build.conf"

BUS=""
BT_BUS="off"
CONFIG_TEST_RAW_TP="${CONFIG_TEST_RAW_TP:-n}"
CONFIG_BT_ENABLED="n"
CROSS_COMPILE="${CROSS_COMPILE:-}"
KERNEL_BUILD_DIR="${KERNEL_BUILD_DIR:-/lib/modules/$(uname -r)/build}"
ARCH="${ARCH:-}"
DO_LOAD="0"
DO_RELOAD="0"
LOG_LEVEL=""
LOAD_PORT=""
LOAD_RESET_GPIO=""
LOAD_CLOCK_MHZ=""
LOAD_SPI_BUS=""
LOAD_SPI_CS=""
LOAD_SPI_MODE=""
LOAD_SPI_HANDSHAKE=""
LOAD_SPI_DATAREADY=""

save_last_build_config() {
	cat > "$LAST_CFG_FILE" <<CFG
BUS=$BUS
BT_BUS=$BT_BUS
CONFIG_BT_ENABLED=$CONFIG_BT_ENABLED
CFG
	echo "==> saved build config: $LAST_CFG_FILE"
}

usage() {
	cat <<EOF2
Usage:
  $0 --bus <spi|sdio> [options]

Required:
  --bus <spi|sdio>

Optional:
  --bt-bus <off|spi|sdio|uart2|uart4> (default: off)
  --log-level <info|debug|verbose>
  --rawtp
  --cross-compile <prefix>
  --kernel <path>
  --arch <arm|arm64>
  --port <name>           forwarded to load.sh with --load/--reload
  --reset-gpio <gpio>     forwarded to load.sh with --load/--reload
  --clock-mhz <mhz>       forwarded to load.sh with --load/--reload
  --spi-bus <n>           forwarded to load.sh with --load/--reload
  --spi-cs <n>            forwarded to load.sh with --load/--reload
  --spi-mode <1|2|3>      forwarded to load.sh with --load/--reload
  --spi-handshake <gpio>  forwarded to load.sh with --load/--reload
  --spi-dataready <gpio>  forwarded to load.sh with --load/--reload
  --load                 build then load module
  --reload               build then unload+load module

Examples:
  $0 --bus sdio
  $0 --bus sdio --bt-bus sdio
  $0 --bus sdio --bt-bus uart4
EOF2
}

while [ $# -gt 0 ]; do
	case "$1" in
		--bus)
			[ $# -ge 2 ] || { echo "Error: --bus requires a value" >&2; exit 1; }
			case "$2" in
				spi|sdio) BUS="$2" ;;
				*) echo "Error: unsupported --bus $2 (use spi or sdio)" >&2; exit 1 ;;
			esac
			shift 2
			;;
		--bt-bus)
			[ $# -ge 2 ] || { echo "Error: --bt-bus requires a value" >&2; exit 1; }
			case "$2" in
				off|spi|sdio|uart2|uart4) BT_BUS="$2" ;;
				*) echo "Error: unsupported --bt-bus $2" >&2; exit 1 ;;
			esac
			shift 2
			;;
		--rawtp)
			CONFIG_TEST_RAW_TP="y"
			shift
			;;
		--log-level)
			[ $# -ge 2 ] || { echo "Error: --log-level requires a value" >&2; exit 1; }
			case "$2" in
				info|debug|verbose) LOG_LEVEL="$2" ;;
				*) echo "Error: --log-level must be one of: info, debug, verbose" >&2; exit 1 ;;
			esac
			shift 2
			;;
		--cross-compile)
			[ $# -ge 2 ] || { echo "Error: --cross-compile requires a value" >&2; exit 1; }
			CROSS_COMPILE="$2"
			shift 2
			;;
		--kernel)
			[ $# -ge 2 ] || { echo "Error: --kernel requires a value" >&2; exit 1; }
			KERNEL_BUILD_DIR="$2"
			shift 2
			;;
		--arch)
			[ $# -ge 2 ] || { echo "Error: --arch requires a value" >&2; exit 1; }
			ARCH="$2"
			shift 2
			;;
		--port)
			[ $# -ge 2 ] || { echo "Error: --port requires a value" >&2; exit 1; }
			LOAD_PORT="$2"
			shift 2
			;;
		--reset-gpio)
			[ $# -ge 2 ] || { echo "Error: --reset-gpio requires a value" >&2; exit 1; }
			LOAD_RESET_GPIO="$2"
			shift 2
			;;
		--clock-mhz)
			[ $# -ge 2 ] || { echo "Error: --clock-mhz requires a value" >&2; exit 1; }
			LOAD_CLOCK_MHZ="$2"
			shift 2
			;;
		--spi-bus)
			[ $# -ge 2 ] || { echo "Error: --spi-bus requires a value" >&2; exit 1; }
			LOAD_SPI_BUS="$2"
			shift 2
			;;
		--spi-cs)
			[ $# -ge 2 ] || { echo "Error: --spi-cs requires a value" >&2; exit 1; }
			LOAD_SPI_CS="$2"
			shift 2
			;;
		--spi-mode)
			[ $# -ge 2 ] || { echo "Error: --spi-mode requires a value" >&2; exit 1; }
			LOAD_SPI_MODE="$2"
			shift 2
			;;
		--spi-handshake)
			[ $# -ge 2 ] || { echo "Error: --spi-handshake requires a value" >&2; exit 1; }
			LOAD_SPI_HANDSHAKE="$2"
			shift 2
			;;
		--spi-dataready)
			[ $# -ge 2 ] || { echo "Error: --spi-dataready requires a value" >&2; exit 1; }
			LOAD_SPI_DATAREADY="$2"
			shift 2
			;;
		--help|-h)
			usage
			exit 0
			;;
		--load)
			DO_LOAD="1"
			shift
			;;
		--reload)
			DO_RELOAD="1"
			shift
			;;
		*)
			echo "Error: unknown option '$1'" >&2
			usage >&2
			exit 1
			;;
	esac
done

if [ -z "$BUS" ]; then
	echo "Error: --bus is required" >&2
	usage >&2
	exit 1
fi
if [ "$DO_LOAD" = "1" ] && [ "$DO_RELOAD" = "1" ]; then
	echo "Error: use only one of --load or --reload" >&2
	exit 1
fi

if [ "$BT_BUS" != "off" ]; then
	CONFIG_BT_ENABLED="y"
fi

if [ "$BT_BUS" = "spi" ] || [ "$BT_BUS" = "sdio" ]; then
	if [ "$BT_BUS" != "$BUS" ]; then
		echo "Error: --bt-bus $BT_BUS must match --bus $BUS for shared-bus HCI" >&2
		exit 1
	fi
fi

if [ -z "$ARCH" ]; then
	if [ "$(getconf LONG_BIT)" = "32" ]; then ARCH="arm"; else ARCH="arm64"; fi
fi

echo "==> resolved config: bus=$BUS bt_bus=$BT_BUS bt_enabled=$CONFIG_BT_ENABLED"
echo "==> building esp32_${BUS}.ko  (arch=$ARCH  kernel=$KERNEL_BUILD_DIR)"

CFG_INFO_LOGS=""
CFG_DEBUG_LOGS=""
CFG_VERBOSE_LOGS=""
case "$LOG_LEVEL" in
	info)
		CFG_INFO_LOGS="y"
		CFG_DEBUG_LOGS="n"
		CFG_VERBOSE_LOGS="n"
		;;
	debug)
		CFG_INFO_LOGS="y"
		CFG_DEBUG_LOGS="y"
		CFG_VERBOSE_LOGS="n"
		;;
	verbose)
		CFG_INFO_LOGS="y"
		CFG_DEBUG_LOGS="y"
		CFG_VERBOSE_LOGS="y"
		;;
esac

cd "$MAKE_DIR"
make -j8 target="$BUS" KERNEL="$KERNEL_BUILD_DIR" ARCH="$ARCH" \
	CROSS_COMPILE="$CROSS_COMPILE" \
	CONFIG_TEST_RAW_TP="$CONFIG_TEST_RAW_TP" \
	CONFIG_BT_ENABLED="$CONFIG_BT_ENABLED" \
	${CFG_INFO_LOGS:+CONFIG_INFO_LOGS="$CFG_INFO_LOGS"} \
	${CFG_DEBUG_LOGS:+CONFIG_DEBUG_LOGS="$CFG_DEBUG_LOGS"} \
	${CFG_VERBOSE_LOGS:+CONFIG_VERBOSE_LOGS="$CFG_VERBOSE_LOGS"}
echo "==> built esp32_${BUS}.ko"
save_last_build_config

LOAD_ARGS=(--bus "$BUS" --bt-bus "$BT_BUS")
[ -n "$LOAD_RESET_GPIO" ] && LOAD_ARGS+=(--reset-gpio "$LOAD_RESET_GPIO")
[ -n "$LOAD_CLOCK_MHZ" ] && LOAD_ARGS+=(--clock-mhz "$LOAD_CLOCK_MHZ")
[ -n "$LOAD_SPI_BUS" ] && LOAD_ARGS+=(--spi-bus "$LOAD_SPI_BUS")
[ -n "$LOAD_SPI_CS" ] && LOAD_ARGS+=(--spi-cs "$LOAD_SPI_CS")
[ -n "$LOAD_SPI_MODE" ] && LOAD_ARGS+=(--spi-mode "$LOAD_SPI_MODE")
[ -n "$LOAD_SPI_HANDSHAKE" ] && LOAD_ARGS+=(--spi-handshake "$LOAD_SPI_HANDSHAKE")
[ -n "$LOAD_SPI_DATAREADY" ] && LOAD_ARGS+=(--spi-dataready "$LOAD_SPI_DATAREADY")

if [ "$DO_RELOAD" = "1" ]; then
	echo "==> reloading module"
	"$SCRIPT_DIR/unload.sh"
	if [ -n "$LOAD_PORT" ]; then
		PORT="$LOAD_PORT" "$SCRIPT_DIR/load.sh" "${LOAD_ARGS[@]}"
	else
		"$SCRIPT_DIR/load.sh" "${LOAD_ARGS[@]}"
	fi
elif [ "$DO_LOAD" = "1" ]; then
	echo "==> loading module"
	if [ -n "$LOAD_PORT" ]; then
		PORT="$LOAD_PORT" "$SCRIPT_DIR/load.sh" "${LOAD_ARGS[@]}"
	else
		"$SCRIPT_DIR/load.sh" "${LOAD_ARGS[@]}"
	fi
fi
