#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Load esp32_{sdio,spi}.ko. Board-specific bits live in scripts/port_${PORT}.sh
# (default PORT=rpi).

set -euE
trap 'rc=$?; echo "Error: load.sh failed at line $LINENO: $BASH_COMMAND (exit=$rc)" >&2' ERR

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DRIVER_DIR="$SCRIPT_DIR/../driver"
DT_DIR="$SCRIPT_DIR/../device_tree"
LAST_CFG_FILE="$SCRIPT_DIR/.last_build.conf"
PORT="${PORT:-rpi}"

BUS=""
BT_BUS="off"
reset_gpio=""
clock_mhz=""
spi_bus=""
spi_cs=""
spi_mode=""
spi_handshake=""
spi_dataready=""

load_last_build_config() {
	local cli_bus="$BUS"
	local cli_bt_bus="$BT_BUS"
	if [ ! -f "$LAST_CFG_FILE" ]; then
		echo "==> no saved config found at $LAST_CFG_FILE"
		return
	fi
	# shellcheck disable=SC1090
	. "$LAST_CFG_FILE"
	if [ -n "$cli_bus" ]; then BUS="$cli_bus"; fi
	if [ "$cli_bt_bus" != "off" ]; then BT_BUS="$cli_bt_bus"; fi
	echo "==> loaded saved config: bus=${BUS:-<unset>} bt_bus=${BT_BUS:-off}"
}

usage() {
	cat <<EOF2
Usage:
  PORT=<port> $0 --bus <spi|sdio> [options]

Required:
  --bus <spi|sdio>

Optional:
  --bt-bus <off|spi|sdio|uart2|uart4> (default: off)
  --reset-gpio <gpio>
  --clock-mhz <mhz>
  --spi-bus <n> --spi-cs <n> --spi-mode <1|2|3>
  --spi-handshake <gpio> --spi-dataready <gpio>

Examples:
  $0 --bus sdio
  $0 --bus sdio --bt-bus sdio
  $0 --bus sdio --bt-bus uart4 --reset-gpio 518 --clock-mhz 10
  $0 --bus spi --bt-bus off --spi-bus 0 --spi-cs 0 --spi-mode 3

Notes:
  - Control path is assumed on the same bus as --bus.
  - Port defaults to 'rpi' (scripts/port_rpi.sh).
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
		--reset-gpio)
			[ $# -ge 2 ] || { echo "Error: --reset-gpio requires a value" >&2; exit 1; }
			reset_gpio="$2"
			shift 2
			;;
		--clock-mhz)
			[ $# -ge 2 ] || { echo "Error: --clock-mhz requires a value" >&2; exit 1; }
			clock_mhz="$2"
			shift 2
			;;
		--spi-bus)
			[ $# -ge 2 ] || { echo "Error: --spi-bus requires a value" >&2; exit 1; }
			spi_bus="$2"
			shift 2
			;;
		--spi-cs)
			[ $# -ge 2 ] || { echo "Error: --spi-cs requires a value" >&2; exit 1; }
			spi_cs="$2"
			shift 2
			;;
		--spi-mode)
			[ $# -ge 2 ] || { echo "Error: --spi-mode requires a value" >&2; exit 1; }
			spi_mode="$2"
			shift 2
			;;
		--spi-handshake)
			[ $# -ge 2 ] || { echo "Error: --spi-handshake requires a value" >&2; exit 1; }
			spi_handshake="$2"
			shift 2
			;;
		--spi-dataready)
			[ $# -ge 2 ] || { echo "Error: --spi-dataready requires a value" >&2; exit 1; }
			spi_dataready="$2"
			shift 2
			;;
		--help|-h)
			usage
			exit 0
			;;
		*)
			echo "Error: unknown option '$1'" >&2
			usage >&2
			exit 1
			;;
	esac
done

if [ -z "$BUS" ]; then
	load_last_build_config
fi
if [ -z "$BUS" ]; then
	echo "Error: --bus is required (or save config via ./build.sh first)" >&2
	usage >&2
	exit 1
fi

if [ "$BT_BUS" = "spi" ] || [ "$BT_BUS" = "sdio" ]; then
	if [ "$BT_BUS" != "$BUS" ]; then
		echo "Error: --bt-bus $BT_BUS must match --bus $BUS for shared-bus HCI" >&2
		exit 1
	fi
fi

PORT_FILE="$SCRIPT_DIR/port_${PORT}.sh"
if [ ! -f "$PORT_FILE" ]; then
	echo "Error: port file $PORT_FILE not found" >&2
	echo "  Available ports: $(ls "$SCRIPT_DIR"/port_*.sh 2>/dev/null | xargs -n1 basename | sed 's/port_//; s/\.sh$//' | tr '\n' ' ')" >&2
	exit 1
fi
# shellcheck source=port_rpi.sh
source "$PORT_FILE"

# Seed defaults from the port (won't overwrite user-provided values).
port_defaults
# Port files still seed legacy variable names; map them if new flags were omitted.
: "${reset_gpio:=${resetpin:-}}"
: "${clock_mhz:=${clockspeed:-}}"

# Clock-speed sanity.
if [ "$BUS" = "spi" ] && [ "${clock_mhz:-0}" -gt 40 ]; then
	echo "Error: SPI clock $clock_mhz MHz exceeds the 40 MHz cap" >&2
	exit 1
fi
if [ "$BUS" = "sdio" ] && [ "${clock_mhz:-0}" -gt 50 ]; then
	echo "Error: SDIO clock $clock_mhz MHz exceeds the 50 MHz cap" >&2
	exit 1
fi

MODULE_NAME="esp32_${BUS}"
MODULE_FILE="$DRIVER_DIR/${MODULE_NAME}.ko"
if [ ! -f "$MODULE_FILE" ]; then
	echo "Error: $MODULE_FILE not found. Run ./build.sh --bus $BUS first." >&2
	exit 1
fi

MODULE_PARAMS=""
add_module_param() {
	local key="$1"
	local val="$2"
	if [ -n "$val" ]; then
		MODULE_PARAMS="$MODULE_PARAMS $key=$val"
	fi
}
add_module_param resetpin "$reset_gpio"
add_module_param clockspeed "$clock_mhz"
if [ "$BUS" = "spi" ]; then
	add_module_param spi_bus "$spi_bus"
	add_module_param spi_cs "$spi_cs"
	add_module_param spi_mode "$spi_mode"
	add_module_param spi_handshake "$spi_handshake"
	add_module_param spi_dataready "$spi_dataready"
fi

if [ "$BUS" = "spi" ]; then
	echo "==> applying SPI device-tree overlay"
	port_dt_overlay_spi "$DT_DIR/spidev_disabler.dts"
fi

if [ "$BT_BUS" != "off" ]; then
	echo "==> enabling bluetooth path: bt_bus=$BT_BUS"
	sudo modprobe bluetooth
	sudo modprobe cfg80211 2>/dev/null || true
	case "$BT_BUS" in
		uart2) echo "==> muxing UART pins (2-wire)"; port_pinmux_bt_uart_2pin ;;
		uart4) echo "==> muxing UART pins (4-wire)"; port_pinmux_bt_uart_4pin ;;
	esac
fi

echo "==> resolved config: bus=$BUS ctrl_bus=$BUS bt_bus=$BT_BUS"
echo "==> inserting $MODULE_NAME$MODULE_PARAMS"
if ! sudo insmod "$MODULE_FILE" $MODULE_PARAMS; then
	echo "Error: insmod failed for $MODULE_NAME. Check: dmesg | tail -n 80" >&2
	if [ "$BT_BUS" != "off" ]; then
		echo "Hint: unresolved hci_* symbols usually mean kernel BT module/version mismatch." >&2
	fi
	exit 1
fi

if ! lsmod | grep -q "^$MODULE_NAME"; then
	echo "Error: $MODULE_NAME insertion reported success but module not loaded; check dmesg" >&2
	exit 1
fi
echo "==> $MODULE_NAME loaded"
