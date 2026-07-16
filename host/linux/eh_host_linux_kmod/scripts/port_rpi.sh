#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Port layer: Raspberry Pi (5 / CM5 / 4).
#
# load.sh sources scripts/port_${PORT}.sh (default PORT=rpi). Port files
# carry everything board-specific: default GPIO numbers, device-tree overlay
# tooling, UART pin-muxing. To support a non-RPi board, copy this file to
# port_<board>.sh, adjust the functions below, and run:
#
#     PORT=<board> ./load.sh wifi=spi ...
#
# Every port_*.sh MUST define these functions (no-op is fine):
#   port_defaults                  - seed module param defaults
#   port_dt_overlay_spi            - apply any DT overlay needed before insmod
#   port_pinmux_bt_uart_2pin       - mux UART TX/RX for BT HCI over UART
#   port_pinmux_bt_uart_4pin       - mux UART TX/RX/RTS/CTS for BT HCI over UART
#   port_cpu_perf <on|off>         - CPU governor knob (no-op if not applicable)

# RPi 5 / CM5 GPIO numbering: gpiochip starts at 512, so userspace SysFS
# numbers are 512+<header-pin-gpio>. resetpin 518 == GPIO 6, handshake 534
# == GPIO 22, dataready 539 == GPIO 27. For RPi 4 / older, drop the 512
# offset (resetpin=6, handshake=22, dataready=27).
_port_gpio_base() {
	local chip label base

	# Preferred method: find the user-facing header controller by label, then
	# read its kernel GPIO base from sysfs.
	# Typical labels:
	#  - pinctrl-rp1      (RPi 5 user-facing header controller)
	#  - pinctrl-bcm2711  (RPi 4)
	#  - pinctrl-bcm2835/2837/... (older models)
	for chip in /sys/class/gpio/gpiochip*; do
		[ -r "$chip/label" ] || continue
		[ -r "$chip/base" ] || continue
		label="$(cat "$chip/label" 2>/dev/null || true)"
		case "$label" in
			pinctrl-rp1|pinctrl-bcm*|*pinctrl-bcm*)
				base="$(cat "$chip/base" 2>/dev/null || true)"
				if [ -n "$base" ]; then
					echo "$base"
					return
				fi
				;;
		esac
	done

	# Fallback: parse debugfs gpiochip entries for pinctrl-backed controller.
	if [ -r /sys/kernel/debug/gpio ]; then
		base="$(awk '/gpiochip[0-9]+/ && /pinctrl/ {sub("gpiochip","",$1); sub(":","",$1); print $1; exit}' /sys/kernel/debug/gpio 2>/dev/null || true)"
		if [ -n "$base" ]; then
			echo "$base"
			return
		fi
	fi

	# Last resort: default to RPi 5 style and print a warning so users can override.
	echo "port_rpi: warning: unable to resolve header gpiochip base; defaulting to 512 (override with --reset-gpio)" >&2
	echo "512"
}

port_defaults() {
	local base reset_default hs_default dr_default
	base="$(_port_gpio_base)"

	# Newer stacks can expose GPIOs as 512+offset; classic numbering starts at 0.
	if [ "$base" -ge 512 ] 2>/dev/null; then
		reset_default=$((base + 6))
		hs_default=$((base + 22))
		dr_default=$((base + 27))
	else
		reset_default=6
		hs_default=22
		dr_default=27
	fi

	: "${resetpin:=$reset_default}"
	: "${clockspeed:=10}"
	: "${spi_bus:=0}"
	: "${spi_cs:=0}"
	: "${spi_mode:=3}"
	: "${spi_handshake:=$hs_default}"
	: "${spi_dataready:=$dr_default}"
}

# Disable spidev on the chosen SPI bus so esp_spi can claim the CS line.
# Requires the dtc + dtoverlay tools shipped with Raspberry Pi OS.
port_dt_overlay_spi() {
	local dts="$1"   # absolute path to spidev_disabler.dts
	local dt_dir
	dt_dir="$(dirname -- "$dts")"

	if [ ! -f "$dts" ]; then
		echo "port_rpi: $dts not present, skipping SPI overlay" >&2
		return 0
	fi
	rm -f "$dt_dir/spidev_disabler.dtbo"
	dtc "$dts" -O dtb -o "$dt_dir/spidev_disabler.dtbo"
	sudo dtoverlay -d "$dt_dir" spidev_disabler
}

# UART mux helpers. RPi exposes `pinctrl` on newer images, `raspi-gpio` on
# older ones. Pick whichever is present.
_port_pinctl() {
	if command -v pinctrl >/dev/null 2>&1; then
		echo pinctrl
	else
		echo raspi-gpio
	fi
}

port_pinmux_bt_uart_2pin() {
	local p
	p="$(_port_pinctl)"
	sudo "$p" set 15 a0 pu   # RXD
	sudo "$p" set 14 a0 pu   # TXD
}

port_pinmux_bt_uart_4pin() {
	local p
	p="$(_port_pinctl)"
	sudo "$p" set 15 a0 pu   # RXD
	sudo "$p" set 14 a0 pu   # TXD
	sudo "$p" set 16 a3 pu   # CTS
	sudo "$p" set 17 a3 pu   # RTS
}

# Switch CPU governor. "on" -> performance, "off" -> ondemand (RPi default).
port_cpu_perf() {
	local mode="${1:-on}"
	local gov
	case "$mode" in
		on)  gov=performance ;;
		off) gov=ondemand ;;
		*) echo "port_cpu_perf: unknown mode '$mode'" >&2; return 1 ;;
	esac
	for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
		[ -w "$f" ] || continue
		echo "$gov" | sudo tee "$f" >/dev/null
	done
}
