#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Clean kmod build artifacts.

set -eu

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAKE_DIR="$SCRIPT_DIR/../driver"
LAST_CFG_FILE="$SCRIPT_DIR/.last_build.conf"
KERNEL_BUILD_DIR="${KERNEL_BUILD_DIR:-/lib/modules/$(uname -r)/build}"
ARCH="${ARCH:-}"

remove_last_build_config() {
	rm -f "$LAST_CFG_FILE"
}

usage() {
	cat <<EOF
Usage: $0 [kernel=<path>] [arch=<arm|arm64>]
EOF
}

while [ $# -gt 0 ]; do
	arg="${1#--}"
	key="${arg%%=*}"
	key="${key//_/-}"
	val="${arg#*=}"
	case "$key" in
		kernel)  KERNEL_BUILD_DIR="$val" ;;
		arch)    ARCH="$val" ;;
		help|-h) usage; exit 0 ;;
		*) echo "Error: unknown option '$1'" >&2; usage >&2; exit 1 ;;
	esac
	shift
done

if [ -z "$ARCH" ]; then
	if [ "$(getconf LONG_BIT)" = "32" ]; then ARCH="arm"; else ARCH="arm64"; fi
fi

echo "==> make clean  (arch=$ARCH)"
cd "$MAKE_DIR"
make ARCH="$ARCH" KERNEL="$KERNEL_BUILD_DIR" clean
remove_last_build_config
echo "==> clean done"
