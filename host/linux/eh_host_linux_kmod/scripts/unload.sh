#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Remove esp32_{sdio,spi}.ko if loaded.

set -eu

if ! lsmod | grep -qE '^(esp32_sdio|esp32_spi) '; then
	echo "==> no esp32_* module loaded"
	exit 0
fi

if lsmod | grep -q '^esp32_sdio '; then
	echo "==> rmmod esp32_sdio"
	sudo rmmod esp32_sdio
fi
if lsmod | grep -q '^esp32_spi '; then
	echo "==> rmmod esp32_spi"
	sudo rmmod esp32_spi
fi

if lsmod | grep -qE '^(esp32_sdio|esp32_spi) '; then
	echo "Error: removal failed, module still present" >&2
	exit 1
fi
echo "==> esp32_* removed"
