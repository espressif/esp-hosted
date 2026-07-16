#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Show whether the esp32_{sdio,spi} kmod is loaded plus relevant kernel logs.

set -u

echo "==> lsmod"
if lsmod | grep -E '^(esp32_sdio|esp32_spi) ' ; then : ; else echo "  (no esp32_* module loaded)"; fi

echo ""
echo "==> last 40 esp/hci dmesg lines"
sudo dmesg 2>/dev/null | grep -iE 'esp|hci' | tail -n 40 || echo "  (none)"

echo ""
echo "==> netdevs"
ip -br link show ethsta0 2>/dev/null || echo "  ethsta0 not present"
ip -br link show ethap0  2>/dev/null || echo "  ethap0 not present"

if [ -e /dev/esps0 ]; then
	echo ""
	echo "==> /dev/esps0"
	ls -l /dev/esps0
fi
