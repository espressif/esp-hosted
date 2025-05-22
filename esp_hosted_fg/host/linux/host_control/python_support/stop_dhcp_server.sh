#!/usr/bin/env bash

# This script stops the custom dnsmasq instance started by run_dhcp_server.sh

echo "[stop_dhcp_server.sh] Stopping DHCP server..."

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "[stop_dhcp_server.sh] This script must be run as root. Exiting."
  exit 1
fi

echo "[stop_dhcp_server.sh] Looking for custom dnsmasq instance(s) running on port 40000..."
PIDS=$(ps -eaf | grep -i "dnsmasq" | grep -- "--port=40000" | grep -v grep | awk '{print $2}')
if [ -n "$PIDS" ]; then
  echo "[stop_dhcp_server.sh] > Stopping custom dnsmasq instance(s) with PID(s): $PIDS"
  sudo kill $PIDS
  echo "[stop_dhcp_server.sh] Custom dnsmasq instance(s) stopped."
else
  echo "[stop_dhcp_server.sh] No custom dnsmasq instance running"
fi

echo "[stop_dhcp_server.sh] DHCP server stop complete."
exit 0
