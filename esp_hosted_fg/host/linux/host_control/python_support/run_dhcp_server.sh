#!/usr/bin/env bash

# This script kills the default dnsmasq service and runs dnsmasq in a specific way.
# This is just a helper script and specific to dnsmasq.
# If your platform doesn't work with dnsmasq, or you use some other DHCP server
# software, you can skip running this script.

echo "[run_dhcp_server.sh] Starting DHCP server setup..."

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "[run_dhcp_server.sh] This script must be run as root. Exiting."
    exit 1
fi

# Install dnsmasq for DHCP server.
# Note that a DHCP server is only needed for softAP. Stations connecting to ESP softAP might need dynamic IP addresses.
# If you do not need DHCP, static IP addresses can potentially work and a DHCP server would not be needed in that case.
# If your environment doesn't allow dnsmasq, please use alternative software.
# Any dnsmasq or DHCP client-server software is out-of-scope for ESP-Hosted but is showcased anyway for user ease.
install_dnsmasq()
{
    echo "[run_dhcp_server.sh] Checking if dnsmasq is installed..."
    which dnsmasq &>/dev/null
    if [ "$?" != "0" ] ; then
        echo "[run_dhcp_server.sh] > Installing dnsmasq"
        sudo apt install dnsmasq &> /dev/null
    else
        echo "[run_dhcp_server.sh] dnsmasq already installed"
    fi

    which dnsmasq &>/dev/null
    if [ "$?" != "0" ] ; then
        echo "[run_dhcp_server.sh] ERROR: Failed to install dnsmasq, exiting"
        exit 1
    else
        echo "[run_dhcp_server.sh] dnsmasq installation verified"
    fi
}

is_dnsmasq_running_in_custom_way()
{
    echo "[run_dhcp_server.sh] Checking if dnsmasq is running in custom way (port 55000)..."
    if ps -eaf | grep -i "dnsmasq" | grep -- "--port=55000" &>/dev/null ; then
        echo "[run_dhcp_server.sh] dnsmasq is running in custom way"
        return 0
    else
        echo "[run_dhcp_server.sh] dnsmasq is NOT running in custom way"
        return 1
    fi
}

run_dnsmasq_in_custom_way()
{
    echo "[run_dhcp_server.sh] > Killing existing dnsmasq instance"
    if systemctl list-unit-files | grep -q '^dnsmasq\.service'; then
        echo "[run_dhcp_server.sh] Disabling dnsmasq systemd service"
        sudo systemctl disable dnsmasq
    fi
    sudo killall dnsmasq
    echo "[run_dhcp_server.sh] > Running dnsmasq in custom way"
    nohup sudo dnsmasq --port=55000 --no-daemon --no-resolv --no-poll --dhcp-script=/system/bin/dhcp_announce --dhcp-range=192.168.4.1,192.168.4.20,1h &> /dev/null &
    echo "[run_dhcp_server.sh] dnsmasq started in background with custom options"
}

install_dnsmasq

if ! is_dnsmasq_running_in_custom_way ; then
    echo "[run_dhcp_server.sh] Attempting to start dnsmasq in custom way..."
    run_dnsmasq_in_custom_way
fi

if ! is_dnsmasq_running_in_custom_way ; then
    echo "[run_dhcp_server.sh] Failed to run dnsmasq"
    exit 1
fi

echo "[run_dhcp_server.sh] DHCP server setup complete."
exit 0
