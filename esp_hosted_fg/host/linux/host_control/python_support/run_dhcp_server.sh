#!/usr/bin/env bash

# This script kills the default dnsmasq service and runs dnsmasq in a specific way.
# This is just a helper script and specific to dnsmasq.
# If your platform doesn't work with dnsmasq, or you use some other DHCP server
# software, you can skip running this script.

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "This script must be run as root. Exiting."
  exit 1
fi

# Install dnsmasq for DHCP server.
# Note that a DHCP server is only needed for softAP. Stations connecting to ESP softAP might need dynamic IP addresses.
# If you do not need DHCP, static IP addresses can potentially work and a DHCP server would not be needed in that case.
# If your environment doesn't allow dnsmasq, please use alternative software.
# Any dnsmasq or DHCP client-server software is out-of-scope for ESP-Hosted but is showcased anyway for user ease.
install_dnsmasq()
{
    # Install if not installed
    which dnsmasq &>/dev/null
    if [ "$?" != "0" ] ; then
        echo "> Installing dnsmasq"
        sudo apt install dnsmasq &> /dev/null
    fi

    # Verify installation
    which dnsmasq &>/dev/null
    if [ "$?" != "0" ] ; then
        echo "ERROR: Failed to install dnsmasq, exiting"
        exit 1
    fi
}

is_dnsmasq_running_in_custom_way()
{
    if ps -eaf | grep -i "dnsmasq" | grep -- "--port=40000" &>/dev/null ; then
        return 0  # Running in custom way
    else
        return 1  # Not running in custom way
    fi
}

run_dnsmasq_in_custom_way()
{
    echo "> Killing existing dnsmasq instance"
    # Kill default dnsmasq
    sudo systemctl disable dnsmasq
    sudo killall dnsmasq

    echo "> Running dnsmasq in custom way"
    # Run manually in the background
    nohup sudo dnsmasq --port=40000 --no-daemon --no-resolv --no-poll --dhcp-script=/system/bin/dhcp_announce --dhcp-range=192.168.4.1,192.168.4.20,1h &> /dev/null &
}

install_dnsmasq

if ! is_dnsmasq_running_in_custom_way ; then
    run_dnsmasq_in_custom_way
fi

if ! is_dnsmasq_running_in_custom_way ; then
    echo "Failed to run dnsmasq"
    exit 1
fi

exit 0
