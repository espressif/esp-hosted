import os
import socket
import re
from hosted_py_header import *

STA_INTERFACE = "ethsta0"
AP_INTERFACE = "ethap0"
MAC_ADDR_LENGTH = 18
SUCCESS = 0
FAILURE = -1

MAC_SIZE_BYTES = 6
MIN_MAC_STR_LEN = 17

# Utility functions for MAC address handling
nw_debug_logs = 0

def mac_bytes_to_str(mac_bytes):
    if isinstance(mac_bytes, str):
        return mac_bytes
    if hasattr(mac_bytes, 'value'):
        mac_bytes = mac_bytes.value
    mac_bytes = bytes(mac_bytes).rstrip(b'\x00')
    if len(mac_bytes) == 6:
        return ':'.join(f'{b:02x}' for b in mac_bytes)
    elif len(mac_bytes) == 17:
        return mac_bytes.decode('utf-8', errors='ignore').rstrip('\x00')
    else:
        return mac_bytes.hex(':')

def mac_str_to_bytes(mac_str):
    print("mac_str" + mac_str)
    if isinstance(mac_str, bytes):
        return mac_str
    mac_str = mac_str.replace('-', ':').lower()
    if ':' in mac_str:
        return bytes(int(b, 16) for b in mac_str.split(':'))
    else:
        return bytes.fromhex(mac_str)

def is_valid_mac_bytes(mac_bytes):
    if isinstance(mac_bytes, str):
        mac_bytes = mac_bytes.encode()
    mac_bytes = bytes(mac_bytes).rstrip(b'\x00')
    # Accept 6-byte binary or 17-byte ASCII MAC
    if len(mac_bytes) == 6 and any(mac_bytes):
        return True
    if len(mac_bytes) == 17:
        try:
            # Try to parse as ASCII MAC
            s = mac_bytes.decode('utf-8')
            parts = s.split(':')
            if len(parts) == 6 and all(len(p) == 2 for p in parts):
                return True
        except Exception:
            return False
    return False

# SoftAP commands
# os_set_mac = "sudo ifconfig ethsta0 hw ether "
# os_ifup_cmd = "sudo ifconfig ethsta0 up"
# os_dhcp_up = "sudo dhclient ethsta0 -v"

#os_run_dhcp_server='sudo bash ./run_dhcp_server.sh'
#os_ifup_softap_cmd = "sudo ifconfig ethap0 up 192.168.4.5"
#os_ifdown_softap_cmd = "sudo ifconfig ethap0 down"

os_softap_static_ip = "192.168.4.5"
os_softap_netmask = "255.255.255.0"
os_softap_gateway = "192.168.4.1"

g_run_dhcp_on_station_connected = True
g_stop_dhclient_on_disconnected = True
g_run_dhcp_server_after_softap_up = True
g_stop_dhcp_server_after_softap_down = True

# HCI commands
down_hci_instance_cmd = "sudo hciconfig | grep  'Bus: SDIO\\| Bus: UART\\| Bus: SPI' | awk -F: '{print $1}' | xargs -I{} sudo hciconfig {} down"
reset_hci_instance_cmd = "sudo hciconfig | grep  'Bus: SDIO\\| Bus: UART\\| Bus: SPI' | awk -F: '{print $1}' | xargs -I{} sudo hciconfig {} reset"


class NetworkInfo:
    def __init__(self, mac_addr=b"", ip_addr="", netmask="", gateway="", dns_addr="", ip_valid=0, dns_valid=0, network_up=0):
        self.mac_addr = mac_addr  # Always store as bytes
        self.ip_addr = ip_addr
        self.netmask = netmask
        self.gateway = gateway
        self.dns_addr = dns_addr
        self.ip_valid = ip_valid
        self.dns_valid = dns_valid
        self.network_up = network_up

g_sta_network_info = NetworkInfo()
g_ap_network_info = NetworkInfo()

def convert_mac_to_bytes(mac_str):
    return mac_str_to_bytes(mac_str)

def set_hw_addr(iface, mac):
    mac_bytes = mac_str_to_bytes(mac) if isinstance(mac, str) else bytes(mac)
    if not is_valid_mac_bytes(mac_bytes):
        return FAILURE
    if iface == STA_INTERFACE:
        g_sta_network_info.mac_addr = mac_bytes
    elif iface == AP_INTERFACE:
        g_ap_network_info.mac_addr = mac_bytes

    # Use ip link set dev <iface> address <mac> (as string)
    mac_str = mac_bytes_to_str(mac_bytes)
    ret = os.system(f"ip link set dev {iface} address {mac_str}")
    return SUCCESS if ret == 0 else FAILURE

def interface_up(iface):
    if iface == STA_INTERFACE:
        g_sta_network_info.network_up = 1
    elif iface == AP_INTERFACE:
        g_ap_network_info.network_up = 1

    # Up interface
    ret = os.system(f"ip link set dev {iface} up")
    return SUCCESS if ret == 0 else FAILURE

def interface_down(iface):
    if iface == STA_INTERFACE:
        g_sta_network_info.network_up = 0
    elif iface == AP_INTERFACE:
        g_ap_network_info.network_up = 0

    # Down interface
    ret = os.system(f"ip link set dev {iface} down")
    return SUCCESS if ret == 0 else FAILURE

def set_network_static_ip(iface, ip, netmask, gateway):
    if iface == STA_INTERFACE:
        g_sta_network_info.ip_addr = ip
        g_sta_network_info.netmask = netmask
        g_sta_network_info.gateway = gateway
    elif iface == AP_INTERFACE:
        g_ap_network_info.ip_addr = ip
        g_ap_network_info.netmask = netmask
        g_ap_network_info.gateway = gateway

    # Set IP and netmask
    ret1 = os.system(f"ip addr flush dev {iface} > /dev/null 2>&1")
    ret2 = os.system(f"ip addr add {ip}/{netmask} dev {iface} > /dev/null 2>&1")

    # Set gateway
    ret3 = os.system(f"ip route add default via {gateway} dev {iface} > /dev/null 2>&1")

    if ret1:
        print("Failed to execute: " + f"ip addr flush dev {iface}")
    elif ret2:
        print("Failed to execute: " + f"ip addr add {ip}/{netmask} dev {iface}")
    elif ret3:
        print("Failed to execute: " + f"ip route add default via {gateway} dev {iface}")
    else:
        pass

    if ret1 == 0 and ret2 == 0:
        return SUCCESS
    return FAILURE

def add_default_gateway(iface):
    if iface == STA_INTERFACE:
        gateway = g_sta_network_info.gateway
    elif iface == AP_INTERFACE:
        gateway = g_ap_network_info.gateway
    if not gateway:
        return FAILURE
    ret = os.system(f"ip route add default via {gateway} > /dev/null 2>&1")
    return SUCCESS if ret == 0 else FAILURE

def remove_default_gateway(iface):
    if iface == STA_INTERFACE:
        gateway = g_sta_network_info.gateway
    elif iface == AP_INTERFACE:
        gateway = g_ap_network_info.gateway
    if not gateway:
        return FAILURE
    ret = os.system(f"ip route del default via {gateway} > /dev/null 2>&1")
    return SUCCESS if ret == 0 else FAILURE

def add_dns(iface):
    if iface == STA_INTERFACE:
        dns = g_sta_network_info.dns_addr
    elif iface == AP_INTERFACE:
        dns = g_ap_network_info.dns_addr
    if not dns:
        return FAILURE
    try:
        with open("/etc/resolv.conf", "a") as f:
            f.write(f"nameserver {dns}\n")
        return SUCCESS
    except Exception:
        return FAILURE

def remove_dns(iface):
    if iface == STA_INTERFACE:
        dns = g_sta_network_info.dns_addr
    elif iface == AP_INTERFACE:
        dns = g_ap_network_info.dns_addr
    if not dns:
        return FAILURE
    try:
        with open("/etc/resolv.conf", "r") as f:
            lines = f.readlines()
        with open("/etc/resolv.conf", "w") as f:
            for line in lines:
                if line.strip() != f"nameserver {dns}":
                    f.write(line)
        return SUCCESS
    except Exception:
        return FAILURE

def update_host_network_port_range(port_start, port_end):
    try:
        found = False
        lines = []
        with open("/etc/sysctl.conf", "r") as f:
            for line in f:
                if "net.ipv4.ip_local_port_range" in line:
                    found = True
                    lines.append(f"net.ipv4.ip_local_port_range = {port_start} {port_end}\n")
                else:
                    lines.append(line)
        if not found:
            lines.append(f"net.ipv4.ip_local_port_range = {port_start} {port_end}\n")
        with open("/etc/sysctl.conf", "w") as f:
            f.writelines(lines)
        os.system("sysctl -p")
        return SUCCESS
    except Exception:
        return FAILURE


def clear_host_network_port_range():
    try:
        found = False
        lines = []
        with open("/etc/sysctl.conf", "r") as f:
            for line in f:
                if "net.ipv4.ip_local_port_range" in line:
                    found = True
                else:
                    lines.append(line)
        if found:
            with open("/etc/sysctl.conf", "w") as f:
                f.writelines(lines)
            os.system("sysctl -p")
        return SUCCESS
    except Exception:
        return FAILURE


def up_sta_netdev__with_static_ip_dns_route(static_ip, netmask, gateway, dns):
    g_sta_network_info.ip_addr = static_ip
    g_sta_network_info.netmask = netmask
    g_sta_network_info.gateway = gateway
    g_sta_network_info.dns_addr = dns
    g_sta_network_info.ip_valid = 1
    g_sta_network_info.dns_valid = 1
    g_sta_network_info.network_up = 1

    if not g_sta_network_info.ip_valid or not g_sta_network_info.dns_valid:
        print("Invalid STA IP [" + g_sta_network_info.ip_addr + "] or DNS [" + g_sta_network_info.dns_addr + "]")
        return FAILURE

    if not g_sta_network_info.network_up:
        print("Network is not up" + g_sta_network_info.network_up)
        return FAILURE

    if up_sta_netdev() != SUCCESS:
        return FAILURE

    # Set static IP

    if set_network_static_ip(STA_INTERFACE, g_sta_network_info.ip_addr, g_sta_network_info.netmask, g_sta_network_info.gateway) == FAILURE:
        print("Failed to set static IP[" + g_sta_network_info.ip_addr + "] NM[" + g_sta_network_info.netmask + "] GW [" + g_sta_network_info.gateway +"]")
        return FAILURE

    # Add default gateway
    if add_default_gateway(STA_INTERFACE) == FAILURE:
        if nw_debug_logs:
            print("Failed to add default gateway" + g_sta_network_info.gateway)

    # Add DNS
    if add_dns(STA_INTERFACE) == FAILURE:
        if nw_debug_logs:
            print("Failed to add DNS" + g_sta_network_info.dns_addr)

    return SUCCESS

def up_sta_netdev():
    if not g_sta_network_info:
        print("Network info is not valid" + g_sta_network_info)
        return FAILURE

    if not is_valid_mac_bytes(g_sta_network_info.mac_addr):
        print("MAC address is not valid (bytes):" + mac_bytes_to_str(g_sta_network_info.mac_addr))
        return FAILURE

    if set_hw_addr(STA_INTERFACE, g_sta_network_info.mac_addr) == FAILURE:
        print("Failed to set MAC address" + mac_bytes_to_str(g_sta_network_info.mac_addr))
        return FAILURE

    if interface_up(STA_INTERFACE) == FAILURE:
        print("Failed to up interface" + STA_INTERFACE)
        return FAILURE
    g_sta_network_info.network_up = 1
    return SUCCESS

def set_mac_addr(mac, iface=STA_INTERFACE):
    mac_bytes = mac_str_to_bytes(mac) if isinstance(mac, str) else bytes(mac)
    if iface == STA_INTERFACE:
        g_sta_network_info.mac_addr = mac_bytes
    elif iface == AP_INTERFACE:
        g_ap_network_info.mac_addr = mac_bytes
    return set_hw_addr(iface, mac_bytes)

def get_printable_mac_addr(mode):
    if mode == "station":
        return mac_bytes_to_str(g_sta_network_info.mac_addr)
    elif mode == "softap":
        return mac_bytes_to_str(g_ap_network_info.mac_addr)
    return None

def down_sta_netdev():
    # Remove DNS
    if remove_dns(STA_INTERFACE) == FAILURE:
        if nw_debug_logs:
            print("Failed to remove DNS" + g_sta_network_info.dns_addr)

    # Remove default gateway
    if remove_default_gateway(STA_INTERFACE) == FAILURE:
        if nw_debug_logs:
            print("Failed to remove default gateway" + g_sta_network_info.gateway)

    # Down interface
    if interface_down(STA_INTERFACE) == FAILURE:
        print("Failed to down interface" + STA_INTERFACE)
        return FAILURE
    g_sta_network_info.network_up = 0
    return SUCCESS

def up_softap_netdev():
    g_ap_network_info.ip_addr = os_softap_static_ip
    g_ap_network_info.netmask = os_softap_netmask
    g_ap_network_info.gateway = os_softap_gateway

    g_ap_network_info.ip_valid = 1
    g_ap_network_info.dns_valid = 1
    g_ap_network_info.network_up = 1

    if not g_ap_network_info.ip_valid or not g_ap_network_info.dns_valid:
        print("Invalid AP IP [" + g_ap_network_info.ip_addr + "] or DNS [" + g_ap_network_info.dns_addr + "]")
        return FAILURE

    if not g_ap_network_info.network_up:
        print("AP Network is not up" + g_ap_network_info.network_up)
        return FAILURE

    if not is_valid_mac_bytes(g_ap_network_info.mac_addr):
        print("AP MAC address is not valid (bytes):" + mac_bytes_to_str(g_ap_network_info.mac_addr))
        return FAILURE

    # Set MAC address
    if set_hw_addr(AP_INTERFACE, g_ap_network_info.mac_addr) == FAILURE:
        print("Failed to set AP MAC address" + mac_bytes_to_str(g_ap_network_info.mac_addr))
        return FAILURE

    # Up interface
    if interface_up(AP_INTERFACE) == FAILURE:
        print("Failed to up AP interface" + AP_INTERFACE)
        return FAILURE
    # Set static IP
    if set_network_static_ip(AP_INTERFACE, g_ap_network_info.ip_addr, g_ap_network_info.netmask, g_ap_network_info.gateway) == FAILURE:
        print("Failed to set static AP IP[" + g_ap_network_info.ip_addr + "] NM[" + g_ap_network_info.netmask + "] GW [" +g_ap_network_info.gateway +"]")
        return FAILURE

    g_ap_network_info.network_up = 1
    return SUCCESS

def down_softap_netdev():
    if not g_ap_network_info:
        print("AP Network info is not valid" + g_ap_network_info)
        return FAILURE

    if not g_ap_network_info.network_up:
        print("AP Network is not up" + g_ap_network_info.network_up)
        return FAILURE

    if not g_ap_network_info.mac_addr:
        print("AP MAC address is not valid" + g_ap_network_info.mac_addr)
        return FAILURE

    if interface_down(AP_INTERFACE) == FAILURE:
        print("Failed to down AP interface" + AP_INTERFACE)
        return FAILURE

    g_ap_network_info.ip_addr = ""
    g_ap_network_info.netmask = ""
    g_ap_network_info.gateway = ""

    g_ap_network_info.ip_valid = 0
    g_ap_network_info.dns_valid = 0
    g_ap_network_info.network_up = 0

    return SUCCESS

def run_dhcp_on_connected():

    if not g_run_dhcp_on_station_connected:
        print("Not running DHCP on connected, as requested in nw_helper_func.py")
        return SUCCESS

    if not g_sta_network_info:
        print("Network info is not valid" + g_sta_network_info)
        return FAILURE

    if not g_sta_network_info.network_up:
        print("Network is not up" + g_sta_network_info.network_up)
        return FAILURE

    if not g_sta_network_info.mac_addr:
        print("MAC address is not valid" + g_sta_network_info.mac_addr)
        return FAILURE

    # Kill existing DHCP client
    os.system(f"nohup killall dhclient > /dev/null 2>&1 &")

    #print("Starting `dhclient` hook. Check `ifconfig ethsta0` in new terminal for IP")
    # Run DHCP
    ret = os.system(f"nohup dhclient -v {STA_INTERFACE} > /dev/null 2>&1 &")

    return SUCCESS if ret == 0 else FAILURE

def stop_dhclient_on_disconnected():
    #print("Stopping `dhclient` hook")
    if not g_stop_dhclient_on_disconnected:
        return SUCCESS

    os.system(f"nohup killall dhclient > /dev/null 2>&1 &")
    return SUCCESS


def run_dhcp_server():
    if not g_run_dhcp_server_after_softap_up:
        print("Not running DHCP server, as requested in nw_helper_func.py")
        return SUCCESS

    if not g_ap_network_info:
        print("Network info is not valid" + g_ap_network_info)
        return FAILURE

    if not g_ap_network_info.network_up:
        print("Network is not up" + g_ap_network_info.network_up)
        return FAILURE

    ret = os.system("sudo bash ./run_dhcp_server.sh")
    if ret != 0:
        print("DHCP server (dnsmasq) not configured/running")
        print("\033[91m Please review/edit and run 'bash -x run_dhcp_server.sh for your platform' \033[0m")
        return FAILURE
    return SUCCESS

def stop_dhcp_server():
    if not g_stop_dhcp_server_after_softap_down:
        print("Not stopping DHCP server, as requested in nw_helper_func.py")
        return SUCCESS

    if not g_ap_network_info:
        print("Network info is not valid" + g_ap_network_info)
        return FAILURE
    ret = os.system("sudo bash ./stop_dhcp_server.sh")
    return SUCCESS if ret == 0 else FAILURE

def down_hci_instance():
    os.system(down_hci_instance_cmd)
    return SUCCESS

def reset_hci_instance():
    os.system(reset_hci_instance_cmd)
    return SUCCESS
