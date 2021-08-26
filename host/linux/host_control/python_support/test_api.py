# Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from commands import *
from test_config import *
from socket import * 
from struct import *
from fcntl import *

sta_interface = 'ethsta0'
ap_interface = 'ethap0'

# From linux/socket.h
AF_UNIX      = 1

# From linux/if.h
IFF_UP       = 0x1

# From linux/sockios.h
SIOCGIFFLAGS = 0x8913
SIOCSIFFLAGS = 0x8914
SIOCSIFHWADDR = 0x8924

# Macros for packing data in 16 bytes short character bytes 
PACK_FORMAT = '16sh'

def get_bytes(string):
    if sys.version_info >= (3, 0):
        return bytes(string, 'utf-8')
    else:
        return string

def interface_down(sockfd, iface):
    ifreq = pack(PACK_FORMAT, get_bytes(iface), 0)
    flags = unpack(PACK_FORMAT, ioctl(sockfd, SIOCGIFFLAGS, ifreq))[1]
    flags = flags & ~IFF_UP
    ifreq = pack(PACK_FORMAT, get_bytes(iface), flags)
    ret = ioctl(sockfd, SIOCSIFFLAGS, ifreq)
    if (not ret):
        return failure
    else:
        return success

def interface_up(sockfd, iface):
    ifreq = pack(PACK_FORMAT, get_bytes(iface), 0)
    flags = unpack(PACK_FORMAT, ioctl(sockfd, SIOCGIFFLAGS, ifreq))[1]
    flags = flags | IFF_UP
    ifreq = pack(PACK_FORMAT, get_bytes(iface), flags)
    ret = ioctl(sockfd, SIOCSIFFLAGS, ifreq)
    if (not ret):
        return failure
    else:
        return success

def setHWaddr(sockfd, iface, mac):
    macbytes = [int(i, 16) for i in mac.split(':')]
    ifreq = pack('16sH6B8x', get_bytes(sta_interface), AF_UNIX, *macbytes)
    ret = ioctl(sockfd, SIOCSIFHWADDR, ifreq)
    if (not ret):
        return failure
    else:
        return success

def test_get_wifi_mode():
    mode = wifi_get_mode()
    if (mode != failure):
        print("Wifi mode is "+str(mode))
    else:
        print("Failed to get wifi mode")
    return

def test_set_wifi_mode(mode):
    ret = wifi_set_mode(mode)
    if (ret != failure):
        print("Wifi mode is "+str(mode))
    else:
        print("Error in setting mode")
    return

def test_set_wifi_mode_none():
    test_set_wifi_mode(WIFI_MODE_NONE)
    return

def test_set_wifi_mode_station():
    test_set_wifi_mode(WIFI_MODE_STATION)
    return

def test_set_wifi_mode_softap():
    test_set_wifi_mode(WIFI_MODE_SOFTAP)
    return

def test_set_wifi_mode_station_softap():
    test_set_wifi_mode(WIFI_MODE_SOFTAP_STATION)
    return

def test_station_mode_get_mac_addr():
    mac = wifi_get_mac(WIFI_MODE_STATION)
    if (mac != failure):
        print("Station mode: mac address "+str(mac))
    else:
        print("Failed to get station mode MAC address")
    return

def test_station_mode_set_mac_addr_of_esp():
    ret = wifi_set_mac(WIFI_MODE_STATION, STATION_MODE_MAC_ADDRESS)
    if (ret != failure):
        print("Station MAC address is set")
    else:
        print("Station MAC address is not set")
    return

def test_softap_mode_get_mac_addr():
    mac = wifi_get_mac(WIFI_MODE_SOFTAP)
    if (mac != failure):
        print("SoftAP mode: mac address "+str(mac))
    else:
        print("Failed to get softap mode MAC address")
    return

def test_softap_mode_set_mac_addr_of_esp():
    ret = wifi_set_mac(WIFI_MODE_SOFTAP, SOFTAP_MODE_MAC_ADDRESS)
    if (ret != failure):
        print("SoftAP MAC address is set")
    else:
        print("SoftAP MAC address is not set")
    return


def test_station_mode_connect():
    ret = wifi_set_ap_config(STATION_MODE_SSID, STATION_MODE_PWD, STATION_MODE_BSSID,\
            STATION_MODE_IS_WPA3_SUPPORTED, STATION_MODE_LISTEN_INTERVAL)
    if (ret != failure):
        print("Connected to "+STATION_MODE_SSID)
    else:
        print("Failed to connect to AP")
        return

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)
    if (not sockfd):
        print("Failed to open socket")
        return

    ret = interface_down(sockfd, sta_interface)
    if (ret != failure):
        print(sta_interface+" interface down")
    else:
        print("Unable to down "+sta_interface+" interface")
        return

    mac = wifi_get_mac(WIFI_MODE_STATION)
    if (mac != failure):
        print("Station mode: mac address "+str(mac))
    else:
        print("Failed to get MAC address")
        return

    ret = setHWaddr(sockfd, sta_interface, mac)
    if (ret != failure):
        print("MAC address "+mac+" set to "+sta_interface+" interface")
    else: 
        print("Unable to set MAC address to "+sta_interface)
        return

    ret = interface_up(sockfd, sta_interface)
    if (ret != failure):
        print(sta_interface+" interface up")
    else:
        print("Unable to up "+sta_interface+" interface")
    return

def test_station_mode_get_info():
    ret = wifi_get_ap_config()
    if (ret == not_connected) :
        print("Not connected to AP")
    elif (ret != failure):
        print("AP's SSID "+ret[0])
        print("AP's BSSID "+ret[1])
        print("AP's channel ID "+str(ret[2]))
        print("AP's RSSI "+str(ret[3]))
        print("AP's encryption mode "+str(ret[4]))
    else:
        print("Failed to get AP config")
    return

def test_get_available_wifi():
    ap_list = wifi_ap_scan_list()
    if (ap_list != failure):
        for obj in ap_list:
            print("AP's ssid: \""+obj.ssid+"\" bssid: \""+obj.bssid+"\" rssi: "+str(obj.rssi)+" channel: "+str(obj.chnl)+" authentication mode: "+str(obj.ecn))
    else:
        print("Failed to get scanned AP list")
    return

def test_station_mode_disconnect():
    ret = wifi_disconnect_ap()
    if (ret != failure):
        print("Disconnected from AP")
    else:
        print("Failed to disconnect from AP")
        return

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)
    if (not sockfd):
        print("Failed to open socket")
        return

    ret = interface_down(sockfd, sta_interface)
    if (ret != failure):
        print(sta_interface+" interface down")
    else:
        print("Unable to down "+sta_interface+" interface")
    return

def test_softap_mode_start():
    ret = wifi_set_softap_config(SOFTAP_MODE_SSID, SOFTAP_MODE_PWD, SOFTAP_MODE_CHANNEL, \
            SOFTAP_MODE_ENCRYPTION_MODE, SOFTAP_MODE_MAX_ALLOWED_CLIENTS, \
            SOFTAP_MODE_SSID_HIDDEN, SOFTAP_MODE_BANDWIDTH)
    if (ret != failure):
        print("ESP32 softAP started")
    else:
        print("Failed to set softAP config")
        return

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)
    if (not sockfd):
        print("Failed to open socket")
        return

    ret = interface_down(sockfd, ap_interface)
    if (ret != failure):
        print(ap_interface+" interface down")
    else:
        print("Unable to down "+ap_interface+" interface")
        return
        
    mac = wifi_get_mac(WIFI_MODE_SOFTAP)
    if (mac != failure):
        print("SoftAP mode: mac address "+str(mac))
    else:
        print("Failed to get MAC address of softAP interface")
        return

    ret = setHWaddr(sockfd, ap_interface, mac)
    if (ret != failure):
        print("MAC address "+mac+" set to "+ap_interface+" interface")
    else:
        print("Unable to set MAC address to "+ap_interface)
        return

    ret = interface_up(sockfd, ap_interface)
    if (ret != failure):
        print(ap_interface+" interface up")
    else:
        print("Unable to up "+ap_interface+" interface")
    return

def test_softap_mode_get_info():
    ret = wifi_get_softap_config()
    if (ret != failure):
        print("SoftAP's SSID "+ret[0])
        print("SoftAP's Password "+ret[1])
        print("SoftAP's channel ID "+str(ret[2]))
        print("SoftAP's encryption mode "+str(ret[3]))

        print("SoftAP's max connections "+str(ret[4]))
        print("SoftAP's ssid broadcast status "+str(ret[5]))

        print("SoftAP's bandwidth mode "+str(ret[6]))
    else:
        print("Failed to get softAP config")
    return

def test_softap_mode_connected_clients_info():
    stations_list = wifi_connected_stations_list()
    if (stations_list != failure):
        for obj in stations_list:
            print("station's bssid: "+obj.mac+" rssi: "+str(obj.rssi))
    else:
        print("Failed to get connected stations list")
    return

def test_softap_mode_stop():
    ret = wifi_stop_softap()
    if (ret != failure):
        print("ESP32 softAP stopped")
    else:
        print("Failed to stop ESP32 softAP")
        return

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)
    if (not sockfd):
        print("Failed to open socket")
        return

    ret = interface_down(sockfd, ap_interface)
    if (ret != failure):
        print(ap_interface+" interface down")
    else:
        print("Unable to down "+ap_interface+" interface")
    return

def test_set_wifi_power_save_mode():
    ret = wifi_set_power_save_mode(WIFI_PS_MIN_MODEM)
    if (ret != failure):
        print("Power save mode is set")
    else:
        print("Power save mode is not set")
    return

def test_get_wifi_power_save_mode():
    power_save_mode = wifi_get_power_save_mode()
    if (power_save_mode != failure):
        print("Power save mode is "+str(power_save_mode))
    else:
        print("Failed to set power save mode")
    return

