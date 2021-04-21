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
import argparse
import time
import os
import subprocess
from distutils.util import strtobool

# WiFi Mode
# NULL              0
# Station           1
# SoftAP            2
# Station + softAP  3

wifi_mode_none = 0
wifi_mode_station = 1
wifi_mode_softap = 2
wifi_mode_station_softap = 3
failure = "failure"
success = "success"
flag = success
station_status = 'Nothing set'

parser = argparse.ArgumentParser(description='station_connect.py is a python script which connect ESP32 station to AP. ex. python station_connect.py \'xyz\' \'xyz123456\' --bssid=\'e5:6c:67:3c:cf:65\' --is_wpa3_supported=True --listen_interval=3')

parser.add_argument("ssid", type=str, default='', help="ssid of AP")

parser.add_argument("password", type=str, default='', help="password of AP")

parser.add_argument("--bssid", type=str, default='', help="bssid i.e MAC address of AP in case of multiple AP has same ssid (default: '')")

parser.add_argument('--is_wpa3_supported', type=lambda x: bool(strtobool(x)), default=False, help="wpa3 support present on AP (False:Unsupported) (True: Supported)")

parser.add_argument("--listen_interval", type=int, default=3, help="Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set. Units: AP beacon intervals. (Defaults to 3 if set to 0)")

args = parser.parse_args()

sta_mac = wifi_get_mac(wifi_mode_station)
if (sta_mac == failure):
    flag = failure
    print("Failed to get station MAC address")
else :
    print("station MAC address "+str(sta_mac))

if (flag == success):
    station_status = wifi_set_ap_config(args.ssid,args.password,args.bssid,args.is_wpa3_supported,args.listen_interval)
    if (station_status != success):
        flag = failure
        print("Failed to set AP config")
    elif (station_status != success):
        print("Connected to "+args.ssid)

if (flag != success):
    print("Failed to connect with AP")
else:
    command = 'sudo ifconfig ethsta0 down'
    os.system(command)
    print(command)

    command = 'sudo ifconfig ethsta0 hw ether '+str(sta_mac)
    os.system(command)
    print(command)

    command = 'sudo ifconfig ethsta0 up'
    os.system(command)
    print(command)

    time.sleep(1)

    for x in range (5):
        command = 'sudo dhclient ethsta0 -r'
        os.system(command)
        print(command)

        command = 'sudo dhclient ethsta0 -v'
        os.system(command)
        print(command)

        try:
            get_ip = subprocess.check_output('ip addr show | grep "ethsta0" | grep -o "inet [0-9]*\.[0-9]*\.[0-9]*\.[0-9]*" | grep -o "[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*"', shell = True)
            if get_ip:
                flag = success
                break
        except subprocess.CalledProcessError:
            time.sleep(0.1)
        flag = failure

    if (flag == failure):
        print("Failed to assign IP address to ethsta0 interface")
    else:
        print("Success in setting AP config")



