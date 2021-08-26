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

from commands_lib import *
import argparse
import time
import os
import subprocess
from distutils.util import strtobool

flag = success
sta_mac = 'not_set'
station_status = 'not_set'

parser = argparse.ArgumentParser(description='station_connect.py is a python script which connect ESP32 station to AP. ex. python station_connect.py \'xyz\' \'xyz123456\' --bssid=\'e5:6c:67:3c:cf:65\' --is_wpa3_supported=True --listen_interval=3')

parser.add_argument("ssid", type=str, default='', help="ssid of AP")

parser.add_argument("password", type=str, default='', help="password of AP")

parser.add_argument("--bssid", type=str, default='', help="bssid i.e MAC address of AP in case of multiple AP has same ssid (default: '')")

parser.add_argument('--is_wpa3_supported', type=lambda x: bool(strtobool(x)), default=False, help="wpa3 support present on AP (False:Unsupported) (True: Supported)")

parser.add_argument("--listen_interval", type=int, default=3, help="Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set. Units: AP beacon intervals. (Defaults to 3 if set to 0)")

args = parser.parse_args()

sta_mac = wifi_get_mac(WIFI_MODE_STATION)
if (sta_mac == failure):
    flag = failure
    print("Failed to get station MAC address")
else :
    print("station MAC address "+str(sta_mac))

if (flag == success):
    station_status = wifi_set_ap_config(args.ssid,args.password,args.bssid,args.is_wpa3_supported,args.listen_interval)
    if (station_status != success):
        flag = failure
        if (station_status == no_ap_found_str):
            print("SSID: "+args.ssid+" not found")
        elif (station_status == invalid_password_str):
            print("Incorrect Password: "+args.password)
        print("Failed to connect with AP")
    else:
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

    print("Interface ethsta0 is up with MAC address "+str(sta_mac))
