# Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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

from host_commands import commands
import argparse
import time
import os

# WiFi Mode
# NULL                  0
# Station               1
# SoftAP                2
# Station+SoftAP        3

none = 0
station = 1
softap = 2
station_softap = 3
success = 'success'
failure = 'failure'
flag = success
softap_config = 'Not set'

parser = argparse.ArgumentParser(description='softap_config.py script to configure ESP32 softAP mode. ex. python softap_config.py \'xyz\' \'xyz123456\' 1 3 --max_conn=4 --ssid_hidden=0 --bw=1')

parser.add_argument("ssid", type=str, default='0', help="ssid")

parser.add_argument("password", type=str, default='0', help="password(password length should be 8~64 bytes ASCII)")

parser.add_argument("channel_id", type=int, default=1, help="channel ID (It can be any number between 1 to 11)")

parser.add_argument("encrp_mthd", type=int, default=0, help="encryption method (0 : OPEN, 2: WPA_PSK, 3:WPA2_PSK, 4: WPA_WPA2_PSK)")

parser.add_argument("--max_conn", type=int, default=1, help="max connection count( number of Stations to which ESP32 SoftAP can be connected, within the range of [1, 10])")

parser.add_argument("--ssid_hidden", type=int, default=0, help="ssid hidden/broadcast (it can set to 1 if softAP shouldnt broadcast its ssid else 0)")

parser.add_argument("--bw", type=int, default=1, help="Bandwidth (1: WIFI_BW_HT20(20MHZ)) , (2: WIFI_BW_HT40(40MHZ)) default is 20MHZ")

args = parser.parse_args()

ap_mac = commands.wifi_get_mac(softap)
if (ap_mac == failure):
    print("Failed to get is SoftAP mac address")
    flag = failure
else :
    print("SoftAP MAC Address "+str(ap_mac))

if (flag == success):
    softap_config = commands.wifi_set_softap_config(args.ssid, args.password, args.channel_id, args.encrp_mthd, args.max_conn, args.ssid_hidden, args.bw)
    if (softap_config == failure):
        print("setting softap config failed")
        flag = failure
    else:
        print("setting softap config success")
 
if (flag == failure):
    print("failure in setting SoftAP config")

if (flag == success):
    command = 'sudo ifconfig ethap0 down'
    os.system(command)
    print(command)    
    
    command = 'sudo ifconfig ethap0 hw ether '+str(ap_mac)
    os.system(command)
    print(command)    
    
    command = 'sudo ifconfig ethap0 up'
    os.system(command)
    print(command)
    
    time.sleep(1)
 
    print("softAP config successfully set")
