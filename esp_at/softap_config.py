# Copyright 2019 Espressif Systems (Shanghai) PTE LTD
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

from at_commands import at_commands
import argparse
import time
import os

parser = argparse.ArgumentParser(description='softap_config.py script to configure ESP32 softAP mode. User should provide parameters like ssid, password(password length should be 8~64 bytes ASCII), channel ID (It can be any number between 1 to 11), encryption method (0 : OPEN, 2: WPA_PSK, 3:WPA2_PSK, 4: WPA_WPA2_PSK), max connection count( number of Stations to which ESP32 SoftAP can be connected, within the range of [1, 10]) and ssid hidden (it can set to 1 if softAP shouldnt broadcast its ssid else 0). max connection count and ssid hidden parameters are optional user can set this filed to 0. ex. python3 softap_config.py \'xyz\' \'xyz123456\' 1 3 4 0 ')

parser.add_argument("ssid", type=str, default='0', help="ssid")

parser.add_argument("password", type=str, default='0', help="password")

parser.add_argument("channel_id", type=int, default='0', help="channel ID")

parser.add_argument("encrp_mthd", type=int, default=0, help="encryption method")

parser.add_argument("max_conn", type=int, default=0, help="max connection")

parser.add_argument("ssid_hidden", type=int, default=0, help="ssid hidden/broadcast")

args = parser.parse_args()
flag = 'success'
softap_config = 'failure'
ssid = '0'
pwd = '0'
chnl = 0
encrp_mthd = 0
max_conn = 0
ssid_hidden = 0
wifi_mode = 0

ap_mac = at_commands.get_ap_mac()
print("AP MAC Address "+str(ap_mac))

if (ap_mac == 'failure'):
    flag = 'failure'

if (flag == 'success'):
    wifi_mode = at_commands.wifi_get_mode()
    print("Current wifi mode "+str(wifi_mode))

if (wifi_mode != '2' and wifi_mode != 'failure'):
    print("set a wifi mode 2 (i.e. softap mode)")
    wifi_mode = at_commands.wifi_set_mode(2)
    print(wifi_mode)

if (wifi_mode == 'failure'):
    print("wifi get\set mode failed")
    flag = 'failure'

if (flag == 'success'):
    print("check soft AP config")
    softap_config = at_commands.wifi_get_softap_config()
    if (softap_config == 'failure'):
        flag = 'failure'
        print("Failed to check current softap config")

if (flag == 'success' and softap_config != 'OK'):
    ssid = softap_config[0]
    pwd = softap_config[1]
    chnl = softap_config[2]
    encrp_mthd = softap_config[3]
    max_conn = softap_config[4]
    ssid_hidden = softap_config[5]

if (flag =='success' and args.ssid == ssid and args.password == pwd and args.channel_id == str(chnl) and args.encrp_mthd == str(encrp_mthd) and args.max_conn == str(max_conn) and args.ssid_hidden == str(ssid_hidden)):
    print("already softAP config set")
elif (flag == 'success'):
    softap_config = at_commands.wifi_set_softap_config(args.ssid, args.password, args.channel_id, args.encrp_mthd, args.max_conn, args.ssid_hidden)
    if (softap_config == 'failure'):
        print("setting softap config failed")
        flag = 'failure'
 
if (flag == 'failure'):
    print("failure in setting AP config")

if (flag == 'success'):
    command = 'sudo ifconfig ethap0 down'
    os.system(command)
    print(command)    
    
    command = 'sudo ifconfig ethap0 hw ether '+str(ap_mac)
    os.system(command)
    print(command)    
    
    command = 'ifconfig ethap0 up'
    os.system(command)
    print(command)
    
    time.sleep(5)
 
    print("softAP config successfully set")
