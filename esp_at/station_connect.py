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

parser = argparse.ArgumentParser(description='station_connect.py is a python script which connect ESPstation to AP. User should provide ssid, password and bssid. In case user dont want to enter password,bssid then set field as 0. bssid i.e MAC address of AP in case of multiple AP has same ssid. ex. python3 station_connect.py \'xyz\' \'xyz123456\' \'0\'')

parser.add_argument("ssid", type=str, default='0', help="ssid")

parser.add_argument("password", type=str, default='0', help="password")

parser.add_argument("bssid", type=str, default='0', help="bssid")

args = parser.parse_args()
flag = 'success'
station_status = 'Nothing set'
current_ssid = '0'
wifi_mode = 0

sta_mac = at_commands.get_sta_mac()
print("station MAC address "+str(sta_mac))

if (sta_mac == 'failure'):
    flag = 'failure'

if (flag == 'success'):
    wifi_mode = at_commands.wifi_get_mode()
    print("current wifi mode is "+str(wifi_mode))
    if (wifi_mode != '1' and wifi_mode != 'failure'):
    	print("set wifi mode to 1 (i.e. station mode)")
    	wifi_mode = at_commands.wifi_set_mode(1)

if (wifi_mode == 'failure'):
    print("wifi get\set mode failed")
    flag = 'failure'

if (flag == 'success'):
    print("Get previously entered AP configuration")
    station_status = at_commands.wifi_get_ap_config()
    if (station_status == 'failure'):    
        print("Failed to get previous AP configuration ")
        flag = 'failure'
if (flag == 'success' and station_status != 'OK'):
    current_ssid = station_status[0]
    current_ssid = current_ssid[1:-1]
    print("previous AP ssid "+str(current_ssid))
if ((flag == 'success') and ((station_status == 'OK') or (current_ssid != args.ssid))):
    print("set AP config")
    station_status = at_commands.wifi_set_ap_config(args.ssid,args.password,args.bssid)
    if (station_status == 'failure'):
        flag = 'failure'
        print("Failed to set AP config")
elif((current_ssid == args.ssid) and flag == 'success'):
    print("entered ssid is same as previous configuration")

if (flag == 'failure'):
    print("failure in setting AP config")

if (flag == 'success'):
    command = 'sudo ifconfig ethsta0 down'
    os.system(command)
    print(command)

    command = 'sudo ifconfig ethsta0 hw ether '+str(sta_mac)
    os.system(command)
    print(command)

    command = 'ifconfig ethsta0 up'
    os.system(command)
    print(command)

    time.sleep(5)

    command = 'dhclient ethsta0 -r'
    os.system(command)
    print(command)

    command = 'dhclient ethsta0 -v'
    os.system(command)
    print(command)

    print("Success in setting AP config")
