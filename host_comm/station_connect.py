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

from host_commands import slave_comm
import argparse
import time
import os

parser = argparse.ArgumentParser(description='station_connect.py is a python script which connect ESPstation to AP. ex. python3 station_connect.py \'xyz\' \'xyz123456\' --bssid=\'e5:6c:67:3c:cf:65\'')

parser.add_argument("ssid", type=str, default='0', help="ssid of AP")

parser.add_argument("password", type=str, default='0', help="password of AP")

parser.add_argument("--bssid", type=str, default='0', help="bssid i.e MAC address of AP in case of multiple AP has same ssid (default: '0')")

args = parser.parse_args()

flag = 'success'
station_status = 'Nothing set'
current_ssid = '0'
wifi_mode = 0

sta_mac = slave_comm.get_mac(1)
print("station MAC address "+str(sta_mac))

station_status = slave_comm.wifi_set_ap_config(args.ssid,args.password,args.bssid)
if (station_status == 'failure'):
    flag = 'failure'
    print("Failed to set AP config")
elif (station_status == 'success'):
    print("Connected to given AP")

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

