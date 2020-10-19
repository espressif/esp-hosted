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

from commands import *
import argparse
import os

# WiFi Mode
# NULL              0
# Station           1
# SoftAP            2
# Station + SoftAP  3

wifi_mode_none = 0
wifi_mode_station = 1
wifi_mode_softap = 2
wifi_mode_station_softap = 3
failure = "failure"
success = "success"
flag = success
disconnect = "Not set"

parser = argparse.ArgumentParser(description='station_disconnect.py script will disconnect ESPStation from AP ex. python station_disconnect.py')

wifi_mode = wifi_get_mode()
print("WiFi Mode: "+str(wifi_mode))

if (wifi_mode == failure):
    print("Failed to get WiFi Mode")
    flag = failure
elif ((wifi_mode == wifi_mode_station) or (wifi_mode == wifi_mode_station_softap)):
    disconnect = wifi_disconnect_ap()
    if (disconnect != success):
        print("Failed to Disconnected from AP")
        flag = failure
    else :
        print("Success in Disconnecting from AP")
else :
    print("Station is not enabled")
    flag = failure

if (flag == success):
    command = 'sudo dhclient ethsta0 -r'
    print('$ '+command)
    os.system(command)
    
    command = 'sudo ifconfig ethsta0 down'
    print('$ '+command)
    os.system(command)

