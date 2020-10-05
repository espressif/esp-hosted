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
import os

# WiFi Mode
# NULL              0
# Station           1
# SoftAP            2
# Station + SoftAP  3

none = 0
station = 1
softap = 2
station_softap = 3
failure = "failure"
success = "success"
flag = success
disconnect = "Not set"

parser = argparse.ArgumentParser(description='station_disconnect.py script will disconnect ESPStation from AP ex. python station_disconnect.py')

wifi_mode = commands.wifi_get_mode()
print("WiFi Mode: "+str(wifi_mode))

if (wifi_mode == failure):
    print("Failed to get WiFi Mode")
    flag = failure
elif (wifi_mode == station or wifi_mode == station_softap):
    disconnect = commands.wifi_disconnect_ap()
    if (disconnect == failure):
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

