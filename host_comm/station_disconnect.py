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

from host_commands import slave_comm
import argparse

# WiFi Mode
# NULL              0
# Station           1
# SoftAP            2
# Station + SoftAP  3

null = 0
station = 1
softap = 2
station_softap = 3
failure = "failure"
disconnect = "Not set"

parser = argparse.ArgumentParser(description='station_disconnect.py script will disconnect ESPStation from AP ex. python station_disconnect.py')

wifi_mode = slave_comm.get_wifi_mode()
print(wifi_mode)

if (wifi_mode == failure):
    print("failure in getting wifi mode")
elif (wifi_mode == station or wifi_mode == station_softap):
    disconnect = slave_comm.wifi_disconnect_ap()
    print(disconnect)
    print("Disconnected from AP")
else :
    print("wifi_disconnect_ap failed, current mode is "+str(wifi_mode))
    print("0: null Mode, Wi-Fi RF will be disabled")
    print("1: station mode")
    print("2: softAP mode")
    print("3: softAP+station mode")
