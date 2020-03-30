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
stop = "Not set"

parser = argparse.ArgumentParser(description='softap_stop.py script will stop ESP32 softap ex. python softap_close.py')

wifi_mode = slave_comm.get_wifi_mode()
print(wifi_mode)

if (wifi_mode == failure):
    print("failure in getting wifi mode")
elif (wifi_mode == softap):
    wifi_mode = slave_comm.set_wifi_mode(null)
    if (wifi_mode == failure):
        print("failure in stopping softap")
    else :
        print("SoftAP stopped")
elif (wifi_mode == station_softap):
    wifi_mode = slave_comm.set_wifi_mode(station)
    if (wifi_mode == failure):
        print("failure in stopping softap")
    else :
        print("SoftAP stopped")
