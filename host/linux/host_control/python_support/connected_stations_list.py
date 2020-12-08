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
stations_list = "No station is connected"

parser = argparse.ArgumentParser(description='connected_stations_list.py is a python script which gives list of mac addresses of stations connected to softAP. ex. python connected_stations_list.py')

get_mode = commands.wifi_get_mode()
if (get_mode == softap or get_mode == station_softap):
    stations_list = commands.wifi_connected_stations_list()
    if (stations_list == failure):
        print("failure in getting connected stations list")
    else:
        for obj in stations_list:
            print(obj.mac)
elif (get_mode == failure):
    print("failure in getting wifi mode")
else :
    print("current mode is not station/station+softap")
