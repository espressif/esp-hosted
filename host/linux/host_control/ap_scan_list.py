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

failure = "failure"
ap_list = 'No APs Available'

parser = argparse.ArgumentParser(description='ap_scan_list.py is a python script which gives scanned list of available APs. list contains ssid, channel number, rssi, mac address and authentication mode of AP. ex. python ap_scan_list.py')

ap_list = commands.wifi_ap_scan_list()
if (ap_list == failure):
    print("Failed to get AP scan list")
else:
    for obj in ap_list:
        print(obj.ssid,obj.chnl,obj.rssi,obj.bssid,obj.ecn)
