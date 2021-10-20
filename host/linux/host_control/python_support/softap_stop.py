# Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
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

from commands_lib import *
import argparse
import os

wifi_mode = 'not_set'
stop_softap = 'not_set'
flag = success

parser = argparse.ArgumentParser(description='softap_stop.py script will stop ESP32 softap ex. python softap_close.py')

wifi_mode = wifi_get_mode()

if (wifi_mode == failure):
    print("Failed to get wifi mode")
    flag = failure
else:
    print("WiFi Mode: "+str(wifi_mode))
    stop_softap = wifi_stop_softap()
    if (stop_softap != success):
        print("Failed to stop softAP")
        flag = failure
    else:
        print("SoftAP stopped")

if (flag == success):
    command = 'sudo ifconfig ethap0 down'
    os.system(command)
    print(command)
    print("SoftAP interface down")
