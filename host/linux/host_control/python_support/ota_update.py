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

from commands import *
import argparse
import requests
import os

ota_status = "not_completed"

# Please do not use chunk size bigger than 4000.
chunk_size = 4000

parser = argparse.ArgumentParser(description='ota_update.py is a python script which runs OTA update on ESP')

parser.add_argument("url", type=str, default='', help="URL to OTA image")

args = parser.parse_args()

def OTA_END():
    ota_end = esp_ota_end()
    if (ota_end == "failure"):
        print("Failure in OTA end")
        exit()
    else:
        print("OTA end "+str(ota_end))

try:
    response = requests.get(args.url, stream = True)
except:
    print("Error while fetching URL")
    exit()

ota_status = esp_ota_begin()
if (ota_status == "failure"):
    print("Failure in OTA update")
    exit()
elif (ota_status == 0):
    print("OTA begin: success")
else:
    print("OTA begin:"+str(ota_status))

if (chunk_size>4000):
    chunk_size = 4000

for chunk in response.iter_content(chunk_size):
    print("|", end="", flush=True)
    ota_status = esp_ota_write(chunk)
    print(".", end="", flush=True)
    if (ota_status == "failure"):
        print("Failed to write OTA update")
        OTA_END()
        exit()
    print(".", end="", flush=True)
print("OTA write success")

OTA_END()
print("ESP32 will restart in 5 sec");
