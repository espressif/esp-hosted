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

from ctypes import *
from hosted_config import *
import os.path
import subprocess
import sys

lib = os.path.isfile("commands.so")
if not lib:
    print("commands.so file is missing, please run ./rpi_init.sh script.")
    sys.exit()

commands_lib = cdll.LoadLibrary(os.path.abspath("commands.so"))

wifi_get_mac = commands_lib.wifi_get_mac
wifi_get_mac.restype = c_int

wifi_get_mode = commands_lib.wifi_get_mode
wifi_get_mode.restype = c_int

wifi_set_mode = commands_lib.wifi_set_mode
wifi_set_mode.restype = c_int

wifi_set_ap_config = commands_lib.wifi_set_ap_config
wifi_set_ap_config.restype = c_int

wifi_get_ap_config = commands_lib.wifi_get_ap_config
wifi_get_ap_config.restype = c_int

wifi_disconnect_ap = commands_lib.wifi_disconnect_ap
wifi_disconnect_ap.restype = c_int

wifi_set_softap_config = commands_lib.wifi_set_softap_config
wifi_set_softap_config.restype = c_int

wifi_get_softap_config = commands_lib.wifi_get_softap_config
wifi_get_softap_config.restype = c_int

wifi_ap_scan_list = commands_lib.wifi_ap_scan_list
wifi_ap_scan_list.restype = c_void_p

wifi_connected_stations_list = commands_lib.wifi_connected_stations_list
wifi_connected_stations_list.restype = c_void_p

wifi_set_mac = commands_lib.wifi_set_mac
wifi_set_mac.restype = c_int

wifi_set_power_save_mode = commands_lib.wifi_set_power_save_mode
wifi_set_power_save_mode.restype = c_int

wifi_get_power_save_mode = commands_lib.wifi_get_power_save_mode
wifi_get_power_save_mode.restype = c_int

wifi_stop_softap = commands_lib.wifi_stop_softap
wifi_stop_softap.restype = c_int

wifi_set_max_tx_power = commands_lib.wifi_set_max_tx_power
wifi_set_max_tx_power.restype = c_int

wifi_get_curr_tx_power = commands_lib.wifi_get_curr_tx_power
wifi_get_curr_tx_power.restype = c_int

esp_hosted_free = commands_lib.esp_hosted_free
esp_hosted_free.restype = None

control_path_platform_init = commands_lib.control_path_platform_init
control_path_platform_init.restype = c_int

esp_ota_begin = commands_lib.esp_ota_begin
esp_ota_begin.restype = c_int

esp_ota_write = commands_lib.esp_ota_write
esp_ota_write.restype = c_int

esp_ota_end = commands_lib.esp_ota_end
esp_ota_end.restype = c_int

interface_up = commands_lib.interface_up
interface_up.restype = c_int

interface_down = commands_lib.interface_down
interface_down.restype = c_int

set_hw_addr = commands_lib.set_hw_addr
set_hw_addr.restype = c_int

create_socket = commands_lib.create_socket
create_socket.restype = c_int

close_socket = commands_lib.close_socket
close_socket.restype = c_int
