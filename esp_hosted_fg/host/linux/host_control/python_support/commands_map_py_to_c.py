# SPDX-License-Identifier: Apache-2.0
# Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
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
from hosted_py_header import *
import os.path
import subprocess
import sys

lib = os.path.isfile("commands.so")
if not lib:
	print("commands.so file is missing, please run ./rpi_init.sh script.")
	sys.exit()

commands_lib = cdll.LoadLibrary(os.path.abspath("commands.so"))

init_hosted_control_lib = commands_lib.init_hosted_control_lib
init_hosted_control_lib.restype = c_int

deinit_hosted_control_lib = commands_lib.deinit_hosted_control_lib
deinit_hosted_control_lib.restype = c_int

wifi_get_mac = commands_lib.wifi_get_mac
wifi_get_mac.restype = POINTER(CONTROL_COMMAND)

wifi_set_mac = commands_lib.wifi_set_mac
wifi_set_mac.restype = POINTER(CONTROL_COMMAND)

wifi_get_mode = commands_lib.wifi_get_mode
wifi_get_mode.restype = POINTER(CONTROL_COMMAND)

wifi_set_mode = commands_lib.wifi_set_mode
wifi_set_mode.restype = POINTER(CONTROL_COMMAND)

wifi_set_power_save_mode = commands_lib.wifi_set_power_save_mode
wifi_set_power_save_mode.restype = POINTER(CONTROL_COMMAND)

wifi_get_power_save_mode = commands_lib.wifi_get_power_save_mode
wifi_get_power_save_mode.restype = POINTER(CONTROL_COMMAND)

wifi_ap_scan_list = commands_lib.wifi_ap_scan_list
wifi_ap_scan_list.restype = POINTER(CONTROL_COMMAND)

wifi_get_ap_config = commands_lib.wifi_get_ap_config
wifi_get_ap_config.restype = POINTER(CONTROL_COMMAND)

wifi_connect_ap = commands_lib.wifi_connect_ap
wifi_connect_ap.restype = POINTER(CONTROL_COMMAND)

wifi_disconnect_ap = commands_lib.wifi_disconnect_ap
wifi_disconnect_ap.restype = POINTER(CONTROL_COMMAND)

wifi_start_softap = commands_lib.wifi_start_softap
wifi_start_softap.restype = POINTER(CONTROL_COMMAND)

wifi_get_softap_config = commands_lib.wifi_get_softap_config
wifi_get_softap_config.restype = POINTER(CONTROL_COMMAND)

wifi_stop_softap = commands_lib.wifi_stop_softap
wifi_stop_softap.restype = POINTER(CONTROL_COMMAND)

wifi_get_softap_connected_station_list = commands_lib.wifi_get_softap_connected_station_list
wifi_get_softap_connected_station_list.restype = POINTER(CONTROL_COMMAND)

wifi_set_vendor_specific_ie = commands_lib.wifi_set_vendor_specific_ie
wifi_set_vendor_specific_ie.restype = POINTER(CONTROL_COMMAND)

wifi_set_max_tx_power = commands_lib.wifi_set_max_tx_power
wifi_set_max_tx_power.restype = POINTER(CONTROL_COMMAND)

wifi_get_curr_tx_power = commands_lib.wifi_get_curr_tx_power
wifi_get_curr_tx_power.restype = POINTER(CONTROL_COMMAND)

config_heartbeat = commands_lib.config_heartbeat
config_heartbeat.restype = POINTER(CONTROL_COMMAND)

ota_begin = commands_lib.ota_begin
ota_begin.restype = POINTER(CONTROL_COMMAND)

ota_write = commands_lib.ota_write
ota_write.restype = POINTER(CONTROL_COMMAND)

ota_end = commands_lib.ota_end
ota_end.restype = POINTER(CONTROL_COMMAND)

hosted_free = commands_lib.hosted_free
hosted_free.restype = None

create_socket = commands_lib.create_socket
create_socket.restype = c_int

close_socket = commands_lib.close_socket
close_socket.restype = c_int

interface_up = commands_lib.interface_up
interface_up.restype = c_int

interface_down = commands_lib.interface_down
interface_down.restype = c_int

set_hw_addr = commands_lib.set_hw_addr
set_hw_addr.restype = c_int

set_event_callback = commands_lib.set_event_callback
set_event_callback.restype = c_int

control_path_platform_init = commands_lib.control_path_platform_init
control_path_platform_init.restype = c_int

reset_event_callback = commands_lib.reset_event_callback
reset_event_callback.restype = c_int
