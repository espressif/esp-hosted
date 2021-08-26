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

from test_api import *
#***** Please Read *****
#* Before use test.py : User must enter user configuration parameter in "test_config.py" file *

# Test APIs

# Below APIs could be used by demo application

test_set_wifi_mode_none()
test_get_wifi_mode()
test_get_available_wifi()

# station mode
test_set_wifi_mode_station()
test_station_mode_set_mac_addr_of_esp()
test_station_mode_get_mac_addr()
test_station_mode_connect()
test_station_mode_get_info()
test_station_mode_disconnect()

# softAP mode

test_set_wifi_mode_softap()
test_softap_mode_set_mac_addr_of_esp()
test_softap_mode_get_mac_addr()
test_softap_mode_start()
test_softap_mode_get_info()
test_softap_mode_connected_clients_info()
test_softap_mode_stop()

# station + softAP mode

test_set_wifi_mode_station_softap()
test_station_mode_connect()
test_softap_mode_start()
test_station_mode_get_info()
test_softap_mode_get_info()
test_station_mode_disconnect()
test_softap_mode_stop()

# power save mode

test_set_wifi_power_save_mode()
test_get_wifi_power_save_mode()
