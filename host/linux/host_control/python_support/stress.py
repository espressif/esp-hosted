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
from time import *

STRESS_TEST_COUNT=50

TEST_MODE_NONE=(1 << 0)
TEST_SCAN_WIFI=(1 << 1)
TEST_STATION_MAC=(1 << 2)
TEST_STATION_CONNECT_DISCONNECT=(1 << 3)
TEST_SOFTAP_MAC=(1 << 4)
TEST_SOFTAP_START_STOP=(1 << 5)
TEST_STATION_SOFTAP_MODE=(1 << 6)
TEST_POWER_SAVE=(1 << 7)

STRESS_TEST=(TEST_MODE_NONE | TEST_SCAN_WIFI | TEST_STATION_MAC | TEST_STATION_CONNECT_DISCONNECT | TEST_SOFTAP_MAC | TEST_SOFTAP_START_STOP | TEST_STATION_SOFTAP_MODE | TEST_POWER_SAVE)

# Test APIs

# Below APIs could be used by demo application

# set wifi mode to none
if (STRESS_TEST & TEST_MODE_NONE):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_set_wifi_mode_none()

        test_get_wifi_mode()

# get available wifi
if (STRESS_TEST & TEST_SCAN_WIFI):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_get_available_wifi()

# station mode

# set MAC addeess for station mode
if (STRESS_TEST & TEST_STATION_MAC):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_set_wifi_mode_station()

        test_station_mode_set_mac_addr_of_esp()

        test_station_mode_get_mac_addr()

# station connect and disconnect
if (STRESS_TEST & TEST_STATION_CONNECT_DISCONNECT):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_station_mode_connect()

        test_station_mode_get_info()

        test_station_mode_disconnect()

# softAP mode

# set MAC address for softap mode
if (STRESS_TEST & TEST_SOFTAP_MAC):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_set_wifi_mode_softap()

        test_softap_mode_set_mac_addr_of_esp()

        test_softap_mode_get_mac_addr()

# softAP start, connect station and stop
if (STRESS_TEST & TEST_SOFTAP_START_STOP):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_softap_mode_start()

        test_softap_mode_get_info()

        print("***********************")
        print("Connect station to softAP :"+str(SOFTAP_MODE_SSID)+" within 15 seconds")

        sleep(15)
        print("***********************")

        test_softap_mode_connected_clients_info()

        test_softap_mode_stop()

# station + softAP mode

# test station+softap mode
if (STRESS_TEST & TEST_STATION_SOFTAP_MODE):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_set_wifi_mode_station_softap()

        test_get_available_wifi()

        test_station_mode_connect()

        test_softap_mode_start()

        test_station_mode_get_info()

        test_softap_mode_get_info()

        test_station_mode_disconnect()

        test_softap_mode_stop()

# power save mode

# set and get power save mode
if (STRESS_TEST & TEST_POWER_SAVE):
    for i in (range(STRESS_TEST_COUNT)):
        print("*************** "+str(i)+" ****************")
        test_set_wifi_power_save_mode()

        test_get_wifi_power_save_mode()
