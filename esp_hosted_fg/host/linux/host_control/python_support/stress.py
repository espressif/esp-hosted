#!/usr/bin/env python3

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

import sys

if sys.version_info[0] < 3:
	print("please re-run using python3")
	exit()

from time import *
from py_parse.cmds import ctrl_cmd
from py_parse.process import process_init_control_lib, process_deinit_control_lib, process_heartbeat
from hosted_py_header import *
import traceback

STRESS_TEST_COUNT=100
sta_mode = 'station'
sta_mac = 'aa:bb:cc:dd:ee:ff'
sta_ssid = 'MyWifi'
sta_pwd = 'MyWifiPass@123'

softap_mode = 'softap'
softap_mac = 'cc:bb:aa:ee:ff:dd'
softap_ssid = 'ESPWiFi'
softap_pwd = 'ESPWiFi@123'

sta_softap_mode = 'station+softap'
max_tx_pwr_input = 22

event1 = 'esp_init'
event2 = 'heartbeat'
event3 = 'sta_disconnect_from_ap'
event4 = 'sta_disconnect_from_softap'

argumentList = sys.argv[1:]
if argumentList and len(argumentList):
	if "--help" in argumentList or "help" in argumentList:
		pass
	else:	
		print("arguments are not required for this script")
	print("Please edit STRESS_TEST_COUNT in stress.py")
	print("Edit bitmasks need to be tested and re-run as -\nsudo python3 stress.py")
	sys.exit()

bit = 0

TEST_EVENTS=(1 << bit)
bit += 1

TEST_MODE_STA=(1 << bit)
bit += 1

TEST_SCAN_WIFI=(1 << bit)
bit += 1

TEST_STATION_MAC=(1 << bit)
bit += 1

TEST_STATION_CONNECT_DISCONNECT=(1 << bit)
bit += 1

TEST_MODE_SOFTAP=(1 << bit)
bit += 1

TEST_SOFTAP_MAC=(1 << bit)
bit += 1

TEST_SOFTAP_VENDOR_IE=(1 << bit)
bit += 1

TEST_SOFTAP_START_STOP=(1 << bit)
bit += 1

TEST_STATION_SOFTAP_MODE=(1 << bit)
bit += 1

TEST_HEARTBEAT=(1 << bit)
bit += 1

TEST_POWER_SAVE=(1 << bit)
bit += 1

TEST_WIFI_TX_POWER=(1 << bit)

STRESS_TEST=(TEST_EVENTS | TEST_MODE_STA | TEST_SCAN_WIFI |
		TEST_STATION_MAC | TEST_STATION_CONNECT_DISCONNECT |
		TEST_MODE_SOFTAP | TEST_SOFTAP_MAC | TEST_SOFTAP_VENDOR_IE |
		TEST_SOFTAP_START_STOP |  TEST_STATION_SOFTAP_MODE |
		TEST_HEARTBEAT | TEST_POWER_SAVE | TEST_WIFI_TX_POWER)


process_init_control_lib()
try:
	cmd = ctrl_cmd()
	
	# Config heartbeat
	if (STRESS_TEST & TEST_EVENTS):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.unsubscribe_event(event1)
			cmd.subscribe_event(event1)
			cmd.unsubscribe_event(event2)
			cmd.subscribe_event(event2)
			cmd.unsubscribe_event(event3)
			cmd.subscribe_event(event3)
			cmd.unsubscribe_event(event4)
			cmd.subscribe_event(event4)
	
	if (STRESS_TEST & TEST_HEARTBEAT):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.heartbeat(False)
			cmd.heartbeat(True, 10)
	
	
	# set wifi mode to station
	if (STRESS_TEST & TEST_MODE_STA):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.wifi_set_mode(sta_mode)
			cmd.wifi_get_mode()
	
	# get available wifi
	if (STRESS_TEST & TEST_SCAN_WIFI):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.get_available_ap()
	
	# station mode
	
	# set MAC addeess for station mode
	if (STRESS_TEST & TEST_STATION_MAC):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.wifi_set_mac(sta_mode, sta_mac)
			cmd.wifi_get_mac(sta_mode)
	
	# station connect and disconnect
	if (STRESS_TEST & TEST_STATION_CONNECT_DISCONNECT):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.connect_ap(sta_ssid, sta_pwd)
			cmd.get_connected_ap_info()
			cmd.disconnect_ap()
	
	# softAP mode
	
	# set wifi mode to softap
	if (STRESS_TEST & TEST_MODE_SOFTAP):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.wifi_set_mode(softap_mode)
			cmd.wifi_get_mode()
	
	if (STRESS_TEST & TEST_SOFTAP_MAC):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.wifi_set_mac(softap_mode, softap_mac)
			cmd.wifi_get_mac(softap_mode)
	
	
	# set MAC address for softap mode
	if (STRESS_TEST & TEST_SOFTAP_VENDOR_IE):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			print("Reset softap vendor IE")
			cmd.softap_vendor_ie(False)
			cmd.softap_vendor_ie(True, "Test Vendor IE")
			print("Set softap vendor IE")
	
	# softAP start, connect station and stop
	if (STRESS_TEST & TEST_SOFTAP_START_STOP):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.start_softap(softap_ssid, softap_pwd)
			cmd.get_softap_info()
	
			print("***********************")
			print("Connect station to softAP :"+str(softap_ssid)+" within 15 seconds")
	
			sleep(15)
			print("***********************")
	
			cmd.softap_connected_clients_info()
			cmd.stop_softap()
	
	# station + softAP mode
	
	# test station+softap mode
	if (STRESS_TEST & TEST_STATION_SOFTAP_MODE):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.wifi_set_mode(sta_softap_mode)
			cmd.wifi_get_mode()
	
			cmd.get_available_ap()
	
			cmd.connect_ap(sta_ssid, sta_pwd)
			cmd.start_softap(softap_ssid, softap_pwd)
	
			cmd.get_connected_ap_info()
			cmd.get_softap_info()
	
			cmd.disconnect_ap()
			cmd.stop_softap()
	
	# power save mode
	
	# set and get power save mode
	if (STRESS_TEST & TEST_POWER_SAVE):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.set_wifi_power_save("max")
			cmd.get_wifi_power_save()
	
	# maximum transmitting power
	
	# set and get maximum transmitting power
	if (STRESS_TEST & TEST_WIFI_TX_POWER):
		for i in (range(STRESS_TEST_COUNT)):
			print("*************** "+str(i)+" ****************")
			cmd.set_wifi_max_tx_power(max_tx_pwr_input)
			cmd.get_wifi_curr_tx_power()
	
	process_deinit_control_lib(True)
except:
	process_deinit_control_lib(True)
	traceback.print_exc()
