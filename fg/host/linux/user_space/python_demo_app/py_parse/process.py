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
import os
from commands_lib import *
from hosted_py_header import *
import re
import subprocess
#from ctypes import byref
from py_parse import nw_helper_func
import traceback
os_ifdown_softap_cmd = "sudo ifconfig ethap0 down"

heartbeat_started = False


def process_is_param_missing(x):
	if not x:
		return True
	return False


def _is_mac_valid(x):
	return re.match("[0-9a-f]{2}([-:]?)[0-9a-f]{2}(\\1[0-9a-f]{2}){4}$", x.lower())


def _get_bool(x):
	if x == True or x == 'y' or x == 'yes' or x == '1':
		return True
	elif x == False or x == 'n' or x == 'no' or x == '0':
		return False
	else:
		return None


def process_init_control_lib():
	if (init_hosted_control_lib()):
		print("init hosted control lib failed")
		quit()

	print("------- ESP-Hosted slave FW [", end='')
	test_get_fw_version()
	print("] --------")
	process_get_mac_addr("station")
	process_get_mac_addr("softap")
	register_all_event_callbacks()
	if is_network_split_on() == True:
		if nw_helper_func.update_host_network_port_range(49152, 61439) != SUCCESS:
			print("Failed to update host network port range")
		if test_sync_get_static_ip_from_slave() != SUCCESS:
			print("Failed to fetch IP status")



def process_deinit_control_lib(stop_heartbeat = False):
	if stop_heartbeat == True:
		process_heartbeat(False)
	commands_map_py_to_c.deinit_hosted_control_lib()


def process_get_mode():
	test_sync_get_wifi_mode()
	return ""


def process_set_mode(mode):
	if mode == "station":
		test_sync_set_wifi_mode_station()
	elif mode == "softap":
		test_sync_set_wifi_mode_softap()
	elif mode == "station+softap":
		test_sync_set_wifi_mode_station_softap()
	elif mode == "none":
		test_sync_set_wifi_mode_none()
	else:
		return "Invalid mode"
	return ""


def process_get_mac_addr(mode):
	if mode == "station":
		if test_sync_station_mode_get_mac_addr():
			return "\nfailed to get station mac addr"

	elif mode == "softap":
		if test_sync_softap_mode_get_mac_addr():
			return "\nfailed to get softap mac addr"
	else:
		return "\nIncorrect Wi-Fi mode"

	mac = nw_helper_func.get_printable_mac_addr(mode)
	if mac:
		return mac
	else:
		return ""


def process_set_mac_addr(mode, mac):

	#verify mac
	if not _is_mac_valid(mac):
		return "\nInvalid MAC " + mac

	# verify mode
	if mode == "station":
		if test_sync_station_mode_set_mac_addr_of_esp(mac):
			return "\nfailed to set station mac addr"
	elif mode == "softap":
		if test_sync_softap_mode_set_mac_addr_of_esp(mac):
			return "\nfailed to set softap mac addr"
	else:
		return "\nIncorrect Wi-Fi mode"
	return ""


def process_get_available_wifi():
	test_sync_get_available_wifi()
	return ""


def process_connect_ap(ssid, pwd, bssid, use_wpa3, listen_interval, band_mode):
	ret_str = ""

	if band_mode < WIFI_BAND_MODE_2G_ONLY or band_mode > WIFI_BAND_MODE_AUTO:
		return "Invalid band_mode parameter " + get_str(band_mode)

	if test_sync_station_mode_connect(ssid, pwd, bssid, use_wpa3, listen_interval, band_mode) != SUCCESS:
		ret_str = "Failed to submit connect AP request"
		return ret_str

	ret_str = ""
	return ret_str


def process_get_connected_ap_info():
	test_sync_station_mode_get_info()
	return ""


def process_disconnect_ap(reset_dhcp):

	test_sync_station_mode_disconnect()
	print("\n")

	down_sta_netdev(network_info)
	return ""


def process_softap_vendor_ie(enable, data):
	if enable == "yes" or enable == 'y':
		if data == '':
			return 'Vendor IE data is expected while enabling'
		flag = True
	elif enable == "no" or enable == 'n':
		flag = False
	else:
		return 'enable takes values: ["yes" | "no"]'
	test_sync_set_vendor_specific_ie(flag, data)
	return ""


def process_start_softap(ssid, pwd, channel, sec_prot, max_conn, hide_ssid, bw, start_dhcp_server, band_mode):
	if sec_prot != "open" and pwd == "":
		return "password mandatory for security protocol"

	if sec_prot == "open":
		encr = WIFI_AUTH_MODE.WIFI_AUTH_OPEN.value
	elif sec_prot == "wpa_psk":
		encr = WIFI_AUTH_MODE.WIFI_AUTH_WPA_PSK.value
	elif sec_prot == "wpa2_psk":
		encr = WIFI_AUTH_MODE.WIFI_AUTH_WPA2_PSK.value
	elif sec_prot == "wpa_wpa2_psk":
		encr = WIFI_AUTH_MODE.WIFI_AUTH_WPA_WPA2_PSK.value
	else:
		return "Unsupported sec prot" + sec_prot

	if bw == 20:
		bw_l = WIFI_BW.WIFI_BW_HT20.value
	elif bw == 40:
		bw_l = WIFI_BW.WIFI_BW_HT40.value
	else:
		return "Unsupported bandwidth " + get_str(bw)

	if max_conn < 1 or max_conn > 10:
		return "max connections should be 1 to 10(hardware_max)"

	if band_mode < WIFI_BAND_MODE_2G_ONLY or band_mode > WIFI_BAND_MODE_AUTO:
		return "Invalid band_mode parameter " + str(band_mode) + ": value should be from " + str(WIFI_BAND_MODE_2G_ONLY) + " to " + str(WIFI_BAND_MODE_AUTO)

	if test_sync_softap_mode_start(ssid, pwd, channel, encr, max_conn, hide_ssid, bw_l, band_mode) != SUCCESS:
		ret_str = "Failed to start ESP softap"
		return ret_str
	print("\n")

	ret_str = "\nSoftAP started"
	return ret_str


def process_get_softap_info():
	test_sync_softap_mode_get_info()
	return ""


def process_softap_connected_clients_info():
	test_sync_softap_mode_connected_clients_info()
	return ""


def process_stop_softap():
	if test_sync_softap_mode_stop() != SUCCESS:
		ret_str = "Failed to stop SoftAP"
		return ret_str
	print()

	print(os_ifdown_softap_cmd)
	os.system(os_ifdown_softap_cmd)
	print()

	ret_str = "SoftAP stopped"
	return ret_str


def process_set_power_save(mode):
	try:
		if mode == "none":
			test_sync_set_wifi_power_save_mode_none()
		elif mode =="max":
			test_sync_set_wifi_power_save_mode_max()
		elif mode =="min":
			test_sync_set_wifi_power_save_mode_min()
		else:
			return "Unsupported power save mode " + mode
	except:
		traceback.print_exc()
	return ""


def process_get_power_save():
	try:
		test_sync_get_wifi_power_save_mode()
	except:
		traceback.print_exc()
	return ""


def process_set_wifi_max_tx_power(power):
	test_sync_wifi_set_max_tx_power(power)
	return ""


def process_wifi_curr_tx_power():
	test_sync_wifi_get_curr_tx_power()
	return ""

def process_enable_wifi():

	if test_feature_enable_wifi() != SUCCESS:
		return "Failed to enable Wi-Fi"

	if test_sync_station_mode_get_mac_addr() != SUCCESS:
		return "Failed to get station MAC address"

	if test_sync_softap_mode_get_mac_addr() != SUCCESS:
		return "Failed to get softap MAC address"

	return "\nSuccess. connect_ap now"

def process_disable_wifi():

	test_feature_disable_wifi()
	down_sta_netdev()
	down_softap_netdev()
	return "\nSuccess"

def process_enable_bluetooth():
	test_feature_enable_bt()
	reset_hci_instance()
	return ""

def process_disable_bluetooth():
	test_feature_disable_bt()
	down_hci_instance()
	return ""

def process_get_fw_version():
	test_get_fw_version()
	return ""

def process_get_country_code():
	test_get_country_code()
	return ""

def process_set_country_code(country, ieee80211d_enabled):
	test_set_country_code(country, ieee80211d_enabled)
	return ""

def process_ota_update(url):
	return test_sync_ota(url)


def process_heartbeat(enable, duration = 30):
	global heartbeat_started

	if None == _get_bool(enable):
		return "Unsupported enable value"
	if _get_bool(enable) == True:
		if duration < 10 or duration > 3600:
			return "Duration should be from 10 to 3600 seconds"
		heartbeat_started = True
		test_sync_config_heartbeat(True, duration)

	elif _get_bool(enable) == False:
		if heartbeat_started == True:
			test_sync_config_heartbeat(False, duration)
		heartbeat_started = False

	else:
		return "Unsupported enable value " + enable

	return ""


def process_subscribe_event(event):
	if event == 'esp_init':
		subscribe_event_esp_init()
		print("notifications enabled for esp_init")
	elif event == 'heartbeat':
		subscribe_event_heartbeat()
		print("notifications enabled for heartbeat")
	elif event == 'sta_connected':
		subscribe_event_sta_connected()
		print("notifications enabled for station connected to AP")
	elif event == 'sta_disconnected':
		subscribe_event_sta_disconnected()
		print("notifications enabled for station disconnection from AP")
	elif event == 'softap_sta_connected':
		subscribe_event_softap_sta_connected()
		print("notifications enabled for station connected to ESP softAP")
	elif event == 'softap_sta_disconnected':
		subscribe_event_softap_sta_disconnected()
		print("notifications enabled for station disconnection from ESP softAP")
	elif event == 'dhcp_dns_status':
		subscribe_event_dhcp_dns_status()
		print("notifications enabled for DHCP DNS status")
	elif event == 'custom_packed_event':
		subscribe_event_custom_packed_event()
		print("notifications enabled for custom RPC unserialised msg")
	elif event == 'all':
		register_all_event_callbacks()
		print("notifications enabled for all possible events")
	else:
		return "Unsupported event " + event
	return ""


def process_unsubscribe_event(event):
	if event == 'esp_init':
		unsubscribe_event_esp_init()
		print("notifications disabled for esp_init")
	elif event == 'heartbeat':
		unsubscribe_event_heartbeat()
		print("notifications disabled for heartbeat")
	elif event == 'sta_connected':
		unsubscribe_event_sta_connected()
		print("notifications disabled for station connected to AP (Although, not recommended)")
	elif event == 'sta_disconnected':
		unsubscribe_event_sta_disconnected()
		print("notifications disabled for station disconnection from AP (Although, not recommended)")
	elif event == 'softap_sta_connected':
		unsubscribe_event_softap_sta_connected()
		print("notifications disabled for station connected to ESP softAP")
	elif event == 'softap_sta_disconnected':
		unsubscribe_event_softap_sta_disconnected()
		print("notifications disabled for station disconnection from ESP softAP")
	elif event == 'dhcp_dns_status':
		unsubscribe_event_dhcp_dns_status()
		print("notifications disabled for DHCP DNS status (Although, not recommended)")
	elif event == 'custom_packed_event':
		unsubscribe_event_custom_packed_event()
		print("notifications disabled for custom RPC unserialised msg")
	elif event == 'all':
		unregister_all_event_callbacks()
		print("notifications disabled for all possible events")
		print("Although, we suggest enabling notifications for at least 'sta_connected', 'sta_disconnected', 'dhcp_dns_status'")
	else:
		return "Unsupported event " + event
	return ""

def process_custom_rpc_demo1():
	test_custom_rpc_demo1_request_only_ack()
	return ""

def process_custom_rpc_demo2():
	test_custom_rpc_demo2_request_echo_back_as_response()
	return ""

def process_custom_rpc_demo3():
	test_custom_rpc_demo3_request_echo_back_as_event()
	return ""
