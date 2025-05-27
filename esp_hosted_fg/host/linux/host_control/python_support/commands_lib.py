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

from hosted_py_header import *
import commands_map_py_to_c
from time import *
import requests
import sys
from py_parse import nw_helper_func
from ctypes import c_uint8, POINTER
import traceback

# import global
g_sta_network_info = nw_helper_func.g_sta_network_info
g_network_down_printed = False

WIFI_VENDOR_IE_ELEMENT_ID = 0xDD
OFFSET = 4
VENDOR_OUI_0 = 1
VENDOR_OUI_1 = 2
VENDOR_OUI_2 = 3
VENDOR_OUI_TYPE = 22

### Enable WiFi Feature needs to get the Mac Addr to configure the interface
### So we store it here after getting the Mac Addr
g_is_network_split_on = False
g_is_network_split_queried = False

# Define needed memmove function
def memmove(dest, src, count):
	"""Copy memory area from src to dest for count bytes.

	This is needed for data transfer in custom RPC.
	"""
	for i in range(count):
		dest[i] = src[i]
	return dest

# Define memset function
def memset(dest, value, count):
	"""Fill memory area with a constant byte.

	Args:
		dest: Pointer to destination memory area
		value: Value to be set
		count: Number of bytes to be set

	Returns:
		Pointer to the destination memory area
	"""
	if not dest or count <= 0:
		return dest

	# Cast to a byte array to be able to set values
	dest_bytes = cast(dest, POINTER(c_ubyte))
	for i in range(count):
		dest_bytes[i] = value

	return dest

# Helper to create event callback
def make_async_callback(py_callback):
	"""Convert a Python function to a callback function for ctypes.

	This is needed to set event callbacks.
	"""
	return CTRL_CB(py_callback)

# Get current event callback
def get_event_callback(event_id):
	"""Get the current event callback for the given event ID.

	This is helpful when we need to restore original handlers.

	Args:
		event_id: The event ID to get the callback for

	Returns:
		The current callback function or None if not set
	"""
	callback = commands_map_py_to_c.get_event_callback(event_id)
	return callback

def get_timestamp():
	tm = gmtime()
	tm_data = "{}-{}-{} {}:{}:{} > ".format(tm.tm_year,
			tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec)
	return tm_data



def mem_free(mem):
	if (mem) :
		commands_map_py_to_c.hosted_free(mem)
		mem = None



def close_sock(sockfd):
	ret = commands_map_py_to_c.close_socket(sockfd)
	if (ret < 0):
		print("Failure to close socket")



def bytes_to_int(b):
	return int.from_bytes(b, "big")



def cleanup_ctrl_msg(app_resp):
	if (app_resp):
		if (app_resp.contents.free_buffer_handle):
			if (app_resp.contents.free_buffer_func):
				app_resp.contents.free_buffer_func(app_resp.contents.free_buffer_handle)
				app_resp.contents.free_buffer_handle = None
		mem_free(app_resp)
		app_resp = None

def successful_response(app_resp):
	if app_resp and (bytes_to_int(app_resp.contents.resp_event_status) == SUCCESS):
		return True
	else:
		return False

def ctrl_app_event_callback(app_event):
	global g_sta_network_info
	global g_network_down_printed
	if (not app_event or
		not app_event.contents or
			bytes_to_int(app_event.contents.msg_type) != CTRL_MSGTYPE.CTRL_EVENT.value):
		if (app_event):
			print("Msg type is not event "+str(app_event.contents.msg_type))
		cleanup_ctrl_msg(app_event)
		return FAILURE

	if ((app_event.contents.msg_id <= CTRL_MSGID.CTRL_EVENT_BASE.value) or
		(app_event.contents.msg_id >= CTRL_MSGID.CTRL_EVENT_MAX.value)):
		print("Event Msg ID "+str(app_event.contents.msg_id)+" is not correct")
		cleanup_ctrl_msg(app_event)
		return FAILURE

	if app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_ESP_INIT.value:
		print("Event: ESP INIT")

	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_HEARTBEAT.value:
		print("Heartbeat event "+
				str(app_event.contents.control_data.e_heartbeat.hb_num))

	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_STATION_CONNECTED_TO_AP.value:
		ssid = app_event.contents.control_data.e_sta_conn.ssid
		ssid_len = app_event.contents.control_data.e_sta_conn.ssid_len
		bssid = app_event.contents.control_data.e_sta_conn.bssid
		channel = app_event.contents.control_data.e_sta_conn.channel
		authmode = app_event.contents.control_data.e_sta_conn.authmode
		aid = app_event.contents.control_data.e_sta_conn.aid

		print("Connected to ssid[" + get_str(ssid) + "] bssid [" +
			get_str(bssid) + "] channel [" + str(channel) + "] authmode [" + str(authmode) +
			"] aid [" + str(aid) + "]")
		if not nw_helper_func.is_valid_mac_bytes(g_sta_network_info.mac_addr):
			print(" MAC address " + nw_helper_func.mac_bytes_to_str(g_sta_network_info.mac_addr) + " is not valid, skipping up network interface")
			print("consider setting the MAC address using test_sync_set_mac_addr(mode, mac)")
			cleanup_ctrl_msg(app_event)
			return FAILURE

		nw_helper_func.up_sta_netdev();
		if g_network_down_printed:
			g_network_down_printed = False

		if not is_network_split_on():
			print("Network " + STA_INTERFACE + " is up -> Start dhcp client -> Check `ifconfig ethsta0` in new terminal")
			nw_helper_func.run_dhcp_on_connected()


	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_AP.value:
		ssid = app_event.contents.control_data.e_sta_disconn.ssid

		if not is_network_split_on():
			if not g_network_down_printed:
				print("Network " + STA_INTERFACE + " down -> Stop dhcp client")
			nw_helper_func.stop_dhclient_on_disconnected()

		if not g_network_down_printed:
			print(" Station disconnected from AP: ssid[" + get_str(ssid) + "]")
			g_network_down_printed = True

		nw_helper_func.down_sta_netdev();

	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_DHCP_DNS_STATUS.value:
		ip_addr = app_event.contents.control_data.e_dhcp_dns_status.dhcp_ip.decode('utf-8').rstrip('\x00')
		subnet_mask = app_event.contents.control_data.e_dhcp_dns_status.dhcp_nm.decode('utf-8').rstrip('\x00')
		gateway = app_event.contents.control_data.e_dhcp_dns_status.dhcp_gw.decode('utf-8').rstrip('\x00')
		dns_server = app_event.contents.control_data.e_dhcp_dns_status.dns_ip.decode('utf-8').rstrip('\x00')
		ifa = app_event.contents.control_data.e_dhcp_dns_status.iface
		net_link_up = app_event.contents.control_data.e_dhcp_dns_status.net_link_up
		dhcp_up = app_event.contents.control_data.e_dhcp_dns_status.dhcp_up
		dns_up = app_event.contents.control_data.e_dhcp_dns_status.dns_up
		dns_type = app_event.contents.control_data.e_dhcp_dns_status.dns_type

		#print("APP EVENT: DHCP DNS status: Network link [" + str(net_link_up) + "] IP [" +
		#	ip_addr + "] Subnet mask [" + subnet_mask + "] Gateway [" +
		#	gateway + "] DNS server [" + dns_server + "]" +
		#	" DHCP [" + str(dhcp_up) + "] DNS [" + str(dns_up) + "] DNS type [" + str(dns_type) + "]")

		if dhcp_up:
			g_sta_network_info.ip_addr = ip_addr
			g_sta_network_info.netmask = subnet_mask
			g_sta_network_info.gateway = gateway
			g_sta_network_info.ip_valid = 1
		else:
			g_sta_network_info.ip_valid = 0
			g_sta_network_info.dns_valid = 0

		if dns_up:
			g_sta_network_info.dns_addr = dns_server
			g_sta_network_info.dns_valid = 1
		else:
			g_sta_network_info.dns_valid = 0

		if net_link_up:
			g_sta_network_info.network_up = 1
		else:
			g_sta_network_info.network_up = 0


		if is_network_split_on():
			if not successful_response(app_event):
				print(" Slave firmware not compiled with network split, skip dhcp/dns event")
				cleanup_ctrl_msg(app_event)
				return FAILURE

			if nw_helper_func.is_valid_mac_bytes(g_sta_network_info.mac_addr):
				if g_sta_network_info.ip_valid and g_sta_network_info.network_up and g_sta_network_info.dns_valid:
					if nw_helper_func.up_sta_netdev__with_static_ip_dns_route(ip_addr, subnet_mask, gateway, dns_server) == FAILURE:
						print(" Failed to up static IP, gateway, DNS from event.")
					else:
						print(STA_INTERFACE +" up: IP [" + ip_addr + "] Subnet mask [" +
							subnet_mask + "] Gateway [" +gateway + "] DNS server [" + dns_server + "]")
				else:
					nw_helper_func.down_sta_netdev()
					if not g_network_down_printed:
						print("Network " + STA_INTERFACE + " brought down!")
						g_network_down_printed = True
			else:
				print("MAC address invalid, skip DHCP/DNS event.")
				print("consider setting the MAC address using test_sync_set_mac_addr(mode, mac)")
		else:
			print("DHCP/DNS event received when network split is off. Use dhclient if needed")


	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value:
		custom_msg_id = app_event.contents.control_data.custom_rpc_unserialised_msg.custom_msg_id
		custom_rpc_data = app_event.contents.control_data.custom_rpc_unserialised_msg.data
		custom_rpc_data_len = app_event.contents.control_data.custom_rpc_unserialised_msg.data_len
		print("Custom RPC event [" + str(custom_msg_id) + "] Data length [" + str(custom_rpc_data_len) + "].")
		print(" Data: " )

		for i in range(min(custom_rpc_data_len, 32)):
			print("{:02X} ".format(custom_rpc_data[i]), end='')
		if custom_rpc_data_len > 32:
			print(" ... ({} more bytes)".format(custom_rpc_data_len - 32), end='')

		print()
		print(" Use your own custom event handler to handle data.")
	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP.value:
		sta_mac = app_event.contents.control_data.e_softap_sta_conn.mac
		aid = app_event.contents.control_data.e_softap_sta_conn.aid
		print("Station connected to ESP SoftAP: MAC [" + get_str(sta_mac) + "] AID [" + str(aid) + "]")

	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP.value:
		sta_mac = app_event.contents.control_data.e_softap_sta_disconn.mac
		aid = app_event.contents.control_data.e_softap_sta_disconn.aid
		print("Station disconnected from ESP SoftAP: MAC [" + get_str(sta_mac) + "] AID [" + str(aid) + "]")
	else:
		print(" Invalid event ["+str(app_event.contents.msg_id)+"] to parse")

	cleanup_ctrl_msg(app_event)
	return SUCCESS

def process_failed_responses(app_msg):
	request_failed_flag = True
	if (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_REQ_IN_PROG.value):
		print("Err: Command In progress, Please wait")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_REQUEST_TIMEOUT.value):
		print("Err: Response Timeout")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_MEMORY_FAILURE.value):
		print("Err: Memory allocation failed")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_UNSUPPORTED_MSG.value):
		print("Err: Unsupported control msg")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_INCORRECT_ARG.value):
		print("Err: Invalid or out of range parameter values")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_PROTOBUF_ENCODE.value):
		print("Err: Protobuf encode failed")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_PROTOBUF_DECODE.value):
		print("Err: Protobuf decode failed")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_SET_ASYNC_CB.value):
		print("Err: Failed to set aync callback")
	elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_TRANSPORT_SEND.value):
		print("Err: Problem while serial driver write")
	else:
		request_failed_flag = False

	# if control request failed, no need to proceed for response checking
	if (request_failed_flag):
		return

	if (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_BEGIN.value):
		print("OTA failed in OTA begin")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_WRITE.value):
		print("OTA failed in OTA write")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_END.value):
		print("OTA failed in OTA end")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_CONNECT_AP.value):
		print("Failed to connect with AP, reason [" + str(app_msg.contents.resp_event_status) + "]")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_START_SOFTAP.value):
		print("Failed to start SoftAP")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_STOP_SOFTAP.value):
		print("Possibly softap is not running/started")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_SOFTAP_CONFIG.value):
		print("Possibly softap is not running/started")
	else:
		print("Failed Control Response")


def unregister_event_callbacks():
	ret = SUCCESS
	for event in range(CTRL_MSGID.CTRL_EVENT_BASE.value+1, CTRL_MSGID.CTRL_EVENT_MAX.value):
		if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(event)):
			print("reset event callback failed for event "+str(event))
			ret = FAILURE
	return ret


def CTRL_CMD_DEFAULT_REQ():
	"""Allocate and initialize a new control command request similar to C version.

	This function matches the C implementation by allocating memory for a new request.
	"""
	new_req = POINTER(CONTROL_COMMAND)()
	new_req = cast(commands_map_py_to_c.hosted_malloc(sizeof(CONTROL_COMMAND)), POINTER(CONTROL_COMMAND))
	if not new_req:
		print("Failed to allocate memory for request")
		return None

	# Initialize the allocated memory with zeros
	memset(new_req, 0, sizeof(CONTROL_COMMAND))

	# Set default values
	new_req.contents.msg_type = CTRL_MSGTYPE.CTRL_REQ.value
	new_req.contents.cmd_timeout_sec = CTRL_RESP_TIMEOUT_SEC

	return new_req

def fail_resp(app_resp) :
	cleanup_ctrl_msg(app_resp)
	return FAILURE


def finish_resp(app_resp) :
	cleanup_ctrl_msg(app_resp)
	return SUCCESS


def ctrl_app_resp_callback(app_resp):
	global g_network_down_printed

	if ((not app_resp) or (not app_resp.contents) or
		(bytes_to_int(app_resp.contents.msg_type) != CTRL_MSGTYPE.CTRL_RESP.value)):
		if (app_resp):
			if (app_resp.contents):
				print("Msg type is not response "+str(app_resp.contents.msg_type))
			fail_resp(app_resp)
		return FAILURE

	if ((app_resp.contents.msg_id <= CTRL_MSGID.CTRL_RESP_BASE.value) or
			(app_resp.contents.msg_id >= CTRL_MSGID.CTRL_RESP_MAX.value)):
		print("Msg ID "+str(app_resp.contents.msg_id)+" is not correct" )
		fail_resp(app_resp)
		return FAILURE

	if not successful_response(app_resp):
		process_failed_responses(app_resp)
		fail_resp(app_resp)
		return FAILURE


	if (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_MAC_ADDR.value) :
		mac_str = get_str(app_resp.contents.control_data.wifi_mac.mac)
		print("mac address is " + mac_str)

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_MAC_ADDRESS.value) :
		print("MAC address is set")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_WIFI_MODE.value) :
		if (app_resp.contents.control_data.wifi_mode.mode == WIFI_MODE_E.WIFI_MODE_STA.value) :
			print("wifi mode is : station")
		elif (app_resp.contents.control_data.wifi_mode.mode == WIFI_MODE_E.WIFI_MODE_AP.value) :
			print("wifi mode is : softap")
		elif (app_resp.contents.control_data.wifi_mode.mode == WIFI_MODE_E.WIFI_MODE_APSTA.value) :
			print("wifi mode is : station+softap")
		elif (app_resp.contents.control_data.wifi_mode.mode == WIFI_MODE_E.WIFI_MODE_NONE.value) :
			print("wifi mode is : none")
		else:
			print("wifi mode is : unknown")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_WIFI_MODE.value) :
		print("wifi mode is set")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_AP_SCAN_LIST.value) :
		w_scan_p = POINTER(WIFI_AP_SCAN_LIST)
		w_scan_p = pointer(app_resp.contents.control_data.wifi_ap_scan_list)
		list = POINTER(WIFI_SCAN_LIST)
		list = w_scan_p.contents.out_list

		if (not w_scan_p.contents.count) :
			print("No AP found")
			finish_resp()
			return FAILURE
		else:
			print("Current AP count is "+str(w_scan_p.contents.count))

		if (not list) :
			print("Failed to get scanned AP list")
			finish_resp()
			return FAILURE
		else:
			print("Number of available APs is "+str(w_scan_p.contents.count))
			for i in range (0, w_scan_p.contents.count) :
				print(str(i)+") ssid \""+list[i].ssid.decode('utf-8')+"\""+
						" bssid \""+get_str(list[i].bssid)+"\""+
						" rssi \""+str(list[i].rssi)+"\""+
						" channel \""+str(list[i].channel)+"\""+
						" auth mode \""+str(list[i].encryption_mode)+"\"")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_CONNECT_AP.value) :
		if not successful_response(app_resp):
			fail_resp(app_resp)
			print("Failed to connect to AP, retry")
			return FAILURE
		else:
			print("Request to connect to AP submitted")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_AP_CONFIG.value) :
		ap_config_p = POINTER(STA_CONFIG)
		ap_config_p = pointer(app_resp.contents.control_data.wifi_ap_config)

		if (get_str(ap_config_p.contents.status) == SUCCESS_STR) :
			print("AP's ssid \""+get_str(ap_config_p.contents.ssid)+"\"")
			print("AP's bssid \""+get_str(ap_config_p.contents.bssid)+"\"")
			print("AP's channel number \""+str(ap_config_p.contents.channel)+"\"")
			print("AP's rssi \""+str(ap_config_p.contents.rssi)+"\"")
			print("AP's encryption mode \""+str(ap_config_p.contents.encryption_mode)+"\"")
			print("AP's band mode \""+str(ap_config_p.contents.band_mode)+"\"")
		else:
			print("Station mode status: "+get_str(ap_config_p.contents.status))

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_DISCONNECT_AP.value) :
		print("Disconnected from AP")
		if nw_helper_func.down_sta_netdev() != SUCCESS:
			print("Failed to bring down station network")
			fail_resp(app_resp)
			return FAILURE
		else:
			if not g_network_down_printed:
				print("Station network brought down")
				g_network_down_printed = True

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_START_SOFTAP.value) :
		print("esp32 softAP started with band_mode "+str(app_resp.contents.control_data.wifi_softap_config.band_mode))
		if (nw_helper_func.up_softap_netdev() != SUCCESS):
			print("Failed to bring up softap network")
			fail_resp(app_resp)
			return FAILURE
		else:
			print("SoftAP started")
			if nw_helper_func.run_dhcp_server() != SUCCESS:
				print("Failed to start DHCP server")
				fail_resp(app_resp)
				return FAILURE

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_SOFTAP_CONFIG.value) :
		softap_config_p = POINTER(SOFTAP_CONFIG)
		softap_config_p = pointer(app_resp.contents.control_data.wifi_softap_config)
		print("softAP ssid \""+get_str(softap_config_p.contents.ssid)+"\"")
		print("softAP pwd \""+get_str(softap_config_p.contents.pwd)+"\"")
		print("softAP channel \""+str(softap_config_p.contents.channel)+"\"")
		print("softAP auth mode \""+str(softap_config_p.contents.encryption_mode)+"\"")
		print("softAP max connections \""+str(softap_config_p.contents.max_connections)+"\"")
		print("softAP hide ssid \""+str(softap_config_p.contents.ssid_hidden)+"\"")
		print("softAP bandwidth \""+str(softap_config_p.contents.bandwidth)+"\"")
		print("softAP band mode \""+str(softap_config_p.contents.band_mode)+"\"")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_SOFTAP_CONN_STA_LIST.value) :
		count = app_resp.contents.control_data.wifi_connected_stations_list.count
		stations_list = POINTER(WIFI_CONNECTED_STATIONS_LIST)
		stations_list = pointer(app_resp.contents.control_data.wifi_connected_stations_list.out_list)

		print("sta list count: "+str(count))
		if (not count):
			print("No station found")
			fail_resp(app_resp)
			return FAILURE

		if (not stations_list):
			print("Failed to get connected stations list")
		elif (count):
			for i in range(0,count):
				print(str(i)+"th station's bssid \""+get_str(stations_list[i].contents.bssid)+
						"\" rssi \""+str(stations_list[i].contents.rssi))

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_STOP_SOFTAP.value) :
		print("ESP32 softAP stopped")
		if nw_helper_func.down_softap_netdev() != SUCCESS:
			fail_resp(app_resp)
			print("Failed to bring down softap network")
			return FAILURE

		if nw_helper_func.stop_dhcp_server() != SUCCESS:
			print("Failed to stop DHCP server")
			fail_resp(app_resp)
			return FAILURE


	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_SOFTAP_VND_IE.value) :
		print("Success in set vendor specific ie")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_PS_MODE.value) :
		print("Wifi power save mode set")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_PS_MODE.value) :
		if (app_resp.contents.control_data.wifi_power_save_mode.ps_mode == WIFI_PS_MODE.WIFI_PS_NONE.value):
			print("Wifi power save mode is: none")
		elif (app_resp.contents.control_data.wifi_power_save_mode.ps_mode == WIFI_PS_MODE.WIFI_PS_MIN_MODEM.value):
			print("Wifi power save mode is: min")
		elif (app_resp.contents.control_data.wifi_power_save_mode.ps_mode == WIFI_PS_MODE.WIFI_PS_MAX_MODEM.value):
			print("Wifi power save mode is: max")
		else:
			print("Wifi power save mode is: Invalid")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_BEGIN.value) :
		print("ota begin ack")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_WRITE.value) :
		pass

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_END.value) :
		print("ota end ack")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_WIFI_MAX_TX_POWER.value) :
		print("Set wifi max tx power success")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_WIFI_CURR_TX_POWER.value) :
		print("wifi curr tx power : "+str(app_resp.contents.control_data.wifi_tx_power.power))

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_CONFIG_HEARTBEAT.value) :
		print("Heartbeat operation successful")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_FW_VERSION.value) :
		version_string = app_resp.contents.control_data.fw_version.project_name.decode('utf-8') + "-" + str(app_resp.contents.control_data.fw_version.major_1) + "." + str(app_resp.contents.control_data.fw_version.major_2) + "." + str(app_resp.contents.control_data.fw_version.minor) + "." + str(app_resp.contents.control_data.fw_version.revision_patch_1) + "." + str(app_resp.contents.control_data.fw_version.revision_patch_2)
		print(version_string, end='')

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_COUNTRY_CODE.value) :
		print("ack")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_COUNTRY_CODE.value) :
		country_code = app_resp.contents.control_data.country_code.country.decode('utf-8')
		print(country_code, end='')

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_ENABLE_DISABLE.value) :
		pass
	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_DHCP_DNS_STATUS.value) :
		print("set dhcp dns success")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_DHCP_DNS_STATUS.value) :
		#print("DHCP DNS status:")
		if app_resp.contents.control_data.e_dhcp_dns_status.iface == 0:
			iface = STA_INTERFACE
		else:
			iface = AP_INTERFACE


		ip_addr = app_resp.contents.control_data.e_dhcp_dns_status.dhcp_ip.decode('utf-8').rstrip('\x00')
		subnet_mask = app_resp.contents.control_data.e_dhcp_dns_status.dhcp_nm.decode('utf-8').rstrip('\x00')
		gateway = app_resp.contents.control_data.e_dhcp_dns_status.dhcp_gw.decode('utf-8').rstrip('\x00')
		dns_server = app_resp.contents.control_data.e_dhcp_dns_status.dns_ip.decode('utf-8').rstrip('\x00')
		ifa = app_resp.contents.control_data.e_dhcp_dns_status.iface
		net_link_up = app_resp.contents.control_data.e_dhcp_dns_status.net_link_up
		dhcp_up = app_resp.contents.control_data.e_dhcp_dns_status.dhcp_up
		dns_up = app_resp.contents.control_data.e_dhcp_dns_status.dns_up
		dns_type = app_resp.contents.control_data.e_dhcp_dns_status.dns_type

		if dhcp_up:
			g_sta_network_info.ip_addr = ip_addr
			g_sta_network_info.netmask = subnet_mask
			g_sta_network_info.gateway = gateway
			g_sta_network_info.ip_valid = 1
		else:
			g_sta_network_info.ip_valid = 0
			g_sta_network_info.dns_valid = 0

		if dns_up:
			g_sta_network_info.dns_addr = dns_server
			g_sta_network_info.dns_valid = 1
		else:
			g_sta_network_info.dns_valid = 0

		if net_link_up:
			g_sta_network_info.network_up = 1
		else:
			g_sta_network_info.network_up = 0


		if is_network_split_on():
			if not successful_response(app_resp):
				print(" Slave firmware not compiled with network split, skip dhcp/dns event")
				cleanup_ctrl_msg(app_resp)
				return FAILURE

			if nw_helper_func.is_valid_mac_bytes(g_sta_network_info.mac_addr):
				if g_sta_network_info.ip_valid and g_sta_network_info.network_up and g_sta_network_info.dns_valid:
					if nw_helper_func.up_sta_netdev__with_static_ip_dns_route(ip_addr, subnet_mask, gateway, dns_server) == FAILURE:
						print(" Failed to up static IP, gateway, DNS from event.")
					else:
						print(STA_INTERFACE +" up: IP [" + ip_addr + "] Subnet mask [" +
							subnet_mask + "] Gateway [" +gateway + "] DNS server [" + dns_server + "]")
				else:
					nw_helper_func.down_sta_netdev()
					if not g_network_down_printed:
						print("Network " + STA_INTERFACE + " brought down")
						g_network_down_printed = True
			else:
				print("MAC address invalid, skip DHCP/DNS event.")
				print("consider setting the MAC address using test_sync_set_mac_addr(mode, mac)")
		else:
			print("DHCP/DNS event received when network split is off. Use dhclient if needed")


	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_CUSTOM_RPC_UNSERIALISED_MSG.value) :
		custom_rpc_data = app_resp.contents.control_data.custom_rpc_unserialised_msg.data
		custom_rpc_data_len = app_resp.contents.control_data.custom_rpc_unserialised_msg.data_len
		print("Custom RPC unserialised msg: Data length [" + str(custom_rpc_data_len) + "]")
	else :
		print("Invalid Response "+ str(app_resp.contents.msg_id) +" to parse")

	cleanup_ctrl_msg(app_resp)
	return SUCCESS



ctrl_app_event_cb = CTRL_CB(ctrl_app_event_callback)
ctrl_app_resp_cb = CTRL_CB(ctrl_app_resp_callback)


def subscribe_event_esp_init():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_ESP_INIT.value, ctrl_app_event_cb)):
		print("event not subscribed for esp_init")
		return False
	return True



def subscribe_event_heartbeat():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_HEARTBEAT.value, ctrl_app_event_cb)):
		print("event not subscribed for heartbeat")
		return False
	return True



def subscribe_event_sta_connected_to_ap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_CONNECTED_TO_AP.value, ctrl_app_event_cb)):
		print("event not subscribed for station connected to AP")
		return False
	return True



def subscribe_event_sta_disconnect_from_ap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_AP.value, ctrl_app_event_cb)):
		print("event not subscribed for station disconnection from AP")
		return False
	return True



def subscribe_event_sta_connected_to_softap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP.value, ctrl_app_event_cb)):
		print("event not subscribed for station connected to ESP softAP")
		return False
	return True



def subscribe_event_sta_disconnect_from_softap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP.value, ctrl_app_event_cb)):
		print("event not subscribed for station disconnection from ESP softAP")
		return False
	return True

def subscribe_event_dhcp_dns_status():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_DHCP_DNS_STATUS.value, ctrl_app_event_cb)):
		print("event not subscribed for DHCP DNS status")
		return False
	return True

def subscribe_event_custom_rpc_unserialised_msg():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value, ctrl_app_event_cb)):
		print("event not subscribed for custom RPC unserialised msg")
		return False
	return True

def local_subscribe_event_custom_rpc_handler():
	"""Subscribe to custom RPC events.

	Returns:
		SUCCESS if successful, FAILURE otherwise
	"""
	ret = FAILURE

	ret = commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value,
		commands_map_py_to_c.make_async_callback(ctrl_app_event_callback))

	if ret != CALLBACK_SET_SUCCESS:
		print("Failed to subscribe to custom RPC events")
		return FAILURE
	else:
		print("Subscribed to custom RPC events successfully")
		return SUCCESS

def local_reset_event_custom_rpc_handler():
	"""Unsubscribe from custom RPC events.

	Returns:
		SUCCESS if successful, FAILURE otherwise
	"""
	ret = FAILURE

	ret = commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value)

	if ret != CALLBACK_SET_SUCCESS:
		print("Failed to unsubscribe from custom RPC events")
		return FAILURE
	else:
		print("Unsubscribed from custom RPC events successfully")
		return SUCCESS

def unsubscribe_event_esp_init():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_ESP_INIT.value)):
		print("stop subscription failed for esp_init")
		return False
	return True



def unsubscribe_event_heartbeat():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_HEARTBEAT.value)):
		print("stop subscription failed for heartbeat")
		return False
	return True



def unsubscribe_event_sta_connected_to_ap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_CONNECTED_TO_AP.value)):
		print("stop subscription failed for station connected to AP")
		return False
	return True



def unsubscribe_event_sta_disconnect_from_ap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_AP.value)):
		print("stop subscription failed for station disconnection from AP")
		return False
	return True



def unsubscribe_event_sta_connected_to_softap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP.value)):
		print("stop subscription failed for station connected to ESP softAP")
		return False
	return True



def unsubscribe_event_sta_disconnect_from_softap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP.value)):
		print("stop subscription failed for station disconnection from ESP softAP")
		return False
	return True

def unsubscribe_event_dhcp_dns_status():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_DHCP_DNS_STATUS.value)):
		print("stop subscription failed for DHCP DNS status")
		return False
	return True


def unsubscribe_event_custom_rpc_unserialised_msg():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value)):
		print("stop subscription failed for custom RPC unserialised msg")
		return False
	return True



def register_all_event_callbacks():
	subscribe_event_esp_init()
	subscribe_event_heartbeat()
	subscribe_event_sta_connected_to_ap()
	subscribe_event_sta_disconnect_from_ap()
	subscribe_event_sta_connected_to_softap()
	subscribe_event_sta_disconnect_from_softap()
	subscribe_event_dhcp_dns_status()
	subscribe_event_custom_rpc_unserialised_msg()


def unregister_all_event_callbacks():
	unsubscribe_event_esp_init()
	unsubscribe_event_heartbeat()
	unsubscribe_event_sta_connected_to_ap()
	unsubscribe_event_sta_disconnect_from_ap()
	unsubscribe_event_sta_connected_to_softap()
	unsubscribe_event_sta_disconnect_from_softap()
	unsubscribe_event_dhcp_dns_status()
	unsubscribe_event_custom_rpc_unserialised_msg()




def init_hosted_control_lib():
	ret = c_int()
	ret = commands_map_py_to_c.init_hosted_control_lib()
	return ret



def test_sync_set_wifi_mode(mode) :
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.contents.control_data.wifi_mode.mode = mode
	resp = commands_map_py_to_c.wifi_set_mode(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_set_wifi_mode_none() :
	return test_sync_set_wifi_mode(WIFI_MODE_E.WIFI_MODE_NONE.value)



def test_sync_set_wifi_mode_station() :
	return test_sync_set_wifi_mode(WIFI_MODE_E.WIFI_MODE_STA.value)



def test_sync_set_wifi_mode_softap() :
	return test_sync_set_wifi_mode(WIFI_MODE_E.WIFI_MODE_AP.value)



def test_sync_set_wifi_mode_station_softap() :
	return test_sync_set_wifi_mode(WIFI_MODE_E.WIFI_MODE_APSTA.value)



def test_async_get_wifi_mode() :
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.ctrl_resp_cb = ctrl_app_resp_cb

	commands_map_py_to_c.wifi_get_mode(req)

	cleanup_ctrl_msg(req)
	return SUCCESS



def test_sync_get_wifi_mode() :
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE

	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_mode(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)

def test_sync_get_static_ip_from_slave():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE

	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.get_dhcp_dns_status(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)

def test_sync_get_wifi_mac_addr(mode):
	if mode != WIFI_MODE_E.WIFI_MODE_STA.value and mode != WIFI_MODE_E.WIFI_MODE_AP.value:
		print("Invalid mode for getting MAC address")
		return FAILURE

	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.wifi_mac.mode = mode
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_mac(req)

	if not successful_response(resp):
		print("test_sync_get_wifi_mac_addr: Failed to get MAC address 1")
		cleanup_ctrl_msg(req)
		cleanup_ctrl_msg(resp)
		return FAILURE

	mac_bytes = resp.contents.control_data.wifi_mac.mac
	if not nw_helper_func.is_valid_mac_bytes(mac_bytes):
		print("test_sync_get_wifi_mac_addr: Failed to get MAC address 2")
		cleanup_ctrl_msg(req)
		cleanup_ctrl_msg(resp)
		return FAILURE

	#print("test_sync_get_wifi_mac_addr: mode: " + str(mode) + " MAC address: " + nw_helper_func.mac_bytes_to_str(mac_bytes))

	if req.contents.control_data.wifi_mac.mode == WIFI_MODE_E.WIFI_MODE_STA.value:
		nw_helper_func.set_mac_addr(mac_bytes, iface=STA_INTERFACE)
	else:
		nw_helper_func.set_mac_addr(mac_bytes, iface=AP_INTERFACE)

	cleanup_ctrl_msg(req)
	cleanup_ctrl_msg(resp)
	return SUCCESS



def test_sync_station_mode_get_mac_addr() :
	return test_sync_get_wifi_mac_addr(WIFI_MODE_E.WIFI_MODE_STA.value)



def test_sync_softap_mode_get_mac_addr() :
	return test_sync_get_wifi_mac_addr(WIFI_MODE_E.WIFI_MODE_AP.value)



def test_sync_set_mac_addr(mode, mac) :
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.wifi_mac.mode = mode
	req.contents.control_data.wifi_mac.mac = bytes(mac, 'utf-8')
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_set_mac(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_station_mode_set_mac_addr_of_esp(mac) :
	if test_sync_set_wifi_mode_station():
		print("Failed to set station wifi mode")
	return test_sync_set_mac_addr(WIFI_MODE_E.WIFI_MODE_STA.value, mac)



def test_sync_softap_mode_set_mac_addr_of_esp(mac) :
	if test_sync_set_wifi_mode_softap():
		print("Failed to set station wifi mode")
	return test_sync_set_mac_addr(WIFI_MODE_E.WIFI_MODE_AP.value, mac)



def test_sync_get_available_wifi():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_ap_scan_list(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_async_station_mode_connect(ssid,pwd,bssid,use_wpa3,listen_interval,band_mode):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE

	req.contents.control_data.wifi_ap_config.ssid = set_str(ssid)
	req.contents.control_data.wifi_ap_config.pwd = set_str(pwd)
	req.contents.control_data.wifi_ap_config.bssid = set_str(bssid)
	req.contents.control_data.wifi_ap_config.is_wpa3_supported = use_wpa3
	req.contents.control_data.wifi_ap_config.listen_interval = listen_interval
	req.contents.control_data.wifi_ap_config.band_mode = band_mode
	req.contents.ctrl_resp_cb = ctrl_app_resp_cb

	commands_map_py_to_c.wifi_connect_ap(req)

	cleanup_ctrl_msg(req)
	return SUCCESS



def test_sync_station_mode_connect(ssid,pwd,bssid,use_wpa3,listen_interval,band_mode):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.wifi_ap_config.ssid = set_str(str(ssid))
	req.contents.control_data.wifi_ap_config.pwd = set_str(str(pwd))
	req.contents.control_data.wifi_ap_config.bssid = set_str(bssid)
	req.contents.control_data.wifi_ap_config.is_wpa3_supported = use_wpa3
	req.contents.control_data.wifi_ap_config.listen_interval = listen_interval
	req.contents.control_data.wifi_ap_config.band_mode = band_mode

	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_connect_ap(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_station_mode_get_info():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_ap_config(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_station_mode_disconnect():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_disconnect_ap(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_start(ssid, pwd, channel, sec_prot, max_conn, hide_ssid, bw, band_mode):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.wifi_softap_config.ssid = set_str(ssid)
	req.contents.control_data.wifi_softap_config.pwd = set_str(str(pwd))
	req.contents.control_data.wifi_softap_config.channel = channel
	req.contents.control_data.wifi_softap_config.encryption_mode = sec_prot
	req.contents.control_data.wifi_softap_config.max_connections = max_conn
	req.contents.control_data.wifi_softap_config.ssid_hidden = hide_ssid
	req.contents.control_data.wifi_softap_config.bandwidth = bw
	req.contents.control_data.wifi_softap_config.band_mode = band_mode
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_start_softap(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_get_info():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_softap_config(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)


def test_sync_set_vendor_specific_ie(enable, data):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE

	# Allocate memory for the data
	v_data = create_string_buffer(data.encode(), len(data))
	if not v_data:
		cleanup_ctrl_msg(req)
		print("Failed to allocate memory for vendor data")
		return FAILURE

	print("Enable: " + str(enable) + " data: " + data)

	req.contents.control_data.wifi_softap_vendor_ie.enable = enable
	req.contents.control_data.wifi_softap_vendor_ie.type = WIFI_VND_IE_TYPE.WIFI_VND_IE_TYPE_BEACON.value
	req.contents.control_data.wifi_softap_vendor_ie.idx = WIFI_VND_IE_ID.WIFI_VND_IE_ID_0.value
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.length = len(data) + OFFSET
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.vendor_oui[0] = VENDOR_OUI_0
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.vendor_oui[1] = VENDOR_OUI_1
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.vendor_oui[2] = VENDOR_OUI_2
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.payload = cast(v_data, POINTER(c_uint8))
	req.contents.control_data.wifi_softap_vendor_ie.vnd_ie.payload_len = len(data)

	# No free like this for create_string_buffer
	#req.contents.free_buffer_func = commands_map_py_to_c.get_free_func()
	#req.contents.free_buffer_handle = cast(v_data, c_void_p)

	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_set_vendor_specific_ie(req)

	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_connected_clients_info():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_softap_connected_station_list(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_stop():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_stop_softap(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_set_wifi_power_save_mode(psmode):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.wifi_power_save_mode.ps_mode = psmode
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_set_power_save_mode(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_set_wifi_power_save_mode_none():
	return test_sync_set_wifi_power_save_mode(WIFI_PS_MODE.WIFI_PS_NONE.value)


def test_sync_set_wifi_power_save_mode_max():
	return test_sync_set_wifi_power_save_mode(WIFI_PS_MODE.WIFI_PS_MAX_MODEM.value)



def test_sync_set_wifi_power_save_mode_min():
	return test_sync_set_wifi_power_save_mode(WIFI_PS_MODE.WIFI_PS_MIN_MODEM.value)



def test_sync_get_wifi_power_save_mode():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_power_save_mode(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_wifi_set_max_tx_power(in_power):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.wifi_tx_power.power = in_power
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_set_max_tx_power(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_wifi_get_curr_tx_power():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_curr_tx_power(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)

def test_feature_config(feature, enable):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.feature_config.feature = feature
	req.contents.control_data.feature_config.enable = enable
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.feature_config(req)
	cleanup_ctrl_msg(req)

	if (feature == HOSTED_FEATURE.HOSTED_FEATURE_NETWORK_SPLIT.value):
		if successful_response(resp):
			ret = SUCCESS
		else:
			ret = FAILURE
		cleanup_ctrl_msg(resp)
		return ret
	else:
		return ctrl_app_resp_callback(resp)

def test_feature_enable_wifi():
	return test_feature_config(HOSTED_FEATURE.HOSTED_FEATURE_WIFI.value, YES)

def test_feature_disable_wifi():
	return test_feature_config(HOSTED_FEATURE.HOSTED_FEATURE_WIFI.value, NO)

def test_feature_enable_bt():
	return test_feature_config(HOSTED_FEATURE.HOSTED_FEATURE_BLUETOOTH.value, YES)

def test_feature_disable_bt():
	return test_feature_config(HOSTED_FEATURE.HOSTED_FEATURE_BLUETOOTH.value, NO)

def is_network_split_on():
	global g_is_network_split_on
	global g_is_network_split_queried
	if (g_is_network_split_queried == False):
		g_is_network_split_on = test_feature_config(HOSTED_FEATURE.HOSTED_FEATURE_NETWORK_SPLIT.value, YES) == SUCCESS
		g_is_network_split_queried = True

		if g_is_network_split_on == False:
			print("Network split is off")

	return g_is_network_split_on

def test_get_fw_version():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.get_fw_version(req)
	cleanup_ctrl_msg(req)
	if not resp:
		print("Failed to get firmware version")
		return FAILURE
	return ctrl_app_resp_callback(resp)

def test_set_country_code(country, ieee80211d_enabled):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.country_code.country = country.encode('utf-8')
	req.contents.control_data.country_code.ieee80211d_enabled = ieee80211d_enabled
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.set_country_code(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)

def test_get_country_code():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.get_country_code(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)

def test_sync_ota_begin():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.ota_begin(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_ota_write(ota_data, ota_data_len):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.ota_write.ota_data = ota_data
	req.contents.control_data.ota_write.ota_data_len = ota_data_len
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.ota_write(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_ota_end():
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.ota_end(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)



def test_sync_ota(image_URL):
	try:
		response = requests.get(image_URL, stream = True)
	except:
		return "Error while fetching URL"

	print("Starting OTA")

	ota_status = test_sync_ota_begin()
	if (ota_status == FAILURE):
		return "Failure in OTA begin"

	chunk_size = 4000
	if (chunk_size>4000):
		chunk_size = 4000

	for chunk in response.iter_content(chunk_size):
		print("|", end="", flush=True)
		ota_status = test_sync_ota_write(chunk, chunk_size)
		if (ota_status == FAILURE):
			ota_status = test_sync_ota_end()
			if (ota_status == FAILURE):
				return "Failure in OTA write->end"
			return "Failed to OTA write"
		print(".", end="", flush=True)

	ota_status = test_sync_ota_end()
	if (ota_status == FAILURE):
		return "Failure in OTA end"

	print("\n\nOTA Successful, ESP32 will restart in 5 sec")
	return ""



def test_sync_config_heartbeat(enable, duration):
	req = CTRL_CMD_DEFAULT_REQ()
	if not req:
		print("Failed to allocate memory for request")
		return FAILURE
	req.contents.control_data.e_heartbeat.enable = enable
	req.contents.control_data.e_heartbeat.duration = duration
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.config_heartbeat(req)
	cleanup_ctrl_msg(req)
	return ctrl_app_resp_callback(resp)


# Global variables for verification in demo3
g_original_data = None
g_original_data_len = 0
g_verification_result = 0  # 0=not verified, 1=success, -1=failure

# Function to set reference data for verification
def custom_rpc_set_verification_reference(data, data_len):
	global g_original_data, g_original_data_len, g_verification_result
	g_original_data = data
	g_original_data_len = data_len
	g_verification_result = 0  # Reset result

# Custom event handler to verify the data received in the event
def custom_rpc_event_handler(app_event):
	global g_original_data, g_original_data_len, g_verification_result

	if (not app_event or
		not app_event.contents or
			bytes_to_int(app_event.contents.msg_type) != CTRL_MSGTYPE.CTRL_EVENT.value):
		if (app_event):
			print("Msg type is not event "+str(app_event.contents.msg_type))
		cleanup_ctrl_msg(app_event)
		return FAILURE

	if app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value:
		p_e = app_event.contents.control_data.custom_rpc_unserialised_data

		# Process based on custom event ID
		if p_e.custom_msg_id == CUSTOM_RPC_EVENT_ID.DEMO_ECHO_BACK_REQUEST.value:
			print("[Demo 3] Received echo back event with " + str(p_e.data_len) + " bytes of data")

			# Verify the data against our stored reference
			if not g_original_data or not g_original_data_len:
				print(" [Demo 3] Verification reference data not set!")
				g_verification_result = -1
			elif g_original_data_len != p_e.data_len:
				print(" [Demo 3] Data length mismatch! Sent: " + str(g_original_data_len) +
					  ", Received: " + str(p_e.data_len))
				g_verification_result = -1
			else:
				# Compare byte by byte
				mismatch = False
				send_data = g_original_data
				recv_data = string_at(p_e.data, p_e.data_len)

				for i in range(p_e.data_len):
					#/* send_data[i] is a bytes object of length 1, so use send_data[i][0] */
					#/* recv_data is a bytes object, so recv_data[i] is int */
					if send_data[i][0] != recv_data[i]:
						print("[Demo 2] Data mismatch at byte " + str(i) +
							": sent 0x" + format(send_data[i][0], '02x') +
							", received 0x" + format(recv_data[i], '02x'))
						mismatch = True
						g_verification_result = -1
						break

				if not mismatch:
					print("[Demo 3] Verified success: (All Rx bytes in event) = (All Tx bytes in request)")
					g_verification_result = 1
		else:
			print(" [Demo 3] Unhandled custom RPC event ID [" + str(p_e.custom_msg_id) +
				  "] with data length: " + str(p_e.data_len) + " bytes")
	else:
		print(" Unhandled base RPC message: " + str(app_event.contents.msg_id))

	cleanup_ctrl_msg(app_event)
	return SUCCESS

def test_custom_rpc_demo3_request_echo_back_as_event():
	"""Send a custom RPC request and get echo back as event"""
	global g_verification_result

	# Register custom event handler
	original_handler = commands_map_py_to_c.get_event_callback(
		CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value)

	if not original_handler:
		print("[Demo 3] No original event handler registered for CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG")

	unsubscribe_event_custom_rpc_unserialised_msg()

	custom_rpc_event_cb = CTRL_CB(custom_rpc_event_handler)

	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value, custom_rpc_event_cb)):
		print("Problem while replacing event callback for custom RPC unserialised msg")
		return FAILURE

	print("[Demo 3] Sending Custom RPC request to slave. Expecting ack response. " +
		  "The data sent should be echoed back as a custom RPC event")

	# Create a byte stream with a specific pattern for verification
	send_data_len = 4000
	from ctypes import c_char
	send_data = (c_char * send_data_len)()

	for i in range(send_data_len):
		send_data[i] = i*3 % 256

	# Set reference data for verification in event handler
	custom_rpc_set_verification_reference(send_data, send_data_len)

	# Use a message ID that will trigger an event response
	custom_msg_id = CUSTOM_RPC_REQ_ID.ECHO_BACK_AS_EVENT.value

	status, recv_data, recv_data_len = FAILURE, None, 0

	status, recv_data, recv_data_len = test_custom_rpc_unserialised_request(
		custom_msg_id, send_data, send_data_len)

	if status != SUCCESS:
		print("[Demo 3] Failed to send custom RPC request")
		# Clean up resources
		if original_handler:
			commands_map_py_to_c.set_event_callback(
				CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value,
				original_handler)
		else:
			commands_map_py_to_c.reset_event_callback(
				CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value)
		return FAILURE

	# Wait for event to arrive
	print("[Demo 3] Waiting for echo event from slave...")

	# Wait for the event to arrive with a timeout
	verification_timeout = 5  # seconds
	start_time = time()
	while g_verification_result == 0 and (time() - start_time) < verification_timeout:
		sleep(0.1)  # Small sleep to avoid CPU spinning

	# Check verification result
	if g_verification_result == 1:
		print("[Demo 3] Verification successful!")
		ret = SUCCESS
	elif g_verification_result == -1:
		print("[Demo 3] Verification failed!")
		ret = FAILURE
	else:
		print("[Demo 3] Verification timeout - no event received within " +
			  str(verification_timeout) + " seconds")
		ret = FAILURE

	# Restore the original event handler and unsubscribe
	if original_handler:
		commands_map_py_to_c.set_event_callback(
			CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value,
			original_handler)
	else:
		commands_map_py_to_c.reset_event_callback(
			CTRL_MSGID.CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.value)
	unsubscribe_event_custom_rpc_unserialised_msg()

	# Reset global verification data
	custom_rpc_set_verification_reference(None, 0)

	print("[Demo 3] Demo completed!")

	return ret

def test_custom_rpc_unserialised_request(custom_msg_id, send_data, send_data_len):
	#/* Send a custom RPC request and process the response. */
	if not send_data or send_data_len <= 0:
		print("send data seem invalid, ignoring request")
		return FAILURE, None, 0

	app_req = CTRL_CMD_DEFAULT_REQ()
	if not app_req:
		print("Failed to allocate memory for request")
		return FAILURE, None, 0

	app_req.contents.msg_id = CTRL_MSGID.CTRL_REQ_CUSTOM_RPC_UNSERIALISED.value
	app_req.contents.control_data.custom_rpc_unserialised_data.custom_msg_id = custom_msg_id

	send_data_ptr = cast(send_data, c_void_p)
	app_req.contents.control_data.custom_rpc_unserialised_data.data_len = send_data_len
	app_req.contents.control_data.custom_rpc_unserialised_data.data = send_data_ptr

	resp = commands_map_py_to_c.send_custom_rpc_unserialised_req_to_slave(app_req)

	cleanup_ctrl_msg(app_req)
	if not resp:
		print("Failed to get response from slave")
		return FAILURE, None, 0

	status = FAILURE
	recv_data = None
	recv_data_len = 0

	if successful_response(resp):
		status = SUCCESS
		#print("success response for " + str(resp.contents.msg_id))
		if (resp.contents.msg_id != CTRL_MSGID.CTRL_RESP_CUSTOM_RPC_UNSERIALISED.value):
			print("response received, unexpected response msg id" + str(resp.contents.msg_id))
			cleanup_ctrl_msg(resp)
			return FAILURE, None, 0
	else:
		print("Failure response")
		cleanup_ctrl_msg(resp)
		return FAILURE, None, 0

	rx_custom_id = resp.contents.control_data.custom_rpc_unserialised_data.custom_msg_id
	data_ptr = resp.contents.control_data.custom_rpc_unserialised_data.data
	data_len = resp.contents.control_data.custom_rpc_unserialised_data.data_len

	if not data_ptr or data_len == 0:
		print(">> Rx Ack for custom_data_id: " + str(rx_custom_id))
	else:
		print(">> Rx Resp for custom_data_id: " + str(rx_custom_id) +
			" with data_len: "+ str(resp.contents.control_data.custom_rpc_unserialised_data.data_len))

	from ctypes import string_at
	recv_data = string_at(data_ptr, data_len)
	recv_data_len = data_len

	if data_ptr and resp.contents.free_buffer_func:
		free_func = resp.contents.free_buffer_func
		free_func(data_ptr)
		resp.contents.free_buffer_handle = None

	cleanup_ctrl_msg(resp)

	return status, recv_data, recv_data_len


def test_custom_rpc_demo1_request_only_ack():
	#/* Send a custom RPC request with only acknowledgement */
	print("[Demo 1] Sending packed RPC request. No response expected. Only Ack expected.")

	send_data_len = 4000
	from ctypes import c_char
	send_data = (c_char * send_data_len)()

	for i in range(send_data_len):
		send_data[i] = (i * 2) % 256

	custom_msg_id = CUSTOM_RPC_REQ_ID.ONLY_ACK.value

	status, recv_data, recv_data_len, recv_data_free_func = test_custom_rpc_unserialised_request(
		custom_msg_id, send_data, send_data_len)

	if status != SUCCESS:
		print("[Demo 1] Failed to send custom RPC request")
		return FAILURE

	print("[Demo 1] Slave Ack for RPC request received")
	return SUCCESS


def test_custom_rpc_demo2_request_echo_back_as_response():
	# /* Send a custom RPC request and get echo back as response */
	print("[Demo 2] Sending a demo byte buffer to slave, expecting echo back as response.")

	send_data_len = 4000
	from ctypes import c_char
	send_data = (c_char * send_data_len)()

	for i in range(send_data_len):
		send_data[i] = i % 256

	custom_msg_id = CUSTOM_RPC_REQ_ID.ECHO_BACK_RESPONSE.value

	status, recv_data, recv_data_len = FAILURE, None, 0

	try:
		status, recv_data, recv_data_len = test_custom_rpc_unserialised_request(
			custom_msg_id, send_data, send_data_len)
	except:
		traceback.print_exc()

	if status != SUCCESS:
		print("[Demo 2] Failed to send custom RPC request")
		return FAILURE

	print("[Demo 2] Received " + str(recv_data_len) + " bytes of data back as response")

	ret = SUCCESS

	try:
		if recv_data_len != send_data_len:
			print("[Demo 2] Data length mismatch! Sent: " + str(send_data_len) +
			", Received: " + str(recv_data_len))
			ret = FAILURE
		else:
			print("[Demo 2] Comparing received data with the one sent in request:")
			mismatch = False
			for i in range(send_data_len):
				#/* send_data[i] is a bytes object of length 1, so use send_data[i][0] */
				#/* recv_data is a bytes object, so recv_data[i] is int */
				if send_data[i][0] != recv_data[i]:
					print("[Demo 2] Data mismatch at byte " + str(i) +
						": sent 0x" + format(send_data[i][0], '02x') +
						", received 0x" + format(recv_data[i], '02x'))
					mismatch = True
					break

			if not mismatch:
				print("[Demo 2] Data verification successful - All " +
					str(send_data_len) + " TX bytes = All " + str(recv_data_len) + " RX bytes!")
			else:
				ret = FAILURE
	except:
		traceback.print_exc()

	return ret
