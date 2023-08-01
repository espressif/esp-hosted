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

WIFI_VENDOR_IE_ELEMENT_ID = 0xDD
OFFSET = 4
VENDOR_OUI_0 = 1
VENDOR_OUI_1 = 2
VENDOR_OUI_2 = 3
VENDOR_OUI_TYPE = 22

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



def ctrl_app_event_callback(app_event):
	ts = create_string_buffer(b"", MIN_TIMESTAMP_STR_SIZE)
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

	s = get_timestamp()
	if app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_ESP_INIT.value:
		print(s +" APP EVENT: ESP INIT")

	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_HEARTBEAT.value:
		print(s +" APP EVENT: Heartbeat event "+
				str(app_event.contents.control_data.e_heartbeat.hb_num))

	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_AP.value:
		print(s +" APP EVENT: Station mode: Disconnect Reason "+
				str(int.from_bytes(app_event.contents.resp_event_status, 'big')))

	elif app_event.contents.msg_id == CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP.value:
		p = app_event.contents.control_data.e_sta_disconnected.mac
		if p and len(p):
			print(s +" APP EVENT: SoftAP mode: Disconnect MAC ["+get_str(p)+"]")
	else:
		print(s+" Invalid event ["+str(app_event.contents.msg_id)+"] to parse")

	cleanup_ctrl_msg(app_event)
	return SUCCESS



def process_resp_connect_ap(app_msg) :
	ret = SUCCESS
	sockfd = c_int()
	sockfd.value = 0
	AF_INET = c_int()
	AF_INET.value = 2
	SOCK_DGRAM = c_int()
	SOCK_DGRAM.value = 2
	IPPROTO_IP = c_int()
	IPPROTO_IP.value = 0

	sta_interface = create_string_buffer(b"ethsta0", len(STA_INTERFACE))
	if (not len(app_msg.contents.control_data.wifi_ap_config.out_mac)):
		print("Failure: station mac is empty")
		return FAILURE

	ret = commands_map_py_to_c.create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, byref(sockfd))
	if (ret < 0):
		print("Failure to open socket")
		return FAILURE

	ret = commands_map_py_to_c.interface_down(sockfd, sta_interface)
	if (ret == SUCCESS) :
		print(STA_INTERFACE+" interface down")
	else:
		print("Unable to down "+STA_INTERFACE+" interface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.set_hw_addr(sockfd, sta_interface,
			app_msg.contents.control_data.wifi_ap_config.out_mac)
	if (ret == SUCCESS):
		mac_l = get_str(app_msg.contents.control_data.wifi_ap_config.out_mac)
		print("MAC address \""+ mac_l +"\" set to "+STA_INTERFACE+" interface")
	else:
		print("Unable to set MAC address to "+STA_INTERFACE+" interface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.interface_up(sockfd, sta_interface)
	if (ret == SUCCESS) :
		print(STA_INTERFACE+" interface up")
	else:
		print("Unable to up "+STA_INTERFACE+" interface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.close_socket(sockfd)
	if (ret < 0):
		print("Failure to close socket")
		return FAILURE

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
		if (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_NO_AP_FOUND.value):
			print("SSID : not found/connectable")
		elif (app_msg.contents.resp_event_status == CTRL_ERR.CTRL_ERR_INVALID_PASSWORD.value):
			print("Invalid password for SSID")
		else:
			print("Failed to connect with AP")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_START_SOFTAP.value):
		print("Failed to start SoftAP")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_STOP_SOFTAP.value):
		print("Possibly softap is not running/started")
	elif (app_msg.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_SOFTAP_CONFIG.value):
		print("Possibly softap is not running/started")
	else:
		print("Failed Control Response")



def process_resp_disconnect_ap(app_resp):
	ret = 0
	sockfd = c_int()
	sockfd.value = 0
	AF_INET = c_int()
	AF_INET.value = 2
	SOCK_DGRAM = c_int()
	SOCK_DGRAM.value = 2
	IPPROTO_IP = c_int()
	IPPROTO_IP.value = 0

	sta_interface = create_string_buffer(b"ethsta0", len(STA_INTERFACE))
	ret = commands_map_py_to_c.create_socket(AF_INET,
			SOCK_DGRAM, IPPROTO_IP, byref(sockfd))
	if (ret < 0):
		print("Failure to open socket")
		return FAILURE

	ret = commands_map_py_to_c.interface_down(sockfd, sta_interface)
	if (ret == SUCCESS):
		print(STA_INTERFACE + " interface down")
	else:
		print("Unable to down " + STA_INTERFACE + "inteface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.close_socket(sockfd)
	if (ret < 0):
		print("Failure to close socket")
		return FAILURE

	return SUCCESS



def process_resp_start_softap(app_resp):
	ret = SUCCESS
	sockfd = c_int()
	sockfd.value = 0
	AF_INET = c_int()
	AF_INET.value = 2
	SOCK_DGRAM = c_int()
	SOCK_DGRAM.value = 2
	IPPROTO_IP = c_int()
	IPPROTO_IP.value = 0

	ap_interface = create_string_buffer(b"ethap0",len(AP_INTERFACE))
	if (not len(app_resp.contents.control_data.wifi_softap_config.out_mac)):
		print("Failure: softap mac is empty")
		return FAILURE

	ret = commands_map_py_to_c.create_socket(AF_INET,
			SOCK_DGRAM, IPPROTO_IP, byref(sockfd))
	if (ret < 0):
		print("Failure to open socket")
		return FAILURE

	ret = commands_map_py_to_c.interface_down(sockfd, ap_interface)
	if (ret == SUCCESS) :
		print(AP_INTERFACE+" interface down")
	else:
		print("Unable to down "+AP_INTERFACE+" interface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.set_hw_addr(sockfd, ap_interface,
			app_resp.contents.control_data.wifi_softap_config.out_mac)
	if (ret == SUCCESS):
		mac_l = get_str(app_resp.contents.control_data.wifi_softap_config.out_mac)
		print("MAC address "+ mac_l +" set to "+AP_INTERFACE+" interface")
	else:
		print("Unable to set MAC address to "+AP_INTERFACE+" interface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.interface_up(sockfd, ap_interface)
	if (ret == SUCCESS) :
		print(AP_INTERFACE+" interface up")
	else:
		print("Unable to up "+AP_INTERFACE+" interface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.close_socket(sockfd)
	if (ret < 0):
		print("Failure to close socket")
		return FAILURE

	return SUCCESS



def process_resp_stop_softap(app_resp):
	ret = 0
	sockfd = c_int()
	sockfd.value = 0
	AF_INET = c_int()
	AF_INET.value = 2
	SOCK_DGRAM = c_int()
	SOCK_DGRAM.value = 2
	IPPROTO_IP = c_int()
	IPPROTO_IP.value = 0

	ap_interface = create_string_buffer(b"ethap0",len(AP_INTERFACE))
	ret = commands_map_py_to_c.create_socket(AF_INET,
			SOCK_DGRAM, IPPROTO_IP, byref(sockfd))
	if (ret < 0):
		print("Failure to open socket")
		return FAILURE

	ret = commands_map_py_to_c.interface_down(sockfd, ap_interface)
	if (ret == SUCCESS):
		print(AP_INTERFACE + " interface down")
	else:
		print("Unable to down " + AP_INTERFACE + "inteface")
		close_sock(sockfd)
		return FAILURE

	ret = commands_map_py_to_c.close_socket(sockfd)
	if (ret < 0):
		print("Failure to close socket")
		return FAILURE
	return SUCCESS



def unregister_event_callbacks():
	ret = SUCCESS
	for event in range(CTRL_MSGID.CTRL_EVENT_BASE.value+1, CTRL_MSGID.CTRL_EVENT_MAX.value):
		if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(event)):
			print("reset event callback failed for event "+str(event))
			ret = FAILURE
	return ret



def CTRL_CMD_DEFAULT_REQ(req):
	req.msg_type = CTRL_MSGTYPE.CTRL_REQ.value
	req.cmd_timeout_sec = CTRL_RESP_TIMEOUT_SEC



def fail_resp(app_resp) :
	cleanup_ctrl_msg(app_resp)
	return FAILURE



def finish_resp(app_resp) :
	cleanup_ctrl_msg(app_resp)
	return SUCCESS



def ctrl_app_resp_callback(app_resp):

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

	if (bytes_to_int(app_resp.contents.resp_event_status) != SUCCESS):
		process_failed_responses(app_resp)
		fail_resp(app_resp)
		return FAILURE


	if (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_MAC_ADDR.value) :
		print("mac address is "+ get_str(app_resp.contents.control_data.wifi_mac.mac))

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
		w_scan_p = pointer(app_resp.contents.control_data.wifi_ap_scan)
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
				print(str(i)+") ssid \""+get_str(list[i].ssid)+"\""+
						" bssid \""+get_str(list[i].bssid)+"\""+
						" rssi \""+str(list[i].rssi)+"\""+
						" channel \""+str(list[i].channel)+"\""+
						" auth mode \""+str(list[i].encryption_mode)+"\"")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_CONNECT_AP.value) :
		if (process_resp_connect_ap(app_resp)):
			fail_resp(app_resp)
			print("Returning failure")
			return FAILURE

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_AP_CONFIG.value) :
		ap_config_p = POINTER(STA_CONFIG)
		ap_config_p = pointer(app_resp.contents.control_data.wifi_ap_config)

		if (get_str(ap_config_p.contents.status) == SUCCESS_STR) :
			print("AP's ssid \""+get_str(ap_config_p.contents.ssid)+"\"")
			print("AP's bssid \""+get_str(ap_config_p.contents.bssid)+"\"")
			print("AP's channel number \""+str(ap_config_p.contents.channel)+"\"")
			print("AP's rssi \""+str(ap_config_p.contents.rssi)+"\"")
			print("AP's encryption mode \""+str(ap_config_p.contents.encryption_mode)+"\"")
		else:
			print("Station mode status: "+get_str(ap_config_p.contents.status))

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_DISCONNECT_AP.value) :
		print("Disconnected from AP")
		if (process_resp_disconnect_ap(app_resp)):
			fail_resp(app_resp)
			return FAILURE

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_START_SOFTAP.value) :
		print("esp32 softAP started")
		if (process_resp_start_softap(app_resp)):
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

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_SOFTAP_CONN_STA_LIST.value) :
		count = app_resp.contents.control_data.wifi_softap_con_sta.count
		stations_list = POINTER(WIFI_CONNECTED_STATIONS_LIST)
		stations_list = pointer(app_resp.contents.control_data.wifi_softap_con_sta.out_list)

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
		if (process_resp_stop_softap(app_resp)):
			fail_resp(app_resp)
			return FAILURE

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_SOFTAP_VND_IE.value) :
		print("Success in set vendor specific ie")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_PS_MODE.value) :
		print("Wifi power save mode set")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_PS_MODE.value) :
		if (app_resp.contents.control_data.wifi_ps.ps_mode == WIFI_PS_MODE.WIFI_PS_MIN_MODEM.value):
			print("Wifi power save mode is: min")
		elif (app_resp.contents.control_data.wifi_ps.ps_mode == WIFI_PS_MODE.WIFI_PS_MAX_MODEM.value):
			print("Wifi power save mode is: max")
		else:
			print("Wifi power save mode is: Invalid")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_BEGIN.value) :
		pass

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_WRITE.value) :
		pass

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_OTA_END.value) :
		pass

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_SET_WIFI_MAX_TX_POWER.value) :
		print("Set wifi max tx power success")

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_GET_WIFI_CURR_TX_POWER.value) :
		print("wifi curr tx power : "+str(app_resp.contents.control_data.wifi_tx_power.power))

	elif (app_resp.contents.msg_id == CTRL_MSGID.CTRL_RESP_CONFIG_HEARTBEAT.value) :
		print("Heartbeat operation successful")

	else :
		print("Invalid Response "+ str(app_resp.contents.msg_id) +" to parse")

	return SUCCESS



ctrl_app_event_cb = CTRL_CB(ctrl_app_event_callback)
ctrl_app_resp_cb = CTRL_CB(ctrl_app_resp_callback)


def subscribe_event_esp_init():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_ESP_INIT.value, ctrl_app_event_cb)):
		print("event not subscribed for esp_init")
	print("notifications enabled for esp_init")



def subscribe_event_heartbeat():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_HEARTBEAT.value, ctrl_app_event_cb)):
		print("event not subscribed for heartbeat")
	print("notifications enabled for heartbeat")



def subscribe_event_sta_disconnect_from_ap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_AP.value, ctrl_app_event_cb)):
		print("event not subscribed for station disconnection from AP")
	print("notifications enabled for station disconnection from AP")



def subscribe_event_sta_disconnect_from_softap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP.value, ctrl_app_event_cb)):
		print("event not subscribed for station disconnection from ESP softAP")
	print("notifications enabled for station disconnection from ESP softAP")



def unsubscribe_event_esp_init():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_ESP_INIT.value)):
		print("stop subscription failed for esp_init")
	print("notifications disabled for esp_init")



def unsubscribe_event_heartbeat():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_HEARTBEAT.value)):
		print("stop subscription failed for heartbeat")
	print("notifications disabled for heartbeat")



def unsubscribe_event_sta_disconnect_from_ap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_AP.value)):
		print("stop subscription failed for station disconnection from AP")
	print("notifications disabled for station disconnection from AP")



def unsubscribe_event_sta_disconnect_from_softap():
	if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.reset_event_callback(
		CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP.value)):
		print("stop subscription failed for station disconnection from ESP softAP")
	print("notifications disabled for station disconnection from ESP softAP")



def register_event_callbacks():
	ret = SUCCESS
	evt = 0
	events = []

	ESP_INIT = c_int()
	ESP_INIT.value = CTRL_MSGID.CTRL_EVENT_ESP_INIT.value

	HEARTBEAT = c_int()
	HEARTBEAT.value = CTRL_MSGID.CTRL_EVENT_HEARTBEAT.value

	STATION_DISCONNECT_FROM_AP = c_int()
	STATION_DISCONNECT_FROM_AP.value = \
                CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_AP.value

	STATION_DISCONNECT_FROM_ESP_SOFTAP = c_int()
	STATION_DISCONNECT_FROM_ESP_SOFTAP.value = \
                CTRL_MSGID.CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP.value

	events.append(EVENT_CALLBACK_TABLE_T(ESP_INIT, ctrl_app_event_cb))
	events.append(EVENT_CALLBACK_TABLE_T(HEARTBEAT, ctrl_app_event_cb))
	events.append(EVENT_CALLBACK_TABLE_T(STATION_DISCONNECT_FROM_AP, ctrl_app_event_cb))
	events.append(EVENT_CALLBACK_TABLE_T(STATION_DISCONNECT_FROM_ESP_SOFTAP, ctrl_app_event_cb))

	for i in range(0, len(events)):
		if (CALLBACK_SET_SUCCESS != commands_map_py_to_c.set_event_callback(events[i].event,
				events[i].fun)):
			print("event callback register failed for event "+str(events[i].event))
			ret = FAILURE
			break
	return ret



def init_hosted_control_lib():
	ret = c_int()
	ret = commands_map_py_to_c.init_hosted_control_lib()
	return ret



def test_sync_set_wifi_mode(mode) :
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.wifi_mode.mode = mode
	resp = commands_map_py_to_c.wifi_set_mode(req)
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
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	req.ctrl_resp_cb = ctrl_app_resp_cb

	commands_map_py_to_c.wifi_get_mode(req)
	return SUCCESS



def test_sync_get_wifi_mode() :
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)

	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_mode(req)
	return ctrl_app_resp_callback(resp)



def test_sync_get_wifi_mac_addr(mode) :
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.wifi_mac.mode = mode
	resp = commands_map_py_to_c.wifi_get_mac(req)
	return ctrl_app_resp_callback(resp)



def test_sync_station_mode_get_mac_addr() :
	return test_sync_get_wifi_mac_addr(WIFI_MODE_E.WIFI_MODE_STA.value)



def test_sync_softap_mode_get_mac_addr() :
	return test_sync_get_wifi_mac_addr(WIFI_MODE_E.WIFI_MODE_AP.value)



def test_sync_set_mac_addr(mode, mac) :
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.wifi_mac.mode = mode
	req.control_data.wifi_mac.mac = bytes(mac, 'utf-8')
	resp = commands_map_py_to_c.wifi_set_mac(req)
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
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_ap_scan_list(req)
	return ctrl_app_resp_callback(resp)



def test_async_station_mode_connect(ssid,pwd,bssid,use_wpa3,listen_interval):
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)

	req.control_data.wifi_ap_config.ssid = set_str(ssid)
	req.control_data.wifi_ap_config.pwd = set_str(pwd)
	req.control_data.wifi_ap_config.bssid = set_str(bssid)
	req.control_data.wifi_ap_config.is_wpa3_supported =  use_wpa3
	req.control_data.wifi_ap_config.listen_interval = listen_interval
	req.ctrl_resp_cb = ctrl_app_resp_cb

	commands_map_py_to_c.wifi_connect_ap(req)
	return SUCCESS



def test_sync_station_mode_connect(ssid,pwd,bssid,use_wpa3,listen_interval):
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	req.control_data.wifi_ap_config.ssid = set_str(str(ssid))
	req.control_data.wifi_ap_config.pwd = set_str(str(pwd))
	req.control_data.wifi_ap_config.bssid = set_str(bssid)
	req.control_data.wifi_ap_config.is_wpa3_supported =  use_wpa3
	req.control_data.wifi_ap_config.listen_interval = listen_interval

	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_connect_ap(req)

	return ctrl_app_resp_callback(resp)



def test_sync_station_mode_get_info():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_ap_config(req)
	return ctrl_app_resp_callback(resp)



def test_sync_station_mode_disconnect():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_disconnect_ap(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_start(ssid, pwd, channel, sec_prot, max_conn, hide_ssid, bw):
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.wifi_softap_config.ssid = set_str(ssid)
	req.control_data.wifi_softap_config.pwd = set_str(pwd)
	req.control_data.wifi_softap_config.channel = channel
	req.control_data.wifi_softap_config.encryption_mode = sec_prot
	req.control_data.wifi_softap_config.max_connections = max_conn
	req.control_data.wifi_softap_config.ssid_hidden = hide_ssid
	req.control_data.wifi_softap_config.bandwidth = bw
	resp = commands_map_py_to_c.wifi_start_softap(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_get_info():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_softap_config(req)
	return ctrl_app_resp_callback(resp)



def test_sync_set_vendor_specific_ie(enable,data):
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None

	vendor_oui = bytearray()
	vendor_oui.append(VENDOR_OUI_0)
	vendor_oui.append(VENDOR_OUI_1)
	vendor_oui.append(VENDOR_OUI_2)
	vnd_ie_t = create_string_buffer(b"", len(data) + 1)
	vnd_ie_t.value = set_str(data)
	print("Enable: " + str(enable) + " data: " + data)

	req.control_data.wifi_softap_vendor_ie.enable = enable
	req.control_data.wifi_softap_vendor_ie.type = WIFI_VND_IE_TYPE.WIFI_VND_IE_TYPE_BEACON.value
	req.control_data.wifi_softap_vendor_ie.idx = WIFI_VND_IE_ID.WIFI_VND_IE_ID_0.value
	req.control_data.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID
	req.control_data.wifi_softap_vendor_ie.vnd_ie.length = len(data) + 1 + OFFSET
	req.control_data.wifi_softap_vendor_ie.vnd_ie.vendor_oui = b'\x01\x02\x03'
	req.control_data.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE
	req.control_data.wifi_softap_vendor_ie.vnd_ie.payload = cast(vnd_ie_t, c_wchar_p)

	req.control_data.wifi_softap_vendor_ie.vnd_ie.payload_len = len(data)

	resp = commands_map_py_to_c.wifi_set_vendor_specific_ie(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_connected_clients_info():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_softap_connected_station_list(req)
	return ctrl_app_resp_callback(resp)



def test_sync_softap_mode_stop():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_stop_softap(req)
	return ctrl_app_resp_callback(resp)



def test_sync_set_wifi_power_save_mode(psmode):
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.wifi_ps.ps_mode = psmode
	resp = commands_map_py_to_c.wifi_set_power_save_mode(req)
	return ctrl_app_resp_callback(resp)



def test_sync_set_wifi_power_save_mode_max():
	return test_sync_set_wifi_power_save_mode(WIFI_PS_MODE.WIFI_PS_MAX_MODEM.value)



def test_sync_set_wifi_power_save_mode_min():
	return test_sync_set_wifi_power_save_mode(WIFI_PS_MODE.WIFI_PS_MIN_MODEM.value)



def test_sync_get_wifi_power_save_mode():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_power_save_mode(req)
	return ctrl_app_resp_callback(resp)



def test_sync_wifi_set_max_tx_power(in_power):
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.wifi_tx_power.power = in_power
	resp = commands_map_py_to_c.wifi_set_max_tx_power(req)
	return ctrl_app_resp_callback(resp)



def test_sync_wifi_get_curr_tx_power():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.wifi_get_curr_tx_power(req)
	return ctrl_app_resp_callback(resp)



def test_sync_ota_begin():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.ota_begin(req)
	return ctrl_app_resp_callback(resp)



def test_sync_ota_write(ota_data, ota_data_len):
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.ota_write.ota_data = ota_data
	req.control_data.ota_write.ota_data_len = ota_data_len
	resp = commands_map_py_to_c.ota_write(req)
	return ctrl_app_resp_callback(resp)



def test_sync_ota_end():
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	resp = commands_map_py_to_c.ota_end(req)
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
	req = CONTROL_COMMAND()
	CTRL_CMD_DEFAULT_REQ(req)
	resp = POINTER(CONTROL_COMMAND)
	resp = None
	req.control_data.e_heartbeat.enable = enable
	req.control_data.e_heartbeat.duration = duration
	resp = commands_map_py_to_c.config_heartbeat(req)
	return ctrl_app_resp_callback(resp)
