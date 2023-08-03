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

from py_parse.process import *


class ctrl_cmd(object):
	def __init__(self):
		self.out = ""

	def __str__(self):
		return self.out


	def wifi_get_mode(self):
		"""Get Wi-Fi mode

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		self.out = process_get_mode()
		return self


	def wifi_set_mode(self, mode : str):
		"""Set Wi-Fi mode

		Args:
			mode(str, mandatory): M | Values: ['station' | 'softap' | 'station+softap']

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		if process_is_param_missing(mode):
			self.out = "Missing param " + "--mode"
			return self

		self.out = process_set_mode(mode)
		return self


	def wifi_get_mac(self, mode : str):
		"""Get MAC address for mode passed

		Args:
			mode(int, mandatory): M | Values: ['station' | 'softap']

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		self.out = process_get_mac_addr(mode)
		return self


	def wifi_set_mac(self, mode : str = "", mac : str = ""):
		"""Set MAC address for mode passed

		Args:
			mode(str, mandatory): M | Values: ['station' | 'softap']
			mac(str, mandatory): M | Mac address

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(mode):
			self.out = "Missing param " + "--mode"
			return self
		if process_is_param_missing(mac):
			self.out = "Missing param " + "--mac"
			return self

		self.out = process_set_mac_addr(mode, mac)
		return self


	def get_available_ap(self):
		"""Get neighboring AP (Wi-Fi routers or hotspots)

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_get_available_wifi()
		return self


	def connect_ap(self, ssid : str = "", pwd : str = "", bssid : str = "", use_wpa3 : bool = False, listen_interval : int = 3, set_dhcp : bool = True):
		"""Connect to AP (Wi-Fi router or hotspot)

		Args:
			ssid (str, mandatory): M | SSID of AP (Wi-Fi router)
			pwd(str, optional): O | Password of AP (Wi-Fi router) to connect to
			bssid(str, optional): O | MAC addr of AP (useful when multiple AP have same SSID) | Default: ''
			use_wpa3(bool, optional): O | Use wpa3 security protocol | Default: False
			listen_interval(int, optional) : O | Number of AP beacons station will sleep | Default:3
			set_dhcp(bool, optional): O | Request DHCP | Default: True

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(ssid):
			self.out = "Missing param " + "--ssid"
			return self

		self.out = process_connect_ap(ssid, pwd, bssid, use_wpa3, listen_interval, set_dhcp)
		return self


	def get_connected_ap_info(self):
		"""Get info of connected AP (Wi-Fi router or hotspot)

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_get_connected_ap_info()
		return self


	def disconnect_ap(self, reset_dhcp : bool = True):
		"""Disconnect from AP (Wi-Fi router or hotspot)

		Args:
			reset_dhcp(bool, optional): O | Clean DHCP DHCP | Default: True

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_disconnect_ap(reset_dhcp)
		return self


	def softap_vendor_ie(self, enable : bool = "True", data: str = ""):
		"""Set vendor specific IE in softap beacon
		   Once vendor IE is set, it cannot be set again unless it is reset first

		Args:
			enable(bool, mandatory): M | Values ['yes' | 'no'] Set or Reset Vendor IE
			data(bool, optional): O | String to set in softap Wi-Fi broadcast beacon | Default: ''

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(enable):
			self.out = "Missing param " + "--enable"
			return self

		self.out = process_softap_vendor_ie(enable,data)
		return self


	def start_softap(self, ssid : str = "", pwd : str = "", channel : int = 1, sec_prot: str = "wpa_wpa2_psk", max_conn: int = 4, hide_ssid: bool = False, bw : int = 20):
		"""Connect to AP (Wi-Fi router or hotspot)

		Args:
			ssid (str, mandatory): M | SSID to configure ESP softAP
			pwd(str, mandatory): M | Password to configure ESP softAP
			channel(int, optional): O | Wi-Fi channel [ 1 to 11] | Default: 1
			sec_prot(str, optional): O | Security Protocol or authentication protocol ['open' | 'wpa_psk' | 'wpa2_psk' | 'wpa_wpa2_psk'] | Default: 'wpa_wpa2_psk'
			max_conn(int, optional) : O | Max num of stations that can connect | Default:4
			hide_ssid(bool, optional): O | Hide SSID broadcasting [ True | False ] | Default: False
			bw(int, optional): O | Wi-Fi Bandwidth [ 20 | 40 ] | Default: 20

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(ssid):
			self.out = "Missing param " + "--ssid"
			return self
		if process_is_param_missing(pwd):
			self.out = "Missing param " + "--pwd"
			return self

		self.out = process_start_softap(ssid, pwd, channel, sec_prot, max_conn, hide_ssid, bw)
		return self


	def get_softap_info(self):
		"""Get info of ESP softAP

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_get_softap_info()
		return self


	def softap_connected_clients_info(self):
		"""Get stations info which are connected to ESP softAP

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_softap_connected_clients_info()
		return self


	def stop_softap(self):
		"""Stop ESP softAP

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_stop_softap()
		return self


	def set_wifi_power_save(self, mode : str = "max"):
		"""Set Wi-Fi power save

		Args:
			mode(str,optional): O | power save mode ['max','min'] | Default: 'max'

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_set_power_save(mode)
		return self


	def get_wifi_power_save(self):
		"""Get current Wi-Fi power save

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_get_power_save()
		return self


	def set_wifi_max_tx_power(self, map_val : int = 0):
		"""Set Wi-Fi maximum TX power
		Please note this is just request, firmware may set maximum possible from input power

		Args:
			map_val(int,mandatory): M | Set Wi-Fi max power in map value

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(map_val):
			self.out = "Missing param " + "--map_val"
			return self

		self.out = process_set_wifi_max_tx_power(map_val)
		return self


	def get_wifi_curr_tx_power(self):
		"""Get current Wi-Fi TX power

		Args:
			no_arg(str,optional): O | Dummy arg

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_wifi_curr_tx_power()
		return self


	def ota_update(self, url : str = ""):
		"""OTA update with HTTP link

		Args:
			url(str,mandatory): M | URL of ESP firmware binary 'network_adapter.bin'

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(url):
			self.out = "Missing param " + "--url"
			return self

		self.out = process_ota_update(url)
		return self


	def heartbeat(self, enable: bool = True, duration: int = 30):
		"""Configure Heartbeat

		Args:
			enable(bool,optional): O | Values ['True' | 'False'] | Default: True
			duration(int,optional): O | Heartbeat duration in sec | Default: 30

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""
		self.out = process_heartbeat(enable, duration)
		return self


	def subscribe_event(self, event: str = ""):
		"""Subscribe event to get notifications

		Args:
			event(str,mandatory): M | Values ['esp_init' | 'heartbeat' | 'sta_disconnect_from_ap' | 'sta_disconnect_from_softap']

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(event):
			self.out = "Missing param " + "--event"
			return self

		self.out = process_subscribe_event(event)
		return self


	def unsubscribe_event(self, event: str = ""):
		"""Unsubscribe event to get notifications

		Args:
			event(str,mandatory): M | Values ['esp_init' | 'heartbeat' | 'sta_disconnect_from_ap' | 'sta_disconnect_from_softap']

		Returns:
			ctrl_cmd: ctrl_cmd object
		"""

		if process_is_param_missing(event):
			self.out = "Missing param " + "--event"
			return self

		self.out = process_unsubscribe_event(event)
		return self
