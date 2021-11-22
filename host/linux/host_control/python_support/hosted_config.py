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
import sys

success = "success"
failure = "failure"
no_ap_found_str = "no_ap_found"
invalid_password_str = "invalid_password_str"
out_of_range_str = "out_of_range"

SSID_LENGTH = 32
PASSWORD_LENGTH = 64
BSSID_LENGTH = 17
STATUS_LENGTH = 14

MAX_SSID_LEN = 32
MAX_PASSWORD_LEN = 64
MIN_PASSWORD_LEN = 8
MAX_MAC_STR_LEN = 17
MIN_CHANNEL_NO = 1
MAX_CHANNEL_NO = 11
MIN_ALLOWED_STATIONS = 1
MAX_ALLOWED_STATIONS = 10
SSID_BROADCAST = 0
SSID_NOT_BROADCAST = 1

NOT_CONNECTED = 1
NO_AP_FOUND = 2 
INVALID_PASSWORD = 3 
INVALID_ARGUMENT = 4
OUT_OF_RANGE = 5

(WIFI_MODE_NONE, WIFI_MODE_STATION,
        WIFI_MODE_SOFTAP, WIFI_MODE_SOFTAP_STATION,
        WIFI_MODE_MAX) = (0, 1, 2, 3, 4)

(WIFI_AUTH_OPEN, WIFI_AUTH_WEP,
        WIFI_AUTH_WPA_PSK, WIFI_AUTH_WPA2_PSK,
        WIFI_AUTH_WPA_WPA2_PSK, WIFI_AUTH_WPA2_ENTERPRISE,
        WIFI_AUTH_WPA3_PSK, WIFI_AUTH_WPA2_WPA3_PSK,
        WIFI_AUTH_MAX) = (0, 1, 2, 3, 4, 5, 6, 7, 8)

(WIFI_BW_HT20, WIFI_BW_HT40) = (1, 2)

(WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM,
        WIFI_PS_INVALID) = (1, 2, 3)

class STA_CONFIG(Structure):
    _fields_ = [("ssid", c_char * SSID_LENGTH),
                ("pwd", c_char * PASSWORD_LENGTH),
                ("bssid", c_char * BSSID_LENGTH),
                ("is_wpa3_supported", c_bool),
                ("rssi", c_int),
                ("channel", c_uint),
                ("encryption_mode", c_uint),
                ("listen_interval", c_ushort),
                ("status", c_char * STATUS_LENGTH)]

class SOFTAP_CONFIG(Structure):
    _fields_ = [("ssid", c_char * SSID_LENGTH),
                ("pwd", c_char * PASSWORD_LENGTH),
                ("channel", c_uint),
                ("encryption_mode", c_uint),
                ("max_connections", c_uint),
                ("ssid_hidden", c_bool),
                ("bandwidth", c_uint)]

class CONTROL_CONFIG(Union):
    _fields_ = [("station", STA_CONFIG),
                ("softap", SOFTAP_CONFIG)]


class WIFI_SCAN_LIST(Structure):
    _fields_ = [("ssid", c_char * SSID_LENGTH),
                ("bssid", c_char * BSSID_LENGTH),
                ("rssi", c_int),
                ("channel", c_uint),
                ("encryption_mode", c_uint)]

class WIFI_STATIONS_LIST(Structure):
    _fields_ = [("bssid", c_char * BSSID_LENGTH),
                ("rssi", c_int)]

class Aplist:
    def __init__(self,ssid,chnl,rssi,bssid,ecn):
        self.ssid = ssid
        self.chnl = chnl
        self.rssi = rssi
        self.bssid = bssid
        self.ecn = ecn

class Stationlist:
    def __init__(self,bssid,rssi):
        self.bssid = bssid
        self.rssi = rssi

def get_str(string):
    if sys.version_info >= (3, 0):
        return string.decode('utf-8')
    else:
        return string

def set_str(string):
    if sys.version_info >= (3, 0):
        return string.encode('utf-8')
    else:
        return string
