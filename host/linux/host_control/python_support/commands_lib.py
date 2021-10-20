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

from hosted_config import *
import commands_map_py_to_c

# Control path platform init
# On success, It clears stale data from ringbuffer
# On failure, It exits currently running python script
def control_path_platform_init():
    ret = commands_map_py_to_c.control_path_platform_init()
    if ret:
        print("Control path init failed")
        exit()
    return

# wifi get mac
# On success, function returns mac address of ESP32's station or softap mode else "failure"
# mode == 1 for station mac
# mode == 2 for softap mac
def wifi_get_mac(mode):
    if ((mode <= WIFI_MODE_NONE ) or (mode >= WIFI_MODE_SOFTAP_STATION)):
        print("Invalid mode")
        return failure
    mac = create_string_buffer(b"",BSSID_LENGTH)
    ret = commands_map_py_to_c.wifi_get_mac(mode, mac)
    if not ret :
        return get_str(mac.value)
    else :
        return failure

# wifi set mac
# Function sets MAC address for Station and SoftAP interface
# mode == 1 for station mac
# mode == 2 for softap mac
# returns "success" or "failure"
# @attention 1. First set wifi mode before setting MAC address for respective station and softap Interface
# @attention 2. ESP32 station and softap have different MAC addresses, do not set them to be the same.
# @attention 3. The bit 0 of the first byte of ESP32 MAC address can not be 1.
# For example, the MAC address can set to be "1a:XX:XX:XX:XX:XX", but can not be "15:XX:XX:XX:XX:XX".
# @attention 4. MAC address will get reset after esp restarts
def wifi_set_mac(mode, mac):
    if (mode <= WIFI_MODE_NONE or mode >= WIFI_MODE_SOFTAP_STATION):
        print("Invalid mode")
        return failure
    if (not(len(mac)) or (len(mac) > MAX_BSSID_LEN)) :
        print("Invalid MAC address")
        return failure
    ret = commands_map_py_to_c.wifi_set_mac(mode, set_str(mac))
    if not ret:
        return success
    else:
        return failure

# wifi get mode
# Function returns ESP32's wifi mode as follows
# 0: null Mode, Wi-Fi mode not set
# 1: station mode
# 2: softap mode
# 3: softap+station mode
# or "failure"

def wifi_get_mode():
    mode = c_uint()
    ret = commands_map_py_to_c.wifi_get_mode(byref(mode))
    if not ret :
        return int(mode.value)
    else :
        return failure

# wifi set mode
# Function sets ESP32's wifi mode
# Input parameter
#   mode : WiFi mode
#           (0: null Mode, Wi-Fi mode not set
#            1: station mode
#            2: softAP mode
#            3: softAP+station mode)
# Returns "success" or "failure"

def wifi_set_mode(mode):
    if (mode < WIFI_MODE_NONE or mode > WIFI_MODE_SOFTAP_STATION):
        print("Invalid mode")
        return failure
    wifi_mode = c_uint()
    wifi_mode.value = mode
    ret = commands_map_py_to_c.wifi_set_mode(wifi_mode)
    if not ret:
        return success
    else:
        return failure

# wifi set ap config
# Function sets AP config to which ESP32 station should connect
# Input parameter
#       ssid              : string parameter, ssid of AP, max 32 bytes
#       pwd               : string parameter, length of password should be 8~63 bytes ASCII
#       bssid             : MAC address of AP, To differentiate between APs, In case multiple AP has same ssid
#       is_wpa3_supported : status of wpa3 supplicant present on AP
#                  (False : Unsupported
#                    True : Supported )
#       listen_interval   : Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set.
#                           Units: AP beacon intervals. Defaults to 3 if set to 0.

def wifi_set_ap_config(ssid, pwd, bssid, is_wpa3_supported, listen_interval):
    if (len(str(ssid)) > MAX_SSID_LEN):
        print("Invalid SSID length")
        return failure
    if (len(str(pwd)) > (MAX_PASSWORD_LEN - 1)) :
        print("Invalid Password length")
        return failure
    if (len(str(bssid)) > MAX_BSSID_LEN) :
        print("Invalid BSSID length")
        return failure
    if (is_wpa3_supported < 0 or listen_interval < 0) :
        print("Invalid Input")
        return failure
    ap_config = CONTROL_CONFIG()
    ap_config.station.ssid = set_str(ssid)
    ap_config.station.pwd = set_str(pwd)
    ap_config.station.bssid = set_str(bssid)
    ap_config.station.is_wpa3_supported = is_wpa3_supported
    ap_config.station.listen_interval = listen_interval
    ret = commands_map_py_to_c.wifi_set_ap_config(ap_config)
    if not ret:
        return success
    else:
        return failure

# wifi get ap config
# Function returns AP config to which ESP32 station is connected
# Output parameter
#       ssid                :   ssid of connected AP
#       bssid               :   MAC address of connected AP
#       channel             :   channel ID, 1 ~ 10
#       rssi                :   rssi signal strength
#       encryption_mode     :   encryption mode
#       (encryption modes are
#             0 :   OPEN
#             1 :   WEP
#             2 :   WPA_PSK
#             3 :   WPA2_PSK
#             4 :   WPA_WPA2_PSK
#             5 :   WPA2_ENTERPRISE
#             6 :   WPA3_PSK
#             7 :   WPA2_WPA3_PSK   )
# In case of not connected to AP, returns "not_connected"

def wifi_get_ap_config():
    ap_config = CONTROL_CONFIG()
    ret = commands_map_py_to_c.wifi_get_ap_config(byref(ap_config.station))
    if not ret:
        ssid = get_str(ap_config.station.ssid)
        bssid = get_str(ap_config.station.bssid)
        channel = int(ap_config.station.channel)
        rssi = int(ap_config.station.rssi)
        ecn = int(ap_config.station.encryption_mode)
        return ssid,bssid,channel,rssi,ecn
    else:
        print("status "+get_str(ap_config.station.status))
        return failure

# wifi disconnect ap
# Function disconnects ESP32 station from connected AP
# returns "success" or "failure"

def wifi_disconnect_ap():
    ret = commands_map_py_to_c.wifi_disconnect_ap()
    if not ret:
        return success
    else:
       return failure

# wifi set softap config
# Function sets ESP32 softap configurations
# returns "success" or "failure"
# Input parameter
#       ssid        : string parameter, ssid of SoftAP
#       pwd         : string parameter, length of password should be 8~64 bytes ASCII
#       chnl        : channel ID, In range of 1 to 11
#       ecn         : Encryption method
#               ( 0 : OPEN,
#                 2 : WPA_PSK,
#                 3 : WPA2_PSK,
#                 4 : WPA_WPA2_PSK)
#       max_conn    : maximum number of stations can connect to ESP32 SoftAP (should be in range of 1 to 10)
#       ssid_hidden : softap should broadcast its SSID or not
#               ( 0 : SSID is broadcast
#                 1 : SSID is not broadcast )
#       bw          : set bandwidth of ESP32 softap
#               ( 1 : WIFI_BW_HT20
#                 2 : WIFI_BW_HT40 )

def wifi_set_softap_config(ssid, pwd, chnl, ecn, max_conn, ssid_hidden, bw):
    if (len(ssid) > MAX_SSID_LEN) :
        print("Invalid SSID length softap")
        return failure
    if ((len(pwd) > MAX_PASSWORD_LEN) or (ecn == WIFI_AUTH_OPEN and (len(pwd)))
            or (ecn != WIFI_AUTH_OPEN and (len(pwd) < MIN_PASSWORD_LEN))):
        print("Invalid softap password length")
        return failure
    if ((chnl < MIN_CHANNEL_NO) or (chnl > MAX_CHANNEL_NO)):
        print("Invalid channel number")
        return failure
    if ((ecn < WIFI_AUTH_OPEN) or (ecn == WIFI_AUTH_WEP)
            or (ecn > WIFI_AUTH_WPA_WPA2_PSK)):
        print("Asked Encryption method is not supported in SoftAP mode")
        return failure
    if (max_conn < MIN_ALLOWED_STATIONS or max_conn > MAX_ALLOWED_STATIONS):
        print("Invalid maximum connection number")
        return failure
    if (ssid_hidden < SSID_BROADCAST or ssid_hidden > SSID_NOT_BROADCAST):
        print("Invalid ssid hidden status")
        return failure
    if (bw < WIFI_BW_HT20 or bw > WIFI_BW_HT40):
        print("Invalid BW")
        return failure

    softap_config = CONTROL_CONFIG()
    softap_config.softap.ssid = set_str(ssid)
    softap_config.softap.pwd = set_str(pwd)
    softap_config.softap.channel = chnl
    softap_config.softap.encryption_mode = ecn
    softap_config.softap.max_connections = max_conn
    softap_config.softap.ssid_hidden = ssid_hidden
    softap_config.softap.bandwidth = bw

    ret = commands_map_py_to_c.wifi_set_softap_config(softap_config)
    if not ret:
        return success
    else:
        return failure

# wifi get softap config
# Funtion gets ESP32 softAP configuration
# Output parameter
# It returns ssid,pwd,chnl,ecn,max_conn,ssid_hidden,bw in case of "success"
#       ssid : string parameter, ssid of SoftAP
#       pwd  : string parameter, length of password should be 8~64 bytes ASCII
#       chnl : channel ID, In range of 1 to 11
#       ecn  : Encryption method
#         ( 0 : OPEN,
#           2 : WPA_PSK,
#           3 : WPA2_PSK,
#           4 : WPA_WPA2_PSK)
#       max_conn : maximum number of stations can connect to ESP32 SoftAP (will be in range of 1 to 10)
#       ssid_hidden : softAP should broadcast its SSID or not
#         ( 0 : SSID is broadcast
#           1 : SSID is not broadcast )
#       bw : bandwidth of ESP32 softAP
#         ( 1 : WIFI_BW_HT20
#           2 : WIFI_BW_HT40 )
# else returns "failure"

def wifi_get_softap_config():
    softap_config = CONTROL_CONFIG()
    ret = commands_map_py_to_c.wifi_get_softap_config(byref(softap_config))
    if not ret:
        ssid = get_str(softap_config.softap.ssid)
        pwd = get_str(softap_config.softap.pwd)
        ecn = int(softap_config.softap.encryption_mode)
        chnl = int(softap_config.softap.channel)
        max_conn = int(softap_config.softap.max_connections)
        ssid_hidden = int(softap_config.softap.ssid_hidden)
        bw = int(softap_config.softap.bandwidth)
        return ssid,pwd,chnl,ecn,max_conn,ssid_hidden,bw
    else:
        return failure

# wifi stop softap
# Function stops ESP32 softAP
# returns "success" or "failure"

def wifi_stop_softap():
    ret = commands_map_py_to_c.wifi_stop_softap()
    if not ret:
        return success
    else:
        return failure

# wifi ap scan list
# Function gives scanned list of available APs
# Output parameter
#       output is list of Aplist class instances(ssid,chnl,rssi,bssid,ecn) in case of "success"
#       AP credentials::
#         ssid                :   ssid of AP
#         channel             :   channel ID, in range of 1 to 10
#         bssid               :   MAC address of AP
#         rssi                :   rssi signal strength
#         encryption_mode     :   encryption mode
#         (encryption modes are
#             0 :   OPEN
#             1 :   WEP
#             2 :   WPA_PSK
#             3 :   WPA2_PSK
#             4 :   WPA_WPA2_PSK
#             5 :   WPA2_ENTERPRISE
#             6 :   WPA3_PSK
#             7 :   WPA2_WPA3_PSK   )
# else returns "failure"

def wifi_ap_scan_list():
    count = c_uint()
    ap_scan_list_ptr = commands_map_py_to_c.wifi_ap_scan_list(byref(count))
    if ap_scan_list_ptr is None:
        return failure
    list_type = WIFI_SCAN_LIST * count.value
    ap_scan_list = cast(ap_scan_list_ptr, POINTER(list_type))
    ap_list = []
    if count.value:
        for i in range(count.value):
            ssid = get_str(ap_scan_list.contents[i].ssid)
            chnl = int(ap_scan_list.contents[i].channel)
            rssi = int(ap_scan_list.contents[i].rssi)
            bssid = get_str(ap_scan_list.contents[i].bssid)
            ecn = int(ap_scan_list.contents[i].encryption_mode)
            ap_list.append(Aplist(ssid,chnl,rssi,bssid,ecn))
    commands_map_py_to_c.esp_hosted_free(ap_scan_list_ptr)
    ap_scan_list_ptr = None
    return ap_list

# wifi connected stations list
# Function gives list of connected stations(maximum 10) to ESP32 softAP
# In case of "success"
# Output parameter
#      Stations credentials::
#          mac         :   MAC address of station
#          rssi        :   rssi signal strength
# If no station is connected, failure return from slave
# output is list of Stationlist class instances
# else returns "failure"

def wifi_connected_stations_list():
    count = c_uint()
    stations_list_ptr = commands_map_py_to_c.wifi_connected_stations_list(byref(count))
    if stations_list_ptr is None:
        return failure
    list_type = WIFI_STATIONS_LIST * count.value
    stations_list = cast(stations_list_ptr, POINTER(list_type))
    if count.value:
        stas_list = []
        for i in range(count.value):
            bssid = get_str(stations_list.contents[i].bssid)
            rssi = int(stations_list.contents[i].rssi)
            stas_list.append(Stationlist(bssid,rssi))
        commands_map_py_to_c.esp_hosted_free(stations_list_ptr)
        stations_list_ptr = None
        return stas_list
    else:
        commands_map_py_to_c.esp_hosted_free(stations_list_ptr)
        stations_list_ptr = None
        print("No station is connected")
        return failure

# wifi set power save mode
# Function sets ESP32's power save mode, returns "success" or "failure"
# power save mode == 1      WIFI_PS_MIN_MODEM,   /**< Minimum modem power saving.
#                           In this mode, station wakes up to receive beacon every DTIM period */
# power save mode == 2      WIFI_PS_MAX_MODEM,   /**< Maximum modem power saving.
#                           In this mode, interval to receive beacons is determined by the
#                           listen_interval parameter in wifi set ap config function*/
# Default :: power save mode is WIFI_PS_MIN_MODEM

def wifi_set_power_save_mode(power_save_mode):
    if ((power_save_mode < WIFI_PS_MIN_MODEM) or (power_save_mode > WIFI_PS_MAX_MODEM)):
        print("Unsupported power save mode")
        return failure
    mode = c_uint()
    mode.value = power_save_mode
    ret = commands_map_py_to_c.wifi_set_power_save_mode(mode)
    if not ret:
        return success
    else:
        return failure

# wifi get power save mode
# Function returns power save mode of ESP32 or "failure"
# power save mode == 1      WIFI_PS_MIN_MODEM,   /**< Minimum modem power saving.
#                           In this mode, station wakes up to receive beacon every DTIM period */
# power save mode == 2      WIFI_PS_MAX_MODEM,   /**< Maximum modem power saving.
#                           In this mode, interval to receive beacons is determined by the
#                           listen_interval parameter in wifi set ap config function*/
# Default :: power save mode is WIFI_PS_MIN_MODEM

def wifi_get_power_save_mode():
    mode = c_uint()
    ret = commands_map_py_to_c.wifi_get_power_save_mode(byref(mode))
    if not ret:
        return mode.value
    else:
        return failure

# OTA begin
# function returns "success" or "failure"
# esp ota begin function performs an OTA begin operation for ESP32
# which sets partition for OTA write and erase it. 

def esp_ota_begin():
    ret = commands_map_py_to_c.esp_ota_begin()
    if not ret:
        return success
    else:
        return failure

# OTA write
# esp ota write function performs an OTA write operation for ESP32,
# function returns "success" or "failure"
# It writes ota_data buffer to OTA partition in flash
#
#  Input parameter:
#      ota_data        : OTA data buffer
#      ota_data_len    : length of OTA data buffer

def esp_ota_write(ota_data, ota_image_len):
    ret = commands_map_py_to_c.esp_ota_write(ota_data, ota_image_len)
    if not ret:
        return success
    else:
        return failure

# OTA end
# esp ota end function performs an OTA end operation for ESP32,
# function returns "success" or "failure"
# It validates written OTA image, set OTA partition as boot partition for next boot,
# Creates timer which reset ESP32 after 5 sec,

def esp_ota_end():
    ret = commands_map_py_to_c.esp_ota_end()
    if not ret:
        return success
    else:
        return failure
