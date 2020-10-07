# Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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

import transport
import esp_hosted_config_pb2
import binascii

class Aplist:
    def __init__(self,ssid,chnl,rssi,bssid,ecn):
        self.ssid = ssid
        self.chnl = chnl
        self.rssi = rssi
        self.bssid = bssid
        self.ecn = ecn

class Stationlist:
    def __init__(self,mac,rssi):
        self.mac = mac
        self.rssi = rssi

#default parameters
interface = "/dev/esps0"
endpoint = "control"
not_set = "0"
failure = "failure"
success = "success"
not_connected = "not_connected"

max_ssid_len = 32
max_password_len = 64

# wifi get mac
# Function returns mac address of ESP32's station or softAP mode
# mode == 1 for station mac
# mode == 2 for softAP mac
def wifi_get_mac(mode):
    get_mac = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_mac.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetMACAddress
    get_mac.cmd_get_mac_address.mode = mode
    protodata = get_mac.SerializeToString()
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,0.3)
    if response == failure :
        return failure
    get_mac.ParseFromString(response)
    return get_mac.resp_get_mac_address.resp

# wifi get mode
# Function returns ESP32's wifi mode as follows
# 0: null Mode, Wi-Fi mode not set
# 1: station mode
# 2: softAP mode
# 3: softAP+station mode

def wifi_get_mode():
     get_mode = esp_hosted_config_pb2.EspHostedConfigPayload()
     get_mode.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetWiFiMode
     protodata = get_mode.SerializeToString()
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     if response == failure :
        return failure
     get_mode.ParseFromString(response)
     return get_mode.resp_get_wifi_mode.mode

# wifi set mode
# Function sets ESP32's wifi mode
# Input parameter
#   mode : WiFi mode
#           (0: null Mode, Wi-Fi mode not set
#            1: station mode
#            2: softAP mode
#            3: softAP+station mode)
def wifi_set_mode(mode):
     set_mode = esp_hosted_config_pb2.EspHostedConfigPayload()
     set_mode.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetWiFiMode
     set_mode.cmd_set_wifi_mode.mode = mode
     protodata = set_mode.SerializeToString()
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     if response == failure :
         return failure
     set_mode.ParseFromString(response)
     return success

# wifi set ap config
# Function sets AP config to which ESP32 station should connect
# Input parameter
#       ssid              : string parameter, ssid of AP
#       pwd               : string parameter, length of password should be 8~64 bytes ASCII
#       bssid             : MAC address of AP, To differentiate between APs, In case multiple AP has same ssid
#       is_wpa3_supported : status of wpa3 supplicant present on AP
#                  (False : Unsupported
#                    True : Supported )
#       listen_interval   : Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set.
#                           Units: AP beacon intervals. Defaults to 3 if set to 0.

def wifi_set_ap_config(ssid, pwd, bssid, is_wpa3_supported, listen_interval):
    set_ap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
    set_ap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetAPConfig
    set_ap_config.cmd_set_ap_config.ssid = str(ssid)
    set_ap_config.cmd_set_ap_config.pwd = str(pwd)
    set_ap_config.cmd_set_ap_config.bssid = str(bssid)
    set_ap_config.cmd_set_ap_config.is_wpa3_supported = is_wpa3_supported
    set_ap_config.cmd_set_ap_config.listen_interval = listen_interval
    protodata = set_ap_config.SerializeToString()
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,10)
    if response == failure :
        return failure
    set_ap_config.ParseFromString(response)
    status = set_ap_config.resp_set_ap_config.status
    return status

# wifi get ap config
# Function returns AP config to which ESP station is connected
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
     get_ap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
     get_ap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetAPConfig
     protodata = get_ap_config.SerializeToString()
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,1)
     if response == failure :
         return failure
     get_ap_config.ParseFromString(response)
     if str(get_ap_config.resp_get_ap_config.status) == not_connected:
         return not_connected
     elif str(get_ap_config.resp_get_ap_config.status) != success:
         return failure
     ssid = str(get_ap_config.resp_get_ap_config.ssid)
     bssid = str(get_ap_config.resp_get_ap_config.bssid)
     channel = get_ap_config.resp_get_ap_config.chnl
     rssi = get_ap_config.resp_get_ap_config.rssi
     ecn = get_ap_config.resp_get_ap_config.ecn
     return ssid,bssid,channel,rssi,ecn

# wifi disconnect ap
# Function disconnects ESP32 station from connected AP
# returns "success" or "failure"
def wifi_disconnect_ap():
     disconnect_ap = esp_hosted_config_pb2.EspHostedConfigPayload()
     disconnect_ap.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdDisconnectAP
     protodata = disconnect_ap.SerializeToString()
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     if response == failure :
         return failure
     disconnect_ap.ParseFromString(response)
     status = disconnect_ap.resp_disconnect_ap.resp
     return status

# wifi set softap config
# Function sets ESP32 softAP configurations
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
#       ssid_hidden : softAP should broadcast its SSID or not
#               ( 0 : SSID is broadcast
#                 1 : SSID is not broadcast )
#       bw          : set bandwidth of ESP32 softAP
#               ( 1 : WIFI_BW_HT20
#                 2 : WIFI_BW_HT40 )

def wifi_set_softap_config(ssid, pwd, chnl, ecn, max_conn, ssid_hidden, bw):
    set_softap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
    set_softap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetSoftAPConfig
    if len(ssid) > max_ssid_len :
        print("SSID length is more than 32 Bytes")
        return failure
    if len(pwd) > max_password_len :
        print("softAP password length is more than 64 bytes")
        return failure
    set_softap_config.cmd_set_softap_config.ssid = str(ssid)
    set_softap_config.cmd_set_softap_config.pwd = str(pwd)
    set_softap_config.cmd_set_softap_config.chnl = chnl
    if ecn < esp_hosted_config_pb2.EspHostedEncryptionMode.Type_WPA3_PSK:
        set_softap_config.cmd_set_softap_config.ecn = ecn
    else:
        set_softap_config.cmd_set_softap_config.ecn = esp_hosted_config_pb2.EspHostedEncryptionMode.Type_WPA2_PSK
        print("Asked Encryption method is not supported in SoftAP mode, Setting Encryption method as WPA2_PSK")
    set_softap_config.cmd_set_softap_config.max_conn = max_conn
    set_softap_config.cmd_set_softap_config.ssid_hidden = ssid_hidden
    set_softap_config.cmd_set_softap_config.bw = bw
    protodata = set_softap_config.SerializeToString()
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,3)
    if response == failure :
        return failure
    set_softap_config.ParseFromString(response)
    status = set_softap_config.resp_set_softap_config.status
    return status

# wifi get softap config
# Funtion gets ESP32 softAP configuration
# Output parameter
# It returns ssid,pwd,chnl,ecn,max_conn,ssid_hidden,status,bw
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
#       status : return SUCCESS or FAILURE as result of read operation
#       bw : bandwidth of ESP32 softAP
#         ( 1 : WIFI_BW_HT20
#           2 : WIFI_BW_HT40 )

def wifi_get_softap_config():
    get_softap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_softap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetSoftAPConfig
    protodata = get_softap_config.SerializeToString()
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,0.3)
    if response == failure :
        return failure
    get_softap_config.ParseFromString(response)
    ssid = str(get_softap_config.resp_get_softap_config.ssid)
    pwd = str(get_softap_config.resp_get_softap_config.pwd)
    ecn = get_softap_config.resp_get_softap_config.ecn
    chnl  = get_softap_config.resp_get_softap_config.chnl
    max_conn = get_softap_config.resp_get_softap_config.max_conn
    ssid_hidden  = get_softap_config.resp_get_softap_config.ssid_hidden
    status  = str(get_softap_config.resp_get_softap_config.status)
    bw = get_softap_config.resp_get_softap_config.bw
    return ssid,pwd,chnl,ecn,max_conn,ssid_hidden,status,bw

# wifi ap scan list
# Function gives scanned list of available APs
# Output parameter
#       output is list of Aplist class instances(ssid,chnl,rssi,bssid,ecn)
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
#
def wifi_ap_scan_list():
    get_ap_scan_list = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_ap_scan_list.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetAPScanList
    protodata = get_ap_scan_list.SerializeToString()
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,10)
    if response == failure :
        return failure
    get_ap_scan_list.ParseFromString(response)
    count = get_ap_scan_list.resp_scan_ap_list.count
    ap_list = []
    for i in range(count) :
        ssid = get_ap_scan_list.resp_scan_ap_list.entries[i].ssid
        chnl = get_ap_scan_list.resp_scan_ap_list.entries[i].chnl
        rssi = get_ap_scan_list.resp_scan_ap_list.entries[i].rssi
        bssid = get_ap_scan_list.resp_scan_ap_list.entries[i].bssid
        ecn = get_ap_scan_list.resp_scan_ap_list.entries[i].ecn
        ap_list.append(Aplist(ssid,chnl,rssi,bssid,ecn))
    return ap_list

# wifi connected stations list
# Function gives list of connected stations(maximum 10) to ESP32 softAP
# Output parameter
#      Stations credentials::
#          mac         :   MAC address of station
#          rssi        :   rssi signal strength
# If no station is connected, failure return from slave
# output is list of Stationlist class instances
def wifi_connected_stations_list():
    get_connected_stations_list = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_connected_stations_list.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetConnectedSTAList
    protodata = get_connected_stations_list.SerializeToString()
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,1)
    if response == failure :
        return failure
    get_connected_stations_list.ParseFromString(response)
    num = get_connected_stations_list.resp_connected_stas_list.num
    if (num == 0) :
        print("No station is connected")
        return failure
    else :
        stas_list = []
        for i in range(num) :
            mac = get_connected_stations_list.resp_connected_stas_list.stations[i].mac
            rssi = get_connected_stations_list.resp_connected_stas_list.stations[i].rssi
            stas_list.append(Stationlist(mac,rssi))
        return stas_list

# wifi set mac
# Function sets MAC address for Station and SoftAP interface
# mode == 1 for station mac
# mode == 2 for softAP mac
# returns success or failure
# @attention 1. First set wifi mode before setting MAC address for respective station and softAP Interface
# @attention 2. ESP32 station and softAP have different MAC addresses, do not set them to be the same.
# @attention 3. The bit 0 of the first byte of ESP32 MAC address can not be 1.
# For example, the MAC address can set to be "1a:XX:XX:XX:XX:XX", but can not be "15:XX:XX:XX:XX:XX".
# @attention 4. MAC address will get reset after esp restarts

def wifi_set_mac(mode, mac):
    set_mac = esp_hosted_config_pb2.EspHostedConfigPayload()
    set_mac.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetMacAddress
    set_mac.cmd_set_mac_address.mode = mode
    set_mac.cmd_set_mac_address.mac = mac
    protodata = set_mac.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,1)
    if response == failure:
        return failure
    #print("response from slave "+str(response))
    set_mac.ParseFromString(response)
    status = set_mac.resp_set_mac_address.resp
    return status

# wifi set power save mode
# Function sets ESP32's power save mode, returns success or failure
# power save mode == 1      WIFI_PS_MIN_MODEM,   /**< Minimum modem power saving.
#                           In this mode, station wakes up to receive beacon every DTIM period */
# power save mode == 2      WIFI_PS_MAX_MODEM,   /**< Maximum modem power saving.
#                           In this mode, interval to receive beacons is determined by the
#                           listen_interval parameter in wifi set ap config function*/
# Default :: power save mode is WIFI_PS_MIN_MODEM

def wifi_set_power_save_mode(power_save_mode):
    set_power_save_mode = esp_hosted_config_pb2.EspHostedConfigPayload()
    set_power_save_mode.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetPowerSaveMode
    set_power_save_mode.cmd_set_power_save_mode.power_save_mode = power_save_mode
    protodata = set_power_save_mode.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,1)
    if response == failure:
        return failure
    #print("response from slave "+str(response))
    set_power_save_mode.ParseFromString(response)
    status = set_power_save_mode.resp_set_power_save_mode.resp
    return status

# wifi get power save mode
# Function returns power save mode of ESP32 or failure
# power save mode == 1      WIFI_PS_MIN_MODEM,   /**< Minimum modem power saving.
#                           In this mode, station wakes up to receive beacon every DTIM period */
# power save mode == 2      WIFI_PS_MAX_MODEM,   /**< Maximum modem power saving.
#                           In this mode, interval to receive beacons is determined by the
#                           listen_interval parameter in wifi set ap config function*/
# Default :: power save mode is WIFI_PS_MIN_MODEM

def wifi_get_power_save_mode():
    get_power_save_mode = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_power_save_mode.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetPowerSaveMode
    protodata = get_power_save_mode.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,1)
    if response == failure:
        return failure
    #print("response from slave "+str(response))
    get_power_save_mode.ParseFromString(response)
    status = get_power_save_mode.resp_get_power_save_mode.resp
    if status != success:
        return failure
    else:
        return get_power_save_mode.resp_get_power_save_mode.power_save_mode
