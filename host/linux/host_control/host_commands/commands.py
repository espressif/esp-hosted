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

#get mac address
# mode == 1 for station mac
# mode == 2 for softAP mac
def get_mac(mode):
    req_sta_mac = esp_hosted_config_pb2.EspHostedConfigPayload()
    req_sta_mac.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetMACAddress
    req_sta_mac.cmd_get_mac_address.mode = mode
    protodata = req_sta_mac.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,0.3)
    if response == failure :
        return failure
    #print("response from slave "+str(response))
    req_sta_mac.ParseFromString(response)
    return req_sta_mac.resp_get_mac_address.resp

#get wifi mode
# 0: null Mode, Wi-Fi mode not set
# 1: station mode
# 2: softAP mode
# 3: softAP+station mode

def get_wifi_mode():
     req_wifi_mode = esp_hosted_config_pb2.EspHostedConfigPayload()
     req_wifi_mode.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetWiFiMode
     protodata = req_wifi_mode.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     if response == failure :
        return failure
     #print("response from slave "+str(response))
     req_wifi_mode.ParseFromString(response)
     return req_wifi_mode.resp_get_wifi_mode.mode

#set wifi mode
# 0: null Mode, Wi-Fi mode not set
# 1: station mode
# 2: softAP mode
# 3: softAP+station mode
def set_wifi_mode(mode):
     set_wifi_mode = esp_hosted_config_pb2.EspHostedConfigPayload()
     set_wifi_mode.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetWiFiMode
     set_wifi_mode.cmd_set_wifi_mode.mode = mode
     protodata = set_wifi_mode.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     if response == failure :
         return failure
     #print("response from slave "+str(response))
     set_wifi_mode.ParseFromString(response)
     return set_wifi_mode.resp_set_wifi_mode.mode

# get AP config to which ESP station is connected
# It returns ssid, bssid, channel, rssi and ecn
def wifi_get_ap_config():
     get_ap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
     get_ap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetAPConfig
     protodata = get_ap_config.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,1)
     if response == failure :
         return failure
     #print("response from slave "+str(response))
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

# set AP config to which ESP station should connect
# User should provide following parameters
#
# ssid              : string parameter, ssid of SoftAP
# pwd               : string parameter, length of password should be 8~64 bytes ASCII
# bssid             : MAC address of AP, To differentiate between APs, In case multiple AP has same ssid
# is_wpa3_supported : status of wpa3 supplicant present on AP
#            (False : Unsupported
#              True : Supported )

def wifi_set_ap_config(ssid, pwd, bssid, is_wpa3_supported):
    set_ap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
    set_ap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetAPConfig
    set_ap_config.cmd_set_ap_config.ssid = str(ssid)
    set_ap_config.cmd_set_ap_config.pwd = str(pwd)
    set_ap_config.cmd_set_ap_config.bssid = str(bssid)
    set_ap_config.cmd_set_ap_config.is_wpa3_supported = is_wpa3_supported
    protodata = set_ap_config.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,10)
    if response == failure :
        return failure
    #print("response from slave "+str(response))
    set_ap_config.ParseFromString(response)
    status = set_ap_config.resp_set_ap_config.status
    return status

# Disconnect from AP
def wifi_disconnect_ap():
     disconnect_ap = esp_hosted_config_pb2.EspHostedConfigPayload()
     disconnect_ap.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdDisconnectAP
     protodata = disconnect_ap.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     if response == failure :
         return failure
     #print("response from slave "+str(response))
     disconnect_ap.ParseFromString(response)
     status = disconnect_ap.resp_disconnect_ap.resp
     return status


# set softAP config
# ssid : string parameter, ssid of SoftAP
# pwd  : string parameter, length of password should be 8~64 bytes ASCII
# chnl : channel ID should be in range of 1 to 11
# ecn  : Encryption method
#   ( 0 : OPEN,
#     2 : WPA_PSK,
#     3 : WPA2_PSK,
#     4 : WPA_WPA2_PSK)
# max_conn : maximum number of stations can connect to ESP32 SoftAP (should be in range of 1 to 10)
# ssid_hidden : softAP should broadcast its SSID or not
#   ( 0 : SSID is broadcast
#     1 : SSID is not broadcast )
# bw : set bandwidth of ESP32 softAP
#   ( 1 : WIFI_BW_HT20
#     2 : WIFI_BW_HT40 )

def wifi_set_softap_config(ssid, pwd, chnl, ecn, max_conn, ssid_hidden, bw):
    set_softap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
    set_softap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdSetSoftAPConfig
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
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,3)
    if response == failure :
        return failure
    #print("response from slave "+str(response))
    set_softap_config.ParseFromString(response)
    status = set_softap_config.resp_set_softap_config.status
    return status

# get softAP configuration
# It returns ssid,pwd,chnl,ecn,max_conn,ssid_hidden,status,bw
# ssid : string parameter, ssid of SoftAP
# pwd  : string parameter, length of password should be 8~64 bytes ASCII
# chnl : channel ID will be in range of 1 to 11
# ecn  : Encryption method
#   ( 0 : OPEN,
#     2 : WPA_PSK,
#     3 : WPA2_PSK,
#     4 : WPA_WPA2_PSK)
# max_conn : maximum number of stations can connect to ESP32 SoftAP (will be in range of 1 to 10)
# ssid_hidden : softAP should broadcast its SSID or not
#   ( 0 : SSID is broadcast
#     1 : SSID is not broadcast )
# status : return SUCCESS or FAILURE as result of read operation
# bw : bandwidth of ESP32 softAP
#   ( 1 : WIFI_BW_HT20
#     2 : WIFI_BW_HT40 )

def wifi_get_softap_config():
    get_softap_config = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_softap_config.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetSoftAPConfig
    protodata = get_softap_config.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,0.3)
    if response == failure :
        return failure
    #print("response from slave "+str(response))
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

# Scan AP list
# It scans the available APs
# output is list of Aplist class instances(ssid,chnl,rssi,bssid,ecn)

def wifi_ap_scan_list():
    get_ap_scan_list = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_ap_scan_list.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetAPScanList
    protodata = get_ap_scan_list.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,10)
    if response == failure :
        return failure
    #print("response from slave "+str(response))
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

# This function returns the number of connected stations to softAP
# Maximum 10 connected stations info can get
# If no station is connected, failure return from slave
# output is list of Stationlist class instances

def wifi_connected_stations_list():
    get_connected_stations_list = esp_hosted_config_pb2.EspHostedConfigPayload()
    get_connected_stations_list.msg = esp_hosted_config_pb2.EspHostedConfigMsgType.TypeCmdGetConnectedSTAList
    protodata = get_connected_stations_list.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,1)
    if response == failure :
        return failure
    #print("response from slave "+str(response))
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
