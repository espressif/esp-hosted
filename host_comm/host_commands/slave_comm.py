# Copyright 2019 Espressif Systems (Shanghai) PTE LTD
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
import slave_config_pb2
import binascii

class Aplist:
    def __init__(self,ssid,chnl,rssi,bssid,ecn):
        self.ssid = ssid
        self.chnl = chnl
        self.rssi = rssi
        self.bssid = bssid
        self.ecn = ecn

#default parameters
interface = "/dev/esps0"
endpoint = "control"
not_set = "0"

#get mac address of station
# mode == 1 for station mac
# mode == 2 for softAP mac
def get_mac(mode):
    req_sta_mac = slave_config_pb2.SlaveConfigPayload()
    req_sta_mac.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdGetMACAddress
    #for station mode set cmd as 1
    req_sta_mac.cmd_get_mac_address.cmd = str(mode)
    protodata = req_sta_mac.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,0.3)
    #print("response from slave "+str(response))
    req_sta_mac.ParseFromString(response)
    #print("parsed output "+str(req_sta_mac.resp_get_mac_address.resp))
    return req_sta_mac.resp_get_mac_address.resp

#get wifi mode
# 0: null Mode, Wi-Fi mode not set
# 1: station mode
# 2: softAP mode
# 3: softAP+station mode

def get_wifi_mode():
     req_wifi_mode = slave_config_pb2.SlaveConfigPayload()                          
     req_wifi_mode.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdGetWiFiMode
     protodata = req_wifi_mode.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     #print("response from slave "+str(response))
     req_wifi_mode.ParseFromString(response)
     #print("parsed output "+str(req_wifi_mode.resp_get_wifi_mode.mode))
     return req_wifi_mode.resp_get_wifi_mode.mode

#set wifi mode
# 0: null Mode, Wi-Fi mode not set
# 1: station mode
# 2: softAP mode
# 3: softAP+station mode
def set_wifi_mode(mode):
     set_wifi_mode = slave_config_pb2.SlaveConfigPayload()                          
     set_wifi_mode.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdSetWiFiMode
     set_wifi_mode.cmd_set_wifi_mode.mode = mode
     protodata = set_wifi_mode.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     #print("response from slave "+str(response))
     set_wifi_mode.ParseFromString(response)
     #print("parsed output "+str(set_wifi_mode.resp_set_wifi_mode.mode )
     return set_wifi_mode.resp_set_wifi_mode.mode

# get AP config to which ESP station is connected
# It returns ssid, bssid, channel and rssi
def wifi_get_ap_config():
     get_ap_config = slave_config_pb2.SlaveConfigPayload()                          
     get_ap_config.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdGetAPConfig
     protodata = get_ap_config.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     #print("response from slave "+str(response))
     get_ap_config.ParseFromString(response)
     #print("parsed output "+str(get_ap_config.resp_set_wifi_mode.mode )
     ssid = str(get_ap_config.resp_get_ap_config.ssid)
     bssid = str(get_ap_config.resp_get_ap_config.bssid)
     channel = get_ap_config.resp_get_ap_config.chnl
     rssi = get_ap_config.resp_get_ap_config.rssi
     return ssid,bssid,channel,rssi

# set AP config to which ESP station should connect
# User should provide ssid, password, bssid
def wifi_set_ap_config(ssid, pwd, bssid):
    set_ap_config = slave_config_pb2.SlaveConfigPayload()
    set_ap_config.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdSetAPConfig
    set_ap_config.cmd_set_ap_config.ssid = str(ssid)
    set_ap_config.cmd_set_ap_config.pwd = str(pwd)
    set_ap_config.cmd_set_ap_config.bssid = str(bssid)
    protodata = set_ap_config.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,10)
    #print("response from slave "+str(response))
    set_ap_config.ParseFromString(response)
    #print("parsed output "+str(set_ap_config.resp_set_ap_config.mode )
    status = set_ap_config.resp_set_ap_config.status
    return status

# Disconnect from AP
def wifi_disconnect_ap():
     disconnect_ap = slave_config_pb2.SlaveConfigPayload()
     disconnect_ap.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdDisconnectAP
     protodata = disconnect_ap.SerializeToString()
     #print("serialized data "+str(protodata))
     tp = transport.Transport_pserial(interface)
     response = tp.send_data(endpoint,protodata,0.3)
     #print("response from slave "+str(response))
     disconnect_ap.ParseFromString(response)
     #print("parsed output "+str(disconnect_ap.resp_disconnect_ap.mode )
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
    set_softap_config = slave_config_pb2.SlaveConfigPayload()
    set_softap_config.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdSetSoftAPConfig
    set_softap_config.cmd_set_softap_config.ssid = str(ssid)
    set_softap_config.cmd_set_softap_config.pwd = str(pwd)
    set_softap_config.cmd_set_softap_config.chnl = chnl
    set_softap_config.cmd_set_softap_config.ecn = ecn
    set_softap_config.cmd_set_softap_config.max_conn = max_conn
    set_softap_config.cmd_set_softap_config.ssid_hidden = ssid_hidden
    set_softap_config.cmd_set_softap_config.bw = bw
    protodata = set_softap_config.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,0.3)
    #print("response from slave "+str(response))
    set_softap_config.ParseFromString(response)
    #print("parsed output "+str(set_softap_config.cmd_set_softap_config.status )
    status = set_softap_config.resp_set_softap_config.status
    return status

def wifi_get_softap_config():
    get_softap_config = slave_config_pb2.SlaveConfigPayload()
    get_softap_config.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdGetSoftAPConfig
    protodata = get_softap_config.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,0.3)
    #print("response from slave "+str(response))
    get_softap_config.ParseFromString(response)
    #print("parsed output "+str(get_softap_config.resp_get_softap_config.status )
    ssid = str(get_softap_config.resp_get_softap_config.ssid)
    pwd = str(get_softap_config.resp_get_softap_config.pwd)
    en = get_softap_config.resp_get_softap_config.ecn
    chnl  = get_softap_config.resp_get_softap_config.chnl
    max_conn = get_softap_config.resp_get_softap_config.max_conn
    ssid_hidden  = get_softap_config.resp_get_softap_config.ssid_hidden
    status  = str(get_softap_config.resp_get_softap_config.status)
    bw = get_softap_config.resp_get_softap_config.bw
    return ssid,pwd,chnl,ecn,max_conn,ssid_hidden,status,bw

def wifi_ap_scan_list(scan_count):
    print(scan_count)
    get_ap_scan_list = slave_config_pb2.SlaveConfigPayload()
    get_ap_scan_list.msg = slave_config_pb2.SlaveConfigMsgType.TypeCmdGetAPScanList
    get_ap_scan_list.cmd_scan_ap_list.count = scan_count
    print(get_ap_scan_list.cmd_scan_ap_list.count )
    protodata = get_ap_scan_list.SerializeToString()
    #print("serialized data "+str(protodata))
    tp = transport.Transport_pserial(interface)
    response = tp.send_data(endpoint,protodata,10)
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
