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

import serial
import string
import time

# Default parameters
success = 'success'
failure = 'failure'

f1 = open("/dev/esps0","w")
f2 = open("/dev/esps0","r")

def check_ok(string):
    if (string == 'OK'):
        return 1
    else :
        return 0

def check_others(string,sub_string):
    #print(string)
    #print(sub_string)
    x = string.find(sub_string)
    #print(x)
    if (x == -1):
        return 0
    else:
        return 1

def check(string, sub_string):
    #print('Inside check function')
    #print(string)
    x = ' '.join([str(elem) for elem in string])
    x = string.split('\n')
    #print(x)
    x = [s for s in x if sub_string in s]
    #print(x)
    x = ' '.join([str(elem) for elem in x])
    #print(x)
    if ('OK' == sub_string):
        ret = check_ok(x)
        return ret
    else:
        ret = check_others(x,sub_string)
        return ret

def file_flush():
    f1.flush()
    f2.flush()
    return

def get_response(command,wait):
    file_flush()
    s = f1.write((command+'\r\n'))
    #print(s)
    time.sleep(wait)
    s = f2.read(2048)
    #print(s)
    return s

def check_response(ip,sp):
    ip = check(ip,sp)
    #print(ip)
    if (ip) :
        return success
    else :
        return failure

def test_at():
    command = 'AT'
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

def restart():
    command = 'AT+RST'
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

def get_version():
    command = 'AT+GMR'
    ret = get_response(command,0.2)
    res = check(ret,'OK')
    if (res == 1):
        #print(ret)
        #x = ' '.join([str(elem) for elem in ret])
        ##print(x)
        x = ret.split('\n')
        at_version = [s for s in x if 'AT version' in s]
        at_version = ' '.join([str(elem) for elem in at_version])
        sdk_version = [s for s in x if 'SDK version' in s]
        sdk_version = ' '.join([str(elem) for elem in sdk_version])
        custom_version = [s for s in x if 'custom_version' in s]
        custom_version = ' '.join([str(elem) for elem in custom_version])
        return at_version,sdk_version,custom_version
    else:
        return failure

def enter_deep_sleep_mode(sleep_in_ms):
    command = 'AT+GSLP='+str(sleep_in_ms)
    #print(command)
    time.sleep(sleep_in_ms*0.001)
    ret = get_response(command,0.2)
    ret = check_response(ret,'ready')
    return ret

def echo_on():
    command = 'ATE1'
    #print(command)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

def echo_off():
    command = 'ATE0'
    #print(command)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

def restore_factory():
    command = 'AT+RESTORE'
    #print(command)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

# Its a query command to get current uart configuration, not saved in flash
# This function sends baudrate, data_bits, stop bit,parity,flow_control
def get_current_uart_config():
    command = 'AT+UART_CUR?'
    ret = get_response(command,0.2)
    res = check(ret,'OK')
    if (res == 1):
        res = check(ret,'UART_CUR:')
        if (res == 1):
            x = ret.split('\n')
            #print(x)
            uart_config = [s for s in x if 'UART_CUR:' in s]
            x = ' '.join([str(elem) for elem in uart_config])
            x = x.split(',')
            flow_control = x[4]
            parity = x[3]
            stop_bit = x[2]
            data_bits = x[1]
            baudrate = x[0]
            baudrate = baudrate[10:]
            #print(x)
            return baudrate,data_bits,stop_bit,parity,flow_control
        else:
            return 'OK'
    else:
        return failure

# It sets current uart configuration, not saved in flash
# This function needs baudrate, data_bits, stop bit,parity,flow_control as input parameter from user
def set_current_uart_config(baudrate,data_bits,stop_bit,parity,flow_control):
    command = 'AT+UART_CUR='+str(baudrate)+','+str(data_bits)+','+str(stop_bit)+','+str(parity)+','+str(flow_control)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

# Its a query command to get default uart configuration, saved in flash
# This function sends baudrate, data_bits, stop bit, parity, flow_control
def get_default_uart_config():
    command = 'AT+UART_DEF?'
    ret = get_response(command,0.2)
    res = check(ret,'OK')
    if (res == 1):
        res = check(ret,'UART_DEF:')
        if (res == 1):
            x = ret.split('\n')
            #print(x)
            uart_config = [s for s in x if 'UART_DEF:' in s]
            x = ' '.join([str(elem) for elem in uart_config])
            x = x.split(',')
            flow_control = x[4]
            parity = x[3]
            stop_bit = x[2]
            data_bits = x[1]
            baudrate = x[0]
            baudrate = baudrate[10:]
            #print(x)
            return baudrate,data_bits,stop_bit,parity,flow_control
        else:
            return 'OK'
    else:
        return failure

# It sets current uart configuration, saved in flash
# This function needs baudrate, data bits, stop bit,parity,flow control as input parameter from user
def set_default_uart_config(baudrate,data_bits,stop_bit,parity,flow_control):
    command = 'AT+UART_DEF='+str(baudrate)+','+str(data_bits)+','+str(stop_bit)+','+str(parity)+','+str(flow_control)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

# Function sets sleep mode
# User can disable or enable the user mode
# 0: disable the sleep mode
# 1: Modem-sleep mode
def set_sleep_mode(val):
    command = 'AT+SLEEP='+str(val)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

# #print available space in RAM in bytes

def check_free_ram_size():
    command = 'AT+SYSRAM?'
    ret = get_response(command,0.2)
    res = check(ret,'OK')
    if (res == 1):
        res = check(ret, 'SYSRAM:')
        if (res == 1):
            x = ret.split('\n')
            #print(x)
            free_ram_size = [s for s in x if 'SYSRAM:' in s]
            x = ' '.join([str(elem) for elem in free_ram_size])
            free_ram_size = x[8:]
            return free_ram_size
        else:
            return 'OK'
    else:
        return failure

#wifi modes supported
#0: null Mode, Wi-Fi RF will be disabled
#1: station mode
#2: softAP mode
#3: softAP+station mode
def wifi_test():
    command = 'AT+CWMODE=?'
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret


# send the configured wifi mode
def wifi_get_mode():
    command='AT+CWMODE?'
    ret = get_response(command,0.2)
    #print("in wifi_get_imode")
    res = check(ret,'OK')
    if (res == 1) :
        res = check(ret, '+CWMODE:')
        if (res == 1):
            #print('OK received futher')
            x = ret.split('\n')
            #print('check x split'+str(x))
            wifi_mode = [s for s in x if '+CWMODE:' in s]
            #print(wifi_mode)
            wifi_mode = ' '.join([str(elem) for elem in wifi_mode])
            #print(wifi_mode)
            wifi_mode = wifi_mode[8:]
            #print(wifi_mode)
            return wifi_mode
        else:
            return 'OK'
    else :
        return failure

# set wifi_mode
# 0: null Mode, Wi-Fi RF will be disabled
# 1: station mode
# 2: softAP mode
# 3: softAP+station mode
def wifi_set_mode(mode):
    command = 'AT+CWMODE='+str(mode)
    #print("In wifi_set_mode")
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

def wifi_get_ap_config():
    command = 'AT+CWJAP?'
    ret = get_response(command,1)
    res = check(ret,'OK')
    #print("wifi_get_ap_config")
    if (res==1):
        #print('OK detected')
        res = check(ret,'+CWJAP:')
        if (res==1):
            x = ret.split('\n')
            #print(x)
            wifi_mode = [s for s in x if '+CWJAP:' in s]
            wifi_mode = ' '.join([str(elem) for elem in wifi_mode])
            wifi_mode = wifi_mode.split(',')
            #print(wifi_mode)
            rssi = wifi_mode[3]
            #print(rssi)
            channel = wifi_mode[2]
            #print(channel)
            bssid = wifi_mode[1]
            #print(bssid)
            wifi_mode = wifi_mode[0]
            ssid = wifi_mode[7:]
            #print(ssid)
            return ssid,bssid,channel,rssi
        else:
            return 'OK'
    else :
        return failure

# user can send bssid as 0, If its not in use
#user must send ssid, pwd, bssid of AP // bssid needed
def wifi_set_ap_config(ssid,pwd,bssid):
    if (str(bssid) != '0'):
        command = 'AT+CWJAP='+'"'+str(ssid)+'"'+','+'"'+str(pwd)+'"'+','+'"'+str(bssid)+'"'
    else :
        command = 'AT+CWJAP='+'"'+str(ssid)+'"'+','+'"'+str(pwd)+'"'
    #print(command)
    #print('wifi_set_ap_config')
    ret = get_response(command,2)
    ret = check_response(ret,'WIFI CONNECTED')
    return ret

#list of available access points
def wifi_get_ap_list():
    command = 'AT+CWLAP'
    ret = get_response(command,0.2)
    res = check(ret,'OK')
    if (res == 1):
        res = check(ret, '+CWLAP:')
        if (res == 1):
            #print(ret)
            wifi_mode = ret.split('\n')
            #print(wifi_mode)
            wifi_mode = [s for s in  wifi_mode if '+CWLAP:' in s]
            wifi_mode = ' '.join([str(elem) for elem in wifi_mode])
            wifi_mode = wifi_mode.split(',')
            return wifi_mode
        else:
            return 'OK'
    else:
        return failure

# get a query from specific ap
# bssid is mandatory parameter while mac_addr,channel are optional
# user can set mac_addr,channel as 0 if its not required
def wifi_get_spec_ap(ssid,mac_addr,chnl):
    if (str(mac_addr) != '0' and str(chnl) != '0'):
        command = 'AT+CWLAP='+'"'+str(ssid)+'"'+','+'"'+str(mac_addr)+'"'+','+str(chnl)
    if (str(mac_addr) != '0' and str(chnl) == '0'):
        command = 'AT+CWLAP='+'"'+str(ssid)+'"'+','+'"'+str(mac_addr)+'"'
    if (str(mac_addr) == '0' and str(chnl) != '0'):
        command = 'AT+CWLAP='+'"'+str(ssid)+'"'+','+','+'"'+str(chnl)+'"'
    if (str(mac_addr) == '0' and str(chnl) == '0'):
        command = 'AT+CWLAP='+'"'+str(ssid)+'"'
    #print(command)
    ret = get_response(command,0.2)
    res = check_response(ret,'OK')
    if (res == 1) :
        #print(ret)
        res = check_response(ret,'+CWLAP:')
        if (res == 1) :
            x = ret.split('\n')
            return ret
        else :
            return 'OK'
    else :
        return failure

# set config to list ap command
def wifi_set_config_list_ap(sort_enable,mask):
    command = 'AT+CWLAPOPT='+str(sort_enable)+','+str(mask)
    #print(command)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

# Disconnect AP
def wifi_disconnect_ap():
    command = 'AT+CWQAP'
    #print(command)
    ret = get_response(command,0.2)
    ret = check_response(ret,'OK')
    return ret

# user will get configuration of ESP32 softAP
# parameters will be ssid, password, channel ID, encription method,
# max connection count of stations to which ESP32 softAP can be connected, ssid hidden
def wifi_get_softap_config():
    command = 'AT+CWSAP?'
    ret = get_response(command,0.2)
    #print(command)
    res = check(ret,'OK')
    if (res == 1):
        res = check(ret,'+CWSAP:')
        if (res == 1):
            x = ret.split('\n')
            #print(x)
            ap_config = [s for s in x if '+CWSAP:' in s]
            ap_config = ' '.join([str(elem) for elem in ap_config])
            ap_config = ap_config.split(',')
            #print(ap_config)
            ssid_hidden = ap_config[5]
            max_conn = ap_config[4]
            encryp_mtd = ap_config[3]
            channel = ap_config[2]
            password = ap_config[1]
            password = password[1:-1]
            ap_config = ap_config[0]
            ssid = ap_config[7:]
            ssid = ssid[1:-1]
            return ssid,password,channel,encryp_mtd,max_conn,ssid_hidden
        else:
            return 'OK'
    else:
        return failure

# If no password needed then set it as 0
# set softap configuration of ESP32
# parameters should be ssid, password, channel ID, encription method,
# max connection count of stations to which ESP32 softAP can be connected, ssid hiddeni
# user can set max_con as 0 and ssid_hidden as 0 if they dont want to use
def wifi_set_softap_config(ssid,pwd,chl,ecn,max_conn,ssid_hidden):
    if (str(max_conn) != '0' and str(ssid_hidden) != '0'):
        command = 'AT+CWSAP='+'"'+str(ssid)+'"'+','+'"'+str(pwd)+'"'+','+str(chl)+','+str(ecn)+','+str(max_conn)+','+str(ssid_hidden)
    if (str(max_conn) != '0' and str(ssid_hidden) == '0'):
        command = 'AT+CWSAP='+'"'+str(ssid)+'"'+','+'"'+str(pwd)+'"'+','+str(chl)+','+str(ecn)+','+str(max_conn)
    if (str(max_conn) == '0' and str(ssid_hidden) != '0'):
        command = 'AT+CWSAP='+'"'+str(ssid)+'"'+','+'"'+str(pwd)+'"'+','+str(chl)+','+str(ecn)+','+','+str(ssid_hidden)
    #print(command)
    ret = get_response(command,2)
    ret = check_response(ret,'OK')
    return ret

#IP station to which ESP32 softAP is connected
def wifi_get_station_details():
    command = 'AT+CWLIF'
    ret = get_response(command,0.2)
    res = check(ret,'OK')
    if(res == 1) :
        res = check(ret,'+CWLIF:')
        if (res == 1) :
            x = ret.split('\n')
            #print(x)
            station_config = [s for s in x if '+CWLIF:' in s]
            station_config = ' '.join([str(elem) for elem in station_config])
            station_config = station_config.split(',')
            mac_addr = station_config[1]
            ip_addr = station_config[0]
            ip_addr = ip_addr[7:]
            #print(station_config)
            return ip_addr, mac_addr
        else:
            return 'OK'
    else:
        return failure

#get mac address of AP
def get_ap_mac():
    command = 'AT+CWAPMAC?'
    ret = get_response(command,2)
    res = check(ret,'OK')
    if (res == 1):
        res = check(ret,'+CWAPMAC:')
        if (res == 1):
            x = ret.split('\n')
            #print(x)
            mac_addr = [s for s in x if '+CWAPMAC:' in s]
            mac_addr = ' '.join([str(elem) for elem in mac_addr])
            #print(mac_addr)
            mac_addr = mac_addr.split(',')
            mac_addr = mac_addr[0]
            mac_addr = mac_addr[9:]
            mac_addr = mac_addr[1:-1]
            return mac_addr
        else:
            return 'OK'
    else:
        return failure

#get mac address of station
def get_sta_mac():
    command = 'AT+CWSTAMAC?'
    ret = get_response(command,2)
    res = check(ret,'OK')
    if (res == 1):
        res = check(ret,'+CWSTAMAC:')
        if (res == 1):
            x = ret.split('\n')
            #print(x)
            mac_addr = [s for s in x if '+CWSTAMAC:' in s]
            mac_addr = ' '.join([str(elem) for elem in mac_addr])
            mac_addr = mac_addr.split(',')
            mac_addr = mac_addr[0]
            mac_addr = mac_addr[10:]
            mac_addr = mac_addr[1:-1]
            #print("MAC"+str(mac_addr))
            return mac_addr
        else:
            return 'OK'
    else:
        return failure
