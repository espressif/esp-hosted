# Control Interface API's: Python Implementation

This document describes python API's provided for control interface. Please refer [test.py](../host/linux/host_control/python_support/test.py) to get an idea how to use these API's.

## 1. `wifi_get_mac`
This is used to retrieve the MAC address of ESP's station or softAP interface

### Parameters

- `mode` :
  - 1: station
  - 2: softap

### Return
String in form of "XX:XX:XX:XX:XX:XX" with MAC address of ESP interface mapping to mode or "failure" string if failed.

---

## 2. `wifi_get_mode`
This is used to retrieve the ESP32's Wi-Fi mode

### Return
- 0: null Mode, Wi-Fi mode not set
- 1: station mode
- 2: softAP mode
- 3: softAP+station mode
- "failure" string: if failed.

---

## 3. `wifi_set_mode`
This is used to set the ESP32's Wi-Fi mode

### Parameters
- `mode` :
  - 0: null Mode, Wi-Fi mode not set
  - 1: station mode
  - 2: softAP mode
  - 3: softAP+station mode

### Return
"success" or "failure" string

---

## 4. `wifi_set_mac`
This is used to set MAC address for ESP's station or softap interface

### Parameters
- `mode` :
  - 1: station mode
  - 2: softAP mode
- `mac` :
string in form of "XX:XX:XX:XX:XX:XX"

### Return
"success" or "failure" string

### **Note**
- First set wifi mode before setting MAC address for respective station and softAP Interface
- ESP32 station and softAP have different MAC addresses, do not set them to be the same.
- The bit 0 of the first byte of ESP32 MAC address can not be 1.
For example, the MAC address can set to be "1a:XX:XX:XX:XX:XX", but can not be "15:XX:XX:XX:XX:XX".
- MAC address will get reset after esp restarts

---

## 5. `wifi_set_power_save_mode`
Set ESP32's power save mode

### Parameters
- `power_save_mode` :
   - 1: WIFI_PS_MIN_MODEM
Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period.
   - 2: WIFI_PS_MAX_MODEM
Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi set ap config function

### Return
"success" or "failure" string

### **Note**
* ESP32 on boot is configured in WIFI_PS_MIN_MODEM

---

## 6. `wifi_get_power_save_mode`
Get the power save mode of ESP32

### Return
- 1: WIFI_PS_MIN_MODEM
Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period.
- 2: WIFI_PS_MAX_MODEM
Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi set ap config function

### **Note**
ESP32 on boot is configured in WIFI_PS_MIN_MODEM

---

## 7. `wifi_set_ap_config`
Set the AP config to which ESP32 station should connect

### Parameters
- `ssid` :
string parameter, ssid of AP, max 32 bytes
- `pwd` :
string parameter, length of password should be 8~64 bytes ASCII
- `bssid` :
MAC address of AP, To differentiate between APs, In case multiple AP has same ssid
- `is_wpa3_supported` :
Status of WPA3 supplicant present on AP
  - False : Unsupported
  - True : Supported
- `listen_interval` :
Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set. Units: AP beacon intervals. Defaults to 3 if set to 0.

### Return
"success" or "failure" string

---

## 8. `wifi_get_ap_config`
Get the AP config to which ESP32 station is connected

### Return
*Successful case* : tuple of (ssid,bssid,channel,rssi,encryption_mode)
- `ssid` :
SSID of connected AP
- `bssid` :
MAC address of connected AP
- `channel` :
Channel ID, 1 ~ 10
- `rssi` :
RSSI signal strength
- `encryption_mode` :
  - 0 : OPEN
  - 1 : WEP
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
  - 5 : WPA2_ENTERPRISE
  - 6 : WPA3_PSK
  - 7 : WPA2_WPA3_PSK

*Failure cases*
- "not_connected" :
In case ESP32 is not connected to any AP
- "failure" string:
In case of transport failure

---

## 9. `wifi_disconnect_ap`
Disconnect the AP to which ESP32 station is connected

### Return
"success" or "failure" string

---

## 10. `wifi_ap_scan_list`
Set the AP config to which ESP32 station should connect

### Return
*Successful case* :
List of Aplist class instances(ssid,chnl,rssi,bssid,ecn)
- `ssid` :
SSID of AP
- `channel` :
Channel ID, in range of 1 to 10
- `bssid` :
MAC address of AP
- `rssi` :
RSSI signal strength
- `encryption_mode` :
  - 0 : OPEN
  - 1 : WEP
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
  - 5 : WPA2_ENTERPRISE
  - 6 : WPA3_PSK
  - 7 : WPA2_WPA3_PSK

*Failure case* :
- "failure" string

---

## 11. `wifi_set_softap_config`
Set the ESP32's softap config

### Parameters
- `ssid` :
String parameter, ssid of SoftAP
- `pwd` :
String parameter, length of password should be 8~64 bytes ASCII
- `chnl` :
Channel ID, In range of 1 to 11
- `ecn` :
Encryption method
  - 0 : OPEN
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
- `max_conn` :
Maximum number of stations can connect to ESP32 SoftAP (should be in range of 1 to 10)
- `ssid_hidden` :
SoftAP should broadcast its SSID or not
  - 0 : SSID should broadcast
  - 1 : SSID should not broadcast
- `bw` : set bandwidth of ESP32 softAP
  - 1 : WIFI_BW_HT20
  - 2 : WIFI_BW_HT40

### Return
"success" or "failure" string

---

## 12. `wifi_get_softap_config`
Get the ESP32's softap config

### Return
*Success case* :
returns (ssid,pwd,chnl,ecn,max_conn,ssid_hidden,bw)
- `ssid` : string parameter, ssid of SoftAP
- `pwd` : string parameter, length of password should be 8~64 bytes ASCII
- `chnl` : channel ID, In range of 1 to 11
- `ecn` : Encryption method
  - 0 : OPEN
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
- `max_conn` :
Maximum number of stations can connect to ESP32 SoftAP (will be in range of 1 to 10)
- `ssid_hidden` : softAP should broadcast its SSID or not
  - 0 : SSID is broadcasted
  - 1 : SSID is not broadcasted
- `bw` : bandwidth of ESP32 softAP
  - 1 : WIFI_BW_HT20
  - 2 : WIFI_BW_HT40

*Failure case* :
- "failure" string

---

## 13. `wifi_stop_softap`
Stop the ESP32's softap

### Return
"success" or "failure" string

---

## 14. `wifi_connected_stations_list`
Get the list of connected station to the ESP32 softap.

### Return

*success case* : list of Stationlist tuple(mac,rssi)
Stations credentials::
- `mac` :
MAC address of station
- `rssi` :
RSSI signal strength

*failure case*:
- "failure" string

---
