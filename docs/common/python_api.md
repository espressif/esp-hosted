# Control Interface API's: Python Implementation

This document describes python API's provided for control interface.
[Python based Demo App](python_demo.md) gives overview of how to test control path interface in application also how to perform stress testing of control path interface.


A [stress.py](../host/linux/host_control/python_support/stress.py) can use for stress testing of control path commands. In which, `STRESS_TEST_COUNT` variable represents number of iterations and `STRESS_TEST` variable defines which test should get executed.

## 1. `wifi_get_mac(mode)`
This is used to retrieve the MAC address of ESP's station or softAP interface

### Parameters

- `mode` :
  - 1: station
  - 2: softAP

### Return
String in form of "XX:XX:XX:XX:XX:XX" with MAC address of ESP interface mapping to mode or "failure" string if failed.

---

## 2. `wifi_get_mode()`
This is used to retrieve the ESP32's Wi-Fi mode

### Return
- 0: null Mode, Wi-Fi mode not set
- 1: station mode
- 2: softAP mode
- 3: station+softAP mode
- "failure" string: if failed.

---

## 3. `wifi_set_mode(mode)`
This is used to set the ESP32's Wi-Fi mode

### Parameters
- `mode` :
  - 0: null Mode, Wi-Fi mode not set
  - 1: station mode
  - 2: softAP mode
  - 3: station+softAP mode

### Return
"success" or "failure" string

---

## 4. `wifi_set_mac(mode, mac)`
This is used to set MAC address for ESP's station or softAP interface

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

## 5. `wifi_set_power_save_mode(power_save_mode)`
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

## 6. `wifi_get_power_save_mode()`
Get the power save mode of ESP32

### Return
- 1: WIFI_PS_MIN_MODEM
Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period.
- 2: WIFI_PS_MAX_MODEM
Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi set ap config function

### **Note**
ESP32 on boot is configured in WIFI_PS_MIN_MODEM

---

## 7. `wifi_set_ap_config(ssid, pwd, bssid, is_wpa3_supported, listen_interval)`
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

## 8. `wifi_get_ap_config()`
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

## 9. `wifi_disconnect_ap()`
Disconnect the AP to which ESP32 station is connected

### Return
"success" or "failure" string

---

## 10. `wifi_ap_scan_list()`
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

## 11. `wifi_set_softap_config(ssid, pwd, chnl, ecn, max_conn, ssid_hidden, bw)`
Set the ESP32's softAP config

### Parameters
- `ssid` :
String parameter, ssid of softAP
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
Maximum number of stations can connect to ESP32 softAP (should be in range of 1 to 10)
- `ssid_hidden` :
softAP should broadcast its SSID or not
  - 0 : SSID should broadcast
  - 1 : SSID should not broadcast
- `bw` : set bandwidth of ESP32 softAP
  - 1 : WIFI_BW_HT20
  - 2 : WIFI_BW_HT40

### Return
"success" or "failure" string

---

## 12. `wifi_get_softap_config()`
Get the ESP32's softAP config

### Return
*Success case* :
returns (ssid,pwd,chnl,ecn,max_conn,ssid_hidden,bw)
- `ssid` : string parameter, ssid of softAP
- `pwd` : string parameter, length of password should be 8~64 bytes ASCII
- `chnl` : channel ID, In range of 1 to 11
- `ecn` : Encryption method
  - 0 : OPEN
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
- `max_conn` :
Maximum number of stations can connect to ESP32 softAP (will be in range of 1 to 10)
- `ssid_hidden` : softAP should broadcast its SSID or not
  - 0 : SSID is broadcasted
  - 1 : SSID is not broadcasted
- `bw` : bandwidth of ESP32 softAP
  - 1 : WIFI_BW_HT20
  - 2 : WIFI_BW_HT40

*Failure case* :
- "failure" string

---

## 13. `wifi_stop_softap()`
Stop the ESP32's softAP

### Return
"success" or "failure" string

---

## 14. `wifi_connected_stations_list()`
Get the list of connected station to the ESP32 softAP.

### Return

*success case* : list of Stationlist tuple(bssid,rssi)
Stations credentials::
- `bssid` :
MAC address of station
- `rssi` :
RSSI signal strength

*failure case*:
- "failure" string

---

## 15. `create_socket(domain, types, protocol)`
This function creates an endpoint for communication

### Parameters
- `domain` :
This specifies a communication domain (like AF_INET, AF_INET6).
- `types` :
This specifies the communication semantics (like SOCK_DGRAM, SOCK_STREAM).
- `protocol` :
This specifies a particular protocol to be used with the socket. Generally protocol value should be 0, please refer socket documentation for more details.

### Return
- File descriptor (integer number) that refers to that endpoint/socket
- "failure" string: if failed.

---

## 16. `close_socket(sock)`
This function closes endpoint of the communication

### Parameters
- `sock` :
This specifies the file descriptor of the endpoint/socket to be closed

### Return
- "success" or "failure" string

---

## 17. `wifi_set_max_tx_power(wifi_max_tx_power)`

Function sets maximum WiFi transmitting power at ESP32.

#### **Note**
- The value set by this API will be mapped to the max_tx_power of the structure wifi_country_t variable in wifi driver.
- Mapping Table {wifi_max_tx_power, max_tx_power} = {{8,   2}, {20,  5}, {28,  7}, {34,  8}, {44, 11}, {52, 13}, {56, 14}, {60, 15}, {66, 16}, {72, 18}, {80, 20}}.
- Input parameter `wifi_max_tx_power` unit is 0.25dBm, range is [8, 84] corresponding to `2dBm to 20dBm`.
- Relationship between set value and actual value. As follows: {set value range, actual value} = {{[8,  19],8}, {[20, 27],20}, {[28, 33],28}, {[34, 43],34}, {[44, 51],44}, {[52, 55],52}, {[56, 59],56}, {[60, 65],60}, {[66, 71],66}, {[72, 79],72}, {[80, 84],80}}.

### Parameters
- `wifi_max_tx_power` :
Maximum WiFi transmitting power.

### Return
*Success case* :
"success" string

*Failure case* :
"failure" string

*Out of range case*:
"out_of_range" string. `wifi_max_tx_power` is not in range of [8, 84] corresponding to `2dBm to 20dBm` tx power.

---

### 18. `wifi_get_curr_tx_power()`

Function gets current WiFi transmiting power at ESP32.

### Return
*Success case* :
returns Current WiFi transmitting power, unit is 0.25dBm. It is possible that the current wifi transmit power is lesser than that of the requested max transmit power as part of `wifi_set_max_tx_power` API.

*Failure case* :
- "failure" string

---

## 19. `esp_ota_begin()`
esp ota begin function performs an OTA begin operation for ESP32 which erases and prepares existing flash partition for new flash writing.

### Return

"success" or "failure" string

---

## 20. `esp_ota_write(ota_data, ota_data_len)`
esp ota write function performs an OTA write operation for ESP32, It writes bytes from `ota_data` buffer with `ota_data_len` number of bytes to OTA partition in flash. Number of bytes can be small than size of complete binary to be flashed. In that case, this caller is expected to repeatedly call this function till total size written equals size of complete binary.
### Parameters

- `ota_data` : OTA data buffer
- `ota_data_len` : length of OTA data buffer

### Return

"success" or "failure" string

---

## 21. `esp_ota_end()`
esp ota end function performs an OTA end operation for ESP32, It validates written OTA image, sets newly written OTA partition as boot partition for next boot, Creates timer which reset ESP32 after 5 sec.

### Return

"success" or "failure" string

---
