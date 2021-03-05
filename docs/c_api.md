# Control Interface: C API's

This document describes C API's provided for control interface. Please refer [test.c](../host/linux/host_control/c_support/test.c) to get an idea how to use these API's.

## 1. Data Structures

### 1.1 _struct_ `esp_hosted_sta_config_t`

This is ESP32 station mode config. It has fields for AP credentials to connect to and current status if ESP32 is connected to AP.

*Public Members*
- `uint8_t ssid[SSID_LENGTH]` :
SSID is name of AP to connect to. It should be string of length 0 to 32.
- `uint8_t pwd[PASSWORD_LENGTH]` :
Password or passphrase. It should be string of length to 63.
- `uint8_t bssid[BSSID_LENGTH]` :
BSSID or MAC address. It should be string of length to 17. ex. `XX:XX:XX:XX:XX:XX`.
- `bool is_wpa3_supported` :
WPA3 support status of AP.
  - 0 : Unsupported
  - 1 : Supported
- `int rssi` :
RSSI signal strength.
- `int channel` :
WLAN Channel ID.
- `int encryption_mode` :
Encryption or authentication mode of AP.
  - 0 : OPEN
  - 1 : WEP
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
  - 5 : WPA2_ENTERPRISE
  - 6 : WPA3_PSK
  - 7 : WPA2_WPA3_PSK
- `uint16_t listen_interval` :
Listen Interval indicates how long the station will be *sleeping* without listening to any Beacon transmitted by the AP when the station enter in power save mode.
This will be expressed in AP beacon intervals. This parameter will be in effect if `WIFI_PS_MAX_MODEM` is set. In case unknown, set this value to 0. This value defaults to 3 AP beacon interval if it is set to 0.
- `char status[STATUS_LENGTH]` :
Status of ESP32 station.
  - "Failure" : Failed to get ESP32 station status
  - "Not connected" : ESP32 station is not connected to AP.
Only in connected or successful case, AP configurations will be valid.

---
### 1.2 _struct_ `esp_hosted_softap_config_t`:

This is ESP32 softap mode configuration. It has fields to set and get ESP32 softap configuration.

*Public Members*
- `uint8_t ssid[SSID_LENGTH]` :
SSID is of ESP32 softap. It should be string of length 0 to 32.
- `uint8_t pwd[PASSWORD_LENGTH]` :
Password or passphrase. It should be string of length 8 to 63.
- `int channel` :
WLAN Channel ID of softap. Supported channels for softap are from 1 to 11.
- `int encryption_mode` :
Encryption or authentication mode of softap. Currently ESP32 support only these encryption mode.
  - 0 : OPEN
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
- `int max_connections` :
Software limit of maximum number of stations which can connect to ESP32 softap.
This can have value from 1 to 10.
Driver/Hardware current limit is 10.
- `bool ssid_hidden` :
  - 0 : SSID broadcasted
  - 1 : SSID not broadcasted
- _wifi_bandwidth_t_ `bandwidth` :
Bandwidth in 2.4 GHz microwave band.
  - `WIFI_BW_HT20` : 20MHz Bandwidth
  - `WIFI_BW_HT40` : 40MHz Bandwidth

---

### 1.3 _struct_ `esp_hosted_wifi_scanlist_t` :

This is structure gives information of external neighbouring AP of ESP32

*Public Members*
- `uint8_t ssid[SSID_LENGTH]` :
SSID is name of AP. It should be string of length 0 to 32.
- `uint8_t bssid[BSSID_LENGTH]` :
BSSID or MAC address of AP. It should be string of length to 17. ex. `XX:XX:XX:XX:XX:XX`.
- `int rssi` :
RSSI signal strength.
- `int channel` :
WLAN Channel ID.
- `int encryption_mode` :
Encryption mode of AP.
  - 0 : OPEN
  - 1 : WEP
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
  - 5 : WPA2_ENTERPRISE
  - 6 : WPA3_PSK
  - 7 : WPA2_WPA3_PSK

---

_struct_ `esp_hosted_wifi_connected_stations_list`:

This contains list of station(s) connected to ESP32 softap.

*Public Members*
- `uint8_t bssid[BSSID_LENGTH]` :
BSSID or MAC address of station of length 17. ex. "XX:XX:XX:XX:XX:XX".
- `int rssi` :
RSSI signal strength of station.

---
## 2. Functions

### 2.1 `int wifi_get_mac (int mode, char *mac)`

This is used to get the MAC address of station or softap interface of ESP32

#### Parameters

- `mode` :
        -`WIFI_MODE_STA` : station
        -`WIFI_MODE_AP` : softap
- `mac` :
String in form of "XX:XX:XX:XX:XX:XX" in success case
It should be large enough to store string in form "XX:XX:XX:XX:XX:XX"

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.2 `int wifi_set_mac (int mode, char *mac)`

This is used to set MAC address of ESP32 interface for given mode.

#### **Note**

- First set wifi mode before setting MAC address for respective station and softap Interface.
- ESP32 station and softap have different MAC addresses, do not set them to be the same.
- The bit 0 of the first byte of ESP32 MAC address can not be 1.
        - For example, the MAC address can set to be "1a:XX:XX:XX:XX:XX", but can not be "15:XX:XX:XX:XX:XX".
- MAC address will get reset after esp restarts

#### Parameters

- `mode` :
  - `WIFI_MODE_STA` : station
  - `WIFI_MODE_AP` : softap
- `mac` :
Custom MAC Address for ESP32 Interface. String in form of "XX:XX:XX:XX:XX:XX".

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.3 `int wifi_get_mode (int *mode)`

This is used to get Wi-Fi mode of ESP32

#### Parameters

- `mode` :
  - `WIFI_MODE_NONE` : null Mode, Wi-Fi mode not set
  - `WIFI_MODE_STA` : station mode
  - `WIFI_MODE_AP` : softap mode
  - `WIFI_MODE_APSTA` : softap+station mode

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.4 `int wifi_set_mode (int mode)`

This is used to set the Wi-Fi mode of ESP32

#### Parameters

- `mode` :
  - `WIFI_MODE_NONE` : null Mode, Wi-Fi mode not set
  - `WIFI_MODE_STA` : station mode
  - `WIFI_MODE_AP` : softap mode
  - `WIFI_MODE_APSTA` : softap+station mode

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.5 `int wifi_set_power_save_mode (int power_save_mode)`

This is used to set Wi-Fi power save mode of ESP32.

#### Parameters

- `power_save_mode` :
  - `WIFI_PS_MIN_MODEM` :
Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period.
  - `WIFI_PS_MAX_MODEM` :
Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi set ap config function.

#### **Note**
- Power save mode is set to 'WIFI_PS_MIN_MODEM' in sdkconfig (boot config)
- In WiFi+BT/BLE mode in ESP-Hosted Firmware, 'WIFI_PS_NONE' i.e No power save mode is not supported.

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.6 `int wifi_get_power_save_mode (int *power_save_mode)`

This is used to get the Wi-Fi power save mode of ESP32.

#### Parameters

- `power_save_mode` :
  - `WIFI_PS_MIN_MODEM` :
Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period.
  - `WIFI_PS_MAX_MODEM` :
Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi set ap config function
  - `WIFI_PS_INVALID` :
Invalid power save mode. In case of failure of command.

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.7 `int wifi_set_ap_config (esp_hosted_control_config_t ap_config)`

This is used to set the AP config to which ESP32 station should connect to.

#### Parameters

- `ap_config` :
Configuration to be set on AP
- `ssid` :
SSID i.e. Name of AP. This should be a string upto 32 characters.
- `pwd` :
Password or passphrase. This should be string of 8 to 63 characters.
- `bssid` :
MAC address of AP in case of multiple AP has same SSID. In case of unknown scenario,  "" should be passed
- `is_wpa3_supported` :
Status of WPA3 support present on AP. In case unknown, keep False.
  - 0 : Unsupported
  - 1 : Supported
- `listen_interval` :
Listen Interval indicates how long the station will be *sleeping* without listening to any Beacon transmitted by the AP when the station enter in power save mode.
This will be expressed in AP beacon intervals. This parameter will be in effect if `WIFI_PS_MAX_MODEM` is set. In case unknown, set this value to 0. This value defaults to 3 AP beacon interval if it is set to 0.

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.8 `int  wifi_get_ap_config(esp_hosted_control_config_t*  ap_config)`

This is used to get the AP config to which ESP32 station is connected. If ESP32
station is not connected to AP, returns FAILURE with error print 'station is not connected to AP'.

#### Parameters
- `ap_config` :
Get configuration of connected AP.
- `ssid` :
SSID of connected AP
- `bssid` :
MAC address of connected AP
- `channel` :
channel ID of connected AP
- `rssi` :
RSSI signal strength
- `encryption_mode` :
encryption/authentication mode of AP, a value from enum `wifi_auth_mode_t`

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.9 `int wifi_disconnect_ap()`

This is used to disconnect ESP32 station from AP.

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.10 `int wifi_set_softap_config(esp_hosted_control_config_t softap_config)`

This is used to set configuration of ESP32 softap .

#### Parameters
- `softap_config` :
Set configuration of ESP32 softap
- `ssid` :
SSID i.e. Name of AP. This should be a string upto 32 characters.
- `pwd` :
Password or passphrase. This should be string of 8 to 63 characters.
- `channel` :
WLAN channel ID. Supported channels for softap are from 1 to 11.
- `encryption_mode` :
Supported encryption *i.e.* authentication modes for softap are:
  - 0 : OPEN
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
- `max_connections` :
Software limit of maximum number of stations which can connect to ESP32 softap.
This can have value from 1 to 10.
Driver/Hardware current limit is 10.
- `ssid_hidden` :
If softap should broadcast its SSID or not
  - 0 : SSID should be broadcasted
  - 1 : SSID should not be broadcasted
- `bandwidth` : set bandwidth of ESP32 softap
  - `WIFI_BW_HT20` : 20MHz Bandwidth
  - `WIFI_BW_HT40` : 40MHz Bandwidth

#### Return

- 0 : SUCCESS
- -1 : FAILURE

---

### 2.11 `int wifi_get_softap_config(esp_hosted_control_config_t *softap_config)`

This is used to get configuration of ESP32 softap .

#### Parameters

- `softap_config` :
Get configuration of ESP32 softap
- `ssid` : SSID or name of softap
- `pwd` : password of softap
- `channel` : channel ID of softap
- `encryption_mode` : encryption mode or authentication mode softap is using
  - 0 : OPEN
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
- `max_connections` : software value of maximum number of stations can connect to softap currently
- `ssid_hidden` : softap is broadcasting its SSID or not
  - 0 : SSID is broadcasted
  - 1 : SSID is not broadcasted
- `bandwidth` : current bandwidth of softap
  - `WIFI_BW_HT20` : 20MHz Bandwidth
  - `WIFI_BW_HT40` : 40MHz Bandwidth

#### Return
- 0 : SUCCESS
- -1 : FAILURE

---

### 2.12 `int wifi_stop_softap()`

This is used to stop ESP32 softap.

#### Return
- 0 : SUCCESS
- -1 : FAILURE

---

### 2.13 `esp_hosted_wifi_scanlist_t* wifi_ap_scan_list(int *count)`
This is used to get list of available neighboring APs of ESP32.

#### Parameters
- `count` :
number of available neighboring APs.

#### Return
`esp_hosted_wifi_scanlist_t` handler :
Handler is pointer to list of scanned neighboring APs.

#### **Note**
User should free `esp_hosted_wifi_scanlist_t` handler after use.

---

### 2.14 `esp_hosted_wifi_connected_stations_list*  wifi_connected_stations_list(int *num)`
This is used to get list of connected stations to ESP32 softap.

#### Parameters
- `num` :
number of stations connected

#### Return
`esp_hosted_wifi_connected_stations_list` handler :
Handler is pointer to list of connected stations to ESP32 softap.

#### **Note**
User should free `esp_hosted_wifi_connected_stations_list` handler after use.

---

## 3. Enumerations

### 3.1 _enum_ `wifi_mode_t`
_Values:_
- `WIFI_MODE_NULL` = 0 :
Wi-Fi null or uininitialized mode
- `WIFI_MODE_STA` :
WiFi station mode
- `WIFI_MODE_AP` :
WiFi softap mode
- `WIFI_MODE_APSTA` :
WiFi station + softap mode
- `WIFI_MODE_MAX`

---

### 3.2 _enum_ `wifi_auth_mode_t` :

Supported authentication or encryption mode in station, softap mode.
_Values_:
- `WIFI_AUTH_OPEN` = 0 : Open mode
- `WIFI_AUTH_WEP` : WEP mode
- `WIFI_AUTH_WPA_PSK` : WPA_PSK mode
- `WIFI_AUTH_WPA2_PSK` : WPA2_PSK mode
- `WIFI_AUTH_WPA_WPA2_PSK` : WPA_WPA2_PSK mode
- `WIFI_AUTH_WPA2_ENTERPRISE` : WPA2_ENTERPRISE mode
- `WIFI_AUTH_WPA3_PSK` : WPA3_PSK mode
- `WIFI_AUTH_WPA2_WPA3_PSK` : WPA2_WPA3_PSK mode

---

### 3.3 _enum_ ` wifi_bandwidth_t` :

Bandwidth in 2.4 GHz band.
_Values_ :
- `WIFI_BW_HT20` = 1 : 20MHz Bandwidth
- `WIFI_BW_HT40` : 40MHz Bandwidth

---

### 3.4 _enum_ `wifi_ps_type_t` :

Power save mode for ESP32
_Values_ :
- `WIFI_PS_MIN_MODEM` = 1 :
Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period.
- `WIFI_PS_MAX_MODEM` :
Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi set ap config function.
- `WIFI_PS_INVALID` :
Invalid power save mode

---

## 4. Unions

### 4.1 _union_ `esp_hosted_control_config_t` :

This is ESP32 control configuration union. Its used for providing configurations to ESP32 station or softap mode.

*Public Members*
- _struct_ `esp_hosted_sta_config_t` :
ESP32 station mode config
- _struct_ `esp_hosted_softap_config_t` :
ESP32 softap mode config

---
