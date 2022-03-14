# Control Interface: C API's

- This document describes C API's provided for control path interface. Please refer [ctrl_api.h](../../host/control_lib/include/ctrl_api.h) for API definition
- [C Demo Application](c_demo.md) gives overview of how to test control path interface in application also how to perform stress testing of control path interface
- A [stress.c](../../host/linux/host_control/c_support/stress.c) can use for stress testing of control path commands. In which, `stress_test_count` variable represents the number of iterations the stress test should run for given configurations
- This document organized as
  - [1. Control path APIs](#1-control-path-apis)
    - These are APIs exposed by hosted control library
    - These are applicable for both MCU and MCP based solutions
	- Applications are free to choose how to use any API, in synchronous or asynchronous way
  - [2. Control path Events](#2-control-path-events)
  - [3. Function callbacks](#3-function-callbacks)
    - These are important structs used in case of asynchronous responses and events
  - [4. Data Structures](#4-data-structures)
    - Structures, which are used by the user application to interact with hosted control library
  - [5. Enumerations](#5-enumerations)
    - Enum values, which are used by the user application to interact with hosted control library

## 1. Control path APIs

### 1.1 int init_hosted_control_lib(void)

- This is the first thing that the application is expected to do before any other operations
- This function initializes hosted control library

#### Return

- 0 : `SUCCESS`
- -1 : `FAILURE`

---

### 1.2 int deinit_hosted_control_lib(void)

- This is a mandatory step, which will clean up the threads and any allocations made by the hosted control library

#### Return

- 0 : `SUCCESS`
- -1 : `FAILURE`

---

### 1.3 int set_event_callback(int event, [ctrl_event_cb_t](#32-typedef-int-ctrl_event_cb_t-ctrl_cmd_t-event) event_cb)

Applications can use this function to set event callback function for expected event

#### Parameters

- `int event` :
Event ID for control events from [AppMsgId_e](#58-enum-appmsgid_e-). Events supported are
  - ESP init : This indicates ESP has reset
  - Heartbeat : This indicates aliveness of ESP
  - station disconnected from AP : This active connection of ESP to AP _i.e._ external Wi-Fi router is lost
  - station disconnected from softAP : This means an external station, which was connected to ESP's softAP, has disconnected
- [ctrl_event_cb_t](#32-typedef-int-ctrl_event_cb_t-ctrl_cmd_t-event) `event_cb`
  - `Non-NULL` :
    - Callback function, which would be called by hosted control library when expected event received from ESP
    - This will start the notifications for `event`
  - `NULL` :
  Stops notifications for `event`

#### Return

- `MSG_ID_OUT_OF_ORDER` : In case of unrecognized or unsupported event
- `CALLBACK_SET_SUCCESS` : Event callback changed successfully

---

### 1.4 int reset_event_callback(int event)

Applications can use this function to reset event callback function for event ID

#### Parameters

- `int event` :
Event ID for control events from [AppMsgId_e](#58-enum-appmsgid_e-). This internally calls [set_event_callback()](#13-int-set_event_callbackint-event-ctrl_event_cb_t-event_cb) with NULL event callback

#### Return

- `MSG_ID_OUT_OF_ORDER` : In case of unrecognized event
- `CALLBACK_SET_SUCCESS` : On successful event de-registration
---

### 1.5 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_get_mac([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to get the MAC address of station or softAP interface of ESP

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - **`req.u.wifi_mac.mode`** :
    - Possible values
      - `WIFI_MODE_STA` : station
      - `WIFI_MODE_AP` : softAP
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`app_resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
  - **`app_resp->u.wifi_mac.mac`** :
  String in form of "XX:XX:XX:XX:XX:XX" in success case
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free
  - `ctrl_cmd_t *app_resp`

---

### 1.6 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_set_mac([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to set the MAC address of ESP interface for a given Wi-Fi mode

#### Note

- First set Wi-Fi mode before setting MAC address for respective station and softAP Interface
- ESP station and softAP have different MAC addresses, do not set them to be the same
- The bit 0 of the first byte of ESP MAC address can not be 1
        - For example, the MAC address can set to be "1a:XX:XX:XX:XX:XX", but can not be "15:XX:XX:XX:XX:XX"
- MAC address will get reset after esp restarts

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following
  - **`req.u.wifi_mac.mode`** :
    - `WIFI_MODE_STA` : station
    - `WIFI_MODE_AP` : softAP
  - **`req.wifi_mac.mac`** :
    - Custom MAC Address for ESP Interface. String in form of "XX:XX:XX:XX:XX:XX"
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.7 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_get_mode([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to get Wi-Fi mode of ESP

- `ctrl_cmd_t req` :
Control request as input with following
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
  - **`resp->wifi_mode.mode`** :
    - `WIFI_MODE_NONE` : NULL Mode, Wi-Fi mode not set
    - `WIFI_MODE_STA` : station mode
    - `WIFI_MODE_AP` : softAP mode
    - `WIFI_MODE_APSTA` : station+softAP mode
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.8 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_set_mode([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to set the Wi-Fi mode of ESP

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following
  - **`req.u.wifi_mode.mode`** :
    - `WIFI_MODE_NONE` : NULL Mode, Wi-Fi mode not set
    - `WIFI_MODE_STA` : station mode
    - `WIFI_MODE_AP` : softAP mode
    - `WIFI_MODE_APSTA` : station+softAP mode
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.9 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_set_power_save_mode([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to set Wi-Fi power save mode of ESP

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following
  - **`req.u.wifi_ps.ps_mode`** :
    - Possible values
      - `WIFI_PS_MIN_MODEM` :
      Minimum modem power saving. In this mode, the station wakes up to receive beacon every DTIM period
      - `WIFI_PS_MAX_MODEM` :
      Maximum modem power saving. In this mode, interval to receive beacons is determined by the `listen_interval` parameter in [wifi_connect_ap()](#112-ctrl_cmd_t-wifi_connect_apctrl_cmd_t-req) API
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Note
- Power save mode is set to 'WIFI_PS_MIN_MODEM' in sdkconfig (boot config)
- In Wi-Fi+BT/BLE mode in ESP-Hosted Firmware, 'WIFI_PS_NONE' i.e No power save mode is not supported
- Power save mode will be reverted to 'WIFI_PS_MIN_MODEM' on ESP reboot
- Application is expected to free `ctrl_cmd_t *app_resp`

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.10 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_get_power_save_mode([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to get the Wi-Fi power save mode of ESP

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
  - **`resp->u.wifi_ps.ps_mode`** :
    - `WIFI_PS_MIN_MODEM` :
    Minimum modem power saving. In this mode, the station wakes up to receive beacon every DTIM period
    - `WIFI_PS_MAX_MODEM` :
    Maximum modem power saving. In this mode, interval to receive beacons is determined by the `listen_interval` parameter in [wifi_connect_ap()](#112-ctrl_cmd_t-wifi_connect_apctrl_cmd_t-req) API
    - `WIFI_PS_INVALID` :
    Invalid power save mode. In case of failure of command
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.11 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_ap_scan_list([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)
This is used to get information of available neighbouring APs of ESP

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Although, default value is **120** sec, as this operation requires longer time to complete than other APIs
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return
- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
  - **`app_resp->u.wifi_ap_scan`** :
  Neighbouring AP list
    - **`app_resp->u.wifi_ap_scan.count`** :
    Number of available neighbouring APs
    - **`app_resp->u.wifi_ap_scan.out_list`** :
      - Dynamically allocated array of `wifi_scanlist_t` of scanned neighbouring APs
      - This pointer is also set in `app_resp->free_buffer_handle` for easy freeing by the application
      - `i` in following section indicates index running for `app_resp->u.wifi_ap_scan.count`
        - **`app_resp->u.wifi_ap_scan.out_list[i].ssid`**
          - SSID is the name of AP. It should be a string of length [0, 32]
          - 0 length means hidden SSID
        - **`app_resp->u.wifi_ap_scan.out_list[i].bssid`** :
          - BSSID or MAC address of AP
          - String of length to 17, excluding NULL character. Ex. "XX:XX:XX:XX:XX:XX"
        - **`app_resp->u.wifi_ap_scan.out_list[i].rssi`** :
          - RSSI signal strength
        - **`app_resp->u.wifi_ap_scan.out_list[i].channel`** :
          - WLAN Channel ID
        - **`app_resp->u.wifi_ap_scan.out_list[i].encryption_mode`** :
          - Encryption mode of AP
            - 0 : OPEN
            - 1 : WEP
            - 2 : WPA_PSK
            - 3 : WPA2_PSK
            - 4 : WPA_WPA2_PSK
            - 5 : WPA2_ENTERPRISE
            - 6 : WPA3_PSK
            - 7 : WPA2_WPA3_PSK
    - **`app_resp->free_buffer_handle`** :
      - `Non-NULL` - Application is expected to free this pointer using 'app_resp->free_buffer_func'
      - `NULL` - Don't care
    - **`app_resp->free_buffer_func`** :
      - `Non-NULL` - Application is expected to free 'app_resp->free_buffer_handle' using this function
      - `NULL` - Don't care
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### **Note**
- Application is expected to free
  - `app_resp->free_buffer_handle` using `app_resp->free_buffer_func`
  - `ctrl_cmd_t *app_resp`

---

### 1.12 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_connect_ap([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

- This is used to set the AP config to which ESP station should connect to
- Application can set up interface after connection is up \
Linux Demo application included showcases this
- As required, the DHCP lease need to be setup in Linux hosts


#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - **`req.u.wifi_ap_config`** :
  Configuration to be set on AP
    - **`req.u.wifi_ap_config.ssid`** :
      - SSID is the name of AP. It should be a string of length [0, 32]
      - 0 length means hidden SSID
    - **`req.u.wifi_ap_config.pwd`** :
      - Password or passphrase. This should be a string of 8 to 63 characters
    - **`req.u.wifi_ap_config.bssid`** :
      - MAC address of AP in case of multiple AP has same SSID. In case of unknown scenario,  "" should be passed
    - **`req.u.wifi_ap_config.is_wpa3_supported`** :
      - Status of WPA3 support present on AP. In case unknown, suggested to False
        - 0 : Unsupported
        - 1 : Supported
    - **`req.u.wifi_ap_config.listen_interval`** :
      - Listen Interval indicates how long the station will be *sleeping* without listening to any Beacon transmitted by the AP when the station enter in power save mode
      - This will be expressed in AP beacon intervals. This parameter will be in effect if `WIFI_PS_MAX_MODEM` is set
      - In case unsure, set this value to 0. This value defaults to 3 AP beacon interval if it is set to 0
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`app_resp->resp_event_status`** :
    - `SUCCESS` : 0
    - `NO_AP_FOUND` : 2 - Failed to search AP with given SSID name
    - `INVALID_PASSWORD` : 3 - AP could not connect with given password
    - `FAILURE` : anything else
  - `app_resp->u.wifi_ap_config.out_mac`
    - String in form of "XX:XX:XX:XX:XX:XX" in success case
	  - This string could be used to set up mac address for native host network interface
      - Linux Demo application showcase this
    - Linux users can set up the DHCP set over this
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- MPU (Linux) host
  - Interface UP
    - Once the AP is connected, In Linux app, getting interface up. Demo app shows a way to do it just for user convenience
    - But how in real system how it is to be done, is up to the user
  - DHCP
    - DHCP could be setup using `sudo dhclient ethsta0 -v`
    - Some Linux may not have same DHCP software package used in demo app
    - User can choose alternative software and way of implementation for that
- All hosts
  - Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.13 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_get_ap_config([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to get the AP config to which ESP station is connected. If ESP station is not connected to AP, returns `FAILURE` with error print 'station is not connected to AP'

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`app_resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - anything else : `FAILURE`
  - **`app_resp->u.wifi_ap_config`** :
  Get configuration of connected AP
    - **`app_resp->u.wifi_ap_config.ssid`** :
      - SSID of connected AP
      - string of length range [0,32]
      - 0 length means hidden SSID
    - **`app_resp->u.wifi_ap_config.bssid`** :
      - BSSID or MAC address of connected AP
      - String length of 17 excluding NULL character
    - **`app_resp->u.wifi_ap_config.channel`** :
      - WLAN channel ID of connected AP
    - **`app_resp->u.wifi_ap_config.rssi`** :
      - RSSI signal strength
    - **`app_resp->u.wifi_ap_config.encryption_mode`** :
      - Encryption/authentication mode of AP
        - 0 : OPEN
        - 1 : WEP
        - 2 : WPA_PSK
        - 3 : WPA2_PSK
        - 4 : WPA_WPA2_PSK
        - 5 : WPA2_ENTERPRISE
        - 6 : WPA3_PSK
        - 7 : WPA2_WPA3_PSK
    - **`char status[STATUS_LENGTH]`** :
      - Status of ESP station
        - "Failure" : Failed to get ESP station status
        - "Not connected" : ESP station is not connected to AP
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.14 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_disconnect_ap([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to disconnect ESP station from AP
In case of Linux host, The application can down the interface and release the DHCP lease after disconnection
Linux Demo application in Linux showcases a way to do this

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
   - **`app_resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - anything else : `FAILURE`
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- MPU (Linux) host
  - Interface Down
    - In Linux Demo Application interface down is showcased
    - Users can choose different way to down interface than shown
  - Release DHCP
    - Linux users can use `sudo dhclient ethsta0 -r` for releasing DHCP lease
    - Users are free to choose and use the software for DHCP
- All hosts
  - Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.15 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_start_softap([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to set configuration of ESP softAP and start broadcasting

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - **`req.u.wifi_softap_config`** :
  Set configuration of ESP softAP
    - **`req.u.wifi_softap_config.ssid`** :
      - SSID is the name of softAP. It should be a string of max length of 32
    - **`req.u.wifi_softap_config.pwd`** :
      - Password or passphrase. This should be a string of 8 to 63 characters
    - **`req.u.wifi_softap_config.channel`** :
      - WLAN channel ID
      - Supported channels for softAP are from 1 to 11
    - **`req.u.wifi_softap_config.encryption_mode`** :
      - Supported encryption *i.e.* authentication modes for softAP are
        - 0 : OPEN
        - 2 : WPA_PSK
        - 3 : WPA2_PSK
        - 4 : WPA_WPA2_PSK
    - **`req.u.wifi_softap_config.max_connections`** :
      - Software limit of maximum number of stations, which can connect to ESP softAP
      - This can have value from 1 to 10
      - Driver/Hardware current limit is 10
    - **`req.u.wifi_softap_config.ssid_hidden`** :
      - If softAP should broadcast its SSID or not
        - 0 : SSID be broadcasted
        - 1 : SSID not be broadcasted
    - **`req.u.wifi_softap_config.bandwidth`** :
      - Set bandwidth of ESP softAP
        - `WIFI_BW_HT20` : 20MHz Bandwidth
        - `WIFI_BW_HT40` : 40MHz Bandwidth
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
   - **`app_resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - anything else : `FAILURE`
  - **`app_resp->u.wifi_softap_config.out_mac`**
    - String in form of "XX:XX:XX:XX:XX:XX" in success case
    - This string could be used to set up mac address for native host network interface
    - Linux user can set up interface. Linux Demo application showcase a way to do this. User can choose other alternative way if need be

- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
-  MPU (Linux) host
  - Interface UP
    - Linux users can choose how to set up the MAC addresses, or use the way showcased in Linux Demo Application for `ethap0` interface
  - DHCP
    - Linux users are free to choose how to set up the IP address, be it static or DHCP
- All hosts
  - Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.16 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_get_softap_config([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to get configuration of ESP softAP

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return
- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
   - **`app_resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - anything else : `FAILURE`
  - **`app_resp->u.wifi_softap_config`** :
  Get configuration of ESP softAP
    - **`app_resp->u.wifi_softap_config.ssid`** :
      - SSID or name of softAP
      - 0 length means hidden SSID
    - **`app_resp->u.wifi_softap_config.pwd`** :
      - Password of softAP
    - **`app_resp->u.wifi_softap_config.channel`** :
      - Channel ID of softAP
    - **`app_resp->u.wifi_softap_config.encryption_mode`** :
      - Supported values for encryption or authentication are
        - 0 : OPEN
        - 2 : WPA_PSK
        - 3 : WPA2_PSK
        - 4 : WPA_WPA2_PSK
    - **`app_resp->u.wifi_softap_config.max_connections`** :
      - Software value of maximum number of stations can connect to softAP currently
    - **`app_resp->u.wifi_softap_config.ssid_hidden`** :
      - SoftAP is broadcasting its SSID or not
        - 0 : SSID is broadcasted
        - 1 : SSID is not broadcasted
    - **`app_resp->u.wifi_softap_config.bandwidth`** :
      - Current bandwidth of softAP
        - `WIFI_BW_HT20` : 20MHz Bandwidth
        - `WIFI_BW_HT40` : 40MHz Bandwidth
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.17 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_stop_softap([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to stop ESP softAP. Once stopped, user can down interface and release DHCP lease

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
   - **`app_resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - anything else : `FAILURE`
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- MPU (Linux) host
  - Interface Down
    - Linux users can choose how to down interface. Demo application showcases a way to do it
  - release DHCP
    - Linux users can reset static IP or release the DHCP lease as per their software
- All hosts
  - Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.18 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_get_softap_connected_station_list([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)
This is used to get information of connected stations to ESP softAP

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following API specific data set
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Although, default value is 30 sec, It is suggested to set this value to >=120 secs, as this operation requires longer time to complete
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return
- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`app_resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - anything else : `FAILURE`
  - **`app_resp->u.wifi_softap_con_sta`** :
  List of stations connected to ESP softAP using structure `wifi_softap_conn_sta_list_t`
  - **`app_resp->u.wifi_softap_con_sta.count`** :
    - Number of connected stations to ESP softAP
  - **`app_resp->u.wifi_softap_con_sta.out_list`** :
    - Dynamically allocated list of connected stations
    - This pointer is also set in `app_resp->free_buffer_handle` for easy freeing by the application
    - `i` in below section is index running for 0 to `app_resp->u.wifi_softap_con_sta.count`
  - **`app_resp->u.wifi_softap_con_sta.out_list[i].bssid`** :
    - BSSID or MAC address of AP
    - String of length to 17, excluding NULL character. Ex. "XX:XX:XX:XX:XX:XX"
  - **`app_resp->u.wifi_softap_con_sta.out_list[i].rssi`** :
    - RSSI signal strength
  - **`app_resp->free_buffer_handle`** :
    - `Non-NULL` - Application is expected to free this pointer using 'app_resp->free_buffer_func'
    - `NULL` - Don't care
  - **`app_resp->free_buffer_func`** :
    - `Non-NULL` - Application is expected to free 'app_resp->free_buffer_handle' using this function
    - `NULL` - Don't care
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free
  - `app_resp->free_buffer_handle` using `app_resp->free_buffer_func`
  - `ctrl_cmd_t *app_resp`

---

### 1.19 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_set_vendor_specific_ie([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)
Function set 802.11 Vendor-Specific Information Element. This function needs to get called before starting of ESP softAP

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following
  - **`req.u.wifi_softap_vendor_ie`** :
  Vendor IE data to be set
    - **`req.u.wifi_softap_vendor_ie.enable`** :
      - If true, specified IE is enabled. If false, specified IE is removed
    - **`req.u.wifi_softap_vendor_ie.type`** :
      - Information Element type. Determines the frame type to associate with the IE. Uses `wifi_vendor_ie_type_t` enum
    - **`req.u.wifi_softap_vendor_ie.idx`** :
      - Index to set or clear. Each IE type can be associated with up to two elements (indices 0 & 1). Uses `wifi_vendor_ie_id_t` enum
    - **`req.u.wifi_softap_vendor_ie.vnd_ie`** :
      - Pointer to vendor specific element data. First 6 bytes should be a header with members matching vendor_ie_data_t. If enable is false, this argument is ignored and can be NULL
    - **`req.u.wifi_softap_vendor_ie.vnd_ie_size`** :
      - size of vnd_ie data
  - **`req.free_buffer_func`** :
    - Function pointer to free `req.free_buffer_handle` by the hosted control library
  - **`req.free_buffer_handle`** :
    - Buffer handle of application, which is expected to be freed after hosted control library is done with the vendor buffer
    - This is a flexible way that the application can use to defer free pointer
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Please make sure `req.free_buffer_handle` and `req.free_buffer_func` is set to correct values
- Application is expected to free `ctrl_cmd_t *app_resp`

---


### 1.20 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_set_max_tx_power([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

Function sets maximum Wi-Fi transmitting power at ESP

#### Note
- The value set by this API will be mapped to the `max_tx_power` member of the structure `wifi_country_t` in ESP Wi-Fi driver
  - Mapping Table {`wifi_max_tx_power`, `max_tx_power`} = {{8,   2}, {20,  5}, {28,  7}, {34,  8}, {44, 11}, {52, 13}, {56, 14}, {60, 15}, {66, 16}, {72, 18}, {80, 20}}
  - Input parameter `req.u.wifi_tx_power.power` unit is 0.25dBm, range is [8, 84] corresponding to `2dBm to 20dBm`
  - Relationship between set value and actual value. As follows: {set value range, actual value} = {{[8,  19],8}, {[20, 27],20}, {[28, 33],28}, {[34, 43],34}, {[44, 51],44}, {[52, 55],52}, {[56, 59],56}, {[60, 65],60}, {[66, 71],66}, {[72, 79],72}, {[80, 84],80}}

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following
  - **`req.u.wifi_tx_power.power`** :
  Maximum Wi-Fi transmitting power
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - -1 : `FAILURE`
    - 5 : OUT_OF_RANGE. `wifi_max_tx_power` is not in range of [8, 84] corresponding to `2dBm to 20dBm` TX power
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Setting the Wi-Fi max TX power is just request. The underlying Wi-Fi driver would set the max power in accordance with multiple factors, which will be closest possible to requested power value
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.21 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * wifi_get_curr_tx_power([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

Function gets current Wi-Fi transmitting power at ESP

#### Note
- It is possible that the current Wi-Fi transmit power is lesser than that of the requested max transmit power as part of `wifi_set_max_tx_power` API


#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
  - **`app_resp->u.wifi_tx_power.power`** :
  Current Wi-Fi power expressed in 0.25 dBm. Please refer to the mapping table in `wifi_tx_power_t` above, for converting to dBm value
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.22 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * config_heartbeat([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

This is used to configure heartbeat event. Be default heartbeat is not enabled
To enable heartbeats, user need to use this API in addition to setting event callback for heartbeat event

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following
  - **`req.u.e_heartbeat.enable`**:
    - 1 - Enable
    - 0 - Disable
  - **`req.u.e_heartbeat.duration`**:
  Duration in seconds to set heartbeat event periodic timer
    - Value capped to 10 sec if value set less than 10 sec
    - Value capped to 3600 sec if value set greater than 3600 sec
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application also need to call [set_event_callback()](#13-int-set_event_callbackint-event-ctrl_event_cb_t-event_cb) in addition to this API to receive heartbeat events
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.23 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * ota_begin([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

- OTA begin function performs an OTA begin operation for ESP, which erases and prepares existing flash partition for new flash writing
- Although asynchronous procedure is supported, This is typically used as synchronous procedure, OTA begin success is expected before OTA write

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return
- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
      - Failure should be considered as complete OTA procedure failure
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.24 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * ota_write([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

- OTA write function performs an OTA write operation for ESP, It writes bytes from `ota_data` buffer with `ota_data_len` number of bytes to OTA partition in flash
- The number of bytes can be smaller than the size of the complete binary to be flashed
- In that case, this caller is expected to repeatedly call this function till total size written equals the size of the complete binary
- Although asynchronous procedure is supported, This is typically used as synchronous procedure, OTA write success is expected before remaining OTA write and/or OTA end procedure

#### Parameters

- `ctrl_cmd_t req` :
Control request as input with following
  - **`req.u.ota_write.ota_data`** :
    - OTA data buffer
  - **`req.u.ota_write.ota_data_len`** :
    - Length of OTA data buffer
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return

- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
      - Failure should be considered as complete OTA procedure failure
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---

### 1.25 [ctrl_cmd_t](#416-struct-ctrl_cmd_t) * ota_end([ctrl_cmd_t](#416-struct-ctrl_cmd_t) req)

OTA end function performs an OTA end operation for ESP, It validates written OTA image, sets newly written OTA partition as boot partition for next boot, creates timer, which reset ESP after 5 sec

#### Parameters
- `ctrl_cmd_t req` :
Control request as input with following
  - `req.ctrl_resp_cb` : optional
    - `NULL` :
      - Treat as synchronous procedure
      - Application would be blocked till response is received from hosted control library
    - `Non-NULL` :
      - Treat as asynchronous procedure
      - Callback function of type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp) is registered
      - Application would be will **not** be blocked for response and API is returned immediately
      - Response from ESP when received by hosted control library, this callback would be called
  - `req.cmd_timeout_sec` : optional
    - Timeout duration to wait for response in sync or async procedure
    - Default value is 30 sec
    - In case of async procedure, response callback function with error control response would be called to wait for response

#### Return
- `ctrl_cmd_t *app_resp` :
dynamically allocated response pointer of type struct `ctrl_cmd_t *`
  - **`resp->resp_event_status`** :
    - 0 : `SUCCESS`
    - != 0 : `FAILURE`
      - Failure should be considered as complete OTA procedure failure
- `NULL` :
  - Synchronous procedure: Failure
  - Asynchronous procedure:
    - Expected as NULL return value as response is processed in callback function
    - In callback function, parameter `ctrl_cmd_t *app_resp` behaves same as above

#### Note
- Application is expected to free `ctrl_cmd_t *app_resp`

---


### 1.26 int create_socket(int domain, int type, int protocol, int *sock)

- This API is only applicable in Unix based systems
- This is used to create an endpoint for communication

#### Parameters

- `domain` :
The domain argument specifies a communication domain (like AF_INET, AF_INET6), this selects the protocol family, which will be used for communication. These families are defined in <sys/socket.h>
- `type` :
This specifies the communication semantics (like SOCK_DGRAM, SOCK_STREAM)
- `protocol` :
This specifies a particular protocol to be used with the socket. Generally protocol value should be 0, please refer the man page of socket for more details
- `sock` :
This will return a file descriptor (integer number) that refers to that endpoint for the new socket on success or -1 on failure

#### Return

- 0 : `SUCCESS`
- != 0 : `FAILURE`

---

### 1.27 int close_socket(int sock)

- This API is only applicable in Unix based systems
- This is used to close an endpoint of the communication

#### Parameters

- `sock` :
This specifies the file descriptor of the endpoint/socket to be closed

#### Return

- 0 : `SUCCESS`
- != 0 : `FAILURE`

---

### 1.28 int set_hw_addr(int sockfd, char* iface, char* mac)

- This API is only applicable in Unix based systems
- Set ethernet interface MAC address `mac` to interface `iface`

#### Parameters

- `sockfd` :
This specifies the file descriptor of the endpoint/socket
- `char *iface` :
Ethernet interface name
- `char *mac` :
MAC address to be set

#### Return

- 0 : `SUCCESS`
- != 0 : `FAILURE`

---

### 1.29 int interface_up(int sockfd, char* iface)

- This API is only applicable in Unix based systems
- Get the interface up for interface `iface`

#### Parameters

- `sockfd` :
This specifies the file descriptor of the endpoint/socket
- `char *iface` :
Ethernet interface name

#### Return

- 0 : `SUCCESS`
- != 0 : `FAILURE`

---

### 1.30 int interface_down(int sockfd, char* iface)

- This API is only applicable in Unix based systems
- Get the interface down for interface `iface`

#### Parameters

- `sockfd` :
This specifies the file descriptor of the endpoint/socket
- `char *iface` :
Ethernet interface name

#### Return

- 0 : `SUCCESS`
- != 0 : `FAILURE`

---

## 2. Control path events
- Event are something that the application would subscribe to and get notification when some condition occurs. This way application doesnot have to poll for that condition
- Event subscribe
  - API [set_event_callback()](#13-int-set_event_callbackint-event-ctrl_event_cb_t-event_cb) is called with `event` and `callback function`
- Event unsubscribe
  - API [reset_event_callback()](#14-int-reset_event_callbackint-event) is called with `event`

### 2.1 ESP init
- When this subscribed, The application will be notified if ESP chipset is reset
- Generally when ESP reboots, it looses context set by host like connect to external AP, start softAP etc
- Host can make use of this event and re-do expected things
- This event also would let host know of any possible ESP silent crash. Please note, intentional ESP reboots also will trigger this event, so not all events of this kind are ESP crash

### 2.2 Heartbeat
- Application need to subscribe heartbeat event to get liveliness status of ESP
- Application need to configure heartbeat using API [config_heartbeat()](#122-ctrl_cmd_t-config_heartbeatctrl_cmd_t-req)
- This event notification will occur every duration seconds as mentioned in [config_heartbeat()](#122-ctrl_cmd_t-config_heartbeatctrl_cmd_t-req)
- Events are only notified if application subscribes this event **and** have configured heartbeat

### 2.3 Station disconnected from AP
- This event is useful to understand if any disconnection with external AP *i.e.* external Wi-Fi router
- Application can re-trigger station connect if the need be

### 2.4 Station disconnected from ESP SoftAP
- This event is useful to understand if any station disconnection with ESP softAP
- MAC address of station disconnecting is given to application

## 3. Function callbacks

### 3.1 typedef int (*ctrl_resp_cb_t) (ctrl_cmd_t * resp)
This is response callback pointer type. In case of synchronous communication, an instance of this type is used by the application to set response callback

---

### 3.2  typedef int (*ctrl_event_cb_t) (ctrl_cmd_t * event)
This is the event callback pointer type. Application uses this type to set callback with event id

---

## 4. Data Structures

### 4.1 _struct_ `wifi_ap_config_t`

- This is ESP station mode config. It has members for AP credentials to connect to and current status if ESP is connected to AP
- APIs that use this structure
  - [wifi_get_ap_config()](#113-ctrl_cmd_t-wifi_get_ap_configctrl_cmd_t-req)
  - [wifi_connect_ap()](#112-ctrl_cmd_t-wifi_connect_apctrl_cmd_t-req)

- `uint8_t ssid[SSID_LENGTH]` :
SSID is the name of AP to connect to. It should be a string of length 0 to 32
- `uint8_t pwd[PASSWORD_LENGTH]` :
Password or passphrase. It should be a string of length to 63
- `uint8_t bssid[BSSID_LENGTH]` :
BSSID or MAC address. It should be a string of length to 17. Ex. `XX:XX:XX:XX:XX:XX`
- `bool is_wpa3_supported` :
WPA3 support status of AP
  - 0 : Unsupported
  - 1 : Supported
- `int rssi` :
RSSI signal strength
- `int channel` :
WLAN Channel ID
- `int encryption_mode` :
Encryption or authentication mode of AP
  - 0 : OPEN
  - 1 : WEP
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
  - 5 : WPA2_ENTERPRISE
  - 6 : WPA3_PSK
  - 7 : WPA2_WPA3_PSK
- `uint16_t listen_interval` :
Listen Interval indicates how long the station will be *sleeping* without listening to any Beacon transmitted by the AP when the station enter in power save mode
This will be expressed in AP beacon intervals. This parameter will be in effect if `WIFI_PS_MAX_MODEM` is set. In case unknown, set this value to 0. This value defaults to 3 AP beacon interval if it is set to 0
- `char status[STATUS_LENGTH]` :
Status of ESP station
  - "Failure" : Failed to get ESP station status
  - "Not connected" : ESP station is not connected to AP
- `char out_mac[MAX_MAC_STR_LEN]` :
This member is only applicable in control API, [wifi_connect_ap()](#112-ctrl_cmd_t-wifi_connect_apctrl_cmd_t-req)
  - NULL string if API is successful
  - MAC address of ESP for Wi-Fi station mode
Only in connected or successful case, AP configurations will be valid

---
### 4.2 _struct_ `softap_config_t`:

- This is ESP softAP mode configuration. It has members to set and get ESP softAP configuration
- APIs that use this structure
  - [wifi_start_softap()](#115-ctrl_cmd_t-wifi_start_softapctrl_cmd_t-req)
  - [wifi_get_softap_config()](#116-ctrl_cmd_t-wifi_get_softap_configctrl_cmd_t-req)

- `uint8_t ssid[SSID_LENGTH]` :
SSID is of ESP softAP. It should be a string of length 0 to 32
- `uint8_t pwd[PASSWORD_LENGTH]` :
Password or passphrase. It should be a string of length 8 to 63
- `int channel` :
WLAN Channel ID of softAP. Supported channels for softAP are from 1 to 11
- `int encryption_mode` :
Encryption or authentication mode of softAP. Currently, ESP support only these encryption modes
  - 0 : OPEN
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
- `int max_connections` :
Software limit of maximum number of stations which can connect to ESP softAP
This can have value from 1 to 10
Driver/Hardware current limit is 10
- `bool ssid_hidden` :
  - 0 : SSID broadcasted
  - 1 : SSID not broadcasted
- _wifi_bandwidth_t_ `bandwidth` :
Bandwidth in 2.4 GHz microwave band
  - `WIFI_BW_HT20` : 20MHz Bandwidth
  - `WIFI_BW_HT40` : 40MHz Bandwidth
- `char out_mac[MAX_MAC_STR_LEN]` :
This member is only applicable in control API, [wifi_start_softap()](#115-ctrl_cmd_t-wifi_start_softapctrl_cmd_t-req)
  - NULL string if API is successful
  - MAC address of ESP for Wi-Fi station mode

---

### 4.3 _struct_ `wifi_scanlist_t` :

This structure gives information of external neighbouring AP of ESP
Member of struct `wifi_ap_scan_list_t`

- `uint8_t ssid[SSID_LENGTH]` :
SSID is the name of AP. It should be a string of length 0 to 32
- `uint8_t bssid[BSSID_LENGTH]` :
BSSID or MAC address of AP. It should be a string of length to 17. Ex. `XX:XX:XX:XX:XX:XX`
- `int rssi` :
RSSI signal strength
- `int channel` :
WLAN Channel ID
- `int encryption_mode` :
Encryption mode of AP
  - 0 : OPEN
  - 1 : WEP
  - 2 : WPA_PSK
  - 3 : WPA2_PSK
  - 4 : WPA_WPA2_PSK
  - 5 : WPA2_ENTERPRISE
  - 6 : WPA3_PSK
  - 7 : WPA2_WPA3_PSK

---

### 4.4 _struct_ `wifi_ap_scan_list_t` :

- This structure gives information of external neighbouring AP of ESP
- This is used for API [wifi_ap_scan_list()](#111-ctrl_cmd_t-wifi_ap_scan_listctrl_cmd_t-req)

- `int count` :
Number of APs found in scan
- `wifi_scanlist_t *out_list` :
Array of AP details found in scanning. This is dynamically allocated after scan and application is responsible to clean up

---

### 4.5 _struct_ `wifi_connected_stations_list_t`:

This contains the information of station(s) connected to ESP softAP
Member of struct [wifi_softap_conn_sta_list_t](#46-struct-wifi_softap_conn_sta_list_t-)

- `uint8_t bssid[BSSID_LENGTH]` :
BSSID or MAC address of station of length 17. Ex. "XX:XX:XX:XX:XX:XX"
- `int rssi` :
RSSI signal strength of station

---

### 4.6 _struct_ `wifi_softap_conn_sta_list_t` :

- This contains the information of station(s) connected to ESP softAP
- This is used in API [wifi_get_softap_connected_station_list()](#118-ctrl_cmd_t-wifi_get_softap_connected_station_listctrl_cmd_t-req)

- `int count` :
Number of stations connected
- `wifi_connected_stations_list_t *out_list` :
Array of station details connected. This is dynamically allocated after scan and application is responsible to clean up

---

### 4.7 _struct_ `wifi_mac_t` :

This structure contains the mac address
- Used for API [wifi_get_mac()](#15-ctrl_cmd_t-wifi_get_macctrl_cmd_t-req) or [wifi_set_mac()](#16-ctrl_cmd_t-wifi_set_macctrl_cmd_t-req)

- `int mode` :
Wi-Fi mode from enum [wifi_mode_e](#51-enum-wifi_mode_e)
- `char mac[MAX_MAC_STR_LEN]` :
Mac address

---

### 4.8 _struct_ `wifi_mode_t` :

- This structure contains the Wi-Fi mode
- Used in APIs [wifi_get_mode()](#17-ctrl_cmd_t-wifi_get_modectrl_cmd_t-req) and [wifi_set_mode()](#18-ctrl_cmd_t-wifi_set_modectrl_cmd_t-req)

- `int mode` :
Wi-Fi mode from enum [wifi_mode_e](#51-enum-wifi_mode_e)

---

### 4.9 _struct_ `wifi_power_save_t` :

- Used in Power save mode for APIs [wifi_set_power_save_mode()](#19-ctrl_cmd_t-wifi_set_power_save_modectrl_cmd_t-req) and [wifi_get_power_save_mode()](#110-ctrl_cmd_t-wifi_get_power_save_modectrl_cmd_t-req)

- `int ps_mode` :
Power save mode from enum [wifi_ps_type_e](#54-enum-wifi_ps_type_e-)

---

### 4.10 _struct_ `vendor_ie_data_t`:

This contains the vendor information element to ESP softAP
Member of struct [wifi_softap_vendor_ie_t](#411-struct-wifi_softap_vendor_ie_t)

- `uint8_t element_id` :
Should be set to WIFI_VENDOR_IE_ELEMENT_ID (0xDD)
- `uint8_t length` :
Length of all bytes in the element data (payload) following this member. Minimum 4(offset for `vendor_oui` and `vendor_oui_type` member)
- `uint8_t vendor_oui[3]` :
Vendor identifier (OUI)
- `uint8_t vendor_oui_type` :
Vendor-specific OUI type
- `uint8_t* payload` :
Payload. Length is equal to value in 'length' member, minus 4

---

### 4.11 _struct_ `wifi_softap_vendor_ie_t`:

- This contains the vendor information element to ESP softAP
- Used in API [wifi_set_vendor_specific_ie()](#119-ctrl_cmd_t-wifi_set_vendor_specific_iectrl_cmd_t-req)

- `bool enable` :
  - 0 - stop  softap vendor IE broadcast
  - 1 - start softap vendor IE broadcast
- `wifi_vendor_ie_type_e type` :
Type of vendor IR type from enum `wifi_vendor_ie_type_e`
- `vendor_ie_data_t *vnd_ie` :
Vendor IE data. This is dynamically allocated and application is responsible for clean up
- `uint16_t vnd_ie_size` :
This is actual size of vendor IE data inclusive of `sizeof(vendor_ie_data_t)` and user data size

---

### 4.12 _struct_ `ota_write_t`:

- This contains the ota_data pointer, from which the data of ota_data_len size is written to ESP flash
- ota_data_len can be maximum of 4000 bytes as limited by the serial driver maximum chunk size
- Used in API [ota_write](#124-ctrl_cmd_t-ota_writectrl_cmd_t-req)

- `uint8_t *ota_data` :
Data pointer to read and write to flash
- `uint32_t ota_data_len` :
total size to flash

---

### 4.13 _struct_ `wifi_tx_power_t`:

This contains the Wi-Fi power map value to be set or get using API [wifi_set_max_tx_power()](#120-ctrl_cmd_t-wifi_set_max_tx_powerctrl_cmd_t-req) or [wifi_get_curr_tx_power()](#121-ctrl_cmd_t-wifi_get_curr_tx_powerctrl_cmd_t-req)

- `int power` :
- The value set by this API will be mapped to the max_tx_power of the structure wifi_country_t variable in Wi-Fi driver
  - Mapping Table {wifi_max_tx_power, max_tx_power} = {{8,   2}, {20,  5}, {28,  7}, {34,  8}, {44, 11}, {52, 13}, {56, 14}, {60, 15}, {66, 16}, {72, 18}, {80, 20}}
  - Input parameter `wifi_max_tx_power` unit is 0.25dBm, range is [8, 84] corresponding to `2dBm to 20dBm`
  - Relationship between set value and actual value. As follows: {set value range, actual value} = {{[8,  19],8}, {[20, 27],20}, {[28, 33],28}, {[34, 43],34}, {[44, 51],44}, {[52, 55],52}, {[56, 59],56}, {[60, 65],60}, {[66, 71],66}, {[72, 79],72}, {[80, 84],80}}

---

### 4.14 _struct_ `event_heartbeat_t`:

- This contains the heartbeat configuration for request or event data
- Used in API [config_heartbeat()](#122-ctrl_cmd_t-config_heartbeatctrl_cmd_t-req)

- `uint32_t hb_num` :
Only applicable in case of event. It is an incremental number pushed by ESP every 'duration' if heartbeat is enabled
- `enable` :
Only applicable in case of request
  - 0 - start heartbeat from ESP
  - 1 - stop heartbeat from ESP
- `duration` :
Only applicable in case of request. Duration of heartbeat is range limited in \
[MIN_HEARTBEAT_INTERVAL, MAX_HEARTBEAT_INTERVAL] if falls out of range
  - `>= MIN_HEARTBEAT_INTERVAL` - 10 sec
  - `<= MAX_HEARTBEAT_INTERVAL` - 1 hour

---

### 4.15 _struct_ `event_station_disconn_t`:

- This contains the event data from ESP when any station is disconnected from ESP softap

- `int32_t reason` :
Reason why the station is disconnected
- `char mac[MAX_MAC_STR_LEN]` :
mac address of station disconnected in NULL terminated string

---
### 4.16 _struct_ `ctrl_cmd_t`:

This is the main structure exposed to application. Using this structure, the application would create control request or receive control response or receive control event

- `uint8_t msg_type` :
Control message type based on enum [AppMsgType_e](#57-enum-appmsgtype_e)
- `uint16_t msg_id` :
Control message ID based on enum [AppMsgId_e](#58-enum-appmsgid_e-)
- `uint8_t resp_event_status` :
status of control response or control event
  - 0 - successful response or event
  - != 0 - Failure response
- `union u` :
Union of some of the control message, which need extra data to be passed/parsed from request/response/event
- `int (*ctrl_resp_cb)(struct Ctrl_cmd_t *data)` : based on type [ctrl_resp_cb_t](#31-typedef-int-ctrl_resp_cb_t-ctrl_cmd_t-resp)
  - Default value for this callback is set to NULL
  - When this callback is set by the application while triggering the request, it will be automatically called asynchronously by hosted control library on receiving control response. In this case, the app will not be blocked on response
  - Whereas, when this is not set i.e. is NULL, it is understood as synchronous response, and app after sending request, will be blocked on response
- `int cmd_timeout_sec` :
  - Default value for this time out is DEFAULT_CTRL_RESP_TIMEOUT _i.e._ 30 seconds
  - Control response would be blocked for this timeout duration, if response not received, it will send timeout response
- `void *free_buffer_handle` :
  - In case of control response - This handle is set to valid data pointer to free by hosted control library so that when application is finished with processing, will clean up this handle using 'free_buffer_func()' at the end
  - In case of control request - This handle is set to valid data pointer to free by the application so that when hosted control library is finished with processing, will clean up this handle using 'free_buffer_func()' at the end
  - Ignored if assigned as NULL, assuming there is no data expected to be free
- `void (*free_buffer_func)(void *free_buffer_handle)` :
  - In case of control response - This handle is set to valid function pointer in association with Non-NULL 'free_buffer_handle' by hosted control library so that when application is finished with processing, will clean up this handle using this function at the end
  - In case of control request - This handle is set to valid function pointer in association with Non-NULL 'free_buffer_handle' by the application so that when hosted control library is finished with processing, will clean up this handle using this function at the end
  - Ignored if assigned as NULL, assuming there is no data expected to be free

---

## 5. Enumerations

### 5.1 _enum_ `wifi_mode_e` \
_Values:_
- `WIFI_MODE_NULL` = 0 :
Wi-Fi NULL or uninitialized mode
- `WIFI_MODE_STA` :
Wi-Fi station mode
- `WIFI_MODE_AP` :
Wi-Fi softAP mode
- `WIFI_MODE_APSTA` :
Wi-Fi station + softAP mode
- `WIFI_MODE_MAX`

---

### 5.2 _enum_ `wifi_auth_mode_e` :

Supported authentication or encryption mode in station, softAP mode. \
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

### 5.3 _enum_ `wifi_bandwidth_e` :

Bandwidth in 2.4 GHz band. \
_Values_ :
- `WIFI_BW_HT20` = 1 : 20MHz Bandwidth
- `WIFI_BW_HT40` : 40MHz Bandwidth

---

### 5.4 _enum_ `wifi_ps_type_e` :

Power save mode for ESP \
_Values_ :
- `WIFI_PS_MIN_MODEM` = 1 :
Minimum modem power saving. In this mode, the station wakes up to receive beacon every DTIM period
- `WIFI_PS_MAX_MODEM` :
Maximum modem power saving. In this mode, interval to receive beacons is determined by the `listen_interval` parameter in [wifi_connect_ap()](#112-ctrl_cmd_t-wifi_connect_apctrl_cmd_t-req) API
- `WIFI_PS_INVALID` :
Invalid power save mode

---

### 5.5 _enum_ `wifi_vendor_ie_type_e` :

Vendor information element type. Determines the frame type that the IE will be associated with. \
_Values_ :
- `WIFI_VND_IE_TYPE_BEACON` = 0 : Type beacon
- `WIFI_VND_IE_TYPE_PROBE_REQ` : Type probe request
- `WIFI_VND_IE_TYPE_PROBE_RESP` : Type probe response
- `WIFI_VND_IE_TYPE_ASSOC_REQ` : Type association request
- `WIFI_VND_IE_TYPE_ASSOC_RESP` : Type association response

---

### 5.6 _enum_ `wifi_vendor_ie_id_e` :

Vendor Information Element index. Each IE type can have up to two associated vendor ID elements. \
_Values_ :
- `WIFI_VND_IE_ID_0` = 0 : ID 0
- `WIFI_VND_IE_ID_1` : ID 1

### 5.7 _enum_ `AppMsgType_e`
Message type of control msg \
_Values_:
- `CTRL_MSGTYPE_INVALID` = 0
- `CTRL_REQ` = 1
- `CTRL_RESP` = 2
- `CTRL_EVENT` = 3
- `CTRL_MSGTYPE_MAX` = 4

#### Note
This structure is map to `CtrlMsgType` from `esp_hosted_config.pb-c.h`

### 5.8 _enum_ `AppMsgId_e` :
The message id of control msg. This could be one of request, response or event appmsgid_e \
_Values_:
#### 5.8.1 Requests
- `CTRL_REQ_BASE`                      = 100
- `CTRL_REQ_GET_MAC_ADDR`              = 101
- `CTRL_REQ_SET_MAC_ADDR`              = 102
- `CTRL_REQ_GET_WIFI_MODE`             = 103
- `CTRL_REQ_SET_WIFI_MODE`             = 104


- `CTRL_REQ_GET_AP_SCAN_LIST`          = 105
- `CTRL_REQ_GET_AP_CONFIG`             = 106
- `CTRL_REQ_CONNECT_AP`                = 107
- `CTRL_REQ_DISCONNECT_AP`             = 108


- `CTRL_REQ_GET_SOFTAP_CONFIG`         = 109
- `CTRL_REQ_SET_SOFTAP_VND_IE`         = 110
- `CTRL_REQ_START_SOFTAP`              = 111
- `CTRL_REQ_GET_SOFTAP_CONN_STA_LIST`  = 112
- `CTRL_REQ_STOP_SOFTAP`               = 113


- `CTRL_REQ_SET_PS_MODE`               = 114
- `CTRL_REQ_GET_PS_MODE`               = 115


- `CTRL_REQ_OTA_BEGIN`                 = 116
- `CTRL_REQ_OTA_WRITE`                 = 117
- `CTRL_REQ_OTA_END`                   = 118


- `CTRL_REQ_SET_WIFI_MAX_TX_POWER`     = 119
- `CTRL_REQ_GET_WIFI_CURR_TX_POWER`    = 120


- `CTRL_REQ_CONFIG_HEARTBEAT`          = 121
- `CTRL_REQ_MAX` = 122

#### 5.8.2 Responses
- `CTRL_RESP_BASE`                     = 200
- `CTRL_RESP_GET_MAC_ADDR`             = 201
- `CTRL_RESP_SET_MAC_ADDRESS`          = 202
- `CTRL_RESP_GET_WIFI_MODE`            = 203
- `CTRL_RESP_SET_WIFI_MODE`            = 204


- `CTRL_RESP_GET_AP_SCAN_LIST`         = 205
- `CTRL_RESP_GET_AP_CONFIG`            = 206
- `CTRL_RESP_CONNECT_AP`               = 207
- `CTRL_RESP_DISCONNECT_AP`            = 208


- `CTRL_RESP_GET_SOFTAP_CONFIG`        = 209
- `CTRL_RESP_SET_SOFTAP_VND_IE`        = 210
- `CTRL_RESP_START_SOFTAP`             = 211
- `CTRL_RESP_GET_SOFTAP_CONN_STA_LIST` = 212
- `CTRL_RESP_STOP_SOFTAP`              = 213


- `CTRL_RESP_SET_PS_MODE`              = 214
- `CTRL_RESP_GET_PS_MODE`              = 215


- `CTRL_RESP_OTA_BEGIN`                = 216
- `CTRL_RESP_OTA_WRITE`                = 217
- `CTRL_RESP_OTA_END`                  = 218


- `CTRL_RESP_SET_WIFI_MAX_TX_POWER`     = 219
- `CTRL_RESP_GET_WIFI_CURR_TX_POWER`    = 220


- `CTRL_RESP_CONFIG_HEARTBEAT`          = 221
- `CTRL_RESP_MAX` = 222

#### 5.8.3 Events
- `CTRL_EVENT_BASE`            = 300
- `CTRL_EVENT_ESP_INIT`        = 301
- `CTRL_EVENT_HEARTBEAT`       = 302
- `CTRL_EVENT_STATION_DISCONNECT_FROM_AP` = 303
- `CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP` = 304
- `CTRL_EVENT_MAX` = 305

#### Note
  This enum is mapping to `CtrlMsgId` from `esp_hosted_config.pb-c.h`

---
