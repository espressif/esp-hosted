# Getting started with STM32F469I(MCU based Host)

## Wi-Fi Connectivity

Host firmware provides wifi connectivity using control path and data path. Control path commands uses `protocomm` layer of ESP-IDF to serialize structured control data and communicates using SPI transport interface between Host(MCU based Host) and Slave(ESP32). User can use control commands to build application.

### Control Path Commands
[commands.c](../../host/host_common/commands.c) is control path commands C library. It implements the communication protocol between the host and ESP32. It contains following functions which can be used to control Wi-Fi functionality of the ESP32 as follows:
| Function | Functionality |
|:--------|:-------------|
| wifi_get_mac(int mode, char* mac) | get MAC address of station or softAP Interface |
| wifi_get_mode(int* mode) | get wifi mode |
| wifi_set_mode(int mode) | set wifi mode |
| wifi_set_ap_config(esp_hosted_ap_config_t ap_config) | connect to AP as a station |
| wifi_get_ap_config (esp_hosted_ap_config_t* ap_config) | get AP configuration and status |
| wifi_disconnect_ap () | disconnect from AP |
| wifi_set_softap_config (esp_hosted_ap_config_t softap_config) | start softAP |
| wifi_get_softap_config (esp_hosted_ap_config_t* softap_config) | get softAP configuration |
| wifi_ap_scan_list(esp_hosted_wifi_scanlist_t** list, int* count) | scan available APs |
| wifi_connected_stations_list(esp_hosted_wifi_connected_stations_list** list, int* num) | list stations connected to softAP |
| wifi_set_mac(int mode, char* mac) | sets custom mac address for station and softAP Interface |
| wifi_set_power_save_mode(int power_save_mode) | set power save mode |
| wifi_get_power_save_mode(int* power_save_mode) | get power save mode |

Above function's parameters and description is present [here](../../host/host_common/include/commands.h).

### Start Project with STM32

We have tested project with STM32F469I-Discovery board. If other than STM32F469I-Discovery board is used, peripheral like SPI, USART need to change as per board needs. As mentioned in [Setup.md](Setup.md), STM32CubeIDE would be needed to follow next steps.

1) Create a workspace_directory outside of `ESP-Hosted` git cloned directory.
2) Browse and Open Workspace directory in STM32CubeIDE. It will take few seconds to open STM32CubeIDE.
3) From `Information Center` tab select Start new project from existing STM32CubeMX configuration file, i.e. ioc file option. It will take few seconds to open dialog box. In STM32CubeMX .ioc file field, browse to `</path/to/esp_hosted>/host/stm32/proj/stm_spi_host.ioc -> Open` and click on `finish` icon. New dialog box will open as Open Associated Perspective, click on `Yes`. It may take 2-3 minutes to open.
4) Close `stm_spi_host.ioc` tab then close STM32CubeIDE and click on `exit`.
5) For Linux and Mac development hosts, In terminal, run
```
$ cd </path/to/esp_hosted>/host/stm32/proj
$ bash ./prepare_project.sh </path/to/workspace_directory>
```
For Windows based systems, open "cmd.exe" or Windows Power Shell and run -
```
> cd <path\to\esp_hosted>\host\stm32\proj
> prepare_project.bat <path\to\workspace_directory>
```
This will copy the project configuration files into workspace_directory

6) Re-open STM32CubeIDE with workspace as workspace_directory.
7) Ignore all warnings under `Problems` tab if any.
8) Configure all build variables as mentioned in [User configuration parameter](#user-configuration-parameter) section below. Variable CODE_BASE should already be populated. All parameters are mandatory to be filled. Please note that, every subsequent change in configuration parameter would need a clean build as mentioned ahead.
9) Uncheck `Build Automatically` from `menu -> Project` and Clean build the project as `Project menu -> Clean -> clean`.
10) Connect STM32 board to your machine if not already connected.
11) Before flashing the project, Open Tera Term or minicom to see STM32 debug logs once project flashed.
```
$ minicom -D /dev/ttyACM0
```
Note: /dev/ttyACM0 is used for For Linux, /dev/cu.usbmodemXXXXXX for Mac and COM port for Windows development host. Baud rate used is 115200. Parity bits configuration is 8N1.

12) In STM32CubeIDE, go to `Project Explorer`, right click on `stm_spi_host` project. Then `menu -> Run -> Run as -> STM32 Cortex-M C/C++ Application `. This will open Edit Configuration box, Click `OK`.
`Debug as` option can also be alternatively used if debugging is desired.

Expected output on `Console` tab in STM32CubeIDE as follows:

```
Download in Progress:

File download complete
Time elapsed during download operation: 00:00:02.277

Verifying ...

Download verified successfully

Debugger connection lost.
Shutting down...
```

13) Expected output log on tera term or minicom will be similar to below:
```

+-----------------------------------+-------------------------------------------+
|           Parameters              |             Values                        |
+-----------------------------------+-------------------------------------------+
|      INPUT__OPERATING_MODE        |             STATION+SOFTAP                |
|      INPUT_GET_AP_SCAN_LIST       |             yes                           |
+-----------------------------------+-------------------------------------------+
|            SOFTAP                 |                                           |
+-----------------------------------+-------------------------------------------+
|        INPUT_SOFTAP__SSID         |             ESPWifi                       |
|       INPUT_SOFTAP_PASSWORD       |             ESPWifi@123                   |
|      INPUT_SOFTAP_BANDWIDTH       |             HT40                          |
|       INPUT_SOFTAP_CHANNEL        |             1                             |
|      INPUT_SOFTAP_ENCRYPTION      |             WPA2_PSK                      |
|      INPUT_SOFTAP_MAX_CONN        |             4                             |
|     INPUT_SOFTAP_SSID_HIDDEN      |             no                            |
|       INPUT_SOFTAP_SRC_IP         |             192.168.2.1                   |
|      INPUT_SOFTAP_ARP_DEST_IP     |             192.168.2.22                  |
+-----------------------------------+-------------------------------------------+
|            STATION                |                                           |
+-----------------------------------+-------------------------------------------+
|        INPUT_STATION__SSID        |             MyWifi                        |
|       INPUT_STATION_BSSID         |             0                             |
|   INPUT_STATION_IS_WPA3_SUPPORTED |             no                            |
|      INPUT_STATION_PASSWORD       |             MyWifiPass@123                |
|       INPUT_STATION_SRC_IP        |             192.168.1.233                 |
|     INPUT_STATION_ARP_DEST_IP     |             192.168.1.203                 |
+-----------------------------------+-------------------------------------------+
Scanned Neighbouring AP list
+----------------------------------+----------------------+---------+---------+---------------+
|                 SSID             |         BSSID        |   rssi  | channel | Auth mode     |
+----------------------------------+----------------------+---------+---------+---------------+
| MyWifi                           | c4:e9:84:24:8b:14    | -10     | 10      | 3             |
| Jupiter                          | 00:1e:a6:27:f2:70    | -86     | 9       | 3             |
| Saturn                           | d8:47:32:50:e1:62    | -90     | 4       | 3             |
| Pluto                            | d8:47:32:50:79:8e    | -91     | 7       | 4             |
| Mars                             | 98:de:d0:83:8f:36    | -93     | 9       | 4             |
| Venus                            | 60:e3:27:92:82:b0    | -94     | 6       | 3             |
| Mercury                          | 84:1b:5e:3c:c2:46    | -95     | 6       | 4             |
+----------------------------------+----------------------+---------+---------+---------------+
Station mode: ssid: MyWifi passwd MyWifiPass@123
Station's MAC address is 3c:71:bf:9a:c2:44
Connected to MyWifi
SoftAP mode: ssid: ESPWifi passwd ESPWifi@123
SoftAP's MAC address is 3c:71:bf:9a:c2:45
started ESPWifi softAP
```

### User configuration parameter

Host firmware has basic user configuration parameters. User needs to manually configure these values. Click on `stm_spi_host` under `Project Explorer` tab. Then `menu -> Project -> Properties -> C/C++ Build -> Build Variables -> < select variable> -> Edit -> OK -> Apply`.

Build Variables are as follows:

| Sr. No. | Parameters | Type | Decription |
|:-------:|:----------|:----:|:----------|
|1| CODE_BASE | Directory Path | Absolute Path to `ESP-Hosted` clonned directory. |
|2| INPUT__OPERATING_MODE | String | Operating mode as `STATION`, `SOFTAP` or `STATION+SOFTAP`.|
|3| INPUT_GET_AP_SCAN_LIST | String | Get list of available APs. ("yes" or "no") |
|4| INPUT_SOFTAP__SSID | String | SSID of softAP. For example. "ESPWifi" |
|5| INPUT_SOFTAP_PASSWORD | String | Password of softAP . Length of password should be 8~64 bytes ASCII. For example. "ESPWifi@123" |
|6| INPUT_SOFTAP_BANDWIDTH | String | Set bandwidth of ESP32 softAP ( HT20 or HT40 ) |
|7| INPUT_SOFTAP_CHANNEL | String | Channel ID (Range: 1 to 11)|
|8| INPUT_SOFTAP_ENCRYPTION | String | Encryption mode. ( OPEN, WPA_PSK, WPA2_PSK, WPA_WPA2_PSK) |
|9| INPUT_SOFTAP_MAX_CONN | String | Maximum number of stations can connect to ESP32 SoftAP (Range: 1 to 10) |
|10| INPUT_SOFTAP_SSID_HIDDEN | String | SoftAP should broadcast its SSID or not ( "yes" : SSID is broadcast, "no" : SSID is not broadcast )
|11| INPUT_SOFTAP_SRC_IP | String | Source IP (IPv4)address of host in softAP mode. |
|12| INPUT_SOFTAP_ARP_DEST_IP | String | Destination IP (IPv4)address of station connected to host. |
|13| INPUT_STATION__SSID | String | SSID of station. ex. "MyWifi" |
|14| INPUT_STATION_BSSID | String | MAC address of AP, To differentiate between APs, In case multiple AP has same SSID. User can put "0" in case MAC address unknown. |
|15| INPUT_STATION_IS_WPA3_SUPPORTED | String | Status of wpa3 supplicant present on AP. ( "yes"  : supported, "no"  : not supported) |
|16| INPUT_STATION_PASSWORD | String | Password of AP. Length of password should be 8~64 bytes ASCII. ex. "MyWifi@123" |
|17| INPUT_STATION_SRC_IP | String | Source IP (IPv4)address of host in station mode.|
|18| INPUT_STATION_ARP_DEST_IP | String | Destination IP (IPv4)address of AP to which host gets connected.|

#### Note:
All string parameters are expected to be enclosed in double quotes.

Verify that `CODE_BASE` variable from `menu -> Project -> Properties -> Resource -> Linked Resources -> Path Variables -> CODE_BASE` points to git cloned directory.

In case there is a warning sign on folder icon, it means STM32CubeIDE has not correctly configured the variable. In that case, re-configure it using, `Edit -> Click On Folder -> <Path/to/esp_hosted> -> Open -> OK`. The warning sign now should disappear. Now click on `Apply and Close`.

## ARP Testing :

With minimal network stub, arping is tested with this project.
Once the connection is setup between STM32 and ESP32, ARP can be tested for that interface.

### ARP Request originated from STM32
For station mode, `INPUT_STATION_SRC_IP` is used as STM32 IPv4 address and `INPUT_STATION_ARP_DEST_IP` is considered as destination IPv4 address. This could be configured as IPv4 address of your machine.
For softap mode, `INPUT_SOFTAP_SRC_IP` will be used as STM32 IPv4 address and `INPUT_SOFTAP_ARP_DEST_IP` used as destination IPv4 address.

Please note,
1. In case of softap mode, only static IPv4 addresses are supported currently. You can use your machine to connect to softap and assign static address as `INPUT_SOFTAP_ARP_DEST_IP`.
2. IP addresses configured are in same subnet for that interface.

ARP request will be triggered around every one second for both softap and station network. ARP responses are displayed on receival. On minicom, typically should be printed like,
```
ARP_RSP_RCVD: XX bytes from 192.168.1.203 (58:a0:23:86:2a:c4)
```

### ARP Responses from STM32
In case of station mode, ARP request could be triggered from station connected to `INPUT_STATION__SSID`. To start arping request, please use ARP destination as `INPUT_STATION_SRC_IP`. For example,
```
sudo arping 192.168.1.233
```
Similarly for softap mode, you would need to connect to Wifi from ESP32, `INPUT_SOFTAP__SSID`. Trigger command,
```
sudo arping 192.168.2.1
```
ARP request received in STM32 are displayed on minicom like,
```
ARP_REQ_RCVD: XX bytes from 192.168.1.205 (a0:88:b4:e5:d5:38)
```
ARP response is triggered for requests received. You should be able see ARP response on your machine as,
```
XX bytes from 3c:71:bf:9a:bc:b8 (192.168.1.233): index=0 time=199.144 msec
```
