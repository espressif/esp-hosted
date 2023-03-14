# Getting started with MCU based Host

This section elaborates about setting up the control path, Wi-Fi connectivity and Bluetooth/BLE connectivity. Before proceeding, ensure pre-requisites [Hardware Setup and Compilation](MCU_based_readme.md#2-setup) are done.

## 1. Control Path

- Control path is intended to setup all configurations at ESP side. These configurations could be related to services like
  - Connect host with external AP (Wi-Fi router)
  - Get configurations of external AP which host is connected
  - Set maximum Wi-Fi transmit power of ESP
  - Find out current Wi-Fi power of ESP
  - Configuring ESP chipset heartbeat
- Control path command could be considered as first step before you can establish data path
- It is way to verify if ESP-Hosted transport like SPI,SDIO is setup correctly
- Overall design is explained in [control path design](../common/contrl_path.md#3-design)
- Underlying [Hosted control path library](../common/contrl_path.md#3-design) is agnostic of platform and common for MPU or MCU based solution.
  - This empowers user to implement/mimic all control path APIs just similar to [Linux demo application in C](../common/c_demo.md)
  - Few sample [control path APIs](../common/ctrl_apis.md) like connecting to station, starting softap are demonstrated as part of [esp_hosted_fg/host/stm32/app/control/control.c](../../host/stm32/app/control/control.c)
  - Rest APIs could be implemeted just similar to ones implemented in Linux demo application 

## 2. Wi-Fi Connectivity

Wi-Fi can be configured as either as `STATION` mode or `SOFTAP` mode or `STATION+SOFTAP` mode.
* **STATION Mode**
    - This mode is used for connecting to external AP i.e. Wi-Fi router. Consider example of smartphone connecting to Wi-Fi router. Like smartphone, host behaves as station and gets authenticated and connected to external AP i.e. Wi-Fi router.
* **SOFTAP Mode**
    - This mode is exactly opposite, wherein host with help of ESP-Hosted solution, instructs ESP peripheral to create Wi-Fi network. Stations like smartphones can authenticate and connect to it.
* **STATION+SOFTAP Mode**
    - This is combination of both the modes. In this mode, host behaves as station and connects to external AP. At the same time, host with help of ESP device, can create the Wi-Fi network.

Host firmware provides Wi-Fi connectivity using control path and data path. Control path commands uses `protocomm` layer of ESP-IDF to serialize structured control data and communicates using SPI transport interface between Host(MCU based Host) and ESP peripheral (ESP32/ESP32-C2/ESP32-C3/ESP32-C6/ESP32-S2/ESP32-S3). User can use control commands to build application.


### 2.1 Start Project with STM32

#### 2.1.1 Supported Hardware
We have tested project for `SPI` transport with STM32F469I-Discovery board and `SDIO` transport with STM32F412ZGT6-Nucleo 144 board. If other than STM32F469I-Discovery board and STM32F412ZGT6-Nucleo 144 board are used, peripheral like SDIO, SPI, USART need to change as per board needs.

| ESP32 Board | STM32 Board  | Transport  |
|:---------:|:------:|:----------:|
| ESP32 | STM32F412ZGT6-Nucleo 144  | SDIO |
| ESP32/S2/S3/C2/C3/C6 | STM32F469I-Discovery  | SPI |

:warning: <code>**Note1:** For SDIO, please check [pull up requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) while choosing ESP module</code>

#### 2.1.2 STM32CubeIDE would be needed to follow next steps.

* Create a workspace_directory outside of `ESP-Hosted` git cloned directory.
* Browse and Open Workspace directory in STM32CubeIDE. It will take few seconds to open STM32CubeIDE.
* From `Information Center` tab select `Start new project` from existing STM32CubeMX configuration file, i.e. ioc file option. It will take few seconds to open dialog box. In STM32CubeMX .ioc file field, choose appropriate .ioc file from `</path/to/esp_hosted>/esp_hosted_fg/host/stm32/proj/<transport>` directory.
* For SPI transport:
```
For ESP32 peripheral: Select stm_spi_host_v1.ioc file
For ESP32-C2/ESP32-C3/ESP32-C6/ESP32-S2/ESP32-S3 peripheral: Select stm_spi_host_v2.ioc file
```
* For SDIO transport:
```
For ESP32 peripheral: Select stm_sdio_host.ioc file
```
* Once file is selected, click `Open` and `Finish`. New dialog box will open as Open Associated Perspective, click on `Yes`. It may take 2-3 minutes to open.
* Close ioc tab then close STM32CubeIDE and click on `exit`.
* For Linux and Mac development hosts, In terminal, run
```
$ cd </path/to/esp_hosted>/esp_hosted_fg/host/stm32/proj
$ bash ./prepare_project.sh <transport> </path/to/workspace_directory>
```
For Windows based systems, open "cmd.exe" or Windows Power Shell and run -
```
> cd <path\to\esp_hosted>\esp_hosted_fg\host\stm32\proj
> prepare_project.bat <transport> <path\to\workspace_directory>
```
This will copy the project configuration files into workspace_directory

* Re-open STM32CubeIDE with workspace as workspace_directory.
* Ignore all warnings under `Problems` tab if any.
* Configure all build variables as mentioned in [User configuration parameter](#22-user-configuration-parameter) section below. Variable `CODE_BASE` should already be populated. All parameters are mandatory to be filled. Please note that, every subsequent change in configuration parameter would need a clean build as mentioned ahead.
* Uncheck `Build Automatically` from `menu -> Project` and Clean build the project as `Project menu -> Clean -> clean`.
* Connect STM32 board to your machine if not already connected.
* Before flashing the project, Open Tera Term or minicom to see STM32 debug logs once project flashed.
```
$ minicom -D /dev/ttyACM0
```
Note: /dev/ttyACM0 is used for For Linux, /dev/cu.usbmodemXXXXXX for Mac and COM port for Windows development host. Baud rate used is 115200. Parity bits configuration is 8N1.

* In STM32CubeIDE, go to `Project Explorer`, right click on `stm_<transport>_host` project. Then `menu -> Run -> Run as -> STM32 Cortex-M C/C++ Application `. This will open Edit Configuration box, Click `OK`.
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

* Expected output log on tera term or minicom will be similar to below:

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
|       INPUT_STATION_BSSID         |                                           |
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

### 2.2 User configuration parameter

Host firmware has basic user configuration parameters. User needs to manually configure these values. Click on `stm_<transport>_host` under `Project Explorer` tab. Then `menu -> Project -> Properties -> C/C++ Build -> Build Variables -> < select variable> -> Edit -> OK -> Apply`.

Build Variables are as follows:

| Sr. No. | Parameters | Type | Decription |
|:-------:|:----------|:----:|:----------|
|1| CODE_BASE | Directory Path | Absolute Path to `ESP-Hosted` clonned directory. |
|2| INPUT__OPERATING_MODE | String | Operating mode as `STATION`, `SOFTAP` or `STATION+SOFTAP`.|
|3| INPUT_GET_AP_SCAN_LIST | String | Get list of available APs. ("yes" or "no") |
|4| INPUT_SOFTAP__SSID | String | SSID of softAP. For example. "ESPWifi" |
|5| INPUT_SOFTAP_PASSWORD | String | Password of softAP . Length of password should be 8~64 bytes ASCII. For example. "ESPWifi@123" |
|6| INPUT_SOFTAP_BANDWIDTH | String | Set bandwidth of ESP softAP ( HT20 or HT40 ) |
|7| INPUT_SOFTAP_CHANNEL | String | Channel ID (Range: 1 to 11)|
|8| INPUT_SOFTAP_ENCRYPTION | String | Encryption mode. ( OPEN, WPA_PSK, WPA2_PSK, WPA_WPA2_PSK) |
|9| INPUT_SOFTAP_MAX_CONN | String | Maximum number of stations can connect to ESP softAP (Range: 1 to 10) |
|10| INPUT_SOFTAP_SSID_HIDDEN | String | SoftAP should broadcast its SSID or not ( "yes" : SSID is broadcast, "no" : SSID is not broadcast )
|11| INPUT_SOFTAP_SRC_IP | String | Source IP (IPv4)address of host in softAP mode. |
|12| INPUT_SOFTAP_ARP_DEST_IP | String | Destination IP (IPv4)address of station connected to host. |
|13| INPUT_STATION__SSID | String | SSID of station. ex. "MyWifi" |
|14| INPUT_STATION_BSSID | String | MAC address of AP, To differentiate between APs, In case multiple AP has same SSID. User can put "" i.e empty string in case MAC address unknown. |
|15| INPUT_STATION_IS_WPA3_SUPPORTED | String | Status of wpa3 supplicant present on AP. ( "yes"  : supported, "no"  : not supported) |
|16| INPUT_STATION_PASSWORD | String | Password of AP. Length of password should be 8~64 bytes ASCII. ex. "MyWifi@123" |
|17| INPUT_STATION_SRC_IP | String | Source IP (IPv4)address of host in station mode.|
|18| INPUT_STATION_ARP_DEST_IP | String | Destination IP (IPv4)address of AP to which host gets connected.|

#### Note:
All string parameters are expected to be enclosed in double quotes.

Verify that `CODE_BASE` variable from `menu -> Project -> Properties -> Resource -> Linked Resources -> Path Variables -> CODE_BASE` points to git cloned directory.

In case there is a warning sign on folder icon, it means STM32CubeIDE has not correctly configured the variable. In that case, re-configure it using, `Edit -> Click On Folder -> <Path/to/esp_hosted> -> Open -> OK`. The warning sign now should disappear. Now click on `Apply and Close`.

## 2.3 ARP Testing :

With minimal network stub, arping is tested with this project.
Once the connection is setup between STM32 and ESP peripheral, ARP can be tested for that interface.

### 2.3.1 ARP Request originated from STM32
For station mode, `INPUT_STATION_SRC_IP` is used as STM32 IPv4 address and `INPUT_STATION_ARP_DEST_IP` is considered as destination IPv4 address. This could be configured as IPv4 address of your machine.
For softAP mode, `INPUT_SOFTAP_SRC_IP` will be used as STM32 IPv4 address and `INPUT_SOFTAP_ARP_DEST_IP` used as destination IPv4 address.

**Please note**,
1. In case of softAP mode, only static IPv4 addresses are supported currently. You can use your machine to connect to softAP and assign static address as `INPUT_SOFTAP_ARP_DEST_IP`.
2. IP addresses configured are in same subnet for that interface.

ARP request will be triggered around every one second for both softAP and station network. ARP responses are displayed on receival. On minicom, typically should be printed like,
```
ARP_RSP_RCVD: XX bytes from 192.168.1.203 (58:a0:23:86:2a:c4)
```

### 2.3.2 ARP Responses from STM32
In case of station mode, ARP request could be triggered from station connected to `INPUT_STATION__SSID`. To start arping request, please use ARP destination as `INPUT_STATION_SRC_IP`. For example,
```
sudo arping 192.168.1.233
```
Similarly for softAP mode, you would need to connect to Wi-Fi from ESP, `INPUT_SOFTAP__SSID`. Trigger command,
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

## 2.4 RAW throughput Testing :

For raw throughput please check [Raw_TP_Testing.md](./Raw_TP_Testing.md)

## 3. BT/BLE connectivity
Bluetooth and BLE handling could be easily ported. Porting details could be found at [ 1.5.2 MCU Host](../../README.md#152-mcu-host) on [README.md](../../README.md)

## 4. OTA operation

OTA (Over The Air) update performs following operations.
* Erase ota flash partition of ESP
* Download chunk from URL and write that chunk into flash, one by one, till whole binary is written
* Validate the complete written binary in flash
* Sets newly written OTA partition as boot partition
* Reboot the ESP after 5 second

OTA also could be ported similar to demo application provided for Linux. This is available in C and python.
Please follow [Linux based OTA update documentation](../Linux_based_host/ota_update.md) for further details.

## 5. Limitations of control path APIs in MCU
- Asynchronous APIs are not tested in MCU, as it is hard implement in constrained MCU's environment
- OTA update is not directly supported just similar to Linux based demo application because,
  - ESP binary is stored in Linux using directory structure path, such is not available in MCU
  - Space constraints - MCU are generally low space devices
  - However, chunked HTTP based OTA update is showcased in [Python based Linux app](../Linux_based_host/ota_update.md#python-implementation) \
Users can use similar HTTP client library and chunked file transfer to do OTA update.
