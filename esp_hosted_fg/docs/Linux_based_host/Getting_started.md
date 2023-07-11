# User Guide for ESP-Hosted with Linux Host (Raspberry-Pi)

This section elaborates about setting up control path, Wi-Fi and Bluetooth/BLE connectivity on Linux host using ESP-Hosted solution.

## 1. Control Path

* Control path is intended to setup all configurations at ESP side. These configurations could be related to services like
  - Connect host with external AP (Wi-Fi router)
  - Get configurations of external AP which host is connected
  - Set maximum Wi-Fi transmit power of ESP
  - Find out current Wi-Fi power of ESP
* Control path command could be considered as first step before you can establish data path
* It is way to verify if ESP-Hosted transport like SPI,SDIO is setup correctly
* Overall [control path design](../common/contrl_path.md#3-design) and easy setup of control path using [demo application](../common/contrl_path.md#5-kick-start-using-control-path) is explained in [Control Path documentation](../common/contrl_path.md)

## 2. Wi-Fi Connectivity

Wi-Fi can be configured as either as `STATION` mode or `SOFTAP` mode or `STATION+SOFTAP` mode.
- **STATION Mode**
  - This mode is used for connecting to external AP i.e. Wi-Fi router. Consider example of smartphone connecting to Wi-Fi router. Like smartphone, Raspberry Pi, *i.e.* host, behaves as station and gets authenticated and connected to external AP i.e. Wi-Fi router.
- **SOFTAP Mode**
  - This mode is exactly opposite, wherein Raspberry Pi with help of ESP-Hosted solution, instructs ESP peripheral to create Wi-Fi network. Stations like smartphones can authenticate and connect to it.
- **STATION+SOFTAP Mode**
  - This is combination of both the modes. In this mode, Raspberry Pi behaves as station and connects to external AP. At the same time, Raspberry Pi with help of ESP device, can create the Wi-Fi network.
- To setup Wi-Fi connectivity, Wi-Fi related [control path APIs](../common/ctrl_apis.md) should be a reference point.
- Using these APIs, all above modes can be easily configured.
- [C based demo app](../common/c_demo.md) and [Python based demo app](../common/python_demo.md) implement these APIs. User can use this application or event develop own application referring those.

### 2.1 Control/Configure Wi-Fi Interface
- This section explains how one can configure and control Wi-Fi interface using provided [python demo app](../common/python_demo.md)
- Python demo app can be used in two modes, shell mode *i.e.* command line mode and CLI mode. In following sections, only shell mode is detailed, just to enough to kick-start
- User can make use of easy and intuitive mode by referring [here](../common/python_demo.md#modes-supported)
- C based solution also could be used from [C based demo app](../common/c_demo.md). For simplicity, 


Python App is placed in [esp_hosted_fg/host/linux/host_control/python_support/](../../host/linux/host_control/python_support/) directory. Use below command to navigate to this directory.
```sh
$ cd esp_hosted_fg/host/linux/host_control/python_support/
```

#### 2.1.1 Wi-Fi Station Mode Operations

- **Scan external access points**
	- `get_connected_ap_info` command initiates Wi-Fi scan and displays list of available APs in the vicinity. The output contains SSID, channel number, RSSI, MAC address and authentication mode of AP.
		```
		$ sudo python3 test.py get_connected_ap_info
		```
---
- **Connect to external access point**
	- `connect_ap` command configures ESP peripheral in Wi-Fi station mode and connects to an external AP with user-provided credentials.
	- The command accepts arguments such as SSID, password, optionally BSSID of an external AP, wpa3 support and listen interval (AP beacon intervals). For example:
		```
		$ sudo python3 test.py connect_ap --ssid SaveEarth PlantMoreTrees123

		or

		$ sudo python3 test.py connect_ap --bssid 'e5:6c:67:3c:cf:65' \
			--use_wpa3 True --listen_interval 3 --set_dhcp True
		```
	- As an end result of this command:
		- Wi-Fi station interface of ESP peripheral will be connected to an external AP
		- `ethsta0` interface will be up and ESP's MAC address will be assigned to it
		- DHCP lease will be set up.

##### Note:
- `use_wpa3` option will be useful if target AP supports WPA3 security protocol
- `set_dhcp` option is only included for user convenience. If DHCP software different than `dhclient`, User can set this option to `False` and use DHCP software shipped with Linux

---
- **Disconnect from external access point**
	- `disconnect_ap` command disconnects ESP peripheral station from an external AP.
		```
		$ sudo python3 test.py disconnect_ap
		```
	- As an end result of this command:
		* `ethsta0` interface will be in down state
		* Network data path will be in closed state, hence further data communication on this interface will be stopped

#### 2.1.2 Wi-Fi softAP Mode Operations

- **Setup and start softAP**
	- `start_softap` command configures ESP peripheral to work in softAP mode. The following parameters should be provided:
		- `ssid` : SSID
		- `pwd` : password, should be 8 ~ 64 bytes ASCII
		- `channel` : Optional field, Wi-Fi channel ID, 1 ~ 11. Default: 1
		- `sec_prot` : Optional field, security protocol is string one of 'open' or 'wpa_psk' or 'wpa2_psk' or 'wpa_wpa2_psk'. Default: 'wpa_wpa2_psk'
		- `max_conn` : Optional field, maximum number of stations, in range of 1 ~ 10. Default: 4
		- `hide_ssid` : Optional field, whether SSID is hidden (True if the softAP shouldn't broadcast its SSID, else False)
		- `bw` : Optional field, bandwidth, 20 (represents 20MHz) or 40 (represents 40MHz)). Default: 20
	- For example:
		```
		$ sudo python3 test.py start_softap --ssid ESP_WiFi --pwd ESP_WiFi@123 \
			--channel 6 --sec_prot wpa_wpa2_psk --max_conn 9 --hide_ssid False --bw 40
		```
	- As an end result of this command:
		* SoftAP interface will be up and running on ESP peripheral
		* `ethap0` interface will be in `up` state
	- To start data connection, set up a static IP address for AP interface (`ethap0`) or set up DHCP server on the Raspberry Pi similar to example as below:

	```
	$ sudo dnsmasq --no-daemon --no-resolv --no-poll \
		--dhcp-script=/system/bin/dhcp_announce \
		--dhcp-range=192.168.4.1,192.168.4.20,1h

	$ sudo ifconfig ethap0 192.168.4.5
	```
---
- **Stop softAP**
	- `stop_softap` command disables Wi-Fi softAP mode on ESP peripheral.
		```
		$ sudo python3 ./test.py stop_softap
		```
	- As an end result of this command:
		* SoftAP on ESP peripheral will no more send beacons
		* `ethap0` interface will be in down state
---
- **List external stations connected to softAP**
	- `softap_connected_clients_info` command displays list of MAC addresses of stations connected to softAP.
		```
		$ sudo python3 ./test.py softap_connected_clients_info
		```
#### 2.1.3 Other Wi-Fi commands
- There are many more commands which are supported for Wi-Fi related operation like Set/Get transmit power, Set/Get power save mode etc.
- Please Check [All supported commands](../common/python_demo.md#supported-commands)


## 3. Bluetooth/BLE Connectivity

* Ensure that BlueZ and related dependency softwares (mentioned [earlier](Linux_based_readme.md#12-host-setup)) are installed on the host
* BlueZ is example Bluetooth stack used, as it is generally available in Linux. Any other Bluetooth stack can also be used.
* In following test, Android device was used as a BT/BLE test device. For BLE testing, [nRF connect for mobile APP](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en_IN) was used.
* Ensure that `hci0` interface is visible. To check that, run `hciconfig`.
```
hci0:	Type: Primary  Bus: SDIO
	BD Address: 3C:71:BF:9A:C2:46  ACL MTU: 1021:9  SCO MTU: 255:4
	UP RUNNING PSCAN
	RX bytes:8801 acl:1000 sco:0 events:406 errors:0
	TX bytes:5097 acl:147 sco:0 commands:52 errors:0
```
* This interface supports all standard HCI commands. Use standard hci tools to control and configure this interface.

### 3.1 BT/BLE Test procedure

* ESP-Hosted related BR/EDR 4.2 and BLE 4.2 functionalities are tested with `bluez` 5.50+.
Whereas BLE 5.X functionalities are tested with `bluez` 5.45+.
* We suggest latest stable `bluez` version to be used. Any other bluetooth stack instead of `bluez` also could be used.
* To upgrade `bluez` for particular version, follow this [link](https://scribles.net/updating-bluez-on-raspberry-pi-from-5-43-to-5-50/). Replace bluez `older version` to `expected version` while following mentioned link.

#### 3.1.1 GATT server
Steps:
1. Run `sudo bluetoothctl`.
2. Run `list` to get MAC address of ESP.
3. To set device name, run `menu advertise`. Then `name <enter_any_name>`.
4. To come back to main menu, run `back`.
5. To start advertising, run `advertise on`.
Perform below steps on Mobile Phone:
6. Turn on mobile phone's bluetooth. Open nRF connect application, ESP's MAC address will be displayed under `SCANNER` tab as a result of scan.
7. Click on connect. Client tab will be open. Click on `Generic Attribute` option.
8. Perform read/write on listed characteristics fields in `Generic Attribute` service.
To disconnet:
9. Run `disconnect <MAC_ADDRESS_of_gatt_client>` on linux host's `bluetoothctrl` OR click on `DISCONNECT` in nRF connect application's `GATT client` screen.

#### 3.1.2 GATT Client

1. Run `sudo bluetoothctl`.
2. To Turn on power, run `power on`.
3. Make device discoverable, run `discoverable on`.
4. Make device pairable, run `pairable on`.
5. Set current agent to default, run `default-agent`.
6. Turn on bluetooth support, run `agent on`.
7. Turn on mobile phone's bluetooth so that linux host can detect it.
8. Start scanning, run `scan on`.
9. Once mobile phone's MAC address is listed in scan list, stop scanning, run `scan off`.
10. Start btmon to collect log in separate terminal window, run `sudo btmon &`.
11. To trust, run `trust <MAC address of mobile phone>`.
12. To pair, run `pair <MAC address of mobile phone>`.
13. To connect, run `connect <MAC address of mobile phone>`.
14. Once connected, please run `discoverable off`.
15. Go to gatt menu, run `menu gatt`.
16. list available attributes, run `list-attributes`.
17. select characteristic of service, run `select-attribute <characteristic_of_service>`.
18. perform read/write operation on selected characteristic.
19. To disconnect, run `disconnect <MAC_ADDRESS_of_gatt_server>`.

#### 3.1.3 BT scan

Run `hcitool scan` for BT device scanning.

#### 3.1.4 BLE scan

Run `hcitool lescan` for BLE device scanning.

### 3.2 BLE 5.X testing

ESP32-C2/C3/S3 HCI controller supports BLE 5.0. ESP32-C6 supports BLE 5.3. Some of the BLE 5.x features are:
1. Slot Availability Mask (SAM)
2. 2 Msym/s PHY for LE
3. LE Long Range
4. High Duty Cycle Non-Connectable Advertising
5. LE Advertising Extensions
6. LE Channel Selection Algorithm #2

To test BLE 5.X on Raspberry Pi, minimum `bluez` version `5.45` and above required. If `bluez` version is less than 5.45 ,then upgrade `bluez` version.

Check current `bluez` version by running following command on Raspberry Pi:

```
$ bluetoothctl -v
```
:warning: `hcitool lescan` is deprecated. Please dont use it.

### 3.2.1 Basic scan, pair, connect

Execute following steps on linux host.
Steps:
1. Run `sudo bluetoothctl`.
2. To Turn on power, run `power on`.
3. Make device discoverable, run `discoverable on`.
4. Make device pairable on, run `pairable on`.
5. Set current agent to default, run `default-agent`.
6. Turn on bluetooth support, run `agent on`.
7. Start btmon to collect log in separate terminal window, run `sudo btmon &`.
8. Turn on mobile phone's bluetooth so that linux host can detect it.
9. Start scanning, run `scan on` in bluetoothctl window.
10. Once mobile phone's MAC address is listed in scan list, stop scanning, run `scan off`.
11. To trust, run `trust <MAC address of your device>`.
12. To pair, run `pair <MAC address of your device>`.
13. To connect, run `connect <MAC address of your device>`.
14. Once connected, please run `discoverable off`.

#### 3.2.2 GATT Server

BLE 5.X has backword compability. It can connect with BLE4.2 devices.
Below example demonstrate linux host as GATT server and mobile phone as GATT client. We are using `nRF connect` application for GATT client operartion.

Follow section [3.1.1](#311-gatt-server) for GATT server connections.

#### 3.2.3 GATT client

BLE 5.X has backword compability. It can connect with BLE4.2 devices.
Below example demonstrate linux host as GATT client and mobile phone as GATT server. We are using `nRF connect` application for GATT server operartion.

Follow section [3.1.2](#312-gatt-client) for GATT client connections.

#### 3.2.4 1M, 2M, CODED phy for LE

BLE5.X supports 1M, 2M and CODED phy. To use 2M and CODED phy for gatt read/write procedure as follow:

Note:
* Default selected phy is 1M. To perform gatt read/write with BLE5.X peripheral, both host and peripheral must have same phy configuration.

* 'PHY' feature in BLE 5.X is verified with btmgmt tool from bluez version 5.56+.

* If `bluez` version is less than 5.56 ,then upgrade `bluez` version.

#####  Using 1M phy:
1M phy is default phy for BLE5.X. Follow above mentioned steps in section 3.2.1
for connection. After connection follow gatt read/write from gatt menu in bluetoothctl.

##### Using 2M phy:
2M phy can not use for connection in BLE5.X . So configure phy as 1M and 2M both, make connection with other BLE5.X device and then set phy as 2M. On peripheral side make primary phy as 1M and secondary phy as 2M.

Steps:
1. To configure phy as 1M and 2M both, run `sudo hcitool cmd 08 31 03 03 03`.
2. To check selected phy, Go to `bluez-5.56` directory. Run `sudo ./tools/btmgmt --index hci0` and run `phy`.
3. Connect to BLE5.X device using above mentioned steps in section 3.2.1.
4. while executing connect command, there is `LE Enhanced Connection Complete` event in `btmon` log. Note down `handle` value.
5. After connection, exit form bluetoothctl. Run `exit` in bluetoothctl.
6. Now configure phy into 2M. Run `sudo hcitool cmd 08 32 <handle value in two bytes in little endian format > 03 02 02 00 00`.
ex. For handle 1 -> `sudo hcitool cmd 08 32 01 00 03 02 02 00 00`
7. Follow gatt read/write from `menu gatt` in bluetoothctl.

##### Using CODED phy:
Configure CODED phy on host and peripheral side.

Steps:
1. To configure phy as CODED phy, run `sudo hcitool cmd 08 31 03 04 04`.
2. To check selected phy, Go to `bluez-5.56` directory. Run `sudo ./tools/btmgmt --index hci0` and run `phy`.
3. Connect to BLE5.X device using above mentioned steps in section 3.2.1.
4. Follow gatt read/write from `menu gatt` in bluetoothctl.

## 4. OTA operation

OTA (Over The Air) update performs following operations.
* Erase ota flash partition of ESP
* Download chunk from URL and write that chunk into flash, one by one, till whole binary is written
* Validate the complete written binary in flash
* Sets newly written OTA partition as boot partition
* Reboot the ESP after 5 second

Please follow [OTA update documentation](ota_update.md) for further details.

## 5. Troubleshoot Instructions

Please refer following for troubleshoot instructions if something goes wrong.

* [Troubleshooting Guide](./Troubleshoot.md)
