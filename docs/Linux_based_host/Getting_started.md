# User Guide for ESP-Hosted with Linux Host (Raspberry-Pi)

This section elaborates about setting up  Wi-Fi and Bluetooth/BLE connectivity on Linux host using ESP-Hosted solution.

## 1. Wi-Fi Connectivity

Wi-Fi can be configured as either as `STATION` mode or `SOFTAP` mode or `STATION+SOFTAP` mode.
* **STATION Mode**
    - This mode is used for connecting to external AP i.e. Wi-Fi router. Consider example of smartphone connecting to Wi-Fi router. Like smartphone, rpi behaves as station and gets authenticated and connected to external AP i.e. Wi-Fi router.
* **SOFTAP Mode**
    - This mode is exactly opposite, wherein rpi with help of ESP-Hosted solution, instructs ESP peripheral to create Wi-Fi network. Stations like smartphones can authenticate and connect to it.
* **STATION+SOFTAP Mode**
    - This is combination of both the modes. In this mode, rpi behaves as station and connects to external AP. At the same time, rpi with help of ESP device, can create the Wi-Fi network.

To setup Wi-Fi connectivity, `control command` APIs are provided. Using these APIs, all above modes can be easily configured. These APIs are available in python and C implementation.

### 1.1 Control/Configure Wi-Fi Interface
This section explains how one can configure and control Wi-Fi interface using provided python scripts. These scripts makes use of control interface API's implemented in python.  

These convenience scripts are placed in `host/linux/host_control/python_support/` directory. Use below command to navigate to this directory.
```sh
$ cd host/linux/host_control/python_support/
```

#### 1.1.1 Wi-Fi Station Mode Operations

* **Scan external access points**
	* `ap_scan_list.py` script initiates Wi-Fi scan and displays list of available APs in the vicinity. The output contains SSID, channel number, RSSI, MAC address and authentication mode of AP.
		```
		$ python ap_scan_list.py
		```
---
* **Connect to external access point**
	* `station_connect.py` script configures ESP peripheral in WiFi station mode and connects to an external AP with user-provided credentials.
	* The script accepts arguments such as SSID, password, optionally BSSID of an external AP, wpa3 support and listen interval (AP beacon intervals). For example:
		```
		$ python station_connect.py 'xyz' 'xyz123456' --bssid='e5:6c:67:3c:cf:65' --is_wpa3_supported=True --listen_interval=3
		```
		:warning:`Note: WPA3 option is only applicable if target AP supports WPA3.`
	* As an end result of this script:
		* Wi-Fi station interface of ESP peripheral will be connected to an external AP
		* `ethsta0` interface will be up and ESP32's MAC address will be assigned to it.

Note:
* User needs to run DHCP client to obtain IP address from an external AP. After that network data path will be open for higher applications to use this interface for data communication. For an example as below.

```
sudo dhclient ethsta0 -r

sudo dhclient ethsta0 -v
```
---
* **Disconnect from external access point**
	* `station_disconnect.py` script disconnects ESP peripheral station from an external AP.
		```
		$ python station_disconnect.py
		```
	* As an end result of this script:
		* `ethsta0` interface will be in down state
		* Network data path will be in closed state, hence there won't be any data communication on this interface

#### 1.1.2 Wi-Fi softAP Mode Operations

* **Setup and start softAP**
	* `softap_config.py` script configures ESP peripheral to work in softAP mode. The following parameters should be provided:
		- SSID
		- password, should be 8 ~ 64 bytes ASCII
		- channel ID, 1 ~ 11
		- encryption method (0: `OPEN`, 2: `WPA_PSK`, 3: `WPA2_PSK`, 4: `WPA_WPA2_PSK`)
		- maximum number of stations, in range of 1 ~ 10.
		- whether SSID is hidden (True if the softAP shouldn't broadcast its SSID, else False)
		- bandwidth (1: `WIFI_BW_HT20` (20MHz), 2: `WIFI_BW_HT40` (40MHz))
	* The maximum number of connections, "SSID hidden", and bandwidth parameters are optional.
	* For example:
		```
		$ python softap_config.py 'xyz' 'xyz123456' 1 3 --max_conn=4 --ssid_hidden=False --bw=1
		```
	* As an end result of this script:
		* SoftAP interface will be up and running on ESP peripheral
		* `ethap0` interface will be in `up` state
	* To start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`). For an example as below:

```
sudo dnsmasq --no-daemon --no-resolv --no-poll --dhcp-script=/system/bin/dhcp_announce --dhcp-range=192.168.4.1,192.168.4.20,1h

sudo ifconfig ethap0 192.168.4.5
```
---
* **Stop softAP**
	* `softap_stop.py` script disables wifi softAP mode on ESP peripheral.
		```
		$ python softap_stop.py
		```
	* As an end result of this script:
		* SoftAP on ESP peripheral will no more send beacons
		* `ethap0` interface will be in down state
---
* **List external stations connected to softAP**
	* `connected_stations_list.py` script displays list of MAC addresses of stations connected to softAP.
		```
		$ python connected_stations_list.py
		```

## 2. Bluetooth/BLE Connectivity

* Ensure that bluez is installed on Raspberry Pi and it is downloaded in source format as well.
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

### 2.1 BT/BLE Test procedure

* ESP-Hosted related BR/EDR 4.2 and BLE 4.2 functionalities are tested with `bluez` 5.43+.
Whereas BLE 5.0 functionalities are tested with `bluez` 5.45+.
* We suggest latest stable `bluez` version to be used. Any other bluetooth stack instead of `bluez` also could be used.
* To upgrade `bluez` for particular version, follow this [link](https://scribles.net/updating-bluez-on-raspberry-pi-from-5-43-to-5-50/). Replace bluez `older version` to `expected version` while following mentioned link.

#### 2.1.1 GATT server

1. Go to `bluez-5.xx` folder. Run `./test/example-gatt-server`. This will start GATT server on Raspberry-Pi.

2. Now start advertising. Run `sudo hciconfig hci0 leadv`.

3. Now ESP peripheral's mac address should be listed in scan list of mobile app.

4. Connect to ESP peripheral's mac address with mobile as GATT client.

5. Check read/write characteristics fields in `Heart Rate` service.

#### 2.1.2 GATT Client

1. Run `./test/example-gatt-client` on Raspberry Pi. This will start GATT client on Raspberry Pi.

2. You should see a `Heart Rate Measurement` field in Raspberry Pi console.

#### 2.1.3 BT scan

Run `hcitool scan` for BT device scanning.

#### 2.1.4 BLE scan

Run `hcitool lescan` for BLE device scanning.

### 2.2 BLE 5.0 testing

Only ESP32C3 HCI controller supports BLE 5.0. Several new features are introduced in BLE 5.0. The major areas of improvement are:
1. Slot Availability Mask (SAM)
2. 2 Msym/s PHY for LE
3. LE Long Range
4. High Duty Cycle Non-Connectable Advertising
5. LE Advertising Extensions
6. LE Channel Selection Algorithm #2

To test BLE 5.0 on RPi minimum `bluez` version `5.45` and above required. If `bluez` version is less than 5.45 ,then upgrade `bluez` version.

Check current `bluez` version by running following command on RPi:

```
bluetoothctl -v
```
:warning: `hcitool lescan` is deprecated. Please dont use it.

### 2.2.1 Basic scan, pair, connect

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

#### 2.2.2 GATT Server

BLE 5.0 has backword compability. It can connect with BLE4.2 devices.
Below example demonstrate linux host as GATT server and mobile phone as GATT client. We are using `nRF connect` application for GATT client operartion.

Execute following steps on linux host.
Steps:
1. Run `sudo bluetoothctl`.
2. Run `list` to get MAC address of ESP32.
3. To set device name, run `menu advertise`. Then `name <enter_any_name>`.
4. To come back to main menu, run `back`.
5. To start advertising, run `advertise on`.
Perform below steps on Mobile Phone:
6. Turn on mobile phone's bluetooth. Open nRF connect application, ESP32's MAC address will be displayed under `SCANNER` tab as a result of scan.
7. Click on connect. Client tab will be open. Click on `Generic Attribute` option.
8. Perform read/write on listed characteristics fields in `Generic Attribute` service.
To disconnet:
9. Run `disconnect <MAC_ADDRESS_of_gatt_client>` on linux host's `bluetoothctrl` OR click on `DISCONNECT` in nRF connect application's `GATT client` screen.

#### 2.2.3 GATT client

BLE 5.0 has backword compability. It can connect with BLE4.2 devices.
Below example demonstrate linux host as GATT client and mobile phone as GATT server. We are using `nRF connect` application for GATT server operartion.

Execute following steps on linux host.
Steps:
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

#### 2.2.4 1M, 2M, CODED phy for LE

BLE5.0 supports 1M, 2M and CODED phy. To use 2M and CODED phy for gatt read/write procedure as follow:

Note:
* Default selected phy is 1M. To perform gatt read/write with BLE5.0 peripheral, both host and peripheral must have same phy configuration.

* 'PHY' feature in BLE 5.0 is verified with btmgmt tool from bluez version 5.56+.

* If `bluez` version is less than 5.56 ,then upgrade `bluez` version.

#####  Using 1M phy:
1M phy is default phy for BLE5.0. Follow above mentioned steps in section 2.2.1
for connection. After connection follow gatt read/write from gatt menu in bluetoothctl.

##### Using 2M phy:
2M phy can not use for connection in BLE5.0 . So configure phy as 1M and 2M both, make connection with other BLE5.0 device and then set phy as 2M. On peripheral side make primary phy as 1M and secondary phy as 2M.

Steps:
1. To configure phy as 1M and 2M both, run `sudo hcitool cmd 08 31 03 03 03`.
2. To check selected phy, Go to `bluez-5.56` directory. Run `sudo ./tools/btmgmt --index hci0` and run `phy`.
3. Connect to BLE5.0 device using above mentioned steps in section 2.2.1.
4. while executing connect command, there is `LE Enhanced Connection Complete` event in `btmon` log. Note down `handle` value.
5. After connection, exit form bluetoothctl. Run `exit` in bluetoothctl.
6. Now configure phy into 2M. Run `sudo hcitool cmd 08 32 <handle_value> 03 02 02 00`.
7. Follow gatt read/write from `gatt menu` in bluetoothctl.

##### Using CODED phy:
Configure CODED phy on host and peripheral side.

Steps:
1. To configure phy as CODED phy, run `sudo hcitool cmd 08 31 03 04 04`.
2. To check selected phy, Go to `bluez-5.56` directory. Run `sudo ./tools/btmgmt --index hci0` and run `phy`.
3. Connect to BLE5.0 device using above mentioned steps in section 2.2.1.
4. Follow gatt read/write from gatt menu in bluetoothctl.

## 3. OTA operation

OTA (Over The Air) update performs following operations.
* Erase ota flash partition of ESP32
* Download chunk from URL and write that chunk into flash, one by one, till whole binary is written
* Validate the complete written binary in flash
* Sets newly written OTA partition as boot partition
* Reboot the ESP32 after 5 second

Please follow [OTA update documentation](ota_update.md) for further details.

## 4. Troubleshoot Instructions

Please refer following for troubleshoot instructions if something goes wrong.

* [Troubleshooting Guide](./Troubleshoot.md)
