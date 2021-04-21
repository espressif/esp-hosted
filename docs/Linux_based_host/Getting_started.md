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
	* This script also runs DHCP client that obtains IP address from an external AP.
	* The script accepts arguments such as SSID, password, optionally BSSID of an external AP, wpa3 support and listen interval (AP beacon intervals). For example:
		```
		$ python station_connect.py 'xyz' 'xyz123456' --bssid='e5:6c:67:3c:cf:65' --is_wpa3_supported=True --listen_interval=3
		```
		:warning:`Note: WPA3 option is only applicable if target AP supports WPA3.`
	* As an end result of this script:
		* Wi-Fi station interface of ESP peripheral will be connected to an external AP
		* `ethsta0` interface will be up and it will also have an IP address
		* Network data path will be open for higher applications to use this interface for data communication
---
* **Disconnect from external access point**
	* `station_disconnect.py` script disconnects ESP peripheral station from an external AP.
		```
		$ python station_disconnect.py
		```
	* As an end result of this script:
		* `ethsta0` interface will be in down state and it won't have IP address
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
	* To start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`).
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


## 3. Troubleshoot Instructions

Please refer following for troubleshoot instructions if something goes wrong.

* [Troubleshooting Guide](./Troubleshoot.md)
