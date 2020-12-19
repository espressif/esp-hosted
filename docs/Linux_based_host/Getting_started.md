# Getting started with Raspberry-Pi (rpi)

This section elaborates about setting up the connectivity for Wi-Fi and Bluetooth.
Before proceeding, ensure pre-requisites [Hardware Setup and Compilation](docs/Linux_based_host/Setup.md) are done.

## Wi-Fi Connectivity

Wi-Fi can be configured as either as `STATION` mode or `SOFTAP` mode or `SOFTAP-STATION` mode.
* **STATION Mode**
    - This mode is used for connecting to external AP i.e. Wi-Fi router. Consider example of smartphone connecting to Wi-Fi router. Like smartphone, rpi behaves as station and gets authenticated and connected to external AP i.e. Wi-Fi router.
* **SOFTAP Mode**
    - This mode is exactly opposite, wherein rpi with help of ESP-Hosted solution, instructs ESP peripheral to create Wi-Fi network. Stations like smartphones can authenticate and connect to it.
* **SOFTAP+STATION Mode**
    - This is combination of both the modes. In this mode, rpi behaves as station and connects to external AP. At the same time, rpi with help of ESP device, can create the Wi-Fi network.

To setup Wi-Fi connectivity, `control command` APIs are provided. Using these APIs, all above modes can be easily configured. These APIs are available in python and C implementation.

### Using Python Implementation
#### Wi-Fi Control Interface
[`python_support`](../../host/linux/host_control/python_support/) in `host/linux/host_control` directory implements Wi-Fi control interface between the host and ESP peripheral. It contains following functions which can be used to control Wi-Fi functionality of the ESP peripheral:

| Function | Functionality |
|:--------|:-------------|
| wifi_get_mac(mode) | get MAC address of station or softAP Interface |
| wifi_get_mode() | get wifi mode |
| wifi_set_mode(mode) | set wifi mode |
| wifi_set_ap_config(ssid, pwd, bssid, is_wpa3_supported, listen_interval) | connect to AP as a station |
| wifi_get_ap_config() | get AP configuration and status |
| wifi_disconnect_ap() | disconnect from AP |
| wifi_set_softap_config(ssid, pwd, chnl, ecn, max_conn, ssid_hidden, bw) | start softAP |
| wifi_get_softap_config() | get softAP configuration |
| wifi_ap_scan_list() | scan available APs |
| wifi_connected_stations_list() | list stations connected to softAP |
| wifi_set_mac(mode, mac) | sets custom mac address for station and softAP Interface |
| wifi_set_power_save_mode(power_save_mode) | set power save mode |
| wifi_get_power_save_mode() | get power save mode |

A utility script `test.py` is provided [host/linux/host_control/python_support/test.py](../../host/linux/host_control/python_support/test.py). This script can be used as an example of using these functions. You can run the script as follows:
```
python test.py
```

#### Host Driver Setup
To compile and load the host driver on a Raspberry Pi, go to `host/linux/host_control/` folder and run `./rpi_init.sh <sdio/spi>`. This script also creates `/dev/esps0` device, which is used as a WLAN control interface.

#### Convenience Scripts
Following are few ready to use convenience script provided in the repository. These scripts make use of control functions mentioned in above section.

##### Scan external access points
`ap_scan_list.py` is a python script which gives a scanned list of available APs. The list contains SSID, channel number, RSSI, MAC address, and authentication mode of AP.

```
python ap_scan_list.py
```

##### Connect to external access point
`station_connect.py` is a python script which configures ESP peripheral in station mode, and connects to an external AP with user-provided credentials. Also it enables the station interface and runs DHCP client. The script accepts arguments such as SSID, password, optionally MAC address, wpa3 support and listen interval (AP beacon intervals). For example:

```
python station_connect.py 'xyz' 'xyz123456' --bssid='e5:6c:67:3c:cf:65' --is_wpa3_supported=True --listen_interval=3
```

You can check that `ethsta0` interface is up (enabled) using `ifconfig`. WPA3 option is only applicable if target AP supports WPA3.

To know status of station, use wifi_get_ap_config() function. In case station is connected with AP, it returns ssid, bssid(MAC address), channel, rssi, encryption mode of AP. and In case of not connected with AP returns `failure` with `not_connected` print.

##### Disconnect from external access point
`station_disconnect.py` is a python script to disconnect ESP peripheral station from AP.

```
python station_disconnect.py
```

You can check that `ethsta0` interface is down (disabled) using `ifconfig`.

##### Setup and start SoftAP
`softap_config.py` is a python script for configuring ESP peripheral to work in softAP mode. The following parameters should be provided:

- SSID
- password, should be 8 ~ 64 bytes ASCII
- channel ID, 1 ~ 11
- encryption method (0: `OPEN`, 2: `WPA_PSK`, 3: `WPA2_PSK`, 4: `WPA_WPA2_PSK`)
- maximum number of stations, in range of 1 ~ 10.
- whether SSID is hidden (1 if the softAP shouldn't broadcast its SSID, else 0)
- bandwidth (1: `WIFI_BW_HT20` (20MHZ), 2: `WIFI_BW_HT40` (40MHZ))

The maximum number of connections, "SSID hidden", and bandwidth parameters are optional.

For example:
```
python softap_config.py 'xyz' 'xyz123456' 1 3 --max_conn=4 --ssid_hidden=0 --bw=1
```

You can check that `ethap0` interface is up (enabled) using `ifconfig`.
`Note: Currently WEP, WPA2_ENTERPRISE, WPA3 support is not present for softAP mode.`

To start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`).

##### Stop SoftAP
`softap_stop.py` python script disables wifi softAP mode on ESP peripheral. This script will change wifi mode to `null` if only softAP is running, or to `station` mode if softAP and station both are on.

```
python softap_stop.py
```

You can check that `ethap0` interface is down (disabled) using `ifconfig`.

##### List external stations connected to SoftAP
`connected_stations_list.py` is a python script that returns a list of MAC addresses of stations connected to softAP.

```
python connected_stations_list.py
```

### Using C Implementation
As an alternative to python implementation of Wi-Fi control interface, a `C language` based implementation is provided. Following API's are provided as a part of this.

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

Similar to `test.py`, a utility test application, [test.c](../../host/linux/host_control/c_support/test.c) is provided that demonstrates usage of these functions. To use this:
* One should make appropriate changes to configuration parameters in `test.c` like Station SSID, Password etc.
* To compile this, run `make` command in `host/linux/host_control/c_support` directory. This will compile and create `test.out` file.
* To execute, run `test.out`.

### Wi-Fi Performance in shielded environment

#### Over SDIO interface
##### ESP32
###### Station mode

| Traffic | 11n 20MHz | 11n 40 MHz |
|:--------|:----------|:-----------|
| TCP Tx | 30.6 Mbps | 24.3 Mbps |
| TCP Rx | 16.0 Mbps | 18.8 Mbps |
| UDP Tx | 41.0 Mbps | 46.4 Mbps |
| UDP Rx | 27.0 Mbps | 26.1 Mbps |

###### SoftAP mode

| Traffic | 11n 20MHz | 11n 40 MHz |
|:--------|:----------|:-----------|
| TCP Tx | 22.9 Mbps | 19.7 Mbps |
| TCP Rx | 17.8 Mbps | 16.6 Mbps |
| UDP Tx | 39.5 Mbps | 46.3 Mbps |
| UDP Rx | 28.7 Mbps | 26.8 Mbps |

#### Over SPI interface
##### ESP32
###### Station mode

| Traffic | 11n 20MHz | 11n 40 MHz |
|:--------|:----------|:-----------|
| TCP Tx | 5.64 Mbps | 5.20 Mbps |
| TCP Rx | 5.33 Mbps | 5.29 Mbps |
| UDP Tx | 5.35 Mbps | 5.37 Mbps |
| UDP Rx | 4.70 Mbps | 5.29 Mbps |

###### SoftAP mode

| Traffic | 11n 20MHz | 11n 40 MHz |
|:--------|:----------|:-----------|
| TCP Tx | 5.12 Mbps | 5.21 Mbps |
| TCP Rx | 5.26 Mbps | 5.26 Mbps |
| UDP Tx | 5.29 Mbps | 5.30 Mbps |
| UDP Rx | 5.39 Mbps | 5.40 Mbps |

##### ESP32S2
###### Station mode

| Traffic | 11n 20MHz | 11n 40 MHz |
|:--------|:----------|:-----------|
| TCP Tx | 11.9 Mbps | 11.0 Mbps |
| TCP Rx | 17.2 Mbps | 16.8 Mbps |
| UDP Tx | 16.8 Mbps | 17.0 Mbps |
| UDP Rx | 18.5 Mbps | 17.7 Mbps |

###### SoftAP mode

| Traffic | 11n 20MHz | 11n 40 MHz |
|:--------|:----------|:-----------|
| TCP Tx | 11.2 Mbps | 11.6 Mbps |
| TCP Rx | 17.1 Mbps | 17.2 Mbps |
| UDP Tx | 17.7 Mbps | 17.4 Mbps |
| UDP Rx | 20.3 Mbps | 20.2 Mbps |


## Bluetooth/BLE Connectivity

- Ensure that bluez is installed on Raspberry Pi and it is downloaded in source format as well. Please refer to [Setup](Setup.md) instructions for more details.
- In following test, Android device was used as a BT/BLE test device. For BLE testing, [nRF connect for mobile APP](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en_IN) was used.
- Go to `host/linux/host_control/` folder to run following script.

### UART based setup
- Execute `./rpi_init.sh btuart` to prepare Raspberry Pi for Bluetooth operation
- Execute `hciattach` command as below to add HCI interface (i.e. hciX)
```
$ sudo hciattach -s 115200 /dev/serial0 any 115200 flow
```

### SDIO based setup
- Execute `./rpi_init.sh` or `./rpi_init.sh sdio` to prepare Raspberry-Pi for SDIO+BT operation.
- HCI interface (i.e hciX) will be available for use as soon as host driver detects ESP peripheral over SDIO interface.
- One can use standard HCI utilities over this interface to make use of BT/BLE feature.

### SPI based setup
- Execute `./rpi_init.sh spi` to prepare Raspberry-Pi for SPI+BT operation.
- HCI interface (i.e hciX) will be available for use as soon as host driver detects ESP peripheral module over SPI interface.
- One can use standard HCI utilities over this interface to make use of BT/BLE feature.

### BT/BLE Test procedure
#### GATT server

1. run `hciconfig`. Output should show only one `SDIO` interface.
```
hci0:	Type: Primary  Bus: SDIO
	BD Address: 3C:71:BF:9A:C2:46  ACL MTU: 1021:9  SCO MTU: 255:4
	UP RUNNING PSCAN
	RX bytes:8801 acl:1000 sco:0 events:406 errors:0
	TX bytes:5097 acl:147 sco:0 commands:52 errors:0
```
2. Go to `bluez-5.xx` folder. Run `./test/example-gatt-server`. This will start GATT server on Raspberry-Pi.

3. Now start advertising. Run `sudo hciconfig hci0 leadv`.

4. Now ESP peripheral's mac address should be listed in scan list of mobile app.

5. Connect to ESP peripheral's mac address with mobile as GATT client.

6. Check read/write characteristics fields in `Heart Rate` service.

#### GATT Client

1. Run `./test/example-gatt-client` on Raspberry Pi. This will start GATT client on Raspberry Pi.

2. You should see a `Heart Rate Measurement` field in Raspberry Pi console.

#### BT scan

Run `hcitool scan` for BT device scanning.

#### BLE scan

Run `hcitool lescan` for BLE device scanning.
