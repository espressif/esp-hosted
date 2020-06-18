# How to Run scripts on Raspberry-Pi(rpi)
## For Wi-Fi Connectivity
There is `host/linux/host_control` folder in which `host_commands` python library is present. It contains following functions.
```
get_mac(mode)
get_wifi_mode()
set_wifi_mode(mode)
wifi_set_ap_config(ssid, pwd, bssid)
wifi_get_ap_config()
wifi_disconnect_ap()
wifi_set_softap_config(ssid, pwd, chnl, ecn, max_conn, ssid_hidden, bw)
wifi_get_softap_config()
wifi_ap_scan_list()
wifi_connected_stations_list()
```

User can make use of these python functions to get access of wifi functionalities of ESP32. Also in `host/linux/host_control/host_commands` folder `test.py` python script present to try those python functions. User can use it by running following command.
```
python test.py
```

Go to `host/linux/host_control/` folder and run `./rpi_init.sh` to compile and insert host driver on Raspberry-Pi. This script also creates `/dev/esps0` which is used as a WLAN control interface.

There are six python scripts for station connect to AP, station disconnect from AP ,start softAP, stop softAP, scan available APs and list stations connected to softAP.

1. `station_connect.py` is a python script which configures ESP32 in `station mode`, connects Raspberry-Pi to external AP with credentials the user has provided. Also it ups the station interface and runs DHCP client. User should provide parameters like ssid, password, mac address of AP(Its optional parameter).

```
ex. python station_connect.py 'xyz' 'xyz123456' --bssid='e5:6c:67:3c:cf:65'
```
User can see ethsta0 interface is up, using _ifconfig_.

2. `station_disconnect.py` is a python script to disconnect ESP32 station from AP. Now ethsta0 interface is down.

```
python station_disconnect.py
```
3. `softap_config.py` is a python script for configuring ESP32 `softAP mode`. User should provide parameters like ssid, password(password length should be 8~64 bytes ASCII), channel ID (It can be any numberbetween 1 to 11), encryption method (0 : OPEN, 2: WPA_PSK, 3:WPA2_PSK, 4: WPA_WPA2_PSK), max connection count( number of Stations to which ESP32 SoftAP can be connected, within the range of [1, 10]), ssid hidden (it can set to 1 if softAP shouldnt broadcast its ssid else 0) and Bandwidth (1: WIFI_BW_HT20(20MHZ)) , (2: WIFI_BW_HT40(40MHZ)). max connection count, ssid hidden and bandwidth parameters are optional.

```
ex. python softap_config.py 'xyz' 'xyz123456' 1 3 --max_conn=4 --ssid_hidden=0 --bw=1
```
---
Note: User can see ethap0 interface is up, using _ifconfig_. To start data connection, user needs to setup a DHCP server on Raspberry-Pi or set static IP address for AP interface i.e. ethap0

---
4. `softap_stop.py` is a python script to stop ESP32 softap. This script will change wifi mode to `null` if only softAP is running or to `station` mode if softAP and station both are on. Now ethap0 interface is down.

```
ex. python softap_stop.py
```
5. `ap_scan_list.py` is a python script which gives a scanned list of available APs. list contains ssid, channel number, rssi, mac address and authentication mode of AP.
```
ex. python ap_scan_list.py
```
6. `connected_stations_list.py` is a python script that returns list of mac addresses of stations connected to softAP.

```
ex. python connected_stations_list.py
```
### Open air throughput test results for WLAN
Following are the test results conducted in open air.
```
UDP Tx: 16.4 Mbps
UDP Rx: 16.8 Mbps
TCP Tx: 14 Mbps
TCP Rx: 12 Mbps
```

## For Bluetooth/BLE functionality
- Ensure that bluez is installed on Raspberry-Pi and it is downloaded in source format as well. Please refer [Setup](docs/Setup.md) instructions for more details.
- In following test, Android device was used as a BT/BLE test device. For BLE testing, [nRF connect for mobile APP](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en_IN) was used.

- Go to `host/linux/host_control/` folder to run following script.

### UART based setup
1. Execute `./rpi_init.sh btuart` to prepare Raspberry-Pi for Bluetooth operation
2. Execute `hciattach` command as below to add hci interface (i.e. hciX)
```
$ sudo hciattach -s 115200 /dev/serial0 any 115200 flow
```

### SDIO based setup
Execute `./rpi_init.sh` to prepare Raspberry-Pi for SDIO+BT operation.
HCI interface (i.e hciX) will be available for use as soon as host driver detects ESP32 module over SDIO interface.
User can use standard hci utilities over this interface to make use of BT/BLE feature.

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
2. Go to `bluez-5.xx` folder. Run `./test/example-gatt-server`. This will start gatt server on Raspberry-Pi.

3. Now start advertising. Run `sudo hciconfig hci0 leadv`.

4. Now ESP32's mac address should be listed in scan list of mobile app.

5. Connect to ESP32's mac address with mobile as gatt client.

6. User can check read/write characteristics fields in `Heart Rate` service.

#### GATT Client

1. User can run `./test/example-gatt-client` on Raspberry-Pi. This will start gatt client on Raspberry-Pi.

2. User will receive `Heart Rate Measurement` field in Raspberry-Pi console.

#### BT scan

User can run `hcitool scan` for BT device scanning.

#### BLE scan

User can run `hcitool lescan` for BLE device scanning.
