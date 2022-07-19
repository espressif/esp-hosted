## Bluetooth/BLE Setup and Test Procedure

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

### BT/BLE Test procedure

* ESP-Hosted related BR/EDR 4.2 and BLE 4.2 functionalities are tested with `bluez` 5.50+.
Whereas BLE 5.0 functionalities are tested with `bluez` 5.45+.
* We suggest latest stable `bluez` version to be used. Any other bluetooth stack instead of `bluez` also could be used.
* To upgrade `bluez` for particular version, follow this [link](https://scribles.net/updating-bluez-on-raspberry-pi-from-5-43-to-5-50/). Replace bluez `older version` to `expected version` while following mentioned link.

##### GATT server
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

##### GATT Client

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

##### BT scan

Run `hcitool scan` for BT device scanning.

##### BLE scan

Run `hcitool lescan` for BLE device scanning.

## BLE 5.0 testing

Only ESP32-C3 HCI controller supports BLE 5.0. Several new features are introduced in BLE 5.0. The major areas of improvement are:
1. Slot Availability Mask (SAM)
2. 2 Msym/s PHY for LE
3. LE Long Range
4. High Duty Cycle Non-Connectable Advertising
5. LE Advertising Extensions
6. LE Channel Selection Algorithm #2

To test BLE 5.0 on Raspberry Pi, minimum `bluez` version `5.45` and above required. If `bluez` version is less than 5.45 ,then upgrade `bluez` version.

Check current `bluez` version by running following command on Raspberry Pi:

```
$ bluetoothctl -v
```
:warning: `hcitool lescan` is deprecated. Please dont use it.

##### Basic scan, pair, connect

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

##### GATT Server

BLE 5.0 has backword compability. It can connect with BLE4.2 devices.
Below example demonstrate linux host as GATT server and mobile phone as GATT client. We are using `nRF connect` application for GATT client operartion.

Follow section [2.1.1](#211-gatt-server) for GATT server connections.

##### GATT client

BLE 5.0 has backword compability. It can connect with BLE4.2 devices.
Below example demonstrate linux host as GATT client and mobile phone as GATT server. We are using `nRF connect` application for GATT server operartion.

Follow section [2.1.2](#212-gatt-client) for GATT client connections.

##### 1M, 2M, CODED phy for LE

BLE5.0 supports 1M, 2M and CODED phy. To use 2M and CODED phy for gatt read/write procedure as follow:

Note:
* Default selected phy is 1M. To perform gatt read/write with BLE5.0 peripheral, both host and peripheral must have same phy configuration.

* 'PHY' feature in BLE 5.0 is verified with btmgmt tool from bluez version 5.56+.

* If `bluez` version is less than 5.56 ,then upgrade `bluez` version.

######  Using 1M phy:
1M phy is default phy for BLE5.0. Follow above mentioned steps in section 2.2.1
for connection. After connection follow gatt read/write from gatt menu in bluetoothctl.

###### Using 2M phy:
2M phy can not use for connection in BLE5.0 . So configure phy as 1M and 2M both, make connection with other BLE5.0 device and then set phy as 2M. On peripheral side make primary phy as 1M and secondary phy as 2M.

Steps:
1. To configure phy as 1M and 2M both, run `sudo hcitool cmd 08 31 03 03 03`.
2. To check selected phy, Go to `bluez-5.56` directory. Run `sudo ./tools/btmgmt --index hci0` and run `phy`.
3. Connect to BLE5.0 device using above mentioned steps in section 2.2.1.
4. while executing connect command, there is `LE Enhanced Connection Complete` event in `btmon` log. Note down `handle` value.
5. After connection, exit form bluetoothctl. Run `exit` in bluetoothctl.
6. Now configure phy into 2M. Run `sudo hcitool cmd 08 32 <handle value in two bytes in little endian format > 03 02 02 00 00`.
ex. For handle 1 -> `sudo hcitool cmd 08 32 01 00 03 02 02 00 00`
7. Follow gatt read/write from `menu gatt` in bluetoothctl.

###### Using CODED phy:
Configure CODED phy on host and peripheral side.

Steps:
1. To configure phy as CODED phy, run `sudo hcitool cmd 08 31 03 04 04`.
2. To check selected phy, Go to `bluez-5.56` directory. Run `sudo ./tools/btmgmt --index hci0` and run `phy`.
3. Connect to BLE5.0 device using above mentioned steps in section 2.2.1.
4. Follow gatt read/write from `menu gatt` in bluetoothctl.

