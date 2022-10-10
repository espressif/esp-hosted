## Bluetooth/BLE Setup and Test Procedure

* Ensure that BlueZ and related dependency softwares (listed in setup document earlier) are installed on the host
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
* Please note that `hciconfig` might be deprecated in latest stable BlueZ versions. You can use alternatives given by BlueZ.

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

- Please refrain for using `hcitool`, as they are deprecated in latest stable releases of `BlueZ`. \
   Alternative tool like `sudo buetoothctl` with command `scan on` could be used for BT scan.

##### BLE scan

- Please refrain for using `hcitool`, as they are deprecated in latest stable releases of `BlueZ`. \
   Alternative tool like `sudo buetoothctl` with command `scan on` could be used for BT scan.
