# Hosted solution with ESP32
This project adds a capability to use ESP32 as a communication processor for Wi-Fi and bluetooth/BLE connectivity with an external host. The project provides ESP32 side firmware, example Linux driver and protocol description. This can directly be used with Linux based hosts or can easily be ported to other MCUs with available open protocol description.

## Wi-Fi connectivity solution
This project uses a protobuf based command-set for control path and uses a separate connection (currently supported on SDIO) for data path. The ESP32 provides a simple interface to the host to provide ethernet interface that can transmit and receive 802.3 frames. This allows the TCP/IP and higher level protocol stack to run on the host.

## Bluetooth/BLE connectivity solution
An external host is provided with a serial interface over UART. ESP32 firmware provides bluetooth/ble functionality over this interface. Linux based host can use standard hci tools/commands to control this interface.

# Setup
Currently we support ESP32 WROVER-Kit with Raspberry-Pi (3 Model B+, 4 Model B) for evaluation with Raspbian operating system.

## Raspberry-Pi Software Setup
We recommend full version Raspbian install on Raspberry-Pi to ensure easy driver compilation. In addition for driver compilation, kernel headers are required. Please install them as:
```sh
$ sudo apt update
$ sudo apt-get install raspberrypi-kernel-headers
```
Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed. In such case, upgrade/downgrade kernel and reinstall kernel headers.
```sh
$ ls /lib/modules/$(uname -r)/build
```
Python is required to run utility scripts. Please install it as:
```sh
$ sudo apt-get install python
```
To start with control path on Raspberry-Pi, `protobuf` and `utils` python modules are needed. User can install these modules by running following commands.
```
pip install utils
pip install protobuf
```
Raspi-gpio utility is required to configure GPIO pins. Please install it as:
```sh
$ sudo apt-get install raspi-gpio
```

## Wi-Fi connectivity Setup
### Hardware Setup/Connections
In this setup, ESP32 board acts as a SDIO peripheral and provides Wi-FI capabilities to host. Please connect ESP32 board to Raspberry-Pi with jumper cables as mentioned below. It may be good to use small length cables to ensure signal integrity.

| RPi Pin | ESP32 Pin | Function |
|:-------:|:---------:|:--------:|
| 13 | IO13 | DAT3 |
| 15 | IO14 | CLK |
| 16 | IO15 | CMD |
| 18 | IO2 | DAT0 |
| 22 | IO4 | DAT1 |
| 37 | IO12 | DAT2 |
| 39 | GND | GND |

RPI pinout can be found [here!](https://pinout.xyz/pinout/sdio)

Setup image is here.

![alt text](setup_image/rpi_esp_setup.jpeg "setup of RPI as host and ESP32 as slave")

Power ESP32 and Raspberry Pi separately with a power supply that provide sufficient power. ESP32 can be powered through PC using micro-USB cable.

### Software setup
By default, the SDIO pins of Raspberry-pi are not configured and are internally used for built-in Wi-Fi interface. Please enable SDIO pins by appending following line to _/boot/config.txt_ file
```
dtoverlay=sdio,poll_once=off
```
Please reboot Raspberry-Pi after changing this file.

## Bluetooth/BLE connectivity Setup
### Hardware Setup/Connections
In this setup, ESP32 board provides Bluetooth/BLE capabilities to host over UART interface. Please connect ESP32 board to Raspberry-Pi with jumper cables as below. As mentioned above, use small length cables.

| RPi Pin Function | RPi Pin | ESP32 Pin | ESP32 Pin Function |
|:-------:|:--------:|:---------:|:--------:|
| RX | 10 | IO5 | TX |
| TX | 8 | IO18 | RX |
| CTS | 36 | IO19 | RTS |
| RTS | 11 | IO23 | CTS |
| Ground | 39 | GND | Ground |

### Software setup
By default, the UART pins on Raspberry-Pi are in disabled state. In order to enable UART and setup it for bluetooth connection, follow below steps.
1. Enable UART pins and disable in built bluetooth on Raspberry-Pi by appending following lines to _/boot/config.txt_ file
```
enable_uart=1
dtoverlay=disable-bt
```
2. Remove following from _/boot/cmdline.txt_. Leave everything else untouched.
```
console=serial0,115200
```
e.g. If _/boot/cmdline.txt_ is as below:
```
# cat /boot/cmdline.txt
dwc_otg.lpm_enable=0 console=tty1 console=serial0,115200 root=PARTUUID=5c2c80d1-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles spidev.bufsiz=32768
````
Then after removal of above mentioned arguments, it should look as below:
```
# cat /boot/cmdline.txt
dwc_otg.lpm_enable=0 console=tty1 root=PARTUUID=5c2c80d1-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles spidev.bufsiz=32768
```
3. Disable hciuart on Raspberry-Pi
```
# systemctl disable hciuart
```
4. Reboot Raspberry-Pi

## ESP32 Setup
The control path between Raspberry-Pi and ESP32 is based on `protobuf`. For that `protocomm` layer from ESP-IDF is used. Make sure ESP-IDF on branch `release/v4.0`. Run follwing command on esp32 to make `protocomm_priv.h` available for control path.
```
git mv components/protocomm/src/common/protocomm_priv.h components/protocomm/include/common/
```

On ESP32 either use pre-provided hosted mode firmware binary or if you have source, compile the app against ESP-IDF 4.0 release by running command as `make` in `slave_driver/network_adapter` directory. Program the WROVER-KIT using standard flash programming procedure with
```sh
$ make flash
```

## Checking the Setup
Once ESP32 has a valid firmware and booted successfully, you should be able to see successful enumeration on Raspberry Pi side as:
```sh
$ dmesg
[  143.606119] mmc1: queuing unknown CIS tuple 0x01 (3 bytes)
[  143.613524] mmc1: queuing unknown CIS tuple 0x1a (5 bytes)
[  143.617844] mmc1: queuing unknown CIS tuple 0x1b (8 bytes)
[  143.620070] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[  143.620167] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[  143.620265] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[  143.622073] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[  143.622169] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[  143.622266] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[  143.622461] mmc1: new SDIO card at address 0001
[  148.095780] esp32: loading out-of-tree module taints kernel.
[  148.314969] Initialising ESP Serial support
[  148.320686] esp32_probe: ESP network device detected
```
Once the module is inserted, you should see ethap0 and ethsta0 interfaces using _ifconfig_ command.

# Protocol Definition
This driver is divided in 3 parts:
1. Network driver - This part registers a network interface with linux kernel and implements netdev ops
2. SDIO driver - This is a transport layer. It interacts with wlan module on ESP32 through sdio interface
3. Serial driver - This provides a control interface through which wlan module on ESP32 can be controlled. This basically uses SDIO for low level transport.

The following section explains the protocol of SDIO driver i.e. transport layer.
ESP32 module advertises 2 SDIO functions, of which function 1 implements WLAN slave device. Though function 2 is advertised, it is not in use.

## Important registers provided by ESP32 wlan device
* 0x3FF5508C: Interrupt vector used by host to interrupt slave
* 0x3FF55058: Interrupt status register used by slave to interrupt host
* 0x3FF55060: Accumulated value of data length sent by slave
* 0x3FF55044: Accumulated number of buffers for receiving packets at slave

## Initialization of slave device
1. Soft reset sdio slave
	1. Host resets sdio part of slave by setting bit 2 of register at 0x3FF5508C
	2. This generates an interrupt for slave device, on which firmware on slave resets its sdio related data structures.
2. Host reads accumulated length and buffer count at slave.
	1. Host reads and processes 0x3FF55060 and 0x3FF55044 registers and stores the values in it's data structure
	2. These counters are required while performing read and write operation on SDIO interface
3. Open data path on slave
	1. Host sets 0th bit of 0x3FF5508C interrupt register
	2. This indicates slave that host is ready for data transmission

## Data transfer from Host to slave
1. Get Buffer count
	1. Host reads the current buffer count from slave [0x3FF55044]
	2. Based on that value, host calculates the number of available buffers at slave
	3. The host transfers the packet only when slave has required number of free buffers.
	4. Size of a buffer at slave is 2048 bytes
2. The host transfers data in multiples of 512 bytes and max data length per write operation is limited to buffer size [2048 bytes]
3. Host then updates it's own counter that keeps track of number of buffers it has transmitted.

## Data transfer from slave to host
1. Whenever slave has data to tranfser, it updates the length in 0x3FF55060 registers and generates an interrupt for host.
2. On interruption, host reads interrupt status register [0x3FF55058]. Bit 23 of this register tells host that slave desires to send data.
3. Host then gets the length set by slave by reading register mentioned in step 1. Based on previous received byte count and this length, host understands the actual length of data packet.
4. Host performs read operation to get data from slave
5. Once it receives the data, it updates it's counter that stores byte count received from slave.

`Note: Slave stays in blocked state during steps 1 to 4 [ i.e till host reads the data packet]`

## Deinit slave device
Host sets bit 1 of 0x3FF5508C interrupt register. This tells slave device to stop the data path.

## Payload format for data transfer
* Host and slave makes use of 8 byte payload header which preceeds every data packet. This payload header provides additional information about the data packet. Based on this header, host/slave consumes transmitted data packet.

* Payload format is as below

| Field | Length | Description |
|:-------:|:---------:|:--------:|
| packet type | 2 bits | Not in use |
| interface type | 3 bits | possible values: STA(0), AP(1), Serial interface(2). Rest all values are reserved |
| interface number | 3 bits | Not in use |
| reserved | 1 byte | Not in use |
| packet length | 2 bytes | Actual length of data packet |
| offset to packet | 2 bytes | Offset of actual data packet |
| reserved | 2 bytes  | Not in use |

# How to Run scripts on Raspberry-Pi(rpi)
## For Wi-Fi Connectivity
There is `host_comm` folder in which `host_commands` python library is present. It contains following functions.
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

User can make use of these python functions to get access of wifi functionalities of ESP32. Also in `host_comm/host_commands` folder `test.py` python script present to try those python functions. User can use it by running following command.
```
python test.py
```

first run `./rpi_init.sh wlan` to compile and insert ESP32 host driver on rpi. This script also creates `/dev/esps0` which is used as WLAN control interface.

There are six python script for station connect to AP, station disconnect from AP and softAP configuration.

1. `station_connect.py` is a python script which configure ESP32 in `station mode`, connects rpi to external AP with credentials user has provided. Also it ups the station interface and run DHCP client. User should provide parameters like ssid, password, mac address of AP(Its optional parameter).

```
ex. python station_connect.py 'xyz' 'xyz123456' --bssid='e5:6c:67:3c:cf:65'
```
2. `station_disconnect.py` is a python script to disconnect ESP32 station from AP.

```
python station_disconnect.py
```
3. `softap_config.py` is a python script for configure ESP32 `softAP mode`. User should provide parameters like ssid, password(password length should be 8~64 bytes ASCII), channel ID (It can be any number between 1 to 11), encryption method (0 : OPEN, 2: WPA_PSK, 3:WPA2_PSK, 4: WPA_WPA2_PSK), max connection count( number of Stations to which ESP32 SoftAP can be connected, within the range of [1, 10]) and ssid hidden (it can set to 1 if softAP shouldnt broadcast its ssid else 0). max connection count and ssid hidden parameters are optional.

```
ex. python softap_config.py 'xyz' 'xyz123456' 1 3 --max_conn=4 --ssid_hidden=0
```
---
Note: To start data connection, user needs to setup a DHCP server on rpi or set static IP address for AP interface i.e. ethap0

---
4. `softap_stop.py` is python script to stop ESP32 softap. This script will change wifi mode to `null` if only softAP is running or to `station` mode if softAP and station both are on.

```
ex. python softap_stop.py
```
5. `ap_scan_list.py` is python script which gives scanned list of available APs. list contains ssid, channel number, rssi, mac address and authentication mode of AP.
```
ex. python ap_scan_list.py
```
=======
---
Note: To start data connection, user needs to setup a DHCP server on rpi or set static IP address for AP interface i.e. ethap0

---
6. `connected_stations_list.py` is python script returns list of mac addresses of stations connected to softAP.

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
1. Execute `./rpi_init.sh bt` to prepare RPi for Bluetooth operation
2. Execute `hciattach` command as below to add hci interface
```
$ sudo hciattach -s 115200 /dev/serial0 any 115200 flow
```
3. Using following command verify that bluetooth is not blocked on raspberry pi
```
rfkill list
```
expected output should be:
```
1: hci0: Bluetooth
	Soft blocked: no
	Hard blocked: no
```
Use below command to unblock bluetooth if it is blocked
```
rfkill unblock bluetooth
```
check `rfkill list` once.

4. Perform bluetooth/ble operations
```
e.g.
$ hcitool scan
```

## For simulteneous usage of Bluetooth/BLE and WLAN
1. Execute `./rpi_init.sh` without any arguments. This will setup RPi for both bluetooth and wlan operations.
2. Follow steps mentioned in above section to initialize and use bluetooth and wlan
