# Hosted solution with ESP32
This project adds a capability to use ESP32 as a communication processor for Wi-Fi connectivity with an external host. This uses a subset of AT command-set for control path and uses a separate connection (currently supported on SDIO) for data path. The ESP32 provides a simple interface to the host to provide ethernet interface that can transmit and receive 802.3 frames. This allows the TCP/IP and higher level protocol stack to run on the host. The project provides ESP32 side firmware, example Linux driver and protocol description. This can directly be used with Linux based hosts or can easily be ported to other MCUs with available open protocol description.

# Setup
Currently we support ESP32 WROVER-Kit with Raspberry-Pi (3 Model B+, 4 Model B) for evaluation with Raspbian operating system.

## Hardware Setup / Connections
Please connect ESP32 board to Raspberry-Pi with jumper cables as mentioned below. It may be good to use small length cables to ensure signal integrity.

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

Power ESP32 and Raspberry Pi separately with a power supply that provide sufficient power. ESP32 can be powered through PC using micro-USB cable.

## Raspberry-Pi Software Setup
We recommend full version Raspbian install on Raspberry-Pi to ensure easy driver compilation. In addition for driver compilation, kernel headers are required. Please install them as:
```sh
$ sudo apt update
$ sudo apt-get install raspberrypi-kernel-headers
```
By default, the SDIO pins of Raspberry-pi are not configured and are internally used for built-in Wi-Fi interface. Please enable SDIO pins by appending following line to _/boot/config.txt_ file
```
dtoverlay=sdio,poll_once=off
```
Please reboot Raspberry-Pi after changing this file.

## ESP32 Setup
On ESP32 either use pre-provided hosted mode firmware binary or if you have source, compile the app against ESP-IDF 3.3 release by running command as `make SILENCE=0 ESP_AT_PROJECT_PLATFORM=esp32_at_core`. Program the WROVER-KIT using standard flash programming procedure with
```sh
$ make flash
```

## Checking the Setup
Once ESP32 has a valid firmware and booted successfully, you should be able to see successful enumeration on Raspberry Pi side as:
```sh
$ dmesg
[  769.531080] mmc1: queuing unknown CIS tuple 0x01 (3 bytes)
[  769.541087] mmc1: queuing unknown CIS tuple 0x1a (5 bytes)
[  769.545546] mmc1: queuing unknown CIS tuple 0x1b (8 bytes)
[  769.547832] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[  769.547928] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[  769.548023] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[  769.549804] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[  769.549900] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[  769.550000] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[  769.550195] mmc1: new SDIO card at address 0001
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
1. Reset slave
	1. Host resets slave by setting bit 2 of register at 0x3FF5508C
	2. This generates an interrupt for slave device, on which slave resets itself
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
There is `esp_at` folder in which "AT commands" python library is present. User can make use of python functions to get access of wifi functionalities of ESP32.

first run `./rpi_init.sh` to compile and insert ESP32 host driver on rpi. This script also creates `/dev/esps0` which is used as WLAN control interface.

There are three python script for station connect to AP, station disconnect from AP and softAP configuration.

1. `station_connect.py` is a python script which configure ESP32 in `station mode`, connects rpi to external AP with credentials user has provided. Also it ups the station interface and run DHCP client. User should provide parameters like ssid, password, mac address of AP(user can set mac address as 0 if doesnt want to set).
---
Note: This script should run in bash

---
```
ex.
python3 station_connect.py 'xyz' 'xyz123456' '0'
```
2. `station_disconnect.py` is a python script to disconnect ESP32 station from AP.

```
python3 station_disconnect.py
```
3. `softap_config.py` is a python script for configure ESP32 `softAP mode`. User should provide parameters like ssid, password(password length should be 8~64 bytes ASCII), channel ID (It can be any number between 1 to 11), encryption method (0 : OPEN, 2: WPA_PSK, 3:WPA2_PSK, 4: WPA_WPA2_PSK), max connection count( number of Stations to which ESP32 SoftAP can be connected, within the range of [1, 10]) and ssid hidden (it can set to 1 if softAP shouldnt broadcast its ssid else 0). max connection count and ssid hidden parameters are optional user can set this filed to 0.
---
Note: This script should run in bash

---

```
ex. python3 softap_config.py 'xyz' 'xyz123456' 1 3 4 0
```
---
Note: To start data connection, user needs to setup a DHCP server on rpi or set static IP address for AP interface i.e. ethap0

---
