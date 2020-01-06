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
Please reboot Raspberry-Pi after changing this file. For compilation and insertion of the ESP32 host driver on Linux:
```sh
$ cd /path/to/esp_hosted/host_driver/esp32/
$ make
$ sudo insmod esp32.ko
```

## ESP32 Setup
On ESP32 either use pre-provided hosted mode firmware binary or if you have source, compile the app against ESP-IDF 3.3 release. Program the WROVER-KIT using standard flash programming procedure with
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
TBD
