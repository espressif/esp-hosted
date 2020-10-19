# Setup
Currently we support ESP32 WROVER and ESP32-S2 as a ESP peripheral. We have tested ESP peripheral with Raspberry-Pi (3 Model B+, 4 Model B) for evaluation with Raspbian operating system.

## Raspberry-Pi Software Setup

### Kernel setup:
We recommend full version Raspbian install on Raspberry-Pi to ensure easy driver compilation. Please make sure to use kernel version `v4.19` and above. Prior kernel versions may work, but are not tested. Kernel headers are required for driver compilation. Please install them as:
```sh
$ sudo apt update
$ sudo apt-get install raspberrypi-kernel-headers
```
Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed correctly. In such case, follow https://github.com/notro/rpi-source/wiki and run rpi-source to get current kernel headers. Alternatively upgrade/downgrade kernel and reinstall kernel headers.
```sh
$ ls /lib/modules/$(uname -r)/build
```
### Dependencies:
```sh
$ sudo apt-get install git
```

### Python setup:
Python is required to run utility scripts. Please install it as:
```sh
$ sudo apt-get install python
or
$ sudo apt-get install python3
```
To start with control path on Raspberry-Pi, `protobuf` python modules is needed. User can install it by running following commands.
```
$ pip install protobuf
or
$ pip3 install protobuf
```
#### Note:
We have tested esp-hosted solution with python 2.7.13 and 3.5.3 and Protobuf version >= 3.13.0 .

### Source code repository:
* Clone esp-hosted repository on Raspberry Pi.
```
$ git clone --recurse-submodules <url_of_esp_hosted_repository>
$ cd esp-hosted
$ git submodule update --init --recursive
```

* To test BT/BLE functionality on Raspberry-Pi `bluez`(official Linux Bluetooth protocol stack) is needed. check if Raspberry-Pi has installed `bluez` or not.
[link that can help to install bluez](https://scribles.net/updating-bluez-on-raspberry-pi-from-5-43-to-5-50/). We have tested BT/BLE solution at bluez 5.43 and 5.45.

### Utilities:
* Raspi-gpio utility is required to configure GPIO pins. Please install it as:
```sh
$ sudo apt-get install raspi-gpio
```
* make sure Raspberry-Pi should have `bluetoothctl`, `bluetoothd`, `hcitool`, `hciconfig` utilities.

## ESP peripheral setup
ESP-IDF release branch to be used for ESP peripherals are

| ESP peripheral | ESP-IDF release |
|:----:|:----:|
| ESP32 | release v4.0 |
| ESP32-S2 | release v4.2 |

Clone appropriate ESP-IDF version as per your ESP peripheral. The control path between Raspberry-Pi and ESP peripheral is based on `protobuf`. For that `protocomm` layer from ESP-IDF is used. Run following command in ESP-IDF directory to make `protocomm_priv.h` available for control path.
```
git mv components/protocomm/src/common/protocomm_priv.h components/protocomm/include/common/
```

## Wi-Fi and BT/BLE connectivity

ESP-Hosted solutions supports SDIO and SPI as transport for Wi-Fi and Bluetooth/BLE. Bluetooth/BLE connectivity over UART. Follow below setup guides according to transport.

* [Wi-Fi and BT/BLE connectivity Setup over SDIO](SDIO_setup.md)

* [Wi-Fi and BT/BLE connectivity Setup over SPI](SPI_setup.md)

* [Bluetooth/BLE connectivity Setup over UART](UART_setup.md)

Once the setup is done, [Getting started](docs/Linux_based_host/Getting_started.md) section would help to establish Wi-Fi or Bluetooth connectivity.
