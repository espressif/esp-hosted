# Getting Started with Linux based host(Raspberry-Pi)

Below diagram shows hardware and software block diagram for a typical linux based system built with ESP-Hosted.

![ESP-Hosted linux based design](./linux_hosted_design.png)

# Development Environment Setup

## Preconditions

* Linux Kernel Setup
	We recommend full version Raspbian install on Raspberry-Pi to ensure easy driver compilation. Please make sure to use kernel version `v4.19` and above. Prior kernel versions may work, but are not tested. Kernel headers are required for driver compilation. Please install them as:
	```sh
	$ sudo apt update
	$ sudo apt-get install raspberrypi-kernel-headers
	```
	Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed correctly. In such case, follow https://github.com/notro/rpi-source/wiki and run rpi-source to get current kernel headers. Alternatively upgrade/downgrade kernel and reinstall kernel headers.
	```sh
	$ ls /lib/modules/$(uname -r)/build
	```

* Install following tools on Linux Host machine

	* Git:
		```sh
		$ sudo apt-get install git
		```

	* Python:
	  `Note: We have tested ESP-Hosted solution with python 2.7.13 and 3.5.3`
		```sh
		$ sudo apt-get install python
		or
		$ sudo apt-get install python3
		```

	* Protobuf:
	  `Note: We have tested ESP-Hosted solution with Protobuf version >= 3.13.0`
		```
		$ pip install protobuf
		or
		$ pip3 install protobuf
		```

	* Raspi-gpio utility:
		```sh
		$ sudo apt-get install raspi-gpio
		```

	* Bluetooth Stack and utilities:
	  `Note: We have tested ESP-Hosted solution with bluez 5.43 and 5.45`
		```sh
		$ sudo apt-get install bluetooth bluez bluez-tools rfkill
		$ sudo apt install bluez-firmware pi-bluetooth
		```

## ESP-IDF Setup
ESP-IDF release branch to be used for ESP peripherals are

| ESP peripheral | ESP-IDF release |
|:----:|:----:|
| ESP32 | release v4.0 |
| ESP32-S2 | release v4.2 |

Clone appropriate ESP-IDF version as per your ESP peripheral. The control path between Linux host and ESP peripheral is based on `protobuf`. For that, corresponding stack layer, `protocomm` from ESP-IDF is used. Run following command in ESP-IDF directory to make `protocomm_priv.h` available for control path.
```
git mv components/protocomm/src/common/protocomm_priv.h components/protocomm/include/common/
```

## ESP-Hosted Code Repository
Clone esp-hosted repository on Linux host.
```
$ git clone --recurse-submodules <url_of_esp_hosted_repository>
$ cd esp-hosted
$ git submodule update --init --recursive
```

# ESP-Hosted Setup and Load Project

ESP-Hosted solutions supports SDIO and SPI as transport for Wi-Fi and Bluetooth/BLE connectivity. Bluetooth/BLE connectivity is supported over UART as well. Follow below setup guides according to transport layer of your choice.

* [Wi-Fi and BT/BLE connectivity Setup over SDIO](SDIO_setup.md)

* [Wi-Fi and BT/BLE connectivity Setup over SPI](SPI_setup.md)

* [Bluetooth/BLE connectivity Setup over UART](UART_setup.md)


# ESP-Hosted Usage Guide

Following guide explains how to use ESP-Hosted solution.
* [User Guide for ESP-Hosted with Linux Host](./Getting_started.md)
