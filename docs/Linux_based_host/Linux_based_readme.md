# Getting Started with Linux based host

Below diagram shows hardware and software block diagram for a typical linux based system built with ESP-Hosted.

![ESP-Hosted linux based design](./linux_hosted_design.png)

This document explains ESP-Hosted setup and usage. The document is divided in two parts:
* [Quick Start Guide](#1-quick-start-guide)  
	This section briefly explains ESP-Hosted setup. One can refer this guide to quickly prepare and test ESP-Hosted solution.

* [ESP-Hosted Comprehensive Guide](#2-esp-hosted-comprehensive-guide)  
	This section provides in depth information about ESP-Hosted setup, available customization options etc.

# 1. Quick Start Guide
* With the help of this guide, one can easily setup and start using ESP-Hosted solution with Raspberry-Pi as a host.
* This section makes use of pre-built ESP firmware release binaries and default configuration.
* Please refer [ESP-Hosted Comprehensive Guide](#2-esp-hosted-comprehensive-guide) for more details and customization options.

### 1.1 Hardware Requirements
* Raspberry-Pi model 3 Model B/B+ or Raspberry-Pi 4 model B
* ESP32 board
* 8-12 jumper wires of length < 10cm

### 1.2 Host Setup
Make sure that Raspberry-Pi is equipped with following:
* Linux Kernel Headers are installed
	```sh
	$ sudo apt update
	$ sudo apt install raspberrypi-kernel-headers
	```
* Following tools are installed on Raspberry-Pi:
	* Git
	* Raspi-gpio utility
	* bluetooth
	* bluez
	* bluez-tools
	* rfkill
	* bluez-firmware
	* pi-bluetooth
	```sh
	$ sudo apt install git raspi-gpio bluetooth bluez bluez-tools rfkill bluez-firmware pi-bluetooth
	```
* Python Requirement
	* Python 2.x or 3.x
		```sh
		$ sudo apt install python
		```
		 or
		```sh
		$ sudo apt install python3
		```
* Clone ESP-Hosted code repository
	```
	$ git clone --recurse-submodules <url_of_esp_hosted_repository>
	$ cd esp-hosted
	$ git submodule update --init --recursive
	```
* Download pre-built ESP-Hosted firmware release binaries from [releases](https://github.com/espressif/esp-hosted/releases)


### 1.3 Setup

Prepare connections based on interface requirements and setup host as below.

* **Wifi and Bluetooth over SDIO**
	* Connection Setup
		* Prepare connections as per section [1.1 Hardware Setup](SDIO_setup.md#11-hardware-setup) of [SDIO setup document](SDIO_setup.md)
	* Host Software
		* Prepare Raspberry-Pi as per section [1.2 Raspberry-Pi Software Setup](SDIO_setup.md#12-raspberry-pi-software-setup) of [SDIO setup document](SDIO_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd host/linux/host_control/
			$ ./rpi_init.sh sdio
			```
* **Wifi and Bluetooth over SPI**
	* Connection Setup
		* Prepare connections as per section [1.1 Hardware Setup](SPI_setup.md#11-hardware-setup) of [SPI Setup document](SPI_setup.md)
	* Host Software
		* Prepare Raspberry-Pi as per [1.2 Raspberry-Pi Software Setup](SPI_setup.md#12-raspberry-pi-software-setup) of [SPI Setup document](SPI_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd host/linux/host_control/
			$ ./rpi_init.sh spi
			```
* **Wifi over SDIO and Bluetooth over UART**
	* Connection Setup
		* Prepare SDIO connections as per section [1.1 Hardware Setup](SDIO_setup.md#11-hardware-setup) of [SDIO setup document](SDIO_setup.md)
		* Prepare UART connections as per section [1.1 Hardware Setup](UART_setup.md#11-hardware-setup) of [UART setup document](UART_setup.md)
	* Host Software
		* Prepare Raspberry-Pi for SDIO operations as per section [1.2 Raspberry-Pi Software Setup](SDIO_setup.md#12-raspberry-pi-software-setup) of [SDIO setup document](SDIO_setup.md)
		* Prepare Raspberry-Pi for UART operations as per section [1.2 Raspberry-Pi Software Setup](UART_setup.md#12-raspberry-pi-software-setup) of [UART setup document](UART_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd host/linux/host_control/
			$ ./rpi_init.sh sdio btuart
			```
		* After loading ESP firmware, execute below command to create `hci0` interface
			```sh
			$ sudo hciattach -s 921600 /dev/serial0 any 921600 flow

* **Wifi over SPI and Bluetooth over UART**
	* Connection Setup
		* Prepare connections as per section [1.1 Hardware Setup](SPI_setup.md#11-hardware-setup) of [SPI Setup document](SPI_setup.md)
		* Prepare UART connections as per section [1.1 Hardware Setup](UART_setup.md#11-hardware-setup) of [UART setup document](UART_setup.md)
	* Host Software
		* Prepare Raspberry-Pi as per [1.2 Raspberry-Pi Software Setup](SPI_setup.md#12-raspberry-pi-software-setup) of [SPI Setup document](SPI_setup.md)
		* Prepare Raspberry-Pi for UART operations as per section [1.2 Raspberry-Pi Software Setup](UART_setup.md#12-raspberry-pi-software-setup) of [UART setup document](UART_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd host/linux/host_control/
			$ ./rpi_init.sh spi btuart
			```
		* After loading ESP firmware, execute below command to create `hci0` interface
			```sh
			$ sudo hciattach -s 921600 /dev/serial0 any 921600 flow
			```
			```

#### 1.3.1 ESP Firmware Setup
* Flash pre-built binaries as below.
```sh
$ esptool.py -p <serial_port> -b 960000 --before default_reset --after hard_reset \
write_flash --flash_mode dio --flash_freq 40m --flash_size detect \
0x8000 esp_hosted_partition-table_<esp_peripheral>_<interface_type>_v<release_version>.bin \
0x1000 esp_hosted_bootloader_<esp_peripheral>_<interface_type>_v<release_version>.bin \
0x10000 esp_hosted_firmware_<esp_peripheral>_<interface_type>_v<release_version>.bin

Where,
	<serial_port>    : serial port of ESP peripheral
	<esp_peripheral> : esp32/esp32s2/esp32c3
	<interface_type> : sdio/spi/sdio_uart
	<release_version>: 0.1,0.2 etc
```

#### 1.3.2 Setup Validation
Once everything is setup and host software and ESP firmware are loaded
* Verify that `ethsta0` and `ethap0` interfaces are seen on host using following command.
	```sh
	$ ifconfig -a
	```
* Verify that hci0 interface is present using below command
	```sh
	$ hciconfig
	```
* Proceed to section [3. ESP-Hosted Usage Guide](#3-esp-hosted-usage-guide) to test Wi-Fi and Bluetooth/BLE functionality.


# 2. ESP-Hosted Comprehensive Guide

### 2.1 Linux Host: Development Environment Setup
* This section list downs environment setup and tools needed to make ESP-Hosted solution work with Linux based host.
* If you are using Raspberry-Pi as a Linux host, both [section 2.1.1](#211-raspberry-pi-specific-setup) and [section 2.1.2](#212-additional-setup) are applicable.
* If you are using other Linux platform, skip to [section 2.1.2](#212-additional-setup)

#### 2.1.1 Raspberry-Pi Specific Setup
This section identifies Raspberry-Pi specific setup requirements.

* Linux Kernel Setup
	* We recommend full version Raspbian install on Raspberry-Pi to ensure easy driver compilation.
	* Please make sure to use kernel version `v4.19` and above. Prior kernel versions may work, but are not tested.
	* Kernel headers are required for driver compilation. Please install them as:
	```sh
	$ sudo apt update
	$ sudo apt install raspberrypi-kernel-headers
	```
	* Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed correctly. In such case, follow https://github.com/notro/rpi-source/wiki and run `rpi-source` to get current kernel headers. Alternatively upgrade/downgrade kernel and reinstall kernel headers.
	```sh
	$ ls /lib/modules/$(uname -r)/build
	```

* Additional Tools
	* Raspi-gpio utility:
		```sh
		$ sudo apt install raspi-gpio
		```
	* Bluetooth Stack and utilities:
		```sh
		$ sudo apt install pi-bluetooth
		```

#### 2.1.2 Additional Setup
* Linux Kernel setup on non Raspberry-Pi
	* Please make sure to use kernel version `v4.19` and above. Prior kernel versions may work, but are not tested.
	* Please install kernel headers as those are required for driver compilation.
	* Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed correctly.
	```sh
	$ ls /lib/modules/$(uname -r)/build
	```

* Install following tools on Linux Host machine.
	* Git
	* Python 2.x or 3.x: We have tested ESP-Hosted solution with python 2.7.13 and 3.5.3
	* Bluetooth Stack and utilities:
	  :warning:`Note: We have tested ESP-Hosted solution with bluez 5.43+`
		* bluetooth
		* bluez
		* bluez-tools
		* rfkill
		* bluez-firmware

### 2.2 ESP-IDF Setup
:warning:`Note: ESP-IDF is needed to compile ESP-Hosted firmware source. Skip this step if you are planning to use pre-built release binaries.`  

ESP-IDF release version to be used for ESP peripherals are

| ESP peripheral | ESP-IDF release |
|:----:|:----:|
| ESP32 | release v4.0 |
| ESP32-S2 | release v4.2 |
| ESP32-C3 | release v4.3 |

Clone appropriate ESP-IDF version as per your ESP peripheral. The control path between Linux host and ESP peripheral is based on `protobuf`. For that, corresponding stack layer, `protocomm` from ESP-IDF is used. Run following command in ESP-IDF directory to make `protocomm_priv.h` available for control path.
```
$ git mv components/protocomm/src/common/protocomm_priv.h components/protocomm/include/common/
```

### 2.3 ESP-Hosted Code Repository
Clone esp-hosted repository on Linux host.
```
$ git clone --recurse-submodules <url_of_esp_hosted_repository>
$ cd esp-hosted
$ git submodule update --init --recursive
```

### 2.4 ESP-Hosted Setup and Load Project

ESP-Hosted solutions supports SDIO and SPI as transport for Wi-Fi and Bluetooth/BLE connectivity. Bluetooth/BLE connectivity is supported over UART as well. Follow below setup guides according to transport layer of your choice.

* [Wi-Fi and BT/BLE connectivity Setup over SDIO](SDIO_setup.md)

* [Wi-Fi and BT/BLE connectivity Setup over SPI](SPI_setup.md)

* [Bluetooth/BLE connectivity Setup over UART](UART_setup.md)


# 3. ESP-Hosted Usage Guide

Following guide explains how to use ESP-Hosted solution.
* [User Guide for ESP-Hosted with Linux Host](./Getting_started.md)


# 4. ESP-Hosted Porting Guide to other Linux Platforms

Following document explains guidelines for porting solution to othe Linux platforms
* [Porting Guide](./porting_guide.md)
