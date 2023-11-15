# Getting Started with Linux based host

- Directory structure for Linux based host is explained [here](directory_structure.md)
- Below diagram shows hardware and software block diagram for a typical linux based system built with ESP-Hosted.

![ESP-Hosted linux based design](./linux_hosted_design.svg)

- This document explains ESP-Hosted setup and usage. The document is divided in two parts:
  - [1. Quick Start Guide](#1-quick-start-guide)  
    - This section briefly explains ESP-Hosted setup. One can refer this guide to quickly prepare and test ESP-Hosted solution.

  - [2. ESP-Hosted Comprehensive Guide](#2-esp-hosted-comprehensive-guide)  
    - This section provides in depth information about ESP-Hosted setup, available customization options etc.

# 1. Quick Start Guide
* With the help of this guide, one can easily setup and start using ESP-Hosted solution with Raspberry-Pi as a host.
* This section makes use of pre-built ESP firmware release binaries and default configuration.
* Please refer [ESP-Hosted Comprehensive Guide](#2-esp-hosted-comprehensive-guide) for more details and customization options.

### 1.1 Hardware Requirements
* Raspberry-Pi model 3 Model B/B+ or Raspberry-Pi 4 model B
* ESP32/ESP32-S2/S3/C2/C3/C6 board
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
* Using released codebase
	* Download pre-built ESP-Hosted firmware release binaries from [releases](https://github.com/espressif/esp-hosted/releases)
	* :warning: Make sure that you use `Source code (zip)` in `Assets` fold with associated release for host building.
* Using latest master
	* Clone ESP-Hosted code repository
	```
	$ git clone --recurse-submodules <url_of_esp_hosted_repository>
	$ cd esp-hosted
	$ git submodule update --init --recursive
	```
	* Please use the exact same git commit for ESP flashing using source code.

### 1.3 Setup

Prepare connections based on interface requirements and setup host as below.

* **Wi-Fi and Bluetooth over SDIO**
	* Connection Setup
		* Prepare connections as per section [1.1 Hardware Setup](SDIO_setup.md#11-hardware-setup) of [SDIO setup document](SDIO_setup.md)
	* Host Software
		* Prepare Raspberry-Pi as per section [1.2 Raspberry-Pi Software Setup](SDIO_setup.md#12-raspberry-pi-software-setup) of [SDIO setup document](SDIO_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd esp_hosted_fg/host/linux/host_control/
			$ ./rpi_init.sh sdio
			```
* **Wi-Fi and Bluetooth over SPI**
	* Connection Setup
		* Prepare connections as per section [1.1 Hardware Setup](SPI_setup.md#11-hardware-setup) of [SPI Setup document](SPI_setup.md)
	* Host Software
		* Prepare Raspberry-Pi as per [1.2 Raspberry-Pi Software Setup](SPI_setup.md#12-raspberry-pi-software-setup) of [SPI Setup document](SPI_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd esp_hosted_fg/host/linux/host_control/
			$ ./rpi_init.sh spi
			```
* **Wi-Fi over SDIO and Bluetooth over UART**
	* Connection Setup
		* Prepare SDIO connections as per section [1.1 Hardware Setup](SDIO_setup.md#11-hardware-setup) of [SDIO setup document](SDIO_setup.md)
		* Prepare UART connections as per section [1.1 Hardware Setup](UART_setup.md#11-hardware-setup) of [UART setup document](UART_setup.md)
	* Host Software
		* Prepare Raspberry-Pi for SDIO operations as per section [1.2 Raspberry-Pi Software Setup](SDIO_setup.md#12-raspberry-pi-software-setup) of [SDIO setup document](SDIO_setup.md)
		* Prepare Raspberry-Pi for UART operations as per section [1.2 Raspberry-Pi Software Setup](UART_setup.md#12-raspberry-pi-software-setup) of [UART setup document](UART_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd esp_hosted_fg/host/linux/host_control/
			$ ./rpi_init.sh sdio btuart
			```
		* After loading ESP firmware, execute below command to create `hci0` interface
			```sh
			$ sudo hciattach -s 921600 /dev/serial0 any 921600 flow

* **Wi-Fi over SPI and Bluetooth over UART**
	* Connection Setup
		* Prepare connections as per section [1.1 Hardware Setup](SPI_setup.md#11-hardware-setup) of [SPI Setup document](SPI_setup.md)
		* Prepare UART connections as per section [1.1 Hardware Setup](UART_setup.md#11-hardware-setup) of [UART setup document](UART_setup.md)
	* Host Software
		* Prepare Raspberry-Pi as per [1.2 Raspberry-Pi Software Setup](SPI_setup.md#12-raspberry-pi-software-setup) of [SPI Setup document](SPI_setup.md)
		* Prepare Raspberry-Pi for UART operations as per section [1.2 Raspberry-Pi Software Setup](UART_setup.md#12-raspberry-pi-software-setup) of [UART setup document](UART_setup.md)
		* Compile and load host driver as below:
			```sh
			$ cd esp_hosted_fg/host/linux/host_control/
			$ ./rpi_init.sh spi btuart
			```
		* After loading ESP firmware, execute below command to create `hci0` interface
			```sh
			$ sudo hciattach -s 921600 /dev/serial0 any 921600 flow
			```

#### 1.3.1 ESP Firmware Setup
* Download pre-built firmware binaries from [releases](https://github.com/espressif/esp-hosted/releases)
* Follow `readme.txt` from release tarball to flash the ESP binary
* :warning: Make sure that you use `Source code (zip)` in `Assets` fold with associated release for host building.

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

Setup is required on two machines.
**First** to setup `ESP-Hosted Linux Host driver building` on Raspberry Pi
**Second** to create a development environment set-up on your native machine so that you can build the ESP-Hosted firmware and flash it on the ESP chipset.
The native machine here refers to your Windows/Linux/Mac desktop/laptop.
Although, if you prefer to use the released binaries for ESP firmware, second part of native machine setup can be skipped.

### 2.1 Linux Host: Development Environment Setup
* This section list downs environment setup and tools needed to make ESP-Hosted solution work with Linux based host.
* If you are using Raspberry-Pi as a Linux host, both [section 2.1.1](#211-raspberry-pi-specific-setup) and [section 2.1.2](#212-additional-setup) are applicable.
* If you are using other Linux platform, skip to [section 2.1.2](#212-additional-setup)

#### 2.1.1 Raspberry-Pi Specific Setup
This section identifies Raspberry-Pi specific setup requirements.

* Linux Kernel Setup
	* We recommend full version Raspberry Pi OS install on Raspberry-Pi to ensure easy driver compilation. 64bit OS is preferred, although 32bit OS is also supported.
	* Please make sure to use kernel version `v4.19` and above. Prior kernel versions may work, but are not tested.
	* Kernel headers are required for driver compilation. Please install them as:
	```sh
	$ sudo apt update
	$ sudo apt install raspberrypi-kernel-headers
	```
	* Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed correctly. In such case, follow https://github.com/RPi-Distro/rpi-source and run `rpi-source` to get current kernel headers. Alternatively upgrade/downgrade kernel and reinstall kernel headers.
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
    * For throughput testing
		```sh
		$ sudo apt install iperf #for iperf2
		$ sudo apt install iperf3 #for iperf3
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
	  :warning:`Note: ESP-Hosted related BR/EDR 4.2 and BLE 4.2 functionalities are tested with bluez 5.43+. Whereas BLE 5.0 functionalities are tested with bluez 5.45+.`
		* bluetooth
		* bluez
		* bluez-tools
		* rfkill
		* bluez-firmware
	* We suggest latest stable bluez version to be used. Any other bluetooth stack instead of bluez also could be used.

### 2.2 ESP-IDF Setup
- If you are going to use released firmware binaries, ESP-IDF setup is not required, please continue with `2.3 ESP-Hosted Code Repository` below.
- Follow steps hereon on your **native machine** (your Windows/Linux/Mac desktop/laptop)
- :warning: Following command is dangerous. It will revert all your local changes. Stash if need to keep them.
- Install the ESP-IDF using script
```sh
$ cd esp_hosted_fg/esp/esp_driver
$ cmake .
```
- Set-Up the build environment using
```sh
$ . ./esp-idf/export.sh
# Optionally, You can add alias for this command in ~/.bashrc for later use
```

### 2.3 ESP-Hosted Code Repository
Clone esp-hosted repository on Linux host.
```
$ git clone --recurse-submodules <url_of_esp_hosted_repository>
$ cd esp-hosted
$ git submodule update --init --recursive
```
Please make sure that ESP and host checkeout to **same git commit**.

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
