# Introduction

ESP-Hosted solution provides a way to use ESP board as a communication processor for Wi-Fi and Bluetooth/BLE connectivity.

Following features are provided as a part of this solution:
	- A standard 802.3 network interface is provided to host for transmitting and receiving 802.3 frames
	- A standard HCI interface is provided to host over which Bluetooth/BLE is supported
	- A control interface to configure and control Wi-Fi on ESP board

For end to end working of this solution, host needs to be equipped with corresponding TCP/IP, TLS and Bluetooth/BLE stack.

Please note that this project doesn't provide a standard 802.11 interface to the host. For the control path between the host and ESP board, ESP-Hosted uses a custom implementation based on [Protobuf](https://developers.google.com/protocol-buffers).

## Connectivity Features

ESP-hosted solution provides following WLAN and BT/BLE features to host:
- WLAN Features:
	- 802.11b/g/n
	- WLAN Station
	- WLAN Soft AP
- BT/BLE Features:
	- v4.2 BR/EDR and BLE

## Supported ESP boards

ESP-Hosted solution is supported on following ESP boards:
- ESP32
- ESP32S2

## Supported Hosts

* ESP-Hosted solution supports following Linux based and MCU based hosts out of the box.
	* Linux Based Hosts
		* Raspberry-Pi 3 Model B
		* Raspberry-Pi 3 Model B+
		* Raspberry-Pi 4 Model B
	* MCU Based Hosts
		* STM32 Discovery Board (STM32F469I-DISCO)
* It is relatively easy to port this solution to other Linux and MCU platforms.

## Supported Transports

ESP-Hosted uses SDIO or SPI bus for interfacing ESP boards and host platform. Not all host platforms support both these interfaces. Further section depicts supported host platforms and corresponding transport interface, ESP boards and feature set.

## Feature Matrix
### Linux Host
Below table explains which feature is supported on which transport interface for Linux based host.

| ESP device | Transport Interface | WLAN support | Virtual serial interface | BT/BLE support |
|:---------:|:-------:|:---------:|:--------:|:--------:|
| ESP32 | SDIO | Yes | Yes | Yes |
| ESP32 | SPI | Yes | Yes | Yes |
| ESP32 | UART | No | No | Yes |
| ESP32-S2 | SDIO | NA | NA | NA |
| ESP32-S2 | SPI | Yes | Yes | NA |
| ESP32-S2 | UART | No | No | NA |

### MCU Host
Below table explains which feature is supported on which transport interface for MCU based host.

| ESP device | Transport Interface | WLAN support | Virtual serial interface | BT/BLE support |
|:---------:|:-------:|:---------:|:--------:|:--------:|
| ESP32 | SDIO | No | No | No |
| ESP32 | SPI | Yes | Yes | HCI interface can be implemented over virtual serial interface |
| ESP32 | UART | No | No | No |
| ESP32-S2 | SDIO | NA | NA | NA |
| ESP32-S2 | SPI | Yes | Yes | NA |
| ESP32-S2 | UART | No | No | NA |

---

# Get Started
This section describes how to setup and use ESP-Hosted solution. Since ESP-Hosted solution supports two distinct platforms, procedure to use it vastly differs. Please use following links to get started with platform of your choice.
1. [Using Raspberry-Pi as a Linux host](docs/Linux_based_host/Linux_based_readme.md)
2. [Using STM32F469I-DISCO as a MCU host](docs/MCU_based_host/MCU_based_readme.md)

---

# Design
This section describes the overall design of ESP-Hosted solution. There are 3 aspects to it:
* System Architecture
* Transport layer communication protocol
* API Reference

## System Architecture

This section discusses building blocks of the ESP-Hosted solution for the supported host platfroms.

These building blocks can be broadly classified as:
* **ESP Host Software**  
This includes ESP Host driver and control interface implementation.

* **ESP Firmware**  
This includes ESP peripheral driver and implementation of control commands.

* **Third party components**  
This includes components that are essential for end to end working of entire system but are not maintained or implemented as a part of this project.


### ESP Host Software

The components of ESP host software are dependent on host platform that is being used. Please refer following documents:

1. [System Architecture: Linux based host](docs/Linux_based_host/Linux_based_architecture.md)
2. [System Architecture: MCU based host](docs/MCU_based_host/MCU_based_architecture.md)


### ESP Firmware
This implements ESP-Hosted solution part that runs on ESP boards. ESP firmware is agnostic of the host platform. It consists of following.

* **ESP Application**  
This implements:
	* SDIO transport layer
	* SPI transport layer
	* Virtual serial interface driver
	* Control interface command implementation
	* Bridges data path between Wi-Fi, HCI controller driver of ESP and Host platform
* **ESP IDF Components**  
ESP firmware mainly uses following components from ESP IDF. Please check ESP IDF documentation (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for more details.
	* SDIO Slave driver
	* SPI Slave driver
	* Wi-Fi driver
	* HCI controller driver
	* Protocomm Layer


#### Third Party Components
Third components such as following are essential for end to end working of ESP-Hosted Solution. Implementation or porting of these third compoenent is not in scope of this project.
* TCP/IP and TLS stack
* BT/BLE stack
* UART driver


## Transport layer communication protocol
This section describes the data communication protocol used at the transport layer. This protocol is agnostic of host platforms that is being used. Please refer following links to get more information on communication protocol per transport interface type.
* [SDIO Communication Protocol](docs/sdio_protocol.md)
* [SPI Communication Protocol](docs/spi_protocol.md)

### Payload Format
This section explains the payload format used for data transfer on SDIO and SPI interfaces.

* Host and peripheral makes use of 8 byte payload header which preceeds every data packet.
* This payload header provides additional information about the data packet. Based on this header, host/peripheral consumes transmitted data packet.
* Payload format is as below

| Field | Length | Description |
|:-------:|:---------:|:--------|
| interface type | 4 bits | possible values: STA(0), SoftAP(1), Serial interface(2), HCI (3), Priv interface(4). Rest all values are reserved |
| interface number | 4 bits | Not in use |
| reserved | 1 byte | Not in use |
| packet length | 2 bytes | Actual length of data packet |
| offset to packet | 2 bytes | Offset of actual data packet |
| reserved | 1 byte  | Not in use |
| packet type | 1 byte | reserved when interface type is 0,1 and 2. Applicable only for interface type 3 and 4 |


## Integration Guide

### Control Interface API's
This section describes control interface API's provided by ESP-Hosted Solution For higher layer applications. One can easily integrate ESP-Hosted solution with other projects using these API's. There are two flavors of these API's:

* [Python API's](docs/python_api.md)
* [C API's](docs/c_api.md)

### API's for MCU Based ESP-Hosted Solution
Below document explains the API's provided for MCU based ESP-Hosted solution
* [API's for MCU based host](docs/MCU_based_host/mcu_api.md)
