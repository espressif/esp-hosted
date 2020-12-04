# ESP-Hosted solution

ESP-Hosted project provides a way to use ESP32 or ESP32-S2 (termed as ESP peripheral) as a communication processor for Wi-Fi and Bluetooth/BLE connectivity. On the host side, ESP-Hosted offers a standard network interface for receiving and transmitting 802.3 frames. On top of the network interface, the host can use its own TCP/IP and TLS stack. For BT connectivity, a standard HCI interface is exposed to the Bluetooth host stack.

Please note that this project doesn't provide a standard 802.11 interface to the host. For the control path between the host and ESP peripheral, ESP-Hosted uses a custom implementation based on [Protobuf](https://developers.google.com/protocol-buffers).

## Connectivity Support

ESP hosted solution provides following WLAN and BT/BLE features to host:
- WLAN Features:
	- 802.11b/g/n
	- WLAN Station
	- WLAN Soft AP
- BT/BLE Features:
	- v4.2 BR/EDR and BLE

## Supported Hosts

ESP-Hosted solution supports microprocessors running Linux as well as STM32 MCU (STM32F469I). It's possible to port this relatively easily on other MCUs too. We support Raspberry-Pi (3 Model B+, 4 Model B) and STM32 Discovery Board (STM32F469I-DISCO) out of the box.

## Supported Transports

ESP-Hosted uses SDIO or SPI interface as a transport interface for data and control path. The host acts as SDIO or SPI host whereas ESP32 or ESP32-S2 acts as a corresponding ESP peripheral. Currently Raspberry Pi supports both SDIO and SPI transport and STM32 supports SPI transport. As a peripheral device, ESP32 supports both SDIO and SPI transport interfaces and ESP32-S2 supports SPI transport interface.

For detailed explanation, please go through -
1. [Using Raspberry-Pi as a Linux host](docs/Linux_based_host/Linux_based_readme.md)
2. [Using STM32F469I-DISCO as a MCU host](docs/MCU_based_host/MCU_based_readme.md)

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
