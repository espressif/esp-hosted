# Porting Guide

## 1. Porting ESP-Hosted Solution to Other Linux Platforms
As mentioned in earlier sections, Linux based ESP-Hosted solution supports Raspberry-Pi as a MCU host. It is fairly easy to use this solution on other Linux MCU platforms, but there are few parts of the solution that are tied to Raspberry-Pi. This document identifies such dependencies. One should take care of these dependencies while porting to other Linux platforms.

### 1.1 Hardware Connections

##### 1.1.1 Peripherals and GPIOs

When you are opting microprocessor with Linux other than Raspberry, hardware peripherals and GPIO functions would need changes. GPIO pins for [UART](UART_setup.md#11-hardware-setup), [SPI](SPI_setup.md#11-hardware-setup) and [SDIO](SDIO_setup.md#11-hardware-setup) would differ. 

##### 1.1.2 DT overlay

[Device Tree (DT)](https://www.raspberrypi.org/documentation/configuration/device-tree.md) and dtoverlay are way to express hardware and hardware configurations of them. While porting, Need to focus how configurations for [UART](UART_setup.md#12-raspberry-pi-software-setup), [SPI](SPI_setup.md#12-raspberry-pi-software-setup) and [SDIO](SDIO_setup.md#12-raspberry-pi-software-setup) are translated for your device. A care should be taken that only one or some of these configurations could be loaded at a time. This depends upon the board/chipset/SoC.

### 1.2 Software porting

##### 1.2.1 Kernel version
Driver underlies heavily over underlying kernel. ESP-Hosted is tested over Linux kernel version 4.19.X, although it should work on lower versions as well.
* HCI
	* HCI had compiled, but not tested for kernel version 3.18.16. Please refer [esp_bt_api.h](host/linux/host_driver/esp32/esp_bt_api.h) for details.
	* In Linux kernel, BT over SPI *i.e.* symbol *HCI_SPI* is supported over kernel version 4.7.0. Please refer [esp_bt.c](host/linux/host_driver/esp32/esp_bt.c). Ideally there should be no change required here.


##### 1.2.2 rpi_init.sh
* [rpi_init.sh](host/linux/host_control/rpi_init.sh) is utility script to load the ESP kernel module
* Reset Pin
	* Reset pin could be chosen over unused GPIO.
	* Input parameter to script, `resetpin=X` to be changed accordingly. This is used to reset ESP32 on loading ESP32 kernel module.
* UART configuration
	* `bt_init()` lists `raspi-gpio` commands.
	* `rapi-gpio` are simple utilities used to configure GPIO pins for their levels, alternate functions and pull up values.
	* As per raspberry Pi documentation, these UART pins are configured.
	* These pins and any other peripheral pins which are not already taken care in their drivers, need to be configured.
* spidev_disabler
	* This is applicable for SPI peripheral configuration and explained in next section.

##### 1.2.3 Peripheral configurations

* SPI
	* Disable default SPI driver
		- Linux kernel has a default SPI controller driver which needs to be disabled in order to make esp32_spi module work. Please see following code snippet from rpi_init.sh script

		```
		# Disable default spidev driver                                                              
		dtc spidev_disabler.dts -O dtb > spidev_disabler.dtbo
		sudo dtoverlay -d . spidev_disabler
		```
		While porting, equivalent commands or steps need to be run to disable default SPI driver.
	* Additional GPIOs
		- Apart from regular MOSI, MISO, CLK and Chip select, there are two additional GPIO pins used for SPI implementation.
		- These pins should be selected such that they would not interfere other any peripheral work.
		- Alter `HANDSHAKE_PIN` and `SPI_DATA_READY_PIN` in [esp_spi.h](host/linux/host_driver/esp32/spi/esp_spi.h).
		- Functionality of these additional pins could be read [here](../spi_protocol.md#111-additional-pin-setup).
	* cs_change
		- Reason why this setting was enabled is, SPI transfer was loosing first byte in transfer. Although this issue is only observed while testing with Raspberry Pi. Enabling cs_change=1 makes CS always de-assert after each transfer request. While porting, you may want to remove line, `trans.cs_change = 1;`.
	* SPI clock
		- ESP32 (WROVER and WROOM) supports 10MHz SPI clock.
		- Whereas ESP32S2 supports 40MHz SPI clock.
		- Raspberry Pi is tested with 10MHz clock with ESP32 and 30MHz with ESP32S2.
		- Raspberry Pi had issues while going to higher frequency than 30MHz.
		- While porting, the SPI clock frequency from low to high could be tested to optimize. Maximum SPI peripheral clock supported at ESP (slave) side is 40MHz.

### 1.3 Additional information
This is list of bug where developers have given their inputs. These are not tested/endorsed but are for informational purpose only.
- [Issue#23](https://github.com/espressif/esp-hosted/issues/23)

### Note:
While porting host from Raspberry-Pi, There should be no changes ESP side. But, it is better to keep in mind the expected peripheral counterpart in ESP and their configurations.

Please feel free to reach us at https://github.com/espressif/esp-hosted/issues in case of issues. 
