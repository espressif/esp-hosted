# Porting Guide

## Porting ESP-Hosted Solution to Other Linux Platforms

As mentioned in earlier sections, ESP-Hosted solution works out of box with Raspberry-Pi as a Linux host. It is fairly easy to use this solution on other Linux MCU platforms, but there are few parts of the solution that are tied to Raspberry-Pi. This document identifies such dependencies. One should take care of these dependencies while porting to other Linux platforms.

### 1 Hardware Connections

##### 1.1 Peripherals and GPIOs

- When you are opting Linux other than Raspberry, hardware peripherals and GPIO functions would need changes. GPIO pins for [SDIO](setup.md#21-sdio-hardware-setup), [SPI](setup.md#31-spi-hardware-setup) and resetpin would differ.

- ResetPin GPIO
	- You can choose any unused GPIO for reseting ESP
- Peripheral GPIOs
	- Check the pincontrol device tree blobs and verify if the correct GPIOs are in place for your expected SDIO hardware instance.
	- SDIO GPIOs
		- You may need to change connections from ESP to mapping pins for SDIO_CLK, SDIO_CMD, SDIO_DAT0, SDIO_DAT1, SDIO_DAT2 and SDIO_DAT3. 
	- SPI GPIOs
		- You may need to change connections from ESP to mapping pins for SPI_ChipSelect, SPI_CLK, SPI_MISO, SPI_MOSI
		- Additionally you can another two unused GPIOs for SPI_Handshake and SPI_DataReady pins


##### 1.2 DT overlay

- [Device Tree (DT)](https://www.raspberrypi.org/documentation/configuration/device-tree.md) and dtoverlay are way to express hardware and hardware configurations of them. While porting, It is important to understand and create device tree configuration for peripheral you use (SPI/SDIO/UART) for your Linux platform.
- Please note that every Linux platform or Soc vary in terms of device instances, their configurations. You might want to check the datasheet and GPIOs supported for the expected peripherals to be used.

##### 1.3 Additional Pull-Ups
- SPI or UART peripherals
	- Nothing additional pull-up requirement
- SDIO peripheral
	- In general, For Most of ESP32 boards, additional pull-up of 10 kOhm resistor will be required for pins CMD and DATA(DAT0-DAT3) lines.
	- Please go through a detailed document which details [pull-up requirement depending upon your ESP chipset](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)

##### 1.4 User space drivers

- It is always best to verify if your device tree configurations are working correctly or not by using some user space drivers (from Linux or some third party applications)
- If the user space drivers work as expected, then we can be assured that the Linux drivers for your SoCs are working fine.

##### 1.5 Power adapters
- Power your Linux SoC with a correct adapter, which confirms the expected power rating.
- If correct power rated adapters are not used, the peripherals may poorly perform or may not work at all.
- ESP chipset can be powered with microUSB cables from your computer.

### 2 Software porting

##### 2.1 Kernel version
Driver underlies heavily over underlying kernel. ESP-Hosted is tested over Linux kernel version 4.19.X, although it should work on lower versions as well.

##### 2.2 GPIOs

- Reset Pin
	- Reset pin could be chosen over any unused GPIO.
	- Input parameter to script, `resetpin=X` to be changed accordingly. This is used to reset ESP on loading the kernel module.

##### 2.3 cross compilation

* Kernel Module  
	Following tags in `make` command should be changed as per platform
	- CROSS_COMPILE - This to be point to toolchain path. For arm example, it should point till `<Toolchain-Path>/bin/arm-linux-gnueabihf-`
	- KERNEL - Place where kernel is checked out and built
	- ARCH - Expected architecture

For example, For architecture `arm64`, toolchain checked out at `/home/user1/arm64_toolchain/` and kernel source built at `/home/user1/arm64_kernel/`, cross compilation commands should look like,

```sh
make target=$IF_TYPE \
ARCH=arm64 \
CROSS_COMPILE=/home/user1/arm64_toolchain/bin/aarch64-linux-gnu- \
KERNEL=/home/user1/arm64_kernel
```
`target` may take have value, `spi` or `sdio`. It defaults to `sdio` if not provided.

### Important Note
- While porting host from Raspberry-Pi, There should be no changes ESP side. But, it is better to keep in mind the expected peripheral counterpart, their GPIOs used in ESP and their configurations.
- Getting Linux platform up and running, Getting the peripherals working for your specific platform is out of scope for ESP-Hosted project. Why? The problem is that, every Linux platform has its own drivers and own way of device configurations.
- Porting sometimes can take very less time and tireless but sometimes a time consuming process.
- Finally, We would be happy know how your board was ported. Please open an issue for this with starting string 'porting done: <your platform>'
This would be tremendous help for someone who is using your platform.

