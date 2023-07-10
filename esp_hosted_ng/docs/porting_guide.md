# Porting Guide

## Porting ESP-Hosted Solution to Other Linux Platforms
As mentioned in earlier sections, Linux based ESP-Hosted solution supports Raspberry-Pi as a MCU host. It is fairly easy to use this solution on other Linux MCU platforms, but there are few parts of the solution that are tied to Raspberry-Pi. This document identifies such dependencies. One should take care of these dependencies while porting to other Linux platforms.

# 1 Hardware Connections

## 1.1 SoC Power
- Please make sure to use exact power adapter and correct power cable which is expected by your platform
- If the power is not enough, the peripherals might not work or underperform

## 1.2 Peripherals and GPIOs

- When you are opting Linux other than Raspberry, hardware peripherals and GPIO functions would need changes. GPIO pins for [SDIO](setup.md), [SPI](setup.md), [UART](setup.md) and resetpin would differ.

###### Host side
- ResetPin GPIO
	- You need to choose any unused GPIO for reseting ESP and configure in your SoC's Device Tree Config
- Peripheral GPIOs
	- Check the pincontrol device tree blobs and verify if the correct GPIOs are in place for your expected SDIO hardware instance.
	- SDIO GPIOs
		- You may need to change connections from ESP to mapping pins for `SDIO_CLK`, `SDIO_CMD`, `SDIO_DAT0`, `SDIO_DAT1`, `SDIO_DAT2` and `SDIO_DAT3` from your SoC's Device Tree Pin Control.
	- SPI GPIOs
		- You may need to change connections from ESP to mapping pins for `SPI_ChipSelect`, `SPI_CLK`, `SPI_MISO`, `SPI_MOSI` from your SoC's Device Tree Pin Control.
		- Additionally you need another two unused GPIOs for `SPI_Handshake` and `SPI_DataReady` new pins to be defined in your DeviceTree

###### Slave side
- Changing GPIOs
	- SPI GPIOs
		- MISO, MOSI, CLK & CS are mapped to IO_MUX. They allowed to be changed with little overhead of GPIO Matrix routing.
		- Data Ready & Handshake could be chosed from any free GPIOs
	- SDIO GPIOs
		- Non changeable
	- UART GPIOs
		- 4 pin uart is suggested to use and all pins are configurable for ESP32/ESP32-C3/S3
		- 2 pin uart is available in ESP32-C2/C6. All pins configurable.
		- From 4 pin to 2 pin solution is possible by disabling hardware flow control at slave (& host).

## 1.3 Additional Pull-Ups
- SPI or UART peripherals
	- Chip Select is suggested to be externally pulled
- SDIO peripheral
	- In general, For Most of ESP32 boards, additional external pull-up of at least 10k Ohm resistor will be required for pins CMD and DATA(DAT0-DAT3) lines. 
	- In our PCB we use 51k Ohm pull-ups
	- Please go through a detailed document which details [pull-up requirement depending upon your ESP chipset](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html). In top left, choose your expected target ESP chipset from combo box.

## 1.4 DT overlay

- [Device Tree (DT)](https://www.raspberrypi.org/documentation/configuration/device-tree.md) and dtoverlay are way to express hardware and hardware configurations of them. While porting, It is important to understand and create device tree configuration for peripheral you use (SPI/SDIO/UART) for your Linux platform.
- Please note that every Linux platform or Soc vary in terms of device instances, their configurations. You might want to check the datasheet and GPIOs supported for the expected peripherals to be used.
- Please spare some time and Cross-check the parameters for peripherals, for example min or max frequncy of peripheral, peripheral config wrt timing phases of SPI protocol
- For SDIO, We suggest `broken-cd;` argument to be added/enabled DT which allows the ESP chipset hot-pluggable
- Please understand all the SoCs have different drivers and do not exactly follow one rule to make them work. Host Device Tree is user's responsibility to configure
- ESP-Hosted expects/assumes the platform with correct Device Tree or configuration is up. Although we try to help, but unfortunately, most of the bugs we received generally end up in host platform issues, which are in fact not ESP-Hosted software bugs.

## 1.5 Power adapters
- Power your Linux SoC with a correct adapter, which confirms the expected power rating.
- If correct power rated adapters are not used, the peripherals may poorly perform or may not work at all.
- ESP development boards can be powered with microUSB/USB-C cables from your computer.

# 2 Software porting

## 2.1 Kernel version
Driver underlies heavily over underlying kernel. ESP-Hosted is tested over Linux kernel version 4.19.X, although it should work on lower versions as well.
- HCI in older kernel versions
	- HCI had compiled, but not tested for kernel version 3.18.16. Please refer [esp_bt_api.h](../../host/linux/host_driver/esp32/esp_bt_api.h) for details.
	- In Linux kernel, BT over SPI *i.e.* symbol *HCI_SPI* is supported over kernel version 4.7.0. Please refer [esp_bt.c](../../host/linux/host_driver/esp32/esp_bt.c). Ideally there should be no change required here.

## 2.2 rpi_init.sh
* [rpi_init.sh](../host/rpi_init.sh) is utility script to load the ESP kernel module
* You can rename script as per your convenience, although it needs to be ported for kernel module building
* Reset Pin
	* Reset pin could be chosen over unused GPIO.
	* Input parameter to script, `resetpin=X` to be changed accordingly. This is used to reset ESP on loading the kernel module.
* UART configuration
	* `bt_init()` lists `raspi-gpio` commands.
	* `rapi-gpio` are simple utilities used to configure GPIO pins for their levels, alternate functions and pull up values.
	* As per raspberry Pi documentation, these UART pins are configured.
	* These pins and any other peripheral pins which are not already taken care in their drivers, need to be configured.
* spidev_disabler
	* This is applicable for SPI peripheral configuration and explained in next section.

## 2.3 Cross Compilation

* Kernel Module
	* Within [rpi_init.sh](../../host/linux/host_control/rpi_init.sh), make command used is Raspberry Pi specific. Following tags in `make` command should be changed as per platform
	- CROSS_COMPILE - This to be point to toolchain path. For arm example, it should point till `<Toolchain-Path>/bin/arm-linux-gnueabihf-`
	- KERNEL - Place where kernel is checked out and built
	- ARCH - Expected architecture
	* For example, For architecture `arm64`, toolchain checked out at `/home/user1/arm64_toolchain/` and kernel source built at `/home/user1/arm64_kernel/`, cross compilation commands should look like,

```sh
make target=$IF_TYPE \
ARCH=arm64 \
CROSS_COMPILE=/home/user1/arm64_toolchain/bin/aarch64-linux-gnu- \
KERNEL=/home/user1/arm64_kernel
```
`target` may take have value, `spi` or `sdio`. It defaults to `sdio` if not provided.

## 2.4 Peripheral configurations

### 2.4.1 SPI

* Verify user space SPI driver
	- If the user space drivers like spidev works as expected in Tx & Rx, then we would be assured that the SPI Linux drivers for your SoCs are working fine & SPI bus is correctly configured

* Disable default SPI driver
		- Linux kernel has a default SPI controller driver which needs to be disabled in order to make esp32_spi module work. Please see following code snippet from rpi_init.sh script

		```
		# Disable default spidev driver
		dtc spidev_disabler.dts -O dtb > spidev_disabler.dtbo
		sudo dtoverlay -d . spidev_disabler
		```
		While porting, equivalent commands or steps need to be run to disable default SPI driver through the Device Tree for expected SoC.
* Verify disabling of spidev
		- Default spidev when not disabled, create a device file, /dev/spidevX.Y where X refers SPI bus and Y refers to Chip Select to be used.
		- For example, Say User disables SPI bus 1 and Chip select 0 through Device Tree, then verify after loading Device Tree changes, /dev/spidev1.0 is no more listed
* Additional GPIOs
		- Apart from regular MOSI, MISO, CLK and Chip select, there are two additional GPIO pins used for SPI implementation.
		- These pins should be selected such that they would not interfere other any peripheral work.
		- Alter `HANDSHAKE_PIN` and `SPI_DATA_READY_PIN` in [esp_spi.h](../host/spi/esp_spi.h).
		- Additional pins functionality details mentioned in [1.1.1 additional pin setup](../spi_protocol.md#111-additional-pin-setup) of [spi protocol documentation](../spi_protocol.md).
* cs_change
		- Reason why this setting was enabled is, SPI transfer was loosing first byte in transfer. Although this issue is only observed while testing with Raspberry Pi. Enabling cs_change=1 makes CS always de-assert after each transfer request. While porting, you may want to remove line, `trans.cs_change = 1;`.
* Tune the SPI slave clock
		- Maximum SPI slave clock supported by ESP chipsets are:
			- ESP32 : 10MHZ
			- ESP32-S2: 40MHz
			- ESP32-C2: 60MHz
			- ESP32-C3: 60MHz
			- ESP32-S3: 60MHz
		- Above frequencies cannot be used while using Raspberry Pi as SPI master because of its limitation. \
		  However, you can increase the SPI clock stepwise to see what maximum frequency works for your SoC.
		- Higher the frequency set, better the throughput would be.
		- SPI clock frequency could be changed from macro `SPI_CLK_MHZ` in `esp/esp_driver/network_adapter/main/spi_slave_api.c`
* Identify peripheral limitations
		- For Raspberry Pi, Please use
		  ```
		  core_freq=250
		  core_freq_min=250
		  ```
		  as mentioned in [SPI setup](SPI_setup.md#12-raspberry-pi-software-setup)
		- Raspberry Pi could not perform reliably when the SPI clock was set higher frequency than 30MHz
		- Any such limitation for your Soc should be checked. Also power saving modes and peripheral clocks for your platform should be known.
* SPI Bus instance and Chip select number
		- Default value for both is 0, _i.e._ SPI0 and chip select 0.
		- It could be changed using variables, `esp_board.bus_num` and `esp_board.chip_select` in function `spi_dev_init()` from file `host/linux/host_driver/esp32/spi/esp_spi.c`
* SPI mode
		- Refrain from using SPI mode 0 at Slave side (no limitation at host for this as such)
		- If the correct software settings are loaded, you can expect first event received from ESP to Host in dmesg
		- If first event is not received, you can change SPI mode at both ESP and Host to other values, lower SPI freq and retry
* SPI interrupt handlers
		- If even after porting only first event received and nothing works ahead, you can suspect the GPIOs `Handshake` and `DataReady` not correctly configured as Interrupts.
		- Try to add log `printk(KERN_ERR "%s\n",__func__);` in `spi_data_ready_interrupt_handler()` and `spi_interrupt_handler()` functions if the interrupts are hit by assessing dmesg log

#### SPI Timing issues -  Tuning

User verified things like:

* Reset Pin
	* On host reload of driver, ESP is reseting
* Data_Ready / Handshake
	* Debug logs in ISR are getting printed on manual logic change on GPIOs
* spidev is correctly disabled
	* host driver is able to claim the spi bus on expected chip select
* Shorten the jumper wires
	* Try to use all jumper wires of smaller than 6cm and all jumper wires be of same length.
* SPI frequency
	* Lower the frequency to 1MHz
* SPI modes
	* Try different SPI modes like SPI mode 1/2/3 (Make sure, both ESP & Host have both have same changed SPI mode in one testing).

**Despite all above trials**, the ESP-Hosted fail to get first INIT event from ESP to Host **OR** <br />
only first event is received and fails to work after that <br />
you can suspect SPI timings mismatch. In such case, tuning might be required

##### Adjust SPI timings

* `Tuning of SPI timing` is a one time activity for any ESP<->Host setup.
* Please refer to [SPI timing considerations](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#speed-and-timing-considerations). Scroll to top, Change your ESP chipset from first combobox for timing consideration for your ESP chipset.
* It is possible that the timings of the sampling and the MISO or MOSI transitions are not aligned within time limit. To correct timings:
	* It is always recommended to check the SPI timings on any Logic analyzer. This way timings could be understood accurately.

###### Option 1 - Tune SPI timing at HAL slave driver
* To adjust timing, Go to IDF setup (esp_hosted_fg/esp/esp_driver/esp-idf), in file components/hal/<esp_chipset>/include/hal/spi_ll.h, locate function, spi_ll_slave_set_mode()
	* In your currently configured SPI mode, tweak values of `ck_idle_edge`, `ck_i_edge`, `miso_delay_mode`, `miso_delay_num`, `mosi_delay_mode` and `mosi_delay_num` and retry on every change

###### Option 2 - Mismatch the slave and Host SPI modes
* Set different SPI modes at Slave and Host and test again

### 2.4.2 SDIO

* File: *esp/esp_driver/network_adapter/main/sdio_slave_api.c*
* If first event is not received at SDIO, you can tune `SDIO Timing`
	* SDIO driver timing could be tuned from ESP slave code
		Function `sdio_init()` ->  `sdio_slave_config_t config` -> `timing` structure element. this can be changed to `SDIO_SLAVE_TIMING_NSEND_PSAMPLE` or [other values](https://github.com/espressif/esp-idf/blob/release/v5.1/components/hal/include/hal/sdio_slave_types.h#L26-L38) to adjust sdio timing.
* For higher SDIO frequencies to be enabled, you can remove line `config.flags |= SDIO_SLAVE_FLAG_DEFAULT_SPEED;`


# 3. Troubleshooting

* Raw throughput testing
	* To test the transport is correctly ported and working fine, please run [Raw throughput](Raw_TP_Testing.md) tests in `ESP-> Host` and `Host->ESP` direction.
	* Reboot/Reset both ESP and Host on raw throughput direction change
	* In this testing, frames are pumped continously and throughput at transport is measured.
	* Reasonable speed in comparison to the Clock used should be deduced (no standard figure)
	* If the speed is very less, this could mean that the transport is not correctly ported

* Verify you loaded correct software
	* Sometimes a silly mistake could happen that we load inconsistent drivers at ESP and Linux.
		- For example, SDIO firmware loaded at ESP and running SPI esp32 module at Linux, will simply fail to work

* Troubleshoot guide
	* Check some common known issues in [Troubleshoot guide](Troubleshoot.md)


### Important Notes
- While porting host from Raspberry-Pi, There should be no changes ESP side. But, it is better to keep in mind the expected peripheral counterpart, their GPIOs used in ESP and their configurations.
- Getting Linux platform up and running, Getting the peripherals working for your specific platform is out of scope for ESP-Hosted project. Why? The problem is that, every Linux platform has its own drivers and own way of device configurations.
- Porting sometimes can take very less time and tireless but sometimes a time consuming process.
- Finally, We would be happy know how your board was ported. Please open an issue for this with starting string 'porting done: <your platform>'
This would be tremendous help for someone who is using your platform.

Please feel free to reach us at https://github.com/espressif/esp-hosted/issues in case of issues.
