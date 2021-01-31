# Bluetooth/BLE connectivity Setup over UART
This section is only applicable to ESP32 boards. ESP32-S2 does not support Bluetooth/BLE.

## Setup
### Hardware Setup
In this setup, ESP32 board provides Bluetooth/BLE capabilities to host over UART interface. Please connect ESP peripheral to Raspberry-Pi with jumper cables as mentioned below. It may be good to use small length cables to ensure signal integrity. Power ESP32 and Raspberry Pi separately with a power supply that provide sufficient power. ESP32 can be powered through PC using micro-USB cable.

| Raspberry-Pi Pin Function | Raspberry-Pi Pin | ESP32 Pin | ESP32 Pin Function |
|:-------:|:--------:|:---------:|:--------:|
| RX | 10 | IO5 | TX |
| TX | 8 | IO18 | RX |
| CTS | 36 | IO19 | RTS |
| RTS | 11 | IO23 | CTS |
| Ground | 39 | GND | Ground |

Raspberry-Pi pinout can be found [here!](https://pinout.xyz/pinout/uart)

### Raspberry-Pi Software Setup
By default, the UART pins on Raspberry-Pi are in disabled state. In order to enable UART and setup it for bluetooth connection, follow below steps.
1. Enable UART pins and disable in built bluetooth on Raspberry-Pi by appending following lines to _/boot/config.txt_ file
```
enable_uart=1
dtoverlay=disable-bt
```
2. Remove following from _/boot/cmdline.txt_. Leave everything else untouched.
```
console=serial0,115200
```
e.g. If _/boot/cmdline.txt_ is as below:
```
# cat /boot/cmdline.txt
dwc_otg.lpm_enable=0 console=tty1 console=serial0,115200 root=PARTUUID=5c2c80d1-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles spidev.bufsiz=32768
````
Then after removal of above mentioned arguments, it should look as below:
```
# cat /boot/cmdline.txt
dwc_otg.lpm_enable=0 console=tty1 root=PARTUUID=5c2c80d1-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles spidev.bufsiz=32768
```
3. Disable hciuart on Raspberry-Pi
```
# sudo systemctl disable hciuart
```
4. Reboot Raspberry-Pi

## Load ESP-Hosted Solution
### Host Software
* Execute following commands in root directory of cloned ESP-Hosted repository on Raspberry-Pi
```sh
$ cd host/linux/host_control/
$ ./rpi_init.sh btuart
```

### ESP Peripheral Firmware
One can load pre-built release binaries on ESP peripheral or compile those from source. Below subsection explains both these methods.

#### ESP-IDF requirement
Please check [ESP-IDF Setup](Linux_based_readme.md#esp-idf-setup) and use appropriate ESP-IDF version

#### Load Pre-built Release Binaries
* Download pre-built firmware binaries from [releases](https://github.com/espressif/esp-hosted/releases)
* Please note that this binary is made for UART baudrate of 921600.
* Linux users can run below command to flash these binaries. Edit <serial_port> with ESP peripheral's serial port.
```sh
esptool.py -p <serial_port> -b 960000 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size detect 0x8000 partition-table_sdio_uart_v0.3.bin 0x1000 bootloader_sdio_uart_v0.2.bin 0x10000 esp_hosted_firmware_sdio_uart_v0.3.bin
```
* Windows user can use ESP Flash Programming Tool to flash the pre-built binary.

#### Source Compilation
* In root directory of ESP-Hosted repository, execute below command

```sh
$ cd esp/esp_driver/network_adapter
```

##### Using cmake
* Set target if the ESP32S2 is being used. Skip if ESP32 is being used.
```
$ idf.py set-target esp32s2
```
* Execute following command to configure the project
```sh
$ idf.py menuconfig
```
* This will open project configuration window.
	* Navigate to `Component config ->  Bluetooth -> Bluetooth controller -> HCI mode -> UART(H4) -> select`
	* Also to set baud rate by navigating to, `Component config ->  Bluetooth -> Bluetooth controller ->  HCI UART(H4) Options -> UART Baudrate for HCI -> <set baudrate> -> Ok`
	* exit from menuconfig.
* Use below command to compile and flash the project. Replace <serial_port> with ESP peripheral's serial port.
```sh
$ idf.py -p <serial_port> build flash
```

##### Using make
:warning: *make* build system is only supported till ESP32. Please refer cmake section above for ESP32-S2.
* Execute following command to configure the project
```sh
$ make menuconfig
```
* This will open project configuration window.
	* Navigate to `Component config ->  Bluetooth -> Bluetooth controller -> HCI mode -> UART(H4) -> select`
	* Also to set baud rate by navigating to, `Component config ->  Bluetooth -> Bluetooth controller ->  HCI UART(H4) Options -> UART Baudrate for HCI -> <set baudrate> -> Ok`
	* exit from menuconfig.
* Use below command to compile and flash the project
```sh
$ make flash
```
