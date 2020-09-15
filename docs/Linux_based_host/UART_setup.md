## Bluetooth/BLE connectivity Setup over UART
### Hardware Setup/Connections
In this setup, ESP32 board provides Bluetooth/BLE capabilities to host over UART interface. Please connect ESP32 board to Raspberry-Pi with jumper cables as below. As mentioned above, use small length cables.

| Raspberry-Pi Pin Function | Raspberry-Pi Pin | ESP32 Pin | ESP32 Pin Function |
|:-------:|:--------:|:---------:|:--------:|
| RX | 10 | IO5 | TX |
| TX | 8 | IO18 | RX |
| CTS | 36 | IO19 | RTS |
| RTS | 11 | IO23 | CTS |
| Ground | 39 | GND | Ground |

Power ESP32 and Raspberry Pi separately with a power supply that provide sufficient power. ESP32 can be powered through PC using micro-USB cable.

### Software setup
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
# systemctl disable hciuart
```
4. Reboot Raspberry-Pi

### ESP32 Setup

For pre built hosted mode firmware is present in `release` tab. Current binaries are made for UART baudrate of 921600. To flash it on ESP32 edit <serial_port> with ESP32's serial port and run following command.
```sh
esptool.py -p <serial_port> -b 960000 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size detect 0x8000 partition-table_sdio_uart_v0.2.bin 0x1000 bootloader_sdio_uart_v0.2.bin 0x10000 esp_hosted_firmware_sdio_uart_v0.2.bin
```
For windows user, you can also program the binaries using ESP Flash Programming Tool.

Or if you have source, compile the app against ESP-IDF 4.0 release. To use `make` build system, run following command in `esp/esp_driver/network_adapter` directory navigate to `Component config ->  Bluetooth -> Bluetooth controller -> HCI mode -> UART(H4) -> select` also to set baud rate, `Component config ->  Bluetooth -> Bluetooth controller ->  HCI UART(H4) Options -> UART Baudrate for HCI -> <set baudrate> -> Ok` and exit from menuconfig.
```
$ make menuconfig
```
run `make` in `esp/esp_driver/network_adapter` directory. Program the WROVER-KIT using standard flash programming procedure with `make`
```sh
$ make flash
```
Or to use `cmake` build system, run following command in `esp/esp_driver/network_adapter` directory navigate to `Component config ->  Bluetooth -> Bluetooth controller -> HCI mode -> UART(H4) -> select`  also to set baud rate, `Component config ->  Bluetooth -> Bluetooth controller ->  HCI UART(H4) Options -> UART Baudrate for HCI -> <set baudrate> -> Ok` and exit from menuconfig. Read more about [idf.py](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#using-the-build-system) here.
```
$ idf.py menuconfig
```
compile and flash the app on WROVER-KIT against ESP-IDF 4.0 release, by running following command in `esp/esp_driver/network_adapter` directory.

```sh
$ idf.py -p <serial_port> build flash
```
