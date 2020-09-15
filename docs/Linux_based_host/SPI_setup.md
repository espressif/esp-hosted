## Wi-Fi and BT/BLE connectivity Setup over SPI
### Hardware Setup/Connections
In this setup, ESP32 board acts as a SPI peripheral and provides Wi-FI capabilities to host. Please connect ESP32 board to Raspberry-Pi with jumper cables as mentioned below. It may be good to use small length cables to ensure signal integrity.

| Raspberry-Pi Pin | ESP32 Pin | Function |
|:-------:|:---------:|:--------:|
| 24 | IO5 | CS0 |
| 23 | IO18 | SCLK |
| 21 | IO19 | MISO |
| 19 | IO23 | MOSI |
| 25 | GND | Ground |
| 11 | IO2 | Handshake |
| 31 | EN  | ESP32 Reset |

Setup image is here.

![alt text](rpi_esp_spi_setup.jpg "setup of Raspberry-Pi as host and ESP32 as slave")

Power ESP32 and Raspberry Pi separately with a power supply that provide sufficient power. ESP32 can be powered through PC using micro-USB cable.

### Software setup
The SPI master driver is disabled by default on Raspberry Pi OS. To enable it add following commands in  _/boot/config.txt_ file
```
dtparam=spi=on
dtoverlay=disable-bt
```
Please reboot Raspberry-Pi after changing this file.


### ESP32 Setup

For pre built hosted mode firmware is present in `release` tab. To flash it on ESP32 edit <serial_port> with ESP32's serial port and run following command.
```sh
esptool.py -p <serial_port> -b 960000 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size detect 0x8000 partition-table_spi_v0.2.bin 0x1000 bootloader_spi_v0.2.bin 0x10000 esp_hosted_firmware_spi_v0.2.bin
```
For windows user, you can also program the binaries using ESP Flash Programming Tool.

Or if you have source, compile the app against ESP-IDF 4.0 release. To use `make` build system, run following command in `esp/esp_driver/network_adapter` directory and navigate to `Example Configuration ->  Transport layer -> SPI interface -> select` and exit from menuconfig.
```
$ make menuconfig
```
run `make` in `esp/esp_driver/network_adapter` directory. Program the WROVER-KIT using standard flash programming procedure with `make`
```sh
$ make flash
```
Or to select SPI transport layer using `cmake`, run following command in `esp/esp_driver/network_adapter` directory navigate to `Example Configuration -> Transport layer -> SPI interface -> select` and exit from menuconfig. Read more about [idf.py](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#using-the-build-system) here.
```
$ idf.py menuconfig
```
compile and flash the app on WROVER-KIT against ESP-IDF 4.0 release, by running following command in `esp/esp_driver/network_adapter` directory.

```sh
$ idf.py -p <serial_port> build flash
```

## Checking the Setup for SPI
Once ESP32 has a valid firmware and booted successfully, you should be able to see successful enumeration on Raspberry Pi side as:
```sh
$ dmesg
[   47.150740] OF: overlay: WARNING: memory leak will occur if overlay removed, property: /soc/spi@7e204000/spidev@0/status
[   47.346754] Bluetooth: Core ver 2.22
[   47.346812] NET: Registered protocol family 31
[   47.346815] Bluetooth: HCI device and connection manager initialized
[   47.346830] Bluetooth: HCI socket layer initialized
[   47.346837] Bluetooth: L2CAP socket layer initialized
[   47.346856] Bluetooth: SCO socket layer initialized
[   65.589406] esp32_spi: loading out-of-tree module taints kernel.
[   65.591409] esp32: Resetpin of Host is 6
[   65.591541] esp32: Triggering ESP reset.
[   65.593385] ESP32 device is registered to SPI bus [0],chip select [0]
[   66.201597] Received INIT event from esp32
[   66.201613] ESP32 capabilities: 0x78
[   66.619381] Bluetooth: BNEP (Ethernet Emulation) ver 1.3
[   66.619388] Bluetooth: BNEP filters: protocol multicast
[   66.619404] Bluetooth: BNEP socket layer initialized
```
