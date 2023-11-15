# Bluetooth/BLE connectivity Setup over UART

| Supported Chipsets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S3 | ESP32-C6 |
| ------------------ | ----- | -------- | -------- | -------- | -------- |
| 4 line UART supported | yes | no | yes | yes | no |
| 2 line UART supported | yes | yes | yes | yes | yes |

In this section, ESP chipset provides a way to run Bluetooth/BLE over UART interface.
Please connect ESP peripheral to Raspberry-Pi with jumper cables (preferably PCB) as mentioned below.
It may be good to use small length cables to ensure signal integrity.
Raspberry Pi should be powered with correct incoming power rating.
ESP chipset can be powered through PC using micro-USB/port-C cable.
In case of ESP32-S3, For avoidance of doubt, You can power using [UART port](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html#description-of-components)

Points to note:
- In case you wish to reduce number of hardware lines, you may consider SPI_only or SDIO_only transports, where Wi-Fi and Bluetooth traffic is multiplexed on same bus and no need of extra UART pins.
- UART pin numbers are configurable
- If you want to switch from 4 line UART mode to 2 lines, hardware flow control need to be turned off
 - Use good quality small (smaller than 10cm) jumper wires, all equal length

## 1. Setup

Raspberry-Pi pinout can be found [here!](https://pinout.xyz/pinout/uart)

### 1.1 Hardware Setup

#### Four Line UART setup

| Raspberry-Pi Pin Function | Raspberry-Pi Pin | ESP32 | ESP32-S3 | ESP32-C3 | ESP32 Pin Function |
|:-------:|:--------:|:---------:|:--------:|:--------:|:--------:|
| RX | 10 | IO5 | IO17 | IO5 | TX |
| TX | 8 | IO18 | IO18 | IO18 | RX |
| CTS | 36 | IO19 | IO19 | IO19 | RTS |
| RTS | 11 | IO23 | IO20 | IO8 | CTS |

Sample SPI+UART setup image looks like:

![alt text](rpi_esp32s3_spi_uart_setup.jpg "setup of Raspberry-Pi as host and ESP32-S3 as slave with UART transport")

#### Two Line UART setup


| Raspberry-Pi Pin Function | Raspberry-Pi Pin | ESP32-C2 | ESP32-C6 | ESP Function |
|:-------:|:--------:|:---------:|:--------:|:--------:|
| RX | 10 | IO5 | IO5 | TX |
| TX | 8 | IO18 | IO12 | RX |

Note:
- ESP32-C2 only
	- Although, `HCI with UART` is supported on `ESP32-C2`, `Wi-Fi + Bluetooth` (together) when used with `SPI+UART` setup, Bluetooth on UART works fine but Wi-Fi on SPI faces low throughput problem. By the time this is rectified, please use 'SPI only' i.e. `HCI over SPI` and `Wi-Fi over SPI` transport combination. In `SPI only` setup, there is no such limitation.


### 1.2 Raspberry-Pi Software Setup
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
$ cat /boot/cmdline.txt
dwc_otg.lpm_enable=0 console=tty1 console=serial0,115200 root=PARTUUID=5c2c80d1-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles spidev.bufsiz=32768
````
Then after removal of above mentioned arguments, it should look as below:
```
$ cat /boot/cmdline.txt
dwc_otg.lpm_enable=0 console=tty1 root=PARTUUID=5c2c80d1-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles spidev.bufsiz=32768
```
3. Disable hciuart on Raspberry-Pi
```
$ sudo systemctl disable hciuart
```
4. Reboot Raspberry-Pi

## 2. Load ESP-Hosted Solution
### 2.1 Host Software
- Clone ESP-Hosted repository on Raspberry-Pi
- Build and load kernel module
```sh
$ cd esp_hosted_fg/host/linux/host_control/
$ ./rpi_init.sh <transport> <bt_over_uart>
```
	- <transport> can take value `sdio` or `spi`. Defaults to `sdio`
	- <bt_over_uart> can take value `btuart` or `btuart_2pins`.
	  - This option will setup HCI over UART
	  - :warning: Note:
	    - For ESP32-C2/C6, 2 pin UART (TX & RX only) is supported. So `btuart_2pins` should be used for ESP32-C2/C6.
		- For other chipsets, `btuart` should be used, where 4 pin UART (TX, RX, CTS, RTS) is used.

### 2.2 ESP Peripheral Firmware
One can load pre-built release binaries on ESP peripheral or compile those from source. Below subsection explains both these methods.

#### 2.2.1 Load Pre-built Release Binaries
- Download pre-built firmware binaries from [releases](https://github.com/espressif/esp-hosted/releases)
- Follow `readme.txt` from release tarball to flash the ESP binary
- :warning: Make sure that you use `Source code (zip)` for host building.
- Windows user can use ESP Flash Programming Tool to flash the pre-built binary.
- Collect firmware log
    - Use minicom or any similar terminal emulator with baud rate 115200 to fetch esp side logs on UART
```sh
$ minicom -D <serial_port>
```
serial_port is device where ESP chipset is detected. For example, /dev/ttyUSB0

#### 2.2.2 Source Compilation

Make sure that same code base (same git commit) is checked-out/copied at both, ESP and Host

##### Set-up ESP-IDF
- :warning: Omit this & move to `Configure, Build & Flash ESP firmware` step if IDF is already setup while SPI/SDIO setup
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

##### Configure, Build & Flash ESP firmware
- Set slave chipset environment
```
$ cd network_adapter
$ rm -rf sdkconfig build
$ idf.py set-target <esp_chipset>
```
where <esp_chipset> could be one from "esp32", "esp32s3", "esp32c2", "esp32c3", "esp32c6"

- Execute following command to configure the project
```sh
$ idf.py menuconfig
```

- Set ESP-Hosted transport
	- ESP32 / ESP32-C6
		- Select transport either SDIO or SPI
		- Navigate to `Example Configuration -> Transport layer` select `SDIO interface` or `SPI interface`, whichever expected
	- ESP32-C2/C3/S3
		- SPI is automatically selected. Nothing to be done, skip to next step

- Set Bluetooth over UART
	- ESP32 / ESP32-C3/S3
		- navigate to `Component config -> Bluetooth -> Bluetooth controller -> HCI mode`, select `UART(H4)`
	- ESP32-C2/C6
		- Navigate to `Component config -> Bluetooth -> Bluetooth -> Controller`, select `Enabled` (Should be enabled by default)
		- Navigate to `Component config -> Bluetooth -> Controller Options -> HCI Config -> Select HCI interface`, select `uart`
		- Navigate to `Component config > Bluetooth > Controller Options > HCI Config`
          - DISABLE `HCI uart Hardware Flow ctrl` (Should be disabled by default)
		- ESP32-C2
          - Change `HCI uart Tx gpio` to `5`
          - Change `HCI uart Rx gpio` to `18`
		- ESP32-C6
          - Change `HCI uart Tx gpio` to `5`
          - Change `HCI uart Rx gpio` to `12`

- Set UART baud rate
	Default HCI over baud rate is `921600`. In case need to change,
	- ESP32
		- Navigate and change using `Component config -> Bluetooth -> Bluetooth controller -> HCI UART(H4) Options -> UART Baudrate for HCI`
	- ESP32-C3/S3
		- Navigate and change using `Component config -> Example Configuration -> UART Baudrate for HCI`
	- ESP32-C2/C6
		- Navigate and change using `Component config -> Bluetooth -> Controller Options -> HCI Config` -> `HCI uart baudrate`
- Please remember the UART baud rate used at ESP, as it is needed while setting up host

- Additional settings
	- ESP32-C3
		- Select chip revision in addition. Navigate to `Component config → ESP32C3-Specific → Minimum Supported ESP32-C3 Revision` and select chip version of ESP32-C3.
	- Other ESP chipsets
		- None, skip to next step

- Build and flash the project. Replace <serial_port> with ESP peripheral's serial port.
```sh
$ idf.py -p <serial_port> build flash
```

## 3. Post Setup
- After setting up host and loading ESP firmware, execute below command to create `hci0` interface
	```sh
	$ sudo hciattach -s <baud_rate> /dev/serial0 any <baud_rate> flow
	```
- <baud_rate> should match UART baud rate while flashing ESP peripheral (Default: 921600)

### For ESP32
- Check `CONFIG_BT_HCI_UART_BAUDRATE` parameter in *esp_hosted_fg/esp/esp_driver/network_adapter/sdkconfig*
- Alternatively baud rate could be located using `idf.py menuconfig` at, `Component config ->  Bluetooth -> Bluetooth controller ->  HCI UART(H4) Options -> UART Baudrate for HCI`

### For ESP32-C3/S3
- Check `CONFIG_EXAMPLE_HCI_UART_BAUDRATE` parameter in *esp_hosted_fg/esp/esp_driver/network_adapter/sdkconfig*
- Alternatively baud rate could be located using `idf.py menuconfig` at, `Component config -> Example Configuration -> UART Baudrate for HCI`

### For ESP32-C2/C6
- Check `CONFIG_BT_LE_HCI_UART_BAUD` parameter in *esp_hosted_fg/esp/esp_driver/network_adapter/sdkconfig*
- Alternatively baud rate could be located using `idf.py menuconfig` at, `Component config > Bluetooth > Controller Options > HCI Config -> HCI uart baudrate`

- Check if UART is setup correctly
```
$ hciconfig
```
should list the hci interface with bus as 'UART' and interface should be 'UP'


## 4. Points to note

#### 4.1 Restart HCI on every reloading of kernel module
- Remove and reattach `HCI over UART`
  ```sh
  $ hciconfig -a
  $ sudo killall hciattach
  $ sudo hciattach -s <baud_rate> /dev/serial0 any <baud_rate> flow
  $ hciconfig -a
  ```
- <baud_rate> should match UART baud rate while flashing ESP peripheral

#### 4.2 Restart Bluetooth apps
- Any third party apps such as pulseaudio or alsa should be restarted after restarting HCI
