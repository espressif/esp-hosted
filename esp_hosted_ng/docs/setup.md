# ESP-Hosted-NG Setup

## 1 Setup Introduction

- In this setup, ESP board acts as a SDIO/SPI/UART peripheral and provides Wi-Fi and/or Bluetooth capabilities to host. 
- This document explains ESP-Hosted setup and usage.:
- [1. Software setup](#1-software-setup)
    + [1.1 Host software setup](#11-host-software-setup)
    + [1.2 ESP Quick start guide](#12-esp-quick-start-guide)
    + [1.3 ESP Comprehensive guide](#13-esp-comprehensive-guide)
- [2. Transport layer configuration](#2-transport-layer-configuration)
    + [2.1 SPI configuration](#21-spi-configuration) Wi-Fi and Bluetooth over SPI
    + [2.2 SDIO configuration](#22-sdio-configuration) Wi-Fi and Bluetooth over SDIO
    + [2.3 SDIO/SPI + Uart configuration](#23-sdiospi-uart-configuration) Wi-Fi over SDIO/SPI and Bluetooth over UART
- [3. Troubleshoot Setup Problems](#3-troubleshoot-setup-problems)
- [4. Points to note](#4-points-to-note)

# 1. Software setup
* This section briefly explains software setup required for esp hosted device and host. Esp hosted device setup is divided into two parts 
    * [Host software setup](#11-host-software-setup) This section provides information about installation of required packages for host device
    * [ESP Quick start guide](#12-esp-quick-start-guide) This section briefly explains ESP-Hosted setup. One can refer this guide to quickly prepare and test ESP-Hosted solution
    * [ESP Comprehensive guide](#13-esp-comprehensive-guide) This section provides in depth information about ESP-Hosted setup, available customization options etc.
    *Use Quick start guide for easy setup or Comprehensive guide for customizable setup.*

### 1.1 Host software setup

Make sure that host machine is equipped with following:
* Linux Kernel Setup
	* We recommend full version Raspbian install on Raspberry-Pi to ensure easy driver compilation.
	* Please make sure to use kernel version `v4.19` and above. Prior kernel versions may work, but are not tested.
	* Kernel headers are required for driver compilation. Please install them as:
	    ```sh
	    $ sudo apt update
	    $ sudo apt install raspberrypi-kernel-headers
	    ```
	* Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed correctly. In such case, follow https://github.com/RPi-Distro/rpi-source/wiki and run `rpi-source` to get current kernel headers. Alternatively upgrade/downgrade kernel and reinstall kernel headers.
	    ```sh
	    $ ls /lib/modules/$(uname -r)/build/
	    ```
* Following tools are required on Raspberry-Pi:
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
	* We suggest latest stable bluez version to be used. Any other bluetooth stack instead of bluez also could be used.

* More transport layer specific host software setup can be found here [Transport layer configuration](#2-transport-layer-configuration)

### 1.2 ESP Quick Start Guide

* With the help of this guide, one can easily setup and start using ESP-Hosted solution with Raspberry-Pi as a host.
* This section makes use of pre-built ESP firmware release binaries and default configuration.
* Pre-built ESP firmware binaries are available on [releases](https://github.com/espressif/esp-hosted/releases).
* Please download the latest release.
* Windows user can use ESP Flash Programming Tool to flash the pre-built ESP binaries.
* Linux or Mac users can use [esptool.py](https://pypi.org/project/esptool/) to flash the pre-built ESP binaries. To install, please run:
    ```sh
    $ pip install esptool
    ```
*  Please browse to desired ESP chipset directory
    ```sh
    $ tar xvf ESP-Hosted-NG-release_1.x.y.tgz
    $ cd ESP-Hosted-NG-release_1.x.y/
    $ cd <esp_chipset>
    $ cd <transport_layer>
    ```
* here,
    * For SDIO
        * <esp_chipset> can be esp32.
        * <transport_layer> can be sdio_only or sdio+uart
    * For SPI
        * <esp_chipset> can be esp32, esp32c3, esp32s3,esp32c2.
        * <transport_layer> can be spi_only or spi+uart
* Run the flashing command from _flashing_cmd.txt_ file.
* Follow these steps to setup required configurations for desired transport layer [Transport layer configuration](#2-transport-layer-configuration)

### 1.3 ESP Comprehensive guide
- **Note on Windows 11**: you can follow [these instructions](/esp_hosted_ng/esp/esp_driver/README.md#building-on-windows-11-using-command-prompt) to setup ESP-IDF to build the esp firmware.
- :warning: **Following command is dangerous. It will revert all your local changes. Stash if need to keep them**.
- Install the ESP-IDF using script
    ```sh
    $ cd esp_hosted/esp_hosted_ng/esp/esp_driver
    $ cmake .
    ```
* This will clone the required esp-idf repository and will setup it up for esp hosted firmware
* Set-Up the esp firmware build environment using 
    ```sh
    $ . ./esp-idf/export.sh
    # Optionally, You can add alias for this command in ~/.bashrc for later use
    ``` 
* Once the environment is ready for esp firmware to be built follow these steps to setup required configurations for desired transport layer [Transport layer configuration](#2-transport-layer-configuration)

# 2. Transport layer configuration

* This section will provide all required configuration to be made for required desired transport layer. 
* Hardware requirements
    1) 8-12 jumper wires of length < 10cm.
    2) ESP32, ESP32 C3, ESP32 S3, ESP32 C2 board.
    3) Raspberry-Pi 3 model B/B+ or Raspberry-Pi 4 model B
* This is section is divided into following transport layers.
   1) [SPI configuration](#21-spi-configuration) Wi-Fi and Bluetooth over SPI
   2) [SDIO configuration](#22-sdio-configuration) Wifi and bluetooth over SDIO
   4) [SDIO/SPI + Uart configuration](#23-sdiospi-uart-configuration) Wi-Fi over SDIO and Bluetooth over UART

### 2.1 SPI configuration
**Wi-Fi and Bluetooth over SPI**

| Supported Targets | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C2 | ESP32-C3 | ESP32-C6 |
| ----------------- | ----- | -------- | -------- |--------- |--------- |--------- |
* Hardware setup
    * In this setup, ESP board acts as a SPI peripheral and provides Wi-Fi capabilities to host. Please connect ESP board to Raspberry-Pi with jumper cables as mentioned below. Please use short jumper cables to ensure signal integrity. Raspberry Pi should be powered with correct incoming power rating. ESP can be powered through PC using micro-USB/USB-C cable.
    * **Pin Connections**

        | Raspberry-Pi Pin | ESP32 | ESP32-S2/S3 | ESP32-C2/C3/C6 | Function |
        |:-------:|:---------:|:--------:|:--------:|:--------:|
        | 24 | IO15 | IO10 | IO10 | CS0 |
        | 23 | IO14 | IO12 | IO6 | SCLK |
        | 21 | IO12 | IO13 | IO2 | MISO |
        | 19 | IO13 | IO11 | IO7 | MOSI |
        | 25 | GND | GND | GND | Ground |
        | 15 | IO2 | IO2 | IO3 | Handshake |
        | 13 | IO4 | IO4 | IO4 | Data Ready |
        | 31 | EN  | RST | RST | ESP32 Reset |
    * Raspberry-Pi pinout can be found [here!](https://pinout.xyz/pinout/spi)
    * Optionally, Add external pull-up of min 10k Ohm on CS line just to prevent bus floating
    * In case of ESP32-S3, For avoidance of doubt, You can power using [UART port](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html#description-of-components)
* Raspberry-Pi Setup
    * The SPI master driver is disabled by default on Raspberry-Pi OS. To enable it add following commands in _/boot/config.txt_ file
        ```
        dtparam=spi=on
        dtoverlay=disable-bt
        ```
    * In addition, below options are set as the SPI clock frequency in analyzer is observed to be smaller than expected clock. This is RaspberryPi specific [issue](https://github.com/raspberrypi/linux/issues/2286).
        ```
        core_freq=250
        core_freq_min=250
        ```
    * Please reboot Raspberry-Pi after changing this file.
* Setting up the environment and getting started
    * Host environment setup  
        * Execute following commands in root directory of cloned ESP-Hosted repository on Raspberry-Pi
            ```sh
            $ cd esp_hosted/esp_hosted_ng/host/
            $ bash rpi_init.sh spi
            ```
        * This script compiles and loads host driver on Raspberry-Pi. It also creates virtual serial interface `/dev/espsta0` which is used as a control interface for Wi-Fi on ESP peripheral

    * For esp firmware if you are using [ESP Quick start guide](#12-esp-quick-start-guide)
        * Please flash the required binaries using with command mentioned in `flashing_cmd.txt` within desired transport configuration folder as explained in [ESP Quick start guide](#12-esp-quick-start-guide).  
        * Use minicom or any similar terminal emulator with baud rate 115200 to fetch esp side logs on UART
            ```sh
            $ minicom -D <serial_port>
            ```
        * serial_port is device where ESP chipset is detected. For example, /dev/ttyUSB0
    * For esp firmware if you are using [ESP Comprehensive guide](#13-esp-comprehensive-guide).
        * Set up the esp-idf environment as mentioned in [ESP Comprehensive guide](#13-esp-comprehensive-guide)
        * To configure slave chip set environment
            ```
            $ cd esp_hosted/esp_hosted_ng/esp/esp_driver/network_adapter
            $ rm -rf sdkconfig build
            $ idf.py set-target <esp_chipset>
            ```
        * For SPI, <esp_chipset> could be one of `esp32`, `esp32s2`, `esp32s3`, `esp32c2`, `esp32c3`, `esp32c6`.
        * Execute following command to configure the project
            ```
            $ idf.py menuconfig
            ```
        *  This will open project configuration window. To select SPI transport interface, navigate to `Example Configuration ->  Transport layer -> SPI interface -> select` and exit from menuconfig.
        * For ESP32-C3, select chip revision in addition. Navigate to `Component config → ESP32C3-Specific → Minimum Supported ESP32-C3 Revision` and select chip version of ESP32-C3.
        * Use below command to compile and flash the project. Replace <serial_port> with ESP peripheral's serial port.
            ```
            $ idf.py -p <serial_port> build flash
            ```
        * Collect the firmware log using
            ```
            $ idf.py -p <serial_port> monitor
            ```
* Checking the Setup
    * On successful flashing and host driver installation, you should see following entry in ESP log:
        ```
        [   77.877892] Features supported are:
        [   77.877901]   * WLAN
        [   77.877906]   * BT/BLE
        [   77.877911]     - HCI over SPI
        [   77.877916]     - BLE
        ```
* Once the one of ESP-Hosted-NG mode is set-up, proceed to how to use [**Wi-Fi** and **Bluetooth** over this setup](../README.md#3-get-started)

### 2.2 SDIO configuration
**Wi-Fi and Bluetooth over SDIO**

| Supported Targets | ESP32 | ESP32-C6 |
| ----------------- | ----- | -------- |
* Hardware setup
    * In this setup, ESP board acts as a SDIO peripheral and provides Wi-Fi capabilities to host. Please connect ESP board to Raspberry-Pi with jumper cables as mentioned below. Raspberry Pi should be powered with correct incoming power rating. ESP can be powered through PC using micro-USB/USB-C cable.
    * **Pin connections**

        | Raspberry-Pi Pin | ESP32 Pin | ESP32-C6 Pin | Function |
        |:-------:|:---------:|:--------:|:--------:|
        | 13 | IO13+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)| IO23+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | DAT3 |
        | 15 | IO14 | IO19 | CLK |
        | 16 | IO15+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) | IO18+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | CMD |
        | 18 | IO2+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)| IO20+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | DAT0 |
        | 22 | IO4+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)| IO21+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | DAT1 |
        | 31 | EN  | ESP Reset |
        | 37 | IO12+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)| IO22+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | DAT2 |
        | 39 | GND | GND | GND|
    * Raspberry-Pi pinout can be found [here!](https://pinout.xyz/pinout/sdio)
    * As SDIO faces signal integrity issues over jumper wires, we strongly recommend to **Design PCB boards with above connections**. If that is not possible. Use good quality extremely small (smaller than 5cm) jumper wires, all equal length. Join all possible grounds interconnected to lower noise. Add at least, 10k Ohm external pull-up resistors on 5 lines: CMD, DAT0-4. We use 51k Ohm resistors in our set-up.

* Raspberry-Pi Setup
    * By default, the SDIO pins of Raspberry-pi are not configured and are internally used for built-in Wi
-Fi interface. Please enable SDIO pins by appending following line to _/boot/config.txt_ file
        ```
        dtoverlay=sdio,poll_once=off
        dtoverlay=disable-bt
        ```
    * Please reboot Raspberry-Pi after changing this file.
* Setting up the environment and getting started
    * Host environment setup  
        * As ESP32 & ESP32C6, both support SDIO, Let host know which slave chipset is being used by changing `ESP_SLAVE_CHIPSET` in `esp_hosted_ng/host/rpi_init.sh` as:
            ```sh
            ESP_SLAVE_CHIPSET="esp32"
            ```
            or
            ```sh
            ESP_SLAVE_CHIPSET="esp32c6"
            ```
        * Execute following commands in root directory of cloned ESP-Hosted repository on Raspberry-Pi
            ```sh
            $ cd esp_hosted/esp_hosted_ng/host/
            $ bash rpi_init.sh sdio
            ```
        * This script compiles and loads host driver on Raspberry-Pi. It also creates virtual serial interface `/dev/espsta0` which is used as a control interface for Wi-Fi on ESP peripheral

    * For esp firmware if you are using [ESP Quick start guide](#12-esp-quick-start-guide)
        * Please flash the required binaries using with command mentioned in `flashing_cmd.txt` within desired transport configuration folder as explained in [ESP Quick start guide](#12-esp-quick-start-guide).  
        * Use minicom or any similar terminal emulator with baud rate 115200 to fetch esp side logs on UART
            ```sh
            $ minicom -D <serial_port>
            ```
        * serial_port is device where ESP chipset is detected. For example, /dev/ttyUSB0
    * For esp firmware if you are using [ESP Comprehensive guide](#13-esp-comprehensive-guide).
        * Set up the esp-idf environment as mentioned in [ESP Comprehensive guide](#13-esp-comprehensive-guide)
        * To configure slave chip set environment
            ```
            $ cd esp_hosted/esp_hosted_ng/esp/esp_driver/network_adapter
            $ rm -rf sdkconfig build
            $ idf.py set-target <esp_chipset>
            ```
        * For SDIO, <esp_chipset> could is `esp32`, `esp32c6`.
        * Execute following command to configure the project
            ```
            $ idf.py menuconfig
            ```
        * This will open project configuration window. To select SDIO transport interface, navigate to `Example Configuration ->  Transport layer -> SDIO interface -> select` and exit from menuconfig.
        * Use below command to compile and flash the project. Replace <serial_port> with ESP peripheral's serial port.
            ```
            $ idf.py -p <serial_port> build flash
            ```
        * Collect the firmware log using
            ```
            $ idf.py -p <serial_port> monitor
            ```
* Checking the Setup
    * On successful flashing and host driver installation, you should see following entry in ESP log:
        ```
        [   77.877892] Features supported are:
        [   77.877901]   * WLAN
        [   77.877906]   * BT/BLE
        [   77.877911]     - HCI over SDIO
        [   77.877916]     - BT/BLE dual mode
        ```
* Once the one of ESP-Hosted-NG mode is set-up, proceed to how to use [**Wi-Fi** and **Bluetooth** over this setup](../README.md#3-get-started)

### 2.3 SDIO/SPI + Uart configuration
**Wi-Fi over SDIO/SPI and Bluetooth over UART**

| Supported Chipsets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S3 | ESP32-C6 |
| ------------------ | ----- | -------- | -------- | -------- | -------- |
| 4 line UART supported | yes | no | yes | yes | no |
| 2 line UART supported | yes | yes | yes | yes | yes |
* In this section, ESP chipset provides a way to run Bluetooth/BLE over UART interface.

* This section is divided in two parts 
* Wi-fi over SPI and Bluetooth over UART
    * Supported targets **ESP32, ESP32S3, ESP32C3, ESP32C2, ESP32C6**
    * Please follows these steps first to setup esp device in SPI mode [SPI configuration](#21-spi-configuration)
* Wi-fi over SDIO and Bluetooth over UART
    * Supported targets **ESP32, ESP32C6**
    * Please follows these steps first to setup esp device in SDIO mode [SDIO configuration](#22-sdio-configuration)
        
* Hardware Uart setup
    * **Pin Connections**  
    * Four line setup
        | Raspberry-Pi Pin Function | Raspberry-Pi Pin | ESP32 | ESP32-S3 | ESP32-C3 | ESP32 Pin Function |
        |:-------:|:--------:|:---------:|:--------:|:--------:|:--------:| 
        | RX | 10 | IO5 | IO17 | IO5 | TX |
        | TX | 8 | IO18 | IO18 | IO18 | RX |
        | CTS | 36 | IO19 | IO19 | IO19 | RTS |
        | RTS | 11 | IO23 | IO20 | IO8 | CTS |

    * Two line setup
        | Raspberry-Pi Pin Function | Raspberry-Pi Pin | ESP32-C2 | ESP32-C6 | ESP Function |
        |:-------:|:--------:|:---------:|:---------:|:--------:|
        | RX | 10 | IO5 | IO5 | TX |
        | TX | 8 | IO18 | IO12 | RX |

    * Raspberry-Pi pinout can be found [here!](https://pinout.xyz/pinout/uart)
    * In case you wish to reduce number of hardware lines, you may consider SPI_only or SDIO_only transports, where Wi-Fi and Bluetooth traffic is multiplexed on same bus and no need of extra UART pins. UART pin numbers are configurable. If you want to switch from 4 line UART mode to 2 lines, hardware flow control need to be turned off.
    * Use good quality small (smaller than 10cm) jumper wires, all equal length

* Raspberry-Pi Setup
    * By default, the UART pins on Raspberry-Pi are in disabled state. In order to enable UART and setup it for bluetooth connection, follow below steps.
        1) Enable UART pins and disable in built bluetooth on Raspberry-Pi by appending following lines to _/boot/config.txt_ file
            ```
            enable_uart=1
            dtoverlay=disable-bt
            ```
        2) Remove following from _/boot/cmdline.txt_. Leave everything else untouched.
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
        3) Disable hciuart on Raspberry-Pi
            ```
            $ sudo systemctl disable hciuart
            ```
    * Please reboot Raspberry-Pi after changing this file.
* Setting up the environment and getting started
    * Host environment setup  
        * Execute following commands in root directory of cloned ESP-Hosted repository on Raspberry-Pi
            ```sh
            $ cd esp_hosted/esp_hosted_ng/host/
            $ bash rpi_init.sh <transport> <bt_over_uart>
            ```
        - `transport` can take value `sdio` or `spi`. Defaults to `sdio`
        - `bt_over_uart` can take value `btuart` or `btuart_2pins`.
	    - :warning: Note:
	    - For ESP32-C2, 2 pin UART (TX & RX only) is supported. So `btuart_2pins` should be used for ESP32-C2.
		- For other chipsets, `btuart` should be used, where 4 pin UART (TX, RX, CTS, RTS) is used.
    * For esp firmware if you are using [ESP Quick start guide](#12-esp-quick-start-guide)
        * Please flash the required binaries using with command mentioned in `flashing_cmd.txt` within desired transport configuration folder as explained in [ESP Quick start guide](#12-esp-quick-start-guide).  
        * Use minicom or any similar terminal emulator with baud rate 115200 to fetch esp side logs on UART
            ```sh
            $ minicom -D <serial_port>
            ```
        * serial_port is device where ESP chipset is detected. For example, /dev/ttyUSB0
    * For esp firmware if you are using [ESP Comprehensive guide](#13-esp-comprehensive-guide).
        * Set up the esp-idf environment as mentioned in [ESP Comprehensive guide](#13-esp-comprehensive-guide)
        * To configure slave chip set environment
            ```
            $ cd esp_hosted/esp_hosted_ng/esp/esp_driver/network_adapter
            $ rm -rf sdkconfig build
            $ idf.py set-target <esp_chipset>
            ```
        * For SDIO, <esp_chipset> could is `esp32`, for SPI it will could be `esp32, esp32s3, esp32c3`.
        * Execute following command to configure the project
            ```
            $ idf.py menuconfig
            ```
        * Set ESP-Hosted transport
            * ESP32
                * Select transport either SDIO or SPI
                * Navigate to `Example Configuration -> Transport layer` select `SDIO interface` or `SPI interface`, whichever expected
            * ESP32-C3/S3
                * SPI is automatically selected. Nothing to be done, skip to next step

        * Set Bluetooth over UART
            * ESP32 / ESP32-C3/S3
                * navigate to `Component config -> Bluetooth -> Bluetooth controller -> HCI mode`, select `UART(H4)`

        * Set UART baud rate
            * Default HCI over baud rate is `921600`. In case need to change,
            * ESP32
                * Navigate and change using `Component config -> Bluetooth -> Bluetooth controller -> HCI UART(H4) Options ->     UART Baudrate for HCI`
            * ESP32-C3/S3
                * Navigate and change using `Component config -> Example Configuration -> UART Baudrate for HCI`

        * Additional settings
            * ESP32-C3
                * Select chip revision in addition. Navigate to `Component config → ESP32C3-Specific → Minimum Supported ESP32-C3 Revision` and select chip version of ESP32-C3.

        * Build and flash the project. Replace <serial_port> with ESP peripheral's serial port.
            ```sh
            $ idf.py -p <serial_port> build flash
            ```

* Post Setup
    * After setting up host and loading ESP firmware, execute below command to create `hci0` interface
        ```sh
        $ sudo hciattach -s <baud_rate> /dev/serial0 any <baud_rate> flow
        ```
    * <baud_rate> should match UART baud rate while flashing ESP peripheral (Default: 921600)

    * For ESP32
        * Check `CONFIG_BT_HCI_UART_BAUDRATE` parameter in *esp_hosted_ng/esp/esp_driver/network_adapter/sdkconfig*
        * Alternatively baud rate could be located using `idf.py menuconfig` at, `Component config ->  Bluetooth -> Bluetooth controller ->  HCI UART(H4) Options -> UART Baudrate for HCI`

    * For ESP32-C3/S3
        * Check `CONFIG_EXAMPLE_HCI_UART_BAUDRATE` parameter in *esp_hosted_ng/esp/esp_driver/network_adapter/sdkconfig*
        * Alternatively baud rate could be located using `idf.py menuconfig` at, `Component config -> Example Configuration -> UART Baudrate for HCI`

    * Check if UART is setup correctly
        ```
        $ hciconfig
        ```
        Should list the hci interface with bus as 'UART' and interface should be 'UP'


* Points to note
    * **Restart HCI on every reloading of kernel module**
    * Remove and reattach `HCI over UART`
        ```sh
        $ hciconfig -a
        $ sudo killall hciattach
        $ sudo hciattach -s <baud_rate> /dev/serial0 any <baud_rate> flow
        $ hciconfig -a
        ```
    * <baud_rate> should match UART baud rate while flashing ESP peripheral
* Once the one of ESP-Hosted-NG mode is set-up, proceed to how to use [**Wi-Fi** and **Bluetooth** over this setup](../README.md#3-get-started)

### 3. Troubleshoot Setup Problems

After following above steps, the host software should be loaded fine. Bootup event from ESP should already would have sent and Host should have received it.

If Bootup event is not recieved in host `dmesg` as sample log above, please try following:

- Please check your connections to be firm
- Try with smaller wires
- Check correct Pull-Ups are in place
- If using different host than Raspberry-Pi, please check [porting guide](porting_guide.md)
- Check [Troubleshoot Guide](./Troubleshoot.md) for further details

### 4. Points to note
- This solution is tested BR/EDR(Classic BT) 4.2 and BLE 4.2 functionalities with BlueZ 5.43+.
- We suggest to use the latest stable BlueZ release.
<!--- TODO Whereas BLE 5.0 functionalities are tested with bluez 5.45+ -->

