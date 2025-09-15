# Wi-Fi and BT/BLE connectivity Setup over SDIO

| Supported Targets | ESP32 | ESP32-C6 | ESP32-C5 |
| ----------------- | ----- | -------- | -------- |

> [!NOTE]
> SDIO support for ESP32-C5 is currently in BETA. It is available as a `--preview` release in the `master` branch of ESP-IDF.

## 1. Setup
### 1.1 Hardware Setup
In this setup, ESP board acts as a SDIO peripheral and provides Wi-Fi capabilities to host. Please connect ESP board to Raspberry-Pi as mentioned below.
Raspberry Pi should be powered with correct incoming power rating.
ESP can be powered through PC using micro-USB/USB-C cable.

| Raspberry-Pi Pin | ESP32 Pin | ESP32-C6 Pin | ESP32-C5 | Function |
| :--------------: | :-------: | :----------: | :------: | :------: |
| 13 | IO13+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) | IO23+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | IO13+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) | DAT3 |
| 15 | IO14 | IO19 | IO9 | CLK |
| 16 | IO15+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) | IO18+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | IO10+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) | CMD |
| 18 | IO2+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) | IO20+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | IO8+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | DAT0 |
| 22 | IO4+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) | IO21+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | IO7+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | DAT1 |
| 31 | EN  | ESP Reset | ESP Reset | Reset |
| 37 | IO12+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)| IO22+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | IO14+[pull-up](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/sd_pullup_requirements.html) | DAT2 |
| 39 | GND | GND | GND | GND |

Raspberry-Pi pinout can be found [here!](https://pinout.xyz/pinout/sdio)

Sample setup image of ESP32-C6 SDIO with RPi looks like:

![alt text](rpi_esp32c6_sdio_setup.png "setup of Raspberry-Pi as host and ESP32-C6 as peripheral")

> [!WARNING]
> As SDIO faces signal integrity issues over jumper wires, we strongly recommend to **Design PCB boards with above connections.**
>
> If that is not possible
> - Use good quality extremely small (smaller than 5cm) jumper wires, all equal length
> - Join all possible grounds interconnected to lower noise
> - Add at least, 10k Ohm external pull-up resistors on 5 lines: CMD, DAT0-4. We use 51k Ohm resistors in our set-up.

### 1.2 Raspberry-Pi Software Setup
By default, the SDIO pins of Raspberry-pi are not configured and are internally used for the built-in Wi-Fi interface. Please enable SDIO pins by appending the following line to the _/boot/firmware/config.txt_ file (prior to _Bookworm_, the file is at _/boot/config.txt_):
```
dtoverlay=sdio,poll_once=off
dtoverlay=disable-bt
```
Please reboot Raspberry-Pi after changing this file.

Note that the default Wi-Fi on your Raspberry Pi will be disabled on reboot. This is because the SDIO used in ESP-Hosted is going to use this SDIO port here on. If you are not using desktop variant of Raspberry Pi, It would be good to set up static IP SSH with ethernet cable.

Also, it is recommended to download (any) software needed (like iperf etc) before rebooting, so that losing Wi-Fi is not an issue.

## 2. Load ESP-Hosted Solution
### 2.1 Host Software
* Execute following commands in root directory of cloned ESP-Hosted repository on Raspberry-Pi:

```sh
$ cd esp_hosted_fg/host/linux/host_control/
$ ./rpi_init.sh wifi=sdio bt=sdio
```

* This script compiles and loads host driver on Raspberry-Pi. It also creates virtual serial interface `/dev/esps0` which is used as a control interface for Wi-Fi on ESP peripheral

Execute `./rpi_init.sh --help` to see the list of options.

> [!NOTE]
> For SDIO+UART, use `bt=uart_2pins` or `bt=uart_4pins` for 2/4 pin UART. For wifi only support, exclude the `bt` parameter.

#### 2.1.1 Manually loading and unloading the Kernel Module

Once built, the kernel module `esp32_sdio.ko` can be found in `esp_hosted_fg/host/linux/host_driver/esp32`. You can manualy load/unload the module as needed.

To add the module:

`$ sudo insmod esp_hosted_fg/host/linux/host_driver/esp32/esp32_sdio.ko resetpin=518 clockspeed=50`

##### Module Parameters

| Parameter | Meaning |
| --- | --- |
| `resetpin` | GPIO to reset the ESP peripheral |
| `clockspeed` | SDIO CLK frequency (in MHz: maximum 50) |

Note: `clockspeed` is optional. Default is to use the default SDIO clock speed.

To remove the module:

`$ sudo rmmod esp32_sdio`

### 2.2 ESP Peripheral Firmware
One can load pre-built release binaries on ESP peripheral or compile those from source. Below subsection explains both these methods.

#### 2.2.1 Load Pre-built Release Binaries
* Download pre-built firmware binaries from [releases](https://github.com/espressif/esp-hosted/releases)
* Follow `readme.txt` from release tarball to flash the ESP binary
* :warning: Make sure that you use `Source code (zip)` in `Assets` fold with associated release for host building.
* Windows user can use ESP Flash Programming Tool to flash the pre-built binary.
* Collect firmware log
    * Use minicom or any similar terminal emulator with baud rate 115200 to fetch esp side logs on UART
```sh
$ minicom -D <serial_port>
```
serial_port is device where ESP chipset is detected. For example, /dev/ttyUSB0


#### 2.2.2 Source Compilation

Make sure that same code base (same git commit) is checked-out/copied at both, ESP and Host

##### Set-up ESP-IDF
- **Note on Windows 11**: follow [these instructions](/esp_hosted_fg/esp/esp_driver/setup_windows11.md) to setup ESP-IDF and build the esp firmware.
- You can install the ESP-IDF using the `setup-idf.sh` script (run `./setup-idf.sh -h` for supported options):
```sh
$ cd esp_hosted_fg/esp/esp_driver
$ ./setup-idf.sh
```
- Once ESP-IDF has been installed, set-up the build environment using
```sh
$ . ./esp-idf/export.sh
```
- If you are using the ESP-WROVER-KIT and plan to use the internal pull-up resistors to quickly evaluate the driver, make sure to set the `SDIO_SLAVE_FLAG_INTERNAL_PULLUP` flag in the `sdio_slave_config_t config` of the `sdio_init` function. (However, the internal pullups are not sufficient and not reliable, please make sure external pullups are connected to the bus in your real design. For more information, check [SDIO pull-up requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html).)

To remove the ESP-IDF installed by `setup-idf.sh`, you can run the `./remove-idf.sh` script.

##### Configure, Build & Flash SDIO ESP firmware
* Set slave chipset environment

```sh
$ cd network_adapter
$ rm -rf sdkconfig build
$ idf.py set-target <esp_chipset>
```
For SDIO, <esp_chipset> could be `esp32` or `esp32c6`
* Execute following command to configure the project
```sh
$ idf.py menuconfig
```
* This will open project configuration window. To select SDIO transport interface, navigate to `Example Configuration ->  Transport layer -> SDIO interface -> select` and exit from menuconfig.
* Use below command to compile and flash the project. Replace <serial_port> with ESP peripheral's serial port.
```sh
$ idf.py -p <serial_port> build flash
```
* Collect the firmware log using
```sh
$ idf.py -p <serial_port> monitor
```


## 3. Checking the Setup for SDIO
- Firmware log
On successful flashing, you should see following entry in ESP log:

```
[   77.877892] Features supported are:
[   77.877901]   * WLAN
[   77.877906]   * BT/BLE
[   77.877911]     - HCI over SDIO
[   77.877916]     - BT/BLE dual mode
```
- Host log
Once ESP peripheral has a valid firmware and booted successfully, you should be able to see successful enumeration on Raspberry Pi side as:
```sh
$ dmesg
[   14.309956] Bluetooth: Core ver 2.22
[   14.310019] NET: Registered protocol family 31
[   14.310022] Bluetooth: HCI device and connection manager initialized
[   14.310038] Bluetooth: HCI socket layer initialized
[   14.310045] Bluetooth: L2CAP socket layer initialized
[   14.310069] Bluetooth: SCO socket layer initialized
[   14.327776] Bluetooth: HCI UART driver ver 2.3
[   14.327784] Bluetooth: HCI UART protocol H4 registered
[   14.328014] Bluetooth: HCI UART protocol Three-wire (H5) registered
[   14.328138] Bluetooth: HCI UART protocol Broadcom registered
[   14.650789] Bluetooth: BNEP (Ethernet Emulation) ver 1.3
[   14.650796] Bluetooth: BNEP filters: protocol multicast
[   14.650810] Bluetooth: BNEP socket layer initialized
[   14.715079] Bluetooth: RFCOMM TTY layer initialized
[   14.715109] Bluetooth: RFCOMM socket layer initialized
[   14.715137] Bluetooth: RFCOMM ver 1.11
[   20.556381] mmc1: card 0001 removed
[   69.245969] mmc1: queuing unknown CIS tuple 0x01 (3 bytes)
[   69.253368] mmc1: queuing unknown CIS tuple 0x1a (5 bytes)
[   69.256622] mmc1: queuing unknown CIS tuple 0x1b (8 bytes)
[   69.258842] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[   69.258939] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[   69.259035] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[   69.260840] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[   69.260939] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[   69.261040] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[   69.261236] mmc1: new SDIO card at address 0001
[   76.892073] esp32: loading out-of-tree module taints kernel.
[   76.893083] esp32: Resetpin of Host is 6
[   76.893202] esp32: Triggering ESP reset.
[   76.894498] esp_sdio: probe of mmc1:0001:1 failed with error -110
[   76.894566] esp_sdio: probe of mmc1:0001:2 failed with error -110
[   77.596173] mmc1: card 0001 removed
[   77.649578] mmc1: queuing unknown CIS tuple 0x01 (3 bytes)
[   77.657019] mmc1: queuing unknown CIS tuple 0x1a (5 bytes)
[   77.660243] mmc1: queuing unknown CIS tuple 0x1b (8 bytes)
[   77.662448] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[   77.662545] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[   77.662643] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[   77.664445] mmc1: queuing unknown CIS tuple 0x80 (1 bytes)
[   77.664541] mmc1: queuing unknown CIS tuple 0x81 (1 bytes)
[   77.664637] mmc1: queuing unknown CIS tuple 0x82 (1 bytes)
[   77.664832] mmc1: new SDIO card at address 0001
[   77.665287] esp_probe: ESP network device detected
[   77.877892] Features supported are:
[   77.877901] 	 * WLAN
[   77.877906] 	 * BT/BLE
[   77.877911] 	   - HCI over SDIO
[   77.877916] 	   - BT/BLE dual mode
[   78.096179] esp_sdio: probe of mmc1:0001:2 failed with error -22
```

**If intended transport is SDIO+UART, please continue ahead with UART setup**
