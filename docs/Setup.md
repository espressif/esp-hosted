# Setup
Currently we support ESP32 WROVER-Kit with Raspberry-Pi (3 Model B+, 4 Model B) for evaluation with Raspbian operating system.

## Raspberry-Pi Software Setup

### Kernel setup:
We recommend full version Raspbian install on Raspberry-Pi to ensure easy driver compilation. Please make sure to use kernel version `v4.19` and above. Prior kernel versions may work, but are not tested. Kernel headers are required for driver compilation. Please install them as:
```sh
$ sudo apt update
$ sudo apt-get install raspberrypi-kernel-headers
```
Verify that kernel headers are installed properly by running following command. Failure of this command indicates that kernel headers are not installed. In such case, upgrade/downgrade kernel and reinstall kernel headers.
```sh
$ ls /lib/modules/$(uname -r)/build
```

### Python setup:
Python is required to run utility scripts. Please install it as:
```sh
$ sudo apt-get install python
```
To start with control path on Raspberry-Pi, `protobuf` and `utils` python modules are needed. User can install these modules by running following commands.
```
pip install utils
pip install protobuf
```

### Source code repository:
* Clone esp-hosted repository on Raspberry Pi.

* To test BT/BLE functionality on Raspberry-Pi `bluez`(official Linux Bluetooth protocol stack) is needed. check if Raspberry-Pi has installed `bluez` or not.
[link that can help to install bluez](https://scribles.net/updating-bluez-on-raspberry-pi-from-5-43-to-5-50/). We have tested BT/BLE solution at bluez 5.43 and 5.45.

### Utilities:
* Raspi-gpio utility is required to configure GPIO pins. Please install it as:
```sh
$ sudo apt-get install raspi-gpio
```
* make sure Raspberry-Pi should have `bluetoothctl`, `bluetoothd`, `hcitool`, `hciconfig` utilities.

## Wi-Fi and BT/BLE connectivity Setup over SDIO
### Hardware Setup/Connections
In this setup, ESP32 board acts as a SDIO peripheral and provides Wi-FI capabilities to host. Please connect ESP32 board to Raspberry-Pi with jumper cables as mentioned below. It may be good to use small length cables to ensure signal integrity.

| Raspberry-Pi Pin | ESP32 Pin | Function |
|:-------:|:---------:|:--------:|
| 13 | IO13 | DAT3 |
| 15 | IO14 | CLK |
| 16 | IO15 | CMD |
| 18 | IO2 | DAT0 |
| 22 | IO4 | DAT1 |
| 37 | IO12 | DAT2 |
| 39 | GND | GND |

Raspberry-Pi pinout can be found [here!](https://pinout.xyz/pinout/sdio)

Setup image is here.

![alt text](rpi_esp_setup.jpeg "setup of Raspberry-Pi as host and ESP32 as slave")

Power ESP32 and Raspberry Pi separately with a power supply that provide sufficient power. ESP32 can be powered through PC using micro-USB cable.

### Software setup
By default, the SDIO pins of Raspberry-pi are not configured and are internally used for built-in Wi-Fi interface. Please enable SDIO pins by appending following line to _/boot/config.txt_ file
```
dtoverlay=sdio,poll_once=off
dtoverlay=disable-bt
```
Please reboot Raspberry-Pi after changing this file.

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

## ESP32 Setup
The control path between Raspberry-Pi and ESP32 is based on `protobuf`. For that `protocomm` layer from ESP-IDF is used. Make sure ESP-IDF on branch `release/v4.0`. Run following command on ESP32 to make `protocomm_priv.h` available for control path.
```
git mv components/protocomm/src/common/protocomm_priv.h components/protocomm/include/common/
```

On ESP32 either use pre-provided hosted mode firmware binary or if you have source, compile the app against ESP-IDF 4.0 release by running command as `make` in `esp/esp_driver/network_adapter` directory. Program the WROVER-KIT using standard flash programming procedure with
```sh
$ make flash
```

## Checking the Setup
Once ESP32 has a valid firmware and booted successfully, you should be able to see successful enumeration on Raspberry Pi side as:
```sh
$ dmesg
[   45.926409] Bluetooth: Core ver 2.22
[   45.926507] NET: Registered protocol family 31
[   45.926513] Bluetooth: HCI device and connection manager initialized
[   45.926542] Bluetooth: HCI socket layer initialized
[   45.926556] Bluetooth: L2CAP socket layer initialized
[   45.926596] Bluetooth: SCO socket layer initialized
[   50.263509] esp32: loading out-of-tree module taints kernel.
[   50.266361] esp_probe: ESP network device detected
[   50.485777] Features supported are:
[   50.485783] 	 * WLAN
[   50.485785] 	 * BT/BLE
[   50.485787] 	   - HCI over SDIO
[   50.485789] 	   - BT/BLE dual mode
[   50.702391] esp_sdio: probe of mmc1:0001:2 failed with error -22
[   50.723218] Bluetooth: BNEP (Ethernet Emulation) ver 1.3
[   50.723232] Bluetooth: BNEP filters: protocol multicast
[   50.723256] Bluetooth: BNEP socket layer initialized
```
