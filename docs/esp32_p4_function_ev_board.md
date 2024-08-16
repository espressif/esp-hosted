# ESP-Hosted on the ESP32-P4-Function-EV-Board DevKit

**Table of Contents**

- [1. Introduction](#1-introduction)
- [2. Building ESP-Hosted as Host for the P4](#2-building-esp-hosted-as-host-for-the-p4)
  - [2.1. Adding Wifi Remote and Hosted Components](#21-adding-wifi-remote-and-hosted-components)
  - [2.2. Building the Firmware](#22-building-the-firmware)
- [3. Checking that ESP-Hosted is Running](#3-checking-that-esp-hosted-is-running)
- [4. Flashing the On-board ESP32-C6 using ESP-Prog](#4-flashing-the-on-board-esp32-c6-using-esp-prog)
- [5. Troubleshooting](#5-troubleshooting)
- [6. Flashing the On-board ESP32-P4 through the Serial Interface](#6-flashing-the-on-board-esp32-p4-through-the-serial-interface)
- [7. References](#7-references)

## 1. Introduction

This page documents using ESP-Hosted on the
ESP32-P4-Function-EV-Board. The board comes with an on-board ESP32-C6
module, pre-flashed with ESP-Hosted slave code (v0.0.6). The board
provides a Wi-Fi connection to the on-board ESP32-P4, which acts as
the host.

The image below shows the board.

<img src="images/esp32-p4-function-ev-board.jpg"
alt="ESP32-P4-Function-EV-Board" width="800" />

*ESP32-P4-Function-EV-Board*

The ESP32-P4 communicates with the ESP32-C6 module using SDIO.

## 2. Building ESP-Hosted as Host for the P4

### 2.1. Adding Wifi Remote and Hosted Components

The Wi-Fi service is provided to the ESP32-P4 using the
`esp_wifi_remote` component. Check your project's `idf_component.yml`
file) to see if it is already present. If not, you can add this
component to your project:

```
idf.py add-dependency "espressif/esp_wifi_remote"
```

The `esp_wifi_remote` component has a dependency on ESP-Hosted and
will also add the `esp_hosted` component to your build.

### 2.2. Building the Firmware

Set the ESP32-P4 as the target, build, flash the firmware and
(optionally) monitor ESP32-P4 console output:

```sh
idf.py set-target esp32p4
idf.py build
idf.py -p <Serial Port> flash monitor`
```

## 3. Checking that ESP-Hosted is Running

When the P4 is running with Hosted, you should see console output similar to this after start-up:

```
I (498) H_API: esp_wifi_remote_init
I (498) transport: Attempt connection with slave: retry[0]
I (498) transport: Reset slave using GPIO[54]
I (498) os_wrapper_esp: GPIO [54] configured
I (508) gpio: GPIO[54]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1678) sdio_wrapper: SDIO master: Data-Lines: 4-bit Freq(KHz)[40000 KHz]
I (1678) sdio_wrapper: GPIOs: CLK[18] CMD[19] D0[14] D1[15] D2[16] D3[17] Slave_Reset[54]
I (1678) H_SDIO_DRV: Starting SDIO process rx task
I (1678) sdio_wrapper: Queues: Tx[20] Rx[20] SDIO-Rx-Mode[3]
I (1718) gpio: GPIO[15]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (1718) gpio: GPIO[17]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
Name:
Type: SDIO
Speed: 40.00 MHz (limit: 40.00 MHz)
Size: 0MB
CSD: ver=1, sector_size=0, capacity=0 read_bl_len=0
SCR: sd_spec=0, bus_width=0
TUPLE: DEVICE, size: 3: D9 01 FF
TUPLE: MANFID, size: 4
  MANF: 0092, CARD: 6666
TUPLE: FUNCID, size: 2: 0C 00
TUPLE: FUNCE, size: 4: 00 00 02 32
TUPLE: CONFIG, size: 5: 01 01 00 02 07
TUPLE: CFTABLE_ENTRY, size: 8
  INDX: C1, Intface: 1, Default: 1, Conf-Entry-Num: 1
  IF: 41
  FS: 30, misc: 0, mem_space: 1, irq: 1, io_space: 0, timing: 0, power: 0
  IR: 30, mask: 1,   IRQ: FF FF
  LEN: FFFF
TUPLE: END
I (1768) sdio_wrapper: Function 0 Blocksize: 512
I (1778) sdio_wrapper: Function 1 Blocksize: 512
I (1778) H_SDIO_DRV: SDIO Host operating in PACKET MODE
I (1788) H_SDIO_DRV: generate slave intr
I (1798) transport: Received INIT event from ESP32 peripheral
I (1798) transport: EVENT: 12
I (1798) transport: EVENT: 11
I (1808) transport: capabilities: 0xd
I (1808) transport: Features supported are:
I (1818) transport:      * WLAN
I (1818) transport:        - HCI over SDIO
I (1818) transport:        - BLE only
I (1828) transport: EVENT: 13
I (1828) transport: ESP board type is : 13

I (1838) transport: Base transport is set-up

I (1838) transport: Slave chip Id[12]
I (1848) hci_stub_drv: Host BT Support: Disabled
I (1848) H_SDIO_DRV: Received INIT event
I (1868) rpc_wrap: Received Slave ESP Init
```

## 4. Flashing the On-board ESP32-C6 using ESP-Prog

> [!NOTE]
> ESP-Prog is only required if you want to flash firmware to the
> ESP32-C6 module using the standard ESP Tools.

The image below shows the board with an ESP-Prog connected to the
header to communicate with the on-board ESP32-C6..

<img src="images/esp32-p4-function-ev-board-esp-prog.jpg"
alt="ESP32-P4-Function-EV-Board with ESP-Prog Connected to ESP32-C6" width="800" />

*ESP32-P4-Function-EV-Board with ESP-Prog Connected to ESP32-C6*

If you need to update the ESP-Hosted slave firmware on the on-board
ESP32-C6 module using ESP-Prog, follow these steps:

1. Check out the ESP-Hosted slave example project:


```
idf.py create-project-from-example "espressif/esp_hosted:slave"
```

2. Set the target and start `Menuconfig`:

```sh
idf.py set-target esp32c6
idf.py menuconfig
```

3. Under **Example Configuration**, ensure that the Hosted transport
   selected is `SDIO`.

4. Build the firmware:

```sh
idf.py build
```

5. Connect the Program Header on the ESP-Prog to the `PROG_C6` header
   on the board. The connections are as follows:

| ESP-Prog | PROG_C6 | Notes          |
| ---      | ---     | ---            |
| ESP\_EN  | EN      |                |
| ESP\_TXD | TXD     |                |
| ESP\_RXD | RXD     |                |
| VDD      | -       | Do not connect |
| GND      | GND     |                |
| ESP\_IO0 | IO0     |                |


6. Flashing the firmware

The on-board ESP32-P4 controls the reset signal for the ESP32-C6. To
prevent the P4 interfering with the C6 while flashing (by asserting
the C6 Reset signal during the firmware download), set the P4 into
Bootloader mode before flashing the firmware to the C6:

    1. hold down the `BOOT` button on the board
    2. press and release the `RST` button on the board
    3. release the `BOOT` button

You can now flash the firmware to the C6 (and monitor the console
output):

```sh
idf.py -p <Serial Port> flash monitor
```

## 5. Troubleshooting

If you encounter issues with using ESP-Hosted, see the following guide:

- [Troubleshooting Guide](troubleshooting.md)

<details>

<summary>Flashing the On-board ESP32-P4 through the Serial Interface</summary>

## 6. Flashing the On-board ESP32-P4 through the Serial Interface

The USB connector on the board is the standard method for flashing the
firmware to the P4. An alternative method is to flash the P4 through
its serial interface using a ESP-Prog.

The image below shows the connection between the ESP-Prog and the
serial port pins on the P4 header for programming.

<img src="images/esp32-p4-esp-prog.jpg"
alt="ESP32-P4 Serial Connection with ESP-Prog" width="600" />

*ESP32-P4 Serial Connection with ESP-Prog*

The connection between the ESP-Prog and the P4 header is as follows:

| ESP-Prog | P4 Header       |
| ---      | ---             |
| ESP\_TXD | U0TXD (GPIO 37) |
| ESP\_RXD | U0RXD (GPIO 38) |
| GND      | GND             |

Leave the other ESP-Prog connected unconnected.

To flash the P4:

1. hold down the `BOOT` button on the board
2. press and release the `RST` button on the board
3. release the `BOOT` button

You can now flash the firmware (and monitor the console output):

```sh
idf.py -p <Serial Port> flash monitor
```

To restart the P4 after flashing, press and release the `RST` button
on the board.

</details>

## 7. References

- ESP32-P4-Function-EV-Board: https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/
- ESP-Prog: https://docs.espressif.com/projects/esp-iot-solution/en/latest/hw-reference/ESP-Prog_guide.html
- `esp_wifi_remote` component: https://components.espressif.com/components/espressif/esp_wifi_remote/
- `esp_hosted` component: https://components.espressif.com/components/espressif/esp_hosted/
