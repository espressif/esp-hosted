| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-H2 | ESP32-S3 | ESP32-P4 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# BLE Host-Only Peripheral VHCI Example

This example uses the Bluetooth VHCI transport provided by ESP-Hosted.

To use the VHCI transport in the application, the Bluetooth controller
should be disabled and the default uart-transport should also be
disabled (when the controller is disabled, by default the
uart-transport is selected). The Bluetooth VHCI transport in
ESP-Hosted should also be enabled.

Refer to the `sdkconfig.defaults` file which has the required
configuration.

To test this demo, any BLE scanner app can be used.

## How to Use This Example

Before project configuration and build, be sure to set the correct
chip target using:

```bash
idf.py set-target <chip_name>
```

### Configure the project

Open the project configuration menu:

```bash
idf.py menuconfig
```

In the `Component config-> Bluetooth` menu:

* Select `controller` to Disabled.
* Disable `Nimble Options-> Host-controller Transport -> Enable Uart Transport`.

In the `Component config-> ESP-Hosted config` menu:

* Enable `Bluetooth Support-> Enable Hosted Bluetooth support`

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the
project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://idf.espressif.com/) for full
steps to configure and use ESP-IDF to build projects.

## Example Output

This is the console output when `bleprph_host_only_vhci` is running on
the ESP32-P4 and using the Bluetooth Controller of the ESP32C6 on the
ESP32-P4-Function-EV-Board. The data is transferred through SDIO
between the ESP32-P4 and ESP32-C6:

```
I (25) boot: ESP-IDF v5.4-dev-2793-g24047f9a04-dirty 2nd stage bootloader
I (26) boot: compile time Sep  9 2024 16:23:48
I (27) boot: Multicore bootloader
I (32) boot: chip revision: v0.1
I (34) boot: efuse block revision: v0.0
I (39) boot.esp32p4: SPI Speed      : 80MHz
I (44) boot.esp32p4: SPI Mode       : DIO
I (49) boot.esp32p4: SPI Flash Size : 2MB
I (53) boot: Enabling RNG early entropy source...
I (59) boot: Partition Table:
I (62) boot: ## Label            Usage          Type ST Offset   Length
I (70) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (77) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (84) boot:  2 factory          factory app      00 00 00010000 00100000
I (93) boot: End of partition table
I (96) esp_image: segment 0: paddr=00010020 vaddr=40070020 size=2ae08h (175624) map
I (135) esp_image: segment 1: paddr=0003ae30 vaddr=30100000 size=0000ch (    12) load
I (137) esp_image: segment 2: paddr=0003ae44 vaddr=3010000c size=00038h (    56) load
I (142) esp_image: segment 3: paddr=0003ae84 vaddr=4ff00000 size=05194h ( 20884) load
I (154) esp_image: segment 4: paddr=00040020 vaddr=40000020 size=638b0h (407728) map
I (226) esp_image: segment 5: paddr=000a38d8 vaddr=4ff05194 size=0b9a0h ( 47520) load
I (237) esp_image: segment 6: paddr=000af280 vaddr=4ff10b80 size=02328h (  9000) load
I (244) boot: Loaded app from partition at offset 0x10000
I (244) boot: Disabling RNG early entropy source...
I (258) cpu_start: Multicore app
W (267) clk: esp_perip_clk_init() has not been implemented yet
I (274) cpu_start: Pro cpu start user code
I (274) cpu_start: cpu freq: 360000000 Hz
I (274) app_init: Application information:
I (277) app_init: Project name:     bleprph_host_only_vhci
I (283) app_init: App version:      1c568c9-dirty
I (288) app_init: Compile time:     Sep  9 2024 16:23:43
I (294) app_init: ELF file SHA256:  ee5a16527...
I (299) app_init: ESP-IDF:          v5.4-dev-2793-g24047f9a04-dirty
I (306) efuse_init: Min chip rev:     v0.1
I (311) efuse_init: Max chip rev:     v0.99
I (316) efuse_init: Chip rev:         v0.1
I (321) heap_init: Initializing. RAM available for dynamic allocation:
I (328) heap_init: At 4FF160B0 len 00024F10 (147 KiB): RAM
I (334) heap_init: At 4FF3AFC0 len 00004BF0 (18 KiB): RAM
I (340) heap_init: At 4FF40000 len 00060000 (384 KiB): RAM
I (347) heap_init: At 50108080 len 00007F80 (31 KiB): RTCRAM
I (353) heap_init: At 30100044 len 00001FBC (7 KiB): TCM
I (360) spi_flash: detected chip: generic
I (363) spi_flash: flash io: dio
W (367) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (381) host_init: ESP Hosted : Host chip_ip[18]
I (409) H_API: ESP-Hosted starting. Hosted_Tasks: prio:23, stack: 5120 RPC_task_stack: 5120
sdio_mempool_create free:181028 min-free:181028 lfb-def:139264 lfb-8bit:139264

I (414) gpio: GPIO[18]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (423) gpio: GPIO[19]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (433) gpio: GPIO[14]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (442) gpio: GPIO[15]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (451) gpio: GPIO[16]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (461) gpio: GPIO[17]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (470) H_API: ** add_esp_wifi_remote_channels **
I (475) transport: Add ESP-Hosted channel IF[1]: S[0] Tx[0x4000d110] Rx[0x4001b4aa]
0x4000d110: transport_drv_sta_tx at /home/kysoh/projects/esp_as_mcu_host/examples/bleprph_host_only_vhci/components/esp_hosted/host/drivers/transport/transport_drv.c:208
0x4001b4aa: esp_wifi_remote_channel_rx at /home/kysoh/projects/esp_as_mcu_host/examples/bleprph_host_only_vhci/managed_components/espressif__esp_wifi_remote/esp_wifi_remote_net.c:19

I (484) transport: Add ESP-Hosted channel IF[2]: S[0] Tx[0x4000d058] Rx[0x4001b4aa]
0x4000d058: transport_drv_ap_tx at /home/kysoh/projects/esp_as_mcu_host/examples/bleprph_host_only_vhci/components/esp_hosted/host/drivers/transport/transport_drv.c:238
0x4001b4aa: esp_wifi_remote_channel_rx at /home/kysoh/projects/esp_as_mcu_host/examples/bleprph_host_only_vhci/managed_components/espressif__esp_wifi_remote/esp_wifi_remote_net.c:19

I (493) main_task: Started on CPU0
I (503) main_task: Calling app_main()
I (513) transport: Attempt connection with slave: retry[0]
I (513) transport: Reset slave using GPIO[54]
I (513) os_wrapper_esp: GPIO [54] configured
I (513) gpio: GPIO[54]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (1693) sdio_wrapper: SDIO master: Data-Lines: 4-bit Freq(KHz)[40000 KHz]
I (1693) sdio_wrapper: GPIOs: CLK[18] CMD[19] D0[14] D1[15] D2[16] D3[17] Slave_Reset[54]
I (1693) H_SDIO_DRV: Starting SDIO process rx task
I (1693) sdio_wrapper: Queues: Tx[20] Rx[20] SDIO-Rx-Mode[1]
I (1733) gpio: GPIO[15]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (1733) gpio: GPIO[17]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
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
I (1783) sdio_wrapper: Function 0 Blocksize: 512
I (1793) sdio_wrapper: Function 1 Blocksize: 512
I (1793) H_SDIO_DRV: SDIO Host operating in STREAMING MODE
I (1803) H_SDIO_DRV: generate slave intr
I (1813) transport: Received INIT event from ESP32 peripheral
I (1813) transport: EVENT: 12
I (1813) transport: EVENT: 11
I (1823) transport: capabilities: 0xd
I (1823) transport: Features supported are:
I (1833) transport:      * WLAN
I (1833) transport:        - HCI over SDIO
I (1843) transport:        - BLE only
I (1843) transport: EVENT: 13
I (1843) transport: ESP board type is : 13

I (1853) transport: Base transport is set-up

I (1853) transport: Slave chip Id[12]
I (1863) vhci_drv: Host BT Support: Enabled
I (1863) vhci_drv:      BT Transport Type: VHCI
I (1873) H_SDIO_DRV: Received INIT event
I (1883) rpc_wrap: Received Slave ESP Init
I (2623) NimBLE_BLE_PRPH: BLE Host Task Started
I (2623) uart: queue free spaces: 8
I (2623) main_task: Returned from app_main()
I (2623) NimBLE: GAP procedure initiated: stop advertising.

I (2633) NimBLE: Device Address:
I (2633) NimBLE: 40:4c:ca:5b:9a:e2
I (2633) NimBLE:

I (2643) NimBLE: GAP procedure initiated: advertise;
I (2643) NimBLE: disc_mode=2
I (2653) NimBLE:  adv_channel_map=0 own_addr_type=0 adv_filter_policy=0 adv_itvl_min=0 adv_itvl_max=0
I (2663) NimBLE:
```

## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-hosted/issues) on ESP-Hosted on GitHub. We will get back to you soon.

## References

* Bluetooth Implementation in ESP-Hosted: https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/bluetooth_implementation.md
