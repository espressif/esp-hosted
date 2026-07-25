# BLE Peripheral — NimBLE Host over Standard HCI UART (`bluetooth/nimble_standard_hci/bleprph_host_only_uart`)

<!-- tags: bluetooth, ble, nimble, standard-hci, uart -->

<!-- common-start -->
NimBLE host-only BLE peripheral (`bleprph` GATT server) that talks HCI over a
**dedicated physical UART** (H4) to an external BT controller. Unlike the
`hosted_hci` examples, HCI here does **not** traverse the ESP-Hosted VHCI bridge:
the **controller** runs standalone on the **co-processor** in native UART-H4
mode, and the **host** drives the link with its own UART transport
(`main/uart_driver.c`). NimBLE's local controller and its built-in UART transport
are both disabled so the application transport takes over.

The host and controller are wired directly UART-to-UART (host TX ↔ controller RX,
host RX ↔ controller TX, common ground) — there is no SDIO / SPI hosted bus in
this scenario. ESP32-S2 has no BT controller and is unsupported.

## Supported Platforms and Transports

### Supported Coprocessors (BT controller in UART-H4 mode)

| Coprocessor | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 | ESP32-S2 |
| :---------- | :---: | :------: | :------: | :------: | :------: | :-------: | :------: | :------: | :------: |
| BLE support | Yes   | Yes      | Yes      | Yes      | Yes      | Yes       | Yes      | Yes      | No (no BT) |

### Supported Host Devices

| Host Device | ESP32-P4 | ESP32 / C-series / S3 | Other MCUs |
| :---------- | :------: | :-------------------: | :--------: |
| Support     | Yes      | Yes                   | [Yes](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md) |

### HCI transport

| HCI over dedicated UART (H4) | Direct wired UART between host and controller       |
| :--------------------------- | :-------------------------------------------------- |
| MCU host                     | Yes — host runs its own UART driver (`main/uart_driver.c`) |
<!-- common-stop -->

## Directory layout

```text
bluetooth/nimble_standard_hci/bleprph_host_only_uart/
├── cp/          BT-controller co-processor firmware in native UART-H4 mode
└── mcu_host/    ESP-IDF NimBLE host app with its own H4 UART transport
```

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow
> [Getting Started: MCU](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md)
> to install tools and learn the board bring-up. This scenario differs from the
> `hosted_hci` examples: instead of the hosted bus, you wire a **dedicated UART**
> between the host and the controller. Match the baud rate and flow-control
> setting on **both** ends.

## Co-processor (BT controller, UART-H4 mode)

<!-- coprocessor-start -->
The co-processor runs the BT controller in native UART-H4 mode — HCI is exposed
on a physical UART, not on the ESP-Hosted bus. The per-target
`sdkconfig.defaults.esp32*` overlays already enable the UART-H4 symbols on most
targets; where they are not pre-populated, enable them in menuconfig under the
Bluetooth controller HCI mode before building. (`esp32c61` follows the `esp32c6`
UART-H4 settings; `esp32s2` has no BT controller.)

```bash
cd examples/bluetooth/nimble_standard_hci/bleprph_host_only_uart/cp
eh.py set-target <cp_chip>
eh.py menuconfig
```

```text
Component config
└── Bluetooth
     └── Controller Options
          └── HCI mode
               ├── ( ) VHCI
               └── (X) UART                    <── HCI over a dedicated UART (H4)
                    ├── (5)  HCI UART Tx pin    ← wire to host UART Rx
                    ├── (12) HCI UART Rx pin    ← wire to host UART Tx
                    └── [ ]  HCI UART flow control
```

The CP dependency config is pre-set in `sdkconfig.defaults` (pins shown are the
`esp32c6` overlay — target-specific; do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_BT=y            # ESP-Hosted BT controller feature
CONFIG_ESP_HOSTED_CP_BT_ENABLED=y         # required by ESP_HOSTED_CP_FEAT_BT
CONFIG_ESP_HOSTED_CP_FEAT_BT_HCI_UART=y   # BT HCI on a dedicated UART (H4), NOT the hosted bus
CONFIG_BT_ENABLED=y                       # BT controller stack on the CP
CONFIG_BT_CONTROLLER_ONLY=y               # controller-only profile
CONFIG_BT_LE_HCI_INTERFACE_USE_UART=y     # controller HCI carried on UART
CONFIG_BT_LE_HCI_UART_TX_PIN=5            # controller UART TX (to host RX)
CONFIG_BT_LE_HCI_UART_RX_PIN=12           # controller UART RX (from host TX)
CONFIG_BT_LE_HCI_UART_FLOWCTRL=n          # no HW flow control (c6 default)
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=n          # Wi-Fi off — BT controller-only CP
```

Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```
<!-- coprocessor-stop -->

## MCU host (NimBLE, application UART transport)

<!-- esp_host-start -->
The host does not use the ESP-Hosted bus for BT — it talks raw H4 to the
controller over a physical UART via `main/uart_driver.c`. NimBLE's local
controller is disabled and its built-in UART transport is disabled so the
application transport is used instead.

```bash
cd examples/bluetooth/nimble_standard_hci/bleprph_host_only_uart/mcu_host
eh.py set-target <host_chip>
eh.py menuconfig
```

Configure the HCI UART and BLE security under **Example Configuration**:

```text
Example Configuration
├── Uart Configuration
│    ├── (921600) UART Baudrate for HCI              ← range 115200–921600
│    ├── (4)  UART Tx Pin                            ← wire to controller UART Rx
│    ├── (5)  UART Rx Pin                            ← wire to controller UART Tx
│    ├── ( ) Uart Flow Control                       ← Disable / Rx / Tx / Rx+Tx / Max
│    ├── (19) UART Rts Pin
│    └── (23) UART Cts Pin
├── I/O Capability                                   ← Just works (default) / Display / Keyboard / …
├── [ ] Use Bonding
├── [ ] MITM security
├── [ ] Use Secure Connection feature                ← depends on BT_NIMBLE_SM_SC
├── [ ] Enable Extended Adv                          ← depends on SOC_BLE_50_SUPPORTED
├── [ ] Advertise RANDOM Address
├── [ ] Enable Link Encryption
└── [ ] Enable resolving peer address
```

The host dependency config is pre-set in `sdkconfig.defaults` (do not remove):

```text
CONFIG_BT_ENABLED=y                 # BT host stack on
CONFIG_BT_CONTROLLER_DISABLED=y     # no local controller — external UART controller supplies it
CONFIG_BT_BLUEDROID_ENABLED=n       # Bluedroid off
CONFIG_BT_NIMBLE_ENABLED=y          # NimBLE host stack
CONFIG_BT_NIMBLE_TRANSPORT_UART=n   # disable NimBLE's built-in UART transport — the app provides its own
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### Verify

- Wire the dedicated HCI UART: host UART Tx (pin 4) ↔ controller UART Rx, host
  UART Rx (pin 5) ↔ controller UART Tx, common ground. Match the baud rate
  (921600 default) and flow-control setting on both ends.
- Because the controller runs in native UART-H4 mode, HCI does not traverse the
  ESP-Hosted bridge — this example needs no ESP-Hosted host transport for BT.
- Test with any BLE scanner app (LightBlue, nRF Connect). The monitor log shows
  the device address and the advertising GAP procedure once the host↔controller
  UART link is up.
<!-- esp_host-stop -->
