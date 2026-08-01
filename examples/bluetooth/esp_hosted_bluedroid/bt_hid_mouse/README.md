# BT Classic HID Mouse — Bluedroid over Hosted HCI (`bluetooth/esp_hosted_bluedroid/bt_hid_mouse`)

<!-- tags: bluetooth, classic-bt, hid, bluedroid, hosted-hci -->

<!-- common-start -->
Acts as a Bluetooth Classic HID mouse — pairs with a host (PC or phone) and
sends mouse-movement reports. Ports upstream `host_bluedroid_bt_hid_mouse_device`
verbatim; the only ESP-Hosted-specific change is one call —
`esp_hosted_bt_stack_setup()` — that brings the controller up and binds Bluedroid
to the hosted HCI. Bluedroid
+ the HID device profile run
on the **host**; the BR/EDR **controller** runs on the ESP-Hosted **co-processor**,
reached over the hosted transport (SDIO / SPI / SPI-HD / UART) via VHCI.

**Classic Bluetooth (BR/EDR) is supported only on an ESP32 co-processor.** The
C/S/H-series chips are BLE-only and will fail at controller init.

## Supported Platforms and Transports

### Supported Coprocessors (BR/EDR controller)

| Coprocessor       | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 | ESP32-S2 |
| :---------------- | :---: | :------: | :------: | :------: | :------: | :-------: | :------: | :------: | :------: |
| Classic BT (BR/EDR) | Yes | No       | No       | No       | No       | No        | No       | No       | No       |

### Supported Host Devices

| Host Device | ESP32-P4 | ESP32-H2 | Other MCUs |
| :---------- | :------: | :------: | :--------: |
| Support     | Yes      | Yes      | [Yes](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md) |

### Supported HCI transports

| HCI over hosted bus (VHCI) | SDIO | SPI Full-Duplex | SPI Half-Duplex | UART |
| :------------------------- | :--: | :-------------: | :-------------: | :--: |
| MCU host                   | Yes  | Yes             | Yes             | Yes  |
<!-- common-stop -->

## Directory layout

```text
bluetooth/esp_hosted_bluedroid/bt_hid_mouse/
├── cp/          BR/EDR-controller co-processor firmware (ESP32 only, VHCI over hosted transport)
└── mcu_host/    ESP-IDF Bluedroid host app (the HID mouse device)
```

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow
> [Getting Started: MCU](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md)
> to wire the boards, install tools, choose a transport, and confirm the
> host↔co-processor handshake. The steps below only add what is specific to
> this example. Host and co-processor must select the **same** transport.

## Co-processor (BR/EDR controller)

<!-- coprocessor-start -->
The co-processor is the BR/EDR (Classic Bluetooth) controller — Classic BT is
supported only on an ESP32 CP. `sdkconfig.defaults` already enables the
controller-only profile with BT HCI carried over the host bus (VHCI); you only
select the transport (must match the host):

```bash
cd examples/bluetooth/esp_hosted_bluedroid/bt_hid_mouse/cp
eh.py set-target esp32
eh.py menuconfig
```

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── CP transport
               ├── Communication bus (co-processor <== bus ==> host)
               │    ├── ( ) SPI Full Duplex
               │    ├── (X) SDIO                    <── default
               │    ├── ( ) SPI Half Duplex         ← MCU host only
               │    └── ( ) UART                    ← MCU host only
               └── SDIO Configuration               ← clock, GPIOs, checksum (defaults OK)
```

The Bluetooth profile is pre-selected by `sdkconfig.defaults`; confirm it under
**Features**:

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── Features
               └── Bluetooth                              <── enabled by sdkconfig.defaults
                    ├── [*] Auto-initialise Bluetooth at boot
                    └── BT HCI transport
                         ├── (X) HCI over VHCI (SPI/SDIO/SPI-HD)   <── default
                         └── ( ) HCI over UART
```

The CP dependency config is pre-set in `sdkconfig.defaults` (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_BT=y            # ESP-Hosted BT controller feature
CONFIG_ESP_HOSTED_CP_BT_ENABLED=y         # required by ESP_HOSTED_CP_FEAT_BT
CONFIG_ESP_HOSTED_CP_FEAT_BT_HCI_VHCI=y   # BT HCI over the host bus (VHCI)
CONFIG_BT_ENABLED=y                       # BT controller stack on the CP
CONFIG_BT_CONTROLLER_ONLY=y               # controller-only profile
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=n          # Wi-Fi off — BT controller-only CP
```

Classic BT (BR/EDR) additionally requires an ESP32 CP (`eh.py set-target esp32`)
— the controller-only profile above provides BR/EDR only on ESP32.

Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```
<!-- coprocessor-stop -->

## MCU host (Bluedroid)

<!-- esp_host-start -->
Select the transport (must match the co-processor):

```bash
cd examples/bluetooth/esp_hosted_bluedroid/bt_hid_mouse/mcu_host
eh.py set-target esp32p4
eh.py menuconfig
```

```text
Component config
└── ESP-Hosted
     └── Configure host
          └── Host transport
               ├── Communication bus (co-processor <== bus ==> host)
               │    ├── ( ) SPI Full Duplex
               │    ├── (X) SDIO                    <── default (match the co-processor)
               │    ├── ( ) SPI Half Duplex
               │    └── ( ) UART
               └── SDIO Configuration               ← slot, bus width, GPIOs (defaults OK)
```

This example adds its own options under **HID Example Configuration**:

```text
HID Example Configuration
├── [*] Secure Simple Pairing                                ← depends on BT_CLASSIC_ENABLED
└── (HID Mouse Example) Local Device Name
```

The host dependency config is pre-set in `sdkconfig.defaults` (do not remove).
`CONFIG_ESP_HOSTED_HOST_FEAT_BT` enables the BT host feature; the host stack is
chosen by the IDF BT Kconfig (`CONFIG_BT_BLUEDROID_ENABLED`) — no separate hosted
BT-port switch:

```text
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y                       # host BT feature (controller runs on the CP)
CONFIG_SLAVE_IDF_TARGET_ESP32=y                        # CP must be ESP32 — Classic BT (BR/EDR) only on ESP32
CONFIG_BT_ENABLED=y                                    # BT host stack on
CONFIG_BT_CONTROLLER_DISABLED=y                        # no local controller — CP supplies it
CONFIG_BT_BLUEDROID_ENABLED=y                          # Bluedroid host stack (selects the hosted BT adapter)
CONFIG_BT_CLASSIC_ENABLED=y                            # Classic BT (BR/EDR)
CONFIG_BT_HID_ENABLED=y                                # HID profile
CONFIG_BT_HID_DEVICE_ENABLED=y                         # HID device role
```

`app_main` calls `esp_hosted_bt_stack_setup()` once, after
`esp_hosted_connect_to_slave()` and before `esp_bluedroid_init()`, to bring the
controller up and bind Bluedroid to the hosted HCI. See
[Porting a BT stack to ESP-Hosted](https://github.com/espressif/esp-hosted/blob/master/docs/design/bluetooth.md#porting-a-bt-stack-to-esp-hosted).

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### Verify

- This is a **Classic Bluetooth** profile (`CONFIG_BT_CLASSIC_ENABLED=y`,
  `CONFIG_BT_HID_DEVICE_ENABLED=y`). C/S/H-series CPs are BLE-only and will
  fail at controller init — use an ESP32 CP.
- Pair the mouse from a PC or phone and confirm cursor movement reports.
- The host enables `CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=y` to drive Bluedroid's
  BR/EDR feature gates even though the local controller is disabled (the CP
  supplies the controller).
<!-- esp_host-stop -->
