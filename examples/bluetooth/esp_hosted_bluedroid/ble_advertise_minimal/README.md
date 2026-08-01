# BLE Advertise Minimal — Bluedroid over Hosted HCI (`bluetooth/esp_hosted_bluedroid/ble_advertise_minimal`)

<!-- tags: bluetooth, ble, advertising, bluedroid, hosted-hci -->

<!-- common-start -->
Smallest possible Bluedroid + ESP-Hosted example: it brings the BT
**controller** up on the ESP-Hosted **co-processor**, attaches Bluedroid's HCI
driver via the `eh_host_bluedroid` port (built into the `esp_hosted`
component), then starts non-connectable BLE advertising. Bluedroid runs on the
**host**; the controller is reached over the hosted transport
(SDIO / SPI / SPI-HD / UART) via VHCI. Steps 1-3 are the only
ESP-Hosted-specific code — everything after `esp_bluedroid_init()` is standard
IDF Bluedroid. The NimBLE counterpart is
`bluetooth/esp_hosted_nimble/bleprph_minimal`.

## Supported Platforms and Transports

### Supported Coprocessors (BT controller)

| Coprocessor | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 | ESP32-S2 |
| :---------- | :---: | :------: | :------: | :------: | :------: | :-------: | :------: | :------: | :------: |
| BLE support | Yes   | Yes      | Yes      | Yes      | Yes      | Yes       | Yes      | Yes      | No (no BLE) |

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
bluetooth/esp_hosted_bluedroid/ble_advertise_minimal/
├── cp/          BT-controller co-processor firmware (VHCI over hosted transport)
└── mcu_host/    ESP-IDF Bluedroid host app (the advertiser)
```

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow
> [Getting Started: MCU](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md)
> to wire the boards, install tools, choose a transport, and confirm the
> host↔co-processor handshake. The steps below only add what is specific to
> this example. Host and co-processor must select the **same** transport.

## Co-processor (BT controller)

<!-- coprocessor-start -->
The co-processor is the BT controller. `sdkconfig.defaults` already enables the
controller-only profile with BT HCI carried over the host bus (VHCI); you only
select the transport (must match the host):

```bash
cd examples/bluetooth/esp_hosted_bluedroid/ble_advertise_minimal/cp
eh.py set-target <cp_chip>
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

Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```
<!-- coprocessor-stop -->

## MCU host (Bluedroid)

<!-- esp_host-start -->
Select the transport (must match the co-processor):

```bash
cd examples/bluetooth/esp_hosted_bluedroid/ble_advertise_minimal/mcu_host
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

This example adds its own options under **BLE Advertise Minimal config**:

```text
BLE Advertise Minimal config
├── (ESP_HOSTED_BLE) BLE device name (advertised)
├── (100) Advertising interval (ms)                          ← range 20–10240
└── [*] Use one-shot eh_host_bluedroid_init() helper
```

The host dependency config is pre-set in `sdkconfig.defaults` (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y                      # host BT feature (controller runs on the CP)
CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID=y           # Bluedroid HCI port (built into the esp_hosted component)
CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID_AUTO_INIT=n # app calls eh_host_bluedroid_init() explicitly (see below)
CONFIG_BT_ENABLED=y                                  # BT host stack on
CONFIG_BT_CONTROLLER_DISABLED=y                      # no local controller — CP supplies it
CONFIG_BT_BLUEDROID_ENABLED=y                        # Bluedroid host stack
```

Auto-init is **off** on purpose: Bluedroid's HCI driver must attach
synchronously *before* `esp_bluedroid_init()`, an ordering the background
auto-init path can't guarantee — so `app_main()` calls the port helper
`eh_host_bluedroid_init()` (from `eh_host_bluedroid.h`) itself.

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### Verify

- Scan with any generic BLE explorer — LightBlue, nRF Connect, `bluetoothctl` —
  and look for the advertised device name.
- The one-shot `eh_host_bluedroid_init()` helper does controller init + enable
  + HCI-driver attach in one call; disable it (set the option above to `n`) to
  step through each stage individually with the low-level
  `hosted_hci_bluedroid_*` ops.
- The `eh_host_bluedroid` port is documented in
  [Porting a BT stack to ESP-Hosted](../../../../docs/design/bluetooth.md#porting-a-bt-stack-to-esp-hosted).
- ESP32 CP supplies BLE 4.2 only — do **not** enable
  `CONFIG_BT_BLE_50_FEATURES_SUPPORTED=y` against an ESP32 CP, the build will
  fail.
- For a connectable peripheral with GATT services, see
  `bluetooth/esp_hosted_bluedroid/ble_gatt_server`.
<!-- esp_host-stop -->
