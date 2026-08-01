# BT Controller MAC Address — Bluedroid over Hosted HCI (`bluetooth/bluedroid_hosted_hci/controller_mac_addr`)

<!-- tags: bluetooth, ble, mac-address, bluedroid, hosted-hci -->

<!-- common-start -->
Reads — and optionally overrides — the BT controller's MAC address on the
co-processor **before** the BT controller is initialised, then brings Bluedroid
up through the shared `esp_hosted_hci_bluedroid` bridge and advertises as a BLE
beacon (`Bluedroid_Beacon`). Ports upstream
`host_bluedroid_bt_controller_mac_addr` verbatim; the only ESP-Hosted-specific
change is the HCI driver. Bluedroid runs on the **host**; the BT **controller**
runs on the ESP-Hosted **co-processor**, reached over the hosted transport
(SDIO / SPI / SPI-HD / UART) via VHCI.

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
bluetooth/bluedroid_hosted_hci/controller_mac_addr/
├── cp/          BT-controller co-processor firmware (VHCI over hosted transport)
└── mcu_host/    ESP-IDF Bluedroid host app (MAC read/override + beacon)
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
cd examples/bluetooth/bluedroid_hosted_hci/controller_mac_addr/cp
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
cd examples/bluetooth/bluedroid_hosted_hci/controller_mac_addr/mcu_host
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

This example adds its own options under **Example Configuration**:

```text
Example Configuration
└── [*] Update MAC Address of BT Controller
```

The host dependency config is pre-set in `sdkconfig.defaults` (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y    # host BT feature (controller runs on the CP)
CONFIG_BT_ENABLED=y                 # BT host stack on
CONFIG_BT_CONTROLLER_DISABLED=y     # no local controller — CP supplies it
CONFIG_BT_BLUEDROID_ENABLED=y       # Bluedroid host stack
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### Verify

- The monitor log prints the controller MAC before and (if enabled) after the
  override; the device advertises as `Bluedroid_Beacon`.
- `esp_hosted_iface_mac_addr_set(ESP_MAC_BT, ...)` must run **before**
  `esp_hosted_bt_controller_init()` — see `main/main.c`.
- The MAC override is **temporary** and reverts on co-processor reset. For a
  permanent change, burn the address into the co-processor's eFuse.
<!-- esp_host-stop -->
