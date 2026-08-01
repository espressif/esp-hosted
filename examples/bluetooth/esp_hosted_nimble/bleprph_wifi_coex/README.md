# BLE + Wi-Fi Coexistence — NimBLE over Hosted HCI (`bluetooth/esp_hosted_nimble/bleprph_wifi_coex`)

<!-- tags: bluetooth, ble, wifi-coex, nimble, hosted-hci -->

<!-- common-start -->
Wi-Fi STA + BLE peripheral running concurrently, both radios on the same
co-processor. Verbatim merge of the upstream IDF NimBLE `bleprph` and
`protocols/icmp_echo` examples: the **host** connects to an AP, pings a target,
and simultaneously advertises a GATT server. Wi-Fi comes up via the hosted
auto-init (`override_path` = `esp_hosted`, no source changes); BLE likewise
needs no hosted-specific call — the NimBLE hosted port is built into the
`esp_hosted` component and [auto-inits at
boot](https://github.com/espressif/esp-hosted/blob/master/docs/design/bluetooth.md#porting-a-bt-stack-to-esp-hosted).
The co-processor runs the combined Wi-Fi + BT controller firmware
(`wifi_hosted_hci`), with BT HCI carried over the hosted transport
(SDIO / SPI / SPI-HD / UART) via VHCI.

## Supported Platforms and Transports

### Supported Coprocessors (Wi-Fi + BT controller, same chip)

| Coprocessor       | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 | ESP32-S2 |
| :---------------- | :---: | :------: | :------: | :------: | :------: | :-------: | :------: | :------: | :------: |
| Wi-Fi + BLE       | Yes   | Yes      | Yes      | Yes      | Yes      | Yes       | No (no Wi-Fi) | Yes | No (no BLE) |

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
bluetooth/esp_hosted_nimble/bleprph_wifi_coex/
├── cp/          Wi-Fi + BT-controller co-processor firmware (wifi_hosted_hci, VHCI over hosted transport)
└── mcu_host/    ESP-IDF NimBLE host app (Wi-Fi STA + ping + GATT server)
```

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow
> [Getting Started: MCU](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md)
> to wire the boards, install tools, choose a transport, and confirm the
> host↔co-processor handshake. The steps below only add what is specific to
> this example. Host and co-processor must select the **same** transport.

## Co-processor (Wi-Fi + BT controller)

<!-- coprocessor-start -->
This example runs Wi-Fi and BLE concurrently on the same co-processor, so it
uses the combined Wi-Fi + BT CP profile. The `wifi_hosted_hci` CP's
`sdkconfig.defaults` already enables Wi-Fi plus the Bluetooth controller with BT
HCI carried over the host bus (VHCI); you only select the transport (must match
the host):

```bash
cd examples/bluetooth/esp_hosted_nimble/bleprph_wifi_coex/cp
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

Wi-Fi and the Bluetooth profile are pre-selected by `sdkconfig.defaults`;
confirm the BT side under **Features**:

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── Features
               ├── WiFi                                   <── enabled by sdkconfig.defaults
               └── Bluetooth                              <── enabled by sdkconfig.defaults
                    ├── [*] Auto-initialise Bluetooth at boot
                    └── BT HCI transport
                         ├── (X) HCI over VHCI (SPI/SDIO/SPI-HD)   <── default
                         └── ( ) HCI over UART
```

The CP dependency config is pre-set in `sdkconfig.defaults` (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y          # Wi-Fi on — Wi-Fi + BT coexistence CP
CONFIG_ESP_HOSTED_CP_FEAT_BT=y            # ESP-Hosted BT controller feature
CONFIG_ESP_HOSTED_CP_BT_ENABLED=y         # required by ESP_HOSTED_CP_FEAT_BT
CONFIG_ESP_HOSTED_CP_FEAT_BT_HCI_VHCI=y   # BT HCI over the host bus (VHCI)
CONFIG_BT_ENABLED=y                       # BT controller stack on the CP
CONFIG_BT_CONTROLLER_ONLY=y               # controller-only profile
```

Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```
<!-- coprocessor-stop -->

## MCU host (NimBLE + Wi-Fi)

<!-- esp_host-start -->
Select the transport (must match the co-processor):

```bash
cd examples/bluetooth/esp_hosted_nimble/bleprph_wifi_coex/mcu_host
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

Set the Wi-Fi credentials under **Example Connection Configuration** (provided by
the `eh_example_connect` helper):

```text
Example Connection Configuration
├── (myssid)     WiFi SSID
└── (mypassword) WiFi Password
```

This example also adds its own options under **Example Configuration**:

```text
Example Configuration
├── (1.1.1.1) Ping target
└── (100) Ping count                                         ← one ping per second
```

The host dependency config is pre-set in `sdkconfig.defaults` (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y       # host Wi-Fi feature (remote radio on the CP)
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y         # host BT feature (controller runs on the CP)
CONFIG_ESP_HOSTED_HOST_BT_PORT_NIMBLE=y  # NimBLE hosted port — auto-inits at boot
CONFIG_BT_ENABLED=y                      # BT host stack on
CONFIG_BT_CONTROLLER_DISABLED=y          # no local controller — CP supplies it
CONFIG_BT_NIMBLE_ENABLED=y               # NimBLE host stack
CONFIG_ESP_WIFI_REMOTE_LIBRARY_HOSTED=y  # Wi-Fi via esp_wifi_remote over Hosted
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### Verify

- Test with any BLE scanner (LightBlue, nRF Connect) **plus** an AP with
  internet reachability to the configured ping target — BLE advertising and the
  Wi-Fi ping stream run concurrently.
- No hosted-specific call is needed on either radio: the NimBLE hosted port
  auto-inits at boot, and Wi-Fi's `override_path` = `esp_hosted` swaps in the
  hosted Wi-Fi implementation transparently — the source is verbatim IDF.
- Currently IPv4-only.
<!-- esp_host-stop -->
