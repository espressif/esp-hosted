# bleprph_wifi_coex (`bluetooth/host_mcu/nimble/examples/hosted_hci/bleprph_wifi_coex`)

Wi-Fi STA + BLE peripheral running concurrently, both radios on the same
coprocessor. Verbatim merge of the upstream IDF `bluetooth/host_mcu/nimble/examples/hosted_hci/bleprph`
and `protocols/icmp_echo` examples: the host connects to an AP, pings a
target, and simultaneously advertises a GATT server. Wi-Fi comes up via the
hosted auto-init (override_path = esp_hosted, no source changes); BLE comes
up via the one-line `esp_hosted_hci_nimble_setup()` call before
`nimble_port_init()`.

## Support

| Host                | Status |
| ------------------- | :----: |
| MCU host (ESP-IDF)  |   Y   |

| Coprocessor (Wi-Fi + BT controller, same chip)              | Status |
| ----------------------------------------------------------- | :----: |
| Espressif chips with Wi-Fi + BLE (ESP32, C2/3/5/6/61, S3)   |   Y   |
| BLE-only chips (H2, H4)                                     |   N   |

<table width="100%">
  <tr>
    <td width="100%" align="center" bgcolor="#f2ffe6">
      <h3><a href="#mcu-host-setup"><img src="../../../../../../../docs/images/mcu.jpeg" height="18" alt="MCU"> MCU Host Setup</a></h3>
      <p><a href="#mcu-cp">1. Coprocessor</a><br><a href="#mcu-host">2. MCU Host</a></p>
    </td>
  </tr>
</table>

---

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow **[Getting Started: MCU](../../../../../../../docs/getting-started-mcu.md)** to wire the boards, install tools, choose a transport, and confirm the host↔co-processor handshake. The steps below only add what is specific to this example.

<h2 id="mcu-host-setup"><img src="../../../../../../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

<h3 id="mcu-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

This example runs Wi-Fi and BLE concurrently on the same co-processor, so it uses the combined Wi-Fi + BT CP profile. The `wifi_hosted_hci` CP's `sdkconfig.defaults` already enables Wi-Fi plus the Bluetooth controller with BT HCI carried over the host bus (VHCI) — you only need to select the transport (must match the host):

```bash
cd examples/bluetooth/cp/wifi_hosted_hci
eh.py set-target esp32c6
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

Wi-Fi and the Bluetooth profile are pre-selected by `sdkconfig.defaults`; you can confirm the BT side under **Features**:

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── Features
               ├── WiFi                                   <── enabled by sdkconfig.defaults
               └── Bluetooth                              <── enabled by sdkconfig.defaults
                    ├── [*] Auto-initialise Bluetooth at boot
                    └── BT HCI transport
                         ├── (X) HCI over VHCI (SPI/SDIO/SPI-HD)   <── default — BT HCI over the host bus
                         └── ( ) HCI over UART
```

The CP dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y          # Wi-Fi on - Wi-Fi + BT coexistence CP
CONFIG_ESP_HOSTED_CP_FEAT_BT=y            # ESP-Hosted BT controller feature
CONFIG_ESP_HOSTED_CP_BT_ENABLED=y         # required by ESP_HOSTED_CP_FEAT_BT (Kconfig warns without it)
CONFIG_ESP_HOSTED_CP_FEAT_BT_HCI_VHCI=y   # BT HCI carried over the host bus (VHCI)
CONFIG_BT_ENABLED=y                       # BT controller stack on the CP
CONFIG_BT_CONTROLLER_ONLY=y               # controller-only profile (host runs the BT host stack)
```

`CONFIG_ESP_HOSTED_CP_FEAT_BT` requires `CONFIG_ESP_HOSTED_CP_BT_ENABLED` (the Kconfig warns otherwise); both are set above.

Each bus has its own settings submenu (pins, clock, checksum) — defaults are fine for bring-up. Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```

<h3 id="mcu-host">2. MCU Host</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

Select the transport (must match the co-processor):

```bash
cd examples/bluetooth/host_mcu/nimble/examples/hosted_hci/bleprph_wifi_coex
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

Set the Wi-Fi credentials under **Example Connection Configuration** (provided by the `eh_example_connect` helper):

```text
Example Connection Configuration
├── (myssid)   WiFi SSID
└── (mypassword) WiFi Password
```

This example also adds its own options under **Example Configuration**:

```text
Example Configuration
├── (1.1.1.1) Ping target
└── (100) Ping count
```

The Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y       # host Wi-Fi feature (remote radio on the CP)
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y         # host BT feature (controller runs on the CP)
CONFIG_BT_ENABLED=y                      # BT host stack on
CONFIG_BT_CONTROLLER_DISABLED=y          # no local controller - CP supplies it
CONFIG_BT_NIMBLE_ENABLED=y               # NimBLE host stack
CONFIG_ESP_WIFI_REMOTE_LIBRARY_HOSTED=y  # Wi-Fi via esp_wifi_remote over Hosted
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- The single line `esp_hosted_hci_nimble_setup()` before
  `nimble_port_init()` is the ONLY hosted-specific addition to the verbatim
  IDF source. Wi-Fi requires no source change — `override_path` =
  `esp_hosted` swaps in the hosted Wi-Fi implementation transparently.

- Currently IPv4-only.
- Test with any BLE scanner (LightBlue, nRF Connect) plus an AP with
  internet reachability to the configured ping target.

