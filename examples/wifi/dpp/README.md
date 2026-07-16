# Wi-Fi DPP enrollee (`wifi/dpp`)

Wi-Fi Easy-Connect (Device Provisioning Protocol) enrollee — the host MCU prints a QR code, a configurator device (typically a phone) scans it and pushes Wi-Fi credentials, and the station joins the AP without ever having the PSK pre-shared. Ports the upstream IDF `wifi/wifi_easy_connect/dpp-enrollee` example; the radio runs on the **coprocessor**, the app on the **host**. The CP exposes the `eh_host_feat_wifi_ext_dpp` extension; the host pulls in `wpa_supplicant` DPP + mbedTLS.

## Support

| Host                              | Folder        | Status |
| --------------------------------- | ------------- | :----: |
| MCU host (ESP-IDF)                | `mcu_host/`   |   Y    |
| Linux user-space (any flavour)    | n/a           |   N    |

| Coprocessor                                       | Status |
| ------------------------------------------------- | :----: |
| Any Espressif chip with Wi-Fi (default ESP32-C6)  |   Y    |
| ESP32-H2 / H4                                     |   N    |

<table width="100%">
  <tr>
    <td width="100%" align="center" bgcolor="#f2ffe6">
      <h3><a href="#mcu-host-setup"><img src="../../../docs/images/mcu.jpeg" height="18" alt="MCU"> MCU Host Setup</a></h3>
      <p><a href="#mcu-cp">1. Coprocessor</a><br><a href="#mcu-host">2. MCU Host</a></p>
    </td>
  </tr>
</table>

---

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow **[Getting Started: MCU](../../../docs/getting-started-mcu.md)** to wire the boards, install tools, choose a transport, and confirm the host↔co-processor handshake. The steps below only add what is specific to this example.

<h2 id="mcu-host-setup"><img src="../../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

<h3 id="mcu-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

The DPP feature gate (`CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_DPP`) is preset in `cp/sdkconfig.defaults`; still enable the upstream supplicant support (`ESP_WIFI_DPP_SUPPORT` under Component config → Wi-Fi → DPP) and select the transport:

```bash
cd examples/wifi/dpp/cp
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

Each bus has its own settings submenu (pins, clock, checksum) — defaults are fine for bring-up.

CP dependency config is **pre-set in `cp/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y           # Wi-Fi (default CP feature)
CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_DPP=y   # DPP (Easy-Connect) feature extension
# requires CONFIG_ESP_WIFI_DPP_SUPPORT=y (Component config → Wi-Fi → DPP) — enable it manually
```

Then flash and monitor:

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

Select the transport (must match the co-processor) and set the DPP options:

```bash
cd examples/wifi/dpp/mcu_host
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

```text
Example Configuration
├── (6) DPP Listen channel list                    ← comma-separated listen channels
├── ()  Bootstrapping key                          ← 64 hex digits; blank = fresh key each boot
└── ()  Additional Device Info                     ← optional, baked into the QR
```

Host dependency config is **pre-set in `mcu_host/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y            # RPC control path to the CP
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y     # RPC ext-v2 wire (required)
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y         # system RPCs
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y           # host Wi-Fi feature
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP=y   # host DPP feature extension
CONFIG_BT_ENABLED=n                          # BT off (DPP is Wi-Fi only)
CONFIG_WIFI_RMT_NVS_ENABLED=n                # ignore saved CP creds — DPP supplies fresh ones
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- On boot the host prints a QR code on its console; scan it with the Wi-Fi Easy-Connect feature of an Android phone (or any DPP configurator).
- Saved Wi-Fi config on the CP is disabled (`CONFIG_WIFI_RMT_NVS_ENABLED=n`) so DPP-supplied credentials always win.
- IDF >= 5.5 delivers DPP results via `WIFI_EVENT_DPP_*`; older IDF uses the supplicant callback path. The example handles both.

