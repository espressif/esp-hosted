# Wi-Fi enterprise (`wifi/enterprise`)

Wi-Fi station bring-up against an **enterprise (802.1X / EAP)** AP — the kind of network found in offices and universities, where each client authenticates with certificates or per-user credentials instead of a shared PSK. The radio runs on the **coprocessor** and the application on the **host**. The CP exposes the `eh_host_feat_wifi_ext_ent` extension; the host pulls in `wpa_supplicant`'s enterprise (EAP) machinery and mbedTLS. EAP configuration (method, identity, certificates, anonymous identity, etc.) is pushed from the host to the CP over RPC.

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

The enterprise feature gate (`CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ENT`) is preset in `cp/sdkconfig.defaults`; still enable the upstream EAP support (`ESP_WIFI_ENTERPRISE_SUPPORT` under Component config → Wi-Fi → Enterprise) and select the transport:

```bash
cd examples/wifi/enterprise/cp
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
CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ENT=y   # Enterprise (802.1X / EAP) feature extension
# NOTE: CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT is =n in defaults — enable it manually
#       (Component config → Wi-Fi → Enterprise) or EXT_ENT will not build
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

Select the transport (must match the co-processor):

```bash
cd examples/wifi/enterprise/mcu_host
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

This example ships no `Example Configuration` menu — the EAP knobs live in the application sources / embedded PEM files rather than Kconfig. Edit `mcu_host/main/main.c` (and any embedded certificate / key files referenced from `idf_component.yml` / `CMakeLists.txt`) to set:

- EAP method (PEAP / TTLS / TLS / FAST)
- SSID, identity, anonymous identity, username, password (where applicable)
- CA certificate (server validation)
- Client certificate + private key (EAP-TLS / where mutual auth is required)

For EAP-PEAP / EAP-TTLS the typical minimum is identity + password + CA cert. For EAP-TLS replace the password with client cert + key.

Host dependency config is **pre-set in `mcu_host/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y            # RPC control path to the CP
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y     # RPC ext-v2 wire (required)
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y         # system RPCs
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y           # host Wi-Fi feature
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ENT=y   # host Enterprise (EAP) feature extension
CONFIG_BT_ENABLED=n                          # BT off
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- The example pulls in `wpa_supplicant` enterprise + mbedTLS, which is why `sdkconfig.defaults` forces a 4 MB flash partition layout (`partitions_eh_cp_ota_4m.csv`).
- Certificates and keys are embedded into the host binary at build time and shipped to the CP via RPC; no plain-text creds on the wire.
- Test with a RADIUS test rig (e.g. FreeRADIUS) or a real enterprise SSID; consumer routers won't serve enterprise SSIDs.

