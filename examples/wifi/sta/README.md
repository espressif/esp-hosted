# Wi-Fi sta (`wifi/sta`)

Connect to an AP as a Wi-Fi station with the radio on the **coprocessor** and the application on the **host**. The application is the upstream IDF `wifi/getting_started/station` example, byte-for-byte: `mcu_host/main/main.c` and `linux_802_3_host/c_app/main/main.c` are the same source as `~/esp-idf/examples/wifi/getting_started/station/main/station_example_main.c`. Same source, every host shape — that identity is the release bar for this tree.

## Support

| Host                              | Folder                       | Status |
| --------------------------------- | ---------------------------- | :----: |
| MCU host (ESP-IDF)                | `mcu_host/`                  |   Y    |
| Linux user-space (C)              | `linux_802_3_host/c_app/`    |   Y    |
| Linux kmod                        | `linux_802_3_host/kmod/`     |   Y    |
| Linux user-space (Python ctypes)  | `linux_802_3_host/py_app/`   |   Y    |

| Coprocessor                                       | Status |
| ------------------------------------------------- | :----: |
| Any Espressif chip with Wi-Fi (default ESP32-C6)  |   Y    |
| ESP32-H2 / H4                                     |   N    |

<table width="100%">
  <tr>
    <td width="50%" align="center" bgcolor="#e6f2ff">
      <h3><a href="#linux-host-setup"><img src="../../../docs/images/lin.jpeg" height="18" alt="Linux"> Linux Host Setup</a></h3>
      <p><a href="#linux-cp">1. Coprocessor</a><br><a href="#linux-host">2. Linux Host</a></p>
    </td>
    <td width="50%" align="center" bgcolor="#f2ffe6">
      <h3><a href="#mcu-host-setup"><img src="../../../docs/images/mcu.jpeg" height="18" alt="MCU"> MCU Host Setup</a></h3>
      <p><a href="#mcu-cp">1. Coprocessor</a><br><a href="#mcu-host">2. MCU Host</a></p>
    </td>
  </tr>
</table>

---

## Scenario

The host application drives Wi-Fi over the control path (RPC); the radio lives on the co-processor. On success the host's netif obtains an IP.

```mermaid
sequenceDiagram
    participant App as Host app
    participant CP as Co-processor (Wi-Fi)
    participant AP as Access Point
    App->>CP: esp_wifi_connect() (RPC)
    CP->>AP: associate + authenticate (SSID/password)
    AP-->>CP: connected
    CP-->>App: WIFI_EVENT_STA_CONNECTED
    Note over App: DHCP over the hosted netif
    App-->>App: IP_EVENT_STA_GOT_IP (got IP)
```

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow **[Getting Started: Linux](../../../docs/getting-started-linux.md)** or **[Getting Started: MCU](../../../docs/getting-started-mcu.md)** to wire the boards, install tools, choose a transport, and confirm the host↔co-processor handshake. The steps below only add what is specific to this example.

<h2 id="linux-host-setup"><img src="../../../docs/images/lin.jpeg" height="22" alt="Linux"> Linux Host Setup</h2>

<h3 id="linux-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

The Wi-Fi radio runs on the co-processor firmware — no extra option to enable. Select the transport:

```bash
cd examples/wifi/sta/cp
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

Wi-Fi is the default CP feature — no extra dependency config (`CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y` is already set in `cp/sdkconfig.defaults`). Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```

<h3 id="linux-host">2. Linux Host</h3>

> [!NOTE]
> Base wiring, OS setup, and transport bring-up live in [Getting Started: Linux](../../../docs/getting-started-linux.md). This section assumes that already works.

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

**Kernel module** (creates `ethsta0` / `ethap0` and `/dev/esps0`):

```bash
cd examples/wifi/sta/linux_802_3_host/kmod
./build.sh --bus sdio --reload --reset-gpio 518 --clock-mhz 50
# SPI instead: ./build.sh --bus spi --reload --reset-gpio 518 --clock-mhz 10 --spi-mode 3
```

`--reload` builds, unloads, and reloads with the given module params. On SPI, start at `--clock-mhz 10`; once stable, raise it in steps to the co-processor's max SPI clock (see [Performance](../../../docs/design/performance.md)).

**Native C app** — set the AP credentials (the app shares `esp_hosted_examples_common`, so the options live under the `Example Connection Configuration` menu):

```bash
cd examples/wifi/sta/linux_802_3_host/c_app
eh.py set-target linux
eh.py menuconfig
```

```text
Example Connection Configuration
├── (myssid) WiFi SSID
├── (mypassword) WiFi Password
├── (6) Maximum retry
└── WiFi band (dual-band CP, e.g. ESP32-C5)
     ├── (X) 2.4 GHz only                     <── default
     ├── ( ) 5 GHz only
     └── ( ) Auto (2.4 + 5 GHz)
```

Host dependency config is **pre-set in `linux_802_3_host/c_app/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y              # RPC control path to the CP
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y       # RPC ext-v2 wire (required)
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y             # host Wi-Fi feature
CONFIG_ESP_HOSTED_HOST_AUTO_FEAT_INIT=y        # auto-init features at boot (Linux host)
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y   # host System feature (FW version / heartbeat / OTA)
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_AUTO_INIT=y   # auto-init Wi-Fi feature
```

Then build and run:

```bash
eh.py build
eh.py run
```

**Python app** — same STA connection; credentials come from the `EH_WIFI_SSID` / `EH_WIFI_PASSWORD` / `EH_WIFI_MAX_RETRY` env vars, not menuconfig:

```bash
cd examples/wifi/sta/linux_802_3_host/py_app
eh.py set-target linux
eh.py build
eh.py run
```

### 3. Verify

- The native Linux C app and the legacy esp_hosted compat variant share `main/`; toggle with `CONFIG_EH_USE_LEGACY_API` (default off → native `eh_host_*`).
- Linux user-space binds a netdev; DHCP runs over it via the host's normal init scripts.
- `linux_802_11/` is intentionally absent — cfg80211 owns netdev integration in-kernel and a user-space `station_example_main.c` isn't the right vehicle there.

---

<h2 id="mcu-host-setup"><img src="../../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

<h3 id="mcu-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

The Wi-Fi radio runs on the co-processor firmware — no extra option to enable. Select the transport:

```bash
cd examples/wifi/sta/cp
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

Wi-Fi is the default CP feature — no extra dependency config (`CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y` is already set in `cp/sdkconfig.defaults`). Then flash and monitor:

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

Select the transport (must match the co-processor) and set the AP credentials:

```bash
cd examples/wifi/sta/mcu_host
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
├── (myssid) WiFi SSID
├── (mypassword) WiFi Password
├── WPA3 SAE mode selection
│    ├── ( ) HUNT AND PECK
│    ├── ( ) H2E
│    └── (X) BOTH                            <── default
├── () PASSWORD IDENTIFIER                    ← SAE H2E / BOTH only
├── (5) Maximum retry
└── WiFi Scan auth mode threshold
     ├── ( ) OPEN
     ├── ( ) WEP
     ├── ( ) WPA PSK
     ├── (X) WPA2 PSK                         <── default
     ├── ( ) WPA/WPA2 PSK
     ├── ( ) WPA3 PSK
     ├── ( ) WPA2/WPA3 PSK
     └── ( ) WAPI PSK
```

Host dependency config is **pre-set in `mcu_host/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y          # RPC control path to the CP
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y   # RPC ext-v2 wire (required)
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y       # system RPCs
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y         # host Wi-Fi feature — routes esp_wifi_* to the CP
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- The host boots and receives the co-processor init event; the capability exchange completes.
- The STA connects to the configured AP and obtains an IP (watch the host console log).
- Traffic flows over the hosted link — e.g. a ping to the gateway succeeds.
