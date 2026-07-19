# mem_monitor (`examples/mem_monitor/`)

<!-- common-start -->
Subscribes the host to periodic heap / task-stat reports from the
coprocessor. The CP samples its own internal-DMA, internal-8bit,
external-DMA, and external-8bit heaps at a configurable interval and
pushes an event up; the host either logs every report or only the ones
that cross a low-water threshold. Useful for catching slow leaks or
sizing buffers without leaving instrumentation in the CP firmware.

## Supported Platforms and Transports

### Supported Coprocessors

| Coprocessor | ESP32 | ESP32-C Series | ESP32-S Series |
| :----------: | :---: | :------------: | :------------: |
| Support     | Yes   | Yes            | Yes            |

### Supported Host Devices

| Host Device | ESP32-P4 | ESP32-H2 | Other MCUs | Linux |
| :---------: | :------: | :------: | :--------: | :---: |
| Support     | Yes | Yes | [Yes](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md) | [Yes](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-linux.md) |

### Supported Connection buses

| Connection bus | SDIO | SPI Full-Duplex | SPI Half-Duplex | UART |
| :------------- | :--: | :-------------: | :-------------: | :--: |
| Linux host     | Yes  | Yes             | No              | No   |
| MCU host       | Yes  | Yes             | Yes             | Yes  |
<!-- common-stop -->

## Directory layout

```text
mem_monitor/
├── cp/                  ESP coprocessor firmware
└── mcu_host/            ESP-IDF MCU host app
```

(MCU-only — no Linux host variant.)

<table width="100%">
  <tr>
    <td width="100%" align="center" bgcolor="#f2ffe6">
      <h3><a href="#mcu-host-setup"><img src="../../docs/images/mcu.jpeg" height="18" alt="MCU"> MCU Host Setup</a></h3>
      <p><a href="#mcu-cp">1. Coprocessor</a><br><a href="#mcu-host">2. MCU Host</a></p>
    </td>
  </tr>
</table>

---

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow **[Getting Started: MCU](../../docs/getting-started-mcu.md)** to wire the boards, install tools, choose a transport, and confirm the host↔co-processor handshake. The steps below only add what is specific to this example.

<h2 id="mcu-host-setup"><img src="../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

<h3 id="mcu-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

<!-- coprocessor-start -->
The co-processor firmware needs no example-specific options — just select the transport:

```bash
cd examples/mem_monitor/cp
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

CP dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y                # Wi-Fi feature on the coprocessor (default)
CONFIG_BT_ENABLED=n                             # BT controller off — not needed here
CONFIG_ESP_HOSTED_CP_FEAT_DEBUG=y               # parent debug feature (hosts mem-monitor)
CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_MEM_MONITOR=y   # mem-monitor sub-feature — heap/task reports
```

Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```
<!-- coprocessor-stop -->

<h3 id="mcu-host">2. MCU Host</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

<!-- esp_host-start -->
Select the transport (must match the co-processor) and set the monitor options:

```bash
cd examples/mem_monitor/mcu_host
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
├── [ ] Use legacy esp_hosted_* compat-surface main            <── default off
├── [ ] Query Memory Info only once                            <── default off
│        (the options below apply while this is off)
├── (10)  Memory Monitor Interval (in seconds)
├── [*] Always send a memory report                            <── default on
├── (8192) Threshold for internal DMA Heap Memory (in bytes)   ⎫
├── (8192) Threshold for internal 8-Bit Heap Memory (in bytes) ⎬ used when
├── (8192) Threshold for external DMA Heap Memory (in bytes)   │ "Always send"
├── (8192) Threshold for external 8-Bit Heap Memory (in bytes) ⎭ is off
└── Wifi Config
      ├── (myssid)      WiFi SSID
      ├── (mypassword)  WiFi Password
      ├── WPA3 SAE mode selection ───────────  (X) BOTH         <── default
      ├── (5)  Maximum retry
      └── WiFi Scan auth mode threshold ──────  (X) WPA2 PSK    <── default
```

Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y            # RPC control path
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y     # RPC ext-v2 (required)
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y         # system feature
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y           # Wi-Fi (example connects to an AP)
CONFIG_ESP_HOSTED_HOST_FEAT_MEM_MONITOR=y    # subscribe to CP mem-monitor reports
CONFIG_ESP_WIFI_REMOTE_LIBRARY_HOSTED=y      # route esp_wifi_remote through ESP-Hosted
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```
<!-- esp_host-stop -->

### 3. Verify

- External-heap thresholds only matter when SPIRAM is enabled on the
  CP — without PSRAM the external counters stay at zero.

- The example connects to Wi-Fi after a delay (`2 * interval`) so a
  memory report lands close to the state transition.

