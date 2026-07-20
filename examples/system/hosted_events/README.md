# hosted_events (`examples/system/hosted_events/`)

<!-- common-start -->
Wires the host's ESP-IDF event loop to ESP-Hosted's lifecycle events
so the user application can react to coprocessor state changes
without polling. Three events on the `ESP_HOSTED_EVENT` base:

- `ESP_HOSTED_EVENT_CP_INIT` — CP came up (or, on a second occurrence,
  rebooted). Carries an `esp_reset_reason_t`.

- `ESP_HOSTED_EVENT_CP_HEARTBEAT` — periodic liveness ping at the
  interval the host asked for; absence implies a CP hang.

- `ESP_HOSTED_EVENT_TRANSPORT_FAILURE` — transport layer gave up on
  the bus.

The example associates to an AP, then waits. If the CP reboots or
the heartbeat times out it tears down the netif, re-inits the
transport, and reconnects — or restarts the host, depending on the
configured recovery method.

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
system/hosted_events/
├── cp/                  ESP coprocessor firmware
├── mcu_host/            ESP-IDF MCU host app
└── linux_802_3_host/    Linux host
     ├── c_app/          native C app
     ├── kmod/           kernel module
     └── py_app/         Python (ctypes) app
```

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

The co-processor firmware needs no example-specific options — just select the transport:

```bash
cd examples/system/hosted_events/cp
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
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y   # Wi-Fi feature on the coprocessor (default)
CONFIG_BT_ENABLED=n                # BT controller off — not needed here
```

Then flash and monitor:

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
cd examples/system/hosted_events/linux_802_3_host/kmod
./build.sh --bus sdio --reload --reset-gpio 518 --clock-mhz 50
# SPI instead: ./build.sh --bus spi --reload --reset-gpio 518 --clock-mhz 10 --spi-mode 3
```

`--reload` builds, unloads, and reloads with the given module params. On SPI, start at `--clock-mhz 10`; once stable, raise it in steps to the co-processor's max SPI clock (see [Performance](../../../docs/design/performance.md)).

**Native C app** — the only build-time knob is the legacy compat-surface toggle (the heartbeat / recovery options are MCU-host only):

```bash
cd examples/system/hosted_events/linux_802_3_host/c_app
eh.py set-target linux
eh.py menuconfig
```

```text
Example Configuration
└── [ ] Use legacy esp_hosted_* compat-surface main    <── default off
```

Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y                 # RPC control path
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y          # RPC ext-v2 (required)
CONFIG_ESP_HOSTED_HOST_AUTO_FEAT_INIT=y           # auto-init hosted features at startup
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y   # host System feature (FW version / heartbeat / OTA)
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y                # Wi-Fi feature
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_AUTO_INIT=y      # auto-init Wi-Fi feature
CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT=y           # CP heartbeat / liveness events
CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT_AUTO_INIT=y # auto-init heartbeat feature
```

Then build and run:

```bash
eh.py build
eh.py run
```

**Python app** — same events, no build-time options:

```bash
cd examples/system/hosted_events/linux_802_3_host/py_app
eh.py set-target linux
eh.py build
eh.py run
```

### 3. Verify

- To trigger a recovery manually, hit the CP's `RST` (reset) or
  `BOOT` (hang simulation) button — e.g. via an ESP-Prog connected to
  the on-board ESP32-C6 on a P4 dev board.

- The same events fire over both SDIO and SPI-FD; only the recovery
  log is transport-specific.

---

<h2 id="mcu-host-setup"><img src="../../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

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
cd examples/system/hosted_events/cp
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
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y   # Wi-Fi feature on the coprocessor (default)
CONFIG_BT_ENABLED=n                # BT controller off — not needed here
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
Select the transport (must match the co-processor) and set the heartbeat / recovery options:

```bash
cd examples/system/hosted_events/mcu_host
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
├── [ ] Use legacy esp_hosted_* compat-surface main        <── default off
├── (5)  Heartbeat Interval in Seconds
├── [*] Enable heartbeat timeout monitor                   <── default on
├── (6)  Heartbeat Timeout in Seconds                       ← needs monitor enabled
├── Co-processor Recovery method
│     ├── (X) Restart ESP-Hosted transport                 <── default
│     ├── ( ) Restart the Host
│     └── ( ) Do nothing
├── (myssid)      WiFi SSID
├── (mypassword)  WiFi Password
├── WPA3 SAE mode selection ───────────────  (X) BOTH       <── default
├── (5)  Maximum retry
└── WiFi Scan auth mode threshold ──────────  (X) WPA2 PSK  <── default
```

Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y          # RPC control path
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y   # RPC ext-v2 (required)
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y       # system feature
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y         # Wi-Fi (example associates to an AP)
CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT=y    # CP heartbeat / liveness events
CONFIG_ESP_HOSTED_HOST_TRANSPORT_RESTART_ON_FAILURE=n  # app handles failures via hosted-events
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```
<!-- esp_host-stop -->

### 3. Verify

- To trigger a recovery manually, hit the CP's `RST` (reset) or
  `BOOT` (hang simulation) button — e.g. via an ESP-Prog connected to
  the on-board ESP32-C6 on a P4 dev board.

- The same events fire over both SDIO and SPI-FD; only the recovery
  log is transport-specific.

