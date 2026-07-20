# mcu_transport_config (`examples/system/mcu_transport_config/`)

<!-- common-start -->
Configures the host-side transport (SDIO / SPI-FD / SPI-HD-dual /
SPI-HD-quad / UART) programmatically at runtime, *before*
`esp_hosted_init()`. The same Kconfig defaults are available via
`menuconfig`, but this example shows how to override them in code —
useful when the bus pins or clock have to come from the application
(board ID, NVS, dip switch) rather than the build configuration.

(MCU-only — Linux host transport is configured via the bus driver,
not this API.)

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
system/mcu_transport_config/
├── cp/                  ESP coprocessor firmware
└── mcu_host/            ESP-IDF MCU host app
```

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

<!-- coprocessor-start -->
This example ships no Kconfig of its own — the co-processor transport selection *is* the point. Pick the bus to match what the host will configure at runtime:

```bash
cd examples/system/mcu_transport_config/cp
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
There is no example-specific Kconfig — the host transport is exactly what this example overrides *in code* at runtime. The `menuconfig` transport choice sets the `INIT_DEFAULT_*` pin / clock defaults; the example then overwrites whichever fields it needs (SDIO 1- or 4-bit, SPI-FD, SPI-HD dual / quad, UART):

```bash
cd examples/system/mcu_transport_config/mcu_host
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

Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN=n  # defer init so app_main can set transport config first
CONFIG_ESP_WIFI_ENABLED=y                           # Wi-Fi (default feature set)
CONFIG_BT_ENABLED=y                                 # BT enabled (optional; Bluedroid host)
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```
<!-- esp_host-stop -->

### 3. Verify

- All overrides must happen *before* `esp_hosted_init()` — the host
  caches the resolved config at init time.

- On ESP32-P4 the SDMMC slot used by ESP-Hosted is fixed to slot 1
  (slot 0 is reserved for an external SD card; see
  `examples/mcu_hosted_sdio_sdmmc_combined/`).

