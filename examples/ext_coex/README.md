# ext_coex (`examples/ext_coex/`)

<!-- common-start -->
Configures the coprocessor's external-coex (PTA) work mode and grant
pin sequence so the CP's Wi-Fi / BT radio can share the air with a
third-party radio sitting next to it. The host sends a single RPC
sequence at boot: pick a role (Leader / Follower / Standalone), point
the CP at the four PTA GPIOs (request, priority, grant, tx_line), and
optionally tune the advanced knobs (grant delay, validate-high).

## Supported Platforms and Transports

### Supported Coprocessors

| Coprocessor | ESP32 | ESP32-C Series | ESP32-S Series |
| :----------: | :---: | :------------: | :------------: |
| Support     | No    | Yes            | Yes            |

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
ext_coex/
├── cp/                  ESP coprocessor firmware
├── mcu_host/            ESP-IDF MCU host app
└── linux_802_3_host/    Linux host
     ├── c_app/          native C app
     ├── kmod/           kernel module
     └── py_app/         Python (ctypes) app
```

> **ESP32 as a co-processor is not supported** for external coexistence.
> On advanced-coex chips (ESP32-C6 / C61 / C5) Bluetooth/BLE may stay
> enabled; on others (e.g. ESP32-S3) the BT controller must be disabled
> when EXT_COEX is enabled.

<table width="100%">
  <tr>
    <td width="50%" align="center" bgcolor="#e6f2ff">
      <h3><a href="#linux-host-setup"><img src="../../docs/images/lin.jpeg" height="18" alt="Linux"> Linux Host Setup</a></h3>
      <p><a href="#linux-cp">1. Coprocessor</a><br><a href="#linux-host">2. Linux Host</a></p>
    </td>
    <td width="50%" align="center" bgcolor="#f2ffe6">
      <h3><a href="#mcu-host-setup"><img src="../../docs/images/mcu.jpeg" height="18" alt="MCU"> MCU Host Setup</a></h3>
      <p><a href="#mcu-cp">1. Coprocessor</a><br><a href="#mcu-host">2. MCU Host</a></p>
    </td>
  </tr>
</table>

---

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow **[Getting Started: Linux](../../docs/getting-started-linux.md)** or **[Getting Started: MCU](../../docs/getting-started-mcu.md)** to wire the boards, install tools, choose a transport, and confirm the host↔co-processor handshake. The steps below only add what is specific to this example.

<h2 id="linux-host-setup"><img src="../../docs/images/lin.jpeg" height="22" alt="Linux"> Linux Host Setup</h2>

<h3 id="linux-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

External-Coexistence support is enabled by the co-processor's
`sdkconfig.defaults`. Select the transport via `eh.py menuconfig`:

```bash
cd examples/ext_coex/cp
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

The CP External-Coexistence feature — and the underlying
`ESP_COEX_EXTERNAL_COEXIST_ENABLE` (**Component config → Wireless
Coexistence → External Coexistence**) — are set in `sdkconfig.defaults`:

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── Features
               └── [*] External Coexistence
                    └── [*] Auto-initialise External Coexistence at boot
```

Each bus has its own settings submenu (pins, clock, checksum) — defaults are fine for bring-up.

The CP dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX=y        # CP external-coex (PTA) feature
CONFIG_ESP_COEX_EXTERNAL_COEXIST_ENABLE=y      # REQUIRED — IDF external-coex core
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y               # Wi-Fi radio sharing the air
CONFIG_BT_ENABLED=n                            # BT must be off for external coex
```

Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```

<h3 id="linux-host">2. Linux Host</h3>

> [!NOTE]
> Base wiring, OS setup, and transport bring-up live in [Getting Started: Linux](../../docs/getting-started-linux.md). This section assumes that already works.

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

**Kernel module** (creates `ethsta0` / `ethap0` and `/dev/esps0`):

```bash
cd examples/ext_coex/linux_802_3_host/kmod
./build.sh --bus sdio --reload --reset-gpio 518 --clock-mhz 50
# SPI instead: ./build.sh --bus spi --reload --reset-gpio 518 --clock-mhz 10 --spi-mode 3
```

`--reload` builds, unloads, and reloads with the given module params. On SPI, start at `--clock-mhz 10`; once stable, raise it in steps to the co-processor's max SPI clock (see [Performance](../../docs/design/performance.md)).

**Native C app:**

```bash
cd examples/ext_coex/linux_802_3_host/c_app
eh.py set-target linux
eh.py menuconfig
```

```text
Example Configuration
└── [ ] Use legacy esp_hosted_* compat-surface main    <── default off
```

The host External-Coexistence feature is enabled by `sdkconfig.defaults`;
the PTA controls live under **Features**:

```text
Component config
└── ESP-Hosted
     └── Configure host
          └── Features
               └── [*] External Coexistence
                    ├── [*] Auto-initialize Ext-Coex feature at boot
                    └── [ ] Enable advanced ext-coex controls    ← set_grant_delay / set_validate_high
```

The Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX=y      # host external-coex control feature
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y              # RPC control path
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y       # required RPC ext-v2
```

Then build and run:

```bash
eh.py build
eh.py run
```

**Python app** — same feature set, driven from `main/main.py`:

```bash
cd examples/ext_coex/linux_802_3_host/py_app
eh.py set-target linux
eh.py build
eh.py run
```

### 3. Verify

- Pin numbers in `main.c` (request=11, priority=4, grant=16,
  tx_line=17) are placeholders — replace with the GPIOs your CP board
  actually routes to the third-party radio's PTA header.

- Wire mode is fixed at `EH_HOST_CP_EXT_COEX_WIRE_3` (3-wire PTA);
  other modes are available via the same RPC.

---

<h2 id="mcu-host-setup"><img src="../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

<h3 id="mcu-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

<!-- coprocessor-start -->
External-Coexistence support is enabled by the co-processor's
`sdkconfig.defaults`. Select the transport via `eh.py menuconfig`:

```bash
cd examples/ext_coex/cp
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

The CP External-Coexistence feature — and the underlying
`ESP_COEX_EXTERNAL_COEXIST_ENABLE` (**Component config → Wireless
Coexistence → External Coexistence**) — are set in `sdkconfig.defaults`:

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── Features
               └── [*] External Coexistence
                    └── [*] Auto-initialise External Coexistence at boot
```

Each bus has its own settings submenu (pins, clock, checksum) — defaults are fine for bring-up.

The CP dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX=y        # CP external-coex (PTA) feature
CONFIG_ESP_COEX_EXTERNAL_COEXIST_ENABLE=y      # REQUIRED — IDF external-coex core
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y               # Wi-Fi radio sharing the air
CONFIG_BT_ENABLED=n                            # BT must be off for external coex
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
Select the transport (must match the co-processor):

```bash
cd examples/ext_coex/mcu_host
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
└── [ ] Use legacy esp_hosted_* compat-surface main    <── default off
```

The host External-Coexistence feature is enabled by `sdkconfig.defaults`;
the PTA controls live under **Features**:

```text
Component config
└── ESP-Hosted
     └── Configure host
          └── Features
               └── [*] External Coexistence
                    ├── [*] Auto-initialize Ext-Coex feature at boot
                    └── [ ] Enable advanced ext-coex controls    ← set_grant_delay / set_validate_high
```

The Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX=y      # host external-coex control feature
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y              # RPC control path
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y       # required RPC ext-v2
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y           # system RPC calls
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```
<!-- esp_host-stop -->

### 3. Verify

- Pin numbers in `main.c` (request=11, priority=4, grant=16,
  tx_line=17) are placeholders — replace with the GPIOs your CP board
  actually routes to the third-party radio's PTA header.

- Wire mode is fixed at `EH_HOST_CP_EXT_COEX_WIRE_3` (3-wire PTA);
  other modes are available via the same RPC.

