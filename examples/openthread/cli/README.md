# openthread / cli (`examples/openthread/cli/`)

<!-- common-start -->
Runs a full OpenThread CLI (`ot ...`) on the host. The co-processor is the 802.15.4 **Radio Co-Processor (RCP)**: the host drives the RCP lifecycle over ESP-Hosted (RPC) and exchanges 802.15.4 (spinel) traffic over a **dedicated UART**. Adapted from `esp-idf/examples/openthread/ot_cli`.

The co-processor plays two roles: the **ESP-Hosted CP** (control, over the
bus) and the **802.15.4 RCP** (radio, over the dedicated UART). Both are the
same ESP32-C6 (Wi-Fi off) — two roles, not two chips, and not wired to each
other. A working deployment is 3 SoCs: host, that co-processor, and a
separate Thread peer.

```text
  Host (ESP32-P4) - OpenThread stack
    |
    |  ESP-Hosted bus (SDIO / SPI / UART)
    +--- RPC: RCP control -----------> [ ESP-Hosted CP ]      (control stops here)
    |
    |  dedicated UART
    +--- 802.15.4 spinel (data) -----> [ 802.15.4 RCP ] --- 802.15.4 RF ---> Thread peer
```

Data path: host -> spinel UART -> RCP radio -> RF -> peer (a separate SoC).

## Supported Platforms and Transports

### Supported Coprocessors

OpenThread uses an 802.15.4 radio co-processor (RCP).

| Coprocessor | ESP32-C6 | ESP32-H2 | ESP32-H4 | ESP32-C5 |
| :---------: | :------: | :------: | :------: | :------: |
| Support     | Yes      | Yes      | Yes      | Yes      |

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
openthread/cli/
├── cp/                  ESP coprocessor firmware
└── esp_host/            ESP-IDF MCU host app
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
The co-processor runs as the OpenThread **RCP** (Wi-Fi off). Select the ESP-Hosted transport (used for RPC control):

```bash
cd examples/openthread/cli/cp
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

The RCP feature and its 802.15.4 spinel UART (to the host) live under Features:

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── Features
               └── [*] OpenThread RCP (Radio Co-Processor)
                    ├── [*] Auto-initialise OpenThread feature at boot
                    └── OpenThread Transport
                         ├── (X) UART                          <── default (spinel to host)
                         ├── (1)  UART Port to Use
                         ├── (21) TX GPIO number               (ESP32-C6)
                         ├── (20) RX GPIO number               (ESP32-C6)
                         └── (460800) Baud Rate
```

The RCP dependency config is **pre-set in `cp/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD=y   # ESP-Hosted OpenThread RCP feature
CONFIG_OPENTHREAD_ENABLED=y              # OpenThread stack
CONFIG_OPENTHREAD_RADIO=y                # Radio-Only (RCP) device
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=n         # Wi-Fi off on the RCP
CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y      # SW coexistence ON — matches host RX_ON_WHEN_IDLE=n
```

> [!IMPORTANT]
> `CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y` is the canonical RCP setting
> (doc §2.1). It changes the OpenThread capability set the RCP advertises,
> so the host **must** pair it with `CONFIG_OPENTHREAD_RX_ON_WHEN_IDLE=n`
> — a mismatch makes OpenThread init fail.

`CONFIG_OPENTHREAD_RADIO=y` corresponds to *Component config → OpenThread → Thread Core Features → Thread device type → Radio Only Device*. Then flash and monitor:

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
Select the transport (must match the co-processor) and configure the OpenThread RCP UART link:

```bash
cd examples/openthread/cli/esp_host
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
Component config
└── ESP-Hosted
     └── Configure host
          └── Features
               └── [*] OpenThread (RCP control)
                    ├── [*] Auto-initialize OpenThread feature at boot
                    └── OpenThread RCP transport (host side)
                         ├── (X) Dedicated UART                       <── default
                         ├── ( ) ESP-Hosted transport (not supported yet)
                         ├── (1)      UART port
                         ├── (11)     Host TX pin (to RCP RX)
                         ├── (10)     Host RX pin (from RCP TX)
                         └── (460800) Baud rate
```

The host dependency config is **pre-set in `esp_host/sdkconfig.defaults`** (`sdkconfig.defaults.esp32p4` for the PSRAM part) — do not remove:

```text
CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD=y     # host OpenThread feature
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y     # required RPC ext-v2
CONFIG_OPENTHREAD_ENABLED=y
CONFIG_SPIRAM=y                              # REQUIRED — host won't start without PSRAM
CONFIG_OPENTHREAD_RX_ON_WHEN_IDLE=n          # keep in sync with RCP coexistence
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```
<!-- esp_host-stop -->

### 3. Verify

- All standard OT CLI commands work over the RCP — `ot help` enumerates
  them. Form a network with `ot dataset init new` → `ot dataset commit
  active` → `ot ifconfig up` → `ot thread start`, then check `ot
  state` / `ot ipaddr`.

- A second OT device (e.g. another P4 here, or an H2 running upstream
  `ot_cli`) joins by setting the active dataset from the leader (`ot
  dataset set active <hex>`).

- iperf over Thread is wired in via the
  [`iperf-cmd`](https://components.espressif.com/components/espressif/iperf-cmd)
  component — use the ML-EID for server/client targets:

  ```text
  esp32p4> ot ipaddr mleid
  esp32p4> iperf -V -s -t 20 -i 3 -p 5001 -f k          # server
  esp32h2> iperf -V -c <mleid> -t 20 -i 1 -p 5001 -l 85 -f k   # client
  ```

- Custom 4 MB partition table (`partitions.csv` / `partitions.esp32p4.csv`),
  mbedTLS ECJPAKE on for commissioning, `BT_ENABLED=n`.

- See the [OT CLI extension commands](https://github.com/espressif/esp-thread-br/blob/main/components/esp_ot_cli_extension/README.md)
  reference for TCP/UDP / iperf / wifi sub-commands.
