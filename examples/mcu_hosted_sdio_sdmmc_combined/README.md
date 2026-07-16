# mcu_hosted_sdio_sdmmc_combined (`examples/mcu_hosted_sdio_sdmmc_combined/`)

Demonstrates running ESP-Hosted's SDIO transport on the same SDMMC
controller that also hosts an external SD card — the two share the
bus driver but live on different slots. The host brings up Wi-Fi
through ESP-Hosted on SDMMC slot 1 (on-board ESP32-C6), mounts a
FAT-formatted SD card on SDMMC slot 0, scans Wi-Fi before *and* after
filesystem I/O to confirm the radio survives the card init, then
writes / renames / reads a file via the standard FATFS / VFS
interface.

## Support

| Host                              | Folder                       | Status |
| --------------------------------- | ---------------------------- | :----: |
| MCU host (ESP-IDF)                | `mcu_host/`                  |   Y   |

(MCU-only — boards: ESP32-P4 (slot 0 for SD + slot 1 for CP), ESP32-S3.)

| Coprocessor                            | Status |
| -------------------------------------- | :----: |
| any Espressif chip (default ESP32-C6)  |   Y   |

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

The co-processor firmware needs no example-specific options — just select the transport (SDIO, since the point of this example is sharing the SDMMC controller):

```bash
cd examples/mcu_hosted_sdio_sdmmc_combined/cp
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

<h3 id="mcu-host">2. MCU Host</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

Select the transport (SDIO, matching the co-processor) and set the SD-card slot / pins:

```bash
cd examples/mcu_hosted_sdio_sdmmc_combined/mcu_host
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
SD/MMC Example Configuration
├── (0)  SD/MMC slot to use                     ← P4: must be 0 (ESP-Hosted uses slot 1)
├── SD/MMC bus width
│     ├── (X) 4 lines (D0 - D3)                 <── default
│     └── ( ) 1 line (D0)
├── (45)  Card Power Reset                       ← only on chips with external SDMMC IO power
├── CMD GPIO number                             ⎫
├── CLK GPIO number                             ⎬ prompted on S3 (defaults 35/36/37/38/33/34);
├── D0 GPIO number                              │ fixed on P4 (CMD 44, CLK 43, D0-D3 39-42)
├── D1 / D2 / D3 GPIO number                    ⎭ (D1-D3 shown only in 4-line mode)
├── [*] SD power supply comes from internal LDO IO (READ HELP!)  <── default on (P4)
└── (4)  LDO ID                                 ← P4 internal LDO for SD VDD
```

Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_FATFS_VFS_FSTAT_BLKSIZE=4096   # FATFS tuning for the shared SD card
CONFIG_WIFI_RMT_NVS_ENABLED=n         # (ESP32-P4) from sdkconfig.defaults.esp32p4 — disable Wi-Fi RMT NVS
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- ESP-IDF master has an open issue where SDMMC + ESP-Hosted on the
  same controller misbehave at init
  ([esp-idf #16233](https://github.com/espressif/esp-idf/issues/16233));
  this example carries the workaround in
  `main/esp_hosted_wifi.c`.

- Breakout cables tend to be flaky at full SD clock — drop the bus
  speed via menuconfig if the card init fails intermittently.

- CD (card detect) and WP (write protect) lines are not used.

