# zigbee / thermostat (`examples/zigbee/thermostat/`)

<!-- common-start -->
Runs a Zigbee **Home Automation Thermostat** (a Zigbee Coordinator) on the host. The co-processor is the 802.15.4 **Radio Co-Processor (RCP)** — the *same* stack-agnostic RCP used by OpenThread: the host drives the RCP lifecycle over ESP-Hosted (RPC) and exchanges 802.15.4 (spinel) traffic over a **dedicated UART**. Zigbee vs OpenThread is purely a host-side choice (the host links `esp-zigbee-lib` instead of `openthread`); the co-processor firmware is identical. Adapted from the [ESP Zigbee SDK Thermostat example](https://github.com/espressif/esp-zigbee-sdk/tree/main/examples/home_automation_devices/thermostat). See the [OpenThread and Zigbee Support](https://github.com/espressif/esp-hosted-mcu/blob/main/docs/openthread_zigbee.md) doc for the RCP architecture.

The co-processor plays two roles: the **ESP-Hosted CP** (control, over the
bus) and the **802.15.4 RCP** (radio, over the dedicated UART). Both are the
same ESP32-C6 (Wi-Fi off) — two roles, not two chips, and not wired to each
other. A working deployment is 3 SoCs: host, that co-processor, and a
separate Zigbee end-device.

```text
  Host (ESP32-P4) - Zigbee Coordinator
    |
    |  ESP-Hosted bus (SDIO / SPI / UART)
    +--- RPC: RCP control -----------> [ ESP-Hosted CP ]      (control stops here)
    |
    |  dedicated UART
    +--- 802.15.4 spinel (data) -----> [ 802.15.4 RCP ] --- 802.15.4 RF ---> Zigbee end-device
```

Data path: host -> spinel UART -> RCP radio -> RF -> end-device (a sensor).

## Supported Platforms and Transports

### Supported Coprocessors

Zigbee uses an 802.15.4 radio co-processor (RCP) — the same RCP firmware as OpenThread.

| Coprocessor | ESP32-C6 | ESP32-H2 | ESP32-H4 | ESP32-C5 |
| :---------: | :------: | :------: | :------: | :------: |
| Support     | Yes      | Yes      | Yes      | Yes      |

### Supported Host Devices

The Zigbee stack (`esp-zigbee-lib`) is ESP-IDF-only, so the host is an ESP-IDF device (role `esp_host`). PSRAM is recommended to be used.

| Host Device | ESP32-P4 | Other MCUs | Linux |
| :---------: | :------: | :--------: | :---: |
| Support     | Yes      | No (esp-zigbee-lib is ESP-IDF-only) | No |

### Supported Connection buses

The ESP-Hosted bus carries the RCP **control** plane (RPC); the 802.15.4 spinel **data** plane always rides a separate dedicated UART.

| Connection bus | SDIO | SPI Full-Duplex | SPI Half-Duplex | UART |
| :------------- | :--: | :-------------: | :-------------: | :--: |
| MCU host       | Yes  | Yes             | Yes             | Yes  |
<!-- common-stop -->

## Directory layout

```text
zigbee/thermostat/
├── cp/                  ESP coprocessor firmware (802.15.4 RCP — byte-identical to openthread/cli/cp)
└── esp_host/            ESP-IDF host app (esp-zigbee-lib thermostat)
```

<table width="100%">
  <tr>
    <td width="100%" align="center" bgcolor="#f2ffe6">
      <h3><a href="#mcu-host-setup"><img src="../../../docs/images/mcu.jpeg" height="18" alt="MCU"> MCU Host Setup</a></h3>
      <p><a href="#mcu-cp">1. Coprocessor</a><br><a href="#mcu-host">2. ESP-IDF Host</a></p>
    </td>
  </tr>
</table>

---

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow **[Getting Started: MCU](../../../docs/getting-started-mcu.md)** to wire the boards, install tools, choose a transport, and confirm the host↔co-processor handshake. The steps below only add what is specific to this example.

> [!IMPORTANT]
> **Two links are required between the SoCs:** the ESP-Hosted bus (SDIO/SPI/UART) for RCP control, **and** a dedicated physical UART wire for the 802.15.4 spinel data (host TX/RX GPIO ↔ CP RX/TX GPIO). They are separate connections.

<h2 id="mcu-host-setup"><img src="../../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

<h3 id="mcu-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

<!-- coprocessor-start -->
The co-processor runs as the 802.15.4 **RCP** (Wi-Fi off) — identical to `openthread/cli/cp`. Select the ESP-Hosted transport (used for RPC control):

```bash
cd examples/zigbee/thermostat/cp
eh.py set-target esp32c6
eh.py menuconfig
```

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── CP transport
               └── Communication bus (co-processor <== bus ==> host)
                    ├── ( ) SPI Full Duplex
                    ├── (X) SDIO                    <── default
                    ├── ( ) SPI Half Duplex         ← MCU host only
                    └── ( ) UART                    ← MCU host only
```

The RCP feature and its 802.15.4 spinel UART (to the host) live under Features → *OpenThread RCP (Radio Co-Processor)* (the RCP is stack-agnostic; the "OpenThread" name is IDF's `esp_openthread` RCP component, which also serves Zigbee hosts).

The RCP dependency config is **pre-set in `cp/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD=y   # ESP-Hosted 802.15.4 RCP feature
CONFIG_OPENTHREAD_ENABLED=y              # RCP is built via IDF's openthread component
CONFIG_OPENTHREAD_RADIO=y                # Radio-Only (RCP) device
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=n         # Wi-Fi off on the RCP
CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y      # SW coexistence ON — matches host RX_ON_WHEN_IDLE=n
```

> [!IMPORTANT]
> `CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y` is the canonical RCP setting
> (doc §2.1). It changes the 802.15.4 capability set the RCP advertises,
> so the host **must** pair it with `CONFIG_OPENTHREAD_RX_ON_WHEN_IDLE=n`
> — a mismatch makes RCP/stack init fail.

Then flash and monitor:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```
<!-- coprocessor-stop -->

<h3 id="mcu-host">2. ESP-IDF Host</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

<!-- esp_host-start -->
Select the transport (must match the co-processor) and configure the RCP UART link:

```bash
cd examples/zigbee/thermostat/esp_host
eh.py set-target esp32p4
eh.py menuconfig
```

```text
Component config
└── ESP-Hosted
     └── Configure host
          ├── Host transport
          │    └── Communication bus (match the co-processor)   ← SDIO default
          └── Features
               └── [*] OpenThread / Zigbee (RCP control)
                    ├── [*] Auto-initialize feature at boot
                    └── OpenThread RCP transport (host side)
                         ├── (X) Dedicated UART                  <── default (spinel to RCP)
                         ├── (11)     Host TX pin (to RCP RX)
                         ├── (10)     Host RX pin (from RCP TX)
                         └── (460800) Baud rate
```

The host dependency config is **pre-set in `esp_host/sdkconfig.defaults`** — do not remove:

```text
CONFIG_ZB_ENABLED=y                          # Zigbee stack (esp-zigbee-lib)
CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD=y     # host RCP-control feature (serves Zigbee too)
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y     # required RPC ext-v2 (FEAT_OPENTHREAD depends on it)
CONFIG_SPIRAM=y                              # PSRAM integration
CONFIG_ESP_HOSTED_DFLT_TASK_FROM_SPIRAM=y
CONFIG_OPENTHREAD_RX_ON_WHEN_IDLE=n          # keep in sync with RCP coexistence
```

Erase NVRAM before the first flash if you don't want stale network state
(`eh.py -p <host_usb_serial_port> erase-flash`), then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```
<!-- esp_host-stop -->

### 3. Verify

You need a **second board** running a Zigbee end-device — e.g. the
[ESP Zigbee SDK temperature_sensor](https://github.com/espressif/esp-zigbee-sdk/tree/main/examples/home_automation_devices/temperature_sensor).

- On boot the RCP spinel UART comes up and the host Zigbee Coordinator forms an
  open network for 180 s:

  ```text
  I (1611) ESP_ZIGBEE_RADIO_SPINEL_UART: Spinel UART interface enable successfully
  I (1691) THERMOSTAT: Initialize Zigbee stack
  I (2831) THERMOSTAT: Network(0x03e2) is open for 180 seconds
  ```

- When a Home-Automation temperature-sensor device joins, the thermostat reads
  its manufacturer/model and adds it to the binding table:

  ```text
  I (12369) THERMOSTAT: Model identifier: esp32h2
  I (12429) THERMOSTAT: Bound HA temperature sensor device (0x0000, 0x0a) to local successfully
  ```

- Pressing the `BOOT` button reads the sensor's temperature value / range /
  tolerance and configures periodic reporting (every 10 s, or on a 2 °C change):

  ```text
  I (25521) THERMOSTAT: Attempt to configure reporting for HA temperature sensor
  I (26741) THERMOSTAT: Temperature sensor measured value: 29.60 degrees Celsius
  ```

Notes:
- The RCP is **stack-agnostic** — `cp/` is byte-identical to `openthread/cli/cp` (an `OPENTHREAD_RADIO=y` RCP). Only the host differs (esp-zigbee-lib vs openthread).
- Custom partition table with a `zb_storage` partition (Zigbee NVRAM); `BT_ENABLED=n`.
