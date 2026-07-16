# openthread / border_router (`examples/openthread/border_router/`)

OpenThread Border Router on the host, driving a **single** ESP-Hosted
co-processor that provides **both** the 802.15.4 **Radio Co-Processor
(RCP)** and the Wi-Fi backhaul. The host drives the RCP lifecycle over
ESP-Hosted (RPC) and exchanges 802.15.4 (spinel) over a **dedicated
UART**; Wi-Fi is exposed to the host through the RPC Wi-Fi feature. The
host runs the full OT BR stack (border routing, mDNS, NAT64 / DHCPv6
hooks via the IDF ot_br adaptation). Because OpenThread and Wi-Fi share
one radio on the CP, expect coexistence trade-offs; for a multi-MCU
split (P4 + C6 Wi-Fi + H2 RCP) see the ESP-Thread-BR P4 example linked
below. Adapted from `esp-idf/examples/openthread/ot_br`.

## Support

| Host                          | Folder              | Status |
| ----------------------------- | ------------------- | :----: |
| MCU host (ESP-IDF, esp32p4)   | `esp_host/`     |   Y    |

| Coprocessor                                                        | Folder    | Status |
| ----------------------------------------------------------------- | --------- | :----: |
| RCP + Wi-Fi CP (ESP32-C5 / C6) — combined 802.15.4 radio + Wi-Fi   | `cp/`     |   Y    |

For a multi-MCU split (P4 + C6 Wi-Fi + H2 RCP), see the
[ESP-Thread-BR P4 example](https://github.com/espressif/esp-thread-br/blob/main/examples/basic_thread_border_router/README_esp32p4.md).

> Running OT and Wi-Fi together on the same ESP32-C6 CP shares one
> radio — expect coexistence trade-offs. See the
> [C6 RF coexistence matrix](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-guides/coexist.html).

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

The co-processor is the OpenThread **RCP** *and* provides the Wi-Fi backbone for the border router (single-radio SW coexistence). Select the ESP-Hosted transport (RPC control):

```bash
cd examples/openthread/border_router/cp
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
               ├── [*] Wi-Fi                                  (backbone for the border router)
               └── [*] OpenThread RCP (Radio Co-Processor)
                    ├── [*] Auto-initialise OpenThread feature at boot
                    └── OpenThread Transport
                         ├── (X) UART                          <── default (spinel to host)
                         ├── (1)  UART Port to Use
                         ├── (21) TX GPIO number               (ESP32-C6)
                         ├── (20) RX GPIO number               (ESP32-C6)
                         └── (460800) Baud Rate
```

CP dependency config is **pre-set in `cp/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD=y   # OpenThread RCP feature
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y         # Wi-Fi backbone for the border router
CONFIG_OPENTHREAD_ENABLED=y
CONFIG_OPENTHREAD_RADIO=y                 # Radio-Only (RCP) device
CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y      # Wi-Fi + 802.15.4 share one radio (SW coex)
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

Select the transport (must match the co-processor), then configure the OpenThread RCP link:

```bash
cd examples/openthread/border_router/esp_host
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

The host drives the OpenThread RCP over a **dedicated UART**, separate from the ESP-Hosted transport bus:

```text
Component config
└── ESP-Hosted
     └── Configure host
          └── Features
               └── [*] OpenThread (RCP control)
                    └── OpenThread RCP transport (host side)
                         ├── (X) Dedicated UART                       <── default
                         └── ( ) ESP-Hosted transport (not supported yet)
                    ├── (1)      UART port
                    ├── (11)     Host TX pin (to RCP RX)
                    ├── (10)     Host RX pin (from RCP TX)
                    └── (460800) Baud rate
```

Border-router role settings live in the upstream `Component config → OpenThread` (border-router on, RCP over UART spinel, IPv6 forwarding hooks) and `Component config → LWIP` (IPv6 forward + custom IP6 input/route hooks for NAT64 / off-mesh routing).

Host dependency config is **pre-set in `esp_host/sdkconfig.defaults`** (`.esp32p4` for the PSRAM part) — do not remove:

```text
CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD=y     # host OpenThread feature
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y           # Wi-Fi backbone
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y     # required RPC ext-v2
CONFIG_OPENTHREAD_ENABLED=y
CONFIG_OPENTHREAD_BORDER_ROUTER=y            # border-router role
CONFIG_SPIRAM=y                              # REQUIRED — host won't start without PSRAM
CONFIG_OPENTHREAD_RX_ON_WHEN_IDLE=n          # keep in sync with RCP coexistence
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- Backhaul Wi-Fi credentials are set with the CLI extension:

  ```text
  esp32p4> ot wifi connect -s <SSID> -p <PASSWORD>
  esp32p4> ot wifi state
  ```

  (see `ot wifi` help in monitor for the full command set).

- Forming the Thread network uses the standard OT CLI (`ot dataset init
  new` / `commit active` / `ifconfig up` / `thread start`) — same
  sequence as the [`thread/cli`](../cli/README.md) example.

- 4 MB flash with a custom partition table (`partitions.csv`); mbedTLS
  ECJPAKE + DTLS enabled for Thread commissioning; multi-instance mDNS
  on; `BT_ENABLED=n` to free coexistence pressure.

- The `esp_hosted_openthread_app.c` shim wires the IDF OT RCP spinel
  layer to ESP-Hosted's Wi-Fi netif so OT BR routes between the Thread
  mesh and the Hosted-backed Wi-Fi netif.

- More: [ESP-IDF ot_br](https://github.com/espressif/esp-idf/tree/master/examples/openthread/ot_br),
  [ESP-Thread-BR SDK](https://github.com/espressif/esp-thread-br).

