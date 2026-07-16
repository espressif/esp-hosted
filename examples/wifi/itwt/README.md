# Wi-Fi iTWT (`wifi/itwt`)

802.11ax **individual Target Wake Time** — the station negotiates a periodic wake schedule with the AP and sleeps the radio between wake-ups, slashing average power for always-connected devices. Ports the upstream IDF `wifi/itwt` example; the radio runs on the **coprocessor**, the app on the **host**. The CP exposes the `eh_host_feat_wifi_ext_itwt` extension. On boot the host opens a serial console (`itwt>`) so TWT setup / teardown / suspend can be driven interactively.

## Support

| Host                              | Folder        | Status |
| --------------------------------- | ------------- | :----: |
| MCU host (ESP-IDF)                | `mcu_host/`   |   Y    |
| Linux user-space (any flavour)    | n/a           |   N    |

| Coprocessor                | Status |
| -------------------------- | :----: |
| ESP32-C5 / C6 / C61 (Wi-Fi 6) |   Y    |
| Any other chip             |   N    |

iTWT is an 802.11ax (Wi-Fi 6) feature — non-AX coprocessors physically cannot run it.

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

The iTWT feature gate (`CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ITWT`) is preset in `cp/sdkconfig.defaults` and needs a Wi-Fi 6 (HE) capable chip. Select the transport:

```bash
cd examples/wifi/itwt/cp
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
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=y            # Wi-Fi (default CP feature)
CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ITWT=y   # iTWT (802.11ax TWT) feature extension
# requires an SoC with Wi-Fi 6 / HE support (CONFIG_SOC_WIFI_HE_SUPPORT) — e.g. ESP32-C5 / C6 / C61
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

Select the transport (must match the co-processor) and set the Wi-Fi / iTWT options:

```bash
cd examples/wifi/itwt/mcu_host
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
├── (myssid) WiFi SSID                             ← point at a Wi-Fi 6 capable AP
├── (mypassword) WiFi Password
├── [*] enable static ip
│    ├── (192.168.4.2) Static IP address
│    ├── (255.255.255.0) Static netmask address
│    └── (192.168.4.1) Static gateway address
├── [ ] enable keep alive qos null                 ← QoS-Null during TWT keeps the association alive
├── iTWT Configuration
│    ├── [*] trigger-enabled
│    ├── [*] announced
│    ├── (255) itwt minimum wake duration          range 1..255, unit 256 us
│    ├── (0) itwt wake duration unit               0 = 256 us, 1 = TU (1024 us)
│    ├── (10) itwt wake interval exponent          range 0..31
│    ├── (512) itwt wake interval mantissa         range 1..65535 (interval = mant × 2^expn us)
│    ├── (0) itwt connection id                    range 0..32767
│    └── (5000) itwt setup timeout times           range 100..65535 ms
├── Maximum CPU frequency                          ← choice, depends on PM_ENABLE
└── Minimum CPU frequency                          ← choice, depends on PM_ENABLE
```

Host dependency config is **pre-set in `mcu_host/sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_RPC=y            # RPC control path to the CP
CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2=y     # RPC ext-v2 wire (required)
CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM=y         # system RPCs
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI=y           # host Wi-Fi feature
CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ITWT=y  # host iTWT feature extension
CONFIG_BT_ENABLED=n                          # BT off
CONFIG_PM_ENABLE=y                           # power management — required to realise TWT savings
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y          # tickless idle so the host sleeps during TWT
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- Power management (`CONFIG_PM_ENABLE`) and tickless idle are on by default to actually realise the TWT savings.
- The host runs a REPL (`itwt>`); built-in commands cover system info, TWT setup / teardown, and RX/TX statistics.
- The example forces the STA to negotiate 11ax (`WIFI_PROTOCOL_11AX`); make sure the AP advertises Wi-Fi 6 or the setup fails with `Must be in 11ax mode to support itwt`.
- Auto light-sleep is currently disabled (SDIO is not light-sleep-safe).

