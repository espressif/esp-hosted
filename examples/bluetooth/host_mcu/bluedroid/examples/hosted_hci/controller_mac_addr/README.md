# BT Controller MAC Address (`bluetooth/host_mcu/bluedroid/examples/hosted_hci/controller_mac_addr`)

Reads — and optionally overrides — the BT controller's MAC address on the
co-processor **before** the BT controller is initialised, then brings
Bluedroid up through the shared `esp_hosted_hci_bluedroid` bridge and
advertises as a BLE beacon (`Bluedroid_Beacon`). Ports upstream
`host_bluedroid_bt_controller_mac_addr` verbatim; the only
ESP-Hosted-specific change is the HCI driver. Bluedroid runs on the
application MCU; the BT controller runs on the ESP-Hosted co-processor.

## Support

| Host                          | Status |
| ----------------------------- | :----: |
| MCU host (ESP-IDF)            |   Y   |
| Linux user-space              |   —    |
| Linux kmod                    |   —    |

| Coprocessor (BT controller)         | Status |
| ----------------------------------- | :----: |
| Any chip with BLE — ESP32 / C2 / C3 / C5 / C6 / C61 / H2 / S3 |   Y   |

<table width="100%">
  <tr>
    <td width="100%" align="center" bgcolor="#f2ffe6">
      <h3><a href="#mcu-host-setup"><img src="../../../../../../../docs/images/mcu.jpeg" height="18" alt="MCU"> MCU Host Setup</a></h3>
      <p><a href="#mcu-cp">1. Coprocessor</a><br><a href="#mcu-host">2. MCU Host</a></p>
    </td>
  </tr>
</table>

---

> [!IMPORTANT]
> **New here? Get a base example working first.** Follow **[Getting Started: MCU](../../../../../../../docs/getting-started-mcu.md)** to wire the boards, install tools, choose a transport, and confirm the host↔co-processor handshake. The steps below only add what is specific to this example.

<h2 id="mcu-host-setup"><img src="../../../../../../../docs/images/mcu.jpeg" height="22" alt="MCU"> MCU Host Setup</h2>

<h3 id="mcu-cp">1. Coprocessor</h3>

Set up the tools once (from the repo root):

```bash
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell
```

The co-processor is the BT controller. The example's `sdkconfig.defaults` already enables the Bluetooth controller-only profile with BT HCI carried over the host bus (VHCI) — you only need to select the transport (must match the host):

```bash
cd examples/bluetooth/cp/hosted_hci
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

The Bluetooth profile is pre-selected by `sdkconfig.defaults`; you can confirm it under **Features**:

```text
Component config
└── ESP-Hosted
     └── Configure coprocessor
          └── Features
               └── Bluetooth                              <── enabled by sdkconfig.defaults
                    ├── [*] Auto-initialise Bluetooth at boot
                    └── BT HCI transport
                         ├── (X) HCI over VHCI (SPI/SDIO/SPI-HD)   <── default — BT HCI over the host bus
                         └── ( ) HCI over UART
```

The CP dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_CP_FEAT_BT=y            # ESP-Hosted BT controller feature
CONFIG_ESP_HOSTED_CP_BT_ENABLED=y         # required by ESP_HOSTED_CP_FEAT_BT (Kconfig warns without it)
CONFIG_ESP_HOSTED_CP_FEAT_BT_HCI_VHCI=y   # BT HCI carried over the host bus (VHCI)
CONFIG_BT_ENABLED=y                       # BT controller stack on the CP
CONFIG_BT_CONTROLLER_ONLY=y               # controller-only profile (host runs the BT host stack)
CONFIG_ESP_HOSTED_CP_FEAT_WIFI=n          # Wi-Fi off - BT controller-only CP
```

`CONFIG_ESP_HOSTED_CP_FEAT_BT` requires `CONFIG_ESP_HOSTED_CP_BT_ENABLED` (the Kconfig warns otherwise); both are set above.

Each bus has its own settings submenu (pins, clock, checksum) — defaults are fine for bring-up. Then flash and monitor:

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

Select the transport (must match the co-processor):

```bash
cd examples/bluetooth/host_mcu/bluedroid/examples/hosted_hci/controller_mac_addr
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

This example adds its own options under **Example Configuration**:

```text
Example Configuration
└── [*] Update MAC Address of BT Controller
```

The Host dependency config is **pre-set in `sdkconfig.defaults`** (do not remove):

```text
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y    # host BT feature (controller runs on the CP)
CONFIG_BT_ENABLED=y                 # BT host stack on
CONFIG_BT_CONTROLLER_DISABLED=y     # no local controller - CP supplies it
CONFIG_BT_BLUEDROID_ENABLED=y       # Bluedroid host stack
```

Then flash and monitor:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

### 3. Verify

- The MAC override is **temporary** and reverts on co-processor reset.
  For a permanent change, burn the address into the co-processor's eFuse.

- `esp_hosted_iface_mac_addr_set(ESP_MAC_BT, ...)` must run **before**
  `esp_hosted_bt_controller_init()` — see `main/main.c`.

