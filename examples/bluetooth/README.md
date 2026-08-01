# ESP-Hosted Bluetooth examples

> [!NOTE]
> **These examples are for MCU / ESP-IDF hosts.** On a **Linux host** you do not need them:
> the ESP-Hosted kernel module exposes a standard HCI interface, so Bluetooth/BLE is handled
> directly by a normal host stack such as **BlueZ** (`bluetoothctl`, `hciconfig`) with HCI
> carried over the same SDIO/SPI transport as Wi-Fi.

## Hierarchy

```text
examples/bluetooth/<family>/<scenario>/<role>/
```

- `<family>` — how the host BT stack reaches the co-processor controller:
  - `esp_hosted_nimble` / `esp_hosted_bluedroid` — **hosted HCI**: VHCI carried
    over the ESP-Hosted transport (SDIO / SPI / UART), using ESP-Hosted's
    built-in BT stack port. This is the common case.
  - `nimble_uart` / `bluedroid_uart` — **standard HCI**: plain H4 HCI over a
    dedicated UART to the controller (no ESP-Hosted RPC channel). For comparison
    / bring-your-own-controller.
- `<role>` — `cp` (co-processor / controller-side firmware) and `mcu_host`
  (generic ESP-IDF MCU host). A few scenarios add extra roles (e.g. `cp_wifi`
  for a Wi-Fi + BLE coexistence controller build).

```text
examples/bluetooth/
├── esp_hosted_bluedroid/          # Bluedroid host stack, HCI over ESP-Hosted
│   ├── ble_advertise_minimal/{cp,mcu_host}
│   ├── ble_compatibility_test/{cp,mcu_host}
│   ├── ble_gatt_server/{cp,mcu_host}
│   ├── bt_hid_mouse/{cp,mcu_host}
│   ├── classic_bt_discovery/{cp,mcu_host}
│   └── controller_mac_addr/{cp,mcu_host}
├── esp_hosted_nimble/             # NimBLE host stack, HCI over ESP-Hosted
│   ├── bleprph_gatt/{cp,mcu_host}
│   ├── bleprph_minimal/{cp,mcu_host}
│   └── bleprph_wifi_coex/{cp,mcu_host}
├── bluedroid_uart/                # Bluedroid over plain H4 UART (no ESP-Hosted)
│   └── bluedroid_host_only_uart/{cp,mcu_host}
└── nimble_uart/                   # NimBLE over plain H4 UART (no ESP-Hosted)
    └── bleprph_host_only_uart/{cp,cp_wifi,mcu_host}
```

## Enabling the hosted-HCI port (Kconfig, no separate component)

For the `esp_hosted_*` families the BT stack port is **built into the
`esp_hosted` component** — there is no external bridge component to add. A host
app just turns it on in Kconfig:

```ini
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y            # HCI byte-pipe over the transport
CONFIG_ESP_HOSTED_HOST_BT_PORT_NIMBLE=y     # …or _BLUEDROID for the Bluedroid port
```

- **NimBLE** auto-inits at boot (`..._BT_PORT_NIMBLE_AUTO_INIT`, default on) —
  the example calls **no** ESP-Hosted setup function; it is pure NimBLE.
- **Bluedroid** must attach its HCI driver *before* `esp_bluedroid_init()`, a
  synchronous ordering auto-init can't guarantee, so Bluedroid examples set
  `..._BT_PORT_BLUEDROID_AUTO_INIT=n` and call `eh_host_bluedroid_init()`
  explicitly (or wire the `hosted_hci_bluedroid_*` driver ops themselves — see
  `controller_mac_addr`). To port a different BT stack, see
  [docs/design/bluetooth.md](../../docs/design/bluetooth.md#porting-a-bt-stack-to-esp-hosted).

The shared `esp_hosted_examples_common` helper (used by the Wi-Fi-coex example)
still lives in `examples/common_components/`.

## Pairing rules

- A host build under `<family>/<scenario>/mcu_host` must be flashed against the
  `cp` build of the **same scenario directory** — the HCI path (hosted vs
  standard UART) must match on both sides.
- Bluedroid vs NimBLE is a host-stack choice only; the `cp` side is
  controller + transport-mode dependent, not stack dependent.

## Standard UART (`*_uart`) examples

- Bluedroid: `bluedroid_uart/bluedroid_host_only_uart/`
- NimBLE:    `nimble_uart/bleprph_host_only_uart/`

These `mcu_host` projects are pure upstream-IDF HCI-over-UART host apps (no
`esp_hosted` dependency). The matching `cp` firmware uses the ESP-Hosted CP
scaffolding only for controller + UART-HCI plumbing — no RPC channel is open in
this mode.

## Capability notes

- **Classic BT** examples (`classic_bt_discovery`, `bt_hid_mouse`) require
  **ESP32** as the co-processor — only ESP32 has a BR/EDR radio.
- **BLE-only** chips (C2/C3/C5/C6/C61/H2/S2/S3) work for the BLE examples,
  except **ESP32-S2**, which has no Bluetooth radio and is excluded from
  all Bluetooth builds.
