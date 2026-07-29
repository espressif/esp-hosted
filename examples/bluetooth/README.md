# ESP-Hosted Bluetooth examples

> [!NOTE]
> **These examples are for MCU / ESP-IDF hosts.** On a **Linux host** you do not need them:
> the ESP-Hosted kernel module exposes a standard HCI interface, so Bluetooth/BLE is handled
> directly by a normal host stack such as **BlueZ** (`bluetoothctl`, `hciconfig`) with HCI
> carried over the same SDIO/SPI transport as Wi-Fi.

## Hierarchy

```text
examples/bluetooth/<family>_<hci-variant>/<scenario>/<role>/
```

- `<family>`      — host BT stack: `bluedroid` or `nimble`.
- `<hci-variant>` — how HCI reaches the co-processor controller:
  - `hosted_hci`   — VHCI carried over the ESP-Hosted transport (SDIO/SPI).
  - `standard_hci` — plain H4 HCI over a dedicated UART (no RPC channel open).
- `<role>` — `cp` (co-processor / controller-side firmware) and
  `mcu_host` (generic ESP-IDF MCU host). A few scenarios add extra roles
  (e.g. `cp_wifi` for a Wi-Fi + BLE coexistence controller build).

```text
examples/bluetooth/
├── bluedroid_hosted_hci/
│   ├── ble_advertise_minimal/{cp,mcu_host}
│   ├── ble_compatibility_test/{cp,mcu_host}
│   ├── ble_gatt_server/{cp,mcu_host}
│   ├── bt_hid_mouse/{cp,mcu_host}
│   ├── classic_bt_discovery/{cp,mcu_host}
│   └── controller_mac_addr/{cp,mcu_host}
├── bluedroid_standard_hci/
│   └── bluedroid_host_only_uart/{cp,mcu_host}
├── nimble_hosted_hci/
│   ├── bleprph_gatt/{cp,mcu_host}
│   ├── bleprph_minimal/{cp,mcu_host}
│   └── bleprph_wifi_coex/{cp,mcu_host}
└── nimble_standard_hci/
    └── bleprph_host_only_uart/{cp,cp_wifi,mcu_host}
```

Shared host-side glue lives outside this tree in
`examples/common_components/` (`esp_hosted_hci_bluedroid`,
`esp_hosted_hci_nimble`, `esp_hosted_examples_common`,
`nimble_peripheral_utils`).

## Pairing rules

- A host build under `<family>_<hci-variant>/<scenario>/mcu_host` must be
  flashed against the `cp` build of the **same scenario directory** — the
  HCI variant (`hosted_hci` vs `standard_hci`) must match on both sides.
- Bluedroid vs NimBLE is a host-stack choice only; the `cp` side is
  controller + transport-mode dependent, not stack dependent.

## Standard UART (`standard_hci`) examples

- Bluedroid: `bluedroid_standard_hci/bluedroid_host_only_uart/`
- NimBLE:    `nimble_standard_hci/bleprph_host_only_uart/`

These `mcu_host` projects are pure upstream-IDF HCI-over-UART host apps
(no `esp_hosted` dependency). The matching `cp` firmware uses the
ESP-Hosted CP scaffolding only for controller + UART-HCI plumbing — no
RPC channel is open in this mode.

## Capability notes

- **Classic BT** examples (`classic_bt_discovery`, `bt_hid_mouse`) require
  **ESP32** as the co-processor — only ESP32 has a BR/EDR radio.
- **BLE-only** chips (C2/C3/C5/C6/C61/H2/S2/S3) work for the BLE examples,
  except **ESP32-S2**, which has no Bluetooth radio and is excluded from
  all Bluetooth builds.
