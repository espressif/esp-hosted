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
  - `esp_hosted_custom` — **hosted HCI**, but **bring-your-own stack**: bind your
    own raw-HCI handler to the byte-pipe (no NimBLE / Bluedroid).
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
├── esp_hosted_custom/             # bring-your-own stack, raw HCI over ESP-Hosted
│   └── hci_smoke/{cp,mcu_host}
├── bluedroid_uart/                # Bluedroid over plain H4 UART (no ESP-Hosted)
│   └── bluedroid_host_only_uart/{cp,mcu_host}
└── nimble_uart/                   # NimBLE over plain H4 UART (no ESP-Hosted)
    └── bleprph_host_only_uart/{cp,cp_wifi,mcu_host}
```

## Enabling the hosted BT adapter (Kconfig + one setup call)

For the `esp_hosted_*` families the BT stack adapter lives in
`examples/common_components/esp_hosted_bt_host_stack` and is built into the `esp_hosted`
component — there is no external bridge component to add. A host app enables the
BT feature plus its chosen IDF BT stack in Kconfig:

```ini
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y   # HCI byte-pipe over the transport
CONFIG_BT_NIMBLE_ENABLED=y         # …or CONFIG_BT_BLUEDROID_ENABLED for Bluedroid
```

The host stack is chosen by the IDF BT Kconfig above — no separate hosted
BT-port switch, and nothing auto-starts. Once, after
`esp_hosted_connect_to_slave()` and before the stack's own init, the app binds
the stack to the hosted HCI:

```c
#include "esp_hosted_bt_host_stack.h"
esp_hosted_bt_host_stack_cfg_t bt = ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_hosted_bt_host_stack_setup(&bt));  // controller up + HCI bound
```

`esp_hosted_bt_host_stack_setup()` brings the co-processor controller up and wires
the selected stack (NimBLE / Bluedroid / a custom stack) to the HCI byte-pipe;
the app then runs its own stack lifecycle (`esp_bluedroid_init/enable` or
`nimble_port_init`). To port a different BT stack, see
[Porting a BT stack to ESP-Hosted](https://github.com/espressif/esp-hosted/blob/master/docs/design/bluetooth.md#porting-a-bt-stack-to-esp-hosted).

The shared `esp_hosted_examples_common` helper (used by the Wi-Fi-coex example)
also lives in `examples/common_components/`.

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
