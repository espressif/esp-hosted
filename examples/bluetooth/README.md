# ESP-Hosted Bluetooth examples

> [!NOTE]
> **These examples are for MCU / ESP-IDF hosts.** On a **Linux host** you do not need them:
> the ESP-Hosted kernel module exposes a standard HCI interface, so Bluetooth/BLE is handled
> directly by a normal host stack such as **BlueZ** (`bluetoothctl`, `hciconfig`) with HCI
> carried over the same SDIO/SPI transport as Wi-Fi.

## Hierarchy

```text
examples/bluetooth/
├── cp/
│   ├── hosted_hci/mcu/cp/            # CP config for Hosted-HCI (VHCI)
│   └── standard_hci_uart/mcu/cp/     # CP config for direct UART H4
└── host_mcu/
    ├── bluedroid/
    │   ├── components/esp_hosted_hci_bluedroid/
    │   └── examples/
    │       ├── hosted_hci/
    │       └── standard_hci_uart/
    │           └── bluedroid_host_only_uart/
    └── nimble/
        ├── components/esp_hosted_hci_nimble/
        └── examples/
            ├── hosted_hci/
            └── standard_hci_uart/
                └── bleprph_host_only_uart/
```

## Pairing rules

- `hosted_hci` host examples must pair with `cp/hosted_hci/...` CP firmware.
- `standard_hci_uart` host examples must pair with `cp/standard_hci_uart/...` CP firmware.
- Bluedroid vs NimBLE is host-stack choice only; CP is controller-side and transport-mode dependent.

## Standard UART examples

- Bluedroid: `host_mcu/bluedroid/examples/standard_hci_uart/bluedroid_host_only_uart/`
- NimBLE: `host_mcu/nimble/examples/standard_hci_uart/bleprph_host_only_uart/`

These are inherited from upstream IDF host-only UART examples with minimal path
adaptation into this hierarchy.

> The `standard_hci_uart` host examples themselves are pure upstream-IDF
> HCI-over-UART (no `esp_hosted` dep). The matching CP firmware under
> `cp/standard_hci_uart/...` uses the ESP-Hosted CP scaffolding only for
> controller + UART-HCI plumbing — no RPC channel is open in this mode.

## Capability notes

- Classic BT examples require ESP32 as CP.
- BLE-only chips (C2/C3/C5/C6/C61/H2/S2/S3) work for BLE examples.
