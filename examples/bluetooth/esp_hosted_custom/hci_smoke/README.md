# Custom BT Stack — raw HCI over Hosted (`bluetooth/esp_hosted_custom/hci_smoke`)

<!-- tags: bluetooth, ble, hci, custom-stack, bring-your-own-stack, hosted-hci -->

<!-- common-start -->
A **bring-your-own BT stack** example. No NimBLE, no Bluedroid — the host binds
its **own raw-HCI handler** to the ESP-Hosted HCI byte-pipe through the **custom**
path, so you can drive any BT host stack you like over the co-processor's
controller.

It brings the **controller** up on the ESP-Hosted **co-processor**, binds a custom
`rx`/`tx` pair with one call — `esp_hosted_bt_host_stack_setup()` using
`ESP_HOSTED_BT_HOST_STACK_CONFIG_CUSTOM(rx, ctx)` — then sends an **HCI Reset** and
prints the controller's **Command-Complete** received on `rx`. That round-trip
(`tx` → CP controller → `rx`) is the proof the custom two-wire path works. Wire a
real stack's transport to the same `rx`/`tx` and you have hosted BT with your own
stack. See [Porting a BT stack to
ESP-Hosted](https://github.com/espressif/esp-hosted/blob/master/docs/design/bluetooth.md#porting-a-bt-stack-to-esp-hosted).

## Supported Platforms and Transports

### Supported Coprocessors (BT controller)

| Coprocessor | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 | ESP32-S2 |
| :---------- | :---: | :------: | :------: | :------: | :------: | :-------: | :------: | :------: | :------: |
| BLE support | Yes   | Yes      | Yes      | Yes      | Yes      | Yes       | Yes      | Yes      | No (no BLE) |

### Supported Host Devices

| Host Device | ESP32-P4 | ESP32-H2 | Other MCUs |
| :---------- | :------: | :------: | :--------: |
| Support     | Yes      | Yes      | [Yes](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md) |

### Supported HCI transports

| HCI over hosted bus (VHCI) | SDIO | SPI Full-Duplex | SPI Half-Duplex | UART |
| :------------------------- | :--: | :-------------: | :-------------: | :--: |
| MCU host                   | Yes  | Yes             | Yes             | Yes  |
<!-- common-stop -->

## Directory layout

```text
bluetooth/esp_hosted_custom/hci_smoke/
├── cp/         # co-processor: BT controller-only over hosted HCI (stack-agnostic)
└── mcu_host/   # host: custom raw-HCI handler (no NimBLE / Bluedroid)
```

## Kconfig

The host enables the BT feature but **no** IDF BT host stack — that is what
selects the custom path:

```ini
CONFIG_ESP_HOSTED_HOST_FEAT_BT=y      # HCI byte-pipe + CP controller lifecycle
# (no CONFIG_BT_NIMBLE_ENABLED / CONFIG_BT_BLUEDROID_ENABLED)
```

## Build & run

Flash `cp/` to a BLE-capable co-processor and `mcu_host/` to the host (paired to
the same transport), then watch the host log for:

```text
custom rx: HCI Reset Command Complete, status=0x00
```
