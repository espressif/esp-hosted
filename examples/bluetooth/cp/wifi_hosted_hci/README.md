# Bluetooth+Wi-Fi CP: wifi_hosted_hci (VHCI over ESP-Hosted transport)

Use this CP firmware with host examples under:

- `examples/bluetooth/host_mcu/*/examples/hosted_hci/*`
- Wi-Fi host examples that use ESP-Hosted transport/RPC.

This profile enables Wi-Fi + BT on CP. BT HCI uses the Hosted VHCI path
(HCI frames carried on Hosted transport: SPI/SDIO/SPI-HD/UART transport channel).

Build:

```bash
eh.py set-target <cp_chip>
eh.py build
```
