# Bluetooth CP: hosted_hci (VHCI over ESP-Hosted transport)

Use this CP firmware with host examples under:

- `examples/bluetooth/host_mcu/*/examples/hosted_hci/*`

This profile enables ESP-Hosted BT feature with VHCI path (HCI frames carried
on Hosted transport: SPI/SDIO/SPI-HD/UART transport channel).

Build:

```bash
eh.py set-target <cp_chip>
eh.py build
```
