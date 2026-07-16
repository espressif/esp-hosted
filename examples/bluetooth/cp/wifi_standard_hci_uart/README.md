# Bluetooth+Wi-Fi CP: wifi_standard_hci_uart (direct UART H4 bypass)

Use this CP firmware with host examples under:

- `examples/bluetooth/host_mcu/*/examples/standard_hci_uart/*`
- Wi-Fi host examples that use ESP-Hosted transport/RPC.

This profile enables Wi-Fi + BT on CP and configures BT controller for UART H4
mode. HCI does not go through ESP-Hosted bridge components in host examples.

Build:

```bash
eh.py set-target <cp_chip>
eh.py build
```
