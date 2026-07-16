# Bluetooth CP: standard_hci_uart (direct UART H4 bypass)

Use this CP firmware with host examples under:

- `examples/bluetooth/host_mcu/*/examples/standard_hci_uart/*`

This profile configures BT controller for UART H4 mode. HCI does not go
through ESP-Hosted bridge components in host examples.

Notes:

- Most `sdkconfig.defaults.esp32*` overlays already enable UART-H4 symbols.
- For targets where UART-H4 symbols are not pre-populated in the overlay,
  enable them in menuconfig under Bluetooth Controller HCI mode before build.

- `esp32c61` overlay follows `esp32c6` UART-H4 settings.
- `esp32s2` is explicitly unsupported for Bluetooth (no BT controller).

Build:

```bash
eh.py set-target <cp_chip>
eh.py build
```
