# network_split/iperf · mcu · cp — coprocessor firmware

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

Coprocessor firmware for the [network_split/iperf](../esp_host/) MCU host
demo.  Brings up ESP-Hosted CP core + V1_MCU RPC; CP-side LWIP runs
the local network split (iperf endpoint owned by the CP, control flow
on the host).

## Build & flash

From this `cp/` directory:

```bash
eh.py set-target <chip>       # a target from Supported Targets above
eh.py menuconfig              # optional: pick the transport (SPI / SDIO / UART)
eh.py -p <cp_usb_serial_port> flash monitor
```
