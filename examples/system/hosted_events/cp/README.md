# system/hosted_events · mcu · cp — coprocessor firmware

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

Coprocessor firmware for the [system/hosted_events](../mcu_host/) MCU
host demo.  Brings up ESP-Hosted CP core + V1_MCU RPC; the system
feature emits the standard hosted events (`ESP_HOSTED_EVENT_CP_INIT`,
`ESP_HOSTED_EVENT_CP_HEARTBEAT`, …) over RPC for the host to subscribe
to.

## Build & flash

From this `cp/` directory:

```bash
eh.py set-target <chip>       # a target from Supported Targets above
eh.py menuconfig              # optional: pick the transport (SPI / SDIO / UART)
eh.py -p <cp_usb_serial_port> flash monitor
```
