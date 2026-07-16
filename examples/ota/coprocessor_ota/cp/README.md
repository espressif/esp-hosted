# ota/coprocessor_ota · mcu · cp — coprocessor firmware

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

Coprocessor firmware that the [ota/coprocessor_ota](../mcu_host/) MCU
host demo writes a new image to.  Brings up ESP-Hosted CP core +
V1_MCU RPC; the OTA feature responds to
`Req_OtaBegin/Write/End/Activate` and writes the streamed bytes to
the inactive partition.

## Build & flash

From this `cp/` directory:

```bash
eh.py set-target <chip>       # a target from Supported Targets above
eh.py menuconfig              # optional: pick the transport (SPI / SDIO / UART)
eh.py -p <cp_usb_serial_port> flash monitor
```
