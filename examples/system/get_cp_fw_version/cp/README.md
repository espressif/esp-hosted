# system/get_cp_fw_version · mcu · cp — coprocessor firmware

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

Coprocessor firmware for the
[system/get_cp_fw_version](../mcu_host/) MCU host demo.  Brings up
ESP-Hosted CP core + V1_MCU RPC; the system feature responds to the
host's `Req_GetCoprocessorFwVersion` RPC with the running firmware
version.

The same image serves the linux_802_3 sibling — only the sdkconfig
differs (`CONFIG_ESP_HOSTED_CP_FOR_LINUX_802_3=y` vs
`CONFIG_ESP_HOSTED_CP_FOR_MCU=y`).

## Build & flash

From this `cp/` directory:

```bash
eh.py set-target <chip>       # a target from Supported Targets above
eh.py menuconfig              # optional: pick the transport (SPI / SDIO / UART)
eh.py -p <cp_usb_serial_port> flash monitor
```
