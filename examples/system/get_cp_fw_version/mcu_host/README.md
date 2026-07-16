# system/get_cp_fw_version · mcu · host — ESP-IDF MCU host app

Smallest possible MCU-host scenario: brings up the ESP-Hosted host
stack on an ESP-IDF / FreeRTOS target, calls
`eh_host_sys_get_cp_fw_version(&fw)` once every 5 seconds, and prints
the structured firmware-version result over the IDF console.

MCU-side counterpart of `../../linux_802_3_host/c_app/` — same
scenario, same RPC, different host platform (ESP MCU + SPI bus
instead of Linux user-space + `/dev/esps0`).

## What it exercises

- `eh_host_init(NULL)` — one-call bring-up of vserial_mcu → base RPC → features.
- `esp_event_handler_register(EH_HOST_EVENT, EH_HOST_EVENT_CP_INIT, ...)`
  — IDF event flow over the host stack's common base.

- `eh_host_sys_get_cp_fw_version(eh_host_coprocessor_fwver_t *)`
  — typed synchronous RPC.

## Required hardware

- One ESP MCU running this **host** firmware.
- One ESP MCU running the matching **CP** firmware (`../cp/`) — a
  second board, wired to the host over SPI.

- Default SPI pins per `host/mcu/eh_host_mcu_transport/Kconfig.host.spi`
  (CLK=18, MOSI=23, MISO=19, CS=5, HANDSHAKE=26, DATA_READY=4).
  Override in `eh.py menuconfig` for your board.

## Build

```sh
cd examples/system/get_cp_fw_version/mcu_host
eh.py set-target esp32p4    # or esp32s3
eh.py build
eh.py -p <host_usb_serial_port> flash monitor
```

## Expected output

Once the CP is up and SPI is wired correctly, the serial console prints
every 5 seconds:

```
I (xxx) get_cp_fw_version: CP firmware: 3.0.0 (revision=0, prerelease=1, build=0)
```

Before the CP responds, expect `eh_host_sys_get_cp_fw_version: 0x...`
errors (timeout / transport).  Normal during bring-up.
