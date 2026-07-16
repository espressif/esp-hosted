# wifi/sta · mcu · host — ESP-IDF MCU Wi-Fi station host app

Drop-in IDF Wi-Fi getting-started station example, running on an ESP
MCU **host** that delegates Wi-Fi to a separate ESP **CP** over the
ESP-Hosted transport.

The `main/main.c` here uses the IDF `esp_wifi_*` API unmodified;
`esp_wifi_remote` routes calls to the CP via RPC.

## What it exercises

Canonical Wi-Fi station bring-up:

- `nvs_flash_init()`
- `esp_netif_init()` + `esp_event_loop_create_default()` + `esp_netif_create_default_wifi_sta()`
- `esp_wifi_init(&cfg)` -> `esp_wifi_set_mode(WIFI_MODE_STA)`
  -> `esp_wifi_set_config(...)` -> `esp_wifi_start()`

- Event-loop wait on `WIFI_EVENT` + `IP_EVENT_STA_GOT_IP`,
  `esp_wifi_connect()` on `WIFI_EVENT_STA_START`.

## Required hardware

- One ESP MCU running this **host** firmware (esp32s3 / esp32p4
  per `idf_component.yml` `targets:`).

- One ESP MCU running the matching **CP** firmware
  (see `../cp/` for pairing notes).

- SPI wiring per `host/mcu/eh_host_mcu_transport/Kconfig.host.spi`.

## Build

```sh
cd examples/wifi/sta/mcu_host
eh.py set-target esp32p4    # or esp32s3
eh.py build
eh.py -p <host_usb_serial_port> flash monitor
```

## Configure

Set `CONFIG_ESP_WIFI_SSID` / `CONFIG_ESP_WIFI_PASSWORD` via
`eh.py menuconfig` (Example Configuration), or override in
`sdkconfig.defaults`.
