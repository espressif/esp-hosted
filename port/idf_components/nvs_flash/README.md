# `eh_host_extra/nvs_flash/`

IDF `nvs_flash_init` / `nvs_flash_erase` API for hosts without a
flash partition.

## Per-port enablement

- `esp_idf`: **off** — real IDF reads/writes the NVS partition.
- `linux_user`: **on** — no-op stubs (no flash partition on Linux).
- `stm32` / `zephyr`: **deferred** — would need a non-volatile backend
  (file / EEPROM / battery-backed RAM).

## Files

- `src/nvs_flash.c` — `nvs_flash_init` and `nvs_flash_erase` return
  `ESP_OK`.
- API + error codes live at `host/compat/include/nvs_flash.h`.

## Upstream origin

Hand-written.  Production Linux apps that need real NVS-equivalent
key-value persistence would add a backing-store implementation
(sqlite / file / etc.) here.

## Periodic-sync checklist

Per `.meta/specs/system/verified/upstream_sync.md`: track IDF
`components/nvs_flash/` for new error codes referenced by upstream
examples.

**Last sync**: 2026-04-28 (initial).
