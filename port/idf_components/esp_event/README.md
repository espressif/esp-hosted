# `eh_host_extra/esp_event/`

IDF `esp_event` runtime port (default event loop, post / handler
registration), pthread-backed on Linux user-space, FreeRTOS-backed
on bare-metal MCU.

## Per-port enablement

- `esp_idf`: **off** — real IDF supplies it natively.
- `linux_user`: **on** — pthread backend.
- `stm32` / `zephyr`: **on** — FreeRTOS backend.

## Status

✅ Vendored / hand-written (linux_user backend functional today).

Files:
- `src/esp_event_loop.c` — pthread-backed default-loop runtime.
- `src/esp_event_handler.c` — handler registration + dispatch.
- `src/esp_event_base_decls.c` — `WIFI_EVENT` / `IP_EVENT` definitions
  for non-IDF ports (where neither `esp_wifi` nor `esp_netif` is in
  scope to provide them).

## Upstream origin

`_upstream_mcu_stm32/host/extra_components/esp_event/` (≈4.8k LOC) is
the canonical IDF-flavoured port for non-IDF MCUs.  Our linux_user
backend is hand-written (pthread), not lifted.

## Periodic-sync checklist

Per `.meta/specs/system/verified/upstream_sync.md`:

1. Diff against `_upstream_mcu_stm32/host/extra_components/esp_event/`
   (FreeRTOS-flavour) AND IDF's `components/esp_event/` (canonical
   semantics).
2. Pull semantic changes (e.g. new API in handler-instance variants).
3. Update "last sync" line below.

**Last sync**: 2026-04-27 (P5/P6 phase landing — initial Tier-1 subset).
