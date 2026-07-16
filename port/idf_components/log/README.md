# `eh_host_extra/log/`

IDF `esp_log` impl, ported per non-IDF host (per S4 §3b matrix).

## Per-port enablement

- `esp_idf`: **off** — real IDF supplies it natively.
- `linux_user`: **on** — pthread-friendly stderr backend.
- `stm32` / `zephyr`: **on** — RTOS-friendly impl.

## Upstream origin

`_upstream_mcu_stm32/host/extra_components/log/` (≈535 LOC, 6 files —
self-contained, no external deps).  Files: `esp_log.c`, `esp_log.h`,
`log_linked_list.c`, `esp_log_level.h`, `tag_log_level.c`.

## Periodic-sync checklist

Per `.meta/specs/system/verified/upstream_sync.md`:

1. Diff `_upstream_mcu_stm32/host/extra_components/log/` against this dir.
2. Track upstream IDF stable tag for `components/log/`.
3. Update "last sync" line below.

**Last sync**: NOT YET — slot reserved, no sources vendored.
