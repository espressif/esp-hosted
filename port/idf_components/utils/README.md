# `eh_host_extra/utils/`

IDF cross-cutting utility headers + helpers, ported per non-IDF host.

## Per-port enablement

- `esp_idf`: **off** — real IDF supplies them natively.
- `linux_user` / `stm32` / `zephyr`: **on** — vendor sources here.

## Upstream origin

`_upstream_mcu_stm32/host/extra_components/utils/` (≈674 LOC, 8 files —
mostly headers, plus `esp_random.c`).  Files: `esp_err.h`,
`esp_random.h` + `.c`, `esp_mac.h`, `esp_compiler.h`, `esp_assert.h`,
`esp_check.h`, `esp_macros.h`.  Standard IDF cross-cutting definitions;
no external deps.

## Periodic-sync checklist

Per `.meta/specs/system/verified/upstream_sync.md`:

1. Diff `_upstream_mcu_stm32/host/extra_components/utils/` against this dir.
2. Track upstream IDF tags for each header's home component.
3. Update "last sync" line below.

**Last sync**: NOT YET — slot reserved, no sources vendored.
