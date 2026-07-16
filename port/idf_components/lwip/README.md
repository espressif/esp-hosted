# `eh_host_extra/lwip/`

lwIP **port glue** for non-IDF hosts (NOT the full lwIP core).

## Per-port enablement

- `esp_idf`: **off** — real IDF supplies its `esp-lwip` variant natively.
- `linux_user`: **off** — Linux kernel TCP/IP supersedes; no lwIP.
- `stm32` / `zephyr`: **on** — port glue lives here; lwIP core comes
  from a separate source (system liblwip, or an external submodule).

## Upstream origin

`_upstream_mcu_stm32/host/extra_components/lwip/port/` (≈9.2k LOC of
adapter glue across `freertos/`, `linux/`, `esp32xx/`, `debug/` —
no full lwIP core).  Apps under `lwip/apps/` (ping, sntp, http,
dhcp_server) can be vendored selectively per scenario.

## Periodic-sync checklist

Per `.meta/specs/system/verified/upstream_sync.md`:

1. Diff `_upstream_mcu_stm32/host/extra_components/lwip/port/` against
   this dir.
2. Track upstream lwIP version (separate from IDF tag — lwIP has its
   own release cadence).
3. Update "last sync" line below.

**Last sync**: 2026-04-30 from upstream STM32
`host/extra_components/lwip/`.

This is a **verbatim copy** of the upstream port-glue sources — no
edits to vendored `.c` / `.h` content.  The lwIP **core** is NOT
vendored here (upstream pulls it from a sibling git submodule);
this slot only carries the OS adapter shims (`port/freertos/`,
`port/esp32xx/`, `port/linux/`, `port/hooks/`, `port/debug/`),
plus the `apps/` helpers (ping, sntp, http, dhcpserver, esp_md5).

Adaptation of the vendored sources to this repo's host-port-layer
primitives (`eh_port_*`, `EH_HOST_*` symbols from
`host/include/eh_host_master_config.h`, etc.) is deferred to a
follow-up phase; until then the slot is gated CMake-side to bare-metal
MCU port types (STM32 / Zephyr) and stays inert on `esp_idf` and
`linux_user` builds.
