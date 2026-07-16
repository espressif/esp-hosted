# `port/esp_idf_port/esp_netif/`

IDF `esp_netif` network-interface abstraction, ported per non-IDF host
(per S4 §3b matrix).  Mirrors upstream's per-backend subdir pattern
(D-ESP-NETIF-LINUX / D4): `lwip/` for STM32 / Zephyr, `linux/` for
Linux POSIX.

## Per-port enablement

- `esp_idf`: **off** — real IDF supplies `esp_netif` natively
  (CMakeLists early-returns).
- `posix` (Linux): **on, no-op stubs only** — compile only
  `linux/esp_netif_linux_stub.c` (13 LOC).  Linux owns interfaces via
  the kernel; this slot exists so unmodified IDF apps that call
  `esp_netif_init()` / `esp_netif_create_default_wifi_*` link cleanly.
  **No LWIP on Linux** (per D-ESP-NETIF-LINUX).
- `stm32` / `zephyr`: **on, lwIP-backed** — compile the top-level
  `esp_netif_{handlers,objects,defaults}.c` plus the vendored `lwip/`
  sources.

## Upstream origin

The `lwip/`-backend sources are a verbatim copy of upstream
`_upstream_mcu_stm32/host/extra_components/esp_netif/` (≈9.6k LOC, 30
files: top-level `.c` files, `include/`, `lwip/`, `vfs_l2tap/`).
Depends on `eh_host_extra_log`, `eh_host_extra_esp_event`, and
`eh_host_extra_lwip` when assembled for STM32 / Zephyr.

The `linux/esp_netif_linux_stub.c` no-op backend is in-tree —
small, self-contained, no upstream counterpart (since upstream lwIP
backends always include lwIP).

## Periodic-sync checklist

Per the project's upstream-sync cadence (twice per month — see
`.meta/specs/system/verified/upstream_sync.md`):

1. Diff the upstream STM32 source against the contents of this dir
   (`diff -r _upstream_mcu_stm32/host/extra_components/esp_netif/ ./`).
2. If new code is in the upstream stable IDF tag, list deltas, decide
   whether to lift.
3. Update this README's "last sync" line below.
4. Commit lifted sources with attribution comment.

**Last sync**: 2026-04-30 from upstream STM32
`host/extra_components/esp_netif/`.

This is a **verbatim copy** of the upstream sources — no edits to
vendored `.c` / `.h` content.  Adaptation of the vendored sources to
this repo's host-port-layer primitives (`eh_port_*`, `EH_HOST_*`
symbols from `host/include/eh_host_master_config.h`, etc.) is deferred
to a follow-up phase; until then the slot is gated CMake-side to
bare-metal MCU port types (STM32 / Zephyr) and stays inert on
`esp_idf` and `linux_user` builds.
