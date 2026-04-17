<!-- %% sp.ho.nv-mmh.o %% - context -->
---
type: spec
last_verified: 2026-04-02
---

# Host Macro Migration Map

Host-side mapping. Deferred to post-V1 (host H1–H6 phases).

## Namespace

| Layer | Prefix | Source |
|-------|--------|--------|
| Host IDF | `EH_HOST_` | Mapped from `CONFIG_ESP_HOSTED_HOST_*` via host port header |
| Host Linux kmod | `EH_HOST_` | Mapped from kernel Kconfig or direct `#define` |

## Known Host Macros (from legacy)

These will need mapping when host-side work begins:

| Area | Legacy symbols | Notes |
|------|---------------|-------|
| Transport | CONFIG_ESP_SPI_HOST_INTERFACE, CONFIG_ESP_SDIO_HOST_INTERFACE etc. | Already renamed to CONFIG_ESP_HOSTED_TRANSPORT_* in new Kconfig |
| WiFi | Host-side WiFi RPC lib symbols | Not yet in new repo |
| RPC | ctrl_api.c symbols (Linux FG) | Not yet in new repo |
| MCU Host | eh_host_rpc_lib symbols | Not yet in new repo |

## Status

Host-side implementation is deferred per `scope.md`. This doc will be populated
when host H1–H6 phases begin.

<!-- %% sp.ho.nv-mmh.c %% -->
