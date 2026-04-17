<!-- %% sp.sy.ve-ar.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# Architecture

<!-- %% sp.sy.ve-ar.top.o %% - always -->
## System Topology

```
┌─────────────────────────────────────────────────┐
│  HOST                                           │
│  ┌───────────────┐   ┌─────────────────────┐   │
│  │ Linux kmod    │   │  MCU IDF host       │   │
│  │ (FG variant)  │   │  (MCU variant)      │   │
│  │ esp_sdio.c    │   │  transport_drv.c    │   │
│  │ esp_spi.c     │   │  spi_drv / sdio_drv │   │
│  └───────┬───────┘   └──────────┬──────────┘   │
└──────────┼──────────────────────┼──────────────┘
           │  SPI / SDIO / UART / SPI-HD / Socket(test)
┌──────────┼──────────────────────────────────────┐
│  CP (IDF / FreeRTOS)                            │
│  ┌───────┴──────────────────────────────────┐   │
│  │  eh_transport_cp                 │   │
│  │  generate_startup_event() → TLV stream   │   │
│  └──────────────────┬───────────────────────┘   │
│  ┌──────────────────▼───────────────────────┐   │
│  │  eh_cp_core                      │   │
│  │    Registry 1: iface RX/TX table         │   │
│  │    Registry 2: capability accumulator    │   │
│  │    Registry 3: RPC dynamic-table         │   │
│  └──────────────────┬───────────────────────┘   │
│  ┌──────────────────▼───────────────────────┐   │
│  │  Extensions (auto-init via linker)        │   │
│  │  eh_cp_feat_rpc (priority 100)       │   │
│  │  eh_cp_feat_* (priority 200+)  │   │
│  └──────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
```
<!-- %% sp.sy.ve-ar.top.c %% -->

<!-- %% sp.sy.ve-ar.paths.o %% - always -->
## Key Paths

| Component | Actual Path |
|-----------|-------------|
| CP core | `modules/coprocessor/eh_cp_core/` |
| CP transport | `modules/coprocessor/eh_cp_transport/` |
| CP features | `modules/coprocessor/features/` |
| Common (caps, interface, mempool) | `modules/common/` |
| Linux kmod | `modules/host/linux/` |
| MCU host | `modules/host/mcu/` |
| Examples (FG) | `fg/cp/examples/cp_linux_fg/` |
| Examples (MCU) | `fg/cp/examples/cp_mcu/` |
| Kconfig entry (CP) | `modules/coprocessor/Kconfig.ext` |
| CMakeLists entry | `modules/CMakeLists.txt` |

**Source link mechanism**: Examples use `idf_component.yml.tbd` with `override_path: ../../../../../../../..` pointing to repo root, which is registered as the `esp_hosted` IDF component via `modules/CMakeLists.txt`.
<!-- %% sp.sy.ve-ar.paths.c %% -->

<!-- %% sp.sy.ve-ar.lay.o %% - context -->
## Extension Init Layer Order

1. **Transport CP** — hardware init, generates startup event with TLV caps
2. **Core** — registries init; waits for `ESP_HOSTED_CP_EVT_PRIV_INIT`
3. **`_rpc_` extensions** (priority 100) — register RPC table entries only
4. **`_feat_` extensions** (priority 200+) — spawn tasks, register caps

`EH_CP_FEAT_REGISTER(&desc)` places descriptor in `.eh_cp_ext_descs` linker section. Core iterates on boot, calls each `init()` in priority order.
<!-- %% sp.sy.ve-ar.lay.c %% -->

<!-- %% sp.sy.ve-ar.host.o %% - context -->
## Host Variants

| Variant | Build system | RPC path | Transport drivers |
|---------|-------------|----------|------------------|
| Linux FG | kernel module + user-space daemon | `ctrl_api.c` → serial_if → PRIV | SDIO, SPI (kernel), Socket(test) |
| MCU | IDF FreeRTOS | `eh_host_rpc_lib` → transport | SPI, SDIO, UART, SPI-HD, Socket(test) |

Both variants share the same CP firmware. CP detects host type from TLV negotiation.
<!-- %% sp.sy.ve-ar.host.c %% -->

<!-- %% sp.sy.ve-ar.c %% -->
