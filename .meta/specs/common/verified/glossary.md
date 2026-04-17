<!-- %% sp.cm.ve-gl.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# Glossary

<!-- %% sp.cm.ve-gl.ac.o %% - always -->
## Acronyms

| Term | Meaning |
|------|---------|
| CP | Coprocessor — ESP32-xx running IDF as wireless slave |
| Host | System driving the CP: Linux FG or MCU |
| FG | "Fully Grounded" — Linux kernel module host driver |
| MCU | Microcontroller host (IDF build, e.g. STM32 / ESP32-S3) |
| TLV | Type-Length-Value — boot-time PRIV frame encoding |
| PRIV | Private interface — control-plane channel between host and CP |
| RPC | Remote Procedure Call — request/response API between host and CP |
| V1 | Legacy 12-byte payload header; legacy protobuf RPC via protocomm |
| V2 | New 20-byte payload header (magic 0xE9); unified RPC namespace |
| HDR | Wire-frame payload header |
| EP | Endpoint — named protocomm channel ("RPCReq", "RPCEvt") |
| `_rpc_` ext | Lightweight extension — registers RPC table entries only |
| `_feat_` ext | Heavy extension — tasks, timers, explicit init/deinit |
| `EH_CP_FEAT_REGISTER` | Linker-section macro for extension auto-init registration |
<!-- %% sp.cm.ve-gl.ac.c %% -->

<!-- %% sp.cm.ve-gl.sym.o %% - context -->
## Key Symbols

| Symbol | File | Value | Meaning |
|--------|------|-------|---------|
| `ESP_HOSTED_HDR_VERSION_V1` | `eh_header.h` | 0x01 | 12-byte V1 header |
| `ESP_HOSTED_HDR_VERSION_V2` | `eh_header.h` | 0x02 | 20-byte V2 header |
| `ESP_HOSTED_HDR_V2_MAGIC` | `eh_header.h` | 0xE9 | Magic byte at byte[0] of V2 frames |
| `ESP_HOSTED_RPC_VERSION_V1` | `eh_tlv.h` | 0x01 | V1 RPC (protocomm-based) |
| `ESP_HOSTED_RPC_VERSION_V2` | `eh_tlv.h` | 0x02 | V2 RPC (unified schema) |
| `ESP_PRIV_EVENT_INIT` | struct field | 0x22 | Event type in `esp_priv_event.event_type` — NOT a TLV tag |
| `ESP_PRIV_RPC_VERSION` | `eh_tlv.h` | 0x22 | TLV tag for RPC version negotiation — same numeric value as above, different context |
<!-- %% sp.cm.ve-gl.sym.c %% -->

<!-- %% sp.cm.ve-gl.nm.o %% - context -->
## Naming Conventions

| Pattern | Example | Rule |
|---------|---------|------|
| Source files | `eh_cp_core.c` | prefix `eh_` + layer |
| Config macros (CP) | `EH_CP_TRANSPORT_SDIO` | `EH_CP_` prefix |
| Config macros (host) | `EH_HOST_SPI_CLK_MHZ` | `EH_HOST_` prefix |
| Extension descriptor | `EH_CP_FEAT_REGISTER(desc)` | defined in linker-section header |
| Proto components | `eh_host_rpc_lib` | host-side RPC library |
<!-- %% sp.cm.ve-gl.nm.c %% -->

<!-- %% sp.cm.ve-gl.c %% -->
