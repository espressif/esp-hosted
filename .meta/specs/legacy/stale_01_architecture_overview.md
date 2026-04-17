# 01 — Architecture Overview
<!-- Last updated: 2026-03-11 Session 15 — post-team-review design (spec 14) -->

## System Topology

```
┌──────────────────────────────────────────────────────────┐
│  HOST                                                    │
│  ┌───────────────┐   ┌──────────────────────────────┐   │
│  │ Linux kmod    │   │  MCU IDF host                │   │
│  │ (FG variant)  │   │  (MCU variant)               │   │
│  │ esp_spi.c     │   │  transport_drv.c             │   │
│  │ esp_sdio.c    │   │  spi_drv.c / sdio_drv.c      │   │
│  └───────┬───────┘   └─────────────┬────────────────┘   │
│          │  SPI / SDIO / UART / SPI-HD / Socket(test)   │
└──────────┼─────────────────────────────────────────────┘
           │
┌──────────┼─────────────────────────────────────────────┐
│  CP      │  (IDF / FreeRTOS)                          │
│  ┌───────┴───────────────────────────────────────┐    │
│  │  esp_hosted_cp_transport                      │    │
│  │  (SPI / SDIO / UART / SPI-HD / Socket-test)   │    │
│  │    generate_startup_event() — emits TLVs      │    │
│  └──────────────────────┬────────────────────────┘    │
│                         │                              │
│  ┌──────────────────────▼────────────────────────┐    │
│  │  esp_hosted_cp_core                           │    │
│  │    Registry 1: iface RX/TX table              │    │
│  │    Registry 2: capability accumulator         │    │
│  │    Registry 3: RPC dynamic table dispatch     │    │
│  └───────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────┘
```

## Layer Diagram

```
┌─────────────────────────────────────────────────────────────┐
│  Extensions  (esp_hosted_cp_ext_rpc_* / _feat_*)            │
│  Self-register via EH_CP_EXT_REGISTER linker section        │
│  Auto-init task calls init_fn sorted by priority            │
│                                                             │
│  Priority 5:   esp_hosted_cp_ext_rpc  ← protocomm init     │
│  Priority 100: esp_hosted_cp_ext_rpc_mcu / _rpc_fg          │
│  Priority 200: esp_hosted_cp_ext_feat_*                     │
├─────────────────────────────────────────────────────────────┤
│  cp_core  (three registries; NO protocomm dependency)       │
│  Registry 1: iface RX/TX   Registry 2: cap bits             │
│  Registry 3: flat realloc table, binary search              │
│  Serial I/O bridge (serial_write/read, fragment reassembly) │
├─────────────────────────────────────────────────────────────┤
│  protocomm  (TLV framing) — owned by ext_rpc ONLY          │
├─────────────────────────────────────────────────────────────┤
│  transport_cp  (SPI/SDIO/UART/SPI-HD/Socket-test)           │
├─────────────────────────────────────────────────────────────┤
│  Common  (components/common/ — config-neutral)              │
│  esp_hosted_common / esp_hosted_frame / serializers         │
└─────────────────────────────────────────────────────────────┘
```

> **Note:** `protocomm` is a dependency of `esp_hosted_cp_ext_rpc` only.
> `esp_hosted_cp_core` has zero protocomm dependency — it calls into
> `ext_rpc` via a 4-function public API. See spec 17 for full design.

## Extension Auto-Init Flow

Extensions self-register at link time using `EH_CP_EXT_REGISTER`. No Kconfig
ladder exists in core. If an extension's CMake component is not enabled its
translation unit is not compiled — zero flash/RAM cost.

```
esp_hosted_cp_core_init()
  ├─ creates ext_init_task  (runs post-scheduler)
  └─ creates transport_init_task  (blocked on s_ext_init_done)

ext_init_task (sorted ascending by priority):
  ├─ [priority 5]  esp_hosted_cp_ext_rpc::init_fn()
  │   └─ protocomm init + endpoint registration  ← PROTOCOMM READY
  ├─ [priority 100] esp_hosted_cp_ext_rpc_mcu::init_fn()
  │   └─ registers RPC table entries + WiFi event subscribers  (safe)
  ├─ [priority 100] esp_hosted_cp_ext_rpc_fg::init_fn()
  │   └─ registers RPC table entries + WiFi event subscribers  (safe)
  ├─ [priority 200] esp_hosted_cp_ext_feat_*::init_fn()  (host_ps, nw_split, ...)
  ├─ sets EH_CP_EXT_INIT_DONE_BIT
  └─ vTaskDelete(NULL)

host_reset_task:
  ├─ waits on EH_CP_EXT_INIT_DONE_BIT
  ├─ calls generate_startup_event()  ← all cap bits accumulated by now
  └─ proceeds with PRIV TLV handshake (host may send back ESP_PRIV_RPC_EP_ACK
       which calls esp_hosted_cp_ext_rpc_upgrade_endpoints() — no-op if
       endpoint names already match)
```

## Canonical Common Headers

All shared wire-protocol types live under
`components/common/esp_hosted_common/include/`.

| File | Contents |
|------|----------|
| `esp_hosted_common.h` | Umbrella include |
| `esp_hosted_common_header.h` | V1 payload header struct (12 B) |
| `esp_hosted_common_header_v2.h` | V2 payload header struct (20 B, magic `0xE9`) |
| `esp_hosted_common_interface.h` | `esp_hosted_if_type_t` enum |
| `esp_hosted_common_caps.h` | Capability bit definitions |
| `esp_hosted_common_tlv.h` | TLV type codes + version constants |
| `esp_hosted_common_log.h` | Log macro shims |
| `esp_hosted_common_fw_version.h` | `fw_version` struct |
| `esp_hosted_rpc_id_map_v1.h` | V1 RPC ID range constants (FG + MCU) |
| `esp_hosted_rpc_id_map_v2.h` | V2 unified RPC ID range constants |

## Config Macro Namespacing

| Layer | Source | Macros used |
|-------|--------|-------------|
| CP components | `esp_hosted_cp_master_config.h` | `EH_CP_XXX` |
| Host components | `esp_hosted_host_master_config.h` | `EH_HOST_XXX` |
| Port files only | `sdkconfig.h` / kernel Kconfig | `CONFIG_XXX` direct |
| Common components | — | None (must be config-neutral) |

## Proto Component Names

| Component | Schema | Used by |
|-----------|--------|---------|
| `esp_hosted_proto_linux_v1` | `esp_hosted_config.proto` (CtrlMsg) | `ext_rpc_fg` |
| `esp_hosted_proto_mcu_v1` | `esp_hosted_rpc.proto` (Rpc) | `ext_rpc_mcu` |
| `esp_hosted_proto_v2` | unified V2 schema (PENDING-006) | future V2 extensions |

## Repository Layout (target)

```
esp_hosted.new/
├── Kconfig / CMakeLists.txt / idf_component.yml
├── port/
│   ├── cp/                              ← [empty] Phase B placeholder
│   └── host/
│       ├── interface/                   ← OSAL + HAL abstraction headers
│       ├── idf/                         ← IDF port (EH_HOST_XXX macros)
│       └── linux/                       ← Linux userspace port
├── components/
│   ├── common/
│   │   ├── esp_hosted_common/           ← canonical wire-protocol headers
│   │   ├── esp_hosted_frame/            ← V1/V2 frame encode/decode
│   │   └── serializers/
│   │       ├── esp_hosted_proto_linux_v1/
│   │       ├── esp_hosted_proto_mcu_v1/
│   │       ├── esp_hosted_proto_v2/     ← placeholder (empty)
│   │       └── third_party/msg_codec/   ← protobuf-c submodule
│   ├── coprocessor/
│   │   ├── esp_hosted_cp_core/          ← registries + auto-init dispatcher
│   │   ├── esp_hosted_cp_transport/     ← SPI/SDIO/UART/SPI-HD drivers
│   │   └── extensions/
│   │       ├── esp_hosted_cp_ext_rpc_fg/
│   │       ├── esp_hosted_cp_ext_rpc_mcu/
│   │       ├── esp_hosted_cp_ext_feat_host_ps/
│   │       ├── esp_hosted_cp_ext_feat_nw_split/
│   │       └── esp_hosted_cp_ext_feat_custom_msg/
│   └── host/
│       ├── esp_hosted_host_core/
│       ├── esp_hosted_host_rpc/
│       ├── esp_hosted_host_queue/
│       └── extensions/  (wifi / bt / ota / host_ps / custom)
├── fg/
│   ├── coprocessor/
│   │   ├── linux/examples/network_adapter/
│   │   └── mcu/examples/minimal/
│   └── host/
│       ├── linux/kmod/
│       ├── linux/user_space/
│       └── mcu/examples/
├── test/
│   ├── transport/socket/    ← socket transport stub (CP + host, no hardware)
│   ├── unit/                ← per-component unit tests
│   └── system/              ← end-to-end system tests (socket transport)
└── ng/                      ← NG variant (unchanged)
```
