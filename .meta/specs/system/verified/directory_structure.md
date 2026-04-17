<!-- %% sp.sy.ve-ds.o %% - context -->
---
type: spec
last_verified: 2026-04-02
---

# Directory Structure

<!-- %% sp.sy.ve-ds.current.o %% - context -->
## Current (V1)

```
modules/
  ├── common/
  │   ├── eh_common/        ← Wire caps, interface types
  │   ├── eh_frame/         ← Header v1/v2 ops
  │   ├── eh_mempool/       ← Zero-copy buffer pool
  │   └── serializers/              ← Protobuf schemas + generated code
  │       ├── eh_proto_linux_v1/
  │       ├── eh_proto_mcu_v1/
  │       └── third_party/msg_codec/
  ├── coprocessor/
  │   ├── eh_cp_core/       ← Registries, auto-init, master_config
  │   ├── eh_cp_transport/  ← SPI/SDIO/UART/SPI-HD drivers
  │   ├── Kconfig.ext               ← CP Kconfig entry point
  │   ├── CMakeLists.txt            ← Extension wiring + cross-deps
  │   └── extensions/
  │       ├── eh_cp_feat_rpc/          ← RPC base (protocomm, Registry 3)
  │       ├── eh_cp_feat_rpc_ext_linux/       ← Linux FG RPC adapter
  │       ├── eh_cp_feat_rpc_ext_mcu/      ← MCU RPC adapter
  │       ├── eh_cp_feat_wifi/    ← WiFi feature
  │       ├── eh_cp_feat_bt/      ← BT feature
  │       ├── eh_cp_feat_system/  ← System (OTA, heartbeat, FW ver)
  │       ├── eh_cp_feat_nw_split/ ← Network split
  │       ├── eh_cp_feat_host_ps/ ← Host power save
  │       ├── eh_cp_feat_cli/     ← CLI
  │       └── eh_cp_feat_peer_data/
  └── host/                          ← (future — post V1)
fg/cp/examples/                      ← Coprocessor examples
  ├── cp_mcu/minimal/{wifi,bt}
  └── cp_linux_fg/minimal/{wifi,bt}
```
<!-- %% sp.sy.ve-ds.current.c %% -->

<!-- %% sp.sy.ve-ds.target.o %% - context -->
## Target (Post V1 — with port layer)

```
port/
  ├── cp/
  │   ├── interface/                 ← OSAL + HAL + MEM contracts
  │   ├── idf/                       ← FreeRTOS impl (zero overhead)
  │   ├── posix/                     ← Linux impl (testing)
  │   └── mock/                      ← CMock stubs (unit tests)
  └── host/
      ├── interface/                 ← OSAL + HAL contracts
      ├── idf/                       ← ESP-as-MCU-host
      ├── stm32/                     ← STM32 port
      └── linux/                     ← Linux user-space port
components/                          ← Business logic (= current modules/)
  ├── common/
  ├── coprocessor/
  └── host/
tests/                               ← Centralized test controller
tools/ci/                            ← CI-agnostic build/test scripts
```

See `.meta/roadmap/port_layer_and_testing.md` for full design.
<!-- %% sp.sy.ve-ds.target.c %% -->

<!-- %% sp.sy.ve-ds.c %% -->
