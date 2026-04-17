<!-- %% sp.co.ve-cpm.o %% - always -->
---
type: spec
last_verified: 2026-04-13
---

# Component Map

<!-- %% sp.co.ve-cpm.core.o %% - always -->
## Core Components

| IDF Component Name | Directory | Kconfig Enable | Role |
|--------------------|-----------|----------------|------|
| `eh_cp_core` | `modules/coprocessor/eh_cp_core/` | `ESP_HOSTED_CP` (always) | Registries 1+2, feature auto-init |
| `eh_cp_transport` | `modules/coprocessor/eh_cp_transport/` | `ESP_HOSTED_CP` (always) | Transport HAL, TLV startup event |
| `eh_common` | `modules/common/eh_common/` | always | Wire caps, interface types, glossary |
| `eh_frame` | `modules/common/eh_frame/` | always | Wire frame headers (V1) |
| `eh_mempool` | `modules/common/eh_mempool/` | always | Zero-copy buffer pool |
| `eh_tlv` | `modules/common/eh_tlv/` | always (header-only INTERFACE) | PRIV-event TLV pack/unpack (V1 Linux / V1 MCU / V2 groups; auto-selected from RPC version) |
<!-- %% sp.co.ve-cpm.core.c %% -->

<!-- %% sp.co.ve-cpm.ext.o %% - always -->
## Feature Components

All feature directories are under `modules/coprocessor/features/`.

| Component | Kconfig Enable | Notes |
|-----------|----------------|-------|
| `eh_cp_feat_rpc` | `ESP_HOSTED_CP_FEAT_RPC_READY` | RPC core |
| `eh_cp_feat_rpc_ext_linux` | `ESP_HOSTED_CP_FEAT_RPC_LINUX_READY` | Linux FG handlers |
| `eh_cp_feat_rpc_ext_mcu` | `ESP_HOSTED_CP_FEAT_RPC_MCU_READY` | MCU handlers |
| `eh_cp_feat_wifi` | `ESP_HOSTED_CP_FEAT_WIFI` (menuconfig) | Parent of WiFi sub-extensions |
| `eh_cp_feat_wifi_ext_ent` | `ESP_HOSTED_CP_FEAT_WIFI_EXT_ENT_READY` | WiFi Enterprise |
| `eh_cp_feat_wifi_ext_itwt` | `ESP_HOSTED_CP_FEAT_WIFI_EXT_ITWT_READY` | iTWT |
| `eh_cp_feat_wifi_ext_dpp` | `ESP_HOSTED_CP_FEAT_WIFI_EXT_DPP_READY` | DPP |
| `eh_cp_feat_bt` | `ESP_HOSTED_CP_FEAT_BT` (menuconfig) | BT/BLE |
| `eh_cp_feat_system` | `ESP_HOSTED_CP_FEAT_SYSTEM_READY` | System events/info |
| `eh_cp_feat_host_ps` | `ESP_HOSTED_CP_FEAT_HOST_PS` (menuconfig) | Host power save |
| `eh_cp_feat_nw_split` | `ESP_HOSTED_CP_FEAT_NW_SPLIT` (menuconfig) | Network split |
| `eh_cp_feat_peer_data` | `ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER` (menuconfig) | Peer data transfer |
| `eh_cp_feat_cli` | `ESP_HOSTED_CP_FEAT_CLI` (menuconfig) | CLI |
| `eh_cp_feat_gpio_exp` | `ESP_HOSTED_CP_FEAT_GPIO_EXP_READY` | GPIO expander |
| `eh_cp_feat_mem_monitor` | `ESP_HOSTED_CP_FEAT_MEM_MONITOR_READY` | Memory monitor |
| `eh_cp_feat_ext_coex` | `ESP_HOSTED_CP_FEAT_EXT_COEX_READY` | External coex |
| `eh_cp_feat_light_sleep` | `ESP_HOSTED_CP_FEAT_LIGHT_SLEEP_READY` | Light sleep (no RPC) |

WiFi sub-extensions (`_ext_ent`, `_ext_itwt`, `_ext_dpp`) use parent-managed lifecycle —
parent `eh_cp_feat_wifi` calls their init/deinit conditionally; they do NOT use
`EH_CP_FEAT_REGISTER` themselves.
<!-- %% sp.co.ve-cpm.ext.c %% -->

<!-- %% sp.co.ve-cpm.kconf.o %% - context -->
## Kconfig Chain

```
modules/coprocessor/Kconfig.ext          ← entry point
  ├─ eh_cp_core/Kconfig.ext
  ├─ eh_cp_transport/Kconfig.ext
  ├─ ../common/eh_tlv/Kconfig.ext         (hidden: TLV group select from RPC version)
  ├─ features/eh_cp_feat_rpc/Kconfig.ext
  └─ menu "Features"
       ├─ features/eh_cp_feat_rpc_ext_linux/Kconfig.ext
       ├─ features/eh_cp_feat_rpc_ext_mcu/Kconfig.ext
       ├─ features/eh_cp_feat_wifi/Kconfig.ext   (sub-ext nested via if block)
       ├─ features/eh_cp_feat_bt/Kconfig.ext
       ├─ features/eh_cp_feat_system/Kconfig.ext
       ├─ features/eh_cp_feat_host_ps/Kconfig.ext
       ├─ features/eh_cp_feat_nw_split/Kconfig.ext
       ├─ features/eh_cp_feat_peer_data/Kconfig.ext
       ├─ features/eh_cp_feat_cli/Kconfig.ext
       ├─ features/eh_cp_feat_gpio_exp/Kconfig.ext
       ├─ features/eh_cp_feat_mem_monitor/Kconfig.ext
       ├─ features/eh_cp_feat_ext_coex/Kconfig.ext
       └─ features/eh_cp_feat_light_sleep/Kconfig.ext
```

Kconfig pattern for feature `_READY` symbol:
```kconfig
config ESP_HOSTED_CP_FEAT_WIFI_READY
    bool
    default y if ESP_HOSTED_CP_FEAT_RPC_MCU && ESP_WIFI
```
CMakeLists guards on `CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY`.
<!-- %% sp.co.ve-cpm.kconf.c %% -->

<!-- %% sp.co.ve-cpm.c %% -->
