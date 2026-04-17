# 05 — Configuration Abstraction (EH_CP_XXX / EH_HOST_XXX Macros)
<!-- Last updated: 2026-03-11 Session 15 — extended to cover CP side -->

## 1. Problem

Three distinct config systems exist across the codebase:
- IDF host/CP code uses `CONFIG_XXX` (sdkconfig) symbols.
- Linux kernel module uses kernel Kconfig or direct `#define`.
- Common components need to compile against both without including either.

## 2. Solution — Port Headers with Namespaced Macros

### CP side
```
components/coprocessor/esp_hosted_cp_core/include/esp_hosted_cp_master_config.h
```
Maps `CONFIG_ESP_HOSTED_CP_XXX` → `EH_CP_XXX`.
All CP component source files include this header (via `esp_hosted_cp_core.h` umbrella).
No CP source file outside `port/` uses `CONFIG_` symbols directly.

### Host side (unchanged from Phase 4)
```
port/host/idf/esp_hosted_host_port_osal_idf/include/esp_hosted_host_master_config.h
port/host/linux/esp_hosted_host_port_osal_linux/include/esp_hosted_host_master_config.h
```
Maps `CONFIG_ESP_HOSTED_HOST_XXX` → `EH_HOST_XXX`.

## 3. Macro Namespace Rules

| Layer | Macros used | Source of truth |
|-------|-------------|-----------------|
| `components/coprocessor/` sources | `EH_CP_XXX` | `esp_hosted_cp_master_config.h` |
| `components/host/` sources | `EH_HOST_XXX` | `esp_hosted_host_master_config.h` |
| `port/` files | `CONFIG_XXX` direct | sdkconfig / kernel Kconfig |
| `components/common/` sources | **none** | must be config-neutral |

## 4. EH_CP_XXX Mapping Examples

```c
/* esp_hosted_cp_master_config.h */
#pragma once
#include "sdkconfig.h"

#ifdef CONFIG_ESP_HOSTED_CP_WIFI_ENABLED
  #define EH_CP_WIFI_ENABLED     1
#else
  #define EH_CP_WIFI_ENABLED     0
#endif

#ifdef CONFIG_ESP_HOSTED_COPROCESSOR_BT_ENABLED
  #define EH_CP_BT_ENABLED       1
#else
  #define EH_CP_BT_ENABLED       0
#endif

#ifdef CONFIG_ESP_SPI_HOST_INTERFACE
  #define EH_CP_TRANSPORT_SPI    1
#else
  #define EH_CP_TRANSPORT_SPI    0
#endif
/* ... one entry per CONFIG_ESP_HOSTED_CP_* symbol */
```

## 5. EH_HOST_XXX Mapping Table (from Phase 4)

| H_XXX / EH_HOST_XXX macro | IDF source | Linux value | Notes |
|---------------------------|------------|-------------|-------|
| `EH_HOST_PLATFORM_MCU` | `CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU` | not defined (0) | Selects MCU vs FG host path |
| `EH_HOST_DFLT_TASK_STACK` | `CONFIG_ESP_HOSTED_DFLT_TASK_STACK` | 3072 | Default FreeRTOS task stack |

> **Note:** The old `H_XXX` prefix (from Phase 4) is unified to `EH_HOST_XXX`
> in Phase E. Both forms are accepted during the transition.

## 6. Excluded from Remapping

| Symbol | Reason |
|--------|--------|
| `CONFIG_IDF_TARGET` | IDF built-in, not a hosted config; stays as-is in MCU code |
| `CONFIG_H_LOWER_MEMCOPY` | Inside `#if 0` dead code block; no change |
| `CONFIG_ESP_HOSTED_USE_WORKQUEUE` | Linux kernel Kconfig for kmod; Linux port header bridges via passthrough `#define` |

