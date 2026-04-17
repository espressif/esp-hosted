<!-- %% sp.co.ve-fsc.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# Extension Scaffold

Canonical reference for creating a new `eh_cp_feat_*` extension.
Load this before implementing any new extension.

<!-- %% sp.co.ve-fsc.files.o %% - always -->
## Required Files

```
modules/coprocessor/features/eh_cp_feat_<name>/
  ├── CMakeLists.txt
  ├── Kconfig.ext
  ├── include/
  │     ├── eh_cp_feat_<name>.h          (public API, optional)
  │     └── priv/                                     (private headers, optional)
  └── src/
        └── eh_cp_feat_<name>.c          (init/deinit + EH_CP_FEAT_REGISTER)
```

**File naming rule**: All source and header files within a feature must use the
`eh_cp_feat_<name>` prefix. No legacy names (`slave_*`, `host_*`).
Additional source files: `eh_cp_feat_<name>_<qualifier>.c`
Sub-extension files use `_ext_` infix matching the dir: `eh_cp_feat_wifi_ext_dpp.c`
(inside dir `eh_cp_feat_wifi_ext_dpp/`).
(e.g. `_uart_esp32.c`, `_event_publisher.c`).

Add `orsource` entry to `modules/coprocessor/Kconfig.ext` inside the `menu "Extensions"` block.
Add to `modules/coprocessor/CMakeLists.txt` (or extension parent CMakeLists).
<!-- %% sp.co.ve-fsc.files.c %% -->

<!-- %% sp.co.ve-fsc.kconf.o %% - always -->
## Kconfig Pattern

```kconfig
# Kconfig.ext for eh_cp_feat_<name>

config ESP_HOSTED_CP_FEAT_<NAME>
    bool "Enable <name> feature"
    depends on ESP_HOSTED_CP              # or another extension if needed
    default n

config ESP_HOSTED_CP_FEAT_<NAME>_READY
    bool
    default y if ESP_HOSTED_CP_FEAT_<NAME>
```

For a sub-feature (e.g. wifi_enterprise depends on wifi):
```kconfig
config ESP_HOSTED_CP_FEAT_WIFI_ENTERPRISE
    bool "Enable WiFi Enterprise"
    depends on ESP_HOSTED_CP_FEAT_WIFI   # Kconfig gate only — no code-level awareness
    default n
```
<!-- %% sp.co.ve-fsc.kconf.c %% -->

<!-- %% sp.co.ve-fsc.cmake.o %% - context -->
## CMakeLists Pattern

```cmake
if(NOT CONFIG_ESP_HOSTED_CP_FEAT_<NAME>_READY)
    return()
endif()

set(srcs
    src/eh_cp_feat_<name>.c
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES eh_cp_core eh_common
    # Add feature-specific IDF deps here (e.g. esp_wifi, bt)
)
```
<!-- %% sp.co.ve-fsc.cmake.c %% -->

<!-- %% sp.co.ve-fsc.src.o %% - always -->
## Source Pattern

```c
#include "eh_cp_core.h"
#include "eh_common_caps.h"

static const char TAG[] = "ehcp_feat_<name>";

static esp_err_t feat_<name>_init(void)
{
    // 1. Initialize hardware/driver
    esp_err_t ret = ...;
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(ret));
        // Do NOT set cap bits on failure
        return ret;
    }

    // 2. Set capability bits ONLY on success
    eh_cp_add_feat_cap_bits(0, ESP_EXT_CAP_<NAME>);
    // eh_cp_add_feat_cap_bits_idx(ESP_HOSTED_FEAT_IDX_<NAME>, 0x1u);

    ESP_LOGI(TAG, "init ok");
    return ESP_OK;
}

static esp_err_t feat_<name>_deinit(void)
{
    // 1. Clear capability bits first
    eh_cp_clear_feat_cap_bits(0, ESP_EXT_CAP_<NAME>);
    // eh_cp_clear_feat_cap_bits_idx(ESP_HOSTED_FEAT_IDX_<NAME>, 0x1u);

    // 2. Deinitialize hardware/driver
    ...

    ESP_LOGI(TAG, "deinit ok");
    return ESP_OK;
}

EH_CP_FEAT_REGISTER(feat_<name>_init, feat_<name>_deinit,
                   "feat_<name>", tskNO_AFFINITY, 200);
```
<!-- %% sp.co.ve-fsc.src.c %% -->

<!-- %% sp.co.ve-fsc.checklist.o %% - always -->
## Post-Creation Checklist

After creating any new extension, verify:
- [ ] `Kconfig.ext` — gate + `_READY` symbol + warning comments
- [ ] `CMakeLists.txt` — STATIC/INTERFACE pattern
- [ ] Source — init/deinit + `EH_CP_FEAT_REGISTER` + cap bits
- [ ] File names — `eh_cp_feat_<name>` prefix (no legacy names)
- [ ] `modules/coprocessor/Kconfig.ext` — orsource added
- [ ] `modules/coprocessor/CMakeLists.txt` — extension + cross-extension deps
- [ ] `master_config.h` — `EH_CP_FEAT_<NAME>_READY` mapping added
- [ ] **Examples** — add `CONFIG_ESP_HOSTED_CP_FEAT_<NAME>=y` to relevant
      `sdkconfig.defaults`. Host-side example is desirable but not mandatory.
- [ ] Feature parity map updated
<!-- %% sp.co.ve-fsc.checklist.c %% -->

<!-- %% sp.co.ve-fsc.c %% -->
