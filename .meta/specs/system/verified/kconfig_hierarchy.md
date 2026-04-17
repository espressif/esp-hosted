<!-- %% sp.sy.ve-kh.o %% - always -->
---
type: spec
last_verified: 2026-03-31
---

# Kconfig & CMake Hierarchy

<!-- %% sp.sy.ve-kh.kc.o %% - always -->
## Kconfig Hierarchy

```
root Kconfig (repo root)
  └── source modules/coprocessor/Kconfig.ext
        ├── orsource eh_cp_core/Kconfig.ext
        ├── orsource eh_cp_transport/Kconfig.ext
        ├── orsource extensions/eh_cp_feat_rpc/Kconfig.ext
        └── menu "Extensions"
              ├── orsource extensions/eh_cp_feat_rpc_ext_linux/Kconfig.ext
              ├── orsource extensions/eh_cp_feat_rpc_ext_mcu/Kconfig.ext
              ├── orsource extensions/eh_cp_feat_* / Kconfig.ext
              └── orsource extensions/eh_cp_feat_peer_data/Kconfig.ext
```

Rules:
- Each extension defines `CONFIG_ESP_HOSTED_CP_EXT_*` and its `_READY` gate.
- `_READY` defaults `y` only when the extension and its dependencies are enabled.
- CP source uses `EH_CP_*` macros mapped from `CONFIG_ESP_HOSTED_CP_*`.

<!-- %% sp.sy.ve-kh.kc.c %% -->

<!-- %% sp.sy.ve-kh.cm.o %% - always -->
## CMake Hierarchy

```
modules/coprocessor/eh_cp_core            ← core library
modules/coprocessor/eh_cp_transport       ← transport library
modules/coprocessor/features/eh_cp_feat_rpc     ← RPC base feature
modules/coprocessor/features/eh_cp_feat_*       ← all features
```

Rules:
- Each extension CMakeLists uses `_READY` to build a real library or an interface stub.
- Extensions link against `eh_cp_core` (and `eh_cp_feat_rpc` where needed).
- Each extension registers itself with `EH_CP_FEAT_REGISTER`.

<!-- %% sp.sy.ve-kh.cm.c %% -->

<!-- %% sp.sy.ve-kh.c %% -->
