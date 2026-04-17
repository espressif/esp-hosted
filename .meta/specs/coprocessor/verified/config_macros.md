<!-- %% sp.co.ve-cf.o %% - always -->
---
type: spec
last_verified: 2026-03-31
---

# Configuration Macros

<!-- %% sp.co.ve-cf.ns.o %% - always -->
## Namespace Rules

| Layer | Macro prefix | Function prefix | Component prefix | Source |
|-------|-------------|----------------|-----------------|--------|
| CP source | `EH_CP_` | `eh_cp_*` | `eh_cp_*` | Mapped from `CONFIG_ESP_HOSTED_CP_*` via port header |
| Host (IDF) | `EH_HOST_` | `eh_host_*` | `eh_host_*` | Mapped from `CONFIG_ESP_HOSTED_HOST_*` via port header |
| Host (Linux kmod) | `EH_HOST_` | `eh_host_*` | `eh_host_*` | Mapped from kernel Kconfig or direct `#define` |
| Common/wire | `EH_*` | `eh_*` | `eh_common`, `eh_frame`, `eh_tlv` | Shared protocol layer |

**Component naming:**

| Old name | New name | Role |
|----------|----------|------|
| `eh_transport_host` | `eh_host_transport` | Host transport HAL |
| `eh_rpc_lib` | `eh_host_rpc_lib` | Host RPC API |
| `esp_hosted_cp_lin_fg_pbuf` (old) | `eh_proto_linux_v1` | Linux FG protobuf codec |

No CP source file outside `port/` or extension cfg headers uses `CONFIG_` symbols directly.

`EH_CP_*` booleans are always defined as 0 or 1 (never undefined).
Use `#if EH_CP_X`, never `#if defined(EH_CP_X)` or `#ifdef EH_CP_X`.
<!-- %% sp.co.ve-cf.ns.c %% -->

<!-- %% sp.co.ve-cf.ports.o %% - context -->
## Port Header Locations

| Side | Path |
|------|------|
| CP | `modules/coprocessor/eh_cp_core/include/eh_cp_master_config.h` |
| Host IDF | Host repo (not in this tree) |
| Host Linux | Host repo (not in this tree) |

Port headers `#include` the IDF sdkconfig or define constants directly, then `#define EH_CP_X CONFIG_ESP_HOSTED_CP_X`.

Extensions with chip/IDF-specific config create a private cfg header at
`include/priv/eh_cp_feat_<name>_cfg.h` mapping their own `CONFIG_ESP_HOSTED_*` → `EH_CP_*`.
IDF component configs (`CONFIG_BTDM_*`, `CONFIG_BT_LE_*`, etc.) stay as-is inside extensions.
<!-- %% sp.co.ve-cf.ports.c %% -->

<!-- %% sp.co.ve-cf.coll.o %% - always -->
## Identifier Collision

Never define the same identifier as both a `#define` macro and an enum value —
the preprocessor silently rewrites the enum token, producing confusing compile errors.

Rule: `#define` is canonical. If an enum needs the same conceptual value, give the
enum a distinct name and reference the macro:

```c
#define EH_CP_FOO  3
typedef enum { EH_CP_FOO_ENUM = EH_CP_FOO } eh_cp_foo_t;
```
<!-- %% sp.co.ve-cf.coll.c %% -->

<!-- %% sp.co.ve-cf.c %% -->
