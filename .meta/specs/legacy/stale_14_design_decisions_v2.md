# 14 — Design Decisions V2 (Post Team Review)
<!-- Session: 2026-03-11 rev-2. Supersedes rev-1 of this file. -->
<!-- All 10 decisions finalised. Open questions resolved. -->

---

## Overview

This document captures the 10 finalised design decisions made after team review
of the old vs new architecture comparison, plus a subsequent clarification round.
The implementation plan (Phases C–F) follows at the end.

---

## Decision 1 — Extension CMake Dependencies via REQUIRES / PRIV_REQUIRES

**Decision:**
Extensions declare all their IDF component dependencies using standard IDF
`idf_component_register(REQUIRES ... PRIV_REQUIRES ...)`. Public headers a
consumer needs go in `REQUIRES`; internal-only headers go in `PRIV_REQUIRES`.

**Proto component names (corrected):**

| Old name in rev-1 | Final name |
|-------------------|-----------|
| `esp_hosted_proto_fg` | `esp_hosted_proto_linux_v1` |
| `esp_hosted_proto_mcu` | `esp_hosted_proto_mcu_v1` |
| `esp_hosted_proto_v2` | `esp_hosted_proto_v2` (placeholder, empty for now) |

All three live under `components/common/serializers/`.

**Delta:**
- Move `ext_rpc_linux_fg_pbuf/` content → `serializers/esp_hosted_proto_linux_v1/`
- Move `ext_rpc_mcu_pbuf/` content → `serializers/esp_hosted_proto_mcu_v1/`
- Delete `_pbuf` sidecar extension dirs.
- FG extension REQUIRES: `esp_hosted_proto_linux_v1`
- MCU extension REQUIRES: `esp_hosted_proto_mcu_v1`

---

## Decision 2 — Extension Auto-Init via Linker-Section Descriptor Table

### Problem statement

Requirements that must all be satisfied simultaneously:
1. Flash + RAM: only compiled features consume resources. Unused features = zero cost.
2. No explicit `#if EH_CP_WIFI_ENABLED` ladder in core to call extensions.
3. No C++ static constructors — run order is pre-scheduler, undefined relative to RTOS.
4. Execution at task level, post-scheduler, with known priority and core affinity.
5. Build-system agnostic — must not require linker script fragments, CMake tricks,
   or IDF-specific startup magic.
6. Extensions register their own capability bits in their own init (Point 3).

### Rejected approaches

**Host-driven init (host sends feature bits → CP inits matching extensions):**
Requires all extension code to be compiled in and waiting. Flash/RAM goal violated.
Also creates a boot ordering problem: CP cannot serve any RPC until the host has
connected and sent its caps — unacceptable.

**Kconfig ladder in core (`#if EH_CP_WIFI_ENABLED ... call wifi_init()`):**
Core becomes coupled to every extension. Every new extension requires editing core.
Ugly. Rejected.

**IDF `ESP_SYSTEM_INIT_FN` macro:**
Uses `esp_system`'s linker fragment. IDF-specific, cannot port to Linux userspace
or the kernel module. Rejected for portability.

### Chosen approach — `EH_CP_EXT_INIT` linker-section self-registration

Works identically to `ESP_SYSTEM_INIT_FN` in principle, but using a section name
we own. Portable to any GCC/Clang compiler target. Does not require linker script
changes — only a `__attribute__((section(...), used))` placement, which the
compiler handles automatically.

#### Descriptor struct (in `esp_hosted_cp_core.h`)

```c
typedef esp_err_t (*esp_hosted_ext_init_fn_t)(void);

typedef struct {
    esp_hosted_ext_init_fn_t init_fn;   /* extension init function                    */
    esp_hosted_ext_init_fn_t deinit_fn; /* extension deinit, NULL if not needed       */
    const char              *name;      /* human-readable name for log output         */
    int                      affinity;  /* CPU affinity: 0, 1, or tskNO_AFFINITY (-1) */
    int                      priority;  /* init order — raw integer, lower runs first */
} esp_hosted_ext_desc_t;
```

**Priority convention — follows IDF pattern exactly:**
Priority is a plain `int` argument to the macro. No named enum constants are
defined — callers pick a number directly, just as `ESP_SYSTEM_INIT_FN` takes
a raw integer for its `priority_within_stage` argument. Suggested convention
(not enforced, not defined as macros):

| Suggested value | Intended use |
|-----------------|-------------|
| 100 | Core RPC table registration (`_rpc_` extensions) |
| 200 | Feature init that depends on RPC being registered (`_feat_` extensions) |
| 300 | Late init (anything that depends on feat_ being ready) |

These are documentation hints only. The dispatcher sorts by `priority` field
ascending (lower number = runs first) and calls `init_fn` in that order.

#### Registration macro (in `esp_hosted_cp_core.h`)

```c
/*
 * EH_CP_EXT_REGISTER(init_fn, deinit_fn, name_str, affinity, priority)
 *
 * Places a descriptor into the .eh_cp_ext_descs linker section.
 * The compiler includes this record if and only if the translation unit
 * is compiled (i.e. if the extension's CMake component is enabled).
 * Zero flash/RAM cost for disabled extensions — their sources are simply
 * not compiled, so no record is emitted.
 *
 * affinity: tskNO_AFFINITY (-1), 0 (PRO_CPU), or 1 (APP_CPU)
 * priority: plain integer — lower runs first. Follow suggested values in
 *           the descriptor struct documentation above.
 *
 * 'used' prevents the compiler from discarding the symbol as unreferenced.
 * 'aligned(4)' ensures consistent struct layout across all entries.
 */
#define EH_CP_EXT_REGISTER(_init, _deinit, _name, _affinity, _prio)         \
    static const esp_hosted_ext_desc_t                                       \
    __esp_hosted_ext_desc_##_init                                            \
    __attribute__((section(".eh_cp_ext_descs"), used, aligned(4))) = {       \
        .init_fn   = (_init),                                                \
        .deinit_fn = (_deinit),                                              \
        .name      = (_name),                                                \
        .affinity  = (_affinity),                                            \
        .priority  = (_prio),                                                \
    }
```

#### Section bounds symbols (in `esp_hosted_cp_core.c`)

```c
/* Defined by the linker: start and end of .eh_cp_ext_descs section.
 * On platforms without a custom linker script (Linux userspace), GCC/Clang
 * emit __start_SECNAME and __stop_SECNAME automatically for any section
 * whose name is a valid C identifier. .eh_cp_ext_descs qualifies.       */
extern const esp_hosted_ext_desc_t __eh_cp_ext_descs_start;
extern const esp_hosted_ext_desc_t __eh_cp_ext_descs_end;
/* Non-IDF builds may also export __start_eh_cp_ext_descs / __stop_eh_cp_ext_descs */
```

#### Dispatcher task (in `esp_hosted_cp_core.c`)

```c
static void ext_init_task(void *arg)
{
    const esp_hosted_ext_desc_t *start = &__eh_cp_ext_descs_start;
    uint32_t n = (uint32_t)(&__eh_cp_ext_descs_end - start);

    /* Build a sorted index array on the stack (n ≤ ~10, negligible) */
    uint8_t order[EH_CP_EXT_MAX_DESCS];  /* EH_CP_EXT_MAX_DESCS = compile-time upper bound for stack */
    for (uint8_t i = 0; i < n; i++) order[i] = i;

    /* Insertion sort ascending by .priority — identical to IDF startup_internal approach */
    for (uint8_t i = 1; i < n; i++) {
        uint8_t key = order[i];
        int8_t  j   = (int8_t)(i - 1);
        while (j >= 0 && start[order[j]].priority > start[key].priority) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }

    for (uint32_t i = 0; i < n; i++) {
        const esp_hosted_ext_desc_t *d = &start[order[i]];
        ESP_LOGI(TAG, "ext init: %s (prio %d)", d->name, d->priority);
        esp_err_t r = d->init_fn();
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "ext %s init failed: %s", d->name, esp_err_to_name(r));
            /* Continue — partial init preferred over hard stop */
        }
    }

    /* All extensions initialised — unblock transport init */
    xEventGroupSetBits(s_ext_init_done, BIT0);
    vTaskDelete(NULL);
}
```

**Note on `affinity`:** The `affinity` field in the descriptor is stored for
future use (e.g. pinning a heavy `_feat_` init to APP_CPU). For the first
implementation, `ext_init_task` itself runs on `tskNO_AFFINITY` and calls
all `init_fn`s sequentially regardless of their `affinity` field. Per-init
task pinning is a later optimisation if it proves necessary.

#### Example usage in an extension (e.g. `esp_hosted_cp_ext_rpc_wifi`)

```c
/* esp_hosted_cp_ext_rpc_wifi.c */

static esp_err_t rpc_wifi_init(void)
{
    /* Register RPC request range */
    esp_hosted_cp_rpc_req_register(EH_RPC_V1_MCU_WIFI_REQ_MIN,
                                   EH_RPC_V1_MCU_WIFI_REQ_MAX,
                                   wifi_rpc_req_handler, NULL);
    /* Register event serialiser range */
    esp_hosted_cp_rpc_evt_register(EH_RPC_V1_MCU_WIFI_EVT_MIN,
                                   EH_RPC_V1_MCU_WIFI_EVT_MAX,
                                   wifi_rpc_evt_serialise, NULL);
    /* Register capability bit */
    esp_hosted_cp_add_cap_bits(ESP_WLAN_SUPPORT, 0);

    return ESP_OK;
}

/* Self-registration — zero cost if this file is not compiled.
 * affinity = tskNO_AFFINITY, priority = 100 (early, _rpc_ tier) */
EH_CP_EXT_REGISTER(rpc_wifi_init, NULL, "rpc_wifi", tskNO_AFFINITY, 100);
```

### Portability notes

| Platform | Section support | `__start_`/`__stop_` symbols |
|----------|----------------|------------------------------|
| ESP-IDF (FreeRTOS, GCC) | ✅ Full | ✅ Auto-generated by GNU ld |
| Linux userspace (GCC/Clang) | ✅ Full | ✅ Auto-generated by GNU ld / lld |
| Linux kernel module (GCC) | ✅ Full | ✅ Available via `extern` |
| macOS (Clang, for host-side tooling) | ⚠️ Uses `__DATA,__eh_cp` segment | Needs `getsectiondata()` — host tooling only, not runtime target |

For the CP firmware (ESP-IDF) and Linux host — the two targets that matter —
this approach is fully portable without any linker script changes.

### Naming convention — unchanged from rev-1

| Pattern | Meaning |
|---------|---------|
| `esp_hosted_cp_ext_rpc_XXX` | Lightweight — only RPC table registration, no tasks |
| `esp_hosted_cp_ext_feat_XXX` | Heavy — may start tasks, allocate buffers, complex teardown |

Both tiers use `EH_CP_EXT_REGISTER`. The distinction is only about what
their `init_fn` does internally, not about how they are registered.

**Renamed extensions:**

| Old name | New name | Tier |
|----------|----------|------|
| `esp_hosted_cp_ext_rpc_mcu` | `esp_hosted_cp_ext_rpc_mcu` | `_rpc_` |
| `esp_hosted_cp_ext_rpc_linux_fg` | `esp_hosted_cp_ext_rpc_fg` | `_rpc_` |
| `esp_hosted_cp_ext_host_ps` | `esp_hosted_cp_ext_feat_host_ps` | `_feat_` |
| `esp_hosted_cp_ext_network_split` | `esp_hosted_cp_ext_feat_nw_split` | `_feat_` |
| `esp_hosted_cp_ext_custom_rpc` | `esp_hosted_cp_ext_feat_custom_msg` | `_feat_` |

---

## Decision 3 — Feature Capability Bits Registered by Each Extension in Its Own Init

**Decision:**
Each extension calls `esp_hosted_cp_add_cap_bits()` inside its own `init_fn`
(which runs via the `EH_CP_EXT_REGISTER` dispatcher, Decision 2). The core
does NOT have a `populate_core_caps()` function that knows about extensions.

**Rationale:**
- Each extension owns its own capability declaration — high cohesion.
- Core remains decoupled from extension internals.
- Flash/RAM savings: if an extension is not compiled, its capability bit is
  never registered, so the PRIV TLV correctly advertises only what is present.
- The only ordering constraint: all extension `init_fn`s must run before
  `generate_startup_event()` fires. This is guaranteed by sequencing in the
  dispatcher: `ext_init_task` calls all extensions, then signals the core
  transport init task to proceed with the PRIV handshake.

**Signalling sequence:**

```
esp_hosted_cp_core_init()
  └─ creates ext_init_task (priority EH_CP_EXT_INIT_TASK_PRIORITY)
  └─ creates transport_init_task (blocked on s_ext_init_done event group)

ext_init_task:
  └─ calls each init_fn (sorted by priority)
     └─ each init_fn calls esp_hosted_cp_add_cap_bits(...)
  └─ sets s_ext_init_done bit
  └─ vTaskDelete(NULL)

transport_init_task:
  └─ waits on s_ext_init_done
  └─ calls generate_startup_event()   ← now has all capability bits
  └─ proceeds with PRIV handshake
```

**`esp_hosted_cp_add_cap_bits()` remains in the public API** (contrary to rev-1
which proposed removing it). Extensions call it; core does not.

---

## Decision 4 — Proto Components in `common/serializers/`

**Final names:**

```
components/common/serializers/
  esp_hosted_proto_linux_v1/        ← IDF component; CtrlMsg (FG Linux V1)
    CMakeLists.txt
    include/esp_hosted_config.pb-c.h
    src/esp_hosted_config.pb-c.c
    proto/esp_hosted_config.proto
    proto/build_proto.sh
  esp_hosted_proto_mcu_v1/          ← IDF component; Rpc (MCU V1)
    CMakeLists.txt
    include/esp_hosted_rpc.pb-c.h
    src/esp_hosted_rpc.pb-c.c
    proto/esp_hosted_rpc.proto
    proto/build_proto.sh
  esp_hosted_proto_v2/              ← placeholder; empty until V2 schema designed
    CMakeLists.txt (minimal, no SRCS yet)
    proto/  (empty)
  third_party/
    msg_codec/protobuf-c/
```

**Extension REQUIRES mapping:**

| Extension | Proto REQUIRES |
|-----------|---------------|
| `esp_hosted_cp_ext_rpc_fg` | `esp_hosted_proto_linux_v1` |
| `esp_hosted_cp_ext_rpc_mcu` | `esp_hosted_proto_mcu_v1` |
| `esp_hosted_host_ext_wifi` (Linux host) | `esp_hosted_proto_linux_v1` |
| `esp_hosted_host_ext_wifi` (MCU host) | `esp_hosted_proto_mcu_v1` |
| Future V2 extensions | `esp_hosted_proto_v2` |

---

## Decision 5 — Proto Encode/Decode Lives in the Extension; `out_max` Added

**Decision:** Extension owns all protobuf encode/decode. No change from rev-1.

**Q3 (clarification on `out_max`):**

The current event serialise callback signature is:
```c
esp_err_t (*serialise)(void *ctx, uint32_t event_id,
                       const void *data, uint16_t data_len,
                       void *out_buf, uint16_t *out_len);
```

The problem: `registries.c::esp_hosted_cp_rpc_send_event()` allocates a buffer
of some size and passes it as `out_buf`. But the serialise function has no way
to know how large that buffer is. If the serialised proto is larger than what
was allocated, the `memcpy` inside the extension writes past the buffer end —
a heap overflow, silent memory corruption.

The fix is to add `out_max` (the allocated buffer size) as the last parameter:
```c
esp_err_t (*serialise)(void *ctx, uint32_t event_id,
                       const void *data,    uint16_t data_len,
                       void *out_buf, uint16_t *out_len,
                       uint16_t out_max);   /* ← NEW: max bytes extension may write */
```

The extension then checks:
```c
size_t packed_sz = rpc__get_packed_size(ntfy);
if (packed_sz > out_max) {
    ESP_LOGE(TAG, "evt %u: packed_sz %zu > out_max %u", event_id, packed_sz, out_max);
    return ESP_ERR_NO_MEM;
}
rpc__pack(ntfy, out_buf);
*out_len = (uint16_t)packed_sz;
```

This is a breaking change to all existing serialise implementations. Since this
is a greenfield internal codebase (no external extensions exist yet), we break
the signature freely. All call sites must be updated in Phase D.

---

## Decision 6 — RPC ID Map: Separate Files for V1 and V2

**Decision:**
Two separate ID map header files, one per protocol version. Both live in
`components/common/esp_hosted_common/include/`.

**File 1:** `esp_hosted_rpc_id_map_v1.h`
```c
/*
 * esp_hosted_rpc_id_map_v1.h
 * V1 RPC ID range map — single source of truth for V1 request, response,
 * and event ID ranges. Both CP and host sides include this file.
 *
 * The proto enum values (CTRL_MSG_ID__Req_Base etc.) are defined in the
 * generated .pb-c.h files. _Static_assert cross-checks below verify that
 * the ranges here agree with the proto enums at compile time.
 */
#pragma once

/* ── Linux FG V1 (CtrlMsg / esp_hosted_config.proto) ───────────── */
#define EH_RPC_V1_FG_REQ_MIN      100u
#define EH_RPC_V1_FG_REQ_MAX      199u
#define EH_RPC_V1_FG_RESP_MIN     200u
#define EH_RPC_V1_FG_RESP_MAX     299u
#define EH_RPC_V1_FG_EVT_MIN      300u
#define EH_RPC_V1_FG_EVT_MAX      399u

/* ── MCU V1 (Rpc / esp_hosted_rpc.proto) ───────────────────────── */
#define EH_RPC_V1_MCU_REQ_MIN     0x100u
#define EH_RPC_V1_MCU_REQ_MAX     0x1FFu
#define EH_RPC_V1_MCU_RESP_MIN    0x200u
#define EH_RPC_V1_MCU_RESP_MAX    0x2FFu
#define EH_RPC_V1_MCU_EVT_MIN     0x300u
#define EH_RPC_V1_MCU_EVT_MAX     0x3FFu

/*
 * Compile-time cross-checks against proto-generated enums.
 * Include the proto headers before this file if _Static_assert is desired.
 * Guard with INCLUDE_EH_RPC_ID_MAP_V1_ASSERTS so files that cannot include
 * proto headers (e.g. common wire-protocol code) can still include the ranges.
 */
#ifdef INCLUDE_EH_RPC_ID_MAP_V1_ASSERTS
#include "esp_hosted_config.pb-c.h"
#include "esp_hosted_rpc.pb-c.h"
_Static_assert(EH_RPC_V1_FG_REQ_MIN  == CTRL_MSG_ID__Req_Base  + 1,
               "FG REQ_MIN mismatch with proto enum");
_Static_assert(EH_RPC_V1_FG_RESP_MIN == CTRL_MSG_ID__Resp_Base,
               "FG RESP_MIN mismatch with proto enum");
_Static_assert(EH_RPC_V1_MCU_REQ_MIN == RPC_ID__Req_Base + 1,
               "MCU REQ_MIN mismatch with proto enum");
_Static_assert(EH_RPC_V1_MCU_RESP_MIN == RPC_ID__Resp_Base,
               "MCU RESP_MIN mismatch with proto enum");
#endif /* INCLUDE_EH_RPC_ID_MAP_V1_ASSERTS */
```

**File 2:** `esp_hosted_rpc_id_map_v2.h`
```c
/*
 * esp_hosted_rpc_id_map_v2.h
 * V2 RPC ID range map. Unified namespace — no FG/MCU split.
 * Both FG Linux and MCU hosts use the same IDs over V2.
 */
#pragma once

/* ── V2 Unified ─────────────────────────────────────────────────── */
#define EH_RPC_V2_REQ_MIN     0x2000u
#define EH_RPC_V2_REQ_MAX     0x3FFFu
#define EH_RPC_V2_RESP_MIN    0x4000u
#define EH_RPC_V2_RESP_MAX    0x5FFFu
#define EH_RPC_V2_EVT_MIN     0x6000u
#define EH_RPC_V2_EVT_MAX     0x7FFFu

/* Sentinels — do NOT move these when adding new V2 IDs */
#define EH_RPC_V2_REQ_SENTINEL   0x3FFFu
#define EH_RPC_V2_RESP_SENTINEL  0x5FFFu
#define EH_RPC_V2_EVT_SENTINEL   0x7FFFu

/*
 * _Static_assert cross-checks added here once esp_hosted_proto_v2
 * schema is finalised (PENDING-006).
 */
```

**Who includes which file:**
- V1 extensions include `esp_hosted_rpc_id_map_v1.h`
- V2 extensions include `esp_hosted_rpc_id_map_v2.h`
- Code that needs both (e.g. the dispatcher) includes both
- Neither file includes the other — clean separation

---

## Decision 7 — Directory Structure as Designed

Directory structure from the attached design document adopted. Name changes
are as listed in Decision 2 and Decision 4. The `port/cp/` placeholder remains
empty (Phase B deferred). The `ng/` silo is unchanged.

---

## Decision 8 — V2 RPC: Advertised, Negotiated, End-to-End

**Decision:**
- CP advertises `ESP_PRIV_RPC_VERSION = 0x02` in PRIV TLV at boot.
- Any host built against this codebase negotiates V2 by default.
- Old hosts (no ACK) fall back to V1 — no breakage.
- V2 is end-to-end: both Linux FG and MCU hosts will implement V2
  (with V1 backward compat). V2 is not CP-only.
- V2 proto schema design is PENDING-006, deferred until Phase F.

---

## Decision 9 — Dynamic Realloc Table for RPC Req/Evt Dispatch

**Decision:**
Replace SLIST with a flat heap-allocated table per direction (req / evt).
No compile-time cap. Grows by 4 entries at a time when full.

### Table entry structs (unchanged from rev-1 except `out_max` on serialise)

```c
typedef struct {
    uint16_t req_id_min;
    uint16_t req_id_max;
    esp_err_t (*handler)(void *ctx, uint32_t msg_id,
                         const void *req_buf, uint16_t req_len,
                         void *resp_buf, uint16_t *resp_len, uint16_t resp_max);
    void *ctx;
} esp_hosted_rpc_req_entry_t;

typedef struct {
    uint16_t evt_id_min;
    uint16_t evt_id_max;
    esp_err_t (*serialise)(void *ctx, uint32_t event_id,
                           const void *data,    uint16_t data_len,
                           void *out_buf, uint16_t *out_len,
                           uint16_t out_max);
    void *ctx;
} esp_hosted_rpc_evt_entry_t;
```

### Growth policy

```c
#define EH_CP_RPC_TABLE_GROW_BY   4u   /* realloc adds 4 slots at a time */

/* On registration when count == capacity: */
new_capacity = capacity + EH_CP_RPC_TABLE_GROW_BY;
new_ptr = realloc(table, new_capacity * sizeof(entry));
/* realloc returns 4-byte aligned memory on all platforms we target     */
/* (heap_caps_malloc with MALLOC_CAP_DEFAULT guarantees 4-byte align).  */
```

Why 4 at a time, not doubling? With 4–8 extensions total we will never grow
beyond the first realloc anyway. Doubling wastes memory at this scale.

**No compile-time maximum.** `EH_CP_MAX_RPC_EXTENSIONS = 16` was proposed in
rev-1 but rejected: there is no good reason to cap this at compile time when
the growth is bounded naturally by what is actually registered. A CP with 6
extensions uses exactly the memory for 8 entries (2 rounds of +4). Done.

### Registration API (public)

```c
/* Register a request handler for msg_ids in [req_id_min, req_id_max] */
esp_err_t esp_hosted_cp_rpc_req_register(
    uint16_t req_id_min, uint16_t req_id_max,
    esp_err_t (*handler)(void *ctx, uint32_t msg_id,
                         const void *req_buf, uint16_t req_len,
                         void *resp_buf, uint16_t *resp_len, uint16_t resp_max),
    void *ctx);

/* Register an event serialiser for event_ids in [evt_id_min, evt_id_max] */
esp_err_t esp_hosted_cp_rpc_evt_register(
    uint16_t evt_id_min, uint16_t evt_id_max,
    esp_err_t (*serialise)(void *ctx, uint32_t event_id,
                           const void *data,    uint16_t data_len,
                           void *out_buf, uint16_t *out_len, uint16_t out_max),
    void *ctx);

/* Deregistration (rare — for hot-reload / deinit) */
esp_err_t esp_hosted_cp_rpc_req_unregister(uint16_t req_id_min, uint16_t req_id_max);
esp_err_t esp_hosted_cp_rpc_evt_unregister(uint16_t evt_id_min, uint16_t evt_id_max);
```

### Dispatch

**Request:** binary search on sorted req table (O(log n)).
**Event:** linear scan on evt table (O(n), fine — events are rare, table tiny).

### Thread safety

Single mutex per table. For dispatch:
- Take mutex → snapshot `count` + `table` pointer → release mutex
- Call handler outside mutex (handler may block for WiFi operations)

---

## Decision 10 — Master Config Headers: `EH_CP_XXX` / `EH_HOST_XXX`

**Decision:**
- `esp_hosted_cp_master_config.h` in `esp_hosted_cp_core/include/` maps
  `CONFIG_ESP_HOSTED_CP_XXX` → `EH_CP_XXX`.
- `esp_hosted_host_master_config.h` in each port's include dir maps
  `CONFIG_ESP_HOSTED_HOST_XXX` → `EH_HOST_XXX`.
- No CP source file outside `port/` uses `CONFIG_` symbols directly.
- `components/common/` sources use neither (must be config-neutral).

> **Note:** Step C7 (create the `esp_hosted_cp_master_config.h` skeleton) is
> **deferred** — it can be done independently of C1–C6 and D1–D9, and adds
> no unblocking value until Phase E when all `EH_CP_XXX` symbols are needed.
> Phase C exit gate (C8 / `idf.py build`) does not require C7.

---

## Decision 11 — Extension Header Ownership; Ops Registry Deleted

### Problem (confirmed by code audit, Session 15)

`esp_hosted_cp_extension.h` (a **core** header) included three extension-type
headers and defined a union of all three ops structs:

```c
// esp_hosted_cp_extension.h  ← THIS IS WRONG AND BEING DELETED
#include "esp_hosted_cp_extension_rpc.h"
#include "esp_hosted_cp_extension_nw_split.h"
#include "esp_hosted_cp_extension_host_ps.h"

typedef struct {
    union {
        esp_hosted_cp_ext_cp_custom_rpc_config_t  rpc_config;
        esp_hosted_cp_ext_cp_nw_split_config_t    nw_split_config;
        esp_hosted_cp_ext_cp_host_ps_config_t     host_ps_config;
    } u;
    esp_hosted_cp_ext_type_t ext_type;
} esp_hosted_cp_ext_cp_config_t;

esp_err_t esp_hosted_cp_ext_cp_register_extension(...);
void     *esp_hosted_cp_ext_cp_get_extension(...);
```

All `get_extension(ESP_HOSTED_EXT_NW_SPLIT)` / `get_extension(ESP_HOSTED_EXT_HOST_PS)`
calls were the only reason these types existed in core. Every caller is an
extension — never core itself. Core's `extension.c` only implements the ops
registry; core's `core.c` / `registries.c` / `rpc_ll.c` never use it.

### Decision

**Delete the ops registry entirely.** This means:

| File | Action |
|------|--------|
| `esp_hosted_cp_core/include/esp_hosted_cp_extension.h` | **Delete** |
| `esp_hosted_cp_core/include/esp_hosted_cp_extension_nw_split.h` | **Delete** |
| `esp_hosted_cp_core/include/esp_hosted_cp_extension_rpc.h` | **Delete** |
| `esp_hosted_cp_core/include/esp_hosted_cp_extension_host_ps.h` | **Delete** |
| `esp_hosted_cp_core/src/esp_hosted_cp_extension.c` | **Delete** |

**Core knows zero extension types.** Core's public headers (`esp_hosted_cp_core.h`,
`esp_hosted_cp_event.h`, etc.) contain no `#include` of any extension header,
no ops struct typedef, and no union of extension configs.

### What replaces the ops registry

The ops registry solved two interaction patterns between extensions:

**1 — Sync query (e.g. `rpc_mcu` asking nw_split for current status):**
Replace with a direct function call. `esp_hosted_ext_cp_feat_nw_split_apis.h`
(in the nw_split extension's own `include/`) exports the query functions.
`rpc_mcu` adds `REQUIRES esp_hosted_cp_ext_feat_nw_split` to its CMakeLists
and calls the function directly.

**2 — Async notify (nw_split → rpc adapters, 1-to-N):**
Already fixed (Session 13). nw_split posts to `ESP_HOSTED_CP_EVENT`. RPC
adapters subscribe in their own `init()`. No ops struct needed.

### Extension header layout (post-rename, post-split)

Each extension owns its own `include/` with up to two public headers:

```
esp_hosted_cp_ext_feat_nw_split/
  include/
    esp_hosted_ext_cp_feat_nw_split_apis.h    ← init/deinit/filter/rx_cb
    esp_hosted_ext_cp_feat_nw_split_events.h  ← esp_hosted_cp_ext_nw_split_status_t
                                                  + #include "esp_hosted_cp_event.h"

esp_hosted_cp_ext_feat_host_ps/
  include/
    esp_hosted_ext_cp_feat_host_ps_apis.h     ← init/deinit/is_host_power_saving
    esp_hosted_ext_cp_feat_host_ps_events.h   ← (add when host_ps posts events)

esp_hosted_cp_ext_feat_custom_msg/
  include/
    esp_hosted_ext_cp_feat_custom_msg_apis.h  ← user-defined RPC registration API

esp_hosted_cp_ext_rpc_mcu/
  include/
    esp_hosted_ext_rpc_mcu.h                  ← public (unchanged)

esp_hosted_cp_ext_rpc_fg/
  include/
    esp_hosted_ext_rpc_fg.h                   ← public (unchanged)
```

**Rule:** Event payload structs live in the extension's own `_events.h`.
Event IDs (integers) live in `esp_hosted_cp_event.h` (core). Core never
includes extension headers. Extensions that subscribe to another extension's
events add that extension to their CMake `REQUIRES` and include its `_events.h`.

### CMake dependency for cross-extension event subscription

```cmake
# rpc_mcu/CMakeLists.txt — add to REQUIRES:
esp_hosted_cp_ext_feat_nw_split   # already runtime-coupled; now explicit at build time
```

```c
// rpc_mcu/event_subscriber.c
#include "esp_hosted_ext_cp_feat_nw_split_events.h"  // gets nw_split_status_t
```

There is no circular dependency: nw_split does not include any rpc_mcu header.
The dependency is one-directional: `rpc_mcu` → `nw_split` (for types only).

### Rule: no core header may include an extension header — ever

This is a hard invariant. If a future extension type needs to be shared across
multiple extensions, the shared type goes into `components/common/` (if truly
generic) or into a dedicated shared-types header inside one of the extensions
(if it is extension-domain-specific). Core is never the intermediary.

---

## Implementation Plan

### Phase C — Restructure (file moves + header split; no functional change)

| Step | Action |
|------|--------|
| C1 | Create `components/common/serializers/esp_hosted_proto_linux_v1/` from `ext_rpc_linux_fg_pbuf/` |
| C2 | Create `components/common/serializers/esp_hosted_proto_mcu_v1/` from `ext_rpc_mcu_pbuf/` |
| C3 | Create `components/common/serializers/esp_hosted_proto_v2/` (empty placeholder) |
| C4 | Delete `_pbuf` extension sidecar dirs; update all REQUIRES |
| C5 | Rename extensions per Decision 2 table |
| C6 | Create `esp_hosted_rpc_id_map_v1.h` and `esp_hosted_rpc_id_map_v2.h` |
| C7 | ~~Create `esp_hosted_cp_master_config.h` skeleton~~ **DEFERRED to Phase E** |
| C8 | Split extension public headers: create `_apis.h` + `_events.h` per extension (Decision 11) |
| C9 | Replace all `get_extension(ESP_HOSTED_EXT_NW_SPLIT / HOST_PS)` call sites with direct function calls; add CMake REQUIRES accordingly |
| C10 | Delete ops registry files: `esp_hosted_cp_extension.h`, `esp_hosted_cp_extension_nw_split.h`, `esp_hosted_cp_extension_rpc.h`, `esp_hosted_cp_extension_host_ps.h`, `esp_hosted_cp_extension.c` |
| C11 | `idf.py build` — Phase C exit gate (no functional change, must still build) |

### Phase D — Registry + Auto-Init (functional changes)

| Step | Action |
|------|--------|
| D1 | Define `esp_hosted_ext_desc_t` + `EH_CP_EXT_REGISTER` macro in `esp_hosted_cp_core.h` |
| D2 | Implement `ext_init_task` + `s_ext_init_done` event group in `esp_hosted_cp_core.c` |
| D3 | Add `EH_CP_EXT_REGISTER(...)` call to each extension's `.c` file |
| D4 | Replace SLIST tables with dynamic realloc tables in `esp_hosted_cp_registries.c` |
| D5 | Update public API in `esp_hosted_cp_core.h` (new register/unregister signatures) |
| D6 | Add `out_max` parameter to all serialise callback implementations |
| D7 | Update all extension registration call sites to new API |
| D8 | Sequence `generate_startup_event()` after `s_ext_init_done` (Decision 3) |
| D9 | `idf.py build` — Phase D exit gate |

### Phase E — Config Macros + Version Wiring

| Step | Action |
|------|--------|
| E1 | Populate `esp_hosted_cp_master_config.h` — all `EH_CP_XXX` symbols |
| E2 | Replace all direct `CONFIG_ESP_HOSTED_CP_XXX` uses in CP sources |
| E3 | Add `_Static_assert` cross-checks in `esp_hosted_rpc_id_map_v1.h` |
| E4 | Update PRIV handshake defaults for V2 negotiation |
| E5 | `idf.py build` — Phase E exit gate |

### Phase F — V2 Proto + Schema (future, after PENDING-006)

| Step | Action |
|------|--------|
| F1 | Design unified `esp_hosted_rpc_v2.proto` (WiFi + BT + OTA + System, single namespace) |
| F2 | `protoc-c` codegen → `esp_hosted_proto_v2/` |
| F3 | V2 extension on CP side |
| F4 | Host-side V2: Linux ctrl_lib + MCU rpc_req.c |
| F5 | End-to-end V2 test — both FG and MCU negotiate and use V2 |

---

## Decision 12 — Protocomm Belongs in an Extension, Not Core

**Decision:**
Move all protocomm code (`rpc_ll.c`) out of `esp_hosted_cp_core` into a new
extension `esp_hosted_cp_ext_rpc` (priority 5). Core retains only the serial
I/O bridge (`rpc.c` — `serial_write_data`, `serial_read_data`,
`esp_hosted_cp_process_serial_rx_pkt`). Core's CMakeLists drops the `protocomm`
IDF dependency entirely.

**Problem solved:**
The `"Protocomm not initialized, drop serial packet"` race condition: protocomm
was previously cold-initialized only after the host completed the PRIV handshake
(post-extension init). WiFi events fired between extension init and handshake
completion were silently dropped. Moving init to priority-5 means protocomm is
ready before any other extension registers event subscribers.

**Public API surface of `esp_hosted_cp_ext_rpc`** (5 functions only):
- `esp_hosted_cp_ext_rpc_is_ready()` — replaces NULL instance guard
- `esp_hosted_cp_ext_rpc_process_req(buf, len)` — called by core serial RX path
- `esp_hosted_cp_ext_rpc_send_evt(ep, id, data, len)` — called by registries send_event
- `esp_hosted_cp_ext_rpc_set_endpoints(req_ep, evt_ep)` — called at init (defaults) and by host_to_slave_reconfig (confirmed names); atomic upgrade under mutex if names change
- `esp_hosted_cp_ext_rpc_get_evt_ep()` — returns current evt ep name for event tagging

**`pserial_task` location:** Stays in `rpc_ll.c`, which moves entirely into `ext_rpc`.
`pserial_task` is a private static function — never exported, never touched outside the component.

**Endpoint name negotiation:** CP starts with default names (`"RPCRsp"`, `"RPCEvt"`) at
priority-5 init. After PRIV handshake, host confirms (or proposes different) names via
`ESP_PRIV_RPC_EP_ACK`. `set_endpoints()` atomically re-registers protocomm endpoints if names
change. No window of broken names because host RPC requests cannot arrive before the handshake.

**NG forward-compatibility:** NG variants that don't use RPC simply omit
`esp_hosted_cp_ext_rpc` from their CMake component list. Core compiles and
links cleanly with zero protocomm dependency.

See **spec 17** for full design, component dependency graph, risks, and
step-by-step implementation plan (Phase G).

---

## Summary Table

| # | Decision | Key change from rev-1 | Phase |
|---|----------|-----------------------|-------|
| 1 | IDF REQUIRES; proto names corrected | `linux_v1` / `mcu_v1` naming | C1–C4 |
| 2 | Auto-init via linker-section descriptor | Replaces Kconfig ladder; post-scheduler task | D1–D3 |
| 3 | Extensions register own cap bits in init | Core does NOT populate caps | D8 |
| 4 | Proto in `common/serializers/` with final names | `linux_v1` / `mcu_v1` | C1–C4 |
| 5 | Proto encode/decode in extension; `out_max` added | Breaking signature change | D6 |
| 6 | Two ID map files: `_v1.h` + `_v2.h` | Separated by version | C6 |
| 7 | Directory structure as designed | Extension renames | C5 |
| 8 | V2 end-to-end, both hosts | Not CP-only | F4 |
| 9 | Realloc table, +4 at a time, no cap | No MAX constant | D4–D7 |
| 10 | `EH_CP_XXX` / `EH_HOST_XXX` macros; C7 deferred | Config isolation | E1–E2 |
| 11 | Ops registry deleted; extension headers owned by extensions | Core includes zero extension types; `_apis.h` + `_events.h` split | C8–C10 |
| 12 | Protocomm moves to `esp_hosted_cp_ext_rpc` (priority 5) | Core loses protocomm dep; race eliminated; NG-compatible | G1–G10 |
