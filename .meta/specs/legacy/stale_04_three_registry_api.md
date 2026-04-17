# 04 — RPC Registry and Dispatch Architecture
<!-- Last updated: 2026-03-12 Decision 9 — SLIST fully replaced by realloc table -->

## Motivation

The original three-registry design used BSD SLISTs for RPC dispatch. After team
review in Decision 9 (2026-03-12) the SLIST was replaced with a flat
dynamically-allocated table. Reasons:

- No `sys/queue.h` dependency anywhere in the codebase.
- No node struct allocation per extension — registration is a plain function call.
- req table binary search is O(log n) vs SLIST O(n) walk.
- Flat table trivially dumpable for debug; SLIST requires pointer-chasing.
- Unregister is clean `memmove` on the inline array; SLIST required keeping
  the node pointer alive after registration.

The two non-RPC registries (iface table, capability accumulator) are unchanged.

---

## Registry 1 — Interface RX/TX Table

**File**: `esp_hosted_cp_core.h` / `esp_hosted_cp_registries.c`

Flat array indexed by `esp_hosted_if_type_t`. Zero-initialised (BSS).

```c
typedef struct {
    hosted_rx_cb_t rx;   /* RX handler; NULL = no handler (frame dropped) */
    hosted_tx_cb_t tx;   /* TX override; NULL = default transport write   */
    void          *ctx;  /* private context for both callbacks             */
} esp_hosted_iface_entry_t;

extern esp_hosted_iface_entry_t g_iface_table[ESP_IF_TYPE_MAX];
```

### API

```c
esp_err_t esp_hosted_cp_register_rx_cb(esp_hosted_if_type_t, hosted_rx_cb_t, void *ctx);
esp_err_t esp_hosted_cp_register_tx_cb(esp_hosted_if_type_t, hosted_tx_cb_t, void *ctx);
esp_err_t esp_hosted_cp_dispatch_rx(esp_hosted_if_type_t, void *buf, uint16_t len, void *eb);
```

### Interface Ownership

| Interface | Owner |
|-----------|-------|
| `ESP_STA_IF` | Default WiFi STA callback (overridable by extension) |
| `ESP_AP_IF` | Default WiFi AP callback (overridable by extension) |
| `ESP_HCI_IF` | CP BT bridge (do not re-register) |
| `ESP_SERIAL_IF` | Core only (protocomm RPC path) |
| `ESP_PRIV_IF` | Core only (PRIV handshake) |

---

## Registry 2 — Capability Accumulator

**File**: `esp_hosted_cp_registries.c`

Atomic OR accumulation via FreeRTOS spinlock. Each extension calls
`esp_hosted_cp_add_cap_bits()` inside its own `init_fn` (invoked by the
auto-init dispatcher — see EH_CP_EXT_REGISTER below).

```c
void esp_hosted_cp_add_cap_bits(uint8_t caps_bits, uint32_t ext_caps_bits);
void esp_hosted_cp_add_feature_cap_bits(uint8_t index, uint32_t feature_bitmask);

uint8_t  esp_hosted_cp_get_caps(void);
uint32_t esp_hosted_cp_get_ext_caps(void);
void     esp_hosted_cp_get_feat_caps(uint32_t out[ESP_HOSTED_FEAT_CAPS_COUNT]);
```

`feat_caps[8]` (8 × uint32 = 256 bits) is the tier-3 capability space.
Transmitted only in the boot-time PRIV TLV frame; never in RPC messages.

**Sequencing guarantee:** `ext_init_task` sets `EH_CP_EXT_INIT_DONE_BIT` only
after all `init_fn`s complete. `generate_startup_event()` waits on this bit
before reading capability registers.

---

## Registry 3 — RPC Realloc-Table Dispatch

**File**: `esp_hosted_cp_registries.c`

Two flat heap-allocated arrays replace the former SLISTs. No `sys/queue.h`,
no node structs, no pointers to keep alive. Registration is a plain call
with `(id_min, id_max, callback, ctx)` — the range is stored inline.

### Callback Types

```c
/* Called by core when a request in [id_min, id_max] arrives */
typedef esp_err_t (*eh_rpc_req_handler_t)(
    void *ctx,
    uint32_t msg_id,
    const void *req_buf, uint16_t req_len,
    void *resp_buf, uint16_t *resp_len, uint16_t resp_max);

/* Called by core when an extension pushes an event in [id_min, id_max] */
typedef esp_err_t (*eh_rpc_evt_serialise_t)(
    void *ctx,
    uint32_t event_id,
    const void *data, uint16_t data_len,
    void *out_buf, uint16_t *out_len, uint16_t out_max);
```

`resp_max` / `out_max` is the size of the pre-allocated output buffer passed
by core — the extension must not write beyond it.

### Public API

```c
/* Registration — stores range inline, no struct to allocate */
esp_err_t esp_hosted_cp_rpc_req_register(uint16_t id_min, uint16_t id_max,
                                          eh_rpc_req_handler_t handler, void *ctx);
esp_err_t esp_hosted_cp_rpc_evt_register(uint16_t id_min, uint16_t id_max,
                                          eh_rpc_evt_serialise_t serialise, void *ctx);

/* Deregistration — match by exact (id_min, id_max) pair */
esp_err_t esp_hosted_cp_rpc_req_unregister(uint16_t id_min, uint16_t id_max);
esp_err_t esp_hosted_cp_rpc_evt_unregister(uint16_t id_min, uint16_t id_max);

/* Push an unsolicited event to the host */
esp_err_t esp_hosted_cp_rpc_send_event(uint32_t event_id,
                                        const void *data, uint16_t len);
```

### Table Layout (internal)

```c
/* req table — sorted ascending by id_min */
typedef struct { uint16_t id_min; uint16_t id_max;
                 eh_rpc_req_handler_t handler; void *ctx; } rpc_req_entry_t;

/* evt table — unsorted (append-only) */
typedef struct { uint16_t id_min; uint16_t id_max;
                 eh_rpc_evt_serialise_t serialise; void *ctx; } rpc_evt_entry_t;
```

### Table Growth Policy

```
Initial state  : NULL pointer, count=0, capacity=0
First register : realloc(NULL, 4 × sizeof(entry))  — equivalent to malloc
Growth trigger : count == capacity
Growth amount  : capacity += EH_CP_RPC_TABLE_GROW_STEP (4)
Deregister     : memmove to close gap, count--, capacity unchanged
```

With ~6 extensions in practice: one growth to 4, possibly one to 8. Done.

### Dispatch — Request (binary search)

req table is kept sorted ascending by `id_min` on every registration (sorted
insert + memmove shift). Dispatch uses binary search: O(log n).

```
lo=0, hi=count
while lo < hi:
    mid = lo + (hi-lo)/2
    if table[mid].id_max < msg_id  →  lo = mid+1   (range is below)
    if table[mid].id_min > msg_id  →  hi = mid      (range is above)
    else                           →  call handler, return
return ESP_ERR_NOT_FOUND
```

Mutex is released before calling the handler — handlers may block for WiFi ops.

### Dispatch — Event (linear scan)

evt table is unsorted. Linear scan O(n) is acceptable: events are rare and
the table is small (typically 2–4 entries total across all extensions).

### Overlap Detection

Both `req_register()` and `evt_register()` scan the existing table before
inserting. Any overlap in `[id_min, id_max]` ranges returns
`ESP_ERR_INVALID_STATE`. Use `esp_hosted_rpc_id_map_v1.h` / `_v2.h` range
constants to pre-coordinate ranges across extensions.

### Thread Safety

A single `SemaphoreHandle_t g_rpc_registry_mutex` serialises all register,
unregister, and dispatch calls. The mutex is released before calling any
handler or serialise callback — callbacks may block freely.

---

## msg_id Field Extraction

`esp_hosted_proto_extract_msg_id()` reads only proto field 2 (varint) from a
raw buffer — no full decode. All `.proto` files place `msg_id` at field 2.

---

## Extension Self-Registration (EH_CP_EXT_REGISTER)

Each extension's `.c` file calls `rpc_req_register()` from inside its
`init_fn`, and places the init_fn in the linker section via:

```c
/* In esp_hosted_cp_ext_rpc_mcu.c, at file scope */
EH_CP_EXT_REGISTER(rpc_mcu_init, NULL, "rpc_mcu", tskNO_AFFINITY, 100);

static esp_err_t rpc_mcu_init(void) {
    esp_hosted_cp_rpc_req_register(MCU_REQ_MSG_ID_MIN, MCU_REQ_MSG_ID_MAX,
                                   mcu_req_adapter, NULL);
    esp_hosted_cp_rpc_evt_register(MCU_EVT_MSG_ID_MIN, MCU_EVT_MSG_ID_MAX,
                                   mcu_evt_adapter, NULL);
    esp_hosted_cp_add_cap_bits(0, ESP_WLAN_SUPPORT);
    return ESP_OK;
}
```

The macro places a descriptor into the `.eh_cp_ext_descs` ELF section.
The section must be retained by the final link and must expose
`__eh_cp_ext_descs_start` / `__eh_cp_ext_descs_end`.
Non-IDF linker fragments also provide compatibility aliases
`__start_eh_cp_ext_descs` / `__stop_eh_cp_ext_descs`.

Portable linker fragment (IDF-independent) lives at:
`components/coprocessor/esp_hosted_cp_core/esp_hosted_cp_ext_descs.ld`.
Non-IDF toolchains should include that fragment (or copy the `SECTIONS` block)
into the final link.

For GCC + CMake (GNU ld), you can use the helper:
`components/coprocessor/esp_hosted_cp_core/esp_hosted_cp_core_linker.cmake`
which adds the companion fragment
`esp_hosted_cp_ext_descs_insert.ld` via `-T`.

IDF builds use `esp_hosted_cp_core.lf` (ldgen fragment) via `LDFRAGMENTS`.
`ext_init_task` walks this section post-scheduler, sorts by priority ascending
(lower = runs first), and calls each `init_fn`. See spec 14 Decision 2/3.

Suggested priority conventions:
- 100 — `_rpc_` tier (lightweight, just registers table entries)
- 200 — `_feat_` tier (may start tasks, allocate buffers)

---

## Endpoint Names

```c
#define RPC_EP_NAME_REQ   "RPCRsp"   /* all hosts send requests here        */
#define RPC_EP_NAME_EVT   "RPCEvt"   /* slave pushes unsolicited events here */
```

Legacy FG alias `"ctrlResp"` registered when `CONFIG_ESP_HOSTED_LEGACY_FG_EP_COMPAT`.

---

## Init Sequence

```c
/* Core boot sequence */
esp_hosted_cp_rpc_registries_init();    /* alloc mutex; tables start NULL */

/* ext_init_task created — runs post-scheduler */
/* Each init_fn calls rpc_req_register() + add_cap_bits()                */
/* EH_CP_EXT_INIT_DONE_BIT set when all init_fns complete                */

/* transport_init_task waits on EH_CP_EXT_INIT_DONE_BIT, then: */
generate_startup_event();               /* reads Registry 2 → PRIV TLV   */
```
