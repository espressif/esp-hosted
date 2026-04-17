# 19 — `esp_hosted_cp_ext_rpc` Component Design

> Status: **Design Final** — ready for implementation.
> Supersedes: `17_rpc_extension_decoupling.md` (partial/incorrect earlier attempt).
> Relates to: `01_architecture_overview`, `12_rpc_call_flow`, `14_design_decisions_v2`.

---

## 1. Goal

Move every RPC/protocomm concern out of `esp_hosted_cp_core` into the new
`esp_hosted_cp_ext_rpc` component.  After this change:

- Core has **zero** knowledge of RPC, protocomm, TLV framing, or serial framing.
- If `esp_hosted_cp_ext_rpc` is not in the build, serial packets are silently
  dropped and no RPC events are sent.  The rest of the system (WiFi data path,
  capability negotiation, extension auto-init) is unaffected.
- Dependency is strictly **one-way**: `esp_hosted_cp_ext_rpc` → `esp_hosted_cp_core`.
  Core never includes any `ext_rpc` header.

---

## 2. What Moves Out of Core

| File (current location in core) | Destination |
|----------------------------------|-------------|
| `src/esp_hosted_cp_rpc.c` | `esp_hosted_cp_ext_rpc/src/` (renamed) |
| `src/esp_hosted_cp_rpc_ll.c` | `esp_hosted_cp_ext_rpc/src/` (renamed) |
| `include/esp_hosted_cp_rpc.h` | `esp_hosted_cp_ext_rpc/include/` |
| `include/esp_hosted_cp_rpc_ll.h` | `esp_hosted_cp_ext_rpc/src/` (private) |
| Registry 3 — req/evt realloc tables | `esp_hosted_cp_ext_rpc/src/` |
| `esp_hosted_cp_rpc_req_register()` etc. (core.h) | `esp_hosted_cp_ext_rpc/include/` |
| `esp_hosted_cp_rpc_send_event()` (registries.c) | `esp_hosted_cp_ext_rpc/src/` |
| `esp_hosted_cp_rpc_dispatch_req()` (registries.c) | internal to `ext_rpc` |
| `initialise_protocomm_hooks()` (core.c) | deleted — replaced by event |
| `rpc_init_with_endpoints()` (core.c) | deleted — replaced by event |
| `EH_CP_PROTOCOMM_READY_BIT` (core.h) | deleted |
| `protocomm` CMake dependency of core | moved to `ext_rpc` CMakeLists |

---

## 3. What Stays in Core

Core retains exactly **two** registries and generic utilities:

**Registry 1 — Interface RX/TX table** (`g_iface_table[ESP_IF_TYPE_MAX]`)
- `esp_hosted_cp_register_rx_cb(iface_type, cb, ctx)`
- `esp_hosted_cp_register_tx_cb(iface_type, cb, ctx)`
- `esp_hosted_cp_dispatch_rx(iface_type, buf, len, eb)`

**Registry 2 — Capability accumulator**
- `esp_hosted_cp_add_cap_bits()` / `esp_hosted_cp_get_caps()` etc.

**Extension auto-init**
- `EH_CP_EXT_REGISTER` macro + linker section + `ext_init_task`

**Negotiation state** (plain scalars, no RPC semantics)
- `volatile uint8_t hdr_ver_negotiated`
- `volatile uint8_t rpc_ver_negotiated`

**Generic proto scanner** (no RPC knowledge — reads a varint from any proto buf)
- `esp_hosted_proto_extract_msg_id()`

**PRIV handshake parser** — `host_to_slave_reconfig()`:
- Parses TLVs, updates `hdr_ver_negotiated` / `rpc_ver_negotiated`.
- Posts **`ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY`** with endpoint strings in
  event data.  That is the last thing it does.  No protocomm call.

**Serial RX demux** — `process_rx_pkt()`:
- Fragment reassembly (seq_num / MORE_FRAGMENT logic) stays in core.
- Once a complete serial payload is assembled, core calls:
  `esp_hosted_cp_dispatch_rx(ESP_SERIAL_IF, data, len, NULL)`
- If `g_iface_table[ESP_SERIAL_IF].rx == NULL` (ext_rpc not loaded):
  `dispatch_rx` returns `ESP_OK` silently.  Core does nothing more.
- Core passes **assembled payload bytes** (`data`, `len`).
  It does NOT pass `buf_handle`, seq_num, or fragmentation metadata —
  those are fully consumed by the reassembly step inside core.

---

## 4. New Event: `ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY`

Added to `esp_hosted_cp_event.h`:

```c
/** Posted by core after PRIV handshake TLVs parsed.
 *  event_data points to esp_hosted_cp_rpc_ep_config_t (stack-allocated,
 *  copied by esp_event into the handler before the post returns). */
#define ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY   4   /* next free value */

typedef struct {
    char req_ep[16];   /* negotiated request endpoint name */
    char evt_ep[16];   /* negotiated event endpoint name   */
} esp_hosted_cp_rpc_ep_config_t;
```

`host_to_slave_reconfig()` fills this struct from the TLV parse and posts it:

```c
esp_hosted_cp_rpc_ep_config_t ep_cfg = {
    .req_ep = RPC_EP_NAME_REQ,   /* default; overwritten if TLV present */
    .evt_ep = RPC_EP_NAME_EVT,
};
/* ... parse TLVs, update ep_cfg if ESP_PRIV_RPC_EP_ACK seen ... */
esp_event_post(ESP_HOSTED_CP_EVENT, ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY,
               &ep_cfg, sizeof(ep_cfg), portMAX_DELAY);
/* function ends here — no protocomm call */
```

`ESP_HOSTED_CP_EVT_ESP_INIT` is **no longer posted from core**.
It is posted from `esp_hosted_cp_ext_rpc` after it processes
`ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY`.

---

## 5. `esp_hosted_cp_ext_rpc` Component

### 5.1 Directory layout

```
components/coprocessor/extensions/esp_hosted_cp_ext_rpc/
  CMakeLists.txt
  include/
    esp_hosted_cp_ext_rpc.h          ← public API (used by rpc_fg, rpc_mcu, feat_*)
  src/
    esp_hosted_cp_ext_rpc.c          ← EH_CP_EXT_REGISTER priority 5; lifecycle
    esp_hosted_cp_ext_rpc_registries.c ← req/evt realloc tables (moved from core)
    esp_hosted_cp_ext_rpc_serial.c   ← serial RX handler + serial_write_data
    esp_hosted_cp_ext_rpc_ll.c       ← protocomm/pserial substrate (private)
    esp_hosted_cp_ext_rpc_ll.h       ← private header (PRIV_INCLUDE_DIRS)
```

### 5.2 Initialization sequence

```
ext_init_task (core) runs all EH_CP_EXT_REGISTER descriptors in priority order:

  priority 5 → esp_hosted_cp_ext_rpc_init()
    1. Allocate req/evt realloc tables.
    2. Initialize protocomm: esp_hosted_cp_protocomm_init(serial_write_data,
                                                          serial_read_data)
       serial_write_data / serial_read_data defined in ext_rpc_serial.c
    3. Register serial RX handler into Registry 1:
         esp_hosted_cp_register_rx_cb(ESP_SERIAL_IF,
                                      esp_hosted_cp_ext_rpc_serial_rx, NULL)
       serial_rx_cb signature: esp_err_t cb(void *ctx, void *buf,
                                            uint16_t len, void *eb)
       buf = assembled payload bytes (from core's reassembly)
       len = payload length
       Inside cb: feed buf/len into protocomm dispatch path.
    4. Subscribe to ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY.
    5. Return ESP_OK.

  priority 100 → rpc_fg / rpc_mcu init (register req/evt table entries)

  (ext_init_task sets EH_CP_EXT_INIT_DONE_BIT)

  host_reset_task sees EH_CP_EXT_INIT_DONE_BIT → generate_startup_event()
  Host responds with ESP_PRIV_EVENT_INIT
  core: host_to_slave_reconfig() → posts ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY

  esp_hosted_cp_ext_rpc: on_private_rpc_ready(ep_cfg)
    → assert(strcmp(ep_cfg.req_ep, RPC_EP_NAME_REQ) == 0)   // future: re-point
    → assert(strcmp(ep_cfg.evt_ep, RPC_EP_NAME_EVT) == 0)
    → esp_event_post(ESP_HOSTED_CP_EVENT, ESP_HOSTED_CP_EVT_ESP_INIT, ...)
```

### 5.3 Serial RX path (complete)

```
transport layer → recv_task (core)
  → process_rx_pkt(buf_handle)
    → ESP_SERIAL_IF branch:
        fragment reassembly (seq_num / MORE_FRAGMENT) — stays in core
        complete payload assembled into r.data / r.len
        → esp_hosted_cp_dispatch_rx(ESP_SERIAL_IF, r.data, r.len, NULL)

  [if ext_rpc not loaded: dispatch_rx sees .rx == NULL → silent drop → done]

  [if ext_rpc loaded]:
  → esp_hosted_cp_ext_rpc_serial_rx(ctx=NULL, buf=r.data, len=r.len, eb=NULL)
      → protocomm_rpc_req_ind(g_protocomm_instance, buf, len, UNKNOWN_MSG_ID)
        enqueues onto pserial req_queue
      → pserial_task dequeues:
          serial_read_data copies r.data → local buf
          protocomm_pserial_ctrl_req_handler()
            parse_tlv() → extracts epname + data bytes
            protocomm_req_handle() → calls rpc_ll_slist_req_handler()
              esp_hosted_proto_extract_msg_id() → msg_id
              esp_hosted_cp_ext_rpc_dispatch_req(msg_id, inbuf, inlen,
                                                  &resp, &resp_len)
                binary search in s_req_table → calls handler
              compose_tlv(epname, resp, resp_len)
              serial_write_data(tlv_buf, tlv_len)
                → send_to_host_queue() → transport TX
```

### 5.4 Serial TX path (event: CP → host)

```
extension calls:
  esp_hosted_cp_ext_rpc_send_event(event_id, data, len)
    → s_evt_table lookup → serialise(ctx, &params) → encoded bytes
    → esp_hosted_cp_protocomm_process_rpc_evt(g_rpc_evt_ep,
                                               event_id, out, out_len)
        → protocomm_rpc_evt_ind → pserial_task
          → protocomm_pserial_ctrl_evnt_handler()
            → compose_tlv(evt_ep, out, out_len)
            → serial_write_data() → send_to_host_queue()
```

---

## 6. Public API of `esp_hosted_cp_ext_rpc`

Exposed in `include/esp_hosted_cp_ext_rpc.h`.
This replaces the Registry 3 API that was in `esp_hosted_cp_core.h`.

```c
/* ── Lifecycle (called automatically via EH_CP_EXT_REGISTER) ─────────────── */
/* Not called directly by application code. */

/* ── RPC Request table ───────────────────────────────────────────────────── */

/**
 * Register a request handler for msg_ids in [id_min, id_max] (inclusive).
 * Replaces: esp_hosted_cp_rpc_req_register() from core.
 */
esp_err_t esp_hosted_cp_ext_rpc_req_register(uint16_t id_min, uint16_t id_max,
                                              eh_rpc_req_handler_t handler,
                                              void *ctx);

esp_err_t esp_hosted_cp_ext_rpc_req_unregister(uint16_t id_min, uint16_t id_max);

/* ── RPC Event table ─────────────────────────────────────────────────────── */

/**
 * Register an event serialiser for event_ids in [id_min, id_max] (inclusive).
 * Replaces: esp_hosted_cp_rpc_evt_register() from core.
 */
esp_err_t esp_hosted_cp_ext_rpc_evt_register(uint16_t id_min, uint16_t id_max,
                                              eh_rpc_evt_serialise_t serialise,
                                              void *ctx);

esp_err_t esp_hosted_cp_ext_rpc_evt_unregister(uint16_t id_min, uint16_t id_max);

/* ── Event send (CP → host) ──────────────────────────────────────────────── */

/**
 * Serialise and send an unsolicited event to the host.
 * Replaces: esp_hosted_cp_rpc_send_event() from core.
 *
 * Looks up the serialise callback for event_id in the evt table,
 * calls serialise() to encode, then transmits via protocomm.
 * Silent drop (ESP_ERR_INVALID_STATE) if protocomm not yet ready.
 */
esp_err_t esp_hosted_cp_ext_rpc_send_event(uint32_t event_id,
                                            const void *data, uint16_t len);

/* ── Readiness ───────────────────────────────────────────────────────────── */

/**
 * Returns true when protocomm is initialised and pserial_task is running.
 * Guards rpc_fg/rpc_mcu init_fn (assert this at start of their init).
 */
bool esp_hosted_cp_ext_rpc_is_ready(void);
```

**Types** (`eh_rpc_req_handler_t`, `eh_rpc_evt_serialise_t`, `eh_rpc_req_params_t`,
`eh_rpc_evt_params_t`) move from `esp_hosted_cp_core.h` into
`esp_hosted_cp_ext_rpc.h`.

---

## 7. Changes to Core's Public API (`esp_hosted_cp_core.h`)

### Removed entirely from core.h:
- All of Registry 3 (`esp_hosted_cp_rpc_req_register`, `_evt_register`,
  `_req_unregister`, `_evt_unregister`, `_dispatch_req`, `_send_event`,
  `_rpc_registries_init`, `_rpc_registry_lock/unlock`)
- `eh_rpc_req_handler_t`, `eh_rpc_evt_serialise_t`, `eh_rpc_req_params_t`,
  `eh_rpc_evt_params_t` typedef block
- `RPC_EP_NAME_REQ`, `RPC_EP_NAME_EVT` defines
- `EH_CP_PROTOCOMM_READY_BIT`

### Added to core.h:
- `ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY` event ID constant
- `esp_hosted_cp_rpc_ep_config_t` struct (two char[] fields, no RPC logic)

### Unchanged in core.h:
- Registry 1 (iface table API)
- Registry 2 (cap accumulator API)
- `EH_CP_EXT_REGISTER` macro + `esp_hosted_ext_desc_t`
- `EH_CP_EXT_INIT_DONE_BIT`
- `esp_hosted_proto_extract_msg_id()`
- `hdr_ver_negotiated`, `rpc_ver_negotiated`

---

## 8. Changes to `esp_hosted_cp_core/CMakeLists.txt`

Remove from `SRCS`:
- `src/esp_hosted_cp_rpc.c`
- `src/esp_hosted_cp_rpc_ll.c`

Remove from `REQUIRES`:
- `protocomm`

Remove from `PRIV_REQUIRES` (if present):
- anything rpc-related

No new REQUIRES added — core has no dependency on `esp_hosted_cp_ext_rpc`.

---

## 9. Changes to `esp_hosted_cp_ext_rpc/CMakeLists.txt`

```cmake
idf_component_register(
    SRCS
        "src/esp_hosted_cp_ext_rpc.c"
        "src/esp_hosted_cp_ext_rpc_registries.c"
        "src/esp_hosted_cp_ext_rpc_serial.c"
        "src/esp_hosted_cp_ext_rpc_ll.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES
        "esp_hosted_cp_core"
        "esp_hosted_common"
        "protocomm"
        "driver"
        "esp_event"
    PRIV_REQUIRES
        "esp_timer"
        "esp_common"
)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${protocomm_dir}/src/common")   # for protocomm_priv.h
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,-u,esp_hosted_cp_ext_rpc_init")
```

---

## 10. Call-site updates in higher extensions

`esp_hosted_cp_ext_rpc_fg` and `esp_hosted_cp_ext_rpc_mcu`:

| Old call (core) | New call (ext_rpc) |
|---|---|
| `esp_hosted_cp_rpc_req_register(...)` | `esp_hosted_cp_ext_rpc_req_register(...)` |
| `esp_hosted_cp_rpc_evt_register(...)` | `esp_hosted_cp_ext_rpc_evt_register(...)` |
| `esp_hosted_cp_rpc_send_event(...)` | `esp_hosted_cp_ext_rpc_send_event(...)` |
| `#include "esp_hosted_cp_core.h"` for types | `#include "esp_hosted_cp_ext_rpc.h"` for types |

Both extensions add `esp_hosted_cp_ext_rpc` to their CMakeLists `REQUIRES`.
Both add `assert(esp_hosted_cp_ext_rpc_is_ready())` at the top of their `init_fn`.

---

## 11. Implementation Steps

| Step | Action | File(s) |
|------|---------|---------|
| I1 | Add `ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY` + `esp_hosted_cp_rpc_ep_config_t` | `esp_hosted_cp_event.h` |
| I2 | Remove Registry 3 block from `core.h`; remove `EH_CP_PROTOCOMM_READY_BIT` | `esp_hosted_cp_core.h` |
| I3 | Rewrite `host_to_slave_reconfig()`: remove protocomm calls, post `EVT_PRIVATE_RPC_READY` | `esp_hosted_cp_core.c` |
| I4 | Rewrite serial branch of `process_rx_pkt()`: reassembly stays, call `dispatch_rx(ESP_SERIAL_IF, r.data, r.len, NULL)` | `esp_hosted_cp_core.c` |
| I5 | Remove `initialise_protocomm_hooks()`, `rpc_init_with_endpoints()`, rpc includes | `esp_hosted_cp_core.c` |
| I6 | Remove `send_event` transport call from `registries.c`; remove `#include rpc_ll.h` | `esp_hosted_cp_registries.c` |
| I7 | Remove `esp_hosted_cp_rpc.c` + `rpc_ll.c` from core SRCS; remove `protocomm` from REQUIRES | `core/CMakeLists.txt` |
| I8 | Create `ext_rpc` directory + CMakeLists | new |
| I9 | Write `ext_rpc_registries.c`: req/evt realloc tables + all registry APIs | new |
| I10 | Write `ext_rpc_serial.c`: `serial_write_data`, `serial_read_data`, `serial_rx_cb` | new |
| I11 | Write `ext_rpc_ll.c` + `ext_rpc_ll.h` (private): protocomm/pserial substrate | moved+renamed |
| I12 | Write `ext_rpc.c`: `EH_CP_EXT_REGISTER(priority=5)`, `init_fn`, event handler, `is_ready`, `send_event` | new |
| I13 | Write `ext_rpc.h`: public API | new |
| I14 | Update `rpc_fg` + `rpc_mcu`: call-site renames + add REQUIRES + add assert | existing |
| I15 | `idf.py build` — exit gate | — |
