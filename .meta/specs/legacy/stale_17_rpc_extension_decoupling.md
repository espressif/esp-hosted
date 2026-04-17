# 17 — RPC Extension Decoupling: `esp_hosted_cp_ext_rpc`

> Status: **Design Proposed** — not yet implemented.
> Relates to: spec 01 (architecture), spec 12 (RPC call flow), spec 14 (design decisions D12), spec 16 (linker auto-init).
> Addresses: protocomm race condition, pserial ownership, endpoint-name negotiation, core bloat, NG forward-compatibility.

---

## 1. Problem Statement

### 1.1 The Race Condition

`"Protocomm not initialized, drop serial packet"` is logged when an event
(WiFi, heartbeat, ESPInit) fires before the host has completed the PRIV handshake.

Current boot sequence:

```
esp_hosted_cp_init()
  └─ ext_init_task (priority 100: rpc_mcu, rpc_fg)
       └─ registers RPC dispatch table entries
       └─ subscribes to WiFi events            ← WiFi events fire NOW
       └─ sets EH_CP_EXT_INIT_DONE_BIT
         └─ host_reset_task: generate_startup_event()
              └─ host sends back ESP_PRIV_EVENT_INIT
                   └─ host_to_slave_reconfig()
                        └─ initialise_protocomm_hooks()
                             └─ esp_hosted_cp_rpc_init_transport()
                                  └─ protocomm init  ← LATE
```

Any event fired between extension init and handshake completion is silently dropped.

### 1.2 Core Owns Things It Should Not

`esp_hosted_cp_rpc_ll.c` and `esp_hosted_cp_rpc.c` both live in `esp_hosted_cp_core`.
This means core directly depends on `protocomm`, owns a FreeRTOS task (`pserial_task`),
owns TLV framing, owns endpoint state — none of which belongs in "core". Any
future NG variant that does not use RPC carries all this code with no opt-out.

### 1.3 Endpoint Name Negotiation is Deferred (by design)

You plan to advertise CP endpoint names in the PRIV TLV. The host either accepts
them or proposes alternatives. The confirmed names are not known until the host
sends back `ESP_PRIV_EVENT_INIT`. This means protocomm cannot be initialised
with *final* names at boot — it must start with defaults and support atomic
upgrade after the handshake.

---

## 2. What Lives Where Today: `rpc.c` vs `rpc_ll.c`

Understanding the exact responsibilities before moving anything:

### `esp_hosted_cp_rpc.c` — Serial transport bridge (lives in core)

| Function | Role |
|----------|------|
| `serial_write_data()` | Writes frames to host via `send_to_host_queue()` |
| `serial_read_data()` | Reads from static `rx_data` buffer |
| `esp_hosted_cp_process_serial_rx_pkt()` | Fragment reassembly; drops if not ready |
| `esp_hosted_cp_rpc_init_transport()` | Calls `protocomm_init` — **this call moves** |
| `esp_hosted_cp_process_rpc_evt()` | Legacy API wrapper |

### `esp_hosted_cp_rpc_ll.c` — Protocomm substrate (moves to `esp_hosted_cp_ext_rpc`)

| Code | Role |
|------|------|
| `struct pserial_config` | FreeRTOS queue + xmit/recv callbacks |
| `pserial_task()` | The core FreeRTOS task: dequeues work, calls req or evt handler |
| `protocomm_pserial_start/stop()` | Starts/stops `pserial_task` + queue |
| `parse_tlv()` / `compose_tlv()` | TLV framing (private helpers) |
| `protocomm_pserial_ctrl_req_handler()` | Parse req TLV → `protocomm_req_handle()` → compose resp TLV → xmit |
| `protocomm_pserial_ctrl_evnt_handler()` | Compose evt TLV → `protocomm_req_handle()` → xmit |
| `protocomm_pserial_data_ready()` | Enqueue incoming data to `req_queue` |
| `rpc_ll_slist_req_handler()` | Protocomm endpoint handler; extracts msg_id, calls `esp_hosted_cp_rpc_dispatch_req()` |
| `rpc_ll_slist_evt_handler()` | Protocomm endpoint handler; passes pre-encoded bytes through |
| `g_rpc_req_ep` / `g_rpc_evt_ep` | Current endpoint name strings |
| `esp_hosted_cp_protocomm_init()` | Allocates `protocomm_t`, registers endpoints, starts `pserial_task` |
| Endpoint registry (legacy) | `g_endpoint_list[]`, `g_endpoint_mutex` |

### Where `pserial_task` belongs

`pserial_task` is the processing backbone of `rpc_ll.c`. It owns the queue, calls
`recv()` to read request data, calls `xmit()` to write responses and events, and
coordinates with `protocomm_req_handle()`. Every reference it makes is to
`pserial_config`, `protocomm_t`, `parse_tlv`, `compose_tlv`, or the two
`ctrl_req/evnt_handler` functions — all of which are private to `rpc_ll`.

**`pserial_task` must move with `rpc_ll.c` into `esp_hosted_cp_ext_rpc`.** There
is no scenario where it belongs in core.

---

## 3. Proposed Architecture: `esp_hosted_cp_ext_rpc`

### 3.1 New Component

```
components/coprocessor/extensions/
  esp_hosted_cp_ext_rpc/
    CMakeLists.txt
    include/
      esp_hosted_cp_ext_rpc.h        ← 5-function public API (see §4)
    src/
      esp_hosted_cp_ext_rpc.c        ← EH_CP_EXT_REGISTER priority 5; init/deinit
      esp_hosted_cp_ext_rpc_ll.c     ← rpc_ll.c moved here (pserial_task lives here)
```

`rpc_ll.c` and `rpc_ll.h` are not split further. `_ll` correctly means
"low-level protocomm + pserial substrate". The header becomes
`esp_hosted_cp_ext_rpc_ll.h` and is `PRIV_REQUIRES` — invisible outside the component.

### 3.2 Priority: 5

```c
EH_CP_EXT_REGISTER(esp_hosted_cp_ext_rpc_init,
                   esp_hosted_cp_ext_rpc_deinit,
                   "ext_rpc", tskNO_AFFINITY, 5);
```

This runs before `rpc_mcu`/`rpc_fg` (priority 100) and all `_feat_` extensions
(priority 200+). By the time any priority-100 extension subscribes to WiFi events,
protocomm is initialised and `pserial_task` is running.

### 3.3 Two-Phase Endpoint Initialisation

Protocomm is initialised at priority 5 with **default** endpoint names
(`"RPCRsp"` / `"RPCEvt"`). The endpoint names may change after the PRIV
handshake completes. This two-phase model works safely because:

**Phase 1 — early init (priority 5):**
- `protocomm_t` allocated, `pserial_task` started
- Default endpoints registered: `"RPCRsp"` → `rpc_ll_slist_req_handler`,
  `"RPCEvt"` → `rpc_ll_slist_evt_handler`
- Events emitted during extensions init use `g_rpc_evt_ep = "RPCEvt"` — the
  endpoint is registered → they go through correctly
- **No host traffic yet** (host hasn't even received `generate_startup_event()`
  at this point — transport isn't open to the host until after all extensions init)

**Phase 2 — endpoint upgrade (after PRIV handshake):**
- Host sends back `ESP_PRIV_EVENT_INIT` → `host_to_slave_reconfig()` →
  calls `esp_hosted_cp_ext_rpc_set_endpoints(req_ep, evt_ep)`
- If names match defaults: no-op
- If names differ (e.g. host proposes `"ctrlResp"`):
  - Take `g_endpoint_mutex`
  - `protocomm_remove_endpoint(old_req_ep)` + `protocomm_add_endpoint(new_req_ep, ...)`
  - Update `g_rpc_req_ep` / `g_rpc_evt_ep`
  - Release mutex
- From this point, both CP and host use the negotiated names

**Why there is no race between phase 1 and phase 2:**
Host RPC requests cannot arrive before phase 2 is complete because the host only
starts sending RPC requests after it has successfully processed the startup event
and confirmed the endpoint names. The PRIV handshake is the gate. Events in the
reverse direction (CP → host) are fine because they use `g_rpc_evt_ep` which is
always the currently-active name — both the sender and the endpoint table agree.

---

## 4. Public API of `esp_hosted_cp_ext_rpc`

Exposed in `include/esp_hosted_cp_ext_rpc.h` — 5 functions only:

```c
/**
 * @brief  Is the RPC layer fully initialized and pserial_task running?
 *
 * Guards core's serial packet processing path.
 * Replaces: (NULL == esp_hosted_cp_protocomm_get_instance())
 */
bool esp_hosted_cp_ext_rpc_is_ready(void);

/**
 * @brief  Feed a reassembled serial request frame into TLV dispatch.
 *
 * Called by core after fragment reassembly in
 * esp_hosted_cp_process_serial_rx_pkt(). Enqueues into pserial_task queue.
 * Replaces: esp_hosted_cp_protocomm_process_rpc_req(buf, len)
 */
esp_err_t esp_hosted_cp_ext_rpc_process_req(const uint8_t *buf, int len);

/**
 * @brief  Send a serialised event frame to the host.
 *
 * Called by esp_hosted_cp_rpc_send_event() (registries.c) after
 * the extension serialiser has produced packed proto bytes.
 * Enqueues into pserial_task event queue.
 * Replaces: esp_hosted_cp_protocomm_process_rpc_evt(ep, id, data, len)
 */
esp_err_t esp_hosted_cp_ext_rpc_send_evt(const char *ep, int event_id,
                                          const void *data, int size);

/**
 * @brief  Set (or upgrade) the endpoint names used by pserial_task.
 *
 * Called at two points in time:
 *   (a) implicitly during init_fn, with default names "RPCRsp"/"RPCEvt"
 *   (b) by host_to_slave_reconfig() after PRIV handshake confirms final names
 *
 * If names are unchanged from current: no-op.
 * If names change: atomically re-registers the protocomm endpoints under
 * the new names and updates g_rpc_req_ep / g_rpc_evt_ep.
 *
 * Thread-safe: takes g_endpoint_mutex.
 *
 * Replaces: rpc_init_with_endpoints() / initialise_protocomm_hooks()
 */
esp_err_t esp_hosted_cp_ext_rpc_set_endpoints(const char *req_ep,
                                               const char *evt_ep);

/**
 * @brief  Get the currently active event endpoint name.
 *
 * Used by registries.c send_event() to tag outgoing event frames.
 * Replaces: esp_hosted_cp_rpc_get_evt_ep()
 */
const char *esp_hosted_cp_ext_rpc_get_evt_ep(void);
```

### What is NOT exposed (stays private to the component)

- `protocomm_t *` — completely hidden
- `struct pserial_config` — hidden
- `pserial_task()` — hidden; it is a `static void` function
- `parse_tlv()` / `compose_tlv()` — hidden
- `protocomm_pserial_ctrl_req/evnt_handler()` — hidden
- `protocomm_pserial_data_ready()` / `protocomm_rpc_req/evt_ind()` — hidden
- `rpc_ll_slist_req/evt_handler()` — hidden
- `esp_hosted_cp_protocomm_init/deinit()` — hidden (called only from `init_fn`)
- `g_endpoint_list[]` / `g_endpoint_mutex` — hidden

The invariant: **callers outside the component never see a `protocomm_t *` or
touch `pserial_task` directly**.

---

## 5. Component Dependency Graph (After Change)

```
esp_hosted_cp_core
  ├─ REQUIRES: esp_hosted_common, esp_hosted_frame, esp_hosted_caps
  ├─ PRIV_REQUIRES: esp_wifi, esp_event, freertos
  └─ NO protocomm dependency  ← core is now protocomm-free

esp_hosted_cp_ext_rpc                              [priority 5]
  ├─ REQUIRES: esp_hosted_cp_core  (send_to_host_queue, registry dispatch)
  ├─ PRIV_REQUIRES: protocomm, freertos
  └─ exposes: esp_hosted_cp_ext_rpc.h  (5 functions)

esp_hosted_cp_ext_rpc_mcu                          [priority 100]
  ├─ REQUIRES: esp_hosted_cp_core
  ├─ REQUIRES: esp_hosted_cp_ext_rpc  (send_evt, is_ready, get_evt_ep)
  └─ PRIV_REQUIRES: esp_hosted_proto_mcu_v1, esp_wifi, esp_event

esp_hosted_cp_ext_rpc_fg                           [priority 100]
  ├─ REQUIRES: esp_hosted_cp_core
  ├─ REQUIRES: esp_hosted_cp_ext_rpc
  └─ PRIV_REQUIRES: esp_hosted_proto_linux_v1, esp_wifi, esp_event

esp_hosted_cp_ext_feat_*                           [priority 200+]
  └─ REQUIRES: esp_hosted_cp_core  (no protocomm dependency)
```

**Hard invariant:** `protocomm` appears in exactly one component's dependency list.

---

## 6. Boot Sequence After Change

```
esp_hosted_cp_init()
  └─ ext_init_task spawned (FreeRTOS task)

ext_init_task (sorted ascending by priority field):

  ┌─ [priority 5]  esp_hosted_cp_ext_rpc::init_fn()
  │    ├─ protocomm_t allocated
  │    ├─ pserial_task started (FreeRTOS task, owns the queue)
  │    ├─ "RPCRsp" + "RPCEvt" registered as default endpoints
  │    ├─ (if LEGACY_COMPAT=y: "ctrlResp" + "ctrlEvnt" also registered)
  │    └─ g_rpc_ready = true
  │                            ↑ PROTOCOMM READY. PSERIAL_TASK RUNNING.
  │
  ├─ [priority 100] esp_hosted_cp_ext_rpc_mcu::init_fn()
  │    ├─ assert(esp_hosted_cp_ext_rpc_is_ready())  ← catches priority bugs
  │    ├─ registers RPC dispatch table entries [0x101..0x183]
  │    └─ subscribes to WiFi events  ← safe: pserial_task running
  │
  ├─ [priority 100] esp_hosted_cp_ext_rpc_fg::init_fn()
  │    ├─ assert(esp_hosted_cp_ext_rpc_is_ready())
  │    ├─ registers RPC dispatch table entries [101..128]
  │    └─ subscribes to WiFi events  ← safe
  │
  ├─ [priority 200] esp_hosted_cp_ext_feat_host_ps::init_fn()
  ├─ [priority 200] esp_hosted_cp_ext_feat_nw_split::init_fn()
  └─ sets EH_CP_EXT_INIT_DONE_BIT

host_reset_task:
  ├─ waits on EH_CP_EXT_INIT_DONE_BIT
  ├─ generate_startup_event()
  │    └─ includes proposed endpoint names in PRIV TLV
  │
  └─ host processes startup event, sends back ESP_PRIV_EVENT_INIT
       └─ host_to_slave_reconfig() parses TLV
            └─ esp_hosted_cp_ext_rpc_set_endpoints(confirmed_req_ep, confirmed_evt_ep)
                 └─ if names unchanged: no-op
                 └─ if names changed: atomic endpoint swap under g_endpoint_mutex
                      └─ g_rpc_req_ep / g_rpc_evt_ep updated
                      └─ protocomm endpoints re-registered
            └─ (from this point: host sends RPC requests with confirmed ep name)
```

**Endpoint upgrade is safe** because:
- No host RPC requests arrive before the handshake completes
- In-flight events use `g_rpc_evt_ep` under the same mutex — atomic relative to upgrade
- After upgrade, all parties agree on the final names

---

## 7. Handling the Endpoint Negotiation: Detailed Design

### What CP advertises in `generate_startup_event()`

Add a new TLV in the startup event payload:

```
Tag: ESP_PRIV_RPC_EP_NAMES  (new TLV type, e.g. 0x24)
Value: <req_ep_len:1><req_ep:N><evt_ep_len:1><evt_ep:M>
```

CP sends its preferred names (`"RPCRsp"`, `"RPCEvt"` by default, or
`"ctrlResp"`, `"ctrlEvnt"` for legacy FG compat mode).

### What host acknowledges in `ESP_PRIV_EVENT_INIT`

Host either:
- **Accepts**: echoes the same names back as `ESP_PRIV_RPC_EP_ACK`
- **Proposes different**: sends `ESP_PRIV_RPC_EP_ACK` with its own preferred names

`host_to_slave_reconfig()` reads `ESP_PRIV_RPC_EP_ACK` and calls
`esp_hosted_cp_ext_rpc_set_endpoints()`. The function signature:

```c
esp_err_t esp_hosted_cp_ext_rpc_set_endpoints(const char *req_ep,
                                               const char *evt_ep);
```

### What `set_endpoints()` does internally

```c
esp_err_t esp_hosted_cp_ext_rpc_set_endpoints(const char *req_ep,
                                               const char *evt_ep)
{
    if (!g_rpc_ready || !g_protocomm_instance) {
        /* Called before init — store as pending defaults */
        strncpy(g_rpc_req_ep, req_ep, EPNAME_MAX - 1);
        strncpy(g_rpc_evt_ep, evt_ep, EPNAME_MAX - 1);
        return ESP_OK;
    }

    xSemaphoreTake(g_endpoint_mutex, portMAX_DELAY);

    bool req_changed = (strcmp(g_rpc_req_ep, req_ep) != 0);
    bool evt_changed = (strcmp(g_rpc_evt_ep, evt_ep) != 0);

    if (req_changed) {
        protocomm_remove_endpoint(g_protocomm_instance, g_rpc_req_ep);
        strncpy(g_rpc_req_ep, req_ep, EPNAME_MAX - 1);
        protocomm_add_endpoint(g_protocomm_instance, g_rpc_req_ep,
                               rpc_ll_slist_req_handler, NULL);
        ESP_LOGI(TAG, "REQ endpoint upgraded: %s", g_rpc_req_ep);
    }
    if (evt_changed) {
        protocomm_remove_endpoint(g_protocomm_instance, g_rpc_evt_ep);
        strncpy(g_rpc_evt_ep, evt_ep, EPNAME_MAX - 1);
        protocomm_add_endpoint(g_protocomm_instance, g_rpc_evt_ep,
                               rpc_ll_slist_evt_handler, NULL);
        ESP_LOGI(TAG, "EVT endpoint upgraded: %s", g_rpc_evt_ep);
    }

    xSemaphoreGive(g_endpoint_mutex);
    return ESP_OK;
}
```

### Why there is no window of broken names

The only moment when ep names matter for *incoming* traffic (host → CP) is
during active RPC sessions. Active RPC sessions only start after the host
has received the startup event, parsed it, confirmed the ep names, and sent
`ESP_PRIV_EVENT_INIT` back. By the time the first real RPC request arrives,
`set_endpoints()` has already run. There is no window.

For *outgoing* events (CP → host): events use `g_rpc_evt_ep`. These may fire
during the handshake. The host has not confirmed the ep name yet, so it does
not have a registered listener for events during this window anyway — the events
are transmitted over the wire tagged with the default name, which is what the
host will confirm. If the host *changes* the name, events emitted during the
handshake with the old name are pre-handshake traffic that the host does not
interpret as RPC events yet. This is acceptable — same as today's behaviour.

---

## 8. Core Simplifications

After this change, `esp_hosted_cp_core` loses:

| Removed | Moved to |
|---------|----------|
| `rpc_ll.c` + `rpc_ll.h` | `esp_hosted_cp_ext_rpc/src/esp_hosted_cp_ext_rpc_ll.c` (private) |
| `pserial_task` FreeRTOS task | Stays in `rpc_ll.c`, now inside `ext_rpc` |
| `esp_hosted_cp_protocomm_init()` call | `ext_rpc` priority-5 `init_fn` |
| `initialise_protocomm_hooks()` | Becomes `ext_rpc_set_endpoints()` call |
| `rpc_init_with_endpoints()` static helper | Removed |
| `protocomm` CMake dependency | Removed from core |
| `g_protocomm_instance` NULL guard | Replaced by `ext_rpc_is_ready()` |
| `esp_hosted_cp_rpc_init_transport()` | Removed from `rpc.c` |
| `g_rpc_req_ep` / `g_rpc_evt_ep` globals | Moved into `rpc_ll.c` inside `ext_rpc` |

What remains in core's `rpc.c`:
- `serial_write_data()` / `serial_read_data()` — serial I/O bridge
- `esp_hosted_cp_process_serial_rx_pkt()` — fragment reassembly; NULL guard → `is_ready()` check
- Legacy `esp_hosted_send_event_to_host()` wrapper if `EH_CP_LEGACY_SEND_EVENT_TO_HOST_API`

What core's `rpc.c` calls outbound:
- `esp_hosted_cp_ext_rpc_is_ready()` — guard
- `esp_hosted_cp_ext_rpc_process_req(buf, len)` — dispatch incoming req
- `esp_hosted_cp_ext_rpc_get_evt_ep()` — get current evt ep name for legacy path

What `registries.c` calls for events:
- `esp_hosted_cp_ext_rpc_send_evt(ep, id, data, len)` — dispatch outgoing event
- `esp_hosted_cp_ext_rpc_get_evt_ep()` — get current evt ep name

---

## 9. NG Forward-Compatibility

For NG (no RPC): omit `esp_hosted_cp_ext_rpc` from CMake. Core compiles cleanly
with zero protocomm dependency. `esp_hosted_cp_ext_rpc_is_ready()` is defined as
a weak stub returning `false` in `rpc.c`. The registry (what gets dispatched) is
completely independent of the transport (how bytes flow). NG provides its own
transport extension at whatever priority it chooses.

---

## 10. Risks and Mitigations

| Risk | Severity | Mitigation |
|------|----------|-----------|
| `ext_rpc` init fails (OOM in protocomm alloc) | High | `is_ready()` returns false; serial packets drop with clear LOGE; system broken-loud not broken-silent |
| Priority misconfiguration: `rpc_mcu` runs before `ext_rpc` | Medium | `assert(esp_hosted_cp_ext_rpc_is_ready())` at start of `rpc_mcu` / `rpc_fg` `init_fn` — immediate crash with readable message |
| `set_endpoints()` races with in-flight request | Low | Takes `g_endpoint_mutex`; same mutex held during `protocomm_req_handle()` dispatch — atomic |
| Pre-handshake event uses old ep name | Informational | Host has no listener during this window anyway; acceptable and identical to today |
| `pserial_task` blocked on queue while `set_endpoints()` runs | None | `set_endpoints()` only touches protocomm endpoint table, not the queue — no deadlock path |
| Legacy COMPAT: old FG host needs `"ctrlResp"` registered before first packet | None | `ext_rpc` priority-5 init registers legacy aliases immediately (same as it registers defaults) |

---

## 11. Implementation Steps (Phase G)

| Step | Action |
|------|--------|
| G1 | Create `extensions/esp_hosted_cp_ext_rpc/` directory + CMakeLists |
| G2 | Move `esp_hosted_cp_rpc_ll.c` → `ext_rpc/src/esp_hosted_cp_ext_rpc_ll.c`; move `esp_hosted_cp_rpc_ll.h` → `ext_rpc/src/` (private) |
| G3 | Create `ext_rpc/src/esp_hosted_cp_ext_rpc.c` with `EH_CP_EXT_REGISTER(..., 5)`; `init_fn` calls `esp_hosted_cp_protocomm_init(serial_write_data, serial_read_data)` |
| G4 | Create `ext_rpc/include/esp_hosted_cp_ext_rpc.h` with the 5-function public API |
| G5 | Update `esp_hosted_cp_rpc.c` (core): remove `esp_hosted_cp_rpc_init_transport()`; change NULL guard to `esp_hosted_cp_ext_rpc_is_ready()`; change dispatch call to `esp_hosted_cp_ext_rpc_process_req()` |
| G6 | Update `core.c`: `initialise_protocomm_hooks()` → calls `esp_hosted_cp_ext_rpc_set_endpoints(req, evt)` only; remove `rpc_init_with_endpoints()` |
| G7 | Update `generate_startup_event()`: include `ESP_PRIV_RPC_EP_NAMES` TLV with CP's proposed ep names |
| G8 | Update `host_to_slave_reconfig()`: parse `ESP_PRIV_RPC_EP_ACK`, call `esp_hosted_cp_ext_rpc_set_endpoints()` |
| G9 | Remove `protocomm` from core CMakeLists `REQUIRES`; add to `ext_rpc` CMakeLists |
| G10 | Add `esp_hosted_cp_ext_rpc` to `REQUIRES` in `ext_rpc_mcu` + `ext_rpc_fg` CMakeLists |
| G11 | Add `assert(esp_hosted_cp_ext_rpc_is_ready())` at start of `rpc_mcu` + `rpc_fg` `init_fn` |
| G12 | Add weak-stub `esp_hosted_cp_ext_rpc_is_ready()` returning false in `rpc.c` (for NG builds) |
| G13 | `idf.py build` — Phase G exit gate |

---

## 12. Updated Component Table

| Component | Priority | protocomm dep | Owns |
|-----------|----------|--------------|------|
| `esp_hosted_cp_core` | — | ❌ None | Registries, serial I/O bridge, recv_task |
| `esp_hosted_cp_ext_rpc` | 5 | ✅ Sole owner | `protocomm_t`, `pserial_task`, TLV, endpoint table, ep names |
| `esp_hosted_cp_ext_rpc_mcu` | 100 | ❌ None | MCU protobuf encode/decode, WiFi req handlers |
| `esp_hosted_cp_ext_rpc_fg` | 100 | ❌ None | FG protobuf encode/decode, WiFi req handlers |
| `esp_hosted_cp_ext_feat_*` | 200+ | ❌ None | Feature-specific logic |

---

## 13. What This Spec Updates in Other Docs

| Spec | Change |
|------|--------|
| `01_architecture_overview.md` | ✅ Updated: layer diagram, boot sequence |
| `12_rpc_call_flow.md` | Steps 5–6: `esp_hosted_cp_ext_rpc_process_req()` instead of direct protocomm call |
| `14_design_decisions_v2.md` | ✅ Updated: Decision 12 added, summary table row G1–G13 |
| `09_implementation_status.md` | Add Phase G rows |
