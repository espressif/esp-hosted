# 18 — Protocomm Removal Analysis

> Status: **Design Decision Pending**
> Question: Is removing protocomm entirely the right call?
> Relates to: spec 17 (RPC extension decoupling)

---

## 1. Precise Audit: What Protocomm Actually Provides

After reading every line of `rpc_ll.c` and tracing all calls into `<protocomm.h>`
and `<protocomm_priv.h>`, the complete list of protocomm functions used is:

```c
protocomm_t *protocomm_new();
void          protocomm_delete(protocomm_t *pc);
esp_err_t     protocomm_add_endpoint(pc, name, handler, priv_data);
esp_err_t     protocomm_remove_endpoint(pc, name);
esp_err_t     protocomm_req_handle(pc, epname, session_id,
                                   inbuf, inlen, &outbuf, &outlen);
// Plus internal struct access via <protocomm_priv.h>:
pc->priv            = pserial_cfg;
pc->add_endpoint    = protocomm_pserial_add_ep;   // stub — does nothing
pc->remove_endpoint = protocomm_pserial_remove_ep; // stub — does nothing
```

**What `protocomm_t` actually is** (from IDF source):

```c
struct protocomm {
    protocomm_add_endpoint_fn_t    add_endpoint;    // function pointer
    protocomm_remove_endpoint_fn_t remove_endpoint; // function pointer
    SLIST_HEAD(, protocomm_ep_t)   endpoints;       // linked list of {name, handler, priv}
    void                          *priv;             // opaque pointer
};
```

`protocomm_req_handle()` does: linear `strcmp` scan over `endpoints` list →
call `handler(session_id, inbuf, inlen, &outbuf, &outlen, priv_data)`.

Total IDF code involved: approximately 130 lines across `protocomm.c`.

---

## 2. What protocomm Does That rpc_ll.c Already Does Itself

| protocomm responsibility | rpc_ll.c reality |
|---|---|
| Endpoint linked list: add/remove/find by name | `rpc_ll.c` has `g_endpoint_list[]` realloc array doing the same thing for the legacy API — **parallel duplicate structure** |
| `protocomm_req_handle`: `strcmp` → call handler | `rpc_ll_slist_req_handler` **ignores** the ep name entirely, goes straight to `esp_hosted_cp_rpc_dispatch_req()` by `msg_id`. The name lookup result is thrown away |
| `pc->priv` opaque pointer | Used to attach `pserial_config`. A plain `static struct pserial_config *g_cfg` is equivalent |
| `pc->add_endpoint` / `pc->remove_endpoint` function pointers | Both set to stubs that log and return `ESP_OK`. Pointless indirection |

---

## 3. The Routing Path Today vs Without Protocomm

### Today (with protocomm):

```
Incoming serial frame (reassembled in rpc.c)
  → protocomm_rpc_req_ind()          enqueue to req_queue
  → pserial_task dequeues
  → protocomm_pserial_ctrl_req_handler()
       → parse_tlv()                 extract epname + proto_bytes
       → protocomm_req_handle()      strcmp scan → finds rpc_ll_slist_req_handler
       → rpc_ll_slist_req_handler()  **ignores epname**, extracts msg_id from proto
       → esp_hosted_cp_rpc_dispatch_req()   binary search by msg_id → call handler
       → response encoded by extension
       → compose_tlv()               wrap response in TLV
       → xmit()                      send to host
```

### Without protocomm:

```
Incoming serial frame (reassembled in rpc.c)
  → enqueue to rpc_task queue
  → rpc_task dequeues
  → parse_tlv()                      extract epname + proto_bytes
  → validate epname (strcmp against g_rpc_req_ep / g_rpc_evt_ep)
  → extract msg_id from proto_bytes
  → esp_hosted_cp_rpc_dispatch_req() binary search by msg_id → call handler
  → response encoded by extension
  → compose_tlv()                    wrap response in TLV
  → xmit()                           send to host
```

The only step that disappears is `protocomm_req_handle()` — which was a
linked-list `strcmp` lookup that found a function pointer, which then did
the actual work of ignoring the ep name and going to the registry.

We already write `parse_tlv`, `compose_tlv`, `xmit` ourselves.
We already write the FreeRTOS queue (`struct pserial_config`).
We already write the ep name strings (`g_rpc_req_ep`, `g_rpc_evt_ep`).
Removing protocomm means removing one `strcmp`-based indirection.

---

## 4. The Costs of Keeping Protocomm

### 4.1 IDF coupling
`<protocomm_priv.h>` is an internal IDF header. Accessing `pc->priv` and
`pc->add_endpoint` directly is coupling to IDF internals that can break on
any IDF version bump. This already causes friction in CI.

### 4.2 Portability wall
`protocomm` is an IDF component. It does not exist on Linux userspace, the
kernel module, or NG. Any build target that is not ESP-IDF either excludes
`rpc_ll.c` entirely or needs a porting shim for `protocomm`. This is the
precise reason spec 17 proposes making RPC an extension — so NG can omit it.
Removing protocomm makes the remaining `rpc_ll.c` code portable C: FreeRTOS
queues, `malloc`, `strcmp`, function pointers — all standard.

### 4.3 Duplicate data structures
`protocomm` maintains its own `SLIST` of endpoints. `rpc_ll.c` maintains
`g_endpoint_list[]` alongside it for the legacy `add_endpoint` API (needed
to track names for later `remove_endpoint` calls). Two structures holding
the same data, kept in sync manually.

### 4.4 Dead code paths
`protocomm_pserial_add_ep` and `protocomm_pserial_remove_ep` are stubs.
They are assigned to `pc->add_endpoint` and `pc->remove_endpoint` and never
called meaningfully. This code exists solely to satisfy protocomm's struct
contract.

### 4.5 The ep name lookup is a no-op
The entire value of the protocomm endpoint table is the `strcmp`-based
dispatch. We disabled that by routing everything through two fixed endpoints
(`"RPCRsp"` / `"RPCEvt"`) that immediately ignore the ep name. The
`protocomm_req_handle` call is architectural overhead with no functional value
in the current design.

---

## 5. What Replacing Protocomm Looks Like

The entire replacement is a small `eh_serial_task.c` — approximately 200 lines,
all portable C + FreeRTOS:

### State (replacing `protocomm_t`)

```c
// eh_serial_task.c  (private to esp_hosted_cp_ext_rpc)

typedef esp_err_t (*eh_xmit_fn_t)(uint8_t *buf, ssize_t len);
typedef ssize_t   (*eh_recv_fn_t)(uint8_t *buf, ssize_t len);

typedef struct {
    int          len;
    uint8_t     *data;
    int          msg_id;
    bool         is_event;
    const char  *epname;     // for events: which ep to tag; for reqs: already in TLV
} eh_serial_item_t;

static QueueHandle_t  s_queue;
static eh_xmit_fn_t   s_xmit;
static eh_recv_fn_t   s_recv;
static char           s_req_ep[EPNAME_MAX];
static char           s_evt_ep[EPNAME_MAX];
static bool           s_ready;
```

### Request path

```c
static void handle_req(const uint8_t *tlv_buf, size_t tlv_len)
{
    char     epname[EPNAME_MAX] = {0};
    uint8_t *proto_bytes = NULL;
    size_t   proto_len   = 0;

    // parse_tlv: already in rpc_ll.c, ~30 lines, pure C, no protocomm
    if (parse_tlv(tlv_buf, tlv_len, epname, &proto_bytes, &proto_len) != ESP_OK)
        return;

    // Validate ep name (replaces protocomm linked-list lookup)
    if (strcmp(epname, s_req_ep) != 0
     && strcmp(epname, RPC_EP_NAME_REQ_LEGACY_FG) != 0) {
        ESP_LOGW(TAG, "Unknown ep '%s', drop", epname);
        return;
    }

    // Extract msg_id from proto field 2 — already in registries.c
    uint32_t msg_id = 0;
    esp_hosted_proto_extract_msg_id(proto_bytes, proto_len, &msg_id);

    // Version gate + dispatch — already in rpc_ll_slist_req_handler
    uint8_t  *resp_buf = NULL;
    uint16_t  resp_len = 0;
    esp_hosted_cp_rpc_dispatch_req(msg_id,
                                   proto_bytes, (uint16_t)proto_len,
                                   &resp_buf, &resp_len);

    // compose_tlv + xmit — already in rpc_ll.c
    compose_and_send(epname, resp_buf, resp_len);
}
```

### Event path

```c
static void handle_evt(const char *epname,
                        const uint8_t *proto_bytes, size_t proto_len)
{
    // compose_tlv + xmit — already in rpc_ll.c
    compose_and_send(epname, proto_bytes, proto_len);
}
```

### The task

```c
static void eh_serial_task(void *arg)
{
    eh_serial_item_t item;
    while (xQueueReceive(s_queue, &item, portMAX_DELAY) == pdTRUE) {
        if (item.is_event)
            handle_evt(item.epname, item.data, item.len);
        else
            handle_req(item.data, item.len);  // data already in TLV form
        if (item.data) { free(item.data); }
    }
}
```

**Nothing above requires IDF. Nothing above requires protocomm.**
`parse_tlv`, `compose_tlv`, `esp_hosted_proto_extract_msg_id`,
`esp_hosted_cp_rpc_dispatch_req` are all already written and already
in the codebase. The task loop is structurally identical to `pserial_task`.

---

## 6. Endpoint Name Table: What Replaces `protocomm_add_endpoint`

The legacy `add_endpoint` API is the one place where multiple ep names still
matter. Without protocomm, the endpoint table becomes a tiny array:

```c
typedef struct {
    char                  name[EPNAME_MAX];
    eh_rpc_req_handler_fn handler;  // NULL for evt ep
    bool                  is_event;
} eh_ep_entry_t;

static eh_ep_entry_t s_ep_table[8];  // max 8 endpoints — always < 4 in practice
static uint8_t       s_ep_count;
static SemaphoreHandle_t s_ep_mutex;
```

`eh_ep_lookup(epname)` — linear `strcmp` scan over 2–4 entries.
Same O(n) as `protocomm_req_handle`. Simpler: no linked list, no malloc per node.

The legacy `esp_hosted_cp_protocomm_add_endpoint` API maps to adding a row to
this table. The legacy code path (`CONFIG_ESP_HOSTED_LEGACY_ADD_ENDPOINT_API`)
is Kconfig-guarded and will be removed eventually anyway.

---

## 7. Decision Summary

| Aspect | Keep protocomm | Remove protocomm |
|--------|---------------|-----------------|
| Lines of code saved | 0 | ~130 lines (IDF) + ~80 lines of wrapper/stub in rpc_ll.c |
| IDF version coupling | `<protocomm_priv.h>` direct struct access | None |
| Portability (Linux, NG) | Requires shim or exclusion | Plain C + FreeRTOS — portable |
| Duplicate data structure | Yes (`g_endpoint_list[]` + protocomm SLIST) | No — one table |
| Dead code (stubs) | `protocomm_pserial_add/remove_ep` stubs | Eliminated |
| Functional capability lost | None — `protocomm_req_handle` was being bypassed | None |
| Risk | Low — existing behaviour | Low — exact same logic, no external dependency |
| Effort | Zero | ~200 lines of portable replacement code |
| Future NG compatibility | Needs CMake exclusion + null shim | Inherently portable — nothing to shim |

**Verdict: Yes, removing protocomm simplifies the design.**

The simplification is not dramatic in lines-of-code terms, but it is significant
in correctness and portability terms:
- One data structure instead of two
- No `<protocomm_priv.h>` struct hacking
- No dead stub functions
- Portable across all build targets without CMake exclusions
- The code that replaces it is already 90% written — it is the existing
  `parse_tlv`, `compose_tlv`, `esp_hosted_proto_extract_msg_id`, and
  `esp_hosted_cp_rpc_dispatch_req` assembled into a ~200-line task file

The right time to do this is during Phase G (moving rpc_ll to `esp_hosted_cp_ext_rpc`).
Rather than moving `rpc_ll.c` as-is, rewrite it without the protocomm dependency
at the same time. The external interface does not change at all.

---

## 8. What Must Be Preserved (Wire Compatibility)

Removing protocomm changes **nothing on the wire**. The TLV framing format
(`EPNAME | ep_len | ep_bytes | DATA | data_len | data_bytes`) is defined
by our own `parse_tlv`/`compose_tlv`, not by protocomm. The host never
calls `protocomm_req_handle`. The host speaks raw TLV bytes over serial.

The only thing that changes is the internal routing mechanism:
`protocomm_req_handle` (linked-list `strcmp` → function pointer) is replaced
by a direct `strcmp` against `s_req_ep` / `s_evt_ep` — same result,
one fewer indirection, no heap allocation per endpoint lookup.

---

## 9. Updated Phase G Steps

Replace spec 17 §11 Phase G with these updated steps:

| Step | Action |
|------|--------|
| G1 | Create `extensions/esp_hosted_cp_ext_rpc/` directory + CMakeLists (no `protocomm` in REQUIRES) |
| G2 | Write `ext_rpc/src/eh_serial_task.c`: queue, `parse_tlv`, `compose_tlv`, `handle_req`, `handle_evt`, `eh_serial_task` — ~200 lines, no protocomm |
| G3 | Write `ext_rpc/src/ep_table.c`: tiny flat array replacing `protocomm_add/remove_endpoint` + legacy compat table |
| G4 | Write `ext_rpc/src/esp_hosted_cp_ext_rpc.c`: `EH_CP_EXT_REGISTER(..., 5)` + `init_fn` that starts the task |
| G5 | Write `ext_rpc/include/esp_hosted_cp_ext_rpc.h`: 5-function public API (unchanged from spec 17) |
| G6 | Delete `esp_hosted_cp_rpc_ll.c` from core (do not move — rewrite in ext_rpc without protocomm) |
| G7 | Update `esp_hosted_cp_rpc.c` (core): remove `protocomm_process_rpc_req` call → `esp_hosted_cp_ext_rpc_process_req()` |
| G8 | Update `core.c`: `initialise_protocomm_hooks()` → `esp_hosted_cp_ext_rpc_set_endpoints()` |
| G9 | Remove `protocomm` from core CMakeLists entirely; it appears nowhere |
| G10 | Add `esp_hosted_cp_ext_rpc` to REQUIRES in `ext_rpc_mcu` + `ext_rpc_fg` |
| G11 | Add `assert(esp_hosted_cp_ext_rpc_is_ready())` in `rpc_mcu` + `rpc_fg` init_fn |
| G12 | `idf.py build` — Phase G exit gate |
