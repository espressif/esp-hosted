<!-- %% sp.co.nv-pr.o %% - context -->
---
type: spec
status: unverified
last_verified: never
---

# Protocomm Removal Analysis

> Unverified — do not implement for V1 without scope.md approval.

<!-- %% sp.co.nv-pr.use.o %% - context -->
## What Protocomm Currently Provides

Only 3 protocomm functions used in `rpc_ll.c`:
1. `protocomm_new()` / `protocomm_delete()`
2. `protocomm_add_endpoint(name, handler, priv)`
3. `protocomm_remove_endpoint(name)`

Plus transport binding: `protocomm_transport_start()` / `stop()` — one call each.
<!-- %% sp.co.nv-pr.use.c %% -->

<!-- %% sp.co.nv-pr.rep.o %% - context -->
## Replacement (~200 lines portable C)

```c
// Replace protocomm endpoint table with:
typedef struct { const char *name; pc_handler_t fn; void *priv; } ep_entry_t;
static ep_entry_t ep_table[EP_MAX];

// Replace transport binding with direct serial registration:
// serial_if_register_ep("RPCReq", rpc_req_cb, NULL);
```

Wire format unchanged — same endpoint names, same protobuf encoding. Host side unaffected.
<!-- %% sp.co.nv-pr.rep.c %% -->

<!-- %% sp.co.nv-pr.dec.o %% - ignore -->
## Decision

Remove protocomm: eliminates IDF component dependency from `ext_rpc`, enables non-IDF hosts, reduces ROM by ~4KB. Risk: low (wire compat unchanged). Scheduled post-V1.
<!-- %% sp.co.nv-pr.dec.c %% -->

<!-- %% sp.co.nv-pr.c %% -->
