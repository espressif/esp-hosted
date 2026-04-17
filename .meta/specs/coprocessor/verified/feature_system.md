<!-- %% sp.co.ve-fs.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# Extension System

<!-- %% sp.co.ve-fs.what.o %% - always -->
## What Lives Where

**Core (`eh_cp`)** owns:
- Registries 1–2 (iface table, cap accumulator)
- Auto-init iterator (linker section walk)
- PRIV handshake state (`hdr_ver_negotiated`, `rpc_ver_negotiated`)
- Serial RX demux (dispatch by `if_type`)
- `ESP_HOSTED_CP_EVT_PRIV_INIT` event

**`eh_cp_feat_rpc` (priority 100)** owns:
- Registry 3 (RPC dynamic table)
- protocomm endpoint registration (`RPCReq`, `RPCRsp`, `RPCEvt`)
- `ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY` event
- Serial RX → RPC path

**`_feat_` extensions (priority 200+)** own:
- Feature-specific RPC handlers (registered via Registry 3)
- Own capability bits (registered via Registry 2 API)
- Own tasks/timers (created in `init()`, destroyed in `deinit()`)

## State Ownership & Globals

- Prefer **module-local `static` state** inside the owning component.
- Avoid exposing globals across components; expose **minimal APIs** only when cross-component coordination is required.
- RPC/proto layers should remain **dumb relay/translation**; they must not own feature state or policy.
- If a feature needs shared state, the owning extension should provide a **small, stable API** for it.
<!-- %% sp.co.ve-fs.what.c %% -->

<!-- %% sp.co.ve-fs.api.o %% - always -->
## Extension Descriptor API

```c
typedef struct {
    esp_err_t (*init_fn)(void);
    esp_err_t (*deinit_fn)(void);
    const char *name;
    BaseType_t affinity;
    int priority;               // lower = init earlier; 100 = rpc, 200+ = feat
} eh_cp_feat_desc_t;

// Registration (in .c file, expands to static const struct in linker section):
EH_CP_FEAT_REGISTER(my_init, my_deinit, "my_ext", tskNO_AFFINITY, 200);

// Cap bit registration (called inside init() on success):
eh_cp_add_feat_cap_bits(ext_caps_bits);
eh_cp_add_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, feature_bitmask);

// Cap bit clear (called inside deinit() or init() on failure):
eh_cp_clear_feat_cap_bits(ext_caps_bits);
eh_cp_clear_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, feature_bitmask);

// RPC handler registration (priority-100 feature, called after RPC_READY):
eh_cp_rpc_req_register(MSG_ID_MIN, MSG_ID_MAX, req_handler, ctx);
eh_cp_rpc_evt_register(MSG_ID_MIN, MSG_ID_MAX, evt_handler, ctx);
```
<!-- %% sp.co.ve-fs.api.c %% -->

<!-- %% sp.co.ve-fs.deps.o %% - context -->
## Component Dependency Graph

```
eh_cp_feat_*  →  eh_cp_feat_rpc  →  eh_cp_core
                                                      →  protocomm
eh_cp_core             →  eh_cp_transport
                          →  eh_proto (serializers)
```

Extension `CMakeLists.txt` must `REQUIRES eh_cp_feat_rpc` (not core directly) if it registers RPC handlers.
<!-- %% sp.co.ve-fs.deps.c %% -->

<!-- %% sp.co.ve-fs.seq.o %% - context -->
## Boot Sequence

```
transport init → generate_startup_event() (builds TLV caps)
core init → walk linker section, sort by priority, call init_fn() in order
  priority 100: ext_rpc init → protocomm start, endpoints registered
                → post ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY
  priority 200+: feat inits → rpc handlers registered, cap bits set
host sends PRIV ACK → core commits negotiated versions
```
<!-- %% sp.co.ve-fs.seq.c %% -->

<!-- %% sp.co.ve-fs.c %% -->
