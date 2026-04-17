<!-- %% sp.co.ve-rd.o %% - always -->
---
type: spec
last_verified: 2026-03-31
---

# RPC Dispatch

<!-- %% sp.co.ve-rd.reg.o %% - always -->
## Three Registries

**Registry 1 — Interface RX/TX table** (`eh_cp_registries.c`)
Flat array indexed by `eh_if_type_t`. Zero-init (BSS).
```c
typedef struct { hosted_rx_cb_t rx; hosted_tx_cb_t tx; } iface_entry_t;
```

**Registry 2 — Capability accumulator** (`eh_cp_registries.c`)
Single `uint32_t` bitmask. Extensions OR in their bits at init.
```c
void cp_cap_bit_set(uint32_t bit);
void cp_cap_bit_clear(uint32_t bit);
uint32_t cp_cap_bits_get(void);
```

**Registry 3 — RPC dynamic table** (`eh_cp_rpc.c`)
`realloc`-grown array sorted by `msg_id`. Binary search O(log n). Grows/shrinks ±4 slots.
```c
esp_err_t cp_rpc_req_register(uint16_t msg_id, rpc_req_handler_t fn);
esp_err_t cp_rpc_evt_register(uint16_t msg_id, rpc_evt_handler_t fn);
esp_err_t cp_rpc_req_unregister(uint16_t msg_id);
```
<!-- %% sp.co.ve-rd.reg.c %% -->

<!-- %% sp.co.ve-rd.flow.o %% - always -->
## RPC Request Call Flow (V1)

**Linux FG host → CP:**
```
Host user-space ctrl_api.c
  → serial_if.c (serialize to protobuf CtrlMsg)
  → Linux kmod esp_serial.c
  → transport (SPI/SDIO) frame TX
CP transport RX
  → cp_core serial demux (if_type == ESP_PRIV_IF → rpc path)
  → protocomm endpoint "RPCReq"
  → ext_rpc req handler
  → Registry 3 binary search on msg_id
  → registered handler fn()
  → response encoded, sent back via "RPCRsp"
```

**MCU host → CP:**
```
eh_host_rpc_lib ctrl_api.c
  → serialize to Rpc protobuf
  → transport_drv TX frame
CP transport RX → same Registry 3 path
```
<!-- %% sp.co.ve-rd.flow.c %% -->

<!-- %% sp.co.ve-rd.files.o %% - context -->
## Key Files

| File | Role |
|------|------|
| `modules/coprocessor/eh_cp_core/src/eh_cp_core.c` | Core init, event loop, serial demux |
| `modules/coprocessor/eh_cp_core/src/eh_cp_registries.c` | Registry 3 impl |
| `modules/coprocessor/features/eh_cp_feat_rpc_ext_linux/` | FG RPC extension (handlers) |
| `modules/coprocessor/features/eh_cp_feat_rpc_ext_mcu/` | MCU RPC extension (handlers) |
| `fg/host/components/eh_host_rpc_lib/src/ctrl_api.c` | Host RPC API |
<!-- %% sp.co.ve-rd.files.c %% -->

<!-- %% sp.co.ve-rd.c %% -->
