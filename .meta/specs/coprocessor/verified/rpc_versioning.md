<!-- %% sp.co.ve-rv.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# RPC Versioning

<!-- %% sp.co.ve-rv.ver.o %% - always -->
## Version Codes

| Code | Name | Transport |
|------|------|-----------|
| 0x01 | V1 | Protobuf via protocomm endpoints `RPCReq`/`RPCRsp`/`RPCEvt`; separate FG + MCU schemas |
| 0x02 | V2 | Single unified schema; endpoints `RPCReqV2`/`RPCEvtV2`; msg_id 0x2000–0x7FFF (deferred) |
<!-- %% sp.co.ve-rv.ver.c %% -->

<!-- %% sp.co.ve-rv.neg.o %% - always -->
## Negotiation

```
CP boot → TLV 0x22 (ESP_PRIV_RPC_VERSION) = 0x02
Host ACK → TLV 0x23 (ESP_PRIV_RPC_VERSION_ACK) = min(host_max, cp_max)
CP commits → rpc_ver_negotiated = agreed value
```

- Old host (no ACK TLV) → `rpc_ver_negotiated` stays V1. No breakage.
- `rpc_ver_negotiated` lives in `cp_core.c`, init = V1.
<!-- %% sp.co.ve-rv.neg.c %% -->

<!-- %% sp.co.ve-rv.ns.o %% - always -->
## RPC msg_id Namespaces

| Range | Proto | Variant |
|-------|-------|---------|
| 100–399 (decimal) | `CtrlMsg` | V1 Linux FG |
| 0x100–0x3FF | `Rpc` | V1 MCU |
| 0x2000–0x3FFE | unified | V2 requests (deferred) |
| 0x4000–0x5FFE | unified | V2 responses (deferred) |
| 0x6000–0x7FFE | unified | V2 events (deferred) |
<!-- %% sp.co.ve-rv.ns.c %% -->

<!-- %% sp.co.ve-rv.compat.o %% - context -->
## Backward Compat Model

Three CP wire fingerprints at boot:
- `V2 default` — sends 0x20–0x23 negotiation TLVs
- `V1_LINUX compat` — sends legacy FG TLVs 0x00–0x04 only
- `V1_MCU compat` — sends MCU TLVs 0x11–0x19, no negotiation TLVs

Compat Kconfig is CP-side only (`CONFIG_ESP_HOSTED_CP_COMPAT_MODE`). Host auto-detects from TLVs present in startup event. Host never needs recompile to talk to old CP.
<!-- %% sp.co.ve-rv.compat.c %% -->

<!-- %% sp.co.ve-rv.c %% -->
