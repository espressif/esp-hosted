<!-- %% sp.sy.ve-ma.o %% - context -->
---
type: spec
last_verified: 2026-03-30
---

# Memory Audit

<!-- %% sp.sy.ve-ma.bugs.o %% - always -->
## Bugs Found & Fixed

| ID | Location | Issue | Fix |
|----|----------|-------|-----|
| MEM-001 | `mcu_rpc_req_handler` | Double free of response buffer on error path | Single-owner handoff; caller frees only on success |
| MEM-002 | `compose_tlv` | Leak when intermediate allocation fails | `goto cleanup` with unified free path |
<!-- %% sp.sy.ve-ma.bugs.c %% -->

<!-- %% sp.sy.ve-ma.budget.o %% - context -->
## Allocation Budget

| Metric | Value |
|--------|-------|
| Per-request peak heap | ~8.3 KB |
| Steady-state heap (tasks + queues) | ~8.5 KB |
| TLV response buffer lifetime | Owned by caller; freed after TX confirm |

Per-request peak includes: RPC request protobuf decode, response encode, TLV wrap. No persistent per-request allocation after response sent.
<!-- %% sp.sy.ve-ma.budget.c %% -->

<!-- %% sp.sy.ve-ma.c %% -->
