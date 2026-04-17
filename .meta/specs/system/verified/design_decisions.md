<!-- %% sp.sy.ve-dd.o %% - context -->
---
type: spec
last_verified: 2026-03-30
---

# Design Decisions

<!-- %% sp.sy.ve-dd.dec.o %% - context -->
## Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | IDF `REQUIRES` for component deps | Enforces clean layering, no implicit includes |
| D2 | Extension auto-init via linker section `EH_CP_FEAT_REGISTER` | Zero coupling: core never imports extension headers |
| D3 | Extensions register own capability bits | Cap ownership stays with the feature, not core |
| D4 | Proto serializers in `common/serializers/` | Shared by CP and host, avoids duplication |
| D5 | Proto encode/decode in extension with explicit `out_max` | Bounds-safe, no hidden allocation |
| D6 | Two RPC ID map files (FG + MCU) | ID spaces don't overlap; separate maps prevent accidental collision |
| D7 | Directory structure: `fg/cp/`, `fg/host/`, `fg/common/` | Clear separation, `fg` = this codebase generation |
| D8 | V2 end-to-end: both FG and MCU hosts implement V2 | No per-host fork of wire format logic |
| D9 | Registry 3 realloc-table ±4 slots | No `sys/queue.h` dep; O(log n) search; simple debug dump |
| D10 | Master config port headers (`EH_CP_*` / `EH_HOST_*`) | Single abstraction layer over IDF/kmod/bare config systems |
| D11 | Ops registry deleted | Overcomplicated; extensions use direct function registration instead |
| D12 | Protocomm lives in `ext_rpc` (priority 100), not core | Core stays protocol-agnostic; protocomm swappable post-V1 |
<!-- %% sp.sy.ve-dd.dec.c %% -->

<!-- %% sp.sy.ve-dd.c %% -->
