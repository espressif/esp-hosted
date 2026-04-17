# ESP-Hosted Documentation Master Index
<!-- Last updated: 2026-03-18 — registered PEER_DATA_TRANSFER_PROGRESS.md; refreshed open work -->

This document inventories all major markdowns, summarizes what is implemented vs remaining,
and links to the most authoritative subdocuments.

Each entry is classified as one of:
- **SPEC** — normative design / architecture specification
- **DESIGN** — design-decision or analysis document
- **IMPL** — implementation journal, status, or progress log
- **TEST** — test framework and QA docs
- **INFRA** — repo setup, structure, CLAUDE.md, READMEs

---

## 1) Current Implementation Status (summary)

Source of truth:
- `/Users/yogesh/code/esp_hosted.new/PHASES.md`
- `/Users/yogesh/code/esp_hosted.new/specs/09_implementation_status.md`
- `/Users/yogesh/code/esp_hosted.new/COMPLETE_IMPLEMENTATION_JOURNAL.md`

### Implemented (per PHASES + impl status)

- Phases **0–7** and **A** are marked **✅ COMPLETE**
- Phase **D** (registry replacement + auto-init) marked **✅ COMPLETE**
- Phase **E** (config macros + version wiring) marked **✅ COMPLETE**
- Phases **C** (proto component rename + extension rename + id-map headers) marked **✅ COMPLETE**

### Open / Remaining work

From `/Users/yogesh/code/esp_hosted.new/PHASES.md`:
- PENDING-010: dynamic response buffer sizing in RPC handler
- PENDING-011: extension idempotent init/deinit + feature bit tracking
- PENDING-012: extension API naming + guards (full prefix)
- PENDING-013: two-stage INIT TLV handshake (stage-1 legacy-safe, stage-2 extended)
- FG-CAP-GAP: FG extension not calling `add_cap_bits()` before startup

Host-side phases H1–H6 are still **HOST-PENDING**.

### Active work: Peer Data Transfer (Custom RPC)

See `/Users/yogesh/code/esp_hosted.new/PEER_DATA_TRANSFER_PROGRESS.md` for the current
step-by-step task list. Summary of remaining steps:

1. **STEP 1** — Linux FG proto (`esp_hosted_config.proto`): rename `CustomRpcUnserialised*`
   → `CtrlMsg_Req/Resp/Event_CustomRpc`; keep numeric IDs 128/228/308; run `build_proto.sh`.
2. **STEP 2** — `fg_system_reqs.c`: update generated type names after proto regen.
3. **STEP 3** — `fg_protobuf_evt_hook.c`: `CtrlMsgEventCustomRpcUnserialisedMsg`
   → `CtrlMsgEventCustomRpc`; field `custom_evt_id` → `custom_event_id`.
4. **STEP 4** — Mirror STEPs 1–3 to `esp_hosted_final`.
5. **STEP 5** — Fix `len=0` bug in both event subscribers: prepend 4-byte `total_len`
   in `esp_hosted_cp_ext_peer_data_send`.
6. **STEP 6** — Build verification (both repos).
7. **STEP 7** — Host-side: implement `esp_hosted_peer_data_send` /
   `esp_hosted_peer_data_register_callback`; register `peer_data_transfer_event_handler`.

### Notes on doc consistency

- Some older docs still refer to **SLIST** even though the registry is now **realloc-table based**.
- Legacy endpoint APIs still documented in some places; plan is to remove and document only new design.
- Doc 04 (`04_three_registry_api.md`) still says SLIST in the title — update pending.


---

## 2) Design & Architecture

### Core specs (SPEC)

| # | Path | Topic |
|---|------|-------|
| 00 | `specs/00_glossary.md` | Terms, abbreviations, component names |
| 01 | `specs/01_architecture_overview.md` | High-level repo structure, component map |
| 02 | `specs/02_wire_protocol.md` | V1 and V2 frame header encoding field-by-field |
| 03 | `specs/03_priv_handshake.md` | Boot-time PRIV TLV handshake; complete TLV registry (0x00–0x48) |
| 04 | `specs/04_three_registry_api.md` | Three registries in cp_core (now realloc-table, not SLIST) |
| 05 | `specs/05_host_config_abstraction.md` | H_XXX config macros and port headers |
| 06 | `specs/06_rpc_versioning.md` | RPC V1 vs V2 dispatch; msg_id namespace (0x400–0x9FF) |
| 07 | `specs/07_race_conditions_and_safety.md` | Threading, memory ordering, known concurrency risks |
| 08 | `specs/08_compatibility_matrix.md` | FG / NG / MCU old-new interop table |
| 12 | `specs/12_rpc_call_flow.md` | End-to-end RPC call chains (FG host, MCU host, events) |
| 13 | `specs/13_frame_component.md` | `esp_hosted_frame` API — encode, decode, dummy, checksum |
| 14 | `specs/14_design_decisions_v2.md` | Post-review design decisions (auto-init, table registry, proto layout) |
| 15 | `specs/15_socket_transport_and_testing.md` | Socket transport design + system test harness (Phase T) |
| 16 | `specs/16_linker_auto_init.md` | Linker auto-init wiring for IDF vs non-IDF |

### Design / analysis docs (DESIGN)

| # | Path | Topic | Status |
|---|------|-------|--------|
| 17 | `specs/17_rpc_extension_decoupling.md` | RPC extension decoupling from protocomm; race fix | Design proposed |
| 18 | `specs/18_protocomm_removal_analysis.md` | Full audit of what protocomm provides; removal feasibility | Decision pending |

### Handshake & transport (DESIGN)

- `ESP_HOSTED_HANDSHAKE_DESIGN.md` — Latest handshake decisions (two-stage TLV, core ownership)
- `COMPLETE_UNIFIED_TRANSPORT_OPS.md` — Transport ops reference

---

## 3) Implementation Plans / Journals / Status (IMPL)

| File | Topic | Authority level |
|------|-------|----------------|
| `PHASES.md` (root) | **Master roadmap** — phase status, task lists | Highest |
| `COMPLETE_IMPLEMENTATION_JOURNAL.md` | Session-by-session decisions and answers | High |
| `specs/09_implementation_status.md` | Session bug-fix log; open pending items | High |
| `specs/09_implementation_status_review_2026-03-12.md` | Disk-level audit 2026-03-12; verified complete phases | Snapshot |
| `PEER_DATA_TRANSFER_PROGRESS.md` | **Active work**: peer data transfer / Custom RPC steps 1–7 | Current |

Deprecated/stale:
- `specs/STALE_10_e2e_system_review.md` — legacy review (superseded by 10_system_review.md)

---

## 4) System Review & Memory Audit (IMPL)

- `specs/10_system_review.md` — Living system review: open bugs 🔴, risks 🟠, gaps 🟡, OK 🟢
- `specs/11_memory_audit.md` — Memory leaks, double frees, allocation budget, performance

---

## 5) Test Framework & QA (TEST)

- `COMPLETE_TEST_FRAMEWORK_DESIGN.md`
- `test_fix/README_TESTS.md`
- `test_fix/TEST_DOCUMENTATION.md`
- `test_fix/QUICK_FIX.md`

---

## 6) Repo Structure & Setup (INFRA)

- `README.md` — How to orient yourself; document index; current state summary
- `STRUCTURE.md` — Canonical target directory layout (authority for all path decisions)
- `CLAUDE.md` — Claude-specific instructions for this repo
- `versioning/README_VERSION.md`
- `versioning/MIGRATION_GUIDE.md`

---

## 7) FG / NG Documentation (INFRA)

FG:
- `fg/README.md`
- `fg/host/README.md`
- `fg/cp/README.md`
- `fg/docs/Linux_based_host/*`
- `fg/docs/MCU_based_host/*`
- `fg/docs/common/*`

NG:
- `ng/README.md`
- `ng/docs/*`

---

## 8) Examples & Extension READMEs (INFRA)

- `fg/cp/examples/README.md`
- `fg/cp/examples/cp_linux_fg/*`
- `fg/cp/examples/cp_mcu/*`
- `components/coprocessor/extensions/esp_hosted_cp_ext_rpc_fg/README.md`

---

## 9) Known Gaps to Address in Docs

1. Replace "SLIST" references with "realloc-table registry" in specs 04, 10, and README.
2. Document two-stage TLV handshake and core ownership of startup TLVs (blocked on PENDING-013).
3. Remove or mark deprecated legacy endpoint API documentation.
4. Update `README.md` current-state section (still says Session 15 / 2026-03-11).
5. After peer data transfer work lands: add a `19_peer_data_transfer.md` spec
   covering the Custom RPC wire format, len-prefixing, and host-side API.
