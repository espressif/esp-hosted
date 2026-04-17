# 09 — Implementation Status
<!-- Last updated: 2026-03-13 Session 20 — Build gate D9/E5: fixed msg_codec EXTRA_COMPONENT_DIRS (all 11 examples), cli.c extracted to esp_hosted_cp_ext_cli extension, proto include path via idf_component_get_property in rpc_fg + rpc_mcu -->

> **This is the definitive bug and session log.**
> For the full phase roadmap see `PHASES.md`.
> For system risks still open see `specs/10_system_review.md`.

---

## Phase Completion Summary

| Phase | Scope | Status | Session |
|-------|-------|--------|---------|
| 0 | Canonical common headers + directory skeleton | ✅ COMPLETE | 1 |
| 1 | Three SLIST registries in `cp_core` | ✅ COMPLETE | 2 |
| 2 | Extension migration to SLIST nodes | ✅ COMPLETE | 3 |
| 3 | V2 wire header negotiation (TLV advertise + ACK parse) | ✅ COMPLETE | 4 |
| 4 | H_XXX config abstraction bridge headers | ✅ COMPLETE | 5 |
| 5A | RPC V2 proto schema + codegen | ✅ COMPLETE | 6 |
| 5B | RPC V2 SLIST dispatch wiring | ✅ COMPLETE | 6 |
| 6 | Extension rename `ext_*` → `cp_ext_*` | ✅ COMPLETE | 9 |
| 7 | Frame component + all 10 transport migrations | ✅ COMPLETE | 8–9 |
| A | CMake wiring + component moves + shims | ✅ COMPLETE | 10–11 |
| B | `port/cp/` port layer | ⚠️ DEFERRED | — |
| C | Restructure: proto components, extension renames, ID map files, header split (_apis.h/_events.h), ops registry deletion | ✅ COMPLETE (C1–C10 done on disk; C11 build gate pending D9) | 11–15 |
| D | Registry replacement (SLIST→realloc table) + auto-init dispatcher (`EH_CP_EXT_REGISTER`) | ✅ COMPLETE (D1–D8 all done; D9 build gate not run) | 16–17 |
| E | Config macro rollout (`EH_CP_XXX`) + V2 version wiring + `_Static_assert` cross-checks | 🔄 IN PROGRESS (E1/E2/E3 done; E4 host-side/deferred; E5 build gate not run) | 18 |
| F | V2 proto schema + end-to-end V2 (CP + FG Linux + MCU) | 🔲 FUTURE | — |
| H1–H6 | All host-side phases | 🚫 HOST-PENDING | — |
| T | Socket transport stub + system test harness | 🔲 TODO | — |

---

## Open PENDING Items

| ID | Description | Blocking? | Priority |
|----|-------------|-----------|---------|
| PENDING-006 | V2 unified proto schema design (`esp_hosted_rpc_v2.proto`) | Phase F | High (future) |
| PENDING-010 | Dynamic response buffer sizing in `rpc_ll_slist_req_handler` (fixed 512 B over inlen) — superseded by table design; fix in Phase D | Phase D | Medium |
| PENDING-001 | NG variant downgrade detection + `cmd.c` deletion risk | No | Low |
| PENDING-002 | NG build isolation | No | Low |
| PENDING-003 | Proto version cross-verify using magic byte `0xE9` + `hdr_version` field | No | Low |

> **Closed:** FG-CAP-GAP — resolved by Decision 3 (extensions register own cap bits in init_fn).

---

## Phase C — Detailed Steps (Restructure + header split; no functional change)

| Step | Action | Exit check |
|------|--------|-----------|
| C1 | Create `components/common/serializers/esp_hosted_proto_linux_v1/` — move content from `ext_rpc_linux_fg_pbuf/` | header + `.c` present, CMakeLists correct |
| C2 | Create `components/common/serializers/esp_hosted_proto_mcu_v1/` — move from `ext_rpc_mcu_pbuf/` | same |
| C3 | Create `components/common/serializers/esp_hosted_proto_v2/` — empty placeholder with minimal CMakeLists | compiles with no SRCS |
| C4 | Delete `_pbuf` sidecar extension dirs; update all REQUIRES in extension CMakeLists | zero refs to `_pbuf` remain |
| C5 | Rename extensions: `ext_rpc_linux_fg`→`ext_rpc_fg`, `ext_host_ps`→`ext_feat_host_ps`, `ext_network_split`→`ext_feat_nw_split`, `ext_custom_rpc`→`ext_feat_custom_msg` | all includes + CMakeLists updated |
| C6 | Create `esp_hosted_rpc_id_map_v1.h` and `esp_hosted_rpc_id_map_v2.h` in `components/common/esp_hosted_common/include/` | files present, ranges correct |
| C7 | **DEFERRED to Phase E** — `esp_hosted_cp_master_config.h` skeleton | — |
| C8 | Split extension public headers — create `_apis.h` + `_events.h` per extension inside each extension's `include/` (Decision 11); move `esp_hosted_cp_ext_nw_split_status_t` and equivalent types out of core into extension headers | no extension type definition remains in core headers |
| C9 | Replace all `esp_hosted_cp_ext_cp_get_extension(ESP_HOSTED_EXT_NW_SPLIT / HOST_PS)` call sites with direct function calls; add `REQUIRES esp_hosted_cp_ext_feat_nw_split` (and `feat_host_ps`) to CMakeLists of `rpc_mcu`, `rpc_fg`, `nw_split` as appropriate | zero `get_extension()` calls remain |
| C10 | Delete ops registry: `esp_hosted_cp_extension.h`, `esp_hosted_cp_extension_nw_split.h`, `esp_hosted_cp_extension_rpc.h`, `esp_hosted_cp_extension_host_ps.h`, `esp_hosted_cp_extension.c` | zero includes of these files remain anywhere |
| C11 | **`idf.py build` — Phase C exit gate** | zero errors |

## Phase D — Detailed Steps (Functional: registry + auto-init)

| Step | Action | Exit check |
|------|--------|-----------|
| D1 | Define `esp_hosted_ext_desc_t` + `EH_CP_EXT_REGISTER` macro in `esp_hosted_cp_core.h` | macro compiles; section emitted in ELF |
| D2 | Implement `ext_init_task` + `s_ext_init_done` event group in `esp_hosted_cp_core.c`; sequence `generate_startup_event()` to wait on done bit | no startup race |
| D3 | Add `EH_CP_EXT_REGISTER(init_fn, deinit_fn, name, affinity, priority)` to each extension `.c` file | descriptors visible in ELF section |
| D4 | Replace SLIST tables with dynamic realloc (+4 per grow) tables in `esp_hosted_cp_registries.c` | binary search on req, linear on evt |
| D5 | Update public API in `esp_hosted_cp_core.h` — new `rpc_req_register(min, max, handler, ctx)` signatures | no node-struct API remains |
| D6 | Add `out_max` parameter to all `serialise()` callback implementations; add bounds check before pack | no heap overflow possible |
| D7 | Update all extension registration call sites to new API | zero old SLIST node references |
| D8 | Remove `add_cap_bits()` calls from core; ensure each extension's `init_fn` calls it instead | caps accumulated before startup event |
| D9 | **`idf.py build` — Phase D exit gate** | zero errors, runtime init sequence verified |

## Phase E — Detailed Steps (Config macros + version wiring)

| Step | Action | Exit check |
|------|--------|-----------|
| E1 | Populate `esp_hosted_cp_master_config.h` with all `EH_CP_XXX` symbols | complete coverage of all `CONFIG_ESP_HOSTED_CP_*` |
| E2 | Replace all direct `CONFIG_ESP_HOSTED_CP_XXX` in CP sources with `EH_CP_XXX` | grep finds zero direct CONFIG refs in `components/coprocessor/` |
| E3 | Add `_Static_assert` cross-checks in `esp_hosted_rpc_id_map_v1.h` | compile-time check passes |
| E4 | Update PRIV handshake: new-codebase host sends RPC ACK = 0x02 by default | V2 negotiated between two new-code peers |
| E5 | **`idf.py build` — Phase E exit gate** | zero errors |

## Phase F — Detailed Steps (V2 proto, future)

| Step | Action |
|------|--------|
| F1 | Design `esp_hosted_rpc_v2.proto` — single unified schema, `msg_id` at field 2 |
| F2 | `protoc-c` codegen → `esp_hosted_proto_v2/` |
| F3 | V2 extension on CP side; register in [0x2000, 0x3FFF] req + [0x6000, 0x7FFF] evt |
| F4 | Host-side V2: Linux ctrl_lib (`esp_hosted_config.proto` replaced) + MCU `rpc_req.c` |
| F5 | End-to-end V2 test — both FG and MCU negotiate and use V2 |

## Phase T — Socket Transport + System Test Harness

| Step | Action |
|------|--------|
| T1 | CP-side socket transport: `esp_hosted_cp_transport_socket.c` — TCP server, same frame encode/decode API as SPI/SDIO |
| T2 | Host-side socket transport: `esp_hosted_mcu` host transport — TCP client connecting to CP socket |
| T3 | Unit tests: per-component tests under `test/unit/` (frame encode/decode, RPC table overlap detection, cap accumulator) |
| T4 | System tests: `test/system/` — spin up CP + host in two processes/threads over socket, run full RPC round-trips without hardware |
| T5 | CI integration: socket-based system tests run on every PR (no hardware required) |

See `specs/15_socket_transport_and_testing.md` for detailed design.

---

### Session 16 — Registry 3 Realloc-Table Cleanup + Doc Update (2026-03-12)

#### registries.c — confirmed clean (no work needed)

Filesystem audit confirmed `esp_hosted_cp_registries.c` already implements the
Decision 9 design exactly:
- `rpc_req_entry_t` and `rpc_evt_entry_t` — inline structs (no node pointers)
- `s_req_table` / `s_evt_table` — flat `realloc()`-grown arrays
- `table_grow()` — shared helper; grows by `EH_CP_RPC_TABLE_GROW_STEP` (4) slots
- Binary search on sorted req table; linear scan on evt table
- Mutex released before calling handler; no SLIST, no `sys/queue.h`
- `out_max` passed to `serialise()` callback; allocates `MAX_SERIAL_DATA_SIZE`

No SLIST cruft was present. The open design question (node struct vs inline entry)
is resolved: inline `rpc_req_entry_t` / `rpc_evt_entry_t` are the only types.

#### registry_test.c — 81/81 passing

All test groups A–K verified:
- A/B: req register/unregister (overlap detection, NULL, bad range, adjacent)
- C/D: evt register/unregister
- E/F: dispatch hit/miss + boundary IDs (min, max, just outside)
- G: send_event hit/miss + protocomm forwarding
- H: table growth past GROW_STEP (9 ranges, realloc triggered multiple times)
- I: out-of-order insert, sort invariant, binary search on all three positions
- J: hot-reload (unregister + re-register, new handler called)
- K: proto field-2 scanner (varint, multi-byte, absent, truncated, LEN-type)

Run: `make -C components/coprocessor/extensions/test run_registry`

#### Docs updated

- `PHASES.md`: date updated to 2026-03-12; Phases C, D (with per-step actual state), E, F, T added
- `specs/09_implementation_status.md`: Phase D steps corrected (D4/D5/D6 ✅, D1–D3/D7–D8 ❌); Key File Locations updated to canonical paths; stale `esp_hosted_fg/coprocessor/...` references removed

---

### Session 15 — Ops Registry Deletion + Header Ownership Audit (2026-03-12)

#### Full repo review — phase completion verified against actual code

Confirmed complete (Phases 0–7, A, Sessions 13–14 fixes): event bus fix (Bug 3.1–3.3),
FG dispatcher switch-case, SLIST unregister, V2 TLV negotiation, cap bits in extensions.

Confirmed not started: all of Phase C, D, E (zero steps done).

#### ops registry — stale, confirmed for deletion (Decision 11)

`esp_hosted_cp_extension.h` (core header) holds a union of all extension ops structs
and the `register_extension` / `get_extension` API. Code audit result:
- Core's own `.c` files (`core.c`, `registries.c`, `rpc_ll.c`) **never** call `get_extension`
- All callers are extension `.c` files calling into other extensions
- The only reason types lived in core was to satisfy this union

Decision: delete the entire ops registry (5 files). Replace sync-query call sites
with direct function calls to the target extension's public `_apis.h`. No core
mediation needed or wanted. See Decision 11 in `14_design_decisions_v2.md`.

#### Extension header ownership — rule established

Each extension owns its types. Two public headers per extension max:
- `_apis.h` — init/deinit and any callable functions
- `_events.h` — event payload structs + re-export of relevant event IDs

Core never includes extension headers. This is a **hard invariant**.
CMake dependency for cross-extension event subscription is explicit (`REQUIRES`).

#### C7 (master config skeleton) — deferred

`esp_hosted_cp_master_config.h` creation deferred to Phase E where it is
actually needed. Does not block Phase C exit gate.

#### Phase C revised

Steps C8 (header split), C9 (get_extension replacement), C10 (ops registry deletion)
added. Old C7 (master config) deferred. Old C8 build gate renumbered to C11.
Phase C is still "no functional change" — the call sites change from
`get_extension` indirect calls to direct calls, but runtime behaviour is identical.

---

### Session 1–5: Foundation + Registry + Handshake (Phases 0–4)

| ID | Bug | Fix | File |
|----|-----|-----|------|
| A1 | `esp_hosted_common_tlv.h` had `HOST_CAPABILITIES=0x01` (should be 0x44) | Removed `HOST_CAPABILITIES`/`RCVD_ESP_*`/`SLV_CONFIG_*` from common_tlv.h | `esp_hosted_common_tlv.h` |
| A2 | `ESP_PRIV_TRANS_SDIO_MODE (0x18)` absent from new `esp_hosted_caps.h` | Added to `ESP_PRIV_TAG_TYPE` enum at 0x18 | `esp_hosted_caps.h` |
| A3 | `ESP_PRIV_FEAT_CAPS (0x19)` not parsed by kmod SPI/SDIO | Parsing block added to both kmod `process_init_event()` | `esp_spi.c`, `esp_sdio.c` |
| A4 | All 4 TLV parsers lacked `len_left` underflow bounds check | `if ((uint16_t)tag_len + 2 > len_left) break;` added to all 4 | All 4 transport init parsers |
| A5 | `hdr_ver_negotiated` initialized to `0` (not a valid version) | Changed to `ESP_HOSTED_HDR_VERSION_V1` | `esp_hosted_cp_core.c` |
| A6 | Linux kmod adapter fields written without `WRITE_ONCE` | `WRITE_ONCE()` added for `hdr_ver_negotiated` and `rpc_ver_negotiated` | `esp_spi.c`, `esp_sdio.c` |
| A7 | MCU-style macro aliases (0x11–0x19) missing from `esp_hosted_common_tlv.h` | Added with `#ifndef` guards so kmod code can use them | `esp_hosted_common_tlv.h` |
| A8 | `esp_priv_tlv_t` struct used `uint8_t` in kernel context | Wrapped in `#ifndef __KERNEL__` guard | `esp_hosted_common_tlv.h` |

### Session 6–7: Frame Component Build (Phase 7, review passes 1 + 2)

| ID | Bug | Fix | File |
|----|-----|-----|------|
| B1 | Wrong checksum algorithm: carry-fold instead of plain truncation | `frame_checksum()` rewritten to match `compute_checksum()` exactly | `esp_hosted_frame.c` |
| B2 | Const-cast UB in checksum verify: zeroed field via cast-away-const pointer | Non-mutating math: `calc = raw - stored_lo - stored_hi` | `esp_hosted_frame.c` |
| B3 | Missing V2 `hdr_version` field validation in `decode_v2()` | Added check: `hdr->hdr_version != 0x02 → INVALID` | `esp_hosted_frame.c` |
| B4 | Incorrect "XOR checksum" comment in V2 header and frame.h | Fixed to "16-bit byte-sum" in both files | `esp_hosted_common_header_v2.h`, `esp_hosted_frame.h` |
| B5 | `<stdbool.h>` in `esp_hosted_frame.h`: unused and breaks Linux kmod builds | Removed; added explanatory comment | `esp_hosted_frame.h` |
| B6 | `esp_hosted_frame.h` included umbrella `esp_hosted_common.h` dragging in kernel-hostile headers | Replaced with the three specific includes the component actually needs | `esp_hosted_frame.h` |
| B7 | `ESP_HOSTED_HDR_VERSION_V1/V2` in `esp_hosted_common_header_v2.h` had no `#ifndef` guard | Added guard; values match `esp_hosted_common_tlv.h` — no numeric change | `esp_hosted_common_header_v2.h` |
| B8 | CP SPI `get_next_tx_buffer` `USE_STATIC_DUMMY_BUFFER` path: NULL dereference writing `throttle_cmd` | Fixed via `esp_hosted_frame_encode_dummy()` which handles NULL internal correctly | `esp_hosted_transport_cp_spi.c` |
| B9 | CP SPI `esp_spi_write`: `wifi_flow_ctrl_en` computed but never written to wire header | Fixed via `esp_hosted_frame_encode_dummy()` which writes `throttle_cmd` correctly | `esp_hosted_transport_cp_spi.c` |


### Session 8: Bug Hunt (Build + Runtime — Phases 5B, 7)

| ID | Bug | Fix | File |
|----|-----|-----|------|
| Build BUG 1.1 | `esp_hosted_transport_cp` CMakeLists missing `esp_hosted_common` + `esp_hosted_frame` in REQUIRES | Added both to REQUIRES | `esp_hosted_transport_cp/CMakeLists.txt` |
| Build RISK 1.2 | MCU host CMakeLists guard `CONFIG_IDF_TARGET` (always true in IDF) defeats Kconfig choice | Changed to `CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU` only | `eh_host_transport/CMakeLists.txt` |
| DPP BUG 5.1 | `#if H_DPP_SUPPORT` block used struct-initializer syntax inside `switch`/`case` — compile error when `H_DPP_SUPPORT` defined. Plus duplicate `Req_SuppDppDeinit` entry | Replaced with proper `case` statements; removed duplicate | `esp_hosted_ext_rpc_mcu_dispatcher_req.c` |
| PENDING-009 | `esp_hosted_cp_dispatch_rx()` never called in `process_rx_pkt()` — extensions registering RX callbacks via Registry 1 never fired | Wired `esp_hosted_cp_dispatch_rx()` for STA_IF and AP_IF with fallback to direct WiFi TX | `esp_hosted_cp_core.c` |
| GAP-001 | kmod SPI/SDIO parse V2 TLVs but never send ACK (0x21, 0x23) back to CP — V2 mode can never activate on kmod path | ACK packet send added to `process_init_event()` in both kmod files | `esp_spi.c`, `esp_sdio.c` |

### Session 9: Frame Migration + Extension Rename (Phases 6, 7)

All 10 transport files migrated to `esp_hosted_frame` encode/decode (PENDING-005 complete).
All 7 extension directories renamed `esp_hosted_ext_*` → `esp_hosted_cp_ext_*` (PENDING-004 complete).

No new bugs found. B8 and B9 from review pass 2 confirmed fixed via `encode_dummy()`.

### Session 10: CMake Build Silo Wiring (Phase A4 + A5)

| Action | Detail |
|--------|--------|
| All 11 example `CMakeLists.txt` updated | Added `repo_root/components/common` to `EXTRA_COMPONENT_DIRS` so IDF can find `esp_hosted_common` and `esp_hosted_frame` |
| Path depth verified | 6-level examples: `../../../../../..`; 7-level examples: `../../../../../../..`. All 11 paths verified to resolve to correct repo root. |
| `PHASES.md` updated | Phase A4/A5 marked done; A1–A3 documented as next work |
| `specs/09_implementation_status.md` (this file) fully rewritten | Session log consolidated, open items updated |

---

## PENDING-005: Frame Component Migration — COMPLETE

Full detail of what was migrated:

| File | TX encode | RX decode | Pre-existing bugs fixed |
|------|-----------|-----------|------------------------|
| `esp_hosted_transport_cp_spi.c` | ✅ | ✅ | B8 (NULL deref dummy), B9 (throttle_cmd) |
| `esp_hosted_transport_cp_sdio.c` | ✅ | ✅ | `generate_startup_event` + 2 RX decode paths |
| `esp_hosted_transport_cp_uart.c` | ✅ | ✅ | — |
| `esp_hosted_transport_cp_spi_hd.c` | ✅ | ✅ | — |
| `mcu/spi/spi_drv.c` | ✅ | ✅ | Power-save direct-TX path |
| `mcu/sdio/sdio_drv.c` | ✅ | ✅ | — |
| `mcu/spi_hd/spi_hd_drv.c` | ✅ | ✅ | — |
| `mcu/uart/uart_drv.c` | ✅ | ✅ | — |
| `kmod/spi/esp_spi.c` | ✅ | ✅ | `compute_checksum` moved to `esp.h` as `static inline`; stale `esp_hosted_transport.h` include removed |
| `kmod/sdio/esp_sdio.c` | N/A | ✅ | SDIO RX; TX header filled upstream |

---

## PENDING-009: Registry 1 Dispatch Wiring — COMPLETE (Session 8)

`esp_hosted_cp_dispatch_rx()` now called from `process_rx_pkt()` for STA_IF and AP_IF.
The hardcoded `esp_wifi_internal_tx()` calls remain as defaults pre-registered in
Registry 1 at init. Custom extensions can override by calling
`esp_hosted_cp_register_rx_cb(ESP_IF_TYPE_STA, my_handler, ctx)`.

---

## Key File Locations Reference

| File | Role |
|------|------|
| `components/common/esp_hosted_common/include/esp_hosted_common_tlv.h` | Canonical TLV code definitions — single source of truth for all TLV tags |
| `components/common/esp_hosted_common/include/esp_hosted_common_header.h` | V1 wire frame struct (`struct esp_payload_header`, 12 B) |
| `components/common/esp_hosted_common/include/esp_hosted_common_header_v2.h` | V2 wire frame struct (`esp_hosted_header_v2_t`, 20 B, magic `0xE9`) |
| `components/common/esp_hosted_frame/include/esp_hosted_frame.h` | Frame encode/decode API (7 functions) |
| `components/common/esp_hosted_frame/src/esp_hosted_frame.c` | Frame implementation |
| `components/coprocessor/esp_hosted_cp_transport/include/common/esp_hosted_caps.h` | `ESP_PRIV_TAG_TYPE` enum (MCU + FG TLV tag types) |
| `components/coprocessor/esp_hosted_cp_core/src/esp_hosted_cp_core.c` | `hdr_ver_negotiated`, `rpc_ver_negotiated` globals; `process_rx_pkt()` |
| `components/coprocessor/esp_hosted_cp_core/src/esp_hosted_cp_registries.c` | All three registries; realloc table req/evt; binary search dispatch |
| `components/coprocessor/esp_hosted_cp_core/src/esp_hosted_cp_rpc_ll.c` | Protocomm `pserial` layer; RPC dispatch invocation |
| `components/common/esp_hosted_common/include/esp_hosted_rpc_id_map_v1.h` | V1 RPC ID ranges |
| `components/common/esp_hosted_common/include/esp_hosted_rpc_id_map_v2.h` | V2 RPC ID ranges |
| `components/coprocessor/extensions/test/registry_test.c` | Registry 3 unit tests — 81/81 passing (groups A–K) |
| `port/host/idf/include/port_esp_hosted_host_config.h` | CONFIG → H_XXX bridge for IDF (MCU host) |
| `port/host/linux/include/port_esp_hosted_host_config.h` | Direct H_XXX defines for Linux kmod |


### Session 11: Phase A1–A3 Component Moves (2026-03-09)

**Phase A fully complete.** All CP components physically moved to canonical paths.

| Action | Detail |
|--------|--------|
| A1 — `esp_hosted_cp_core` created | `components/coprocessor/esp_hosted_cp_core/` populated with 17 headers (`include/`) and 11 source files (`src/`). New `CMakeLists.txt` written: IDF component name `esp_hosted_cp_core`, REQUIRES `esp_hosted_common`, `esp_hosted_frame`, `esp_hosted_cp_transport`, `esp_event`, `protocomm`, `driver`. Kconfig copied. |
| A2 — `esp_hosted_cp_transport` created | `components/coprocessor/esp_hosted_cp_transport/` populated: `include/` (3 headers), `include/common/` (6 headers), `private_include/` (2 headers), `src/` (6 sources). New `CMakeLists.txt` written: IDF component name `esp_hosted_cp_transport`, split-driver aware (IDF ≥5.3). |
| A3 — 7 extensions copied to `components/coprocessor/extensions/` | All 7 `esp_hosted_cp_ext_*` extensions copied from legacy path. All 5 extension CMakeLists updated: `esp_hosted_cp` → `esp_hosted_cp_core` in REQUIRES. `main/CMakeLists.txt` for host_power_save example also updated. |
| 11 example CMakeLists updated | All 11 example top-level `CMakeLists.txt` files now include `components/coprocessor` and `components/coprocessor/extensions` in `EXTRA_COMPONENT_DIRS` (in addition to `components/common` from Session 10). |
| Verification | Zero bare `"esp_hosted_cp"` references remain in canonical tree. All 11 examples confirmed to have both new dirs. All include paths resolved correctly. |
| Remaining | `idf.py build` verification pending (requires IDF environment). Legacy source dirs at `esp_hosted_fg/coprocessor/components/` still present as original; safe to delete after build is verified. |


### Session 11: Phase A1–A3 Component Moves (2026-03-09)

**Phase A is now fully complete.** All CP components physically moved to canonical paths.

| Action | Detail |
|--------|--------|
| **A1: `esp_hosted_cp_core/` created** | 17 headers copied to `include/`, 13 sources to `src/`, Kconfig copied, new `CMakeLists.txt` written with IDF component name `esp_hosted_cp_core`, REQUIRES: `esp_hosted_common`, `esp_hosted_frame`, `esp_hosted_cp_transport`, `esp_event`, `protocomm`, `driver` |
| **A2: `esp_hosted_cp_transport/` created** | 9 headers to `include/` + `include/common/`, 2 private headers to `private_include/`, 6 sources to `src/`, all 5 Kconfig files copied, new `CMakeLists.txt` written with IDF name `esp_hosted_cp_transport`, split-driver aware (IDF ≥5.3 uses `esp_driver_*`), REQUIRES: `esp_timer`, `esp_hosted_common`, `esp_hosted_frame` |
| **A3: All 7 extensions copied** | `esp_hosted_cp_ext_rpc_linux_fg`, `esp_hosted_cp_ext_rpc_linux_fg_pbuf`, `esp_hosted_cp_ext_rpc_mcu`, `esp_hosted_cp_ext_rpc_mcu_pbuf`, `esp_hosted_cp_ext_host_ps`, `esp_hosted_cp_ext_network_split`, `esp_hosted_cp_ext_custom_rpc` — all moved to `components/coprocessor/extensions/`. All REQUIRES updated from `esp_hosted_cp` → `esp_hosted_cp_core` |
| **Extension REQUIRES updated** | 5 canonical extension CMakeLists updated: `esp_hosted_cp` → `esp_hosted_cp_core`. Zero stale refs remain in canonical tree. |
| **Shims placed at old paths** | `esp_hosted_fg/coprocessor/components/esp_hosted_cp/CMakeLists.txt` and `esp_hosted_transport_cp/CMakeLists.txt` replaced with shims that emit a `WARNING` message and register nothing. Prevents stale path confusion. |
| **Legacy extensions updated** | All 5 extension CMakeLists under `esp_hosted_fg/coprocessor/extensions/` also updated (`esp_hosted_cp` → `esp_hosted_cp_core`) for consistency — these are now shadowed by the canonical copies and won't be compiled by IDF. |
| **EXTRA_COMPONENT_DIRS complete** | All 11 example `CMakeLists.txt` now have: `coprocessor_dir/components`, `coprocessor_dir/extensions`, `coprocessor_dir/third_party`, `repo_root/components/common`, `repo_root/components/coprocessor`, `repo_root/components/coprocessor/extensions` |
| **main/CMakeLists.txt updated** | `host_power_save` example `main/CMakeLists.txt` REQUIRES updated: `esp_hosted_cp` → `esp_hosted_cp_core` |

**Remaining gate before Phase A can be signed off completely:**
`idf.py build` for `cp_linux_fg/minimal/wifi/` and `cp_mcu/minimal/wifi/` — needs IDF environment. All file-level work is done.


### Session 11 — Review Pass: SRCS Filename Bugs Fixed

After the copy in Session 11, the `gsed` rename had incorrectly changed source filenames inside
CMakeLists SRCS lists, even though the actual `.c` files were never renamed. This would have
caused `idf.py build` to immediately fail with "file not found" errors for every extension.

| Extension | Wrong SRCS reference | Correct (actual filename) |
|-----------|---------------------|--------------------------|
| `esp_hosted_cp_ext_host_ps` | `src/esp_hosted_cp_ext_host_ps.c` | `src/esp_hosted_ext_host_ps.c` |
| `esp_hosted_cp_ext_network_split` | `src/esp_hosted_cp_ext_nw_split.c` | `src/esp_hosted_ext_nw_split.c` |
| `esp_hosted_cp_ext_rpc_mcu` | `src/esp_hosted_cp_ext_rpc_mcu.c` (×9 files) | `src/esp_hosted_ext_rpc_mcu*.c` |
| `esp_hosted_cp_ext_rpc_linux_fg` | `src/esp_hosted_cp_ext_rpc_linux_fg_*.c` (×8 files) | `src/esp_hosted_ext_rpc_linux_fg_*.c` |
| `esp_hosted_cp_ext_custom_rpc` | `src/esp_hosted_cp_ext_user_defined_rpc.c` | `src/esp_hosted_ext_user_defined_rpc.c` |

**Root cause**: `gsed -i 's/\besp_hosted_cp\b/esp_hosted_cp_core/g'` was run on
CMakeLists files at the time of the extension rename (Phase 6 / Session 9). However
the gsed word-boundary pattern `\besp_hosted_cp\b` also matched the SRCS path strings
like `esp_hosted_cp_ext_rpc_mcu.c` — changing them to `esp_hosted_cp_core_ext_rpc_mcu.c`.
Wait — actually the pattern changed `esp_hosted_cp` at word boundary, and `esp_hosted_cp_ext`
has `_ext` after `cp` — so the word boundary did NOT match those. The actual cause is that
the Session 11 copy used the **already-renamed** CMakeLists from `esp_hosted_fg/coprocessor/extensions/`,
and those files had their SRCS strings renamed (from the Phase 6 work) even though the
actual `.c` files were not renamed. All 20 SRCS references verified correct after fix.

**Verification**: Python script confirmed `0 errors` across all 20 source file references.



---

### Session 13 — Event Architecture Fixes + Header Cleanup (2026-03-10)

**All three event coupling bugs fixed. Headers audited. V2 ID ranges expanded.**

#### FG-CAP-GAP — Confirmed Already Fixed
`esp_hosted_ext_rpc_linux_fg_core.c` calls `esp_hosted_cp_add_cap_bits(ESP_WLAN_SUPPORT, 0)` at init time
(guarded by `CONFIG_ESP_HOSTED_CP_WIFI_ENABLED`). Spec doc was stale. No code change needed. ✅

#### Bug 3.1 — nw_split posts to wrong event base ✅ FIXED
- Removed `ESP_EVENT_DEFINE_BASE(ESP_HOSTED_CP_EXT_NW_SPLIT_EVENT)` from `esp_hosted_ext_nw_split.c`
- Changed `esp_event_post()` to use `ESP_HOSTED_CP_EVENT` (the core bus)
- RPC adapters already subscribed to `ESP_HOSTED_CP_EVENT` — events now reach them correctly

#### Bug 3.2 — subscribe_custom_rpc declared in core header ✅ FIXED
- Removed `esp_hosted_cp_ext_cp_subscribe_custom_rpc_events()` / `unsubscribe` from `esp_hosted_cp_extension_rpc.h`
- The 3 RPC config types (`esp_hosted_cp_ext_rpc_data_t`, handler typedef, config struct) remain — they are legitimate core-extension interface types used in the registry union

#### Bug 3.3 — subscribe_nw_split declared in nw_split public header ✅ FIXED
- Removed `esp_hosted_cp_ext_cp_subscribe_network_split_events()` / `unsubscribe` from `esp_hosted_ext_nw_split.h`
- Removed the calls from `esp_hosted_extension_cp_nw_split_init()` / `deinit()` — nw_split is the producer and must never call into its consumers

#### Subscription ownership — correct pattern now enforced
- **RPC adapters own all subscriptions** for events they handle
- `esp_hosted_cp_ext_rpc_linux_fg_init()`: now subscribes to wifi + system + custom_rpc + nw_split events
- `esp_hosted_cp_ext_rpc_mcu_init()`: same
- Both deinit() functions call corresponding unsubscribe for all four groups
- Declarations moved to private headers: `esp_hosted_ext_rpc_linux_fg_priv.h` and `esp_hosted_ext_rpc_mcu_priv.h`
- `esp_hosted_cp_ext_cp_user_defined_rpc_init()`: no longer calls subscribe — comment explains why

#### Header cleanup — stale file moved
- `esp_hosted_cp_extension_power_save.h`: unused orphan (not included anywhere, superseded by `esp_hosted_cp_extension_host_ps.h`) → moved to `include/.stale/`

#### Section 4 (legacy endpoint compat) — already correct
- `CONFIG_ESP_HOSTED_LEGACY_FG_EP_COMPAT` Kconfig exists and guards `ctrlResp`/`ctrlEvnt` aliases in `rpc_ll.c`. No additional legacy code found outside this guard.

#### V2 msg_id ranges — expanded to wide layout
Proto3 varint note: values ≤ 0x7FFF encode in 3 bytes; values ≤ 0x1FFFFF encode in 4 bytes. New layout stays in 3-byte territory throughout.

New ranges in both `rpc_ll.c` defines and `RpcIdV2` enum in `esp_hosted_rpc.proto`:

| Band | Old range | New range | IDs available |
|------|-----------|-----------|---------------|
| Requests | 0x0400–0x05FF (512) | 0x2000–0x3FFF | 8191 |
| Responses | 0x0600–0x07FF (512) | 0x4000–0x5FFF | 8191 |
| Events | 0x0800–0x09FF (512) | 0x6000–0x7FFF | 8191 |

Sentinel rule changed: `Req_V2_Max` / `Resp_V2_Max` / `Event_V2_Max` are now fixed end-of-band values (0x3FFF / 0x5FFF / 0x7FFF). Do NOT move them when adding new IDs — just insert before them.

#### PENDING-010 — noted, not yet fixed (medium priority)
Fixed 4096-byte calloc in `rpc_ll_slist_req_handler` for every request regardless of actual response size. Risk: silent truncation if response ever exceeds 4096 bytes. Deferred — not breaking today.

#### subscribe_events / unsubscribe_events in ops struct — final consensus
**Do NOT put them in the ops struct.** The ops struct is the interface the core uses to call *into* the extension (e.g. `wakeup_host()`, `is_host_power_saving()`). Event subscription is internal lifecycle wiring — the extension does it for itself at `init()` time. Nothing outside needs to invoke it. The established pattern (already correct in `fg_core.c`) is: extension's own `init()` calls its own `subscribe_*()` directly. Core is passive. No ops struct exposure needed or wanted.

#### 5.2 (string-unified registry) — deferred, noted for future
No action. Design note: if ever implemented, it would be an optional named lookup on top of the existing SLIST, not a replacement.

---

### Session 14 — RPC Path Review, Dispatcher Cleanup, SLIST Unregister (2026-03-10)

#### V1-first confirmed, V2 plan documented
V1 (FG: `esp_hosted_config.proto` / MCU: `esp_hosted_rpc.proto`) ships in this release.
V2 activates only after host+CP negotiate `ESP_HOSTED_RPC_VERSION_V2` at boot via TLV handshake.
For V2, the proto will be a **single unified schema** — no "fg" / "mcu" naming split.
Both host targets (Linux FG and MCU) will use one proto file, one namespace, same RPC IDs.
The `RpcIdV2` enum already lays the correct foundation for this.

#### RPC request/event path: complete review
Full path verified end-to-end for both FG and MCU. Both are structurally complete and correct.

**FG V1 request path:**
`Transport RX → protocomm RPCReq → rpc_ll_slist_req_handler → msg_id extract →
SLIST dispatch → fg_slist_req_handler → linux_rpc_req_handler → ctrl_msg__unpack →
fg_rpc_req_dispatcher (switch) → individual handler → ctrl_msg__pack → TX`

**FG V1 event path:**
`esp_event_post(ESP_HOSTED_CP_EVENT) → event_subscriber → esp_hosted_cp_rpc_send_event →
SLIST dispatch → fg_slist_evt_serialise → linux_rpc_event_handler →
fg_rpc_evt_dispatcher (switch) → ctrl_ntfy_XXX → ctrl_msg__pack → TX`

**MCU paths:** identical structure, different proto types (`Rpc` vs `CtrlMsg`), switch-based dispatcher throughout.

#### `session_id` naming — design note (no code change)
Both `linux_rpc_event_handler()` and `mcu_rpc_event_handler()` receive `event_id` via a
parameter named `session_id`. This is inherited from the protocomm API signature. The value
is correct; only the name is a holdover. A comment is already present. Accepted as-is.

#### `Resp_Base` sentinel protocol — by design, documented
When a command is unknown or disabled by Kconfig, both FG and MCU dispatchers set
`resp->msg_id = Resp_Base` (FG: `CTRL_MSG_ID__Resp_Base` = 200; MCU: `RPC_ID__Resp_Base`).
This is **intentional**: `Resp_Base` is the sentinel value the host ctrl_lib interprets as
"unsupported RPC invoked". This allows a well-formed response frame while signalling failure.
Example: all WiFi RPCs are compiled out when `CONFIG_ESP_HOSTED_CP_WIFI_ENABLED=n`. If the
host calls a WiFi RPC anyway, it receives `Resp_Base` and handles it as unsupported.
**Host contract:** host ctrl_lib must treat `Resp_Base` as a negative / unsupported response.

#### `resp->msg_id` offset formula — FG and MCU are in parity (no change needed)
Both hooks set `resp->msg_id` before the dispatcher runs using identical logic:
- FG: `resp->msg_id = req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base`
- MCU: `resp->msg_id = req->msg_id - RPC_ID__Req_Base + RPC_ID__Resp_Base`
The MCU switch-case handlers do NOT override `resp->msg_id`; they rely on the pre-set value.
Both are in parity. This relies on Req/Resp IDs being perfectly aligned — which is by design
and enforced in the proto schema. No change needed.

#### `send_event` output buffer allocation — verified correct, stale comment fixed
`registries.c::esp_hosted_cp_rpc_send_event()` allocates:
`alloc_sz = (len < 3840) ? 4096 : len + 256`
Fixed floor of 4096 bytes covers all known event payloads. The stale comment in
`mcu_slist_evt_serialise()` said "data_len + 256 headroom" — incorrect. Fixed.

#### FG req dispatcher: linear scan → switch-case ✅ CHANGED
`esp_hosted_ext_rpc_linux_fg_protobuf_req_hook.c`:
- Removed `esp_ctrl_msg_req_t req_table[]` (28-entry struct array) and `lookup_req_handler()` (O(n))
- Replaced with `switch(req->msg_id)` — O(1) jump table, matching MCU pattern exactly
- WiFi cases still guarded by `#ifdef CONFIG_ESP_HOSTED_CP_WIFI_ENABLED`
- `Resp_Base` sentinel comment added to `default:` and handler-failure paths

#### SLIST unregister — implemented ✅
Previously both deinit() paths left SLIST nodes in place with a PENDING comment.

| File | Change |
|------|--------|
| `esp_hosted_cp_core.h` | Added declarations for `esp_hosted_cp_rpc_req_unregister()` and `esp_hosted_cp_rpc_evt_unregister()` |
| `esp_hosted_cp_registries.c` | Implemented both. Walk SLIST by pointer identity, use `SLIST_REMOVE_HEAD` / `SLIST_REMOVE_AFTER(prev)` pattern (BSD SLIST has no `SLIST_REMOVE`). Protected by registry mutex. Returns `ESP_ERR_NOT_FOUND` if node absent. |
| `esp_hosted_ext_rpc_linux_fg_core.c` deinit | Now calls `esp_hosted_cp_rpc_req_unregister(&s_fg_req_node)` and `esp_hosted_cp_rpc_evt_unregister(&s_fg_evt_node)` |
| `esp_hosted_ext_rpc_mcu.c` deinit | Same for MCU nodes |

Nodes are BSS-allocated; unregistering is safe. A subsequent `init()` call will `memset` and re-register them cleanly, so hot-reload works correctly.

---

## Phase D — Detailed Steps (Functional: registry + auto-init)

| Step | Action | Status | Exit check |
|------|--------|--------|-----------|
| D1 | Define `esp_hosted_ext_desc_t` + `EH_CP_EXT_REGISTER` macro in `esp_hosted_cp_core.h`; add `.eh_cp_ext_descs` section in `esp_hosted_cp_core.lf` | ✅ Done (Session 17) | macro compiles; section emitted in ELF |
| D2 | Implement `ext_init_task` + `g_ext_init_done_eg` event group in `esp_hosted_cp_core.c`; `host_reset_task` waits on done bit | ✅ Done (Session 17) | no startup race |
| D3 | All 3 auto-init extensions wire `EH_CP_EXT_REGISTER` with correct public fn names and correct arg order | ✅ Done (Session 17) | `host_ps`(50), `rpc_fg`(100), `rpc_mcu`(100) descriptors in section |
| D4 | Replace SLIST tables with `rpc_req_entry_t`/`rpc_evt_entry_t` flat realloc arrays in `esp_hosted_cp_registries.c` | ✅ Done (Session 16) | binary search on req, linear on evt |
| D5 | Update public API in `esp_hosted_cp_core.h` — new `rpc_req_register(min, max, handler, ctx)` signatures | ✅ Done (Session 16) | no node-struct API remains |
| D6 | Add `out_max` parameter to all `serialise()` callback implementations; add bounds check before pack | ✅ Done (Session 16) | no heap overflow possible |
| D7 | Update all extension registration call sites to new API | ✅ Done (Session 17) | `rpc_fg`, `rpc_mcu` use `esp_hosted_cp_rpc_req_register`; `host_ps` has no RPC table registration (correct) |
| D8 | Move `add_cap_bits()` calls into each extension's `init_fn` | ✅ Done (Session 17) | `rpc_fg`: `ESP_WLAN_SUPPORT`; `rpc_mcu`: WLAN + BT + OTA feat bits; `host_ps`: no caps (correct) |
| D9 | **`idf.py build` — Phase D exit gate** | ❌ Not run | zero errors; runtime init sequence verified in log |

**Note (Session 16)**: `registries.c` already uses inline `rpc_req_entry_t` / `rpc_evt_entry_t` structs,
`realloc()`-grown tables, mutex-protected dispatch, binary search on req table, and linear scan on evt table.
No `sys/queue.h`, no SLIST macros, no node pointer indirection anywhere in registries.c.
Unit tests: `components/coprocessor/extensions/test/registry_test.c` — **81/81 passing** (groups A–K).
Run: `make -C components/coprocessor/extensions/test run_registry`

## Phase E — Detailed Steps (Config macros + version wiring)

| Step | Action | Exit check |
|------|--------|-----------|
| E1 | Populate `esp_hosted_cp_master_config.h` — one `EH_CP_XXX` per `CONFIG_ESP_HOSTED_CP_XXX` | complete; no CONFIG_ symbol missing |
| E2 | Replace all direct `CONFIG_ESP_HOSTED_CP_XXX` refs in `components/coprocessor/` with `EH_CP_XXX` | grep finds zero bare CONFIG refs under `components/coprocessor/` |
| E3 | Add `_Static_assert` cross-checks in `esp_hosted_rpc_id_map_v1.h` — range vs proto enum sentinels | compile-time check passes; guarded by `INCLUDE_EH_RPC_ID_MAP_V1_ASSERTS` |
| E4 | Update PRIV handshake defaults: new-codebase host sends RPC ACK = 0x02 by default | two new-code peers negotiate V2 automatically |
| E5 | **`idf.py build` — Phase E exit gate** | zero errors |

## Phase F — Detailed Steps (V2 unified proto — future, after PENDING-006)

| Step | Action |
|------|--------|
| F1 | Design `esp_hosted_rpc_v2.proto` — single unified schema, `msg_id` at field 2, IDs in 0x2000–0x7FFF |
| F2 | `protoc-c` codegen → `components/common/serializers/esp_hosted_proto_v2/` |
| F3 | CP V2 extension: register req [0x2000, 0x3FFF] + evt [0x6000, 0x7FFF] in the same dynamic table |
| F4 | Host-side V2: Linux ctrl_lib + MCU `rpc_req.c` both updated to the single V2 schema |
| F5 | End-to-end V2 system test via socket transport (Phase T) |

## Phase T — Socket Transport + System Test Harness

Allows full RPC round-trips to be validated without any real hardware.

| Step | Action |
|------|--------|
| T1 | **CP socket transport** (`esp_hosted_cp_transport_socket.c`) — FreeRTOS task wraps POSIX TCP; same `esp_hosted_frame` encode/decode as SPI/SDIO; TCP server on configurable port |
| T2 | **Host socket transport** (`esp_hosted_mcu` side) — TCP client connecting to CP; replaces SPI HAL calls with `send()`/`recv()`; same frame API |
| T3 | **Unit tests** under `test/unit/` — frame encode/decode correctness, RPC table overlap detection, cap accumulator atomicity, EH_CP_EXT_REGISTER section iteration |
| T4 | **System tests** under `test/system/` — spawn CP + host as two threads/processes over loopback socket; run full RPC request/response and event paths; assert correct response IDs and payloads |
| T5 | **CI integration** — socket system tests run on every PR (no hardware gate); hardware tests remain optional |

See `specs/15_socket_transport_and_testing.md` for detailed design decisions.

---

## Session Log

### Session 17 — 2026-03-12  D1–D3/D7/D8 completion + linker section fix

**Files changed:**

| File | Change |
|------|--------|
| `esp_hosted_cp_core/src/esp_hosted_cp_core.c` | Fixed `ext_init_task`: stale `__esp_hosted_ext_start/end` → `&__start/stop_eh_cp_ext_descs`; `d->init_priority` → `d->priority`; `d->init` → `d->init_fn`; `d->deinit` → `d->deinit_fn` |
| `esp_hosted_cp_core/esp_hosted_cp_core.lf` | Updated section name `.esp_hosted_ext` → `.eh_cp_ext_descs` to match `EH_CP_EXT_REGISTER` macro; updated all names consistently |
| `ext_rpc_fg/src/esp_hosted_cp_ext_rpc_fg_core.c` | Fixed `EH_CP_EXT_REGISTER`: `rpc_linux_fg_init` (undefined) → `esp_hosted_cp_ext_rpc_linux_fg_init` |
| `ext_rpc_mcu/src/esp_hosted_cp_ext_rpc_mcu.c` | Fixed `EH_CP_EXT_REGISTER`: `rpc_mcu_init` (undefined) → `esp_hosted_cp_ext_rpc_mcu_init` |
| `ext_feat_host_ps/src/esp_hosted_cp_ext_feat_host_ps.c` | Fixed `EH_CP_EXT_REGISTER`: scrambled arg order → correct `(init_fn, deinit_fn, name, affinity, prio)` |

**Design decisions confirmed:**
- `feat_nw_split` and `feat_custom_msg` intentionally have **no** `EH_CP_EXT_REGISTER` — they require app-level config structs; user calls them directly after `esp_hosted_cp_init()`.
- `host_ps` runs at priority 50 (before RPC adapters at 100) so power-save hardware is ready before any RPC handler tries to query it.

**Remaining before D9 build gate:**
- D9: `idf.py build` — first real compile test of the entire Phase D work

### Session 18 — 2026-03-12  Pre-build fixes (GAP-F/G/H) + Phase E E1–E3

#### Pre-build fixes

| GAP | File | Fix |
|-----|------|-----|
| GAP-F | `esp_hosted_cp_core.c` | Added `static void ext_init_task(void *pvParameters);` forward declaration near top of file |
| GAP-G | `esp_hosted_cp_core.c` | Added `assert(g_ext_init_done_eg)` before `xEventGroupWaitBits` in `host_reset_task` |
| GAP-H | `rpc_fg_core.c`, `rpc_mcu.c` | Wired `deinit_fn` in `EH_CP_EXT_REGISTER`: `NULL` → `esp_hosted_cp_ext_rpc_linux_fg_deinit` / `esp_hosted_cp_ext_rpc_mcu_deinit` |

#### Phase E — E1: master config header created

New file: `components/coprocessor/esp_hosted_cp_core/include/esp_hosted_cp_master_config.h`

- One `EH_CP_XXX` alias per `CONFIG_ESP_HOSTED_CP_XXX` symbol from Kconfig
- Covers: host type, feature gates (WiFi, BT, DPP), task config, queue sizes, debug, legacy compat
- Includes compile-time `_Static_assert` that exactly one host type is selected
- All values have safe defaults for non-IDF builds

#### Phase E — E2: CONFIG_ESP_HOSTED_CP_* references replaced with EH_CP_XXX

| File | Symbols replaced |
|------|-----------------|
| `esp_hosted_cp_core.c` | `CONFIG_ESP_HOSTED_CP_WIFI_ENABLED` → `EH_CP_WIFI_ENABLED` (×4); `CONFIG_ESP_HOSTED_CP_EXT_CLI` → `EH_CP_CLI_ENABLED`; `CONFIG_ESP_HOSTED_LEGACY_SEND_EVENT_TO_HOST_API` → `EH_CP_LEGACY_SEND_EVENT_API`; task stack/priority CONFIG_ → `EH_CP_TASK_STACK_SIZE` / `EH_CP_TASK_PRIO_DEFAULT` (×3) |
| `esp_hosted_cp_rpc_ll.c` | `CONFIG_ESP_HOSTED_LEGACY_FG_EP_COMPAT` → `EH_CP_LEGACY_FG_EP_COMPAT`; `CONFIG_ESP_HOSTED_LEGACY_ADD_ENDPOINT_API` → `EH_CP_LEGACY_ADD_ENDPOINT_API` (×4); task CONFIG_ → `EH_CP_TASK_*` (×1) |
| `esp_hosted_cp_ext_rpc_fg_core.c` | `CONFIG_ESP_HOSTED_CP_WIFI_ENABLED` → `EH_CP_WIFI_ENABLED` |
| `esp_hosted_cp_ext_rpc_mcu.c` | `CONFIG_ESP_HOSTED_CP_WIFI_ENABLED` → `EH_CP_WIFI_ENABLED` (×2); `CONFIG_ESP_HOSTED_CP_BT_ENABLED` → `EH_CP_BT_ENABLED` |
| All 4 files | Added `#include "esp_hosted_cp_master_config.h"` |

**Note**: Transport Kconfig symbols (`CONFIG_ESP_HOSTED_TRANSPORT_CP_SPI` etc.) are intentionally left as `CONFIG_` — they belong to the transport component's Kconfig scope, not to the CP feature Kconfig. A separate `esp_hosted_cp_transport_master_config.h` can be added in a future pass if needed.

#### Phase E — E3: _Static_assert cross-checks

Already present in `esp_hosted_rpc_id_map_v1.h` (added in Session 13):
- `EH_RPC_V1_FG_REQ_MIN` vs `CTRL_MSG_ID__Req_Base + 1`
- `EH_RPC_V1_FG_RESP_MIN` vs `CTRL_MSG_ID__Resp_Base`
- `EH_RPC_V1_MCU_REQ_MIN` vs `RPC_ID__Req_Base + 1`
- `EH_RPC_V1_MCU_RESP_MIN` vs `RPC_ID__Resp_Base`
Guarded by `INCLUDE_EH_RPC_ID_MAP_V1_ASSERTS`. Confirmed present, no change needed.

#### Phase E — E4: PRIV handshake default (deferred — host-side)

E4 requires the host (Linux kmod / MCU host) to default to sending `ESP_PRIV_RPC_VERSION_ACK = 0x02` during the TLV handshake so that two new-codebase peers automatically negotiate V2.
The CP side already handles this correctly in `host_to_slave_reconfig()`.
E4 is a **host-side change** — it belongs in Phase H (host phases, not yet started).
Marked deferred in Phase E table; will be implemented when Phase H begins.

#### Phase E — E5: build gate

Not run. Requires IDF environment.
Run: `idf.py -C esp_hosted_fg/coprocessor/examples/cp_linux_fg/minimal/wifi/ build`
Or:  `idf.py -C esp_hosted_fg/coprocessor/examples/cp_mcu/minimal/wifi/ build`

Both examples pull in `repo_root/components/coprocessor` and
`repo_root/components/common/serializers` via EXTRA_COMPONENT_DIRS in their
top-level CMakeLists.txt — they already point at the canonical new layout.

#### Summary table

| Step | Status |
|------|--------|
| E1 — `esp_hosted_cp_master_config.h` | ✅ Done |
| E2 — Replace `CONFIG_ESP_HOSTED_CP_*` in coprocessor sources | ✅ Done (4 files) |
| E3 — `_Static_assert` in `rpc_id_map_v1.h` | ✅ Done (pre-existing from Session 13) |
| E4 — PRIV handshake host default | ⚠️ Deferred (host-side, Phase H) |
| E5 — `idf.py build` Phase E exit gate | ❌ Not run |

---

### Session 20 — 2026-03-13  Build gate D9/E5: CMake + compile fixes

**Build got to 1070/1119 objects before failing — good progress. Three compile bugs fixed:**

#### Fix 1 — `msg_codec` EXTRA_COMPONENT_DIRS (all 11 examples)
All 11 example `CMakeLists.txt` now include `${repo_root}/components/common/serializers/third_party` so IDF can find the `msg_codec` protobuf-c runtime component. Three examples were still missing it: `cp_mcu/extensions/host_power_save`, `cp_mcu/extensions/network_split/station`, `cp_mcu/extensions/network_split/iperf`.

#### Fix 2 — `esp_hosted_cp_ext_feat_host_ps` removed from core REQUIRES (Decision 11)
**Root cause**: `esp_hosted_cp_core/CMakeLists.txt` had `esp_hosted_cp_ext_feat_host_ps` in `REQUIRES` and `core.c` / `cli.c` both included `esp_hosted_cp_ext_feat_host_ps_apis.h` — violating the hard invariant that core never depends on extensions.

**Fix in `core.c`**:
- `esp_hosted_cp_ext_feat_host_ps_apis.h` is included only under
  `CONFIG_ESP_HOSTED_CP_EXT_HOST_PS_READY`, so the core stays extension‑agnostic.
- No weak stubs are used; the extension builds only when the config is enabled.

**Fix in `CMakeLists.txt`**: Removed `esp_hosted_cp_ext_feat_host_ps` from `REQUIRES`. Removed `console` from `PRIV_REQUIRES` (moved to CLI extension).

#### Fix 3 — `esp_hosted_cp_cli` extracted to its own extension

`cli.c` and `cli.h` moved out of `esp_hosted_cp_core` into a new extension:
`components/coprocessor/extensions/esp_hosted_cp_ext_cli/`

| File | Action |
|------|--------|
| `esp_hosted_cp_core/src/esp_hosted_cp_cli.c` | Moved to `ext_cli/src/` |
| `esp_hosted_cp_core/include/esp_hosted_cp_cli.h` | Moved to `ext_cli/include/` |
| `ext_cli/CMakeLists.txt` | New: REQUIRES `esp_hosted_cp_core`, `esp_hosted_cp_ext_feat_host_ps`, `esp_console`, `lwip`, `esp_wifi` |
| `esp_hosted_cp_core/CMakeLists.txt` | Removed `esp_hosted_cp_cli.c` from SRCS; removed `console` from PRIV_REQUIRES |
| `esp_hosted_cp_core.c` | Removed `#include "esp_hosted_cp_cli.h"` and `#include "esp_hosted_log.h"` (was already unused) |

The `ext_cli` extension REQUIRES `esp_hosted_cp_ext_feat_host_ps` directly and uses its header properly. The `extern` forward-declaration workaround is gone. Kconfig symbol `ESP_HOSTED_CP_EXT_CLI` and `EH_CP_CLI_ENABLED` in `master_config.h` stay in core (configuration is a core concern; implementation is an extension concern).

#### Fix 4 — Proto include path in rpc_fg and rpc_mcu extensions

`esp_hosted_cp_ext_rpc_fg_priv.h` includes `esp_hosted_cp_ext_rpc_fg_pbuf.h` (a shim header that lives in `esp_hosted_proto_linux_v1/include/`). Similarly `rpc_mcu_priv.h` includes `esp_hosted_cp_ext_rpc_mcu_pbuf.h`.

IDF's `REQUIRES` propagates include dirs for public API consumers but not for `PRIV_INCLUDE_DIRS` users within the same component. Fix: added `idf_component_get_property` + `target_include_directories` after `idf_component_register` in both:

```cmake
# rpc_fg CMakeLists.txt
idf_component_get_property(proto_fg_inc esp_hosted_proto_linux_v1 INCLUDE_DIRS)
target_include_directories(${COMPONENT_LIB} PRIVATE ${proto_fg_inc})

# rpc_mcu CMakeLists.txt
idf_component_get_property(proto_mcu_inc esp_hosted_proto_mcu_v1 INCLUDE_DIRS)
target_include_directories(${COMPONENT_LIB} PRIVATE ${proto_mcu_inc})
```

#### Extension directory after Session 20

| Extension | Description | Auto-init |
|-----------|-------------|-----------|
| `esp_hosted_cp_ext_cli` | **NEW** — diagnostic CLI commands | No (called by app) |
| `esp_hosted_cp_ext_feat_host_ps` | Host power-save state machine | Yes (priority 50) |
| `esp_hosted_cp_ext_rpc_fg` | Linux FG RPC adapter | Yes (priority 100) |
| `esp_hosted_cp_ext_rpc_mcu` | MCU RPC adapter | Yes (priority 100) |
| `esp_hosted_cp_ext_feat_nw_split` | Network split | No (user init) |
| `esp_hosted_cp_ext_custom_msg` | Custom RPC | No (user init) |
