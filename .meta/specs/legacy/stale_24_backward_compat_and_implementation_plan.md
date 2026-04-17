# 24 — Backward Compatibility Design & Consolidated Implementation Plan
<!-- Created: 2026-03-24. Captures all decisions from the backward-compat design session. -->
<!-- Supersedes stale portions of spec 06 (range was wrong) and spec 08 (compat logic was wrong). -->
<!-- All prior phase decisions (C–G) remain valid; this spec adds phases H-COMPAT and I-V2. -->

---

## 1. Final Goal (Restated)

One unified CP firmware (`esp_hosted.new`) that:
- Sends a **V2 wire header** (`0xE9` magic, 20 bytes) by default
- Speaks a **single unified proto v2** (`0x2000–0x7FFF`) serving both Linux and MCU hosts
- Compat Kconfig modes let it **imitate old deployed FG or MCU slave** for testing
- New host auto-detects old vs new CP from the first frame — silently, no config needed
- Old FG Linux kmod and old MCU host remain unmodified and work with the new unified host

---

## 2. Three Deployed CP Variants — Wire Fingerprints

The host identifies which CP it is talking to by inspecting the first PRIV bootup frame.

### 2.1 Header — byte 0 is the primary discriminator

```
Byte 0 of any frame from CP:
  == 0xE9  →  New unified CP. V2 header (20 bytes). V2 serialiser by default.
  != 0xE9  →  Old CP. V1 header (12 bytes). Inspect further (see 2.2).
```

### 2.2 Old CP disambiguation — bytes inside V1 header PRIV frame

When byte 0 ≠ 0xE9, the host is talking to an old CP. The host reads:
- **`if_type`** (lower nibble of byte 0 in V1 header) — expected to be `ESP_PRIV_IF`
- **`priv_pkt_type`** (byte 11, union last byte of V1 header)
- **`event_type`** (first byte after header offset, inside `struct esp_priv_event`)

| Deployed firmware | `if_type` (wire) | `priv_pkt_type` | `event_type` | Endpoints | Proto |
|---|---|---|---|---|---|
| Old h4 FG slave (deployed) | `0x04` (PRIV=4, 0-indexed) | `0x00` | `0x00` | `"ctrlResp"` / `"ctrlEvnt"` | CtrlMsg (100–308) |
| Old MCU slave (deployed) | `0x05` (PRIV=5, 1-indexed) | `0x33` | `0x22` | `"RPCRsp"` / `"RPCEvt"` | Rpc (0x100–0x314) |
| New unified CP (default) | `0x05` (PRIV=5, 1-indexed) | `0x11` (new) | `0x44` (new) | `"RPCReqV2"` / `"RPCEvtV2"` | RpcV2 (0x2000–0x7FFF) |

**Note:** New unified CP uses new distinct values `0x11` / `0x44` for `priv_pkt_type` /
`event_type` so it cannot be confused with either old variant even when sending V1 header
(compat modes). The exact values `0x11` / `0x44` are proposals — must be locked in
`esp_hosted_common_tlv.h` before any release.

### 2.3 Interface Type Numbering

| | Old h4 FG slave | Old MCU slave | New unified CP |
|---|---|---|---|
| `STA_IF` wire value | `0` | `1` | `1` |
| `AP_IF` wire value | `1` | `2` | `2` |
| `SERIAL_IF` wire value | `2` | `3` | `3` |
| `HCI_IF` wire value | `3` | `4` | `4` |
| `PRIV_IF` wire value | `4` | `5` | `5` |
| `TEST_IF` wire value | `5` | `6` | `6` |

Old h4 FG slave is **0-indexed** (no `ESP_INVALID_IF=0`). Old MCU and new unified CP are
**1-indexed** (`ESP_INVALID_IF=0` shifts everything up). `esp_hosted_if_type_to_wire()` /
`esp_hosted_if_type_from_wire()` in `esp_hosted_common_interface.h` handle this via
`CONFIG_ESP_HOSTED_LEGACY_WIRE_INTERFACE_TYPES` Kconfig.

### 2.4 Old h4 FG Slave — No Host Reconfig Message

The old h4 FG slave sends **no version negotiation TLVs** in its PRIV bootup event.
It only sends capability tags `0x00–0x04` (`ESP_PRIV_CAPABILITY`,
`ESP_PRIV_FIRMWARE_CHIP_ID`, `ESP_PRIV_FW_DATA`, `ESP_PRIV_SPI_CLK_MHZ`,
`ESP_PRIV_TEST_RAW_TP`). The new host must detect this (no `0x20`/`0x22` TLVs present)
and skip version negotiation entirely, treating the connection as V1 header + FG CtrlMsg.

---

## 3. Kconfig Mode — Single Choice (Corrected)

**Replaces the scattered `CONFIG_ESP_HOSTED_LEGACY_*` flags.**

```kconfig
choice ESP_HOSTED_CP_RPC_MODE
    prompt "RPC serialisation mode"
    default ESP_HOSTED_CP_RPC_V2

config ESP_HOSTED_CP_RPC_V2
    bool "RPC V2 — unified proto (new hosts)"
    help
      Default. New unified CP. V2 header (0xE9) + unified proto (0x2000–0x7FFF).
      Endpoints: "RPCReqV2" / "RPCEvtV2".
      priv_pkt_type=0x11, event_type=0x44.
      Both Linux and MCU new hosts use the same endpoints and proto schema.

config ESP_HOSTED_CP_RPC_V1_LINUX
    bool "Compat: imitate old Linux FG slave (esp-hosted h4 deployed)"
    help
      Imitates the old h4 FG slave for regression testing with a new host.
      V1 header (12 bytes, no 0xE9). Endpoints "ctrlResp"/"ctrlEvnt".
      CtrlMsg proto (msg_ids 100–308). priv_pkt_type=0x00, event_type=0x00.
      if_type is 0-indexed on wire (CONFIG_ESP_HOSTED_LEGACY_WIRE_INTERFACE_TYPES=y).
      No host reconfig TLVs sent.

config ESP_HOSTED_CP_RPC_V1_MCU
    bool "Compat: imitate old MCU slave (esp_hosted_mcu deployed)"
    help
      Imitates the old MCU slave for regression testing with a new host.
      V1 header (12 bytes, no 0xE9). Endpoints "RPCRsp"/"RPCEvt".
      Rpc proto (msg_ids 0x100–0x314). priv_pkt_type=0x33, event_type=0x22.
      if_type is 1-indexed on wire.

endchoice
```

**Semantics:**
- `ESP_HOSTED_CP_RPC_V2` (default): CP is the **new unified CP**. Uses V2 header and V2
  serialiser. Requires a new host that speaks V2.
- `ESP_HOSTED_CP_RPC_V1_LINUX`: CP **imitates the old h4 FG slave**. Used only for testing
  that the new host correctly handles the old FG wire format.
- `ESP_HOSTED_CP_RPC_V1_MCU`: CP **imitates the old MCU slave**. Used only for testing that
  the new host correctly handles the old MCU wire format.

**The compat modes are not for production.** Production deployments use `ESP_HOSTED_CP_RPC_V2`.

---

## 4. Host Auto-Detection (New Host Side)

The new host (new Linux kmod or new MCU host driver) always defaults to V2 but
auto-detects old CPs from the first received frame. No Kconfig needed on the host.

```
New host receives first frame from CP:

  if (frame[0] == 0xE9):
    → New unified CP (V2 header)
    → Parse 20-byte V2 header
    → Expect V2 serialiser (RPCReqV2 / RPCEvtV2 / 0x2000–0x7FFF)
    → Proceed with V2 handshake

  else:
    → Old CP (V1 header, 12 bytes)
    → Read priv_pkt_type = frame[11]
    → Read event_type = frame[sizeof(V1_header) + 0]

    if (priv_pkt_type == 0x00 AND event_type == 0x00):
      → Old h4 FG slave
      → if_type is 0-indexed; apply ESP_LEGACY_* mapping
      → Use CtrlMsg serialiser; endpoints "ctrlResp"/"ctrlEvnt"
      → Skip version negotiation (no TLVs expected)
      → Parse capability tags 0x00–0x04

    elif (priv_pkt_type == 0x33 AND event_type == 0x22):
      → Old MCU slave
      → if_type is 1-indexed (standard)
      → Use Rpc serialiser; endpoints "RPCRsp"/"RPCEvt"
      → Proceed with V1 handshake (TLVs 0x11–0x17, no 0x20/0x22)

    else:
      → Unknown CP variant. Log warning. Best-effort V1 MCU parse.
```

**The host never requires manual configuration to support old CPs.** Detection
is automatic from the first frame.

---

## 5. RPC V2 Unified Schema — Corrected Ranges

*Spec 06 had an incorrect placeholder range (`0x400–0x9FF`). Correct values per spec 06 section 4:*

| Direction | Range | Sentinel |
|---|---|---|
| Requests | `0x2000 – 0x3FFE` | `0x3FFF` |
| Responses | `0x4000 – 0x5FFE` | `0x5FFF` |
| Events | `0x6000 – 0x7FFE` | `0x7FFF` |

Gap between req and resp base: `0x2000`. Symmetrical. 8190 IDs per direction.
Sentinels at `0x3FFF` / `0x5FFF` / `0x7FFF` — do NOT move when adding IDs.

These ranges are in `esp_hosted_rpc_id_map_v2.h` (see spec 14 Decision 6).

---

## 6. V2 Endpoint Names (Decided)

| Direction | Endpoint name |
|---|---|
| Host → CP (requests) | `"RPCReqV2"` |
| CP → Host (events) | `"RPCEvtV2"` |

**Rationale:** Clear version suffix, no collision with `"RPCRsp"` / `"RPCEvt"` (MCU V1)
or `"ctrlResp"` / `"ctrlEvnt"` (FG V1). 8 chars each, consistent length.
These strings must be locked in `esp_hosted_cp_ext_rpc_ll.h` and never changed.

---

## 7. RPC Version Field Inside V2 Proto

The V2 unified proto (`esp_hosted_rpc_v2.proto`) must include an `rpc_version` field
in the top-level message to allow independent version tracking of the schema:

```protobuf
message RpcV2 {
    uint32 msg_id      = 2;   // at field 2 for scanner compatibility
    uint32 rpc_version = 3;   // e.g. 0x0200 = v2.0; increment on schema changes
    MsgType msg_type   = 4;
    // ... payload oneof
}
```

This allows CP and host to cross-verify schema version within the proto payload,
independent of the wire header negotiation. Future schema bumps increment this field.

---

## 8. Proto V2 Architecture — Single File, Per-Feature Handler Ranges

**Decision:** Keep one unified `esp_hosted_rpc_v2.proto` file. Do **not** split into
per-feature proto files.

**Rationale:** Feature mini-protos introduce field-number ownership conflicts in the
`oneof` of the top-level message, break the `msg_id` field-2 scanner, complicate
`protoc` codegen dependencies, and make schema version management incoherent. The C
handler registration layer already provides per-feature modularity via range registration.

Feature modularity is achieved at the C level (each feature extension registers its own
ID sub-range within `0x2000–0x7FFF`) not at the proto level.

---

## 9. RPC Endpoint Registration — One Endpoint Pair at a Time

At runtime, only the endpoint pair matching the negotiated mode is active:
- `ESP_HOSTED_CP_RPC_V2` (default): only `"RPCReqV2"` / `"RPCEvtV2"` registered
- `ESP_HOSTED_CP_RPC_V1_LINUX`: only `"ctrlResp"` / `"ctrlEvnt"` registered
- `ESP_HOSTED_CP_RPC_V1_MCU`: only `"RPCRsp"` / `"RPCEvt"` registered

The current dual-registration of FG + MCU endpoints is a transitional state.
Post-PENDING-013, endpoint registration is driven by the Kconfig choice at boot,
not by dual-always-on registration.

---

## 10. NG (Next-Gen) Compat — Explicitly Out of Scope

NG uses an incompatible command/event architecture: different `if_type` enum (no
`SERIAL_IF`, different numbering), `esp_internal_bootup_event` instead of
`esp_priv_event`, `PACKET_TYPE_DATA/COMMAND_REQUEST/etc.` instead of
`ESP_PACKET_TYPE_EVENT`. NG compat is deferred (`PENDING-001/002`) and requires
separate design work.

---

## 11. Peer Data Transfer Extension — Status (Completed This Session)

### MCU path (fully working)
- Inbound (`Req_CustomRpc 0x184` → `peer_data_dispatch`): complete
- Outbound (`peer_data_send` → `peer_data_rpc_send` hook → `rpc_send_event` →
  `rpc_evt_custom_rpc` → proto → host): complete
- `ESP_HOSTED_EXT_CP_EVT_RPC_CUSTOM_EVENT` event enum: **removed**
- `ext_custom_rpc_event_dispatcher` + subscribe/unsubscribe: **removed**
- `rpc_evt_custom_rpc_unserialised_msg` **renamed** → `rpc_evt_custom_rpc`
- `app_main.c` msg_ids corrected (CAT=1/MEOW=2, DOG=3/WOOF=4, HUMAN=5/HELLO=6)
- Test result: 30/30 messages, all sizes (CAT/DOG/HUMAN), responses received ✅

### FG path (complete after this session)
- Inbound (`Req_CustomRPC 128` → `peer_data_dispatch`): was already complete
- Outbound: `peer_data_rpc_send` hook added to `rpc_fg_core.c`, registered at init
- `ext_custom_rpc_event_dispatcher` + subscribe/unsubscribe: **removed**
- `rpc_fg_priv.h`: subscribe/unsubscribe declarations **removed**
- `rpc_fg/CMakeLists.txt`: `target_include_directories` added for peer_data headers

### Design decision (peer_data send hook pattern)
`peer_data` extension does NOT use `esp_event_post` for the send path.
Reasons: `esp_event` is **asynchronous** (`xQueueSendToBack` → returns before handler runs),
so a pointer in event_data would dangle. FAM (flexible array member) inline copy would work
but causes double-copy for large payloads. The correct design is a **registered function
pointer** (`esp_hosted_cp_peer_data_send_fn_t`) that `rpc_mcu`/`rpc_fg` register at their
init. `peer_data_send` calls the hook directly — synchronous, one copy, explicit error return,
no silent drops.

`esp_event` is correct for **state notifications** (WiFi connected, network up/down) where
data is small fixed structs. It is wrong for **variable-length data transfer** paths.

---

## 12. Log Verbosity Changes (This Session)

Demoted from `ESP_LOGI` to `ESP_LOGD` (too noisy per-packet):
- `mcu_rpc_req_handler: req msg_id=... resp msg_id=... uid=...`
- `mcu_rpc_event_handler: event_id=... inbuf=... inlen=...`
- `Received Req [0x...]`
- `serial_rx: len=...`

Kept at `ESP_LOGI` (useful for fragment reassembly visibility):
- `serial_rx_pkt: seq=... flags=... payload_len=... assembled=...`

---

## 13. What Is Complete

| Item | Status |
|---|---|
| V2 wire header (20 bytes, 0xE9 magic, hdr_version=0x02) | ✅ Phase 7 complete |
| V2 header negotiation via PRIV TLV handshake | ✅ Phase 7 complete |
| RPC realloc-table registry (binary search req, linear evt) | ✅ Phase D complete |
| Extension auto-init via linker section (`EH_CP_EXT_REGISTER`) | ✅ Phase D complete |
| Protocomm moved to `esp_hosted_cp_ext_rpc` (priority 5) | ✅ Phase G complete |
| Interface type wire compat (`to_wire`/`from_wire` Kconfig-gated) | ✅ complete |
| FG Linux kmod V2 header detection + negotiation | ✅ complete |
| Peer data MCU path (inbound + outbound) | ✅ this session |
| Peer data FG path (inbound + outbound) | ✅ this session |
| `ESP_HOSTED_EXT_CP_EVT_RPC_CUSTOM_EVENT` removed from event enum | ✅ this session |
| `peer_data_send` direct-hook design replacing `esp_event_post` | ✅ this session |
| Log verbosity cleanup | ✅ this session |

---

## 14. What Is Remaining (Priority Order)

### PENDING-013 — Two-Stage PRIV Handshake + Single Endpoint Registration
- Currently both FG and MCU endpoints are always registered simultaneously
- Must: register only the Kconfig-selected endpoint pair at boot
- `ESP_HOSTED_CP_EVT_PRIVATE_RPC_READY` mechanism designed in spec 19 but not wired
- **Blocks:** Kconfig-clean compat mode; clean V2-only default mode
- **Risk:** Medium. Well-specified in spec 19. Mainly plumbing.

### PENDING-011 — Idempotent Extension Init/Deinit
- Extensions must check and set a feature bit on first init, clear on deinit
- Prevents double-init on host reset cycles
- **Risk:** Low. Mechanical.

### PENDING-012 — Extension API Naming + Guards
- All public APIs must have full extension prefix
- No weak stubs; extension targets are compiled only when enabled
- **Risk:** Low. Mostly renaming.

### Phase F — Unified Proto V2 Schema
- Design `esp_hosted_rpc_v2.proto` — single file, all WiFi + BT + OTA + System RPCs
- ID range: `0x2000–0x7FFF` (req/resp/evt as above)
- Endpoint: `"RPCReqV2"` / `"RPCEvtV2"`
- Include `rpc_version` field at proto field 3
- Both FG Linux and MCU hosts use same IDs
- `protoc-c` codegen → `esp_hosted_proto_v2/`
- **Blocks:** Everything else at V2 level
- **Risk:** High. Largest single design piece. Proto field layout must be locked forever.

### Phase I-COMPAT — Kconfig Choice Implementation
- Wire the `choice ESP_HOSTED_CP_RPC_MODE` into the build
- `ESP_HOSTED_CP_RPC_V1_LINUX`: set `priv_pkt_type=0x00`, `event_type=0x00`,
  0-indexed if_type, register only `"ctrlResp"/"ctrlEvnt"`
- `ESP_HOSTED_CP_RPC_V1_MCU`: set `priv_pkt_type=0x33`, `event_type=0x22`,
  1-indexed if_type, register only `"RPCRsp"/"RPCEvt"`
- `ESP_HOSTED_CP_RPC_V2`: set `priv_pkt_type=0x11`, `event_type=0x44`,
  1-indexed if_type, register only `"RPCReqV2"/"RPCEvtV2"`
- **Blocks:** Regression testing of old-host compat

### Phase I-COMPAT (Follow-up) — Core-Owned Startup Frame + Slave Intr
- **Goal:** keep `generate_slave_intr` in core and move **startup frame creation**
  (PRIV TLVs + header encode) into core. Transports should only transmit bytes,
  not decide endpoints or compose TLVs.
- **Why it was in transport:** legacy CP code built the bootup TLVs in each
  transport because the driver owned queue sizes, link-specific caps, and the
  raw bus TX path before the frame component existed.
- **What changes are needed:**
  - Create a core-owned `esp_hosted_cp_build_startup_event()` that assembles
    the TLV payload and calls `esp_hosted_frame_encode()` (core owns endpoint
    names and RPC mode).
  - Add a tiny transport hook (per bus) to **append transport-specific TLVs**
    (queue sizes, link caps) into a provided buffer, or return a small struct
    of transport caps for core to encode.
  - Add a transport TX shim (`esp_hosted_cp_transport_send_priv_frame(buf,len)`)
    so core can enqueue the bootup event without including transport headers.
  - Remove the need for `esp_hosted_cp_master_config.h` from transport files by
    keeping endpoint selection in core only.
  - Ensure any `generate_slave_intr` calls remain core-facing (transport only
    executes the wire action).

### Phases H1–H6 — New Unified Host (Largest Body of Work)
- Portable host engine, RPC client, queue abstraction, WiFi/BT/OTA extensions
- Linux port (new kmod) + MCU port (new host driver)
- **Status:** Not started
- **Risk:** Highest. This is the bulk of the remaining work.

### NG Compat (PENDING-001/002)
- Deferred. Structurally incompatible. Separate design required.

---

## 15. Spec Cross-References — What Changed

| Spec | Status | Note |
|---|---|---|
| `02_wire_protocol.md` | Valid | V2 header structure correct |
| `03_priv_handshake.md` | Valid | TLV definitions correct |
| `04_three_registry_api.md` | Valid | Realloc table design complete |
| `06_rpc_versioning.md` | ⚠️ Partially stale | Section 4 range `0x2000–0x7FFF` is correct; ignore any `0x400–0x9FF` reference (was wrong placeholder). Compat logic corrected by this spec. |
| `08_compatibility_matrix.md` | ⚠️ Partially stale | Compat model was inverted. Host auto-detects CP; compat Kconfig is CP-side only for imitation. Matrix needs refresh. |
| `12_rpc_call_flow.md` | ⚠️ References SLIST | SLIST fully replaced by realloc table. Flows still valid conceptually. |
| `14_design_decisions_v2.md` | Valid | All 12 decisions still apply |
| `19_esp_hosted_cp_ext_rpc_design.md` | Valid | Spec for Phase G (protocomm moved). PENDING-013 not yet implemented. |
| `PHASES.md` | Needs update | Add Phase I-COMPAT; mark peer_data session items complete |
