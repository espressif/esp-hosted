# 06 — RPC Versioning
<!-- Last updated: 2026-03-11 Session 15 — aligned with spec 14 decisions -->
<!-- ⚠️ PARTIAL UPDATE 2026-03-24 (spec 24): -->
<!--   Section 4 range 0x2000–0x7FFF is CORRECT. Any reference to 0x400–0x9FF is a stale wrong placeholder. -->
<!--   V2 endpoint names decided: "RPCReqV2" / "RPCEvtV2" (spec 24 section 6). -->
<!--   Compat logic (compat is CP-side only, host auto-detects) superseded by spec 24 section 3–4. -->

## 1. Version Codes

| Code | Name | Constant | Transport |
|------|------|----------|-----------|
| 0x01 | V1 | `ESP_HOSTED_RPC_VERSION_V1` | Proto over protocomm endpoints `RPCRsp`/`RPCEvt`; two separate schemas (FG + MCU) |
| 0x02 | V2 | `ESP_HOSTED_RPC_VERSION_V2` | Single unified schema; msg_id 0x2000–0x7FFF namespace (PENDING-006) |

## 2. Negotiation Flow

```
CP startup event  →  ESP_PRIV_RPC_VERSION (0x22) TLV, value = 0x02  (CP supports V2)
Host ACK frame    →  ESP_PRIV_RPC_VERSION_ACK (0x23) TLV, value = min(host_max, cp_max)
CP commits        →  rpc_ver_negotiated = agreed value
```

- Any host built against this codebase sends ACK = 0x02 → V2 negotiated.
- Old host (no ACK) → `rpc_ver_negotiated` stays at V1. No breakage.
- V2 is end-to-end: both Linux FG and MCU hosts implement V2 (Phase F).

## 3. Runtime Variables

| Variable | Location | Type | Init value |
|----------|----------|------|-----------|
| `rpc_ver_negotiated` | `esp_hosted_cp_core.c` | `volatile uint8_t` | `ESP_HOSTED_RPC_VERSION_V1` |
| `host_rpc_ver_agreed` | `transport_drv.c` (MCU host) | `volatile uint8_t` | `ESP_HOSTED_RPC_VERSION_V1` |
| `adapter->rpc_ver_negotiated` | `struct esp_adapter` (kmod) | `u8` | `ESP_HOSTED_RPC_VERSION_V1` |

## 4. RPC msg_id Namespace

Full authoritative map in `esp_hosted_rpc_id_map_v1.h` and `esp_hosted_rpc_id_map_v2.h`
under `components/common/esp_hosted_common/include/`.

| Range | Version | Direction | Schema |
|-------|---------|-----------|--------|
| 100–199 | V1 | FG requests | `esp_hosted_config.proto` (CtrlMsg) |
| 200–299 | V1 | FG responses | CtrlMsg |
| 300–399 | V1 | FG events | CtrlMsg |
| 0x100–0x1FF | V1 | MCU requests | `esp_hosted_rpc.proto` (Rpc) |
| 0x200–0x2FF | V1 | MCU responses | Rpc |
| 0x300–0x3FF | V1 | MCU events | Rpc |
| 0x2000–0x3FFF | V2 | Unified requests | `esp_hosted_rpc_v2.proto` (PENDING-006) |
| 0x4000–0x5FFF | V2 | Unified responses | V2 unified |
| 0x6000–0x7FFF | V2 | Unified events | V2 unified |

V2 sentinels (do NOT move when adding IDs): `0x3FFF` / `0x5FFF` / `0x7FFF`.

## 5. Dispatch Architecture

**V1 dispatch:** dynamic table binary search.
- Request table sorted ascending by `req_id_min`.
- FG V1 extension registered for [100, 199] req / [300, 399] evt.
- MCU V1 extension registered for [0x100, 0x1FF] req / [0x300, 0x3FF] evt.

**V2 dispatch:** same table mechanism, separate entries in [0x2000+] ranges.
- V1 and V2 entries coexist in the same table — ranges never overlap.
- `rpc_ver_negotiated` determines which ID range the wire sends; the dispatch
  table handles whichever ID arrives without per-version branching.

The SLIST-based dispatch from sessions 1–14 is **retired**. See spec 04.

## 6. Proto Component Names

| Version | Component | Schema file |
|---------|-----------|-------------|
| V1 FG   | `esp_hosted_proto_linux_v1` | `esp_hosted_config.proto` |
| V1 MCU  | `esp_hosted_proto_mcu_v1` | `esp_hosted_rpc.proto` |
| V2      | `esp_hosted_proto_v2` | `esp_hosted_rpc_v2.proto` (PENDING-006, empty placeholder) |

## 7. V2 Proto Design Requirements (PENDING-006)

1. Single `.proto` file — no FG/MCU split in naming.
2. Both Linux FG and MCU hosts use the same IDs over V2.
3. `msg_id` at proto field position 2 (lightweight scanner unchanged).
4. Must accommodate all V1 commands without renumbering V1 IDs.
5. V1 and V2 coexist in the same dispatch table — range isolation guarantees no collision.
