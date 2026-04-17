# 08 — Compatibility Matrix
<!-- ⚠️ PARTIALLY STALE as of 2026-03-24 (spec 24). -->
<!-- The compat model in this spec is inverted from the correct design. -->
<!-- Correct model: compat Kconfig is CP-side only (for imitation); host auto-detects old vs new CP. -->
<!-- See spec 24 sections 2–4 for the authoritative wire fingerprint and host detection logic. -->
<!-- Matrix rows/columns need refresh once Phase I-COMPAT is implemented. -->

## 1. Host × Slave Interoperability Table

Rows = slave firmware version.  Columns = host driver version.

| Slave ↓ / Host → | Old FG Linux kmod | New kmod (Phase 3/5) | MCU host V1 | MCU host V2 (Phase 5) |
|-------------------|-------------------|----------------------|-------------|----------------------|
| **Old FG (0x00–0x04 TLVs only)** | ✅ Full | ⚠️ Degraded¹ | ⚠️ Degraded¹ | ⚠️ Degraded¹ |
| **MCU V1 (0x11–0x17)** | ⚠️ Partial² | ✅ Full | ✅ Full | ✅ Full |
| **MCU V2 (0x11–0x19 + Phase 3/5)** | ⚠️ Partial³ | ✅ Full | ✅ Full V1 | ✅ Full V2 |

**Notes:**
1. Host parses unknown tags 0x00-0x04 as "Unsupported tag" warnings; falls back to MCU-style TLV parsing which is absent in old slave → no caps discovered.  Connection may still work if host has compiled-in defaults.
2. New host handles deprecated tags 0x00–0x04 with deprecation log, derives caps from them; old slave never sends 0x18/0x19/0x20+ → missing feat_caps, negotiation TLVs absent, host stays at V1 header and V1 RPC.
3. New slave sends 0x11–0x19 + 0x20–0x23; old kmod ignores unknown tags (0x18, 0x19, 0x20, 0x21, 0x22, 0x23) → caps partially parsed, V2 negotiation not performed.

---

## 2. TLV Version Compatibility Per Tag

| Tag | Slave emits it? | Host must handle it? | If missing: |
|-----|-----------------|---------------------|-------------|
| 0x00–0x04 | Old slaves only | Accept + warn | No legacy caps |
| 0x11–0x17 | All current | Must handle | No caps discovered |
| 0x18 | MCU repo only | Accept (log only) | No SDIO mode info |
| 0x19 | New unified | Accept (log only) | No feat_caps |
| 0x20 | New unified | Must handle to upgrade | Stays at V1 header |
| 0x21 | Host only (ACK) | CP must handle | No V2 header on CP |
| 0x22 | New unified | Must handle to upgrade | Stays at V1 RPC |
| 0x23 | Host only (ACK) | CP must handle | No V2 RPC on CP |
| 0x44–0x48 | Host only (config) | CP must handle | No host config |

---

## 3. Wire Header Version Negotiation Outcomes

| CP max ver | Host max ver | Agreed ver | Result |
|------------|--------------|------------|--------|
| V1 (no TLV) | any | V1 | 12-byte header |
| V2 | V1 (no ACK) | V1 (timeout) | 12-byte header |
| V2 | V2 | V2 | 20-byte header (PENDING-005) |

CP falls back to V1 if no ACK received within startup window.

---

## 4. RPC Version Negotiation Outcomes

| CP RPC ver | Host RPC ver | Agreed ver | Dispatch path |
|------------|--------------|------------|---------------|
| V1 (no TLV) | any | V1 | FG: `RPCRsp`/`RPCEvt` protocomm EP |
| V2 | V1 | V1 | Same as above |
| V2 | V2 | V2 | msg_id 0x400+ (PENDING, Phase 5 schema) |

---

## 5. Repository Origin per Component

| Component | Source repo | Notes |
|-----------|-------------|-------|
| `esp_hosted_transport_cp` (SPI/SDIO/UART/SPI-HD) | `esp_hosted_final` | Adapted + Phase 3/5 TLVs added |
| `esp_hosted_cp_core` | `esp_hosted_final` | Extended with dynamic table registries + `EH_CP_EXT_REGISTER` auto-init |
| `transport_drv.c` (MCU host) | `esp_hosted_mcu` | Phase 3/5 negotiation added |
| `esp_spi.c` / `esp_sdio.c` (kmod) | `esp_hosted_final` kmod | Phase 3/5 negotiation added |
| `esp_hosted_caps.h` | merged from both | FG + MCU TLV enums unified |
| `esp_hosted_common_tlv.h` | NEW | Canonical single-source TLV definitions |
| Port headers (`port/host/*/`) | NEW | Phase 4 H_XXX abstraction |
