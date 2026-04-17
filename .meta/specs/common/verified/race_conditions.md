<!-- %% sp.cm.ve-rc.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# Race Conditions & Safety

<!-- %% sp.cm.ve-rc.fix.o %% - always -->
## Fixed Bugs

| ID | Severity | Location | Fix |
|----|----------|----------|-----|
| RC-001 | CRITICAL | `eh_common_tlv.h` | `#define HOST_CAPABILITIES 0x01` shadowed enum value 0x44 — removed macro |
| RC-002 | HIGH | All 4 TLV parsers | `len_left` as `uint8_t` wraps on underflow — cast to `uint16_t` bounds check |
| RC-003 | HIGH | `esp_sdio.c`, `esp_spi.c` | Missing `ESP_PRIV_FEAT_CAPS` (0x19) parsing — added |
| RC-004 | HIGH | `eh_caps.h` | Missing `ESP_PRIV_TRANS_SDIO_MODE` (0x18) in enum — added |
| RC-005 | MEDIUM | `cp_core.c` | `hdr_ver_negotiated` init to 0 instead of `ESP_HOSTED_HDR_VERSION_V1` — fixed |
| RC-006 | MEDIUM | `esp_spi.c`, `esp_sdio.c` | SMP: no `WRITE_ONCE` on adapter fields — added |
<!-- %% sp.cm.ve-rc.fix.c %% -->

<!-- %% sp.cm.ve-rc.open.o %% - context -->
## Open / Deferred Risks

| ID | Severity | Location | Status |
|----|----------|----------|--------|
| RC-007 | LOW | `cp_core.c` | `volatile` on negotiation vars (Xtensa dual-core) — documented, safe for single-byte |
| RC-008 | — | All transports | V2 header TX/RX path not implemented — PENDING-005 (deferred to post-V1) |
<!-- %% sp.cm.ve-rc.open.c %% -->

<!-- %% sp.cm.ve-rc.c %% -->
