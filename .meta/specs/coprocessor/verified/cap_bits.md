<!-- %% sp.co.ve-cb.o %% - context -->
---
type: spec
last_verified: 2026-04-07
---

# Capability Bits Architecture

<!-- %% sp.co.ve-cb.overview.o %% - always -->
## Overview

Capability bits advertise feature support from CP to host during the PRIV boot-time handshake (TLV stream). The host uses these to enable/disable feature-specific code paths.

Three tiers exist for historical and sizing reasons:

| Tier | Field | Wire type | TLV tag | Purpose |
|------|-------|-----------|---------|---------|
| 1 | `caps` | `uint8_t` | `0x11` (`EH_PRIV_CAPABILITY`) | Transport-level (WLAN, BT, SPI/SDIO, checksum) |
| 2 | `ext_caps` | `uint32_t` | `0x16` (`EH_PRIV_CAP_EXT`) | Extended features (enterprise, DPP, host PS, nw split, custom RPC) |
| 3 | `feat_caps[8]` | `uint32_t[8]` | `0x19` (`EH_PRIV_FEAT_CAPS`) | Per-feature bitmask array (V2 TLV group only) |

All bit definitions live in `eh_common_caps.h` (single source of truth, shared by CP and host).
Transport-specific TLV tags and legacy structs live in `eh_caps.h`.
<!-- %% sp.co.ve-cb.overview.c %% -->

<!-- %% sp.co.ve-cb.tier1.o %% - context -->
## Tier 1: `caps` (uint8_t)

Legacy 8-bit word. Set by transport layer at init based on Kconfig.

| Bit | Define | Meaning |
|-----|--------|---------|
| 0 | `ESP_WLAN_SDIO_SUPPORT` | WLAN over SDIO |
| 1 | `ESP_BT_UART_SUPPORT` | BT over UART |
| 2 | `ESP_BT_SDIO_SUPPORT` | BT over SDIO |
| 3 | `ESP_BLE_ONLY_SUPPORT` | BLE only (no BR/EDR) |
| 4 | `ESP_BR_EDR_ONLY_SUPPORT` | BR/EDR only (no BLE) |
| 5 | `ESP_WLAN_SPI_SUPPORT` | WLAN over SPI |
| 6 | `ESP_BT_SPI_SUPPORT` | BT over SPI |
| 7 | `ESP_CHECKSUM_ENABLED` | Payload checksum enabled |

All 8 bits allocated. No room for expansion (use Tier 2/3 for new features).
<!-- %% sp.co.ve-cb.tier1.c %% -->

<!-- %% sp.co.ve-cb.tier2.o %% - context -->
## Tier 2: `ext_caps` (uint32_t)

Extended capability word. Set by features during init via `eh_cp_add_feat_cap_bits()`.

| Bit | Define | Meaning |
|-----|--------|---------|
| 0 | `ESP_WLAN_SUPPORT` | WLAN feature enabled |
| 1 | `ESP_WLAN_UART_SUPPORT` | WLAN over UART |
| 2 | `ESP_HOSTED_TRANSPORT_CP_SPI_HD_4_DATA_LINES` | SPI-HD 4 data lines |
| 3 | `ESP_HOSTED_TRANSPORT_CP_SPI_HD_2_DATA_LINES` | SPI-HD 2 data lines |
| 4 | `ESP_EXT_CAP_WIFI_ENT` | WiFi Enterprise |
| 5 | `ESP_EXT_CAP_WIFI_DPP` | WiFi DPP |
| 6 | `ESP_EXT_CAP_HOST_PS` | Host power save |
| 7 | `ESP_EXT_CAP_NW_SPLIT` | Network split |
| 8 | `ESP_EXT_CAP_CUSTOM_RPC` | Custom RPC (peer data) |
| 9-31 | — | Reserved |

23 bits available for future features.
<!-- %% sp.co.ve-cb.tier2.c %% -->

<!-- %% sp.co.ve-cb.tier3.o %% - context -->
## Tier 3: `feat_caps[8]` (uint32_t array)

Per-feature bitmask array. Each index is an independent 32-bit word. Features set bits via `eh_cp_add_feat_cap_bits_idx(index, mask)` during init.

| Index | Define | Feature area |
|-------|--------|-------------|
| 0 | `ESP_HOSTED_FEAT_IDX_WIFI` | WiFi feature flags |
| 1 | `ESP_HOSTED_FEAT_IDX_BT` | BT/BLE feature flags |
| 2 | `ESP_HOSTED_FEAT_IDX_OTA` | OTA feature flags |
| 3 | `ESP_HOSTED_FEAT_IDX_PS` | Power save feature flags |
| 4 | `ESP_HOSTED_FEAT_IDX_NW_SPLIT` | Network split feature flags |
| 5 | `ESP_HOSTED_FEAT_IDX_CUSTOM_RPC` | Custom RPC feature flags |
| 6-7 | — | Reserved |

Total capacity: 8 x 32 = 256 feature bits. Sent as `ESP_HOSTED_FEAT_CAPS_COUNT` (8) uint32_t values in TLV tag `0x19` **only when `EH_TLV_V2` is enabled** (i.e., RPC V2 builds).

Individual bit definitions within each index are feature-specific and defined by the owning feature module (not centralized).
<!-- %% sp.co.ve-cb.tier3.c %% -->

<!-- %% sp.co.ve-cb.api.o %% - context -->
## API

```c
// Set bits in ext_caps (Tier 2)
void eh_cp_add_feat_cap_bits(uint32_t bits);
void eh_cp_clear_feat_cap_bits(uint32_t bits);

// Set bits in feat_caps[index] (Tier 3)
void eh_cp_add_feat_cap_bits_idx(uint8_t index, uint32_t bits);
void eh_cp_clear_feat_cap_bits_idx(uint8_t index, uint32_t bits);
```

Called during feature init (typically in `EH_CP_FEAT_REGISTER` callback). Core accumulates all bits and sends the combined result in the PRIV TLV handshake.
<!-- %% sp.co.ve-cb.api.c %% -->

<!-- %% sp.co.ve-cb.design.o %% - context -->
## Design Rationale

**Why three tiers?**
- Tier 1 is legacy (V0 wire compat, 8 bits, cannot change).
- Tier 2 was added when 8 bits ran out (V1 wire extension).
- Tier 3 provides per-feature namespacing — avoids a single global bitmask that every feature fights over.

**Why `#define` not `enum`?**
- Bit flags are OR'd together: `caps |= ESP_WLAN_SDIO_SUPPORT | ESP_BLE_ONLY_SUPPORT`.
- Enums in C have implicit int promotion and don't compose well for bitmasks.
- `#define` with explicit `(1u << N)` is idiomatic for hardware/protocol bit fields.

**Wire-breaking rule:**
Adding, removing, or reordering any bit definition is a wire-breaking change. Bump `ESP_HOSTED_CAPS_VERSION` and update both CP and host.
<!-- %% sp.co.ve-cb.design.c %% -->

<!-- %% sp.co.ve-cb.c %% -->
