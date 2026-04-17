<!-- %% sp.cm.ve-ph.o %% - always -->
---
type: spec
last_verified: 2026-04-13
---

# PRIV Boot-Time Handshake

<!-- %% sp.cm.ve-ph.pkt.o %% - always -->
## Packet Structure

```
esp_payload_header (12B V1)
  if_type = ESP_PRIV_IF
  priv_pkt_type = ESP_PACKET_TYPE_EVENT (0x33)
esp_priv_event
  event_type = 0x22  ← struct field, NOT a TLV tag
  event_len  = <TLV byte count>
  event_data[0..event_len-1]  ← flat TLV stream
    [tag:1][len:1][value:len] ...
```

Flow: CP sends `ESP_PRIV_EVENT_INIT` → host parses TLVs → host sends `ESP_PRIV_EVENT_INIT_ACK` with agreed versions.
<!-- %% sp.cm.ve-ph.pkt.c %% -->

<!-- %% sp.cm.ve-ph.tlv.o %% - always -->
## TLV Tag Registry

**CP→Host (legacy FG, 0x00–0x04) — deprecated, still parsed:**

| Tag | Name | Value meaning |
|-----|------|---------------|
| 0x01 | `ESP_PRIV_FIRMWARE_CHIP_ID` | chip ID |
| 0x02 | `ESP_PRIV_SPI_CLK_MHZ` | SPI clock MHz |
| 0x03 | `ESP_PRIV_CAPABILITIES` | bitmask (FG-specific) |
| 0x04 | `ESP_PRIV_TEST_RAW_TP` | raw throughput test flag |

**CP→Host (MCU style, 0x11–0x17) — current V1 MCU group:**

| Tag | Name | Value |
|-----|------|-------|
| 0x11 | `EH_PRIV_CAPABILITY` | basic caps (uint8) |
| 0x12 | `EH_PRIV_FIRMWARE_CHIP_ID` | chip ID (uint8) |
| 0x13 | `EH_PRIV_TEST_RAW_TP` | raw TP flags (uint8) |
| 0x14 | `EH_PRIV_RX_Q_SIZE` | RX queue depth (uint8) |
| 0x15 | `EH_PRIV_TX_Q_SIZE` | TX queue depth (uint8) |
| 0x16 | `EH_PRIV_CAP_EXT` | extended caps (uint32 LE) |
| 0x17 | `EH_PRIV_FIRMWARE_VERSION` | FW version (uint32 LE) |

**V2-only TLVs (sent only when `EH_TLV_V2` / RPC V2 enabled):**

| Tag | Name | Value |
|-----|------|-------|
| 0x19 | `EH_PRIV_FEAT_CAPS` | feature caps array (uint32[N] LE) |
| 0x20 | `ESP_PRIV_HDR_VERSION` | proposed header version |
| 0x22 | `ESP_PRIV_RPC_VERSION` | proposed RPC version |
| 0x24 | `ESP_PRIV_RPC_EP_REQ` | RPC request endpoint name |
| 0x25 | `ESP_PRIV_RPC_EP_EVT` | RPC event endpoint name |

**Host→CP ACK (V2-only, 0x21–0x26):**

| Tag | Name | Direction |
|-----|------|-----------|
| 0x21 | `ESP_PRIV_HDR_VERSION_ACK` | Host→CP: agreed hdr version |
| 0x23 | `ESP_PRIV_RPC_VERSION_ACK` | Host→CP: agreed RPC version |
| 0x26 | `ESP_PRIV_RPC_EP_ACK` | Host→CP: endpoint ACK |

**Host→CP config (0x44–0x48):**

| Tag | Name |
|-----|------|
| 0x44 | `HOST_CAPABILITIES` |
| 0x45 | `FIRMWARE_CHIP_ID` |
| 0x46 | `TEST_RAW_TP` |
| 0x47 | `RX_HEARTBEAT` |
| 0x48 | `FEATURE_FLAGS` |
<!-- %% sp.cm.ve-ph.tlv.c %% -->

<!-- %% sp.cm.ve-ph.rules.o %% - context -->
## Compat Rules

- Unknown tags: host logs warning, continues (never hard-fail on unknown tag)
- Missing 0x19 / 0x20+ is expected for V1; host stays at V1 header + V1 RPC, no breakage
- `0x22` collision: `event_type=0x22` is a struct field; `ESP_PRIV_RPC_VERSION=0x22` is a TLV tag — different parsing contexts, no runtime collision
<!-- %% sp.cm.ve-ph.rules.c %% -->

<!-- %% sp.cm.ve-ph.tlvgrp.o %% - always -->
## TLV Groups (`eh_tlv` component)

Pack/unpack lives in `modules/common/eh_tlv/` (header-only INTERFACE library).
Group selection is hidden Kconfig auto-derived from RPC version:

| RPC version | Pack groups | Approx size |
|-------------|-------------|-------------|
| V1 Linux    | `eh_tlv_pack_v1_linux` (tags 0x00–0x04)    | ~21 B |
| V1 MCU      | `eh_tlv_pack_v1_mcu`   (tags 0x11–0x17)    | ~27 B |
| V2          | `v1_mcu` + `eh_tlv_pack_v2` (0x19, 0x20, 0x22, 0x24, 0x25) | ~83 B |

Build-time gates: `EH_TLV_V1_LINUX`, `EH_TLV_V1_MCU`, `EH_TLV_V2` (from `eh_tlv_defs.h`).
CP transports (`eh_cp_transport_{sdio,spi,spi_hd,uart}.c`) call these in `generate_startup_event()`.
Host parsers should use `eh_tlv_unpack_v1_linux/v1_mcu/v2` (host migration tracked in todo P3-7).
<!-- %% sp.cm.ve-ph.tlvgrp.c %% -->

<!-- %% sp.cm.ve-ph.c %% -->
