# 03 — PRIV Boot-Time Handshake & Complete TLV Registry

## 1. Overview

At slave power-up, the coprocessor (CP) sends a single `ESP_PRIV_EVENT_INIT` packet
containing a flat TLV stream inside `esp_priv_event.event_data[]`.  The host parses
this stream to learn the slave's capabilities and negotiate wire-header and RPC versions.

The host replies with an `ESP_PRIV_EVENT_INIT_ACK` packet whose TLV stream carries
the agreed versions back to the slave plus the host's own capability advertisement.

---

## 2. Packet Structure

```
┌────────────────────────────────────────────┐  ← esp_payload_header (12 B V1 / 20 B V2)
│  if_type = ESP_PRIV_IF                     │
│  priv_pkt_type = ESP_PACKET_TYPE_EVENT (0x33) │
├────────────────────────────────────────────┤  ← struct esp_priv_event
│  event_type = ESP_PRIV_EVENT_INIT  (0x22)  │   ← NOT a TLV tag — this is a struct field
│  event_len  = <total TLV bytes>            │
├────────────────────────────────────────────┤  ← event_data[0..event_len-1]
│  [tag:1][len:1][value:len]                 │   ← TLV stream (slave→host direction)
│  [tag:1][len:1][value:len]                 │
│  ...                                       │
└────────────────────────────────────────────┘
```

**Field disambiguation — the value 0x22 appears in two different positions:**

| Field | Value | Meaning |
|-------|-------|---------|
| `esp_priv_event.event_type` | `0x22` | Event is `ESP_PRIV_EVENT_INIT` — not a TLV tag |
| TLV tag byte in `event_data[]` | `0x22` | `ESP_PRIV_RPC_VERSION` TLV |

These occupy different byte offsets.  There is **no runtime collision**.
See [07_race_conditions_and_safety.md §4](07_race_conditions_and_safety.md) for the
naming-confusion risk and the guard in `esp_hosted_common_tlv.h`.

---

## 3. Complete Slave → Host TLV Registry (startup event)

All values authoritative.  Do not renumber existing entries.

### 3.1 Legacy Linux-FG style  (0x00–0x04)

Source: original `esp-hosted` public repository (kernel module era).
**Deprecated** — slaves should no longer emit these; hosts must still accept them.

| Tag | Name | Size | Direction | Notes |
|-----|------|------|-----------|-------|
| `0x00` | `ESP_LINUX_FG_PRIV_CAPABILITY` | 1 B | slave→host | Basic caps bitmap. Deprecated. |
| `0x01` | `ESP_LINUX_FG_PRIV_SPI_CLK_MHZ` | 1 B | slave→host | SPI clock MHz. Deprecated. |
| `0x02` | `ESP_LINUX_FG_PRIV_FIRMWARE_CHIP_ID` | 1 B | slave→host | Chip ID. Deprecated. |
| `0x03` | `ESP_LINUX_FG_PRIV_TEST_RAW_TP` | 1 B | slave→host | Raw TP flags. Deprecated. |
| `0x04` | `ESP_LINUX_FG_PRIV_FW_DATA` | N B | slave→host | `struct fw_version`. Deprecated. |

Defined in: `esp_hosted_caps.h` (`ESP_LINUX_FG_PRIV_TAG_TYPE` enum).

### 3.2 MCU style  (0x11–0x19)

Source: `esp-hosted-mcu` public repository + FG coprocessor alignment.
**Current standard** — all new slaves emit these.

| Tag | Name | Size | Direction | Notes |
|-----|------|------|-----------|-------|
| `0x11` | `ESP_PRIV_CAPABILITY` | 1 B | slave→host | `ESP_CAPABILITIES` bitfield |
| `0x12` | `ESP_PRIV_FIRMWARE_CHIP_ID` | 1 B | slave→host | `CONFIG_IDF_FIRMWARE_CHIP_ID` |
| `0x13` | `ESP_PRIV_TEST_RAW_TP` | 1 B | slave→host | `ESP_RAW_TP_MEASUREMENT` flags |
| `0x14` | `ESP_PRIV_RX_Q_SIZE` | 1 B | slave→host | Transport RX queue depth |
| `0x15` | `ESP_PRIV_TX_Q_SIZE` | 1 B | slave→host | Transport TX queue depth |
| `0x16` | `ESP_PRIV_CAP_EXT` | 4 B LE | slave→host | `ESP_EXTENDED_CAPABILITIES` uint32 |
| `0x17` | `ESP_PRIV_FIRMWARE_VERSION` | 4 B LE | slave→host | Packed `major<<16|minor<<8|patch` |
| `0x18` | `ESP_PRIV_TRANS_SDIO_MODE` | 1 B | slave→host | SDIO sub-mode (MCU repo only) |
| `0x19` | `ESP_PRIV_FEAT_CAPS` | 32 B LE | slave→host | `feat_caps[8]` uint32 array |

Defined in: `esp_hosted_caps.h` (`ESP_PRIV_TAG_TYPE` enum) and mirrored as
`#define` macros (with `#ifndef` guard) in `esp_hosted_common_tlv.h` for use
by kernel-module code that cannot include IDF-style enum headers.

**`feat_caps[8]` encoding** — each of the 8 uint32 words is a bitmask.  Bit
assignments are application-defined.  All zeros means "no feature bits set".
The TLV is omitted entirely if all words are zero.

### 3.3 New negotiation TLVs  (0x20–0x2F)

Added in Phase 3 (wire header) and Phase 5 (RPC version).
Both directions (slave→host in startup; host→slave in ACK frame).

| Tag | Name | Size | Dir | Notes |
|-----|------|------|-----|-------|
| `0x20` | `ESP_PRIV_HEADER_VERSION` | 1 B | slave→host | Proposed wire-header version |
| `0x21` | `ESP_PRIV_HEADER_VERSION_ACK` | 1 B | host→slave | Agreed wire-header version |
| `0x22` | `ESP_PRIV_RPC_VERSION` | 1 B | slave→host | Proposed RPC protocol version |
| `0x23` | `ESP_PRIV_RPC_VERSION_ACK` | 1 B | host→slave | Agreed RPC protocol version |
| `0x24`–`0x2F` | — | — | — | Reserved for future negotiation TLVs |

Wire-header version codes: `0x01` = V1 (12 B), `0x02` = V2 (20 B, magic `0xE9`).
RPC version codes: `0x01` = V1 (protobuf/protocomm), `0x02` = V2 (msg_id dispatch).

Defined in: `esp_hosted_common_tlv.h` (macros) and `esp_hosted_transport.h` CP-side
(`ESP_PRIV_TLV_TYPE` enum).

---

## 4. Host → Slave TLV Registry (ACK / config frame)

These tag bytes appear in a **separate packet** the host sends back to the slave.
`priv_pkt_type = ESP_PACKET_TYPE_EVENT (0x33)` with
`event_type = ESP_PRIV_EVENT_INIT_ACK`.

| Tag | Name | Value | Notes |
|-----|------|-------|-------|
| `0x44` | `HOST_CAPABILITIES` | uint8 | Host basic cap bits |
| `0x45` | `RCVD_ESP_FIRMWARE_CHIP_ID` | uint8 | Echo of slave chip ID |
| `0x46` | `SLV_CONFIG_TEST_RAW_TP` | uint8 | Raw TP test command |
| `0x47` | `SLV_CONFIG_THROTTLE_HIGH_THRESHOLD` | uint8 | Flow-ctrl high watermark |
| `0x48` | `SLV_CONFIG_THROTTLE_LOW_THRESHOLD` | uint8 | Flow-ctrl low watermark |
| `0x21` | `ESP_PRIV_HEADER_VERSION_ACK` | uint8 | Agreed wire-hdr version |
| `0x23` | `ESP_PRIV_RPC_VERSION_ACK` | uint8 | Agreed RPC version |

⚠️ **Critical naming note:** `HOST_CAPABILITIES` / `RCVD_ESP_FIRMWARE_CHIP_ID` /
`SLV_CONFIG_*` are **enum constants** in `SLAVE_CONFIG_PRIV_TAG_TYPE` in
`esp_hosted_transport.h` with values `0x44`–`0x48`.  They are **not** defined
in `esp_hosted_common_tlv.h`.  Adding them there with any different value would
silently shadow the enum in any translation unit including both headers.

---

## 5. Namespace Map (visual)

```
0x00–0x04  Legacy FG Linux slave→host TLVs (deprecated)
0x05–0x10  UNALLOCATED — do not use
0x11–0x19  MCU-style slave→host TLVs (current)
0x1A–0x1F  RESERVED — gap before negotiation block
0x20–0x23  Phase-3/5 negotiation TLVs (bidirectional)
0x24–0x2F  Reserved for future negotiation TLVs
0x30–0x43  UNALLOCATED
0x44–0x48  Host→slave config TLVs (SLAVE_CONFIG_PRIV_TAG_TYPE)
0x49–0xFF  UNALLOCATED

Special field values (NOT TLV tags):
  0x22  ESP_PRIV_EVENT_INIT   — event_type field in struct esp_priv_event
  0x33  ESP_PACKET_TYPE_EVENT — priv_pkt_type field in esp_payload_header
```

---

## 6. Backward Compatibility Rules

1. A **new slave** (emitting 0x11–0x19 + 0x20–0x23) communicating with an
   **old FG-Linux host** (knows only 0x00–0x04): host logs "Unsupported tag"
   for all unknown tags but continues; slave gets no ACK for 0x21/0x23 and
   remains at V1 header / V1 RPC. ✓ Safe.

2. An **old slave** (emitting only 0x00–0x04) communicating with a **new MCU host**:
   host parses the legacy tags with deprecation warnings; negotiation TLVs absent,
   so host stays at V1. ✓ Safe.

3. A **new slave** communicating with a **new MCU host** (FreeRTOS `transport_drv.c`):
   full handshake, V2 wire header and V2 RPC may be negotiated.

4. A **new slave** communicating with a **new kmod host** (`esp_spi.c`/`esp_sdio.c`):
   full handshake, `adapter->hdr_ver_negotiated` and `adapter->rpc_ver_negotiated`
   updated; V2 TX/RX encoding deferred (PENDING-005).
