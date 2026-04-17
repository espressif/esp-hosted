# Wire Protocol — Payload Headers

## V1 Header (12 bytes, legacy)

All existing hosts and coprocessors use this format.

```
Byte  Field            Width  Description
────  ─────────────    ─────  ──────────────────────────────────────────
 0    if_type:6        6 bit  Interface type (esp_hosted_if_type_t)
 0    if_num:2         2 bit  Interface instance (unused, set to 0)
 1    flags            1 B    ESP_HOSTED_FLAG_* bits
 2    packet_type      1 B    ESP_PACKET_TYPE_* or reserved
 3    reserved1        1 B    must be 0x00
 4-5  len              2 B    Payload length in bytes (little-endian)
 6-7  offset           2 B    Byte offset to payload within frame (little-endian)
 8-9  checksum         2 B    Sum of all bytes in (header + payload) (little-endian)
10    priv_pkt_type    1 B    Packet type for priv/HCI interface
11    throttle_cmd     1 B    Flow control command
```

Struct: `struct esp_payload_header` in `esp_hosted_fg/common/esp_hosted_header.h`
(shim → `components/common/esp_hosted_common/include/esp_hosted_common_header.h`)

## V2 Header (20 bytes)

Negotiated at boot time. Only used after both sides agree via PRIV TLV handshake.

```
Byte  Field            Width  Description
────  ─────────────    ─────  ──────────────────────────────────────────
 0    magic_byte        1 B   Always 0xE9 — identifies V2 frame
 1    hdr_version       1 B   0x02 for V2; used to gracefully fall back to V1
 2-3  pkt_num          2 B   Sequential packet ID, little-endian (loss detection)
 4    if_type:6        6 bit  Interface type (esp_hosted_if_type_t)
 4    if_num:2         2 bit  Interface instance (unused, 0)
 5    flags             1 B   ESP_HOSTED_FLAG_* bits
 6    packet_type       1 B   ESP_PACKET_TYPE_*
 7    frag_seq_num      1 B   Fragment sequence number
 8-9  offset           2 B   Byte offset to payload within frame (little-endian)
10-11 len              2 B   Payload length in bytes (little-endian)
12-13 checksum         2 B   XOR checksum over header + payload (little-endian)
14    tlv_offset        1 B   Byte offset to optional per-frame TLV block (0 = none)
15-18 reserved[4]      4 B   Must be 0x00
19    hci/priv_pkt_type 1 B  Packet type for HCI or priv interface
```

Struct: `esp_hosted_header_v2_t` in `esp_hosted_common_header_v2.h`
Compile-time assert: `sizeof(esp_hosted_header_v2_t) == 20`

### V2 Magic and Identification

- Byte[0] = `0xE9` (ESP_HOSTED_HDR_V2_MAGIC) uniquely identifies V2 frames
- V1 frames have `if_type:6` at byte[0] which is max 0x3F — never 0xE9
- RX path (PENDING-005): inspect byte[0]; if `== 0xE9` use V2 parsing, else V1

## Frame Checksum

V1: `sum of all bytes in (header[12] + payload[len])`, stored at header[8-9]
V2: same algorithm covering `header[20] + payload[len]`

The `compute_checksum()` inline is in `esp_hosted_transport.h` (CP-side) and
the equivalent MCU transport header.

## Constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `ESP_HOSTED_HDR_V2_MAGIC` | 0xE9 | V2 frame magic byte |
| `ESP_HOSTED_HDR_VERSION_V1` | 0x01 | Version field for V1 |
| `ESP_HOSTED_HDR_VERSION_V2` | 0x02 | Version field for V2 |
| `H_ESP_PAYLOAD_HEADER_OFFSET` | 12 | sizeof V1 header |
| `H_ESP_PAYLOAD_HEADER_V2_OFFSET` | 20 | sizeof V2 header |
