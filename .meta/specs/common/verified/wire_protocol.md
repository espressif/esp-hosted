<!-- %% sp.cm.ve-wp.o %% - always -->
---
type: spec
last_verified: 2026-03-30
---

# Wire Protocol — Payload Headers

<!-- %% sp.cm.ve-wp.v1.o %% - always -->
## V1 Header (12 bytes)

```
Byte  Field           Width  Notes
  0   if_type:6       6 bit  eh_if_type_t
  0   if_num:2        2 bit  instance (set 0)
  1   flags           1 B    ESP_HOSTED_FLAG_* bits
  2   packet_type     1 B    ESP_PACKET_TYPE_*
  3   reserved1       1 B    0x00
4–5   len             2 B    payload length (LE)
6–7   offset          2 B    payload offset in frame (LE)
8–9   checksum        2 B    sum of all header+payload bytes (LE)
 10   priv_pkt_type   1 B    PRIV/HCI packet type
 11   throttle_cmd    1 B    flow control
```

`struct esp_payload_header` → `fg/common/eh_header.h`
<!-- %% sp.cm.ve-wp.v1.c %% -->

<!-- %% sp.cm.ve-wp.v2.o %% - always -->
## V2 Header (20 bytes)

Byte 0 = `0xE9` (magic) discriminates V2 from V1. Only active after PRIV TLV negotiation.

```
Byte  Field           Width  Notes
  0   magic           1 B    0xE9
  1   hdr_version     1 B    0x02
  2   if_type:6       6 bit
  2   if_num:2        2 bit
  3   flags           1 B
  4   packet_type     1 B
  5   reserved1       1 B    0x00
6–7   len             2 B    LE
8–9   offset          2 B    LE
10–11 checksum        2 B    LE
 12   priv_pkt_type   1 B
 13   throttle_cmd    1 B
14–19 reserved2       6 B    0x00
```

Discriminator logic: `frame[0] == 0xE9` → V2; else V1.
<!-- %% sp.cm.ve-wp.v2.c %% -->

<!-- %% sp.cm.ve-wp.const.o %% - context -->
## Key Constants

| Constant | Value | File |
|----------|-------|------|
| `ESP_HOSTED_HDR_VERSION_V1` | 0x01 | `eh_header.h` |
| `ESP_HOSTED_HDR_VERSION_V2` | 0x02 | `eh_header.h` |
| `ESP_HOSTED_HDR_V2_MAGIC` | 0xE9 | `eh_header.h` |
| `ESP_PACKET_TYPE_EVENT` | 0x33 | `eh_header.h` |
| `ESP_PRIV_IF` | — | `eh_if_type_t` enum |

Checksum: 16-bit sum of every byte in (header + payload), stored LE at bytes 8–9.
<!-- %% sp.cm.ve-wp.const.c %% -->

<!-- %% sp.cm.ve-wp.c %% -->
