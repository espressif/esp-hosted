# 13 — esp_hosted_frame Component

> Wire frame encode/decode — single point of truth for V1/V2 header handling.
> Terminology: **coprocessor / cp** (never "slave").

---

## Purpose

`struct esp_payload_header` is open-coded across 23 transport files (~203 uses).
Every transport independently casts a DMA buffer, reads/writes fields with
manual endian calls, and computes the checksum.  There is no V2 decode path
anywhere in the existing codebase.

`esp_hosted_frame` replaces all of that with a 7-function API.
It is the **only** component that reads or writes V1/V2 wire header structs.
Everything above it works with `interface_buffer_handle_t`. This type is owned by the
frame component, so extensions should include `esp_hosted_frame.h` (not transport
headers) when they need the handle type.

---

## Non-Goals

- Does NOT own `hdr_ver_negotiated`.  The transport layer owns that and calls
  `esp_hosted_frame_init()` when the agreed version changes.
- Does NOT touch protobuf, SLIST registries, or RPC dispatch.
- Does NOT manage DMA buffers — callers allocate; this component writes fields.

---

## Component Location

```
components/common/esp_hosted_frame/
├── CMakeLists.txt
├── include/
│   └── esp_hosted_frame.h
└── src/
    └── esp_hosted_frame.c
```

REQUIRES (CMake): `esp_hosted_common`
No OS includes — usable by CP firmware (IDF/FreeRTOS), MCU host
(IDF/FreeRTOS), Linux kmod (plain C, `__KERNEL__` guards), STM32 bare-metal.

---

## Wire Layouts

**V1** — `struct esp_payload_header`, 12 bytes, `__attribute__((packed))`

| Offset | Field | Description |
|--------|-------|-------------|
| 0 | `if_type:4 \| if_num:4` | Interface type and instance |
| 1 | `flags` | `MORE_FRAGMENT`, wakeup, power-save bits |
| 2–3 | `len` (LE) | Payload byte count |
| 4–5 | `offset` (LE) | Header size (always 12) |
| 6–7 | `checksum` (LE) | 16-bit byte-sum (see below) |
| 8–9 | `seq_num` (LE) | Sequence number |
| 10 | `throttle_cmd:2 \| reserved2:6` | CP→host flow control |
| 11 | union `hci_pkt_type / priv_pkt_type` | Packet sub-type |

If `ESP_PKT_NUM_DEBUG` is defined the struct grows by 2 bytes (14 total);
`sizeof(*hdr)` handles this transparently everywhere.

**V2** — `esp_hosted_header_v2_t`, 20 bytes, `#pragma pack(push,1)`

| Offset | Field | Description |
|--------|-------|-------------|
| 0 | `magic_byte` = 0xE9 | Version probe byte |
| 1 | `hdr_version` = 0x02 | Must be 0x02 for V2 |
| 2–3 | `pkt_num` (LE) | Sequential packet ID |
| 4 | `if_type:6 \| if_num:2` | Interface type and instance |
| 5 | `flags` | Same flag bits as V1 |
| 6 | union `packet_type / reserved_1` | Packet sub-type |
| 7 | `frag_seq_num` | Fragment sequence number |
| 8–9 | `offset` (LE) | Header size (always 20) |
| 10–11 | `len` (LE) | Payload byte count |
| 12–13 | `checksum` (LE) | 16-bit byte-sum |
| 14 | union `tlv_offset / reserved_2` | TLV block offset (0=none) |
| 15–18 | `reserved_3..6` | Must be 0x00 |
| 19 | union `hci_pkt_type / priv_pkt_type` | Packet sub-type |

**Checksum algorithm (both V1 and V2):** plain 16-bit byte accumulation with
natural `uint16_t` truncation on overflow.  This is identical to
`compute_checksum()` in `esp_hosted_transport.h`.  Stored little-endian.
*(The V2 struct comment previously said "XOR checksum" — that was wrong; it
is a 16-bit byte-sum.)*

---

## `interface_buffer_handle_t` — version-transparent buffer handle

Produced by `esp_hosted_frame_decode()`, consumed by upper layers.
The frame component writes: `if_type, if_num, flags, pkt_type, payload_len,`
`seq_num, payload, throttle_cmd, frag_seq, tlv_offset`.
The frame component **never** touches: `priv_buffer_handle, free_buf_handle,
payload_zcopy` — those are set by the transport layer after decode.

```c
typedef struct {
    void     *priv_buffer_handle;   /* transport handle (skb, SDIO handle…) */
    uint8_t   if_type;              /* esp_hosted_if_type_t                  */
    uint8_t   if_num;               /* interface instance (unused, 0)        */
    uint8_t   flags;                /* MORE_FRAGMENT, power-save bits        */
    uint8_t   pkt_type;             /* hci_pkt_type or priv_pkt_type         */
    uint16_t  payload_len;          /* payload bytes only (NOT incl. header) */
    uint16_t  seq_num;              /* V1: seq_num field; V2: pkt_num field  */
    uint8_t  *payload;              /* zero-copy pointer into DMA buffer     */
    uint8_t   throttle_cmd;         /* CP flow-control; 0 on V2 frames       */
    uint8_t   payload_zcopy;        /* transport-private (MCU SPI-HD only)   */
    uint8_t   frag_seq;             /* V2 fragment seq (0 on V1 frames)      */
    uint8_t   tlv_offset;           /* V2 TLV block offset (0 on V1 frames)  */
    void    (*free_buf_handle)(void *priv_buffer_handle);
} interface_buffer_handle_t;
```

**Field rename notes for PENDING-005 migration:**
- Legacy CP/MCU structs use `flag` (singular) → rename to `flags`.
- Legacy CP structs use `wifi_flow_ctrl_en` → rename to `throttle_cmd`.
- Legacy CP transports set `payload_len = len + offset` (total) →
  change to `payload_len = len` (payload only, what this struct carries).

---

## Init Config

`variant` and `role` are **compile-time Kconfig constants** — they are not
fields in `esp_hosted_frame_cfg_t`.  The struct only carries runtime-variable
fields.

```c
typedef struct {
    esp_hosted_transport_t transport;   /* SPI / SDIO / UART / SPI_HD        */
    uint16_t max_buf_size;              /* max valid (len+offset) for xport   */
    uint8_t  hdr_version;              /* V1 or V2                            */
    uint8_t  checksum_enabled;         /* 0 = disabled; 1 = enabled           */
    uint8_t  reserved[2];              /* must be {0, 0}                      */
} esp_hosted_frame_cfg_t;
```

### Helper init macros (always use these, never fill fields manually)

Naming: `ESP_HOSTED_FRAME_CFG_<SIDE>_<HOST_TYPE>_<TRANSPORT>_DEFAULT`

| Macro | Checksum default |
|-------|-----------------|
| `ESP_HOSTED_FRAME_CFG_CP_FG_LINUX_{SPI,SDIO,UART,SPI_HD}_DEFAULT` | 1 |
| `ESP_HOSTED_FRAME_CFG_CP_MCU_{SPI,SDIO,UART,SPI_HD}_DEFAULT` | 1 |
| `ESP_HOSTED_FRAME_CFG_HOST_MCU_{SPI,SDIO,UART,SPI_HD}_DEFAULT` | 1 |
| `ESP_HOSTED_FRAME_CFG_HOST_LINUX_{SPI,SDIO}_DEFAULT` | 0 |

Linux kmod defaults to `checksum_enabled = 0` at boot.  After reading the
`ESP_CHECKSUM_ENABLED` capability bit from the coprocessor PRIV event, re-call
`esp_hosted_frame_init()` with `checksum_enabled = 1`.

All macros set `hdr_version = ESP_HOSTED_HDR_VERSION_V1`.  After the PRIV
handshake confirms V2, re-call `init()` with `hdr_version = V2`.
`init()` is a pure config write — no allocation, safe to call multiple times.

---

## Full API — 7 functions

```c
/* ── Lifecycle ──────────────────────────────────────────────────────────── */

esp_err_t esp_hosted_frame_init(const esp_hosted_frame_cfg_t *cfg);
void      esp_hosted_frame_deinit(void);  /* no-op stub, call for symmetry */

/* ── TX path ────────────────────────────────────────────────────────────── */

/*
 * Write wire header into buf[] using the configured hdr_version.
 * Caller pre-places payload at buf[hdr_size..hdr_size+payload_len-1].
 * Returns header bytes written (12 or 20); 0 on NULL arg.
 * h->throttle_cmd → V1 throttle_cmd:2 field; ignored on V2 (no wire field).
 */
uint8_t esp_hosted_frame_encode(uint8_t *buf,
                                const interface_buffer_handle_t *h,
                                uint16_t payload_len);

/*
 * Write dummy/idle header: if_type=ESP_MAX_IF, if_num=0xF, len=0.
 * throttle_cmd piggybacked (V1 only; ignored on V2).
 * Returns 12 (V1) or 20 (V2); 0 on NULL buf.
 */
uint8_t esp_hosted_frame_encode_dummy(uint8_t *buf, uint8_t throttle_cmd);

/* Header byte count for configured version (12 or 20). */
uint8_t esp_hosted_frame_hdr_size(void);

/* Header byte count for specific version without needing an init(). */
static inline uint8_t esp_hosted_frame_hdr_size_for_ver(uint8_t version);

/* ── RX path ────────────────────────────────────────────────────────────── */

/*
 * Parse inbound DMA buffer. Returns one of:
 *   ESP_HOSTED_FRAME_OK      — valid, h is populated, process it
 *   ESP_HOSTED_FRAME_DUMMY   — idle/no-data frame, discard silently
 *   ESP_HOSTED_FRAME_CORRUPT — checksum mismatch, log and drop
 *   ESP_HOSTED_FRAME_TOOBIG  — len+offset > max_buf_size, log and drop
 *   ESP_HOSTED_FRAME_INVALID — NULL/short buf, bad offset, bad V2 hdr_version
 *
 * Version auto-detection: buf[0]==0xE9 → V2 path, else → V1 path.
 * (Independent of cfg.hdr_version — a V1-configured node can receive a V2
 * frame and decode it correctly.  This is the safe degraded mode.)
 *
 * Validation order:
 *   1. buf!=NULL, h!=NULL, buf_len>=1           → INVALID on fail
 *   2. buf_len >= hdr_size for detected version → INVALID on fail
 *   3. V2 only: hdr->hdr_version == 0x02        → INVALID on fail
 *   4. if_type==ESP_MAX_IF OR len==0             → DUMMY (silent)
 *   5. offset != hdr_size                        → INVALID
 *   6. len+offset > cfg.max_buf_size             → TOOBIG
 *   7. buf_len < len+offset                      → INVALID
 *   8. checksum mismatch (if checksum_enabled)   → CORRUPT
 *
 * h->payload       — points into buf (zero-copy, no memcpy)
 * h->payload_len   — payload bytes only
 * h->priv_buffer_handle, h->free_buf_handle, h->payload_zcopy — NOT set;
 *                    caller (transport layer) sets these after decode returns.
 */
esp_hosted_frame_result_t esp_hosted_frame_decode(const uint8_t *buf,
                                                   uint16_t buf_len,
                                                   interface_buffer_handle_t *h);

/* Probe version from byte[0] only. Returns V1 or V2. No validation. */
uint8_t esp_hosted_frame_detect_version(const uint8_t *buf, uint16_t buf_len);
```

---

## Checksum Verification — Const-Safe Implementation

We must not mutate `const uint8_t *buf` to zero the checksum field before
computing (UB when buf points into a DMA-mapped kernel page).

Instead:
```
raw  = frame_checksum(buf, total)          /* sum including stored checksum bytes */
calc = raw - stored_lo - stored_hi         /* mathematically equivalent to zeroing first */
if (_to_le16(calc) != stored) → CORRUPT
```
This is exact — proven by `sum_with_zero + lo + hi == sum_without_zeroing`.

---

## Thread Safety

`s_cfg` is a plain global with no lock.  Callers must ensure `init()` completes
(negotiation task) before concurrent `encode()/decode()` (TX/RX tasks) use it.
In all known deployments, `init()` runs during transport open and once after
the PRIV handshake, both sequenced before data frames flow — so the lack of
a mutex is safe in practice.

---

## Dummy Frame Protocol

CP SPI is the only transport that uses dummy frames (SDIO, UART, SPI-HD are
interrupt/event driven and do not need them).

| Version | Wire content |
|---------|--------------|
| V1 | `if_type=ESP_MAX_IF`, `if_num=0xF`, `len=0`, rest=0 |
| V2 | `magic=0xE9`, `hdr_version=0x02`, `if_type=ESP_MAX_IF`, rest=0 |

`esp_hosted_frame_decode()` returns `DUMMY` on either: `if_type==ESP_MAX_IF`
or `len==0` — both checked before the offset/TOOBIG/checksum steps.

**Pre-existing CP SPI bug (PENDING-005):** In `get_next_tx_buffer()` with
`USE_STATIC_DUMMY_BUFFER=1`, `throttle_cmd` is never written to the static
dummy buffer — `sendbuf` is NULL at that point (latent UB, compiler eliminates
the dead store).  The static buffer's `throttle_cmd` stays 0 from init.
This will be fixed when PENDING-005 migrates the transport to call
`esp_hosted_frame_encode_dummy(dummy_buffer, find_wifi_tx_throttling_to_be_set())`.

---

## Test Harness

`/tmp/frame_test.c` — compile and run:

```sh
gcc -std=c11 -Wall -Wextra -Wpedantic -Wshadow \
  -I components/common/esp_hosted_common/include \
  -I components/common/esp_hosted_frame/include \
  -o /tmp/frame_test_bin \
  components/common/esp_hosted_frame/src/esp_hosted_frame.c \
  /tmp/frame_test.c
/tmp/frame_test_bin
```

**Coverage (92 tests, 0 failures, 0 warnings as of review-pass-2):**

| Group | Tests |
|-------|-------|
| init() validation | null cfg, bad version, zero bufsize, bad transport, valid |
| hdr_size | V1=12, V2=20, `hdr_size_for_ver` V1/V2/unknown |
| V1 encode+decode | full roundtrip, cksum-on/off, corrupt detection, all field round-trips |
| V2 encode+decode | full roundtrip, cksum-on/off, corrupt detection, bad hdr_version, all field round-trips |
| Auto-detect | V2 frame decoded correctly when cfg says V1 |
| Dummy frames | V1+V2 encode+decode+null-buf |
| Edge cases | TOOBIG V1+V2, bad offset V1+V2, null/short inputs, exact buf_len |
| Boundary | if_type=ESP_MAX_IF→DUMMY, if_type=5 (PRIV_IF)→OK, len=0→DUMMY |
| Re-init | V1→V2 switch, hdr_size and encode version both change |

---

## Backward Compatibility

| Scenario | Behaviour |
|----------|-----------|
| Old CP + old host | All V1; decode never sees 0xE9 magic |
| New CP + old host | CP defaults V1; no V2 ACK from host; stays V1 |
| New CP + new host, V2 negotiated | Both re-init with V2 after handshake |
| Un-migrated transport files | Still compile; direct struct casts valid for V1 |

Migration is incremental: each of the 23 transport files can be moved to
`encode()/decode()` independently.  V2 only activates after both sides
negotiate it.

---

## Pending Items

| ID | Description |
|----|-------------|
| **PENDING-005** | Migrate all 23 transport files to `esp_hosted_frame` encode/decode |
| NEXT-B | ~~Update specs/13_frame_component.md~~ **Done (this file)** |
