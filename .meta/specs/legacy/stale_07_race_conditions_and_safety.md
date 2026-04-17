# 07 — Race Conditions, Memory Ordering, and Safety Analysis

## 1. Summary Table

| Risk | Severity | Affected Code | Status |
|------|----------|---------------|--------|
| Macro shadows enum (`HOST_CAPABILITIES` 0x01 vs 0x44) | CRITICAL | `esp_hosted_common_tlv.h` | **FIXED** — bad macros removed |
| `ESP_PRIV_EVENT_INIT` not defined in common_tlv.h | CRITICAL | Previously a 0x01 macro | **FIXED** — guarded comment added |
| Missing `ESP_PRIV_TRANS_SDIO_MODE` (0x18) | HIGH | `esp_hosted_caps.h` | **FIXED** — added to enum |
| Missing `ESP_PRIV_FEAT_CAPS` (0x19) parsing in kmod | HIGH | `esp_spi.c`, `esp_sdio.c` | **FIXED** — parsing added |
| `len_left` underflow in TLV loops (uint8 wrap) | HIGH | All 4 TLV parsers | **FIXED** — `(uint16_t)` bounds check |
| `hdr_ver_negotiated` init to 0 instead of V1 | MEDIUM | `cp_core.c` | **FIXED** — initialized to `ESP_HOSTED_HDR_VERSION_V1` |
| Linux kmod SMP: no `WRITE_ONCE` on adapter fields | MEDIUM | `esp_spi.c`, `esp_sdio.c` | **FIXED** — `WRITE_ONCE()` added |
| `ESP_PRIV_RPC_VERSION` and `ESP_PRIV_EVENT_INIT` both = 0x22 | LOW | `esp_hosted_transport.h` | Documented; no runtime collision |
| `volatile` on CP-side vars (Xtensa dual-core) | LOW | `cp_core.c` | Documented; safe for single-byte |
| V2 header TX/RX path not implemented | — | All transports | PENDING-005 (deferred) |

---

## 2. Macro-Shadows-Enum Bug (FIXED)

**Root cause:** The original `esp_hosted_common_tlv.h` defined:
```c
#define HOST_CAPABILITIES  0x01   // WRONG value
```
The actual protocol value from `esp_hosted_transport.h` is:
```c
typedef enum { HOST_CAPABILITIES = 0x44, ... } SLAVE_CONFIG_PRIV_TAG_TYPE;
```
In any translation unit including both headers, the `#define` macro (processed by
the preprocessor before the compiler sees the enum) would shadow the enum constant.
Code writing `*pos = HOST_CAPABILITIES` would write `0x01` instead of `0x44`.
The slave would never recognize the host config frame.

**Fix:** All five `HOST_CAPABILITIES` / `RCVD_ESP_FIRMWARE_CHIP_ID` / `SLV_CONFIG_*`
defines removed from `esp_hosted_common_tlv.h`.  The canonical definition lives
exclusively in `SLAVE_CONFIG_PRIV_TAG_TYPE` (esp_hosted_transport.h).

---

## 3. TLV Parser `len_left` Underflow (FIXED)

**Root cause:** All four TLV parsers had:
```c
uint8_t len_left = len;
while (len_left) {
    tag_len = *(pos + 1);
    // No bounds check here
    pos += (tag_len + 2);
    len_left -= (tag_len + 2);   // ← wraps to 253 if tag_len+2 > len_left
}
```
A malformed or truncated frame with `tag_len + 2 > len_left` causes `len_left`
(uint8_t) to wrap to a large value, iterating far past the buffer end.

**Fix applied to all four parsers:**
```c
if ((uint16_t)tag_len + 2 > len_left) {
    esp_err("TLV truncated: tag=0x%X len=%u left=%u — aborting parse\n",
            *pos, tag_len, len_left);
    break;
}
```
The cast to `uint16_t` prevents addition overflow.  Applied in:
- `esp_hosted_cp_core.c` (`host_to_slave_reconfig`)
- `transport_drv.c` (`process_init_event`, MCU host)
- `esp_spi.c` (`process_init_event`, kmod SPI)
- `esp_sdio.c` (`process_init_event`, kmod SDIO)

---

## 4. `ESP_PRIV_EVENT_INIT` vs `ESP_PRIV_RPC_VERSION` — Same Value (0x22)

These two constants share the numeric value `0x22` but are used in **completely
different byte positions** within the packet:

| Constant | Value | Position in packet |
|----------|-------|--------------------|
| `ESP_PRIV_EVENT_INIT` | `0x22` | `esp_priv_event.event_type` — struct field before TLV stream |
| `ESP_PRIV_RPC_VERSION` | `0x22` | TLV tag byte inside `event_data[]` |

**There is no runtime collision** because the code that checks `event_type` runs
before the TLV scanning loop begins, and the TLV scanner never reads `event_type`.

**Risk remaining:** A programmer might accidentally write:
```c
if (event->event_type == ESP_PRIV_RPC_VERSION)  // compiles, silently wrong
```
This would always be true, since both are 0x22.  The guard comment in
`esp_hosted_common_tlv.h` and the distinct enum type names mitigate this.

**Not changing the value of `ESP_PRIV_RPC_VERSION`** because:
- No shipped code uses it yet (Phase 5 is new).
- Keeping it at 0x22 preserves the implementation as-is.
- A future cleanup pass can renumber to 0x24 with a coordinated change.

---

## 5. `volatile` on CP-Side Negotiation Variables (Xtensa dual-core)

```c
volatile uint8_t hdr_ver_negotiated = ESP_HOSTED_HDR_VERSION_V1;
volatile uint8_t rpc_ver_negotiated = ESP_HOSTED_RPC_VERSION_V1;
```

**Write context:** FreeRTOS task `host_to_slave_reconfig()` (RX task).
**Read context:** Future PENDING-005 TX path (separate TX task, possibly different core).

On the ESP32-S3 (dual-core Xtensa LX7):
- `volatile` prevents compiler caching in a register, ensuring the CPU actually
  reads/writes DRAM on every access.
- Single-byte (`uint8_t`) loads and stores are **atomic** on Xtensa; no torn-write risk.
- The Xtensa memory architecture guarantees that a store by core 0 to a DRAM address
  is visible to core 1 on the next load from that address (cache coherency via
  write-through or cache invalidation, depending on region).

**Current risk:** LOW.  The variables are only *written* by `host_to_slave_reconfig`
(boot-time, once) and *read* by no production code path yet (PENDING-005).

**When PENDING-005 is implemented:** Add an explicit memory barrier after writing:
```c
hdr_ver_negotiated = ESP_HOSTED_HDR_VERSION_V2;
__asm__ volatile("" ::: "memory");  // compiler barrier
// or use esp_idf's portMEMORY_INCOHERENT_STORE() if on shared memory region
```
Or switch to a FreeRTOS event group / semaphore to signal negotiation completion.

---

## 6. Linux Kernel SMP — `WRITE_ONCE` / `READ_ONCE` (FIXED)

**Root cause:** `adapter->hdr_ver_negotiated` and `adapter->rpc_ver_negotiated`
are plain `u8` fields (not `volatile`).  Linux's optimizer may cache them in
a register.  On SMP, a CPU running the interrupt/work-queue write and a CPU
running the data-path read may observe stale values without an explicit barrier.

**Fix applied:** All writes in `esp_spi.c` and `esp_sdio.c` now use:
```c
WRITE_ONCE(adapter->hdr_ver_negotiated, agreed);
WRITE_ONCE(adapter->rpc_ver_negotiated, agreed_rpc);
```
`WRITE_ONCE` emits a `volatile` store + compiler barrier, preventing the compiler
from caching the value and ensuring the write reaches the cache line before any
subsequent load by another CPU.

---

## 7. Startup Event Buffer Overflow Analysis

The CP alloc buffer is `ESP_TRANSPORT_SPI_MAX_BUF_SIZE = 1600 B` (SPI) or
`1536 B` (SDIO).  Maximum startup event TLV bytes (worst case, all TLVs present):

| Block | TLVs | Bytes |
|-------|------|-------|
| esp_payload_header | — | 12 |
| esp_priv_event header | — | 2 |
| Legacy FG TLVs (0x00–0x04) | 5 × (1+1+1) | 15 |
| MCU TLVs 0x11–0x17 | 7 | ~30 |
| SDIO mode (0x18) | 1 | 3 |
| feat_caps (0x19) | 1 | 2+32=34 |
| Phase-3/5 TLVs (0x20–0x23) | 4 × 3 | 12 |
| **Total** | | **~108 B** |

Well within the 1536 B minimum buffer. No overflow risk.

---

## 8. Deferred Risks (PENDING-005)

The following risks do not exist in the current codebase but will arise when
PENDING-005 (V2 header TX/RX encoding) is implemented:

1. **Atomicity of version flag check + header-format switch:** The TX path will
   read `hdr_ver_negotiated` to select V1 vs V2 header.  The flag is set once at
   boot and never changed, so there is no TOCTOU risk in steady state.

2. **RX magic-byte detection:** The RX path must check byte[0] for `0xE9` before
   casting to `esp_payload_header_v2`.  A misaligned or corrupted frame must not
   be blindly cast.  Add: `if (frame[0] == ESP_HDR_V2_MAGIC) { ... }`.

3. **Checksum algorithm change:** V2 header has a wider `checksum_v2` field.  The
   legacy checksum function operates on the V1 12-byte range.  A new function is
   needed for V2 that covers the full 20-byte header.
