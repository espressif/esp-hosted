# 10 — Full System End-to-End Review
<!-- Last updated: 2026-03-12. Authoritative living doc — update on every finding. -->
<!-- Decision 9 applied: SLIST → realloc-table. Sections 4.3, 4.5, 6.3 updated accordingly. -->

## How to Read This Document

Each section covers one system area. Findings are tagged:
- 🔴 **BUG** — compile error or silent runtime breakage
- 🟠 **RISK** — latent issue that activates in a specific scenario
- 🟡 **GAP** — feature absent or pending that leaves a flow incomplete
- 🟢 **OK** — verified correct
- ⚪ **NG-PENDING** — out of scope for current phase; needed for NG variant

Variants in scope:
- **FG** = Linux FG host (kernel module) + ESP coprocessor
- **MCU** = MCU host (IDF FreeRTOS) + ESP coprocessor
- **NG** = Linux NG host — out of scope for now, items flagged

---

## 1. Build System (CMake / Kconfig)

### 1.1 esp_hosted_transport_cp — missing `esp_hosted_common` in REQUIRES
🔴 **BUG — will fail to build**

`CMakeLists.txt` for `esp_hosted_transport_cp`:
```
REQUIRES "esp_timer" ${public_requires}
```
No `esp_hosted_common` listed. Yet all four transport sources include
`esp_hosted_tlv.h` (the shim), which forward-includes
`esp_hosted_common_tlv.h` from `components/common/esp_hosted_common/include/`.
That canonical dir is NOT in `esp_hosted_transport_cp`'s `INCLUDE_DIRS`.
In IDF, include paths do NOT flow from dependent→dependency; they flow from
dependency→dependent. So `esp_hosted_cp` having the canonical path does not
help `esp_hosted_transport_cp` at compile time.

**Fix:**
```cmake
REQUIRES "esp_timer" "esp_hosted_common" ${public_requires}
```

### 1.2 MCU host CMakeLists — `CONFIG_IDF_TARGET` always true in IDF
🟠 **RISK — Linux kmod path never builds via CMake (by design, but confusing)**

```cmake
if(CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU OR CONFIG_IDF_TARGET)
```
`CONFIG_IDF_TARGET` is always set in any IDF build (it equals `"esp32s3"` etc.),
so the MCU source list is always included even if `CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU`
is not selected. For now this is harmless because the Linux kmod is built via
Makefile only, but it means the Kconfig choice has no effect on what sources get
compiled. Use only `CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU` for the guard.

**Fix:**
```cmake
if(CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU)
```

### 1.3 kmod Makefile — include paths correct
🟢 **OK**

`EXTRA_CFLAGS` includes all of:
- `COMMON_TRANSPORT_INC` (esp_hosted_transport_cp/include/common)
- `TRANSPORT_HOST_INC` (eh_host_transport/include)
- `CANONICAL_COMMON_INC` (components/common/esp_hosted_common/include)
- `COMMON_DIR` (esp_hosted_fg/common shim dir)
- `MAKEFILE_DIR/include` and subdirs

All needed headers are reachable.

### 1.4 MCU extension CMakeLists — REQUIRES correct
🟢 **OK**

`esp_hosted_ext_rpc_mcu` lists:
```
REQUIRES esp_hosted_cp esp_hosted_common esp_hosted_ext_rpc_mcu_pbuf esp_event
```
Transport dependency flows through `esp_hosted_cp` → `esp_hosted_transport_cp`. ✓

### 1.5 Linux FG extension CMakeLists — needs verification
🟡 **GAP**

`esp_hosted_ext_rpc_linux_fg/CMakeLists.txt` was not fully read; needs audit
for `esp_hosted_common` and `esp_hosted_transport_cp` in REQUIRES.

### 1.6 Kconfig: host platform selection asymmetry
🟡 **GAP**

CP-side Kconfig has:
```
config ESP_HOSTED_CP_FOR_LINUX__802_03   # selects Linux FG host
config ESP_HOSTED_CP_FOR_MCU        # selects MCU host
```
Host-side Kconfig has:
```
config ESP_HOSTED_HOST_PLATFORM_MCU  # MCU host
config ESP_HOSTED_HOST_PLATFORM_LINUX
```
The naming differs (CP_TYPE vs HOST_PLATFORM). There is no single "variant"
Kconfig that selects both simultaneously. A user must set both CP and host
Kconfigs consistently. Risk of misconfiguration (e.g., CP built for MCU,
host built for Linux). Recommend a top-level `ESP_HOSTED_VARIANT` Kconfig
that sets both downstream.

### 1.7 Kconfig: transport selection dependency chain
🟢 **OK** (with caveat)

`ESP_HOSTED_TRANSPORT_CP_SPI_HD` and `ESP_HOSTED_TRANSPORT_CP_UART` correctly
`depends on ESP_HOSTED_CP_FOR_MCU`. FG variants are constrained to SPI/SDIO. ✓
However there is no Kconfig constraint preventing a user from selecting
`ESP_HOSTED_TRANSPORT_CP_SPI_HD` for an FG Linux host (not yet guarded on host side).

---

## 2. Boot-Time PRIV Handshake (TLV at Init)

### 2.1 CP startup event — all four transports
🟢 **OK**

All four CP transports (SPI, SDIO, SPI-HD, UART) call `generate_startup_event()`
with caps, ext_caps, and feat_caps. The TLV stream emits:
- Legacy FG TLVs (chip ID at 0x02, raw TP at 0x03 — using MCU-style 0x12/0x13 now)
- MCU TLVs 0x11–0x17 (capability, chip_id, raw_tp, q sizes, cap_ext, fw_ver)
- `ESP_PRIV_FEAT_CAPS` (0x19) if any feat_caps word non-zero
- `ESP_PRIV_HEADER_VERSION` (0x20) = 0x02 (V2 proposed)
- `ESP_PRIV_RPC_VERSION` (0x22) = 0x02 (V2 proposed)

Total TLV payload ≈ 108 bytes — well within 1536-byte buffer minimum. ✓

### 2.2 Host → Slave ACK (MCU host path)
🟢 **OK** (structure), 🟠 **RISK** (one subtle issue)

`send_slave_config()` in `transport_drv.c` constructs an ACK event with:
- `HOST_CAPABILITIES` (0x44), `RCVD_ESP_FIRMWARE_CHIP_ID` (0x45)
- `SLV_CONFIG_TEST_RAW_TP` (0x46), throttle thresholds (0x47, 0x48)
- `ESP_PRIV_HEADER_VERSION_ACK` (0x21) = `host_hdr_ver_agreed`
- `ESP_PRIV_RPC_VERSION_ACK` (0x23) = `host_rpc_ver_agreed`

This is sent via `esp_hosted_transport_send_buffer(ESP_PRIV_IF, ...)`. ✓

🟠 **RISK**: `send_slave_config` casts `sendbuf` directly to `struct esp_priv_event*`
without an `esp_payload_header` prefix:
```c
event = (struct esp_priv_event *) (sendbuf);  // no header space reserved!
```
This is intentional only if `esp_hosted_transport_send_buffer` adds the header
automatically for `ESP_PRIV_IF`. That transport-layer behavior must be verified —
if the send function expects `payload` to start at the data (not the header),
this is correct. If the send function writes the header INTO `sendbuf`, the first
2 bytes of `event_type` are overwritten. Needs explicit confirmation in transport
send implementation for PRIV packets. **PENDING verification.**

### 2.3 Host → Slave ACK (kmod path)
🟡 **GAP**

The kmod SPI/SDIO parsers parse 0x20 and 0x22 TLVs, compute agreed versions, and
write to `adapter->hdr_ver_negotiated` / `adapter->rpc_ver_negotiated`. But the
ACK frame (0x21 and 0x23 back to slave) is **never sent** from the kmod path.
The slave's `host_to_slave_reconfig()` waits for `ESP_PRIV_HEADER_VERSION_ACK`
(0x21) TLV to commit V2 mode. If that ACK never arrives, `hdr_ver_negotiated`
stays at V1 on the CP side. For the kmod path, V2 wire headers will never be
enabled even after PENDING-005 is done.

**Fix needed:** kmod must send a PRIV ACK packet (same structure as MCU's
`send_slave_config()`) after parsing the startup event.

### 2.4 TLV parser bounds check — all paths
🟢 **OK** (fixed in session 6)

All four parsers have `if ((uint16_t)tag_len + 2 > len_left) break;`. ✓

### 2.5 Capability accumulation flow
🟢 **OK** (MCU), 🟡 **GAP** (FG)

MCU extension calls:
```c
esp_hosted_cp_add_cap_bits(0, ESP_WLAN_SUPPORT);
esp_hosted_cp_add_feature_cap_bits(ESP_HOSTED_FEAT_IDX_WIFI, 0x1u);
```
These accumulate into Registry 2 before `host_reset_task` calls
`generate_startup_event()` which reads them via `esp_hosted_cp_get_caps()`. ✓

Linux FG extension (`esp_hosted_ext_rpc_linux_fg_core.c` line 29):
> "Capability bits for Linux FG extension are NOT yet accumulated into Registry 2."

So for FG host builds, `ESP_WLAN_SUPPORT`, `ESP_BT_INTERFACE_SUPPORT` etc.
are NOT set in the startup TLV's capability field. The host will see all
capability bits as 0. **Needs explicit `add_cap_bits()` calls in FG extension init.**

---

## 3. Interface RX/TX Registry (Registry 1)

### 3.1 Registration API
🟢 **OK**

`esp_hosted_cp_register_rx_cb(iface_type, rx_cb, ctx)` checks:
- iface_type bounds
- NULL handler guard
- Re-registration guard (STA/AP are allowed to override; all others blocked)

`g_iface_table[ESP_IF_TYPE_MAX]` is zero-initialized (BSS). ✓

### 3.2 Dispatch path — CP RX
🟠 **RISK — dispatch bypasses Registry 1 for most interfaces**

In `esp_hosted_cp_core.c → process_rx_pkt()`:
```c
if (buf_handle->if_type == ESP_PRIV_IF) {
    process_priv_pkt(payload, payload_len);
} else if (buf_handle->if_type == ESP_STA_IF && station_connected) {
    esp_wifi_internal_tx(WIFI_IF_STA, payload, payload_len);
} else if (buf_handle->if_type == ESP_AP_IF && softap_started) {
    esp_wifi_internal_tx(WIFI_IF_AP, payload, payload_len);
} else if (buf_handle->if_type == ESP_SERIAL_IF) {
    esp_hosted_cp_process_serial_rx_pkt(buf_handle->payload);
} else if (buf_handle->if_type == ESP_HCI_IF) {
    process_hci_rx_pkt(payload, payload_len);
}
```
`esp_hosted_cp_dispatch_rx()` (the Registry 1 dispatcher) is **not called** from
`process_rx_pkt`. The `g_iface_table` is populated but never consulted during RX.
This means extensions that register a custom RX handler via
`esp_hosted_cp_register_rx_cb()` will never have their handler invoked.

**Fix required:** Replace the hardcoded `if/else` chain in `process_rx_pkt` with:
```c
esp_err_t r = esp_hosted_cp_dispatch_rx(buf_handle->if_type,
                                         payload, payload_len, eb);
```
with per-interface default handlers pre-registered at init for PRIV_IF, SERIAL_IF,
HCI_IF. Or keep PRIV_IF and SERIAL_IF as hardcoded special cases (they are
not extension-overridable) and route only STA/AP/HCI through the registry.

### 3.3 TX registry — state
🟡 **GAP**

`esp_hosted_cp_register_tx_cb()` and `g_iface_table[i].tx` exist in the API
and structure, but `process_tx_pkt()` in `esp_hosted_cp_core.c` does not consult
`g_iface_table[i].tx`. TX overrides are silently ignored.

### 3.4 ESP_STA_IF guard: `station_connected` flag
🟠 **RISK**

Packets for `ESP_STA_IF` are dropped (no route) if `station_connected == false`,
even if an RX handler has been registered. There is no fallback to the registry
for an extension that wants to intercept STA frames before association. This is
probably intentional (no point forwarding if STA not up) but the interaction
with the registry is undocumented.

---

## 4. RPC Flow (Serial TLV → Protocomm → SLIST Dispatch)

### 4.1 Serial TLV framing (cp_rpc_ll.c)
🟢 **OK**

Protocomm `pserial` layer parses the serial TLV frame:
```
[type:1][len_le16:2][epname]  [type:1][len_le16:2][data]
```
Endpoint name is extracted; if epname == "RPCReq", protocomm_req_handle routes
to `rpc_ll_slist_req_handler`. The response is re-wrapped in the same TLV
format and transmitted. ✓

### 4.2 msg_id extraction (proto field-2 scanner)
🟢 **OK** (with one edge case)

`esp_hosted_proto_extract_msg_id()` scans only field 2 (wire type 0 = varint).
All `.proto` files place `msg_id` at field 2 by convention.

🟠 **RISK**: If a proto message has field 1 encoded as a LEN type with a
very large length, the scanner's varint skip for field 1 could overrun
`uint16_t i` if `i + slen > 65535`. This is theoretical with well-formed
proto but worth adding an explicit overflow check:
```c
if ((uint32_t)i + slen > (uint32_t)len) return 0;
```

### 4.3 Realloc-table request dispatch (Decision 9, 2026-03-12)
🟢 **OK**

SLIST replaced with flat heap arrays. req table sorted ascending by `id_min`;
binary search dispatches in O(log n). Overlap detection at registration
prevents ambiguous dispatch. Mutex released before calling handler so
handlers may block. ✓

### 4.4 Response buffer sizing in rpc_ll_slist_req_handler
🟠 **RISK — may be insufficient for large responses**

```c
uint16_t resp_max = (uint16_t)(inlen + 512u);
```
A scan result with 20 APs encodes to ~4 KB. `inlen` for the request is ~20 bytes,
so `resp_max = 532` — response will be truncated.

**Fix:** Use a fixed large minimum:
```c
uint16_t resp_max = (uint16_t)(MAX(inlen + 512u, 4096u));
```
Or allocate from the transport mempool with the proper max frame size.

### 4.5 Event serialise buffer — fixed (Decision 9, 2026-03-12)
🟢 **OK**

Previously `send_event()` allocated only `len + 256` bytes with no `out_max`
passed to `serialise()`. Decision 9 fixed both sides:
- `send_event()` now allocates `MAX_SERIAL_DATA_SIZE` (4096 B floor).
- `eh_rpc_evt_serialise_t` signature now includes `uint16_t out_max` as last arg.
- Extension adapters (`mcu_evt_adapter`, `fg_evt_adapter`) forward `out_max`.
The `out_max` arg is enforced at the boundary; extensions must check before packing.
PENDING-011 tracks switching to a two-pass `get_packed_size` model later.

### 4.6 Response msg_id population (MCU dispatcher)
🟠 **RISK — not uniformly set**

In `esp_hosted_ext_rpc_mcu_dispatcher_req.c`, the error/fallthrough paths set:
```c
resp->msg_id = RPC_ID__Resp_Base;
```
For success paths, each `req_wifi_*()` handler is expected to set
`resp->msg_id = RPC_ID__Resp_<CommandName>` before returning. There is no
enforcement that every handler actually does this. A handler that forgets
returns `resp->msg_id = 0` (uninitialized), which the host will decode as
`Resp_Base` (unknown response) and time out waiting for the real response.

**Recommendation:** Add an assertion or log after dispatch:
```c
if (resp->msg_id == 0 || resp->msg_id == RPC_ID__Resp_Base) {
    ESP_LOGW(TAG, "Handler for req 0x%x did not set resp->msg_id", req->msg_id);
}
```

---

## 5. DPP Dispatcher Syntax Error

### 5.1 `#if H_DPP_SUPPORT` block inside switch/case
🔴 **BUG — compile error when H_DPP_SUPPORT is defined**

In `esp_hosted_ext_rpc_mcu_dispatcher_req.c`, lines ~394–420:
```c
#if H_DPP_SUPPORT
    {
        .req_num = RPC_ID__Req_SuppDppInit,
        .command_handler = req_supp_dpp_init
    },
    ...
#endif
```
This is **struct initializer syntax inserted inside a `switch` statement body**.
C does not allow compound literal initializers as bare statements in a switch.
When `H_DPP_SUPPORT` is enabled this will fail to compile.

Additionally, `RPC_ID__Req_SuppDppDeinit` appears **twice** in the block (lines
400–401 and 404–405) — a duplicate handler registration bug.

**Fix:** Convert each DPP entry to a proper `case` statement:
```c
#if H_DPP_SUPPORT
    case RPC_ID__Req_SuppDppInit:
        ret = req_supp_dpp_init(req, resp, priv_data);
        break;
    case RPC_ID__Req_SuppDppDeinit:
        ret = req_supp_dpp_deinit(req, resp, priv_data);
        break;
    case RPC_ID__Req_SuppDppBootstrapGen:
        ret = req_supp_dpp_bootstrap_gen(req, resp, priv_data);
        break;
    case RPC_ID__Req_SuppDppStartListen:
        ret = req_supp_dpp_start_listen(req, resp, priv_data);
        break;
    case RPC_ID__Req_SuppDppStopListen:
        ret = req_supp_dpp_stop_listen(req, resp, priv_data);
        break;
#endif
```

---

## 6. RPC Registry: Old vs New Command Ranges

### 6.1 Namespace map (no collision)
🟢 **OK**

| Extension | Request range | Response range | Event range |
|-----------|---------------|----------------|-------------|
| Linux FG  | 101–128 | 201–228 | 301–308 |
| MCU       | 0x101–0x183 (257–387) | 0x201–0x283 | 0x301–0x314 |
| V2 (future) | 0x400–0x5FF | 0x600–0x7FF | 0x800–0x9FF |

FG range is decimal 101–128; MCU range starts at decimal 257. No overlap. ✓

### 6.2 Both extensions registered simultaneously?
🟡 **GAP**

Both FG and MCU extensions will each try to register their req/evt nodes.
There is no guard preventing both from registering in the same build — and
since their msg_id ranges do not overlap, it would not cause an overlap error.
But a build should never have both active simultaneously (CP talks to ONE host
type). The Kconfig guard `if(CONFIG_ESP_HOSTED_CP_FOR_MCU)` in the MCU
extension CMakeLists ensures MCU sources only compile for MCU builds. The FG
extension CMakeLists needs the equivalent `if(CONFIG_ESP_HOSTED_CP_FOR_LINUX__802_03)`.

**Verify FG extension CMakeLists has the proper Kconfig guard.**

### 6.3 Unregister API — implemented (Decision 9, 2026-03-12)
🟢 **OK**

`esp_hosted_cp_rpc_req_unregister(id_min, id_max)` and
`esp_hosted_cp_rpc_evt_unregister(id_min, id_max)` are now implemented.
Both extension deinit functions call them. Hot-reload (unregister + re-register)
is covered by test group J in `registry_test.c`.

---

## 7. Host-Side Port Layer / H_XXX Abstraction (Phase 4)

### 7.1 MCU host: H_XXX mapping complete?
🟢 **OK** (known set), 🟡 **GAP** (additional macros not yet audited)

Mapped macros:
- `H_ESP_HOSTED_HOST_PLATFORM_MCU` ← `CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU`
- `H_ESP_HOSTED_DFLT_TASK_STACK` ← `CONFIG_ESP_HOSTED_DFLT_TASK_STACK` (default 3072)
- `H_LOWER_MEMCOPY` ← 0 (unused)
- `H_ESP_HOSTED_USE_WORKQUEUE` ← 0 (IDF never uses Linux workqueue)

`transport_drv.h` includes `port_esp_hosted_host_config.h`. The include chain is:
```
transport_drv.c → transport_drv.h → port_esp_hosted_host_config.h
```
Confirmed via grep. ✓

🟡 **GAP**: Transport-specific port headers are referenced in `transport_drv.h`:
```c
#if H_TRANSPORT_IN_USE == H_TRANSPORT_SPI
    #include "port_esp_hosted_host_spi.h"
```
`H_TRANSPORT_IN_USE` and `H_TRANSPORT_SPI` must be defined somewhere. Not
confirmed whether those are in `port_esp_hosted_host_config.h` or separate
port files. If missing, transport_drv.h won't compile.

### 7.2 Linux kmod: H_XXX abstraction
🟢 **OK**

`port/host/linux/include/port_esp_hosted_host_config.h` sets:
- `H_ESP_HOSTED_HOST_PLATFORM_MCU` — NOT defined (correct)
- `H_ESP_HOSTED_USE_WORKQUEUE` ← `CONFIG_ESP_HOSTED_USE_WORKQUEUE`
- `H_ESP_HOSTED_DFLT_TASK_STACK` = 0 (unused in kmod)

The kmod Makefile does NOT add `port/host/linux/include` to `EXTRA_CFLAGS` —
kmod files do not include port_esp_hosted_host_config.h directly (they use
kernel Kconfig). This is OK because kmod files don't use H_XXX macros;
they use Linux kernel APIs directly. ✓

---
