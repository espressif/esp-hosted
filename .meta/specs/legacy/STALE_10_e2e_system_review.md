# 10 — End-to-End System Review

> **Purpose:** Complete architectural review — FG, MCU, NG-pending scenarios.
> Findings classified: ✅ Correct · ⚠️ Risk/Incomplete · ❌ Bug/Gap
> Applied fixes are marked **FIXED** with the session they were resolved in.

---

## 1. Boot / PRIV Handshake — End-to-End

### 1.1 CP Side (Slave)

```
main() / app_main()
  → esp_hosted_cp_init()
      → esp_hosted_cp_rpc_registries_init()   ← mutex created
      → extensions init (add_cap_bits, rpc_req_register, rpc_evt_register)
      → transport init (SPI/SDIO/UART/SPI-HD)
      → esp_hosted_cp_protocomm_init(write_cb, read_cb)
          → protocomm_add_endpoint("RPCRsp", rpc_ll_slist_req_handler)
          → protocomm_add_endpoint("RPCEvt", rpc_ll_slist_evt_handler)
      → xTaskCreate(host_reset_task)
      → xTaskCreate(recv_task)

host_reset_task:
  → esp_hosted_cp_get_caps/ext_caps/feat_caps()
  → generate_startup_event(caps, ext_caps, raw_tp_cap, feat_caps)
      TLVs emitted: 0x12 chip_id, 0x11 caps, 0x14/0x15 q_size, 0x16 ext_caps,
                    0x17 fw_ver, 0x18 sdio_mode, 0x19 feat_caps[8],
                    0x20 hdr_version, 0x22 rpc_version
```

✅ All 4 transport variants emit the full TLV set including 0x20/0x22.
✅ feat_caps[8] populated from Registry 2 accumulator.
✅ Extensions must call add_cap_bits() before host_reset_task start — sequencing is
   caller responsibility, documented in core.h.

### 1.2 Host Side — MCU (FreeRTOS)

```
transport_drv.c: process_init_event()
  → parses TLVs 0x11–0x19, 0x20, 0x22
  → agrees min(host,CP) version → host_hdr_ver_agreed, host_rpc_ver_agreed
  → send_slave_config(..., host_hdr_ver_agreed, host_rpc_ver_agreed)
      → TLVs sent: 0x44–0x48 (slave config) + 0x21 + 0x23 (ACK)
CP host_to_slave_reconfig():
  → parses 0x21 → WRITE_ONCE(hdr_ver_negotiated, agreed)
  → parses 0x23 → WRITE_ONCE(rpc_ver_negotiated, agreed)
```

✅ Full negotiation round-trip complete.
✅ ACK TLVs 0x21/0x23 correctly sent by MCU host.

### 1.3 Host Side — Linux kmod (FIXED: GAP-001, Session 7)

```
esp_spi.c / esp_sdio.c: process_init_event()
  → parses TLVs 0x11–0x19, 0x20, 0x22
  → WRITE_ONCE(adapter->hdr_ver_negotiated / rpc_ver_negotiated)
  → esp_add_card()  (first bootup)
  → esp_send_host_config(adapter, hdr_ver_ack, rpc_ver_ack)   ← NEW (GAP-001 fix)
      → builds ESP_PRIV_EVENT_INIT frame with 0x44–0x48 + 0x21 + 0x23
      → esp_send_packet() → transport write
CP host_to_slave_reconfig():
  → parses 0x21/0x23 → updates hdr_ver_negotiated / rpc_ver_negotiated
```

✅ esp_send_host_config() added to main.c, declared in esp_api.h.
✅ Both SPI and SDIO kmod call it after first boot esp_add_card().
✅ Adapter fields are read once before building the ACK packet.

---

## 2. RPC — Request Path

### 2.1 Endpoint Name (FIXED: FIX-002, Session 7)

RPC_EP_NAME_REQ in cp_core.h is now `"RPCRsp"` (wire-compatible with all existing
FG and MCU hosts which send to `"RPCRsp"`). Comment documents wire-compat constraint.

✅ CP registers `"RPCRsp"` endpoint.
✅ FG host (ctrl_lib/serial_if.c) sends to `CTRL_EP_NAME_RESP = "RPCRsp"`. ✓
✅ MCU host (transport_drv.c) sends to `RPC_EP_NAME_RSP = "RPCRsp"`. ✓

### 2.2 Dispatch Chain (FG + MCU paths)

```
SERIAL_IF frame → esp_hosted_cp_process_serial_rx_pkt()
  → protocomm_pserial_ctrl_req_handler()
      → parse_tlv() [bounds-checked — FIX-006 applied]
      → protocomm_req_handle(pc, "RPCRsp", data, len, &out, &outlen)
          → rpc_ll_slist_req_handler()
              → esp_hosted_proto_extract_msg_id() → msg_id
              → calloc(4096) for response buffer [FIX-007: fixed-size replaces inlen+512]
              → esp_hosted_cp_rpc_dispatch_req(msg_id, ...)
                  → walks g_rpc_req_list sorted ascending
                  → FG [100-128]: fg_slist_req_handler → fg_rpc_req_handler (protobuf)
                  → MCU [256-387]: mcu_slist_req_handler → mcu_rpc_req_handler (protobuf)
              → sets *outbuf = resp, *outlen = resp_len
      → compose_tlv("RPCRsp", &out, &outlen)
      → xmit(out, outlen)
```

✅ SLIST dispatch correctly routes by msg_id range.
✅ FG range [100-128] and MCU range [256-387] do not overlap.

### 2.3 Response Message ID

```
mcu_rpc_req_handler() (hooks.c):
  resp->msg_id = req->msg_id - RPC_ID__Req_Base + RPC_ID__Resp_Base
               = req->msg_id - 256 + 512 = req->msg_id + 256
  resp->uid    = req->uid           ← host matches async responses by uid
  resp->payload_case = resp->msg_id ← correct for protobuf oneof dispatch
```

✅ Response ID calculation is correct for all MCU request IDs.
✅ uid echoed back for async matching.
⚠️ Error fallback: `resp->msg_id = RPC_ID__Resp_Base` (512) means "unknown command".
   Host must distinguish this from a valid `Resp_Base` response. Currently handled
   by checking `resp->payload_case` — if both are 512 and payload_case has no
   payload set, host treats it as error. Acceptable.

---

## 3. RPC — Event Path

### 3.1 CP→Host Event Flow

```
WiFi ESP event → ext_rpc_mcu event subscriber callback
  → esp_hosted_cp_rpc_send_event(event_id, data, len)
      → finds matching evt node [MCU: 769-788, FG: 301-308]
      → node->serialise() → mcu_rpc_event_handler():
          ntfy->msg_id = event_id; ntfy->payload_case = event_id
          → encoded proto bytes in *outbuf
      → alloc max(4096, len+256) bytes [FIX-003 applied]
      → esp_hosted_cp_protocomm_process_rpc_evt("RPCEvt", event_id, out, out_len)
          → compose_tlv("RPCEvt", &out, &outlen)
          → xmit
```

✅ Event routing correct.
✅ Event buffer uses 4096B floor to avoid small-alloc truncation.

### 3.2 Event ID Ranges

| Extension | Evt Range    | Decimal    |
|-----------|-------------|------------|
| FG Linux  | 301 – 308   | decimal    |
| MCU       | 769 – 788   | (0x301–0x314 hex) |

✅ No overlap. Confusing naming (both use `*_Base+1`) documented in specs/03.

---

## 4. Interface RX/TX Registry — Status

### 4.1 iface_table vs process_rx_pkt (GAP-002 — documented, not fixed)

`g_iface_table[]` and `esp_hosted_cp_dispatch_rx()` are defined and work correctly
as an API, but `process_rx_pkt()` in cp_core.c uses hard-coded if_type checks
and does NOT call `esp_hosted_cp_dispatch_rx()`.

This means extensions that call `esp_hosted_cp_register_rx_cb()` for ESP_STA_IF
or ESP_AP_IF will NOT receive frames via that callback — the frames go directly to
`esp_wifi_internal_tx()` instead.

**Current use:** No existing extension registers an RX callback for STA/AP — the
WiFi forwarding is handled directly in core.c and is correct for the current use
case. The iface_table API is presently used only for TX overrides (network split).

**Action needed (PENDING-009):** Wire `esp_hosted_cp_dispatch_rx()` into `process_rx_pkt()`,
or document that iface_table RX callbacks are unsupported and remove the API.

---

## 5. Capabilities — All Three Tiers

### 5.1 CP Registry 2 → Startup TLVs

| Source | caps (0x11) | ext_caps (0x16) | feat_caps[8] (0x19) |
|--------|------------|-----------------|---------------------|
| FG ext | ESP_WLAN_SUPPORT [FIXED FIX-004] | 0 | 0 |
| MCU ext | 0 | ESP_WLAN_SUPPORT | 0 |
| Core   | bits set by transport | — | — |

✅ FG extension now calls esp_hosted_cp_add_cap_bits(ESP_WLAN_SUPPORT, 0).
✅ feat_caps[8] emitted in all 4 transport startup events.

### 5.2 Host MCU — All Three Tiers Parsed and Stored

✅ capabilities, ext_capabilities, feat_caps[8] all stored in transport context.

### 5.3 kmod — All Three Tiers

✅ adapter->capabilities (0x11) stored.
✅ adapter->feat_caps[8] (0x19) stored — FIXED FIX-005.
⚠️ ext_caps (0x16) parsed and logged but not stored in adapter struct.
   Low impact now; may need `adapter->ext_caps` field if features are gated by it.

---

## 6. Serial / RPC Framing

### 6.1 protocomm TLV Serial Format

```
[ Type(1) | Length:2LE | Value(Length) ]
Fields: EPNAME(1), DATA(2)
```

✅ parse_tlv / compose_tlv consistent.
✅ Bounds check added (FIX-006): rejects truncated TLVs.

### 6.2 Response Buffer Sizing (FIX-007)

`rpc_ll_slist_req_handler` now pre-allocates 4096 bytes.
`mcu_slist_req_handler` detects overflow and logs if encoded size > 4096.

⚠️ Absolute 4096B cap: if a future response is larger (e.g., very long scan list,
   OTA data), it will fail. A dynamic sizing path (handler allocates own buffer)
   is the correct long-term fix — tracked as PENDING-010.

---

## 7. Transport Layer — All Variants

### 7.1 CP Transports

✅ All 4 transports (SPI, SDIO, UART, SPI-HD): correct TLV emission, startup event.
✅ Kconfig correctly gates each transport; CMakeLists compiles only the selected one.

### 7.2 MCU Host Transports (FIXED: FIX-008)

CMakeLists now conditionally includes only the selected MCU transport driver:
```cmake
if(CONFIG_ESP_HOSTED_HOST_MCU_TRANSPORT_SPI) → spi_drv.c
elseif(CONFIG_ESP_HOSTED_HOST_MCU_TRANSPORT_SDIO) → sdio_drv.c
...
```

✅ Dead transport code no longer compiled.

### 7.3 Port Include Path (FIXED: FIX-009, FIX-011)

host transport CMakeLists now includes:
- `port/host/idf/include` (port_esp_hosted_host_config.h)
- `components/common/esp_hosted_common/include` (canonical TLV/caps headers)

✅ Build no longer broken by missing include path.

---

## 8. Kconfig and CMake

✅ CP Kconfig: host type, transport type, queue sizes all correct.
✅ `ESP_HOSTED_CP_RPC_NODE_CONTAINS_NAME` added to Kconfig (FIX-010).
✅ Host Kconfig: platform + transport choice correct.
✅ IDF port header (port_esp_hosted_host_config.h) correctly bridges CONFIG_XXX → H_XXX.
✅ Linux kmod Makefile correctly adds canonical common to EXTRA_CFLAGS.
⚠️ `CONFIG_ESP_HOSTED_CP_WIFI_ENABLED` used in core.c — verify it is defined in
   the CP Kconfig or board sdkconfig default.

---

## 9. NG Variant — Pending

| ID | Item |
|----|------|
| NG-001 | NG build isolation: separate cp_mcu_ng example/target needed |
| NG-002 | Downgrade detection: 0x20 TLV mismatch handling |
| NG-003 | Reserve 0x1A–0x1F for NG-specific TLVs |
| NG-004 | NG proto file (PENDING-006), SLIST routing for NG msg_ids |
| NG-005 | NG MCU-style host must implement ACK path (now resolved for Linux kmod) |

---

## 10. Summary of All Findings

### Applied Fixes

| ID | Severity | Description | Status |
|----|----------|-------------|--------|
| FIX-001 | MEDIUM | `PCOM_EP_REQ = "RPCRsp"` — misleading legacy constant | ⚠️ Not renamed — still in priv.h as legacy; no runtime impact since not used by SLIST path |
| FIX-002 | **CRITICAL** | RPC_EP_NAME_REQ changed to `"RPCRsp"` for wire compat | ✅ FIXED S7 |
| FIX-003 | HIGH | Event alloc: min 4096B floor | ✅ FIXED S6 |
| FIX-004 | HIGH | FG ext now calls add_cap_bits(ESP_WLAN_SUPPORT, 0) | ✅ FIXED S6 |
| FIX-005 | HIGH | feat_caps[8] added to struct esp_adapter + stored | ✅ FIXED S6 |
| FIX-006 | HIGH | parse_tlv bounds check | ✅ FIXED S7 |
| FIX-007 | **CRITICAL** | resp buffer: 4096B fixed alloc (was inlen+512) | ✅ FIXED S6 |
| FIX-008 | LOW | MCU transport CMakeLists: conditional compile | ✅ FIXED S7 |
| FIX-009 | **CRITICAL** | port/host/idf/include added to host transport CMakeLists | ✅ FIXED S7 |
| FIX-010 | LOW | RPC_NODE_CONTAINS_NAME added to Kconfig | ✅ FIXED S6 |
| FIX-011 | MEDIUM | canonical common dir added to host transport CMakeLists | ✅ FIXED S7 |
| GAP-001 | HIGH | kmod sends ACK TLVs 0x21/0x23 via esp_send_host_config() | ✅ FIXED S7 |

### Remaining / Tracked

| ID | Severity | Description |
|----|----------|-------------|
| GAP-002 | HIGH | iface_table dispatch_rx not called in process_rx_pkt (PENDING-009) |
| RISK-001 | MEDIUM | Extension caps ordering — enforced by call order only |
| RISK-002 | LOW | Absolute 4096B resp cap; dynamic sizing needed long-term (PENDING-010) |
| RISK-003 | LOW | kmod ext_caps not stored in adapter struct |
| RISK-004 | LOW | FG/MCU event range naming confusion (documented in specs/03) |

### Pending (carry forward)

| ID | Description |
|----|-------------|
| PENDING-005 | V2 wire header TX encode + RX magic-byte decode |
| PENDING-006 | esp_hosted_rpc_v2.proto schema (0x400–0x9FF) |
| PENDING-007 | RPC SLIST switch to V2 based on rpc_ver_negotiated |
| PENDING-009 | Wire iface_table dispatch_rx into process_rx_pkt |
| PENDING-010 | Dynamic response buffer sizing in rpc_ll_slist_req_handler |
