# 12 — RPC End-to-End Call Flow

> Complete function-level call chains for every host+CP variant.
> Files are shown as `file.c::function()`.

---

## Background: the endpoint name problem

The TLV serial framing protocol tags each frame with a string "endpoint name".
The CP uses this string to route the frame to a registered handler via `protocomm_req_handle()`.
Two host variants historically used different strings:

| Side | Variant | Req→Slave ep_name | Evt←Slave ep_name |
|------|---------|--------------------|-------------------|
| Host | Linux FG (old ctrl_lib) | `"ctrlResp"` | `"ctrlEvnt"` |
| Host | MCU (esp-hosted-mcu) | `"RPCRsp"` | `"RPCEvt"` |
| CP (unified) | registers | `"RPCRsp"` + `"RPCEvt"` | — |
| CP (legacy compat ON) | also registers | `"ctrlResp"` + `"ctrlEvnt"` | — |

Both endpoint strings route to **the same SLIST dispatch handlers** on the CP.
The msg_id field inside the protobuf payload determines which extension handles the call.

---

## 1. Linux FG Host → Coprocessor: Request/Response

### Host side files
```
esp_hosted_fg/host/components/eh_host_rpc_lib/src/
  serial_if.c
  platform_wrapper.c (Linux port)
```

### CP side files
```
esp_hosted_fg/coprocessor/components/esp_hosted_cp/src/
  esp_hosted_cp_rpc_ll.c      — protocomm + TLV layer
  esp_hosted_cp_rpc.c         — serial RX entry point
  esp_hosted_cp_registries.c  — SLIST dispatcher
esp_hosted_fg/coprocessor/extensions/esp_hosted_ext_rpc_linux_fg/src/
  esp_hosted_ext_rpc_linux_fg_core.c            — FG SLIST adapter
  esp_hosted_ext_rpc_linux_fg_protocomm_hook.c  — protobuf encode/decode
  esp_hosted_ext_rpc_linux_fg_wifi_reqs.c etc.  — per-command handlers
```

### Call flow

```
HOST (Linux userspace / kernel module)
──────────────────────────────────────────────────────────────────────

1. User calls ctrl API e.g. esp_wifi_get_mode()
   → eh_host_rpc_lib/src/ctrl_core.c :: ctrl_app_send_req()

2. Serialise request into CtrlMsg protobuf
   → ctrl_core.c :: ctrl_msg_encode_req()    [ctrl_msg__pack()]

3. Build TLV frame — ep_name = "ctrlResp", data = packed proto bytes
   → serial_if.c :: compose_tlv(buf, data, data_len)
     [CTRL_EP_NAME_RESP = "ctrlResp" embedded in frame]

4. Write TLV frame to /dev/espspi or /dev/espsdio
   → platform_wrapper.c :: send_to_device(buf, len)
   → kernel driver SPI/SDIO write → SPI/SDIO wire

COPROCESSOR (ESP32 CP firmware)
──────────────────────────────────────────────────────────────────────

5. SPI/SDIO interrupt, DMA complete
   → esp_hosted_transport_cp/src/esp_hosted_transport_cp_spi.c
       :: transport_cp_spi_rx_task()
   → calls esp_hosted_cp_process_serial_rx_pkt(buf)

6. Reassemble fragments, store in static rx_data buffer
   → esp_hosted_cp_rpc.c :: esp_hosted_cp_process_serial_rx_pkt()
   → calls protocomm_rpc_req_ind(pc, buf, len, msg_id)

7. Copy buf into queue item, enqueue
   → esp_hosted_cp_rpc_ll.c :: protocomm_pserial_data_ready()
       malloc(len), memcpy, xQueueSend

8. pserial_task dequeues
   → esp_hosted_cp_rpc_ll.c :: pserial_task()
   → calls protocomm_pserial_ctrl_req_handler(pc, arg.data, len)

9. Parse TLV: extract ep_name ("ctrlResp") and proto data
   → esp_hosted_cp_rpc_ll.c :: parse_tlv() loop
   → ep_name = "ctrlResp"  (or "RPCRsp" from a new FG host)

10. Dispatch to registered handler by ep_name
    → IDF protocomm :: protocomm_req_handle(pc, "ctrlResp", 0, data, len, &out, &outlen)
    → looks up "ctrlResp" in protocomm endpoint table
    → calls rpc_ll_slist_req_handler()
    [CONFIG_ESP_HOSTED_LEGACY_FG_EP_COMPAT enables "ctrlResp" alias;
     both "ctrlResp" and "RPCRsp" map to rpc_ll_slist_req_handler]

11. Extract msg_id from raw proto bytes (field 2 scanner, no full decode)
    → esp_hosted_cp_rpc_ll.c :: rpc_ll_slist_req_handler()
        :: extract_msg_id_from_proto(inbuf, inlen) → msg_id e.g. 102

12. Walk SLIST, find matching request node by msg_id range
    → esp_hosted_cp_registries.c :: esp_hosted_cp_rpc_dispatch_req()
    → finds s_fg_req_node  (registered by FG extension: range [100,128])
    → calls node->handler(ctx, msg_id, req_buf, req_len, resp_buf, &resp_len, resp_max)
       = fg_slist_req_handler()

13. FG SLIST adapter: bridge to protocomm-style handler
    → esp_hosted_ext_rpc_linux_fg_core.c :: fg_slist_req_handler()
    → calls linux_rpc_req_handler(msg_id, inbuf, inlen, &out, &out_len, NULL)

14. Unpack CtrlMsg, dispatch to per-command handler
    → esp_hosted_ext_rpc_linux_fg_protocomm_hook.c :: linux_rpc_req_handler()
        ctrl_msg__unpack(NULL, inlen, inbuf) → req
        ctrl_msg__init(&resp)
        resp.msg_id = req.msg_id - Req_Base + Resp_Base
        esp_hosted_ext_rpc_linux_fg_rpc_req_dispatcher(req, &resp, NULL)
        → e.g. req_get_wifi_mode_handler(req, &resp, NULL)
        ctrl_msg__get_packed_size(&resp) → *out_len
        calloc(1, *out_len) → *out
        ctrl_msg__pack(&resp, *out)

15. Back in fg_slist_req_handler: copy packed resp into resp_buf, free out

16. Back in rpc_ll_slist_req_handler: *outbuf = resp_buf (the 4096B calloc)

17. Back in protocomm_req_handle: out = resp_buf, outlen = resp_len

18. Wrap response in TLV: ep_name = "ctrlResp", data = packed proto
    → esp_hosted_cp_rpc_ll.c :: compose_tlv("ctrlResp", &out, &outlen)

19. Transmit TLV frame back to host
    → (pserial_cfg->xmit)(out, outlen)
    = esp_hosted_cp_rpc.c :: serial_write_data(data, len)
    → builds interface_buffer_handle_t
    → send_to_host_queue(&buf_handle, PRIO_Q_SERIAL)
    → transport layer DMA → SPI/SDIO wire → host

HOST (receives response)
──────────────────────────────────────────────────────────────────────

20. Read TLV frame from /dev/espspi
    → platform_wrapper.c :: recv_from_device()

21. Parse TLV, extract ep_name ("ctrlResp" matches CTRL_EP_NAME_RESP)
    → serial_if.c :: read_from_slave() :: parse_tlv loop

22. Unpack CtrlMsg, extract response fields
    → ctrl_core.c :: ctrl_msg_decode_resp()

23. Return result to caller
```

---

## 2. MCU Host → Coprocessor: Request/Response

### Host side files
```
esp_hosted_mcu/host/drivers/rpc/
  core/rpc_req.c          — request builder
  core/rpc_rsp.c          — response parser
  slaveif/rpc_slave_if.c  — serial TLV framing
esp_hosted_mcu/host/drivers/virtual_serial_if/serial_if.c
```

### CP side files
Same as FG except the SLIST node is from the MCU extension:
```
esp_hosted_fg/coprocessor/extensions/esp_hosted_ext_rpc_mcu/src/
  esp_hosted_ext_rpc_mcu.c              — MCU SLIST adapter
  esp_hosted_ext_rpc_mcu_hooks.c        — protobuf encode/decode (Rpc proto)
  esp_hosted_ext_rpc_mcu_handler_req_wifi.c etc.
```

### Call flow (steps 1–4 differ, steps 5–11 identical)

```
HOST (MCU, e.g. STM32 or Arduino)
──────────────────────────────────────────────────────────────────────

1. Application calls rpc API e.g. wifi_get_mode()
   → rpc_req.c :: rpc_wifi_get_mode()

2. Serialise into Rpc protobuf (msg_id in MCU range e.g. 0x109)
   → rpc_req.c :: rpc_send_req(Rpc *req)
   → rpc__get_packed_size(req), rpc__pack(req, buf)

3. Build TLV frame — ep_name = "RPCRsp" (RPC_EP_NAME_RSP)
   → serial_if.c :: compose_tlv(buf, packed_data, packed_len)

4. Write to SPI/SDIO/UART bus
   → serial_ll_if.c :: serial_ll_send_frame()

COPROCESSOR (steps 5-10 identical to FG path above)
──────────────────────────────────────────────────────────────────────

(5-9 identical: SPI RX → esp_hosted_cp_process_serial_rx_pkt
 → protocomm_pserial_data_ready → pserial_task
 → protocomm_pserial_ctrl_req_handler → parse_tlv)

10. protocomm_req_handle(pc, "RPCRsp", 0, data, len, &out, &outlen)
    → "RPCRsp" is the primary registered endpoint
    → calls rpc_ll_slist_req_handler()

11. extract_msg_id_from_proto() → msg_id e.g. 0x109

12. Walk SLIST, find s_mcu_req_node (range [0x100, 0x183])
    → node->handler = mcu_slist_req_handler()

13. MCU SLIST adapter: bridge to protocomm-style handler
    → esp_hosted_ext_rpc_mcu.c :: mcu_slist_req_handler()
    → calls mcu_rpc_req_handler(msg_id, inbuf, inlen, &out, &out_len, NULL)

14. Unpack Rpc, dispatch to per-command handler
    → esp_hosted_ext_rpc_mcu_hooks.c :: mcu_rpc_req_handler()
        rpc__unpack(NULL, inlen, inbuf) → req
        calloc(1, sizeof(Rpc)) → resp
        rpc__init(resp)
        resp.msg_id = req.msg_id - RPC_ID__Req_Base + RPC_ID__Resp_Base
        esp_hosted_ext_rpc_mcu_rpc_req_dispatcher(req, resp, NULL)
        → e.g. rpc_req_wifi_get_mode(req, resp, NULL)
        calloc(1, rpc__get_packed_size(resp)) → *outbuf
        rpc__pack(resp, *outbuf)
        esp_rpc_cleanup(resp)

15. mcu_slist_req_handler: memcpy outbuf→resp_buf, free(outbuf)

(16-19 identical: compose_tlv → xmit → serial_write_data → SPI wire)

HOST (receives response)
──────────────────────────────────────────────────────────────────────

20. SPI interrupt, read TLV frame
    → serial_if.c :: parse_tlv_frame()
    → ep_name = "RPCRsp" matches RPC_EP_NAME_RSP

21. Unpack Rpc response, route by msg_id
    → rpc_rsp.c :: rpc_process_resp(buf, len)
    → e.g. rpc_resp_wifi_get_mode()

22. Wake waiting caller, return result
```

---

## 3. Coprocessor → Linux FG Host: Event

### Flow

```
CP (ESP32 firmware, e.g. WiFi STA connected event fires)
──────────────────────────────────────────────────────────────────────

1. WiFi event callback fires
   → esp_hosted_ext_rpc_linux_fg_event_subscriber.c
       :: fg_wifi_event_handler()  [ESP_EVENT_HANDLER registered]

2. Call legacy event API (guarded by CONFIG_ESP_HOSTED_LEGACY_SEND_EVENT_TO_HOST_API)
   → esp_hosted_send_event_to_host("ctrlEvnt", CTRL_MSG_ID__Event_StaConnected,
                                    raw_data, raw_len)
   → esp_hosted_cp_core.c :: esp_hosted_send_event_to_host()
   → esp_hosted_cp_rpc.c  :: esp_hosted_cp_process_rpc_evt("ctrlEvnt", ...)
   → esp_hosted_cp_rpc_ll.c :: esp_hosted_cp_protocomm_process_rpc_evt()
   → protocomm_rpc_evt_ind(pc, "ctrlEvnt", data, size, event_id)
   → protocomm_pserial_data_ready(pc, "ctrlEvnt", data, size, event_id, PROTO_EVT_ENDPOINT)
     [malloc(size), memcpy, xQueueSend]

   NOTE: When legacy API is disabled, FG extension migrates to:
   → esp_hosted_cp_registries.c :: esp_hosted_cp_rpc_send_event(event_id, data, len)
     (finds FG evt SLIST node, calls fg_slist_evt_serialise → linux_rpc_event_handler
      → CtrlMsg encode → protocomm_process_rpc_evt("RPCEvt", packed, len))

3. pserial_task dequeues (PROTO_EVT_ENDPOINT)
   → esp_hosted_cp_rpc_ll.c :: pserial_task()
   → calls protocomm_pserial_ctrl_evnt_handler(pc, "ctrlEvnt", data, size, event_id)

4. protocomm_req_handle(pc, "ctrlEvnt", event_id, data, size, &out, &outlen)
   → "ctrlEvnt" endpoint is registered (alias, same rpc_ll_slist_evt_handler)
   → rpc_ll_slist_evt_handler() copies inbuf → *outbuf (pass-through)

5. compose_tlv("ctrlEvnt", &out, &outlen)
   → wraps proto bytes in TLV envelope with ep_name = "ctrlEvnt"

6. xmit(out, outlen) → serial_write_data → SPI/SDIO wire → host

HOST
──────────────────────────────────────────────────────────────────────

7. Read TLV, ep_name = "ctrlEvnt" matches CTRL_EP_NAME_EVENT
   → serial_if.c :: read_from_slave() parse_tlv
   → ep_name matches → extract payload

8. Unpack CtrlMsg event, dispatch to registered callback
   → ctrl_core.c :: process_event_msg()
   → e.g. event_ESPInit_handler() / event_StaConnected_handler()
```

---

## 4. Coprocessor → MCU Host: Event

```
CP
──────────────────────────────────────────────────────────────────────

1. WiFi event callback fires
   → esp_hosted_ext_rpc_mcu_event_subscriber.c
       :: mcu_wifi_event_handler()

2. Call legacy event API
   → esp_hosted_send_event_to_host("RPCEvt", RPC_ID__Event_StaConnected,
                                    raw_data, raw_len)

   OR (new SLIST path):
   → esp_hosted_cp_rpc_send_event(RPC_ID__Event_StaConnected, raw_data, len)
     → registries.c send_event()
     → finds s_mcu_evt_node (range covers this event_id)
     → calls n->serialise() = mcu_slist_evt_serialise()
         [in esp_hosted_ext_rpc_mcu.c]
         calls mcu_rpc_event_handler(event_id, raw, len, &out, &out_len, NULL)
         → esp_hosted_ext_rpc_mcu_hooks.c :: mcu_rpc_event_handler()
             calloc(sizeof(Rpc)) → ntfy
             ntfy.msg_id = event_id
             ntfy.msg_type = RPC_TYPE__Event
             rpc_evt_dispatcher(ntfy, raw, len)   [fills event payload fields]
             calloc(packed_size) → *outbuf
             rpc__pack(ntfy, *outbuf)
             esp_rpc_cleanup(ntfy)
         → copies packed bytes into SLIST-provided out buffer
         → free(mcu_outbuf)
     → esp_hosted_cp_protocomm_process_rpc_evt("RPCEvt", packed, packed_len)

3. pserial_task → protocomm_pserial_ctrl_evnt_handler(pc, "RPCEvt", ...)
4. protocomm_req_handle → rpc_ll_slist_evt_handler → pass-through copy
5. compose_tlv("RPCEvt", &out, &outlen)
6. xmit → serial_write_data → SPI wire

MCU HOST
──────────────────────────────────────────────────────────────────────

7. SPI interrupt, read TLV
   → serial_if.c :: parse_tlv_frame()
   → ep_name = "RPCEvt" matches RPC_EP_NAME_EVT

8. Unpack Rpc event, dispatch by msg_id
   → rpc_evt.c :: rpc_process_event(buf, len)
   → e.g. rpc_evt_sta_connected()

9. Call user event callback
```

---

## 5. How Both Hosts Can Coexist on One CP

The CP simply registers both endpoint names pointing to the same handlers:

```
protocomm endpoint table (after init with LEGACY_FG_EP_COMPAT=y):

  "RPCRsp"   →  rpc_ll_slist_req_handler   ← MCU host uses this
  "RPCEvt"   →  rpc_ll_slist_evt_handler   ← CP uses for MCU events
  "ctrlResp" →  rpc_ll_slist_req_handler   ← FG host uses this (alias)
  "ctrlEvnt" →  rpc_ll_slist_evt_handler   ← CP uses for FG events (alias)
```

When a request arrives, the SLIST dispatcher looks at msg_id inside the proto
payload — not the endpoint name — to decide which extension handles it:

```
msg_id  100–128   →  FG Linux extension   (FG Req_Base..Req_Max)
msg_id  200–228   →  FG Linux extension   (FG Resp_Base..Resp_Max)
msg_id  301–308   →  FG Linux extension   (FG Event range)
msg_id  0x100–0x183 → MCU extension       (MCU Req range)
msg_id  0x200–0x283 → MCU extension       (MCU Resp range)
msg_id  0x301–0x314 → MCU extension       (MCU Event range)
```

This means:
- A FG host sending `"ctrlResp"` with msg_id 102 → hits FG extension node → correct
- A MCU host sending `"RPCRsp"` with msg_id 0x109 → hits MCU extension node → correct
- A FG host sending `"ctrlResp"` with msg_id 0x109 would hit MCU extension → wrong,
  but this cannot happen because the FG host uses msg_ids in the 100–128 range only.

**Events in the reverse direction** (CP → host) always use the ep_name that
matches the host's expectation. The FG extension calls
`esp_hosted_send_event_to_host("ctrlEvnt", event_id, data, len)` and
the MCU extension calls with `"RPCEvt"`. The host filters by ep_name in its
TLV parse step, so each host only receives events tagged for it.

---

## 6. Kconfig: How to Migrate

| Config option | Default | Remove when |
|---------------|---------|-------------|
| `CONFIG_ESP_HOSTED_LEGACY_FG_EP_COMPAT` | `y` | All FG hosts updated to send `"RPCRsp"` |
| `CONFIG_ESP_HOSTED_LEGACY_ADD_ENDPOINT_API` | `y` | All extensions use `esp_hosted_cp_rpc_req_register()` |
| `CONFIG_ESP_HOSTED_LEGACY_SEND_EVENT_TO_HOST_API` | `y` | All event callers use `esp_hosted_cp_rpc_send_event()` |

When all three are `n`, the build will fail if any legacy caller remains — which
is the intended mechanism to force the migration to completion.
