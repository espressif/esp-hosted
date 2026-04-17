# Glossary

## Acronyms

| Term | Meaning |
|------|---------|
| CP | Coprocessor — the ESP32-xx device running IDF as the wireless slave |
| Host | The system driving the CP: either a Linux PC/SBC (FG variant) or MCU (MCU variant) |
| FG | "Fully Grounded" — Linux kernel module host driver variant |
| MCU | Microcontroller host variant (IDF build, e.g. ESP32-S3) |
| TLV | Type-Length-Value — encoding used in PRIV boot-time frames |
| PRIV | Private interface — the control-plane channel between host and CP |
| SLIST | Singly-Linked List — **retired** from RPC dispatch; replaced by dynamic table |
| RPC | Remote Procedure Call — the request/response API between host and CP |
| V1 | Legacy 12-byte payload header; legacy protobuf RPC via protocomm |
| V2 | New 20-byte payload header (magic 0xE9); future unified RPC namespace |
| HDR | Wire-frame payload header |
| EP | Endpoint — a named protocomm channel ("RPCReq", "RPCEvt") |
| `_rpc_` ext | Lightweight extension — only registers RPC table entries, no background tasks |
| `_feat_` ext | Heavy extension — complex lifecycle: tasks, timers, explicit init/deinit |
| `EH_CP_EXT_REGISTER` | Linker-section macro for extension auto-init self-registration |

## Key Symbol Names

| Symbol | File | Value | Meaning |
|--------|------|-------|---------|
| `ESP_HOSTED_HDR_VERSION_V1` | `esp_hosted_common_header_v2.h` | 0x01 | 12-byte V1 header |
| `ESP_HOSTED_HDR_VERSION_V2` | `esp_hosted_common_header_v2.h` | 0x02 | 20-byte V2 header |
| `ESP_HOSTED_HDR_V2_MAGIC` | `esp_hosted_common_header_v2.h` | 0xE9 | Magic byte at byte[0] of V2 frames |
| `ESP_HOSTED_RPC_VERSION_V1` | `esp_hosted_common_tlv.h` | 0x01 | V1 variant-specific proto |
| `ESP_HOSTED_RPC_VERSION_V2` | `esp_hosted_common_tlv.h` | 0x02 | V2 unified proto (future) |
| `ESP_PRIV_HEADER_VERSION` | `esp_hosted_common_tlv.h` | 0x20 | TLV: slave→host wire-hdr version |
| `ESP_PRIV_HEADER_VERSION_ACK` | `esp_hosted_common_tlv.h` | 0x21 | TLV: host→slave agreed hdr version |
| `ESP_PRIV_RPC_VERSION` | `esp_hosted_common_tlv.h` | 0x22 | TLV: slave→host RPC version |
| `ESP_PRIV_RPC_VERSION_ACK` | `esp_hosted_common_tlv.h` | 0x23 | TLV: host→slave agreed RPC version |
| `ESP_PRIV_EVENT_INIT` | `esp_hosted_transport.h` | 0x22 | event_type in struct esp_priv_event |

## Naming Rules

### Source files
- CP coprocessor components: `esp_hosted_cp_*` or `esp_hosted_transport_cp_*`
- Host components: `eh_host_transport_*`
- Common canonical headers: `esp_hosted_common_*` under `components/common/`
- Proto components: `esp_hosted_proto_linux_v1`, `esp_hosted_proto_mcu_v1`, `esp_hosted_proto_v2`
- Extensions (lightweight): `esp_hosted_cp_ext_rpc_XXX`
- Extensions (heavy lifecycle): `esp_hosted_cp_ext_feat_XXX`

### Config macros
- IDF sdkconfig (port files only): `CONFIG_ESP_HOSTED_*`
- CP components: `EH_CP_XXX` (from `esp_hosted_cp_master_config.h`)
- Host components: `EH_HOST_XXX` (from `esp_hosted_host_master_config.h`)
- Common components: no config macros (must be config-neutral)

### TLV type codes
- Legacy (V1 backwards compat): `HOST_CAPABILITIES` = 0x01 through 0x05
- MCU-style PRIV tags: 0x11–0x19 (defined in MCU transport headers, do not alias)
- Phase-3+ negotiation TLVs: 0x20–0x2F (defined in `esp_hosted_common_tlv.h`)

## CRITICAL: ESP_PRIV_EVENT_INIT Is NOT a TLV Code

`ESP_PRIV_EVENT_INIT` (value 0x22) is the **event_type** byte inside
`struct esp_priv_event`, NOT a TLV type code. It must never be added to
`esp_hosted_common_tlv.h` as a `#define`. If defined as a macro, it shadows
the enum value in `esp_hosted_transport.h`, corrupting the startup handshake.
