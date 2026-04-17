# 15 — Socket Transport and System Testing
<!-- Created: 2026-03-11 Session 15 -->

## 1. Motivation

All existing CP transports (SPI, SDIO, UART, SPI-HD) require real hardware to
exercise. The socket transport replaces the physical bus with a TCP loopback so
that the full RPC stack — frame encode/decode, PRIV handshake, capability
accumulation, RPC dispatch, proto pack/unpack — can be exercised in CI without
any ESP32 attached.

This is **Phase T** in the implementation plan (`specs/09_implementation_status.md`).

---

## 2. Design Decision — Socket vs Other Approaches

| Option | Pro | Con |
|--------|-----|-----|
| **Socket transport (chosen)** | Identical frame path; both peers are real code; covers PRIV handshake | Needs both CP and host built and running |
| Unity test stubs (mock transport) | Fast; no second process | Cannot test PRIV negotiation or real proto encoding |
| QEMU | Full hardware simulation | Complex to set up; IDF QEMU support limited for SPI/SDIO |
| Hardware-in-the-loop | Most realistic | Cannot run in CI; expensive gate |

Socket transport is the best balance: it exercises the real code paths while
staying fully CI-compatible. Mock/unit tests cover individual components; socket
system tests cover integration.

---

## 3. CP-Side Socket Transport

### File
`components/coprocessor/esp_hosted_cp_transport/src/esp_hosted_transport_cp_socket.c`

### Behaviour
- TCP server on `EH_CP_SOCKET_PORT` (default: 8765, from `esp_hosted_cp_master_config.h`)
- Accepts one connection at a time (same as SPI: one host)
- RX task: `recv()` → `esp_hosted_frame_decode()` → `process_rx_pkt()`
- TX path: `esp_hosted_frame_encode()` → `send()`
- On IDF/FreeRTOS: uses lwIP sockets. On Linux userspace: uses POSIX sockets.
- `generate_startup_event()` called after connection accepted (same as SPI after handshake)

### CMake
Conditionally compiled: `if (EH_CP_TRANSPORT_SOCKET)` — never included in
production firmware builds. Enabled only in test/socket example and CI builds.

### Kconfig
```
config ESP_HOSTED_CP_TRANSPORT_SOCKET
    bool "Socket (test only — no real hardware)"
    default n
    help
      Enables TCP socket transport for system testing without hardware.
      Do not enable in production firmware.
```

---

## 4. Host-Side Socket Transport

### Location
`esp_hosted_mcu/host/components/transport/socket/`
(mirrors SPI/SDIO transport dirs in the MCU repo)

### Behaviour
- TCP client connecting to `EH_HOST_SOCKET_IP`:`EH_HOST_SOCKET_PORT`
- Same frame API as SPI transport: `transport_drv.c` calls the same
  `frame_encode()` / `frame_decode()` regardless of physical medium
- Reconnect-on-disconnect with configurable retry interval

### Integration
The MCU host transport abstraction (`transport_drv.c`) already selects the
transport at build time via `CONFIG_ESP_HOSTED_HOST_TRANSPORT_*`. Socket is
added as one more option, enabled only for test builds.

---

## 5. Test Directory Layout

```
esp_hosted.new/
└── test/
    ├── unit/                         ← per-component, no transport needed
    │   ├── test_frame/               (encode/decode, checksum, magic byte)
    │   ├── test_rpc_table/           (overlap detection, binary search, grow)
    │   ├── test_cap_accumulator/     (atomic OR, sequencing)
    │   └── test_ext_register/        (section iteration, priority sort)
    ├── system/                       ← end-to-end, uses socket transport
    │   ├── test_priv_handshake/      (TLV exchange, version negotiation)
    │   ├── test_rpc_roundtrip/       (req→resp for FG V1, MCU V1)
    │   ├── test_events/              (CP→host event delivery)
    │   └── test_cap_bits/            (startup caps reported correctly)
    └── transport/
        └── socket/
            ├── cp_main.c             ← CP side: starts socket transport + core
            └── host_main.c           ← Host side: connects + runs test RPC calls
```

---

## 6. System Test Execution Model

```
[ CI / local machine ]

  Process A: cp_main (IDF Linux target or native build)
    ├─ esp_hosted_cp_core_init()
    ├─ ext_init_task runs → extensions register via EH_CP_EXT_REGISTER
    ├─ socket transport listens on :8765
    └─ responds to RPC requests from host

  Process B: host_main (MCU host, Linux native build)
    ├─ connects to :8765
    ├─ performs PRIV TLV handshake
    ├─ sends RPC requests
    └─ asserts correct responses + capability bits
```

Both processes exit 0 on success. Test runner checks exit codes. stdout logs
are captured for debugging failures.

---

## 7. What the System Tests Cover

| Test | Validates |
|------|-----------|
| PRIV handshake | TLV exchange; V2 header + RPC version negotiated |
| Capability bits | Extensions accumulate caps; host reads them from TLV frame |
| FG V1 RPC round-trip | `CtrlMsg` request → correct `CtrlMsg` response |
| MCU V1 RPC round-trip | `Rpc` request → correct `Rpc` response |
| Event delivery | CP posts event → host receives correct proto payload |
| Unknown RPC | CP returns `Resp_Base` sentinel; host handles gracefully |
| Extension priority | Init order logged; `_rpc_` extensions run before `_feat_` |

---

## 8. Why Not Pure Unity Mocks?

Unity mocks can test individual functions in isolation but they cannot test:
- The PRIV TLV byte sequence over a real framed transport
- Version negotiation (`rpc_ver_negotiated` set correctly by both peers)
- Proto pack → wire → unpack round-trip fidelity
- Capability accumulation timing relative to `generate_startup_event()`

The socket transport adds these integration guarantees without needing hardware.
Unit tests and system tests are complementary — both are needed.
