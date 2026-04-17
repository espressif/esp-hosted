<!-- %% sp.tr.nv-td.o %% - context -->
---
type: spec
status: unverified
last_verified: never
---

# Transport Design

> Unverified — socket transport and logging controls not yet fully implemented.

<!-- %% sp.tr.nv-td.sock.o %% - context -->
## Socket Transport (Test/Dev)

Purpose: run CP and host on same Linux machine without physical hardware.

| Side | Role |
|------|------|
| CP (`eh_transport_cp_socket.c`) | TCP server, port configurable |
| Host (`eh_host_transport_socket.c`) | TCP client |

Same frame format as SPI/SDIO. Enables system-test without hardware. Not for production.

Directory layout (planned):
```
fg/cp/test/
  socket_transport_test.c   ← system test harness
  eh_common_stub.c  ← stubs for non-transport symbols
```
<!-- %% sp.tr.nv-td.sock.c %% -->

<!-- %% sp.tr.nv-td.log.o %% - context -->
## Transport Logging Controls (Planned)

| Kconfig | Default | Effect |
|---------|---------|--------|
| `ESP_HOSTED_TRANSPORT_DEBUG_LOGS` | off | Enable verbose frame-level logs |
| `ESP_HOSTED_TRANSPORT_BUS_LOG_LEVEL` | WARN | Per-bus log verbosity |

Checksum errors: always logged at ERROR regardless of level.
<!-- %% sp.tr.nv-td.log.c %% -->

<!-- %% sp.tr.nv-td.c %% -->
