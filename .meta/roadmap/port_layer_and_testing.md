# Roadmap: Port Layer, Socket Transport & Testing

> Status: DESIGN — questions answered, hierarchy decided
> Created: 2026-04-02

## Goal

Platform abstraction enabling socket transport, unit/system testing, CI/CD,
and future multi-platform host support (IDF, STM32, Linux).

## Design Principles

- **Zero overhead on hardware**: `static inline` + compile-time dispatch
- **Transport**: vtable acceptable (I/O-bound, ~5 cycle indirection)
- **Structured not monolithic**: OSAL + HAL as separate components
- **Reuse posix_port patterns**: opaque structs, timeout standardization, validity flags
- **IDF-native testing**: CMock, Unity, pytest-embedded, IDF Linux target
- **AI-centric**: `igrr/agent_dev_utils` for structured crash/test output
- **CI-agnostic**: test infra independent; GitHub/GitLab hooks separate

## Directory Hierarchy (aligned with original plan)

```
port/
  ├── cp/                                    ← Coprocessor port
  │   ├── interface/                         ← Abstract headers (contracts)
  │   │   ├── eh_cp_port_osal.h             ← OS: mutex, sem, task, timer, event_group
  │   │   ├── eh_cp_port_mem.h              ← Memory: malloc, calloc, free, caps
  │   │   └── eh_cp_port_hal.h              ← HW: bus init, GPIO (if needed)
  │   ├── idf/                               ← IDF/FreeRTOS implementation
  │   │   ├── eh_cp_port_osal_idf/
  │   │   │   ├── CMakeLists.txt
  │   │   │   ├── include/eh_cp_port_osal_idf.h    ← static inline FreeRTOS wrappers
  │   │   │   └── src/eh_cp_port_osal_idf.c        ← non-inlinable functions
  │   │   └── eh_cp_port_mem_idf/
  │   │       ├── CMakeLists.txt
  │   │       ├── include/eh_cp_port_mem_idf.h
  │   │       └── src/eh_cp_port_mem_idf.c
  │   ├── posix/                             ← Linux/test implementation
  │   │   ├── eh_cp_port_osal_posix/         ← Reuse patterns from esp-rainmaker posix_port
  │   │   └── eh_cp_port_mem_posix/
  │   └── mock/                              ← CMock stubs for WiFi/BT HAL
  │       └── eh_cp_port_hal_mock/
  │
  └── host/                                  ← Host port (existing plan)
      ├── interface/
      │   ├── esp_hosted_host_port_osal.h
      │   └── esp_hosted_host_port_hal.h
      ├── idf/
      ├── stm32/
      └── linux/
```

### Key decisions:
- **CP and Host have separate port dirs** — different abstraction needs
- **Interface headers define contracts** — implementations swap at build time
- **OSAL and HAL are separate components** — not monolithic
- **Mock dir** for test-time WiFi/BT stubs (CMock-generated)
- **`master_config.h` selects port**: `EH_CP_PORT_FREERTOS` or `EH_CP_PORT_POSIX`

### Reuse from posix_port:
- Opaque struct pattern for all OS primitives
- Timeout standardization (ms, MAX_DELAY=0xFFFFFFFF, 0=try)
- Validity flags for safe deletion
- Timespec normalization helper
- Metadata tracking (optional, compile-time)
- Circular buffer queue implementation

## Transport

- **vtable pattern** (single indirection, I/O-bound): `eh_transport_t` with send/recv/open/close
- **Socket transport**: pure transport impl alongside SPI/SDIO/UART/SPI-HD
- CP runs on Linux target → socket transport → host simulator on same machine
- Also useful for dev/debug (not test-only)

## Testing

### Unit tests
- `test/` inside each component (IDF convention)
- Linux target (`CONFIG_IDF_TARGET=linux`) + CMock for IDF API mocks
- Unity assertions
- Granular invocation: per-component, per-test, or full suite

### System tests
- Centralized test controller at `tests/` repo root
- Inherits same Kconfig as production code
- Controls unit + system test invocation
- AI-centric: structured JSON output via `igrr/agent_dev_utils`

### Hardware tests
- pytest-embedded on real hardware
- Dedicated PCBs with known configurations
- Transport loopback (CP ↔ host)
- Log capture and assertion

## CI/CD

### Infrastructure (CI-agnostic)
```
tools/ci/
  ├── build_matrix.yml          ← Target × example × transport matrix
  ├── run_unit_tests.sh         ← Linux target + Unity
  ├── run_system_tests.sh       ← pytest-embedded
  └── trigger.sh                ← Minimal-input entry point
```

### Hooks (platform-specific)
```
.github/workflows/
  ├── pr_check.yml              ← Build + unit tests on PR
  └── merge_test.yml            ← Hardware tests on merge to main

.gitlab-ci.yml                  ← Mirror if needed
```

### Matrix
- Build: {esp32c5, esp32c6, esp32s3} × {mcu_wifi, mcu_bt, fg_wifi, fg_bt}
- Unit: Linux target (fast, every PR)
- Hardware: self-hosted runner with dedicated PCBs (merge to main)

### agent_dev_utils integration
- `idf.py run-project` for structured test execution
- JSON crash output for AI-driven debugging
- `idf.py agent-info` for AI agent instructions

## Phases

| Phase | Name | Deliverable | Depends on |
|-------|------|-------------|------------|
| T1 | Port headers + IDF impl | `port/cp/interface/` + `port/cp/idf/` — zero-overhead wrappers, objdump verified | None |
| T2 | Socket transport + Linux target | `eh_transport_socket.c` + POSIX port — CP runs on Linux | T1 |
| T3 | Test framework | CMock stubs + Unity tests per component, `test/` dirs | T2 |
| T4 | CI pipeline | GitHub Actions workflow, build matrix, unit tests on PR | T3 |
| T5 | Hardware tests | pytest-embedded + agent_dev_utils, self-hosted runner | T4 |

**Recommended order**: T1 → T2 → T3 → T4 → T5 (sequential, each builds on previous)

## Open items (for brainstorm)

- CP port: should `eh_cp_port_osal.h` be a dispatch header (`#include` platform impl)
  or should CMake select the component? (CMake selection = cleaner, no ifdefs in header)
- How much of posix_port to fork vs rewrite? (rewrite with same patterns, ESP-Hosted naming)
- Socket transport: TCP or Unix domain socket? (TCP = remote-capable, UDS = simpler)
- agent_dev_utils: add as managed component or vendored?

## Host macro migration (`host/unverified/macro_migration_map_host.md`)
**Port layer + testing** (see `.meta/roadmap/port_layer_and_testing.md`)
- T1: Port headers + FreeRTOS impl (zero-overhead wrappers)
- T2: Socket transport + Linux target
- T3: CMock stubs + Unity test framework
- T4: CI pipeline (GitHub/GitLab Actions)
- T5: Integration tests (pytest-embedded + real hardware)

# For pytest based verifications,

pip3 install --user pytest-embedded pytest-embedded-serial pytest-embedded-serial-esp pytest-embedded-idf
