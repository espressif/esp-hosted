# Test Infrastructure Architecture

## Status
- pytest-embedded working, 9 test files, 46 test cases, all pass on real HW
- Manual flash per test pair — needs automation
- No hardware probe — fails confusingly if devices not connected
- No simulated mode — everything requires real HW
- `agent_dev_utils` evaluated — single-DUT only, not suitable for orchestration

## `agent_dev_utils` Decision
- **Single-DUT only** — cannot orchestrate dual-DUT flash/test
- **Useful for**: individual device crash analysis, coredump-to-JSON, `idf.py agent-info`
- **Not used for**: test orchestration, flash management, dual-DUT correlation
- **Action**: Add as optional dep for crash debugging; build our own dual-DUT orchestration

## Design Principles
1. **Hardware probe first** — detect if devices connected before attempting anything
2. **Mode selection** — `real` (hardware) vs `stub` (simulated RPC, Linux target)
3. **Declarative matrix** — YAML defines test pairs, HW config, dependencies
4. **Robust flash** — all esptool direct (not idf.py flash), handles P4 boot mode
5. **Structured output** — JSON results + JUnit XML for CI
6. **Fail fast** — clear error messages when HW not available
7. **Reusable** — same framework for any host↔CP pair, any transport, any board

## Architecture

```
tests/
  ├── conftest.py              ← Shared fixtures, path helpers
  ├── pytest.ini               ← Markers, default settings
  ├── test_matrix.yaml         ← Declarative: pairs + HW config + dependencies
  │
  ├── infra/                   ← Test infrastructure library
  │   ├── __init__.py
  │   ├── hardware.py          ← HW probe: port exists? device responds? chip match?
  │   ├── flasher.py           ← Flash orchestration: P4 boot → C6 → pre-flash → P4
  │   ├── builder.py           ← Build: idf.py build + managed component patches
  │   └── reporter.py          ← JSON + JUnit report generation
  │
  ├── hw/                      ← Hardware integration tests (dual-DUT)
  │   ├── test_boot_handshake.py
  │   ├── test_gpio.py
  │   ├── test_peer_data.py
  │   ├── test_mem_monitor.py
  │   ├── test_bt.py
  │   ├── test_bt_mac.py
  │   ├── test_wifi_connect.py
  │   ├── test_heartbeat.py
  │   └── test_ota.py
  │
  ├── stub/                    ← RPC simulated tests (no HW, Linux target) — Phase T2
  │
  └── run_all.py               ← Matrix runner: probe → build → flash → test → report

```

## Hardware Probe

```python
# infra/hardware.py
def probe_device(port, expected_chip=None, timeout=5):
    """
    Returns: {
        'connected': bool,
        'chip': str or None,     # 'ESP32-P4', 'ESP32-C6'
        'port': str,
        'error': str or None
    }
    """
    # 1. os.path.exists(port)
    # 2. esptool --port <port> chip_id (with timeout)
    # 3. Compare chip vs expected_chip

def probe_setup(config):
    """Probe both devices. Return go/no-go with details."""
    host = probe_device(config.host_port, config.host_chip)
    slave = probe_device(config.slave_port, config.slave_chip)
    if not host['connected']:
        return False, f"Host not found on {config.host_port}: {host['error']}"
    if not slave['connected']:
        return False, f"Slave not found on {config.slave_port}: {slave['error']}"
    return True, f"Host: {host['chip']} | Slave: {slave['chip']}"
```

## Flash Orchestration

```python
# infra/flasher.py — ALL operations use esptool directly
def flash_pair(host_port, slave_port, cp_build, host_build,
               host_chip, slave_chip, pre_flash=None):
    """
    1. esptool --port <host> run --after no-reset     (P4 bootloader mode)
    2. esptool --port <slave> write_flash @flash_args  (flash C6)
    3. Execute pre_flash actions (e.g., OTA binary)
    4. esptool --port <host> write_flash --force @flash_args  (flash P4)
    """
```

## Mode Selection

```
--mode on-target     Hardware required, fail if not found
--mode simulated     No hardware, run stub/unit tests only (Linux target)
--mode auto     Probe HW → real if found, stub if not
--dry-run       Show what would happen, don't execute
```

## Run Flow

```
run_all.py --pair boot_wifi,gpio --mode on-target --junit results/
  │
  ├─ Load test_matrix.yaml
  ├─ Probe hardware → go/no-go
  ├─ For each pair:
  │   ├─ Build CP (idf.py set-target + build)
  │   ├─ Build Host (idf.py set-target + build)
  │   ├─ Patch managed component (assert fix)
  │   ├─ Rebuild Host (if patched)
  │   ├─ Flash pair (infra/flasher.py)
  │   ├─ Run pytest-embedded (--skip-autoflash)
  │   └─ Collect pass/fail/crash
  ├─ Generate report
  └─ Exit code: 0 if all pass, 1 if any fail
```

## Open Questions
1. `infra/` as local imports or pip-installable?
2. Support multiple board configs (core board, function EV)?
3. Crash coredump: decode inline or save for later?
4. Parallel pair execution possible? (probably not — shared serial ports)
