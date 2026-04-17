# ESP-Hosted Test Infrastructure

## Structure

```
tests/
  ├── env.json.example      ← Config template (committed)
  ├── env.json              ← Your local config (gitignored)
  │
  ├── infra/                ← Shared test infrastructure
  │   ├── config.py         ← Load env.json, merge with matrix
  │   ├── hardware.py       ← Device probe (port + chip detection)
  │   ├── flasher.py        ← Flash orchestration (P4 boot → C6 → P4)
  │   ├── builder.py        ← Build + managed component patching
  │   └── reporter.py       ← JSON + terminal report generation
  │
  ├── hw/                   ← On-target tests (pytest-embedded, real HW)
  │   ├── test_matrix.json  ← Test pair definitions
  │   ├── run_all.py        ← Matrix runner
  │   ├── conftest.py       ← Shared fixtures
  │   ├── pytest.ini        ← Test markers
  │   └── test_*.py         ← 9 test files, 46 test cases
  │
  ├── system/               ← System tests (Linux target, socket) — Phase T2
  └── unit/                 ← Unit tests (pure logic, no IDF) — Phase T3
```

## First Time Setup

```bash
# 1. Source IDF
. ~/esp-idf/export.sh

# 2. Install pytest-embedded
pip install pytest-embedded pytest-embedded-serial pytest-embedded-serial-esp pytest-embedded-idf

# 3. Configure your environment
cp tests/env.json.example tests/env.json
# Edit env.json: set serial ports, WiFi credentials, enable/disable suites
```

## Configuration (env.yaml)

User-local config — never committed to repo.

```json
{
  "hardware": {
    "host_port": "/dev/cu.usbserial-120",
    "slave_port": "/dev/cu.usbserial-1301",
    "board": "p4_c6_core_board"
  },
  "wifi_ap": {
    "ssid": "YourAP",
    "password": "YourPassword"
  },
  "suites": {
    "boot": true,
    "wifi_connect": true,
    "gpio": true,
    "bt": true,
    "ota": true
  }
}
```

Tests with unmet requirements are automatically skipped.

## Running Tests

```bash
cd tests/hw

# List available test pairs
python run_all.py --list

# Run all enabled suites (auto-detect hardware)
python run_all.py --skip-build

# Run specific pair
python run_all.py --pair boot_wifi,gpio --skip-build

# Require hardware (fail if not connected)
python run_all.py --mode on-target --skip-build

# Dry run (show plan)
python run_all.py --dry-run

# Build + flash + test (full cycle)
python run_all.py

# Generate JSON report
python run_all.py --skip-build --json results.json
```

## Modes

| Mode | Flag | Behavior |
|------|------|----------|
| Auto | `--mode auto` (default) | Probe HW → on-target if found, simulated if not |
| On-target | `--mode on-target` | Require hardware, fail if not connected |
| Simulated | `--mode simulated` | No hardware, RPC stub tests (Phase T2) |

## Test Levels

| Level | Directory | What it tests | Hardware | Speed |
|-------|-----------|--------------|----------|-------|
| **On-target** | `hw/` | Full stack on real chips | P4+C6 | ~30s/pair |
| **System** | `system/` | RPC flow via socket (Phase T2) | None | ~5s |
| **Unit** | `unit/` | Individual functions (Phase T3) | None | <1s |

## Adding a New Test Pair

1. Add pair to `hw/test_matrix.yaml`
2. Create `hw/test_<name>.py` with `dut[0]` (host) / `dut[1]` (CP)
3. Use `host.expect('pattern', timeout=N)` for assertions
4. Add suite toggle to `env.json.example`
5. Run: `python run_all.py --pair <name>`
