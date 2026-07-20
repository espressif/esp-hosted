<!-- SPDX-License-Identifier: Apache-2.0 -->

# API exerciser

<!-- common-start -->
A generic host+CP example that exposes the native `eh_host_*` API surface as
console commands, so functional API coverage is added as **data** (a console
command + an expected line) rather than a new example per scenario.

## Supported Platforms and Transports

### Supported Coprocessors

| Coprocessor | ESP32 | ESP32-C Series | ESP32-S Series |
| :----------: | :---: | :------------: | :------------: |
| Support     | Yes   | Yes            | Yes            |

### Supported Host Devices

| Host Device | ESP32-P4 | ESP32-H2 | Other MCUs | Linux |
| :---------: | :------: | :------: | :--------: | :---: |
| Support     | Yes | Yes | [Yes](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-mcu.md) | [Yes](https://github.com/espressif/esp-hosted/blob/master/docs/getting-started-linux.md) |

### Supported Connection buses

| Connection bus | SDIO | SPI Full-Duplex | SPI Half-Duplex | UART |
| :------------- | :--: | :-------------: | :-------------: | :--: |
| Linux host     | Yes  | Yes             | No              | No   |
| MCU host       | Yes  | Yes             | Yes             | Yes  |
<!-- common-stop -->

## Directory layout

```text
system/api_exerciser/
├── cp/                  ESP coprocessor firmware
├── mcu_host/            ESP-IDF MCU host app
└── linux_802_3_host/    Linux host
     └── c_app/          native C app
```

<!-- esp_host-start -->
## The result contract

Every command prints exactly one newline-terminated line:

```
EH rc=<int> cmd=<name> [k=v ...]
```

- `rc=0` on success; a non-zero `rc` (with `err=<name>`) on failure.
- getters add fields (e.g. `wifi_get_ps` → `EH rc=0 cmd=wifi_get_ps ps=2`).
- Round-trips (set → get → compare) and negative/arg-validation checks are
  expressed entirely in the test, not in firmware.

Type `help` at the `eh>` prompt for the full command list.
<!-- esp_host-stop -->

## Run

```
# emulator (P4 host ⇄ C6 CP), via the pytest bench
python3 tools/eh.py test emu-mcu -k api_exerciser
```

## Adding coverage

Add a row to `mcu_host/main/eh_api_cmd.c`'s command table (one thin
`argv → eh_host_* call → eh_out(...)` wrapper), then a pytest data file under
`tests/emu-mcu/eh_test_feat_api/`. No emulator change is needed for the
scalar/string API surface (SCENARIO_BACKLOG bucket **B2**).
