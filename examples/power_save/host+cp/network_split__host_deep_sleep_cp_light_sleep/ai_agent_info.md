# ai_agent_info — engineering reference

Deeper technical reference for this example, for integrators, maintainers, and
their AI agents. Complements the user docs (`readme.md`, `power_measurements.md`,
`esp_host/readme_*.md`). Keep it factual; **never put real SSIDs/passwords here**
— use `<SSID>` / `<PW>` placeholders.

## Overview

Host (ESP32-P4) in deep sleep + coprocessor (ESP32-C5) in light sleep,
network-split. The CP holds the Wi-Fi association while the host is deep-asleep
and wakes it via a GPIO pulse on host-bound traffic. Host deep-sleep is a full
host reboot; on wake the host re-establishes the transport and re-associates
through the still-associated CP.

## Configuration options

- **CP light-sleep power mode** — `ESP_HOSTED_CP_LIGHT_SLEEP_POWER` choice:
  - `DISABLED` — no PM / tickless; the example runs always-on ("power save off").
  - `MIN` — development; selects `PM_ENABLE` + `FREERTOS_USE_TICKLESS_IDLE`.
  - `MAX` — production; also selects `..._UNLOAD_BUS_WHILE_SLEEPING` and
    peripheral power-down.
  MIN/MAX `select PM_ENABLE` + tickless, so PM cannot be turned off unless you
  pick `DISABLED`.
- **`UNLOAD_BUS_WHILE_SLEEPING`** — must be `y` or light sleep never really
  engages (the bus keeps the CP awake).
- **Host-wake GPIO** — mandatory for host-PS deep-sleep; the build fails if it is
  left unset (no silent default pin).
- **Wi-Fi band** — default `AUTO` scans both bands, which makes a *cold* associate
  slow (~28 s). Pin the band (`band 2g`) to speed a cold connect. Not relevant to
  wake, where the CP already holds the association.
- **Wi-Fi power-save mode / listen interval** — `WIFI_PS_MAX_MODEM` wakes every N
  listen intervals; `WIFI_PS_MIN_MODEM` wakes on the AP's DTIM (listen interval
  does not apply).

## Measurements

Bench: P4+C5 core board, JS110 Joulescope on the C5 rail, 2.4 GHz AP. Parked
current is the Joulescope mean (µ) over the light-sleep window.

**Modem power-save current** (graphs in `power_measurements.md`):

| power-save mode     | listen interval | 30 s marker | full run (≥30 min) |
|---------------------|:---------------:|:-----------:|:------------------:|
| `WIFI_PS_MAX_MODEM` | 3               | 1.95 mA     | 2.06 mA            |
| `WIFI_PS_MAX_MODEM` | 6               | 1.13 mA     | 1.02 mA            |
| `WIFI_PS_MAX_MODEM` | 10              | 1.02 mA     | 1.08 mA            |
| `WIFI_PS_MIN_MODEM` | n/a (DTIM)      | 3.87 mA     | 3.00 mA            |

**Light-sleep across transports** (50 sleep/wake cycles, 30 s dwell): parked
median spi_fd 2.17 / spi_hd 2.27 / uart 3.01 / sdio 3.26 mA; deep floor ~1.3 mA,
uniform across transports.

**Wake → IP time** (host-PS wake, CP held the association):

| transport | wake → IP | cold fresh boot → IP |
|-----------|:---------:|:--------------------:|
| spi_fd    | ~0.55 s   | ~28 s                |
| spi_hd    | ~0.73 s   | ~28 s                |
| uart      | ~0.7 s    | ~29 s                |
| sdio      | ~0.6 s    | ~28 s                |

Wake is sub-second because the CP keeps the association; the host just
re-establishes the transport. SDIO's wake time is dominated by SD-card
enumeration (a protocol handshake), not the datapath. Cold boot → IP is
dominated by the AUTO dual-band scan, not the transport.

**Throughput** — hosted-SDIO vs native-C5 IDF `wifi/power_save`, 2.4 GHz (Mbps):

| test   | hosted SDIO | native C5 |
|--------|:-----------:|:---------:|
| TCP TX | 14.1        | 26.7      |
| TCP RX | 32.9        | 39.0      |
| UDP TX | 57 (sender) | 58        |
| UDP RX | 57.0        | 59.2      |

Hosted overhead is concentrated in TCP uplink (~half of native); downlink and
UDP are near-native.

## Reproduce

Prerequisites: P4+C5 core board wired for the transport (+ reset + host-wake
GPIO); the measuring machine on the **same L2** as the DUT (the wake packet must
arrive); ESP-IDF v6.0.2; for `--power` a JS110 + the `joulescope` package. The
test venv is separate from IDF (sourcing IDF's export repoints `python3`, so call
the venv interpreter by path).

Power/wake soak (build + flash both + bring up + N cycles + measure):

```bash
. ~/esp-idf/export.sh
<venv>/bin/python3 test/hw_ps_soak.py \
    --transports sdio --cycles 50 --dwell 30 --power \
    --ssid <SSID> --password <PW> \
    --host-port /dev/<P4_PORT> --cp-port /dev/<C5_PORT>
```

By hand: build/flash cp then esp_host, associate from the host console
(`sta <SSID> <PW>`), trigger `host_power_save`, confirm the CP logs
`Light sleep ENABLED`, wake with a packet to the host IP (`nc <HOST_IP> 22`).

Emulator (no bench): `EH_PS_WAKE_CYCLES=<N≥8> pytest
tests/emu-mcu/eh_test_feat_power_save/test_power_save_wake.py -k sdio
--regression --timeout=3000`.

Throughput: `test/hw_iperf.py` drives the iperf-cmd console on the DUT against a
bound peer. The `-b` unit differs by iperf-cmd version (older wants a bare
integer Mbits, newer wants `100M`); `--dut-udp-bw` handles it. On a dual-homed
measuring machine pass `--bind <its-iface-IP>` so traffic uses the interface that
reaches the DUT.

## Operational notes

- **P4 download-boot latch**: escape with `esptool --after watchdog-reset`, not a
  hard reset. Park the P4 in download with `--before default-reset --after
  no-reset` (e.g. to flash the C5 alone).
- **Transport mismatch** (CP flashed for one bus, host for another) makes
  `esp_wifi_init` (an RPC to the CP) fail and the host boot-loops — always
  rebuild BOTH sides for the same transport.
- **`reason=201` / rssi −128** on associate = AP not found / out of range (not a
  bug) — check the AP/band is up.
- **Joulescope**: only one process may claim a JS110 — close the desktop app
  before a `--power` run.
- Two IDF python envs may coexist (e.g. py3.10 / py3.14); a project configured
  with one and flashed by `idf.py` under the other aborts — flash via `esptool`
  directly with the matching env, or `idf.py fullclean`.

## Test tooling (`test/`)

- `hw_ps_soak.py` — build/flash/cycle/measure soak (SSID/PW are required args, no
  defaults).
- `hw_iperf.py` — iperf matrix over the DUT console (TCP/UDP × TX/RX per AP).
- `hw_power_measure.py` / `bench_book.py` — single power run + CSV bookkeeping
  (`bench_measurements.csv`).
