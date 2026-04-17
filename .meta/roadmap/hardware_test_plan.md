# Hardware Test Plan

## Status: 46/46 PASS (2026-04-05)

## Setup
- Host: ESP32-P4 at /dev/cu.usbserial-120 (esp_hosted_mcu6 examples)
- CP: ESP32-C6 at /dev/cu.usbserial-1301 (esp_hosted.new CP examples)
- Transport: SDIO, P4-C6 Core Board
- Framework: pytest-embedded 2.7.0 dual-DUT

## Test Pairs (all verified)

| Pair | CP | Host | Tests | Verifies |
|------|-----|------|-------|----------|
| boot_wifi | minimal/wifi | host_minimal_test | 7 | Handshake, FW ver, WiFi scan |
| gpio | extensions/gpio_exp | host_gpio_expander | 3 | GPIO toggle, read, pull-up |
| peer_data | extensions/peer_data_transfer | host_peer_data_transfer | 5 | Echo 3 sizes + GHOST |
| mem_monitor | extensions/mem_monitor | host_hosted_cp_meminfo | 5 | Heap stats, monitor config |
| bt_nimble | minimal/bt | host_nimble_bleprph_host_only_vhci | 5 | BLE advertising, GAP |
| bt_mac | minimal/bt | host_bt_controller_mac_addr | 2 | BT MAC query |
| wifi_connect | minimal/wifi | host_hosted_events | 7 | WiFi connect, GOT_IP, events |
| ota | minimal/wifi | host_performs_slave_ota | 6 | 992KB OTA, activate, reboot |

## Not Tested (needs external HW)

| Feature | Reason |
|---------|--------|
| WiFi Enterprise | Needs RADIUS server |
| WiFi DPP | Needs DPP configurator |
| WiFi iTWT | Needs 802.11ax AP |
| Power Save | Needs function EV board |
| External Coex | Needs coex hardware |
| BT HID | Needs BT peer |
| UART HCI | Different transport |

## Infrastructure

See `tests/README.md` for usage.

- `tests/infra/hardware.py` — probe devices before testing
- `tests/infra/flasher.py` — P4 boot mode → flash C6 → flash P4 sequence
- `tests/infra/builder.py` — build + managed component patching
- `tests/infra/reporter.py` — JSON + terminal reports
- `tests/hw/run_all.py` — matrix runner
- `tests/hw/test_matrix.yaml` — declarative pair definitions

## Host-side Fixes Required

Applied to esp_hosted_mcu6 examples for P4-C6 core board testing:

1. `assert(len < 64)` → `MAX_TRANSPORT_BUFFER_SIZE` in managed component transport_drv.c
2. `CONFIG_ESP_HOSTED_P4_C6_CORE_BOARD=y` in all host sdkconfig.defaults
3. `CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6=y` in all host sdkconfig.defaults
4. `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` for P4 rev 1.3 chip

## Next Steps

- Improve mode naming in run_all.py
- End-to-end test of run_all.py on hardware
- Simulated RPC stub tests (Phase T2 — Linux target)
- CI integration (GitHub Actions with self-hosted runner)
