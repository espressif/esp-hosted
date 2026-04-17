<!-- %% sp.sy.ve-im.o %% - always -->
---
type: spec
last_verified: 2026-04-07
---

# Implementation Status

<!-- %% sp.sy.ve-im.phases.o %% - always -->
## Phase Completion

| Phase | Name | Status |
|-------|------|--------|
| 0 | Directory skeleton | ✅ done |
| A | Common headers shim | ✅ done |
| B | Transport CP | ✅ done |
| C | Core registries 1–2 | ✅ done |
| D | Registry 3 realloc-table | ✅ done |
| E | Extension auto-init (linker section) | ✅ done |
| F | V2 proto end-to-end | ⏸ deferred (post-V1) |
| G/H | Protocomm removal + ext_rpc refactor | ⏸ deferred (post-V1) |
| I | Backward compat Kconfig modes | ⏸ deferred (with Header V2) |
| P0 | Build gate, extension registry, cap bits | ✅ done |
| P0 | Macro migration (IDF compat, system, BT) | ✅ done |
| P0 | WiFi/BT/Enterprise/ITWT/DPP extensions | ✅ done |
| P1-R | Full rename (R1-R15) | ✅ done |
| P1d | Utilities (malloc, gpio pin guard, stats, fw ver) | ✅ done |
| P1e | Feature ports (GPIO exp, mem monitor, ext coex, light sleep) | ✅ done |
| P1f | Handler fixes + verification | ✅ done |
| P1g | Dispatcher wiring + handler port | ✅ done |
| P1h | Architectural cleanup | 🔲 partial (caps guard done, shim pending delete) |
| P1-T | Test infrastructure | ✅ done |
| P2 | Pre-release cleanup | 🔲 in progress |
<!-- %% sp.sy.ve-im.phases.c %% -->

<!-- %% sp.sy.ve-im.pending.o %% - always -->
## Open PENDING Items

| ID | Description | Blocks |
|----|-------------|--------|
| PENDING-005 | V2 header TX/RX path | Phase F |
| PENDING-006 | V2 unified proto schema | Phase F |
| PENDING-013 | Two-stage TLV for V2 handshake | Phase I |

> V1 release does not require any PENDING items to be resolved.
<!-- %% sp.sy.ve-im.pending.c %% -->

<!-- %% sp.sy.ve-im.naming.o %% - always -->
## Naming Convention (P1-R Complete)

All coprocessor code follows `eh_cp_*` naming:

| Component | Convention | Example |
|-----------|-----------|---------|
| Core | `eh_cp_core/`, `eh_cp_transport/` | `eh_cp_init()` |
| Features | `eh_cp_feat_*/` | `eh_cp_feat_wifi/`, `eh_cp_feat_bt/` |
| Sub-extensions | `eh_cp_feat_*_ext_*/` | `eh_cp_feat_wifi_ext_ent/` |
| RPC | `eh_cp_feat_rpc/`, `eh_cp_feat_rpc_ext_mcu/` | |
| Kconfig | `ESP_HOSTED_CP_FEAT_*` | `ESP_HOSTED_CP_FEAT_WIFI` |
| Macros | `EH_CP_FEAT_*` | `EH_CP_FEAT_WIFI_READY` |
| Events | `EH_CP_FEAT_*_EVENT` | `EH_CP_FEAT_WIFI_EVENT` |
| Linker | `.eh_cp_feat_descs` | `EH_CP_FEAT_REGISTER` |
<!-- %% sp.sy.ve-im.naming.c %% -->

<!-- %% sp.sy.ve-im.ext.o %% - context -->
## Feature Registry

| Feature | Directory | Status |
|---------|-----------|--------|
| WiFi | `eh_cp_feat_wifi/` | ✅ done |
| BT/BLE | `eh_cp_feat_bt/` | ✅ done |
| WiFi Enterprise | `eh_cp_feat_wifi_ext_ent/` | ✅ done (sub-ext of wifi) |
| WiFi iTWT | `eh_cp_feat_wifi_ext_itwt/` | ✅ done (sub-ext of wifi) |
| WiFi DPP | `eh_cp_feat_wifi_ext_dpp/` | ✅ done (sub-ext of wifi) |
| System | `eh_cp_feat_system/` | ✅ done |
| Network Split | `eh_cp_feat_nw_split/` | ✅ done |
| Host Power Save | `eh_cp_feat_host_ps/` | ✅ done |
| CLI | `eh_cp_feat_cli/` | ✅ done |
| RPC (MCU) | `eh_cp_feat_rpc_ext_mcu/` | ✅ done |
| RPC (Linux FG) | `eh_cp_feat_rpc_ext_linux/` | ✅ done |
| GPIO Expander | `eh_cp_feat_gpio_exp/` | ✅ done |
| Memory Monitor | `eh_cp_feat_mem_monitor/` | ✅ done |
| External Coex | `eh_cp_feat_ext_coex/` | ✅ done |
| Light Sleep | `eh_cp_feat_light_sleep/` | ✅ done |
| Peer Data Transfer | `eh_cp_feat_peer_data/` | ✅ done |
<!-- %% sp.sy.ve-im.ext.c %% -->

<!-- %% sp.sy.ve-im.test.o %% - context -->
## Test Infrastructure (P1-T)

| Component | Status |
|-----------|--------|
| pytest-embedded dual-DUT (serial service) | ✅ done |
| Test infra library (`tests/infra/`) | ✅ done |
| Matrix runner (`eh_test_runner.py`) | ✅ done |
| Workspace isolation (`tests/workspace/`) | ✅ done |
| Binary hash flash cache | ✅ done |
| App-only flash optimization | ✅ done |
| ANSI colored terminal output | ✅ done |
| Artifact management + log retention | ✅ done |
| idf-ci plugin integration | ✅ done |
| Simulated tests (Linux target) | 🔲 Phase T2 |

**Test pairs validated on hardware:**

| Pair | CP Example | Host Example | Status |
|------|-----------|--------------|--------|
| boot_wifi | minimal/wifi | host_minimal_test | ✅ pass |
| gpio | extensions/gpio_exp | host_gpio_expander | ✅ pass |
| peer_data | extensions/peer_data_transfer | host_peer_data_transfer | 🔲 needs full run |
| mem_monitor | extensions/mem_monitor | host_hosted_cp_meminfo | 🔲 needs full run |
| bt_nimble | minimal/bt | host_nimble_bleprph_host_only_vhci | 🔲 needs full run |
| bt_mac | minimal/bt | host_bt_controller_mac_addr | 🔲 needs full run |
| wifi_connect | minimal/wifi | host_hosted_events | 🔲 needs wifi_ap |
| ota | minimal/wifi | host_performs_slave_ota | 🔲 needs full run |
| host_ps | extensions/host_power_save | host_shuts_down_slave_to_power_save | 🔲 needs wifi_ap |
| nw_split | extensions/network_split/station | host_network_split__power_save | 🔲 needs wifi_ap |
| nw_split_ps | extensions/network_split__host_power_save | host_network_split__power_save | 🔲 needs wifi_ap |
| light_sleep | extensions/light_sleep | host_network_split__power_save | 🔲 needs wifi_ap |
<!-- %% sp.sy.ve-im.test.c %% -->

<!-- %% sp.sy.ve-im.remaining.o %% - always -->
## Remaining for V1

| Task | Priority | Notes |
|------|----------|-------|
| `fg/common.del` delete | P1h | Shims unused, directory marked for deletion |
| Cap bits architecture spec | P1h | Document three-tier system |
| Per-feature auto-init Kconfig | P1h | Individual feature opt-out from auto-init |
| Mempool port-layer abstraction | deferred | Needed for Linux host |
| Simulated tests | P1-T2 | RPC mock tests on Linux target |
| Full test suite validation | P2 | Run all 12 pairs on hardware |
<!-- %% sp.sy.ve-im.remaining.c %% -->

<!-- %% sp.sy.ve-im.c %% -->
