<!-- %% sp.ta-to.o %% - always -->
# Active Todo

> Checked = done. Unchecked = pending.
> **Handoff protocol**: New session → load `AGENT_CONTEXT.md` → read this file → load specs listed under the first unchecked task → read source files listed under that task → implement.
> Add future/deferred items to bottom under "Future" — do not implement without scope.md approval.

<!-- %% sp.ta-to.v1.o %% - always -->
## V1 Release — Current Sprint

### P0 — Completed

- [x] Build gate, extension registry, capability bits API
- [x] Macro migration (IDF compat, system compat, BT derived)
- [x] WiFi, BT, WiFi Enterprise, WiFi ITWT, WiFi DPP extensions

### P1-R — Full Rename + Feature Restructuring

**Scope**: rename ALL `esp_hosted_cp` → `eh_cp` across entire coprocessor codebase.
This includes core, transport, features, common — everything under `modules/`.
Only `fg/` example directory structure stays. IDF component name stays `esp_hosted`.
No sdkconfig.rename — not public, no backward compat.

**New naming convention**:
- Core: `eh_cp_core/`, `eh_cp_transport/`
- Common: `eh_common/` → `eh_common/`, `esp_hosted_frame/` → `eh_frame/`, `esp_hosted_mempool/` → `eh_mempool/`
- Features: `eh_cp_feat_wifi/`, `eh_cp_feat_bt/`, `eh_cp_feat_system/`
- Sub-extensions: `eh_cp_feat_wifi_ext_ent/`, `eh_cp_feat_wifi_ext_itwt/`, `eh_cp_feat_wifi_ext_dpp/`
- RPC: `eh_cp_feat_rpc/`, `eh_cp_feat_rpc_ext_mcu/`, `eh_cp_feat_rpc_ext_linux/`
- Macros: `EH_CP_FEAT_*` (features), `EH_CP_FEAT_*_EXT_*` (sub-extensions)
- Public APIs: `eh_cp_init()`, `eh_cp_feat_wifi_init()`, `eh_cp_add_feat_cap_bits()`
- Kconfig: `ESP_HOSTED_CP_FEAT_*` (features), `ESP_HOSTED_CP_FEAT_*_EXT_*` (sub)
- Events: `EH_CP_FEAT_WIFI_EVENT`, `EH_CP_FEAT_WIFI_EVT_*`
- Linker: `.eh_cp_feat_descs`, `EH_CP_FEAT_REGISTER`, `eh_cp_feat_desc_t`

**Execution order**: R1 → R2 → R3-R8 (text replace on new paths) → R9-R11 (structural) → R12-R14 → R15

- [x] **R1: Update specs for new naming**
  - Rename spec files: `feature_scaffold.md` → `feature_scaffold.md`, `feature_system.md` → `feature_system.md`
  - Update tag_registry.conf: `esc` → `fsc`, `es` → `fs`
  - Update ALL verified specs content: component_map, config_macros, events, kconfig_hierarchy, architecture, directory_structure, feature_parity_map, macro_migration_map, design_decisions, linker_auto_init
  - Update AGENT_CONTEXT.md routing table
  - Update CLAUDE.md invariants if needed
  - Acceptance: `check.sh --soft` passes

- [x] **R2: Rename ALL directories + files**
  - **Core**: `eh_cp_core/` → `eh_cp_core/`, all files inside `esp_hosted_cp_*` → `eh_cp_*`
  - **Transport**: `eh_cp_transport/` → `eh_cp_transport/`, all files inside
  - **Features** (13 dirs): see mapping above
  - **LD files**: `esp_hosted_cp_ext_descs.ld` → `eh_cp_feat_descs.ld`, `_insert.ld` same
  - **LF file**: `eh_cp_core.lf` → `eh_cp_core.lf`
  - **Parent LD**: `esp_hosted_cp_descs.ld` → `eh_cp_feat_descs.ld`
  - **Common**: `eh_common/` → `eh_common/`, `esp_hosted_frame/` → `eh_frame/`, `esp_hosted_mempool/` → `eh_mempool/`
  - **Serializers**: `esp_hosted_proto_mcu_v1/` → `eh_proto_mcu_v1/`, `esp_hosted_proto_linux_v1/` → `eh_proto_linux_v1/`
  - Include test/ files in sweep
  - Source files inside each dir: match parent prefix
  - Acceptance: `find modules/ -name "esp_hosted_cp_*" -type f` returns 0 (excluding common wire headers if kept)

- [x] **R3: Bulk text replace — Kconfig symbols**
  - `ESP_HOSTED_CP_FEAT_` → `ESP_HOSTED_CP_FEAT_`
  - `ESP_HOSTED_CP_FEAT_RPC_` → `ESP_HOSTED_CP_FEAT_RPC_`
  - `ESP_HOSTED_CP_FEAT_NW_SPLIT` → `ESP_HOSTED_CP_FEAT_NW_SPLIT`
  - `ESP_HOSTED_CP_FEAT_PEER_DATA` → `ESP_HOSTED_CP_FEAT_PEER_DATA`
  - `ESP_HOSTED_CP_FEAT_HOST_PS` → `ESP_HOSTED_CP_FEAT_HOST_PS`
  - `ESP_HOSTED_CP_FEAT_CLI` → `ESP_HOSTED_CP_FEAT_CLI`
  - Sub-ext Kconfig: `FEAT_WIFI_ENTERPRISE` → `FEAT_WIFI_EXT_ENT`, `FEAT_WIFI_ITWT` → `FEAT_WIFI_EXT_ITWT`, `FEAT_WIFI_DPP` → `FEAT_WIFI_EXT_DPP`
  - Acceptance: `grep -r "EXT_FEAT_\|EXT_RPC_\|EXT_NW_SPLIT\|EXT_PEER_DATA\|EXT_HOST_PS\|EXT_CLI" modules/` returns 0

- [x] **R4: Bulk text replace — EH_CP_* macros**
  - `EH_CP_FEAT_` → `EH_CP_FEAT_`
  - `EH_CP_FEAT_NW_SPLIT` → `EH_CP_FEAT_NW_SPLIT`
  - `EH_CP_FEAT_PEER_DATA` → `EH_CP_FEAT_PEER_DATA`
  - `EH_CP_FEAT_HOST_PS` → `EH_CP_FEAT_HOST_PS`
  - `EH_CP_FEAT_CLI` → `EH_CP_FEAT_CLI`
  - Acceptance: zero `EH_CP_EXT_` in source (except `EH_CP_EXT_` that became `EH_CP_FEAT_`)

- [x] **R5: Bulk text replace — linker section + macro + struct**
  - `.eh_cp_ext_descs` → `.eh_cp_feat_descs` (in .ld, .lf, .c, .h)
  - `_eh_cp_ext_descs_start` → `_eh_cp_feat_descs_start` (and _end)
  - `__eh_cp_ext_descs_start` → `__eh_cp_feat_descs_start` (and _end, __start_, __stop_)
  - `EH_CP_FEAT_REGISTER` → `EH_CP_FEAT_REGISTER`
  - `esp_hosted_ext_desc_t` → `eh_cp_feat_desc_t`
  - `EH_CP_EXT_INIT_DONE_BIT` → `EH_CP_FEAT_INIT_DONE_BIT`
  - Acceptance: `grep -r "ext_descs\|EXT_REGISTER\|EXT_INIT_DONE\|esp_hosted_ext_desc_t" modules/` returns 0

- [x] **R6: Bulk text replace — public function names**
  - `eh_cp_feat_` → `eh_cp_feat_`
  - `eh_cp_feat_rpc_` → `eh_cp_feat_rpc_`
  - `esp_hosted_cp_ext_register_` → `eh_cp_feat_register_` (nw_split evt handlers)
  - `esp_hosted_cp_ext_unregister_` → `eh_cp_feat_unregister_`
  - `esp_hosted_cp_ext_peer_data_` → `eh_cp_feat_peer_data_`
  - `eh_cp_feat_cli_` → `eh_cp_feat_cli_`
  - Core APIs: `esp_hosted_cp_init` → `eh_cp_init`, `esp_hosted_cp_deinit` → `eh_cp_deinit`
  - `esp_hosted_cp_rx_get` → `eh_cp_rx_get`, `esp_hosted_cp_dispatch_rx` → `eh_cp_dispatch_rx`
  - `esp_hosted_cp_register_rx_cb` → `eh_cp_register_rx_cb`
  - Cap APIs: `esp_hosted_cp_add_cap_bits` → `eh_cp_add_feat_cap_bits`
  - `esp_hosted_cp_add_feature_cap_bits` → `eh_cp_add_feat_cap_bits_idx`
  - `esp_hosted_cp_clear_cap_bits` → `eh_cp_clear_feat_cap_bits`
  - `esp_hosted_cp_clear_feature_cap_bits` → `eh_cp_clear_feat_cap_bits_idx`
  - `esp_hosted_cp_rpc_send_event` → `eh_cp_rpc_send_event`
  - Acceptance: `grep -r "esp_hosted_cp_ext_\|esp_hosted_cp_init\|esp_hosted_cp_add_cap" modules/ --include="*.c" --include="*.h"` returns 0

- [x] **R7: Bulk text replace — include guards + event bases + enums**
  - Guards: `ESP_HOSTED_CP_FEAT_*_H` → `EH_CP_FEAT_*_H`
  - Also core guards: `__ESP_HOSTED_CP_*_H__` → `EH_CP_*_H`
  - Event bases: `ESP_HOSTED_CP_EXT_WIFI_EVENT` → `EH_CP_FEAT_WIFI_EVENT`
  - `ESP_HOSTED_CP_EXT_SYSTEM_EVENT` → `EH_CP_FEAT_SYSTEM_EVENT`
  - `ESP_HOSTED_CP_EXT_WIFI_ITWT_EVENT` → `EH_CP_FEAT_WIFI_EXT_ITWT_EVENT`
  - `ESP_HOSTED_CP_FEAT_NW_SPLIT_EVENT` → `EH_CP_FEAT_NW_SPLIT_EVENT`
  - `ESP_HOSTED_CP_EXT_WIFI_DPP_EVENT` → `EH_CP_FEAT_WIFI_EXT_DPP_EVENT`
  - Event enum types: `esp_hosted_cp_ext_wifi_evt_t` → `eh_cp_feat_wifi_evt_t` etc.
  - Event IDs: `ESP_HOSTED_CP_EXT_WIFI_EVT_*` → `EH_CP_FEAT_WIFI_EVT_*`
  - Core events: `ESP_HOSTED_CP_EVT_*` → `EH_CP_EVT_*`
  - `ESP_HOSTED_CP_EVENT` → `EH_CP_EVENT`
  - `ESP_HOSTED_CP_TIMEOUT_IN_MSEC` → `EH_CP_TIMEOUT_IN_MSEC`
  - `ESP_HOSTED_CP_BSS_STORE` → `EH_CP_BSS_STORE`
  - Acceptance: `grep -r "ESP_HOSTED_CP_EXT_.*EVENT\|ESP_HOSTED_CP_EVT_\|ESP_HOSTED_CP_BSS_STORE" modules/` returns 0

- [x] **R8: Bulk text replace — core internals**
  - `auto_feat_init_task` → `auto_feat_init_task`
  - `g_ext_init_done_eg` → `g_auto_feat_init_done_eg`
  - `eh_cp_master_config.h` → `eh_cp_master_config.h` (file + all includes)
  - `esp_hosted_cp_idf_compat.h` → `eh_cp_idf_compat.h`
  - `esp_hosted_cp_ext_dep_enforce.h` → `eh_cp_feat_dep_enforce.h`
  - `esp_hosted_cp_event.h` → `eh_cp_event.h`
  - `eh_cp_core.h` → `eh_cp_core.h`
  - `esp_hosted_cp.h` → `eh_cp.h`
  - `esp_hosted_cp_rpc.h` → `eh_cp_rpc.h`
  - All `#include "esp_hosted_cp_*"` → `#include "eh_cp_*"`
  - All `#include "eh_common_*"` → `#include "eh_common_*"` (if common renamed)
  - Acceptance: `grep -r '#include "esp_hosted_cp_' modules/` returns 0

- [x] **R9: Restructure Kconfig menus**
  - Top: `menu "Features"` (was `menu "Extensions"`)
  - `menuconfig ESP_HOSTED_CP_FEAT_WIFI` with sub-ext inside `if` block
  - `menuconfig ESP_HOSTED_CP_FEAT_BT`
  - `menuconfig ESP_HOSTED_CP_FEAT_SYSTEM`
  - `menuconfig ESP_HOSTED_CP_FEAT_NW_SPLIT`
  - Sub-ext visible only when parent enabled
  - Acceptance: `idf.py menuconfig` shows nested hierarchy

- [x] **R10: Parent-managed lifecycle for sub-extensions**
  - Sub-ext (ent/itwt/dpp) remove `EH_CP_FEAT_REGISTER`
  - WiFi feature init/deinit calls sub-ext init/deinit conditionally
  - Deinit in reverse order of init
  - Sub-ext CMake libs linked into parent feat, not top-level
  - Acceptance: no `EH_CP_FEAT_REGISTER` in sub-ext source

- [x] **R11: Auto-init Kconfig**
  - Master: `ESP_HOSTED_CP_AUTO_FEAT_INIT` (default y) in eh_cp_core Kconfig
  - Per-feature: `ESP_HOSTED_CP_FEAT_WIFI_AUTO_INIT` etc. (default y if master y)
  - Per-sub-ext: `ESP_HOSTED_CP_FEAT_WIFI_EXT_ENT_AUTO_INIT` etc.
  - `EH_CP_FEAT_REGISTER` conditional on auto-init
  - `auto_feat_init_task` not created if master auto-init is n
  - Acceptance: master=n → no features auto-init

- [x] **R12: Update all source guards + includes**
  - All remaining `#if EH_CP_EXT_*` → `#if EH_CP_FEAT_*`
  - All `#include` paths updated
  - All `extern` declarations match new names
  - Acceptance: clean compile (no warnings about missing symbols)

- [x] **R13: Update sdkconfig.defaults**
  - All examples: old CONFIG_ symbols → new
  - Acceptance: all examples `idf.py fullclean && idf.py build`

- [x] **R14: Rename parent directory**
  - `modules/coprocessor/extensions/` → `modules/coprocessor/features/`
  - Update ALL `add_subdirectory` paths in CMakeLists
  - Update ALL `orsource` paths in Kconfig.ext
  - Update `-T` LD path in parent CMakeLists
  - Acceptance: `extensions/` dir gone

- [x] **R15: Final verification**
  - Build all 6 examples: mcu_wifi, mcu_bt, fg_wifi, fg_bt, wifi_enterprise, wifi_itwt
  - `check.sh --soft` passes
  - Zero legacy naming: `grep -r "esp_hosted_cp_ext_\|EXT_FEAT_\|ext_descs\|EXT_REGISTER\|auto_feat_init_task\|ESP_HOSTED_CP_EXT_" modules/` returns 0
  - Zero old function prefix: `grep -r "esp_hosted_cp_init\|esp_hosted_cp_add_cap\|esp_hosted_cp_rx_get" modules/ --include="*.c" --include="*.h"` returns 0
  - Acceptance: ALL above greps return empty

### P1d — Utilities (after R1-R15, before feature ports)

- [x] **eh_cp_malloc / eh_cp_calloc** — port `slave_util.c/h`
  - Ref: `esp_hosted_mcu/slave/main/slave_util.c/h`
  - Location: `eh_cp_core/` (CP-specific mem cap selection)
  - APIs: `eh_cp_malloc(size, cap)`, `eh_cp_calloc(n, size, cap)`, `eh_cp_free(ptr)`
  - Acceptance: used by transport + features, compiles clean

- [x] **eh_cp_transport_gpio_pin_guard** — port `slave_transport_gpio_pin_guard.c/h`
  - Ref: `esp_hosted_mcu/slave/main/slave_transport_gpio_pin_guard.c/h`
  - Location: `eh_cp_transport/`
  - API: `eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num)`
  - Acceptance: GPIO Expander and Ext Coex use this before configuring pins

- [x] **eh_cp_stats** — port `stats.c/h`
  - Ref: `esp_hosted_mcu/slave/main/stats.c/h`
  - Location: `eh_cp_core/` or `eh_cp_transport/`
  - Verify: transport .c files already reference stats — check if inline or needs port
  - Acceptance: `EH_CP_PKT_STATS` guard, APIs match legacy

- [x] **FW version header** — port `esp_hosted_coprocessor_fw_ver.h`
  - Ref: `esp_hosted_mcu/slave/main/esp_hosted_coprocessor_fw_ver.h`
  - Location: `eh_cp_core/include/`
  - Name: `eh_cp_fw_version.h`
  - Acceptance: `req_get_coprocessor_fw_version` handler uses it correctly

### P1e — Feature ports (after P1d utilities)

- [x] **GPIO Expander** (`eh_cp_feat_gpio_exp`)
  - Ref: `esp_hosted_mcu/slave/main/slave_gpio_expander.c/h`
  - Depends: `eh_cp_transport_gpio_pin_guard` (P1d)
  - RPC: 7 handlers (config, reset, set_level, get_level, set_direction, input_enable, set_pull_mode)
  - Work: scaffold + port handlers + add to rpc dispatcher + example
  - Acceptance: example builds, handlers in dispatcher, gpio_pin_guard check before pin config

- [x] **Memory Monitor** (`eh_cp_feat_mem_monitor`)
  - Ref: `esp_hosted_mcu/slave/main/slave_control.c:769+`
  - RPC: 1 handler (req_mem_monitor) + FreeRTOS timer + periodic event
  - Work: scaffold + port handler + timer + event base + example
  - Acceptance: example builds, timer fires heap report, configurable interval

- [x] **External Coex** (`eh_cp_feat_ext_coex`)
  - Ref: `esp_hosted_mcu/slave/main/slave_ext_coex.c/h`
  - Depends: `eh_cp_transport_gpio_pin_guard` (P1d)
  - RPC: 1 handler (req_ext_coex)
  - Precondition: BT disabled, not ESP32, CONFIG_ESP_COEX_EXTERNAL_COEXIST_ENABLE
  - Work: scaffold + port + example
  - Acceptance: example builds, pin guard validates GPIO, coex wire type configured

- [x] **Light Sleep** (`eh_cp_feat_light_sleep`)
  - Ref: `esp_hosted_mcu/slave/main/slave_light_sleep.c/h`
  - APIs: init/deinit/start/stop/is_configured (5 functions, no RPC)
  - Work: scaffold + port + example (refer IDF light_sleep example)
  - Acceptance: example builds, PM lock acquired/released correctly

### P1f — Handler fixes + verification

- [x] **MAC addr get/set** — verify `req_iface_mac_addr_set_get` functional
  - Acceptance: handler reads/writes MAC per interface type correctly

- [x] **FW version struct** — match legacy 6-field format
  - Fields: `major/minor/patch/revision/prerelease/build` + `chip_id` + `idf_target`
  - Acceptance: `req_get_coprocessor_fw_version` returns all fields correctly

- [x] **Network Split deeper** — verify `nw_split_router.c` logic ported
  - Acceptance: DHCP/DNS routing, port-based filtering works per legacy

- [x] **Verify existing examples** — http_client, mqtt_client, peer_data_transfer
  - Check: `fg/cp/examples/cp_mcu/extensions/network_split/iperf/main/extra/` has http+mqtt
  - Check: `fg/cp/examples/cp_mcu/extensions/peer_data_transfer/` exists and builds
  - Acceptance: all extension examples build clean

### P1g — Dispatcher wiring + handler port (scaffolds exist, handlers need wiring)

- [x] **GPIO Expander dispatcher wiring**
  - Add `req_gpio_*` (7 handlers) to rpc_ext_mcu dispatcher switch-case
  - Add forward declarations to `eh_cp_feat_rpc_ext_mcu_priv.h`
  - Add cross-dep: rpc_ext_mcu → gpio_exp + eh_cp_transport (for pin guard)
  - Acceptance: build with `CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP=y`, handler reachable

- [x] **Memory Monitor handler port**
  - Port `req_mem_monitor` + timer + `mem_monitor_timer_cb` + helper functions from legacy `slave_control.c:58-804`
  - Create `eh_cp_feat_rpc_ext_mcu_handler_req_mem_monitor.c` in rpc_ext_mcu
  - Add to dispatcher + rpc_ext_mcu CMakeLists
  - Add event `RPC_ID__Event_MemMonitor` to evt handler
  - Acceptance: build, timer fires, event sent to host

- [x] **External Coex handler port**
  - Port `req_ext_coex` from legacy `slave_ext_coex.c`
  - Create `eh_cp_feat_rpc_ext_mcu_handler_req_ext_coex.c`
  - Add to dispatcher
  - Acceptance: build with coex enabled, wire-type configurable

- [x] **Light Sleep handler port**
  - Port `slave_light_sleep_init/deinit/start/stop/is_configured` into feature source
  - No RPC handler — local PM control only
  - Acceptance: build, PM lock management works

- [x] **DPP example target fix**
  - Add `sdkconfig.defaults.esp32c6` to wifi_dpp example
  - Acceptance: `idf.py -DIDF_TARGET=esp32c6 build` works

### P1h — Architectural cleanup

- [x] **Duplicate ESP_HOSTED_FEAT_CAPS_COUNT` removal**
  - Defined in both `eh_caps.h:91` and `eh_common_caps.h`
  - Remove from `eh_caps.h`, keep in `eh_common_caps.h` (canonical)
  - Acceptance: single definition, builds clean

- [x] **eh_caps.h enum cleanup** — enums already removed in P1-R. Updated include guards to `EH_CAPS_H`/`EH_COMMON_CAPS_H`.

- [ ] **Mempool port-layer abstraction** (deferred to Phase T) — `eh_mempool` calls `heap_caps_malloc` directly; needs `os_malloc`/`eh_malloc` via port layer for Linux host. Note: esp_hosted_mcu already has common mempool we can reuse pattern from.

- [x] **`fg/common/` shim elimination**
  - Already renamed to `fg/common.del`, no CMakeLists references remain
  - Manual step: `rm -rf fg/common.del`
  - Acceptance: no CMakeLists references `fg/common` ✅

- [x] **Cap bits architecture spec**
  - Three-tier system documented in `coprocessor/verified/cap_bits.md`
  - Covers: tiers, bit definitions, API, design rationale, wire-breaking rules

- [x] **Per-feature auto-init Kconfig (R11 remainder)**
  - 14 features: `ESP_HOSTED_CP_FEAT_*_AUTO_INIT` (default y, depends on master)
  - `EH_CP_FEAT_REGISTER` wrapped in `#if EH_CP_FEAT_*_AUTO_INIT` in all source files
  - Aliases in `eh_cp_master_config.h`
  - Acceptance: individual features can opt out of auto-init ✅

### P1-T — Test Infrastructure

- [x] **pytest-embedded dual-DUT** — 9 test files, 46 cases, all pass on HW
- [x] **Test infra library** — `tests/infra/` (hardware probe, flasher, builder, reporter)
- [x] **Test matrix** — `test_matrix.yaml` with 8 pairs
- [x] **Matrix runner** — `run_all.py` with mode selection, dry-run, JSON/JUnit output
- [x] **Mode naming** — "real/stub" is too informal. Rename to "hardware/simulated" or "on-target/host-emulated"
- [x] **End-to-end runner test** — run_all.py not tested on actual hardware (only list/dry-run)
- [ ] **Simulated tests** — RPC simulated tests on Linux target (Phase T2)
  - Build CP for Linux target
  - Inject mock RPC via socket/pipe
  - Verify handler response without hardware
  - Covers: enterprise, DPP, iTWT, ext_coex, power save

### P2 — Pre-release cleanup

- [x] Re-run all examples, capture build + runtime logs
- [x] Update `implementation_status.md` — full rewrite reflecting P1-R through P1-T completion
- [x] `check.sh --soft` passes

### P3 — `eh_tlv` Common Component (Init TLV Backward Compat)

**Problem**: New CP sends ~83 bytes init TLV; old MCU hosts assert `len < 64`.
**Solution**: New standalone `eh_tlv` common component with pack/unpack APIs.
TLV groups auto-selected by RPC version Kconfig. No host patches needed.

**Naming convention**:
- Component: `modules/common/eh_tlv/`
- Macros: `EH_TLV_V1_LINUX`, `EH_TLV_V1_MCU`, `EH_TLV_V2`
- Kconfig: `ESP_HOSTED_TLV_V1_LINUX`, `ESP_HOSTED_TLV_V1_MCU`, `ESP_HOSTED_TLV_V2`
- Pack APIs: `eh_tlv_pack_v1_linux()`, `eh_tlv_pack_v1_mcu()`, `eh_tlv_pack_v2()`
- Unpack APIs: `eh_tlv_unpack_v1_linux()`, `eh_tlv_unpack_v1_mcu()`, `eh_tlv_unpack_v2()`
- Primitives: `eh_tlv_add_u8()`, `eh_tlv_add_u32_le()`, `eh_tlv_add_buf()`, `eh_tlv_add_str()`, `eh_tlv_add_u32_array_le()`
- Parser prims: `eh_tlv_read_*()` or iterator API

**Selection matrix** (hidden Kconfig, auto from RPC version):

| RPC version | V1_LINUX | V1_MCU | V2 | Payload |
|------------|:---:|:---:|:---:|---------|
| V1 Linux | y | n | n | ~21 B |
| V1 MCU | n | y | n | ~27 B |
| V2 (default) | n | y | y | ~83 B |

- [x] **P3-1: Create `eh_tlv` component scaffold**
  - Dir: `modules/common/eh_tlv/`
  - `CMakeLists.txt`: INTERFACE library, no deps
  - `Kconfig.ext`: hidden `ESP_HOSTED_TLV_V1_LINUX`, `ESP_HOSTED_TLV_V1_MCU`, `ESP_HOSTED_TLV_V2` (default from RPC version)
  - `include/eh_tlv_defs.h`: `EH_TLV_V1_LINUX`/`V1_MCU`/`V2` macros from Kconfig
  - Source parent `modules/common/CMakeLists.txt` to include `eh_tlv`
  - Source `Kconfig.ext` from coprocessor parent Kconfig
  - Acceptance: `idf.py menuconfig` shows no new visible options; macros auto-set correctly per RPC choice

- [x] **P3-2: Builder + parser primitives (`eh_tlv.h`)**
  - `include/eh_tlv.h`: all `static inline`, no deps
  - Builder: `eh_tlv_builder_t`, `eh_tlv_builder_init()`, `eh_tlv_builder_len()`
  - `eh_tlv_add_u8(b, tag, val)` → `int` (0 ok, -1 overflow)
  - `eh_tlv_add_u32_le(b, tag, val)` → same
  - `eh_tlv_add_buf(b, tag, data, len)` → same
  - `eh_tlv_add_str(b, tag, str)` → same (strlen, no null)
  - `eh_tlv_add_u32_array_le(b, tag, arr, count)` → same (for feat_caps)
  - Parser: `eh_tlv_parser_t`, `eh_tlv_parser_init()`, iterator or struct-based read API
  - Acceptance: unit-testable in isolation; no Kconfig or IDF deps in core builder

- [x] **P3-3: Pack functions — `eh_tlv_v1_linux.h`, `eh_tlv_v1_mcu.h`, `eh_tlv_v2.h`**
  - Each file: `#if EH_TLV_V1_XXX` guard around entire content
  - `eh_tlv_pack_v1_linux(b, cap, raw_tp_cap)` — tags 0x00-0x04, ~21 bytes
  - `eh_tlv_pack_v1_mcu(b, cap, ext_cap, raw_tp_cap, rx_q, tx_q)` — tags 0x11-0x17, ~27 bytes
  - `eh_tlv_pack_v2(b, feat_caps)` — tags 0x19, 0x20, 0x22, 0x24, 0x25, ~56 bytes
  - Each returns 0 on success, -1 on overflow
  - Acceptance: each function compiles only when its macro is 1; total sizes match expected

- [x] **P3-4: Unpack functions**
  - Output structs: `eh_tlv_v1_linux_t`, `eh_tlv_v1_mcu_t`, `eh_tlv_v2_t` (fields matching TLV payloads)
  - `eh_tlv_unpack_v1_linux(buf, len, &out)` — parse legacy FG TLVs
  - `eh_tlv_unpack_v1_mcu(buf, len, &out)` — parse MCU TLVs
  - `eh_tlv_unpack_v2(buf, len, &out)` — parse feat_caps + negotiation TLVs
  - Each gated by `#if EH_TLV_V1_XXX`
  - Acceptance: round-trip test — pack then unpack, verify identical fields

- [x] **P3-5: Refactor CP transport — use `eh_tlv_pack_*` in `generate_startup_event()`**
  - Replace inline TLV building in all 4 transport files:
    - `modules/coprocessor/eh_cp_transport/src/eh_cp_transport_sdio.c`
    - `modules/coprocessor/eh_cp_transport/src/eh_cp_transport_spi.c`
    - `modules/coprocessor/eh_cp_transport/src/eh_cp_transport_spi_hd.c`
    - `modules/coprocessor/eh_cp_transport/src/eh_cp_transport_uart.c`
  - Caller uses `#if EH_TLV_V1_LINUX` / `#if EH_TLV_V1_MCU` / `#if EH_TLV_V2` macros
  - SPI/SPI-HD/UART: align with SDIO's mutually-exclusive model (currently send BOTH legacy + MCU unconditionally)
  - CMake: add `eh_tlv` dep to `eh_cp_transport`
  - Acceptance: each transport ~30 lines instead of ~120; build all 4 transport types

- [x] **P3-6: Migrate `LEGACY_BOOT_CAP_TLVS` → `TLV_V1_LINUX`**
  - Replace `CONFIG_ESP_HOSTED_LEGACY_BOOT_CAP_TLVS` with `EH_TLV_V1_LINUX` in:
    - `eh_transport.h`: `esp_hosted_priv_pkt_type_event_wire()`, `esp_hosted_priv_event_init_wire()`
    - `eh_caps.h`: legacy FG struct + version defines
  - Remove `ESP_HOSTED_LEGACY_BOOT_CAP_TLVS` from `eh_cp_feat_rpc/Kconfig.ext`
  - Acceptance: `grep -r LEGACY_BOOT_CAP_TLVS modules/` returns 0

- [ ] **P3-7: Refactor host-side — use `eh_tlv_unpack_*` in `process_init_event()`**
  - MCU host: `fg/host/components/eh_host_transport/src/mcu/transport_drv.c`
  - Kmod SPI: `fg/host/components/eh_host_transport/src/kmod/spi/esp_spi.c`
  - Kmod SDIO: `fg/host/components/eh_host_transport/src/kmod/sdio/esp_sdio.c`
  - Fix `uint8_t len_left` → `uint16_t len_left` (future-proof for TLV > 255)
  - Replace inline TLV parsing with `eh_tlv_unpack_*` calls
  - Acceptance: build MCU host + kmod; same behavior as before

- [ ] **P3-8: Build verification**
  - Build CP with `RPC V1 MCU` → verify init TLV ~27 bytes (hex log)
  - Build CP with `RPC V2` (default) → verify ~83 bytes
  - Build CP with `RPC V1 Linux` → verify legacy format ~21 bytes
  - All 6 examples build clean
  - Acceptance: `idf.py build` passes for all target combinations

- [x] **P3-9: Spec updates**
  - Updated `priv_handshake.md`: TLV group architecture + `eh_tlv` component (new section)
  - Updated `component_map.md`: added `eh_tlv` + `eh_frame`, refreshed feature list (extensions/→features/, current 17 features), Kconfig chain
  - Updated `scope.md`: removed "Kconfig compat mode" from Deferred
  - Bonus: fixed `cap_bits.md` tag prefix `sp.cp.ve-cb` → `sp.co.ve-cb`, registered `cb` in tag_registry
  - `check.sh --soft` passes (132 tags, 0 warnings)
<!-- %% sp.ta-to.v1.c %% -->

<!-- %% sp.ta-to.specfix.o %% - always -->
## Spec Corrections (Ongoing)

- [x] All prior corrections absorbed into specs
<!-- %% sp.ta-to.specfix.c %% -->

<!-- %% sp.ta-to.fut.o %% - context -->
## Future (Post V1 — Do Not Implement)

- Header V2 negotiation
- RPC V2 unified proto (blocked on PENDING-006)
- Host-side H1–H6 (MCU host first, then Linux FG)
- Socket transport + system test harness (Phase T)
- Phase F (V2 proto end-to-end, CP + FG + MCU)
- Kmod RX reassembly fix + RB size increase
- PENDING-013 two-stage INIT TLV handshake (feat_caps delivery when host consumer exists)
- CI lint script for specs
- Host macro migration
- Proto file rename (align with component naming convention):
  - `esp_hosted_config.proto` → `eh_proto_linux_v1.proto` (CP + host both sides)
  - `esp_hosted_config.pb-c.{h,c}` → `eh_proto_linux_v1.pb-c.{h,c}`
  - `esp_hosted_rpc.proto` → `eh_proto_mcu_v1.proto`
  - `eh_rpc.pb-c.{h,c}` → `eh_proto_mcu_v1.pb-c.{h,c}`
  - `esp_hosted_cp_lin_fg_pbuf.h` → `eh_proto_linux_v1.h` (host component header)
  - Also rename stale `.proto.old` / `.proto.bk_*` files in `modules/common/serializers/`
  - Requires coordinated update: CP serializers + host component + host linux user_space
  - Wire compat: safe (proto `package` absent — symbols from message names only)
  - Also rename: `fg/host/linux/user_space/c_demo_app/common/esp_hosted_custom_rpc.h` → `eh_custom_rpc.h`
  - Deferred reason: host repo (`esp_hosted_mcu`) includes these by old name; rename breaks host build until host is updated
- Delete `.meta/specs/legacy/` (stale spec archive — all content absorbed into verified specs)
- Port layer + testing (`.meta/roadmap/port_layer_and_testing.md`)
- Parity maps (deferred): Kconfig coverage table (esp_hosted_mcu → EH_CP_*), RPC API coverage table (slave APIs → CP extensions), WiFi logic diff review
- Fix component dependency wiring for `eh_cp_feat_wifi` headers (export public include via feature CMake and use `REQUIRES` instead of example include hacks)
- Add explicit core host-state APIs (option B: per-state getter/setter functions with shared mutex; no registry/uint32)
<!-- %% sp.ta-to.fut.c %% -->

<!-- %% sp.ta-to.c %% -->
