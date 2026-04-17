<!-- %% sp.sy.ve-sc.o %% - always -->
---
segment: system
status: verified
last_verified: 2026-04-13
release: v1.0
---

# Release Scope

<!-- %% sp.sy.ve-sc.v1.o %% - always -->
## V1 — Confirmed Working (Do Not Break)

Legacy header + legacy RPC paths verified on all examples:

| Example | Status |
|---------|--------|
| `fg/cp/examples/cp_linux_fg/minimal/wifi/` | ✅ builds + runs |
| `fg/cp/examples/cp_linux_fg/minimal/bt/` | ✅ builds + runs |
| `fg/cp/examples/cp_mcu/minimal/wifi/` | ✅ builds + runs |
| `fg/cp/examples/cp_mcu/minimal/bt/` | ✅ builds + runs |

What "verified" means here: legacy host ↔ coprocessor RPC and events work end-to-end.
V2 header negotiation, V2 RPC, host-side H1–H6 are NOT part of this confirmation.
<!-- %% sp.sy.ve-sc.v1.c %% -->

<!-- %% sp.sy.ve-sc.v1rem.o %% - always -->
## V1 — Remaining Work Before Release

These are in scope for V1 but not yet complete:

- [ ] **Verify D1–D3**: `EH_CP_FEAT_REGISTER` macro, linker section `.eh_cp_ext_descs`,
      `auto_feat_init_task` + `g_ext_init_done_eg` event group — read `fg/cp/` source, confirm with build
- [ ] **Capability bits**: Centralized core API (`cap_bits_set/clear/get`).
      For V1 hosts this applies to Tier‑1 (`caps`, 0x11) and Tier‑2 (`ext_caps`, 0x16).
      Tier‑3 `feat_caps` (0x19) is **V2 TLV only** and not required for V1‑only release.
- [ ] **MCU feature parity** — onboard as individual extensions (each follows extension rules):
  - GPIO Expander (`slave_gpio_expander.c` reference)
  - Memory Monitor (`CONFIG_ESP_HOSTED_MEM_MONITOR`)
  - ITWT — 4 events (StaItwtSetup/Teardown/Suspend/Probe)
  - DPP — wpa_supp + native wifi paths
  - WiFi Enterprise (`slave_wifi_enterprise.c` reference)
  - External Coex (`slave_ext_coex.c` reference)
  - Light Sleep (`slave_light_sleep.c` reference)
  - Network Split deeper LWIP/DHCP integration
  - MAC addr get/set by type
  - FW version struct — must match 6-field format (major/minor/patch/revision/prerelease/build + chip_id + idf_target)
- [ ] **Build gate**: `idf.py build` both cp_linux_fg and cp_mcu examples after above changes
<!-- %% sp.sy.ve-sc.v1rem.c %% -->

<!-- %% sp.sy.ve-sc.def.o %% - always -->
## Explicitly Deferred (Not V1)

| Item | Reason | Where tracked |
|------|--------|--------------|
| Header V2 | Requires new handshake design | `specs/common/unverified/header_v2.md` (future) |
| RPC V2 unified proto | Blocked on PENDING-006 schema | `specs/coprocessor/unverified/rpc_v2.md` (future) |
| Host-side H1–H6 | After CP complete | `specs/host/unverified/architecture.md` (future) |
| Socket transport (Phase T) | Test infrastructure, post-V1 | `specs/system/unverified/` (future) |
| Phase F (V2 proto end-to-end) | Depends on RPC V2 + Header V2 | Future |
| Kmod RX reassembly fix | Fix exists, not urgent | `specs/tasks/todo.md` |
| PENDING-013 two-stage TLV | Brainstorm at Header V2 time | Future |
<!-- %% sp.sy.ve-sc.def.c %% -->

<!-- %% sp.sy.ve-sc.rules.o %% - context -->
## Scope Rules for Agents

1. If a task is in the "Deferred" table → stop, add to `specs/tasks/todo.md`, do not implement
2. `specs/{segment}/NV/` specs → do not implement for V1 unless this file says otherwise
3. Any feature from `slave_control.c` not in the "V1 Remaining" list → treat as deferred
4. When onboarding MCU parity features: each must be a standalone extension with its own
   Kconfig guard, capability bit registration, and init/deinit following extension rules
<!-- %% sp.sy.ve-sc.rules.c %% -->

<!-- %% sp.sy.ve-sc.c %% -->
