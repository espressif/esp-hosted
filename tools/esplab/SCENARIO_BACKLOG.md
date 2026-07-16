# ESP-Hosted scenario backlog

Exhaustive scenario vision beyond the live `tests.json`, gathered by combing the
codebase + examples. Each entry: honest support verdict for the **current C6+P4
SDIO emulator bench**, and the **blocker** that gates it. Promote an entry into
`tests.json` once its blocker is cleared and it's verified on the bench. Status
legend: ✅ runnable now · 🟡 tbd (blocker named) · ⚪ unsupported (fundamental).

## Unlock buckets (build these to flip whole groups)
- **B1 raw frame/TLV injection hook** on the SDIO bridge → all frame/TLV integrity negatives.
- **B2 eh_host API-exerciser** — **now exists**: `examples/system/api_exerciser`
  (console-drives the `eh_host_*` surface, one uniform `EH rc=.. cmd=.. [k=v]`
  line per call) + `tests/emu-mcu/eh_test_feat_api/`. Scalar/string
  wifi + system set/get commands are wired; a new row = a test-data file (no
  firmware). gpio/coex commands are the next batch (same pattern). *No emulator change.*
- **B3 CP fault hooks** (stall heartbeat / hold an RPC response / corrupt a payload) → recovery, rpc-timeout, heartbeat negatives.
- **B4 CP image with a flag flipped** (feature off / version / caps drift) → caps/version/feature-gating/nw-split-no-feature.
- **B5 second virtual STA client + multi/empty AP** in the Wi-Fi medium → softap-client / apsta-client / scan-empty / scan-truncation.
- **B6 emulator HW features** (light-sleep, SPI-FD/HD, BLE-VHCI, 802.15.4) — see esp-emulator `docs/hosted/test-coverage-features.md`.

---

## Wi-Fi · Station / SoftAP / Scan
| Scenario | Kind | Verdict | Blocker / note |
|---|---|---|---|
| softap_ssid_hidden | pos | ✅ overlay (`ssid_hidden`) on wifi/softap | verify "wifi_init_softap finished" |
| softap_channel_specific | pos | ✅ overlay (`channel`) | verify |
| scan_passive | pos | ✅ overlay (scan method) on wifi/scan | verify "Total APs scanned" |
| sta_bssid_pinned | pos | ✅ | api_exerciser: wifi_cfg_set sta_bssid → set_config sta (marshalled) |
| sta_powersave_min / _max | pos | ✅ | api_exerciser: `wifi_set_ps`/`wifi_get_ps` round-trip (eh_test_feat_api) |
| sta_country_code_us / _eu | pos | ✅ | api_exerciser: `wifi_set_country`/`wifi_get_country` round-trip |
| sta_max_tx_power_min / _full | pos | ✅ | api_exerciser: wifi_set/get_max_tx_power (eh_test_feat_api) |
| sta_get_ap_info_post_connect | pos | 🟡 B2 | eh_host_wifi_sta_get_ap_info:377 |
| sta_config_roundtrip_set_get | pos | ✅ | api_exerciser: `wifi_cfg_set`→`wifi_set_config sta`→`wifi_get_config sta` (ssid round-trip; eh_test_feat_api) |
| sta_set/get_config_null_arg | neg | ✅ | api_exerciser: wifi_set_config_null → INVALID_ARG guard |
| sta_mode_switch_sta_to_apsta | pos | ✅ | api_exerciser: wifi_set_mode 3/1 → get_mode round-trip |
| softap_config_roundtrip (+beacon) / dtim / pmf / pairwise | pos | ✅/🟡 | api_exerciser: AP ssid/channel/beacon round-trip ✅ (mode APSTA); dtim/pmf/pairwise 🟡 (add fields) |
| scan_clear_ap_list / scan_get_ap_num_zero / scan_async | pos/neg | ✅/🟡 | api_exerciser: clear_ap_list + scan_get_ap_num ✅, scan_start drivable; async-specific 🟡 |
| softap_client_assoc / _wrong_password / _max_clients | pos/neg | 🟡 B5 / ⚪ | needs virtual STA client |
| sta_wpa3 | pos | 🟡 B5 | emulator AP is WPA2-PSK only |
| sta_itwt (HE) | pos | ⚪ | no 802.11ax AP |
| sta_dpp / sta_enterprise | pos | ⚪ | external DPP configurator / RADIUS |
| sta_protocol_11b/g_only, sta_bandwidth_20mhz | pos | ⚪ | no API exposed |

## System / heartbeat / mem-monitor
| Scenario | Kind | Verdict | Blocker |
|---|---|---|---|
| sys_get_cp_info (chip_id / target) | pos | 🟡 B2 | eh_host_sys_get_cp_info |
| sys_get/set_mac (sta/ap) | pos | ✅ | api_exerciser: `sys_get_mac`/`sys_set_mac` (readback asserted) |
| sys_set_mac_invalid_oui | neg | 🟡 B2+B3 | CP-side validation |
| heartbeat_enable_disable / interval_3s / interval_10s | pos | 🟡 B2 | eh_host_heartbeat_configure |
| heartbeat_timeout_recovery | neg | 🟡 B3 | stall CP heartbeat |
| mem_monitor_threshold_alert | pos | 🟡 B3 | heap-stress to breach threshold |

## GPIO expander / external coex
| Scenario | Kind | Verdict | Blocker |
|---|---|---|---|
| gpio mode input/output/od, pull up/down, reset_pin, set_direction, input_enable, set_pull_mode | pos | ✅ | api_exerciser: gpio_* commands; emu models CP GPIO — set_level→get_level round-trip (eh_test_feat_api) |
| gpio_invalid_pin_out_of_range | neg | 🟡 B2 | invalid-pin variant |
| coex work_mode leader/follower, gpio 3wire/4wire, grant_delay, validate_high, disable | pos | 🟡 B2/HW | api_exerciser: coex_* commands drivable; coex_init ok but config ops return ESP_FAIL on emu (need physical wires) — rc=0 assertion belongs on HW |
| coex_invalid_gpio | neg | 🟡 B2 | invalid GPIO |

## Frame / TLV integrity  (all need **B1**)
frame: v1/v2 offset min/max + below/above reject (INVALID), v2 strict offset==20,
v1/v2 checksum mismatch (CORRUPT), checksum-disabled-accepted, len0=DUMMY, dummy encode v1/v2,
version detect v1/v2/empty, max_buf exceeded (TOOBIG), truncated header/payload, seq/pkt/frag encode.
tlv: truncated tag/len/value, zero-len valid, max-len 255, builder overflow u8/u32/buf, val short-defaults, u32_array bounds.
→ ~28 scenarios, all 🟡 B1 (raw frame/TLV injection). Anchors: common/eh_frame/src/eh_frame.c:213-315, common/eh_tlv/include/eh_tlv.h:35-143.

## RPC / mempool / lifecycle / capabilities
| Scenario | Kind | Verdict | Blocker |
|---|---|---|---|
| rpc_uid_never_zero / concurrent_ordering | pos | 🟡 B2 | API-exerciser harness |
| rpc_uid_wraparound | pos | 🟡 B2+ | counter manipulation |
| rpc_sync_table_exhaustion (>16) | neg | 🟡 B2/B3 | 17 concurrent / CP stall |
| rpc_async_timeout_callback | neg | 🟡 B3 | hold CP response |
| rpc_unknown_uid_response_dropped | neg | 🟡 B1 | raw RPC inject |
| mempool alloc-on-empty / free-invalid / double-free / alloc-alignment | pos/neg | 🟡 B2 | mempool exerciser |
| mempool_exhaustion_burst | neg | 🟡 B2 | burst stimulus |
| host_init_idempotent / use_before_init_rejected / deinit_reinit_cycle | pos/neg | 🟡 B2 | lifecycle harness (already stubbed in tests.json as tbd) |
| host_connect_timeout_no_cp | neg | ⚪ | emu always boots both images |
| caps_version_hash_mismatch / chip_id_mismatch | neg | 🟡 B4 | CP built w/ modified caps/chip-id |
| fw_version_mismatch_reported | neg | 🟡 B4 | CP built w/ modified version string |

## Power-save / transports / radios (emulator HW — bucket B6)
| Scenario | Verdict | Blocker |
|---|---|---|
| cp_light_sleep | ✅ supported (functional) | runs on current emu (tickless-idle/WFI); CP configures auto light-sleep + integration. Power/timing accuracy would need the spec'd feature (HW-lab) |
| cp_light_sleep_wake_on_packet | ✅ supported (functional) | full real flow end-to-end, **stable across 20 consecutive wake cycles**: host PS → CP light sleep → TCP packet to a host priority port → CP wakes host → host re-inits. Required fixing two real bugs the bench exposed: (emu) the C6 SDIO slave flushes its data-plane on a host-power-save bridge notice (esp-emulator OP_HOST_SLEEP); (host) the Wi-Fi RX path drops frames until the lwip netif input is bound, so the CP forwarding a frame on wake before the netif is up no longer faults wlanif_input. |
| cp_light_sleep_non_priority_pkt_no_wake (neg) | ✅ supported (functional) | negative counterpart: a UDP packet to a non-priority host-range port (50000) is dropped during PS (`host pkt dropped in power save (dst 50000`) and the host does **not** wake (no `Wakeup needed` / `Host power save off`). Bounded — anchored on the real drop marker, not a blind never-wait |
| cp_light_sleep_wake_race_cycles (neg/recovery) | ✅ supported (recovery) | consecutive wake cycles with a wake racing the CP teardown each cycle (runner `repeat`+`burst` net stimulus: a short TCP burst keyed on "Host Sleep" straddles the ~400ms deinit window; settle watches ~120s). Guards a 3rd real CP bug the bench exposed: a check-then-act race between `host_power_save_alert(PS_ON)` deinit and `wakeup_host`'s `if_handle->state` re-init decision — a wake landing mid-deinit skipped the re-init and left SDIO dead → host `bring-up timed out` → abort (`udp.c:793`). Fixed by a transition mutex serialising teardown vs wake re-init (`eh_cp_feat_host_ps_internal.c`); upstream esp-hosted-mcu has the identical unfixed logic. Validated both ways: fix reverted → FAIL by cycle 2; fix in place → 22 cycles, 0 timeouts. |
| (emulator bridge desync — 4th bug) | ✅ fixed; guarded by an emulator unit test | Exposed by **slow-paced** multi-iteration wake cycles. The CP's `wakeup_host` loop pulses `OP_WAKE` every ~200ms until the host returns its wake semaphore, so while a just-woken host spends ~1.3s re-enumerating SDIO, a stale `OP_WAKE` frame lands mid-CMD52/53 and the host bridge reader desynced (read it as the CMD response) → garbage blocksize (`Function 1 Blocksize: 65535`) / `bring-up timed out` → abort. Fast cycles wake the host too quickly to expose it; slow manual pacing hits it (the user's "not fixed for multiple iterations"). Fixed in esp-emulator `sdio_bridge.rs`: `recv_response_for` now consumes an async `OP_WAKE` out-of-band like `OP_INT_RAISE` (real HW has a separate wake wire; the bridge multiplexes both on one socket). This is a probabilistic race — not a reliable esplab scenario — so it's guarded by a **deterministic Rust unit test** in `sdio_bridge.rs` (feeds an `OP_WAKE` between a CMD header and its response, asserts no desync); verified it fails without the fix. |
| (emulator stale-wake — 5th bug) | ✅ fixed | Exposed by **manual-paced** multi-iteration cycles — the user's "easy to reproduce" cycle-2 failure. The CP pulses `OP_WAKE` until the host acks; late pulses (after the host already woke in cycle 1) set the P4's `wake_requested`, which was only ever cleared on *leaving* deep sleep, never on *entering*. So on cycle 2 the host armed deep sleep and immediately consumed the stale flag — a wake the CP never drove (CP log shows **no** `Wakeup needed`) → CP stays parked, SDIO never re-inited → host re-enumerates into a dead link → `bring-up timed out` → abort. Deterministic by hand; packet-every-cycle automation hid it (a real trigger always reached the CP). Fixed in esp-emulator `esp32p4/chip.rs`: clear `wake_requested` + drain the bridge's latched pulse on deep-sleep-hold entry, so only a wake asserted *while held* counts. Validated both ways with a manual-style stdin driver: fix reverted → cycle-2 timeout; fix in place → cycle 2 holds and awaits a real wake, 0 timeouts. |
| spi_transport → spi_fd / spi_hd | 🟡 B6 | SPI FD/HD bridge (spec'd) |
| ble_adv_vhci | 🟡 B6 | BLE VHCI routing (spec'd) |
| ot_cli / ot_border_router | ⚪/🟡 B6 | 802.15.4 PHY + medium (giant, spec'd) |
| uart_transport | 🟡 B6 | UART bridge (not yet spec'd; mirror SDIO/SPI) |
| throughput_iperf / *_latency | ⚪ | emulator timing not cycle-accurate (HW lab) |
