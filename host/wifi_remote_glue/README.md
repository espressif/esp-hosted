# `host/wifi_remote_glue/`

esp-hosted's **implementor side** of the `esp_wifi_remote_*` hook
contract.  Defines the strong bodies that override the upstream
`esp_wifi_remote` library's WEAK fallbacks (which return
`ESP_ERR_NOT_SUPPORTED` and would silently break every Wi-Fi call),
plus the STA/AP netif channel registration glue.

**Permanent component.** Relocated out of `host/compat/` in P41 to
make this explicit — `host/compat/` is for back-compat shims that
will retire as downstream apps migrate to native `eh_host_*` APIs.
This archive is NOT that.  Its strong overrides must exist for the
host stack to function on every supported port whenever wifi is in
the build.

## Pieces

All gated at `host/CMakeLists.txt` on `FEAT_WIFI_READY`:

| TUs | Purpose | Built when |
|---|---|---|
| `esp_wifi_remote_glue.c` + `esp_wifi_remote_extended.c` + `esp_wifi_he_with_hosted.c` | ~50 strong overrides forwarding to `eh_host_wifi_*` / `eh_host_sys_*` | `FEAT_WIFI_READY` |
| `esp_eap_dpp_with_hosted.c` | EAP / DPP forwarders | one of `FEAT_WIFI_EXT_ENT_READY` / `FEAT_WIFI_EXT_DPP_READY` |

STA / AP channel registration is inlined inside `esp_wifi_remote_init` /
`_deinit` (in `esp_wifi_remote_glue.c`), gated on
`EH_HOST_TYPE_MCU && EH_HOST_FEAT_WIFI_READY`, and goes through the
upstream-compatible `esp_hosted_add_channel` / `esp_hosted_remove_channel`
public API exposed by `eh_host_mcu_transport`.

## Header source

| Port | Where `esp_wifi_remote.h` comes from |
|---|---|
| IDF | IDF managed component (or inlined `esp_wifi/remote/` in IDF ≥6.0) |
| non-IDF | Our wrapper at `port/esp_idf_port/esp_wifi_remote/` |

Glue's `CMakeLists.txt` conditionally links the non-IDF wrapper target
for include propagation.  On IDF the header is reachable via IDF's
component requirement system.

## Archive retention (P41 — no `--whole-archive`)

GNU ld resolves strong-over-weak across archives: a weak definition
does not satisfy an undefined symbol; the linker keeps searching for
a strong def.  Every strong override in this archive has at least one
symbol that the upstream submodule's adapter forwarders reference
once the app calls a corresponding `esp_wifi_*` API:

```
   app: esp_wifi_init(...)
     → submodule esp_wifi_with_remote.c::esp_wifi_init
        → esp_wifi_remote_init   ← undefined symbol
            ↓ ld scans archives
            ↓ submodule's weak: ESP_ERR_NOT_SUPPORTED  (weak — keep searching)
            ↓ this archive's strong: forwards to eh_host_wifi_init  (strong wins)
```

So `--whole-archive` is NOT required for correctness; standard archive
scan + strong/weak resolution handles every override naturally.
(Pre-P41 the bracket existed as a safety net; P41 attempted removal
and verified Linux builds + ABI gate clean — kept removed.)

## Status — wifi forwarders

✅ Tier-1 (12 hooks): init / deinit / start / stop / connect /
disconnect / set+get_mode / set+get_config / set+get_mac.

**Station getting-started coverage (release-bar gate)**: the
unmodified IDF station example
(`examples/wifi/getting_started/station/main/station_example_main.c`)
makes exactly five distinct `esp_wifi_*` calls — `esp_wifi_init`,
`esp_wifi_set_mode`, `esp_wifi_set_config`, `esp_wifi_start`,
`esp_wifi_connect` — all five resolve onto Tier-1 hooks above.  The
softAP variant uses the same set minus `connect`.  Indirect calls
through `esp_netif_create_default_wifi_*` / shutdown handler are
limited to `esp_wifi_stop` + `esp_wifi_get_mac`, also covered.
Verified 2026-04-28 against IDF master station example.

Plus ~30 additional Tier-2 hooks in `esp_wifi_remote_extended.c`
(scan, restore, deauth_sta, fast_connect, etc.) — none required for
the getting-started release-bar gate, but bring up the broader
station-mode surface.

## Upstream origin

Hand-written, not vendored.  Reference for the canonical hook list:

- `_upstream_mcu/host/api/priv/esp_hosted_api_priv.h` — declares
  ~128 entries (wifi core, scan, sta/AP info, country, channel,
  bandwidth, ps, storage, iTWT, dual-band, enterprise, DPP).
- `_upstream_mcu/host/api/src/esp_hosted_api.c` — body counterpart
  (forwards to upstream's `rpc_wifi_*`; we forward to
  `eh_host_wifi_*` / `eh_host_sys_*` instead).
- `_upstream_mcu_stm32/host/extra_components/esp_wifi_remote/`
  (≈647 LOC) — STM32 port's lumped impl, similar shape but smaller
  surface; useful as a cross-check.

## Periodic-sync checklist

Per `.meta/specs/system/verified/upstream_sync.md`:

1. Diff `_upstream_mcu/host/api/priv/esp_hosted_api_priv.h` (the
   canonical declaration list) — list any new entries.
2. For each new entry, check whether the corresponding `eh_host_*`
   lib symbol exists; if yes, add a tier-N hook here.
3. Update "last sync" line below with the upstream tag / date.

**Last sync**: 2026-04-28 (P1.2 — verified Tier-1 12 hooks fully cover
IDF getting-started station + softAP examples; no new hooks needed).
