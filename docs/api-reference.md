# ESP-Hosted host API reference

[Home](../README.md) · [Getting Started: MCU](getting-started-mcu.md) · [Getting Started: Linux](getting-started-linux.md) · [Bluetooth](features/bluetooth.md) · [Troubleshooting](troubleshooting.md)

This page lists the **public** host-side API of ESP-Hosted, grouped by feature.
It is a map, not a substitute for the headers — each row names the exact symbol
and the header that declares it.

---

## Two API surfaces

Almost every feature is reachable two ways:

| Surface | Prefix | Where | Use it when |
| :--- | :--- | :--- | :--- |
| **Native** | `eh_host_*` | `host/features/eh_host_feat_*/include/`, `host/eh_host_core/include/`, `port/os/include/` | Writing new ESP-Hosted apps |
| **Compat** | `esp_hosted_*` | `host/compat/include/` | Porting an existing upstream `esp-hosted-mcu` app unchanged |

The compat symbols are thin aliases (mostly `#define`s) over the native ones —
both are publicly usable. Two umbrella headers pull everything in:

```c
#include "eh_host.h"       // native surface (per-feature, gated on EH_HOST_FEAT_*)
#include "esp_hosted.h"    // compat surface (upstream esp_hosted_* names)
```

Which features are compiled in is gated by `EH_HOST_FEAT_*` (from
`eh_host_port_master_config.h`, generated from Kconfig). A feature's API only
exists when its `CONFIG_ESP_HOSTED_HOST_FEAT_*` is enabled.

> The Wi-Fi data path is not on this list: unmodified `esp_wifi_*` calls work
> directly under the compat build. This page covers the ESP-Hosted control APIs.

---

## Transport bring-up & configuration
Header: `eh_host_transport_config.h` · compat `esp_hosted_transport_config.h`

| Native | Compat | Purpose |
| :--- | :--- | :--- |
| `eh_host_transport_set_default_config()` | `esp_hosted_set_default_config()` | Apply Kconfig defaults for the active transport |
| `eh_host_transport_get_config()` | `esp_hosted_transport_get_config()` | Read the active transport config |
| `eh_host_transport_is_config_valid()` | `esp_hosted_is_config_valid()` | Validate the current config |
| `eh_host_{sdio,spi,spi_hd,uart}_get_config` / `_set_config` | `esp_hosted_*_get/set_config` | Per-bus get/set |
| `eh_host_get_default_{sdio,spi,spi_hd,uart}_config()` | `esp_hosted_get_default_*_config()` | Per-bus defaults (also `INIT_DEFAULT_HOST_SDIO_CONFIG()`) |

Transport IDs: `EH_HOST_TRANSPORT_{NONE,SDIO,SPI_HD,SPI,UART}`.

## Core init & lifecycle
Header: `eh_host_core.h` · compat `esp_hosted.h`

| Native | Compat | Purpose |
| :--- | :--- | :--- |
| `eh_host_init(cfg)` | `esp_hosted_init()` | One-call init (vserial + base RPC + auto-init features); idempotent |
| `eh_host_connect_to_slave()` | `esp_hosted_connect_to_slave()` | Bring the transport up; posts `EH_HOST_EVENT_TRANSPORT_UP` |
| `eh_host_wait_auto_init_ready(timeout_ms)` | — | Block until constructor auto-init has finished |
| `eh_host_deinit()` | `esp_hosted_deinit()` | Reverse-order teardown; safe in any state |

Roles: `eh_host_role_t` (`MCU`, `LINUX_USER`, `LINUX_KMOD`).

## System — MAC, firmware version, CP info
Header: `eh_host_sys.h` · compat `esp_hosted_misc.h`, `esp_hosted.h`

| Native | Compat | Purpose |
| :--- | :--- | :--- |
| `eh_host_sys_get_cp_fw_version()` | `esp_hosted_get_coprocessor_fwversion()` | CP firmware version |
| `eh_host_sys_get_cp_info()` | `esp_hosted_get_cp_info()` | CP chip id + idf target name |
| `eh_host_sys_get_cp_app_desc()` | `esp_hosted_get_coprocessor_app_desc()` | CP app descriptor |
| `eh_host_iface_mac_addr_get/set()` | `esp_hosted_iface_mac_addr_get/set()` | Get/set a MAC by `esp_mac_type_t` (incl. `ESP_MAC_BT`) |
| `eh_host_iface_mac_addr_len_get()` | `esp_hosted_iface_mac_addr_len_get()` | Required MAC buffer length |

## Event bus
Header: `eh_host_event.h` · compat `esp_hosted_event.h`

Base `EH_HOST_EVENT` (compat `ESP_HOSTED_EVENT`); IDs `EH_HOST_EVENT_{CP_INIT,
CP_HEARTBEAT, TRANSPORT_UP, TRANSPORT_DOWN, TRANSPORT_FAILURE, MEM_MONITOR,
PEER_DATA_RX, NW_SPLIT_STATUS, GPIO_EXP_INT}` (each aliased `ESP_HOSTED_EVENT_*`).

## Wi-Fi (control)
Header: `eh_host_wifi.h` — no compat aliases; upstream apps call `esp_wifi_*`.

Lifecycle (`eh_host_wifi_init/deinit/start/stop/connect/disconnect/restore`),
mode/config (`set_mode`/`get_mode`, `set_config`/`get_config`, `sta_get_ap_info`),
scan (`scan_start`/`scan_stop`/`scan_get_ap_*`/`set_scan_parameters`),
power/PHY (`set_ps`/`get_ps`, `set_max_tx_power`, `set_protocol`, `set_bandwidth`,
`set_channel`, `set_country_code`, `disable_pmf_config`), AP-side
(`ap_get_sta_list`, `ap_get_sta_aid`, `deauth_sta`), and STA queries
(`sta_get_rssi`, `sta_get_aid`, `sta_get_negotiated_phymode`, `sta_twt_config`).
Dual-band (`set_band`/`get_band`, …) is gated on `EH_HOST_WIFI_DUALBAND_SUPPORT`.

**Wi-Fi extensions:** DPP (`eh_host_wifi_dpp_*`, header `eh_host_wifi_dpp.h`),
Enterprise/EAP (`eh_host_wifi_ent_*`, header `eh_host_wifi_ent.h`), and iTWT
(`eh_host_wifi_itwt_*`, header `eh_host_wifi_itwt.h`).

## Bluetooth — controller lifecycle
Header: `eh_host_bt.h`, `eh_host_feat_bt_mcu.h` · compat `esp_hosted_misc.h`

| Native | Compat | Purpose |
| :--- | :--- | :--- |
| `eh_host_bt_controller_init()` | `esp_hosted_bt_controller_init()` | Init the CP BT controller |
| `eh_host_bt_controller_enable()` | `esp_hosted_bt_controller_enable()` | Enable it |
| `eh_host_bt_controller_disable()` | `esp_hosted_bt_controller_disable()` | Disable it |
| `eh_host_bt_controller_deinit(release_mem)` | `esp_hosted_bt_controller_deinit()` | Deinit it |
| `eh_host_bt_apply_mac(mac6)` | — | Apply the BT MAC before controller bring-up |
| `eh_host_bt_mcu_hci_register(rx, ctx)` → tx fn / `_unregister()` | — | The raw H4 HCI byte-pipe (MCU host) |

## Bluetooth — host-stack adapter
Header: `esp_hosted_bt_host_stack.h` (component `examples/common_components/esp_hosted_bt_host_stack`)

The one call that binds a BT host stack (NimBLE / Bluedroid / custom) to the HCI
pipe. See [Bluetooth](features/bluetooth.md) and the [design doc](design/bluetooth.md).

| Symbol | Purpose |
| :--- | :--- |
| `esp_hosted_bt_host_stack_setup(&cfg)` | Apply MAC (opt) + controller up + bind the stack to HCI |
| `esp_hosted_bt_host_stack_teardown()` | Symmetric teardown |
| `esp_hosted_bt_host_stack_cfg_t` / `esp_hosted_bt_host_stack_t` | Config + stack selector (`NIMBLE`/`BLUEDROID`/`CUSTOM`) |
| `ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT()` / `_NIMBLE()` / `_BLUEDROID()` / `_CUSTOM(rx,ctx)` | Config presets |

## OTA — co-processor firmware update
Header: `eh_host_cp_ota.h` · compat `esp_hosted_ota.h`

| Native | Compat | Purpose |
| :--- | :--- | :--- |
| `eh_host_cp_ota_begin()` | `esp_hosted_cp_ota_begin()` / `esp_hosted_slave_ota_begin()` | Start an OTA session |
| `eh_host_cp_ota_write(buf, len)` | `esp_hosted_cp_ota_write()` | Write a chunk (≤ `EH_RPC_OTA_CHUNK_MAX`) |
| `eh_host_cp_ota_end()` | `esp_hosted_cp_ota_end()` | Finish the write phase |
| `eh_host_cp_ota_activate()` | `esp_hosted_cp_ota_activate()` | Mark image pending-boot |
| — | `esp_hosted_slave_ota(url)` | Legacy one-shot URL-driven OTA (compat-only) |

## Network-split
Header: `eh_host_nw_split.h` · compat: event aliases in `esp_hosted_event.h`

`eh_host_nw_split_bind_mac()`, `eh_host_nw_split_get_status()`,
`eh_host_nw_split_set_status()` + init/deinit. Status struct
`eh_host_nw_split_status_t` rides `EH_HOST_EVENT_NW_SPLIT_STATUS`.

## GPIO expander (CP GPIO)
Header: `eh_host_cp_gpio.h` · compat `esp_hosted_cp_gpio.h`

`eh_host_cp_gpio_config()`, `_set_level`/`_get_level`, `_set_direction`,
`_input_enable`, `_set_pull_mode`, `_reset_pin` — each aliased `esp_hosted_cp_gpio_*`.

## External coexistence
Header: `eh_host_cp_ext_coex.h` · compat `esp_hosted_cp_ext_coex.h`

`eh_host_cp_ext_coex_set_work_mode()`, `_set_gpio_pin()`, `_set_grant_delay()`,
`_set_validate_high()`, `_disable()` — each aliased `esp_hosted_cp_ext_coex_*`.

## Power-save (MCU host)
Header: `eh_host_power_save.h` · compat `esp_hosted_power_save.h`

`eh_host_power_save_start()`/`_stop()`, `_timer_start()`/`_timer_stop()`,
`_enabled()`, `_power_saving()`, `_woke_from_power_save()` — aliased `esp_hosted_power_save_*`.

## OpenThread (RCP control)
Header: `eh_host_openthread.h` · compat `esp_hosted_openthread.h`

`eh_host_openthread_rcp_init/deinit/start/stop()`, `_rcp_query()`,
`eh_host_port_openthread_get_radio_config()` — aliased `esp_hosted_openthread_*`.

## Heartbeat
Header: `eh_host_heartbeat.h` · compat `esp_hosted_misc.h`

`eh_host_heartbeat_configure(enable, duration_sec)` (compat
`esp_hosted_configure_heartbeat`). Fires `EH_HOST_EVENT_CP_HEARTBEAT`.

## Peer / custom data transfer
Header: `eh_host_peer_data.h` · compat `esp_hosted_misc.h`

`eh_host_peer_data_send(msg_id, buf, len)` (compat `esp_hosted_send_custom_data`),
`eh_host_peer_data_register(msg_id, cb, ctx)` (compat `esp_hosted_register_custom_callback`).
RX rides `EH_HOST_EVENT_PEER_DATA_RX`.

## Memory monitor
Header: `eh_host_mem_monitor.h` · compat `esp_hosted_misc.h`

`eh_host_set_mem_monitor(cfg, out)` (compat `esp_hosted_set_mem_monitor`). Fires
`EH_HOST_EVENT_MEM_MONITOR`.

## CLI (console commands)
Header: `eh_host_cli.h`

`eh_host_feat_cli_register_commands()` — register per-feature console commands
(call after `esp_console_new_repl_*`).

---

## Advanced / manual surfaces

- **Per-feature `eh_host_feat_<x>_init/deinit()`** exist in every feature header
  but are normally driven by the core auto-init path (`eh_host_init` +
  `eh_host_wait_auto_init_ready`). Call them directly only for manual control.
- **Base RPC engine** (`eh_host_feat_rpc.h`) — `eh_host_feat_rpc_request_sync()`,
  `_send()`, `_register_event()`, etc. Native-only; used by features and advanced
  apps that add their own RPC messages.

## Notes

- **The BT host-stack adapter ships under `examples/`**, not `host/`, because it
  pulls in `idf::bt`; the core `esp_hosted` component stays stack-agnostic. It is
  still the intended app-facing BT entry point.
- **Enterprise/EAP and iTWT expose two coexisting name-sets** — the umbrella
  wrapper names (`eh_host_wifi_{ent,itwt}_*`, primary) and the upstream-shaped
  `eh_host_wifi_sta_{eap,itwt}_*` names in the feature headers.
- **Compat-only functions** with no native twin: `esp_hosted_tx()`,
  `esp_hosted_slave_ota(url)`.
- Types generally have a native `eh_host_*` name and an `esp_hosted_*` compat
  alias; enum tokens follow the same pattern (`EH_HOST_*` ↔ `ESP_HOSTED_*`).
