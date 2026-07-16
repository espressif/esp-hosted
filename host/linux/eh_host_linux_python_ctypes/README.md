# `host/linux/eh_host_linux_python_ctypes/`

Packaging slot for the Python `ctypes` binding to ESP-Hosted host.
Per `system/verified/build_model.md` §5, a binding component sits on
top of the per-feature CMake targets and consumes the SHARED
packaging target underneath.

## Layout

```
host/linux/eh_host_linux_python_ctypes/
├── CMakeLists.txt         # SHARED add_library(eh_host) — slot, not yet active
├── build_libeh_host.sh    # raw-cc build of libeh_host.so (active today)
├── eh_host.lds            # linker version script (export filter)
└── README.md              # this file
```

## What's here today

- **`build_libeh_host.sh`** — produces
  `${TMPDIR:-/tmp}/eh_libeh_host/libeh_host.so` from the same source
  tree the example linux_host build.sh scripts use, with `-fPIC` and
  `-Wl,--version-script=eh_host.lds`.  Mirrors the existing raw-cc
  pattern (build_model.md §2 "Linux user-space build") because the
  linux_user tree has no top-level CMake project yet.
- **`eh_host.lds`** — visibility filter.  Only symbols matching
  `eh_host_*` are exported (the documented public C ABI from each
  feature's public header).  Everything else (`eh_port_*`, `eh_rpc_*`,
  `eh_frame_*`, `eh_mempool_*`, `protobuf_c_*`, IDF compat shims) is
  collapsed to local visibility.
- **`CMakeLists.txt`** — spec-aligned SHARED target sketch.  Inactive
  today (early-returns); becomes the integration point once a
  top-level linux_user CMake project lands.

## Python package — `eh_host`

Per build_model.md §5, per-feature modules declare ctypes
argtypes/restype directly against the SHARED `libeh_host.so`.
No upstream-FG-style `commands_map_py_to_c.py` triple.

```
host/linux/eh_host_linux_python_ctypes/
├── pyproject.toml            # package metadata
├── eh_host/
│   ├── __init__.py           # public API re-export
│   ├── _lib.py               # CDLL load (single-point)
│   ├── _types.py             # esp_err_t / common types
│   ├── loader.py             # find libeh_host.so (env / dev / system)
│   ├── core.py               # eh_host_init / eh_host_deinit
│   └── system.py             # eh_host_sys_get_cp_fw_version / get_mac / set_mac
```

What's wrapped today:
  - `core` — init / deinit (typed `init(role=…)` and
    `init_linux_default()`)
  - `system` — `get_cp_fw_version()`, `get_mac()`, `set_mac()`,
    `config_heartbeat()` (FwVer struct mirrors
    `esp_hosted_coprocessor_fwver_t`)
  - `event` — IDF event-loop bridge.  `loop_create_default()` /
    `loop_delete_default()`, `handler_register(base, id, py_cb)` /
    `handler_unregister`, event-base data globals
    (`ESP_HOSTED_EVENT`, `WIFI_EVENT`, `IP_EVENT`), id constants
    (`ESP_HOSTED_EVENT_CP_INIT`, `_CP_HEARTBEAT`, `_PEER_DATA_RX`,
    `_NW_SPLIT_STATUS`, `_GPIO_EXP_INT`, `_MEM_MONITOR`), plus
    typed payload structs (`HeartbeatPayload`, `CpInitPayload`,
    `PeerDataPayload`, `NwSplitStatusPayload`, `MemMonitorPayload`)
    and a `cast_payload(ptr, struct_t)` helper.
  - `wifi` — STA lifecycle (init/start/stop/connect/disconnect),
    set_mode / set_config / get_mode / get_config / restore,
    scan_start / scan_stop / scan_get_ap_num / clear_ap_list,
    sta_get_ap_info, set_ps / get_ps, set_max_tx_power /
    get_max_tx_power, set_country_code / get_country_code, plus
    `make_sta_config(ssid, password, authmode)`.
  - `bt`, `peer_data`, `gpio_exp`, `ext_coex`, `ota`, `nw_split`,
    `heartbeat`, `mem_monitor`, `wifi_dpp`, `wifi_ent`,
    `wifi_itwt` — see each module's docstring for scope.

What's not wrapped yet (tracked in `.meta/specs/tasks/todo.md`):
  - Upstream FG `host_control/python_support` parity — softAP APIs,
    country DHCP/DNS controls, vendor IE.  Need C-side wrappers
    first.
  - Advanced WiFi extension structs (cert-and-key pair, EAP-TTLS
    phase-2, EAP-FAST config, iTWT setup_config) inside
    `wifi_ent.py / wifi_dpp.py / wifi_itwt.py`.

## Mirror demo

`examples/system/linux_host/main.py` is the Python mirror of the C
demo `main.c` — full parity, including the event-driven heartbeat
subscription path.

## Build

```bash
./build_libeh_host.sh
```

Produces `${TMPDIR:-/tmp}/eh_libeh_host/libeh_host.so`.
Verifies via `nm -D --defined-only` that no non-`eh_host_*` symbols
leaked through the version script.

## Smoke-test the .so via the package

```bash
# 1. build the .so
./build_libeh_host.sh

# 2. invoke a synchronous call (needs CP up + /dev/esps0)
PYTHONPATH=$(pwd) python3 - <<'PY'
import eh_host
eh_host.init_linux_default()
print(eh_host.get_cp_fw_version())
print(":".join(f"{b:02x}" for b in eh_host.get_mac()))
eh_host.deinit()
PY
```

Or run the mirror demo:

```bash
PYTHONPATH=/path/to/repo/host/linux/eh_host_linux_python_ctypes \
    python3 /path/to/repo/examples/system/linux_host/main.py
```

## Library lookup order

`eh_host.loader.find_libeh_host()` resolves in this order:
  1. `EH_HOST_LIB` env var (explicit override)
  2. `${TMPDIR:-/tmp}/eh_libeh_host/libeh_host.so` (developer build
     output of `build_libeh_host.sh`)
  3. Standard ld.so search path (`/usr/lib`, `/usr/local/lib`, …)

## Filesystem placement

The `.so` lives in `${TMPDIR:-/tmp}/eh_libeh_host/` to match the
existing example build pattern (no in-tree artifacts).  When a
distributable package shape is needed the Python wrapper task will
add the standard `data_files` / `package_data` plumbing.
