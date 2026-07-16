"""Memory monitor — CP-side periodic heap-watermark reporter.

`set_mem_monitor(config, return_curr=True)` drives the CP's
mem-monitor synchronously; signature mirrors upstream
`esp_hosted_set_mem_monitor(config, curr_mem_info)`.  Inbound
mem-monitor events ride the host event base (slot
``ESP_HOSTED_EVENT_MEM_MONITOR``) on the IDF default loop;
subscribe via `eh_host.event` to receive them.
"""

from ctypes import POINTER, Structure, byref, c_bool, c_int, c_uint32

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


# ── enums (mirror upstream esp_hosted_misc_types.h) ────────────────
ESP_HOSTED_MEMMONITOR_NO_CHANGE = 0
ESP_HOSTED_MEMMONITOR_DISABLE   = 1
ESP_HOSTED_MEMMONITOR_ENABLE    = 2


# ── struct mirrors (upstream esp_hosted_misc_types.h:19-51) ────────

class _Threshold(Structure):
    """Mirror of `esp_hosted_mem_monitor_threshold_t`."""

    _fields_ = [
        ("threshold_mem_dma",  c_uint32),
        ("threshold_mem_8bit", c_uint32),
    ]


class ConfigMemMonitor(Structure):
    """Mirror of `esp_hosted_config_mem_monitor_t` (configure-Req payload)."""

    _fields_ = [
        ("config",        c_int),         # esp_hosted_mem_monitor_config_t
        ("report_always", c_bool),
        ("interval_sec",  c_uint32),
        ("internal_mem",  _Threshold),
        ("external_mem",  _Threshold),
    ]


class _MemInfo(Structure):
    """Mirror of `esp_hosted_mem_info_t`."""

    _fields_ = [
        ("free_size",          c_uint32),
        ("largest_free_block", c_uint32),
    ]


class _CapInfo(Structure):
    """Mirror of `esp_hosted_cap_info_t`."""

    _fields_ = [
        ("cap_dma",  _MemInfo),
        ("cap_8bit", _MemInfo),
    ]


class CurrMemInfo(Structure):
    """Mirror of `esp_hosted_curr_mem_info_t` (configure-Resp payload)."""

    _fields_ = [
        ("config",               c_int),
        ("report_always",        c_bool),
        ("interval_sec",         c_uint32),
        ("curr_total_heap_size", c_uint32),
        ("curr_internal",        _CapInfo),
        ("curr_external",        _CapInfo),
    ]


# ── ctypes signatures ──────────────────────────────────────────────

_register        = _opt_bind("eh_host_mem_monitor_register_event_handlers",   [], c_int)
_unregister      = _opt_bind("eh_host_mem_monitor_unregister_event_handlers", [], c_int)
_set_mem_monitor = _opt_bind("eh_host_set_mem_monitor",
                             [POINTER(ConfigMemMonitor), POINTER(CurrMemInfo)], c_int)


# ── public API ─────────────────────────────────────────────────────

def register_event_handlers() -> None:
    rc = _must(_register, "eh_host_mem_monitor_register_event_handlers")
    if rc != 0:
        raise EhHostError("eh_host_mem_monitor_register_event_handlers", rc)


def unregister_event_handlers() -> None:
    rc = _must(_unregister, "eh_host_mem_monitor_unregister_event_handlers")
    if rc != 0:
        raise EhHostError("eh_host_mem_monitor_unregister_event_handlers", rc)


def set_mem_monitor(config: ConfigMemMonitor,
                    return_curr: bool = True) -> dict | None:
    """Configure the CP-side mem-monitor and (optionally) read back the
    current heap snapshot in one round-trip.

    `config` is a populated `ConfigMemMonitor` ctypes Structure.
    Build it with the field setters (config / report_always /
    interval_sec / internal_mem.threshold_mem_dma / ...).

    If `return_curr` is True (default), returns a dict mirroring
    `esp_hosted_curr_mem_info_t` — the CP's echo of the applied config
    plus a heap snapshot at the moment of the request.

    Raises `EhHostError` on transport or CP-side error.
    """
    if not isinstance(config, ConfigMemMonitor):
        raise TypeError("config must be a ConfigMemMonitor instance")
    curr = CurrMemInfo() if return_curr else None
    rc = _must(_set_mem_monitor, "eh_host_set_mem_monitor",
               byref(config), byref(curr) if curr else None)
    if rc != 0:
        raise EhHostError("eh_host_set_mem_monitor", rc)
    if not return_curr:
        return None
    return {
        "config":               curr.config,
        "report_always":        bool(curr.report_always),
        "interval_sec":         curr.interval_sec,
        "curr_total_heap_size": curr.curr_total_heap_size,
        "curr_internal": {
            "cap_dma":  {"free_size": curr.curr_internal.cap_dma.free_size,
                         "largest_free_block": curr.curr_internal.cap_dma.largest_free_block},
            "cap_8bit": {"free_size": curr.curr_internal.cap_8bit.free_size,
                         "largest_free_block": curr.curr_internal.cap_8bit.largest_free_block},
        },
        "curr_external": {
            "cap_dma":  {"free_size": curr.curr_external.cap_dma.free_size,
                         "largest_free_block": curr.curr_external.cap_dma.largest_free_block},
            "cap_8bit": {"free_size": curr.curr_external.cap_8bit.free_size,
                         "largest_free_block": curr.curr_external.cap_8bit.largest_free_block},
        },
    }
