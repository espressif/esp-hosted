"""System feature — CP firmware version, MAC get/set, heartbeat config.

Wraps the C surface in
`host/features/eh_host_feat_rpc/include/eh_host_sys.h`.

Async heartbeat events flow through `eh_host.event` — register a
handler against ESP_HOSTED_EVENT / ESP_HOSTED_EVENT_CP_HEARTBEAT and
cast the payload to `eh_host.event.HeartbeatPayload`.  See
`examples/system/linux_host/main.py` for the full pattern.
"""

from ctypes import (
    Structure,
    c_bool,
    c_int,
    c_int32,
    c_uint32,
    c_uint8,
    POINTER,
    byref,
)

from eh_host._lib import _LIB, _opt_bind, _must
from eh_host.core import EhHostError


# ---- structs (mirror eh_host_feat_system.h) ------------------------

class FwVer(Structure):
    """Mirror of `esp_hosted_coprocessor_fwver_t`."""

    _fields_ = [
        ("major1",     c_uint32),
        ("minor1",     c_uint32),
        ("patch1",     c_uint32),
        ("revision",   c_int32),
        ("prerelease", c_int32),
        ("build",      c_int32),
    ]

    def __repr__(self) -> str:
        return (
            f"FwVer({self.major1}.{self.minor1}.{self.patch1}, "
            f"revision={self.revision}, prerelease={self.prerelease}, "
            f"build={self.build})"
        )


# ---- wifi_interface_t enum (mirrors esp_wifi_types.h) --------------

WIFI_IF_STA = 0
WIFI_IF_AP  = 1


# ---- ctypes signatures --------------------------------------------

_get_fwver = _LIB.eh_host_sys_get_cp_fw_version
_get_fwver.argtypes = [POINTER(FwVer)]
_get_fwver.restype  = c_int  # esp_err_t

_get_mac = _LIB.eh_host_sys_get_mac
_get_mac.argtypes = [c_int, POINTER(c_uint8 * 6)]
_get_mac.restype  = c_int

_set_mac = _LIB.eh_host_sys_set_mac
_set_mac.argtypes = [c_int, POINTER(c_uint8 * 6)]
_set_mac.restype  = c_int

_config_heartbeat = _opt_bind("eh_host_heartbeat_configure", [c_bool, c_int], c_int)


# ---- public Pythonic wrappers --------------------------------------

def get_cp_fw_version() -> FwVer:
    """Return the CP's firmware version as an `FwVer` struct."""
    fw = FwVer()
    rc = _get_fwver(byref(fw))
    if rc != 0:
        raise EhHostError("eh_host_sys_get_cp_fw_version", rc)
    return fw


def get_mac(iface: int = WIFI_IF_STA) -> bytes:
    """Read the CP's MAC for `iface` (WIFI_IF_STA / WIFI_IF_AP)."""
    buf = (c_uint8 * 6)()
    rc = _get_mac(iface, byref(buf))
    if rc != 0:
        raise EhHostError("eh_host_sys_get_mac", rc)
    return bytes(buf)


def set_mac(mac: bytes, iface: int = WIFI_IF_STA) -> None:
    """Set the CP's MAC for `iface`.  `mac` must be 6 bytes."""
    if len(mac) != 6:
        raise ValueError(f"mac must be 6 bytes; got {len(mac)}")
    buf = (c_uint8 * 6)(*mac)
    rc = _set_mac(iface, byref(buf))
    if rc != 0:
        raise EhHostError("eh_host_sys_set_mac", rc)


def config_heartbeat(enable: bool, duration_sec: int) -> None:
    """Enable or disable the CP-side heartbeat ticker.

    When `enable=True`, the CP emits a heartbeat event every
    `duration_sec` seconds against ESP_HOSTED_EVENT /
    ESP_HOSTED_EVENT_CP_HEARTBEAT.  Subscribe via
    `eh_host.event.handler_register(ESP_HOSTED_EVENT,
    ESP_HOSTED_EVENT_CP_HEARTBEAT, on_heartbeat)`.
    """
    rc = _must(_config_heartbeat, "eh_host_heartbeat_configure", enable, duration_sec)
    if rc != 0:
        raise EhHostError("eh_host_heartbeat_configure", rc)
