from ctypes import (
    POINTER,
    byref,
    c_int,
)

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


_teardown        = _opt_bind("eh_host_wifi_itwt_teardown",                     [c_int],          c_int)
_suspend         = _opt_bind("eh_host_wifi_itwt_suspend",                      [c_int, c_int],   c_int)
_send_probe_req  = _opt_bind("eh_host_wifi_itwt_send_probe_req",               [c_int],          c_int)
_set_offset      = _opt_bind("eh_host_wifi_itwt_set_target_wake_time_offset",  [c_int],          c_int)
_get_flow_status = _opt_bind("eh_host_wifi_itwt_get_flow_id_status",           [POINTER(c_int)], c_int)


def teardown(flow_id: int) -> None:
    rc = _must(_teardown, "eh_host_wifi_itwt_teardown", flow_id)
    if rc != 0:
        raise EhHostError("eh_host_wifi_itwt_teardown", rc)


def suspend(flow_id: int, suspend_time_ms: int) -> None:
    rc = _must(_suspend, "eh_host_wifi_itwt_suspend", flow_id, suspend_time_ms)
    if rc != 0:
        raise EhHostError("eh_host_wifi_itwt_suspend", rc)


def send_probe_req(timeout_ms: int) -> None:
    rc = _must(_send_probe_req, "eh_host_wifi_itwt_send_probe_req", timeout_ms)
    if rc != 0:
        raise EhHostError("eh_host_wifi_itwt_send_probe_req", rc)


def set_target_wake_time_offset(offset_us: int) -> None:
    rc = _must(_set_offset, "eh_host_wifi_itwt_set_target_wake_time_offset", offset_us)
    if rc != 0:
        raise EhHostError("eh_host_wifi_itwt_set_target_wake_time_offset", rc)


def get_flow_id_status() -> int:
    out = c_int(0)
    rc = _must(_get_flow_status, "eh_host_wifi_itwt_get_flow_id_status", byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_itwt_get_flow_id_status", rc)
    return out.value
