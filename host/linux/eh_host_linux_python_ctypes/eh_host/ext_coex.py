from ctypes import (
    POINTER,
    Structure,
    byref,
    c_bool,
    c_int,
    c_int32,
    c_uint8,
    c_uint32,
)

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


LEADER_ROLE   = 0
FOLLOWER_ROLE = 2
UNKNOWN_ROLE  = 3

WIRE_1 = 0
WIRE_2 = 1
WIRE_3 = 2
WIRE_4 = 3


class GpioSet(Structure):
    _fields_ = [
        ("request",  c_int32),
        ("priority", c_int32),
        ("grant",    c_int32),
        ("tx_line",  c_int32),
    ]


_set_work_mode     = _opt_bind("eh_host_cp_ext_coex_set_work_mode",     [c_int],                       c_int)
_set_gpio_pin      = _opt_bind("eh_host_cp_ext_coex_set_gpio_pin",      [c_uint32, POINTER(GpioSet)],  c_int)
_set_grant_delay   = _opt_bind("eh_host_cp_ext_coex_set_grant_delay",   [c_uint8],                     c_int)
_set_validate_high = _opt_bind("eh_host_cp_ext_coex_set_validate_high", [c_bool],                      c_int)
_disable           = _opt_bind("eh_host_cp_ext_coex_disable",           [],                            c_int)


def set_work_mode(mode: int) -> None:
    rc = _must(_set_work_mode, "eh_host_cp_ext_coex_set_work_mode", mode)
    if rc != 0:
        raise EhHostError("eh_host_cp_ext_coex_set_work_mode", rc)


def set_gpio_pin(wire_type: int, gpio_pins: GpioSet) -> None:
    rc = _must(_set_gpio_pin, "eh_host_cp_ext_coex_set_gpio_pin", wire_type, byref(gpio_pins))
    if rc != 0:
        raise EhHostError("eh_host_cp_ext_coex_set_gpio_pin", rc)


def set_grant_delay(delay_us: int) -> None:
    rc = _must(_set_grant_delay, "eh_host_cp_ext_coex_set_grant_delay", delay_us)
    if rc != 0:
        raise EhHostError("eh_host_cp_ext_coex_set_grant_delay", rc)


def set_validate_high(is_high_valid: bool) -> None:
    rc = _must(_set_validate_high, "eh_host_cp_ext_coex_set_validate_high", is_high_valid)
    if rc != 0:
        raise EhHostError("eh_host_cp_ext_coex_set_validate_high", rc)


def disable() -> None:
    rc = _must(_disable, "eh_host_cp_ext_coex_disable")
    if rc != 0:
        raise EhHostError("eh_host_cp_ext_coex_disable", rc)
