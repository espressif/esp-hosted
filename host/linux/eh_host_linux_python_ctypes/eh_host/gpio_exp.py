from ctypes import (
    POINTER,
    Structure,
    byref,
    c_int,
    c_uint32,
    c_uint64,
)

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


GPIO_MODE_DISABLE   = 0
GPIO_MODE_INPUT     = 1 << 0
GPIO_MODE_OUTPUT    = 1 << 1
GPIO_MODE_OD        = 1 << 2
GPIO_MODE_OUTPUT_OD = GPIO_MODE_OUTPUT | GPIO_MODE_OD


class GpioConfig(Structure):
    _fields_ = [
        ("pin_bit_mask",  c_uint64),
        ("mode",          c_uint32),
        ("pull_up_en",    c_uint32),
        ("pull_down_en",  c_uint32),
        ("intr_type",     c_uint32),
    ]


_config        = _opt_bind("eh_host_cp_gpio_config",         [POINTER(GpioConfig)],   c_int)
_reset_pin     = _opt_bind("eh_host_cp_gpio_reset_pin",      [c_uint32],              c_int)
_set_level     = _opt_bind("eh_host_cp_gpio_set_level",      [c_uint32, c_uint32],    c_int)
_get_level     = _opt_bind("eh_host_cp_gpio_get_level",      [c_uint32, POINTER(c_int)], c_int)
_set_direction = _opt_bind("eh_host_cp_gpio_set_direction",  [c_uint32, c_uint32],    c_int)
_input_enable  = _opt_bind("eh_host_cp_gpio_input_enable",   [c_uint32],              c_int)
_set_pull_mode = _opt_bind("eh_host_cp_gpio_set_pull_mode",  [c_uint32, c_uint32],    c_int)


def config(cfg: GpioConfig) -> None:
    rc = _must(_config, "eh_host_cp_gpio_config", byref(cfg))
    if rc != 0:
        raise EhHostError("eh_host_cp_gpio_config", rc)


def reset_pin(gpio_num: int) -> None:
    rc = _must(_reset_pin, "eh_host_cp_gpio_reset_pin", gpio_num)
    if rc != 0:
        raise EhHostError("eh_host_cp_gpio_reset_pin", rc)


def set_level(gpio_num: int, level: int) -> None:
    rc = _must(_set_level, "eh_host_cp_gpio_set_level", gpio_num, level)
    if rc != 0:
        raise EhHostError("eh_host_cp_gpio_set_level", rc)


def get_level(gpio_num: int) -> int:
    out = c_int(0)
    rc = _must(_get_level, "eh_host_cp_gpio_get_level", gpio_num, byref(out))
    if rc != 0:
        raise EhHostError("eh_host_cp_gpio_get_level", rc)
    return out.value


def set_direction(gpio_num: int, mode: int) -> None:
    rc = _must(_set_direction, "eh_host_cp_gpio_set_direction", gpio_num, mode)
    if rc != 0:
        raise EhHostError("eh_host_cp_gpio_set_direction", rc)


def input_enable(gpio_num: int) -> None:
    rc = _must(_input_enable, "eh_host_cp_gpio_input_enable", gpio_num)
    if rc != 0:
        raise EhHostError("eh_host_cp_gpio_input_enable", rc)


def set_pull_mode(gpio_num: int, pull_mode: int) -> None:
    rc = _must(_set_pull_mode, "eh_host_cp_gpio_set_pull_mode", gpio_num, pull_mode)
    if rc != 0:
        raise EhHostError("eh_host_cp_gpio_set_pull_mode", rc)
