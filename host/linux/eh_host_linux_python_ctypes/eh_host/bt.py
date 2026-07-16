from ctypes import c_bool, c_int

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


_init    = _opt_bind("eh_host_bt_controller_init",    [],       c_int)
_deinit  = _opt_bind("eh_host_bt_controller_deinit",  [c_bool], c_int)
_enable  = _opt_bind("eh_host_bt_controller_enable",  [],       c_int)
_disable = _opt_bind("eh_host_bt_controller_disable", [],       c_int)


def controller_init() -> None:
    rc = _must(_init, "eh_host_bt_controller_init")
    if rc != 0:
        raise EhHostError("eh_host_bt_controller_init", rc)


def controller_deinit(release_memory: bool = False) -> None:
    rc = _must(_deinit, "eh_host_bt_controller_deinit", release_memory)
    if rc != 0:
        raise EhHostError("eh_host_bt_controller_deinit", rc)


def controller_enable() -> None:
    rc = _must(_enable, "eh_host_bt_controller_enable")
    if rc != 0:
        raise EhHostError("eh_host_bt_controller_enable", rc)


def controller_disable() -> None:
    rc = _must(_disable, "eh_host_bt_controller_disable")
    if rc != 0:
        raise EhHostError("eh_host_bt_controller_disable", rc)
