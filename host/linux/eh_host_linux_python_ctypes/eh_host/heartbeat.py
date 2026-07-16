from ctypes import c_bool, c_int

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


_configure = _opt_bind("eh_host_heartbeat_configure", [c_bool, c_int], c_int)


def configure(enable: bool, duration_sec: int = 0) -> None:
    rc = _must(_configure, "eh_host_heartbeat_configure", enable, duration_sec)
    if rc != 0:
        raise EhHostError("eh_host_heartbeat_configure", rc)
