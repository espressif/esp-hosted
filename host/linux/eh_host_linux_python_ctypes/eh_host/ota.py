from ctypes import c_int, c_uint32, c_uint8, POINTER

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


_begin    = _opt_bind("eh_host_cp_ota_begin",    [],                              c_int)
_write    = _opt_bind("eh_host_cp_ota_write",    [POINTER(c_uint8), c_uint32],    c_int)
_end      = _opt_bind("eh_host_cp_ota_end",      [],                              c_int)
_activate = _opt_bind("eh_host_cp_ota_activate", [],                              c_int)


def begin() -> None:
    rc = _must(_begin, "eh_host_cp_ota_begin")
    if rc != 0:
        raise EhHostError("eh_host_cp_ota_begin", rc)


def write(data: bytes) -> None:
    buf = (c_uint8 * len(data))(*data)
    rc = _must(_write, "eh_host_cp_ota_write", buf, len(data))
    if rc != 0:
        raise EhHostError("eh_host_cp_ota_write", rc)


def end() -> None:
    rc = _must(_end, "eh_host_cp_ota_end")
    if rc != 0:
        raise EhHostError("eh_host_cp_ota_end", rc)


def activate() -> None:
    rc = _must(_activate, "eh_host_cp_ota_activate")
    if rc != 0:
        raise EhHostError("eh_host_cp_ota_activate", rc)
