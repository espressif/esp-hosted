"""Host bring-up / tear-down — wraps `eh_host_init*` / `eh_host_deinit`.

Mirrors the C surface in
`host/eh_host_core/include/eh_host_core.h`:
  - `eh_host_init(const eh_host_init_cfg_t *)` — typed init
  - `eh_host_init_linux_default()` — convenience wrapper for Linux
  - `eh_host_deinit()`
"""

from ctypes import (
    Structure,
    c_char_p,
    c_int,
    c_uint32,
    POINTER,
    byref,
)

from eh_host._lib import _LIB


class EhHostError(RuntimeError):
    """Raised when an `eh_host_*` call returns a non-zero status."""

    def __init__(self, op: str, rc: int):
        super().__init__(f"{op} failed: rc={rc}")
        self.op = op
        self.rc = rc


# ---- enums + structs (mirror eh_host_core.h) -----------------------

ROLE_MCU        = 0
ROLE_LINUX_USER = 1
ROLE_LINUX_KMOD = 2


class _HostInitCfg(Structure):
    _fields_ = [
        ("role",                  c_int),
        ("vserial_device_path",   c_char_p),
        ("flags",                 c_uint32),
    ]


# ---- ctypes signatures --------------------------------------------

_eh_host_init = _LIB.eh_host_init
_eh_host_init.argtypes = [POINTER(_HostInitCfg)]
_eh_host_init.restype  = c_int

_eh_host_init_linux_default = _LIB.eh_host_init_linux_default
_eh_host_init_linux_default.argtypes = []
_eh_host_init_linux_default.restype  = c_int

_eh_host_deinit = _LIB.eh_host_deinit
_eh_host_deinit.argtypes = []
_eh_host_deinit.restype  = c_int


# ---- public Pythonic wrappers --------------------------------------

def init(role: int = ROLE_LINUX_USER, vserial_device_path: str | None = None) -> None:
    """Bring the host stack up.

    `role` defaults to `ROLE_LINUX_USER` since the .so is built for
    Linux user-space.  `vserial_device_path` is honoured only on
    Linux (NULL == default `/dev/esps0`).

    Raises `EhHostError` on non-zero return.
    """
    cfg = _HostInitCfg(
        role=role,
        vserial_device_path=vserial_device_path.encode()
            if vserial_device_path else None,
        flags=0,
    )
    rc = _eh_host_init(byref(cfg))
    if rc != 0:
        raise EhHostError("eh_host_init", rc)


def init_linux_default() -> None:
    """Linux-user convenience — equivalent to `init(ROLE_LINUX_USER)`."""
    rc = _eh_host_init_linux_default()
    if rc != 0:
        raise EhHostError("eh_host_init_linux_default", rc)


def deinit() -> None:
    """Tear the host stack down.  Safe even if init never succeeded."""
    rc = _eh_host_deinit()
    if rc != 0:
        raise EhHostError("eh_host_deinit", rc)
