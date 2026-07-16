"""Wi-Fi Enterprise (WPA2/WPA3-Enterprise) — credential setters.

Wraps the byte-buffer setters (identity / username / password / new
password / CA cert) plus enable / disable.  The more complex
helpers (cert-and-key pair, EAP-TTLS phase-2 method, EAP-FAST
config) take IDF-side struct types and aren't exposed here today —
add as Python ctypes mirrors when an enterprise demo lands.
"""

from ctypes import (
    POINTER,
    c_int,
    c_uint8,
)

from eh_host._lib import _LIB
from eh_host.core import EhHostError


def _wrap_buf_setter(name: str):
    fn = getattr(_LIB, name)
    fn.argtypes = [POINTER(c_uint8), c_int]
    fn.restype  = c_int
    def call(data: bytes) -> None:
        if isinstance(data, str):
            data = data.encode()
        buf = (c_uint8 * len(data))(*data)
        rc = fn(buf, len(data))
        if rc != 0:
            raise EhHostError(name, rc)
    return call


def _wrap_void_returning(name: str):
    """Wrap an esp_err_t-returning argless C function (returns int)."""
    fn = getattr(_LIB, name)
    fn.argtypes = []
    fn.restype  = c_int
    def call() -> None:
        rc = fn()
        if rc != 0:
            raise EhHostError(name, rc)
    return call


def _wrap_void(name: str):
    """Wrap a true-void argless C function (no return value).

    Mirrors the upstream esp_eap_client_clear_* signatures (void(void)).
    """
    fn = getattr(_LIB, name)
    fn.argtypes = []
    fn.restype  = None
    def call() -> None:
        fn()
    return call


# ── Public API — function objects defined via the wrappers above. ──
enterprise_enable      = _wrap_void_returning("eh_host_wifi_ent_enterprise_enable")
enterprise_disable     = _wrap_void_returning("eh_host_wifi_ent_enterprise_disable")

set_identity           = _wrap_buf_setter("eh_host_wifi_ent_set_identity")
clear_identity         = _wrap_void("eh_host_wifi_ent_clear_identity")

set_username           = _wrap_buf_setter("eh_host_wifi_ent_set_username")
clear_username         = _wrap_void("eh_host_wifi_ent_clear_username")

set_password           = _wrap_buf_setter("eh_host_wifi_ent_set_password")
clear_password         = _wrap_void("eh_host_wifi_ent_clear_password")

set_new_password       = _wrap_buf_setter("eh_host_wifi_ent_set_new_password")
clear_new_password     = _wrap_void("eh_host_wifi_ent_clear_new_password")

set_ca_cert            = _wrap_buf_setter("eh_host_wifi_ent_set_ca_cert")
clear_ca_cert          = _wrap_void("eh_host_wifi_ent_clear_ca_cert")

set_pac_file           = _wrap_buf_setter("eh_host_wifi_ent_set_pac_file")
clear_certificate_and_key = _wrap_void(
    "eh_host_wifi_ent_clear_certificate_and_key")
