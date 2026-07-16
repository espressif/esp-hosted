"""Wi-Fi DPP — Device Provisioning Protocol bootstrap helpers.

Sync RPCs only.  The bespoke single-slot subscribe APIs
(`eh_host_wifi_dpp_subscribe_*`) aren't wrapped here today; for
Python event reception use `eh_host.event` to register against
WIFI_EVENT / WIFI_EVENT_DPP_* (the same event flow IDF apps see).
"""

from ctypes import (
    CFUNCTYPE,
    c_char_p,
    c_int,
    c_uint32,
    c_void_p,
)

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


# ── DPP bootstrap types (mirror esp_supp_dpp_bootstrap_t) ─────────
DPP_BOOTSTRAP_QR_CODE  = 0
DPP_BOOTSTRAP_PKEX     = 1
DPP_BOOTSTRAP_NFC_URI  = 2

# ── DPP event ids passed to the lifecycle callback ───────────────
DPP_URI_READY        = 0
DPP_CFG_RECEIVED     = 1
DPP_FAIL             = 2

# CFUNCTYPE for the lifecycle callback passed to dpp_init.
# void (*)(esp_supp_dpp_event_t evt, void *data)
DppEventCb = CFUNCTYPE(None, c_int, c_void_p)


_init          = _opt_bind("eh_host_wifi_dpp_init",          [DppEventCb], c_int)
_deinit        = _opt_bind("eh_host_wifi_dpp_deinit",        [],           c_int)
_start_listen  = _opt_bind("eh_host_wifi_dpp_start_listen",  [],           c_int)
_stop_listen   = _opt_bind("eh_host_wifi_dpp_stop_listen",   [],           c_int)
_bootstrap_gen = _opt_bind("eh_host_wifi_dpp_bootstrap_gen",
                           [c_char_p, c_uint32, c_char_p, c_char_p], c_int)


# Keep CFUNCTYPE thunks alive for as long as a callback is registered.
_LIVE_CBS: list = []


def init(evt_cb=None) -> None:
    """`evt_cb` is `f(event_id, data_ptr)` — wrapped via CFUNCTYPE.
    Pass None for no-callback init."""
    if evt_cb is None:
        rc = _must(_init, "eh_host_wifi_dpp_init", DppEventCb())
    else:
        thunk = DppEventCb(evt_cb)
        _LIVE_CBS.append(thunk)
        rc = _must(_init, "eh_host_wifi_dpp_init", thunk)
    if rc != 0:
        raise EhHostError("eh_host_wifi_dpp_init", rc)


def deinit() -> None:
    rc = _must(_deinit, "eh_host_wifi_dpp_deinit")
    if rc != 0:
        raise EhHostError("eh_host_wifi_dpp_deinit", rc)
    _LIVE_CBS.clear()


def start_listen() -> None:
    rc = _must(_start_listen, "eh_host_wifi_dpp_start_listen")
    if rc != 0:
        raise EhHostError("eh_host_wifi_dpp_start_listen", rc)


def stop_listen() -> None:
    rc = _must(_stop_listen, "eh_host_wifi_dpp_stop_listen")
    if rc != 0:
        raise EhHostError("eh_host_wifi_dpp_stop_listen", rc)


def bootstrap_gen(chan_list: bytes,
                  bootstrap_type: int = DPP_BOOTSTRAP_QR_CODE,
                  key: bytes = b"",
                  info: bytes = b"") -> None:
    if isinstance(chan_list, str):
        chan_list = chan_list.encode()
    if isinstance(key, str):
        key = key.encode()
    if isinstance(info, str):
        info = info.encode()
    rc = _must(_bootstrap_gen, "eh_host_wifi_dpp_bootstrap_gen",
               chan_list, bootstrap_type, key, info)
    if rc != 0:
        raise EhHostError("eh_host_wifi_dpp_bootstrap_gen", rc)
