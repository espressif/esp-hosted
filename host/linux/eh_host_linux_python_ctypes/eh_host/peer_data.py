from ctypes import c_int, c_size_t, c_uint32, c_uint8, POINTER

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError


_send       = _opt_bind("eh_host_peer_data_send",
                        [c_uint32, POINTER(c_uint8), c_size_t], c_int)
_register   = _opt_bind("eh_host_peer_data_register_event_handlers",   [], c_int)
_unregister = _opt_bind("eh_host_peer_data_unregister_event_handlers", [], c_int)


def send(msg_id: int, data: bytes) -> None:
    buf = (c_uint8 * len(data))(*data)
    rc = _must(_send, "eh_host_peer_data_send", msg_id, buf, len(data))
    if rc != 0:
        raise EhHostError("eh_host_peer_data_send", rc)


def register_event_handlers() -> None:
    rc = _must(_register, "eh_host_peer_data_register_event_handlers")
    if rc != 0:
        raise EhHostError("eh_host_peer_data_register_event_handlers", rc)


def unregister_event_handlers() -> None:
    rc = _must(_unregister, "eh_host_peer_data_unregister_event_handlers")
    if rc != 0:
        raise EhHostError("eh_host_peer_data_unregister_event_handlers", rc)
