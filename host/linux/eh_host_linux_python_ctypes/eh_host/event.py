"""IDF event-loop bridge — Python <-> esp_event_handler_register.

Exposes the small slice of `esp_event_*` that ctypes consumers need
to subscribe to events emitted by the host stack.  The relevant
symbols (`esp_event_handler_register`, `esp_event_loop_*`,
`ESP_HOSTED_EVENT`, `WIFI_EVENT`, `IP_EVENT`) are explicitly
allowlisted in `eh_host.lds`.

Threading: callbacks run on the IDF default-event-loop dispatcher
thread, NOT the thread that called `register`.  Python user code
must be thread-safe (or marshal back via a queue) just like C
handlers — same contract as upstream IDF.

Reference:
  - `host/features/eh_host_feat_rpc/include/eh_host_event.h` —
    ESP_HOSTED_EVENT base + id enum + payload structs
  - `port/esp_idf_port/esp_event/src/esp_event_loop.c` —
    pthread-backed default-loop runtime (linux_user)
"""

import socket
import struct as _struct

from ctypes import (
    CFUNCTYPE,
    POINTER,
    Structure,
    Union,
    c_bool,
    c_char_p,
    c_int,
    c_int32,
    c_size_t,
    c_uint8,
    c_uint32,
    c_void_p,
    cast,
)

from eh_host._lib import _LIB
from eh_host.core import EhHostError


# ---- event-base globals (pointer values from libeh_host.so) --------

ESP_HOSTED_EVENT = c_char_p.in_dll(_LIB, "ESP_HOSTED_EVENT")
WIFI_EVENT      = c_char_p.in_dll(_LIB, "WIFI_EVENT")
IP_EVENT        = c_char_p.in_dll(_LIB, "IP_EVENT")
# ETH_EVENT is allowlisted in the version script for forward
# compatibility but no current TU defines it on linux_user, so it's
# omitted here.  Add when the eth path lands.


# ---- EH_HOST_EVENT ids — mirror eh_host_event.h verbatim ------------

EH_HOST_EVENT_CP_INIT            = 0
EH_HOST_EVENT_CP_HEARTBEAT       = 1
EH_HOST_EVENT_TRANSPORT_FAILURE  = 2
EH_HOST_EVENT_TRANSPORT_UP       = 3
EH_HOST_EVENT_TRANSPORT_DOWN     = 4
EH_HOST_EVENT_MEM_MONITOR        = 5
EH_HOST_EVENT_PEER_DATA_RX       = 6
EH_HOST_EVENT_NW_SPLIT_STATUS    = 7
EH_HOST_EVENT_GPIO_EXP_INT       = 8

# Upstream-MCU spellings (back-compat aliases for ESP_HOSTED_EVENT_*).
ESP_HOSTED_EVENT_CP_INIT             = EH_HOST_EVENT_CP_INIT
ESP_HOSTED_EVENT_CP_HEARTBEAT        = EH_HOST_EVENT_CP_HEARTBEAT
ESP_HOSTED_EVENT_TRANSPORT_FAILURE   = EH_HOST_EVENT_TRANSPORT_FAILURE
ESP_HOSTED_EVENT_TRANSPORT_UP        = EH_HOST_EVENT_TRANSPORT_UP
ESP_HOSTED_EVENT_TRANSPORT_DOWN      = EH_HOST_EVENT_TRANSPORT_DOWN
ESP_HOSTED_EVENT_MEM_MONITOR         = EH_HOST_EVENT_MEM_MONITOR
ESP_HOSTED_EVENT_PEER_DATA_RX        = EH_HOST_EVENT_PEER_DATA_RX
ESP_HOSTED_EVENT_NW_SPLIT_STATUS     = EH_HOST_EVENT_NW_SPLIT_STATUS
ESP_HOSTED_EVENT_GPIO_EXP_INT        = EH_HOST_EVENT_GPIO_EXP_INT

ESP_EVENT_ANY_ID = -1


# ---- payload structs -----------------------------------------------

class HeartbeatPayload(Structure):
    """Mirror of `esp_hosted_event_heartbeat_t`."""

    _fields_ = [("heartbeat", c_uint32)]


class CpInitPayload(Structure):
    """Mirror of `esp_hosted_event_init_t`."""

    _fields_ = [("reset_reason", c_uint32)]


class PeerDataPayload(Structure):
    """Mirror of `esp_hosted_event_peer_data_t`.

    `data` is a `c_void_p` (const view into the RPC ext's decoded
    ctrl_cmd; valid only for the duration of the handler call —
    handlers must copy bytes before returning if they want to retain).
    """

    _fields_ = [
        ("msg_id", c_uint32),
        ("data",   c_void_p),
        ("len",    c_size_t),
    ]


# ---- IPv4/IPv6 mirrors of esp_netif_ip_addr.h ---------------------
#
# IMPORTANT byte order: esp_ip4_addr_t.addr stores the address such
# that memory byte[0] is the first octet (network byte order in
# memory regardless of host endianness). On little-endian (every host
# stack target today) that means the LSB of the u32 is the first
# octet — `addr == 0x2c020a0a` represents "10.10.2.44".
#
# `ip4_addr_to_str(addr)` below converts back to dotted-quad.

class Ip4Addr(Structure):
    """Mirror of `esp_ip4_addr_t`."""
    _fields_ = [("addr", c_uint32)]


class Ip6Addr(Structure):
    """Mirror of `esp_ip6_addr_t`."""
    _fields_ = [
        ("addr", c_uint32 * 4),
        ("zone", c_uint8),
    ]


class _IpAddrUnion(Union):
    _fields_ = [
        ("ip6", Ip6Addr),
        ("ip4", Ip4Addr),
    ]


class IpAddr(Structure):
    """Mirror of `esp_ip_addr_t` — tagged union with `.type`."""
    _fields_ = [
        ("u_addr", _IpAddrUnion),
        ("type",   c_uint8),
    ]


# `esp_ip_addr_t.type` enum values (from esp_netif_ip_addr.h)
ESP_IPADDR_TYPE_V4 = 0
ESP_IPADDR_TYPE_V6 = 6


class NetifIpInfo(Structure):
    """Mirror of `esp_netif_ip_info_t` (IPv4 triple)."""
    _fields_ = [
        ("ip",       Ip4Addr),
        ("netmask",  Ip4Addr),
        ("gw",       Ip4Addr),
    ]


def ip4_addr_to_str(addr: int) -> str:
    """Convert an `esp_ip4_addr_t.addr` u32 to dotted-quad text.

    The address bytes in memory match the on-wire octet order, so on
    little-endian the LSB of `addr` is the first octet. We pack in
    native order so byte[0] (LSB on LE) is the first octet.
    """
    return socket.inet_ntoa(_struct.pack("=I", addr & 0xffffffff))


def ip_addr_to_str(ip: IpAddr) -> str:
    """Render an `esp_ip_addr_t` (v4 or v6) as printable text."""
    if ip.type == ESP_IPADDR_TYPE_V4:
        return ip4_addr_to_str(ip.u_addr.ip4.addr)
    if ip.type == ESP_IPADDR_TYPE_V6:
        # esp_ip6_addr_t.addr[] is 4 x u32 in the same "memory matches
        # network order" convention; inet_ntop wants 16 bytes in NBO.
        raw = b"".join(_struct.pack("=I", w & 0xffffffff)
                       for w in ip.u_addr.ip6.addr)
        return socket.inet_ntop(socket.AF_INET6, raw)
    return "<unset>"


class NwSplitStatusPayload(Structure):
    """Mirror of `esp_hosted_event_nw_split_status_t`.

    Family-agnostic shape: v4 lane is `ipv4` (esp_netif_ip_info_t),
    v6 lane reserved (not populated by the v1 MCU wire today). DNS
    servers use `IpAddr` tagged unions so they can hold v4 or v6.

    Population rules (v1 wire):
      - link_up / dhcp_up: always from wire.
      - has_ipv4: True when dhcp_up and ipv4.ip is non-zero.
      - has_ipv6: False (reserved).
      - dns_main: type == ESP_IPADDR_TYPE_V4 when populated; type == 0
                  means "no entry".
      - dns_backup: always type == 0 today (no wire field).

    Render addresses with `ip4_addr_to_str(payload.ipv4.ip.addr)` /
    `ip_addr_to_str(payload.dns_main)`.
    """

    _fields_ = [
        ("link_up",      c_bool),
        ("dhcp_up",      c_bool),
        ("has_ipv4",     c_bool),
        ("ipv4",         NetifIpInfo),
        ("has_ipv6",     c_bool),
        ("ipv6",         Ip6Addr),
        ("ipv6_gateway", Ip6Addr),
        ("dns_main",     IpAddr),
        ("dns_backup",   IpAddr),
    ]


class _MemInfoBasic(Structure):
    """Mirror of `esp_hosted_mem_info_t` (per-cap-class slot)."""

    _fields_ = [
        ("free_size",          c_uint32),
        ("largest_free_block", c_uint32),
    ]


class _CapInfo(Structure):
    """Mirror of `esp_hosted_cap_info_t` (DMA + 8-bit slots)."""

    _fields_ = [
        ("cap_dma",  _MemInfoBasic),
        ("cap_8bit", _MemInfoBasic),
    ]


class MemMonitorPayload(Structure):
    """Mirror of `esp_hosted_event_mem_info_t`.

    Snapshot reported by the CP-side mem-monitor when a heap watermark
    is crossed.  Internal/external each have DMA-capable + 8/16-bit-
    capable cap slots; use `payload.curr_internal.cap_dma.free_size`
    etc. to inspect.
    """

    _fields_ = [
        ("curr_total_free_heap_size", c_uint32),
        ("curr_min_free_heap_size",   c_uint32),
        ("curr_internal",             _CapInfo),
        ("curr_external",             _CapInfo),
    ]


# ---- ctypes signatures --------------------------------------------

_create_default = _LIB.esp_event_loop_create_default
_create_default.argtypes = []
_create_default.restype  = c_int

_delete_default = _LIB.esp_event_loop_delete_default
_delete_default.argtypes = []
_delete_default.restype  = c_int

# typedef void (*esp_event_handler_t)(void *event_handler_arg,
#                                     esp_event_base_t event_base,
#                                     int32_t event_id,
#                                     void *event_data);
_HANDLER_T = CFUNCTYPE(None, c_void_p, c_char_p, c_int32, c_void_p)

_register = _LIB.esp_event_handler_register
_register.argtypes = [c_char_p, c_int32, _HANDLER_T, c_void_p]
_register.restype  = c_int

_unregister = _LIB.esp_event_handler_unregister
_unregister.argtypes = [c_char_p, c_int32, _HANDLER_T]
_unregister.restype  = c_int


# Keep CFUNCTYPE wrappers alive — IDF stores the function pointer in
# the handler list and calls back asynchronously.  If Python GCs the
# wrapper, the next dispatch segfaults.  Map (base, event_id, py_cb)
# tuples to their ctypes wrappers; unregister evicts.
_LIVE_HANDLERS: dict[tuple, object] = {}


# ---- public Pythonic wrappers --------------------------------------

ESP_ERR_INVALID_STATE = 0x103  # already-created loop returns this; tolerate.


def loop_create_default() -> None:
    """Bring up the IDF default event loop.  Idempotent at the C level
    (already-up returns ESP_ERR_INVALID_STATE; we treat that as OK)."""
    rc = _create_default()
    if rc != 0 and rc != ESP_ERR_INVALID_STATE:
        raise EhHostError("esp_event_loop_create_default", rc)


def loop_delete_default() -> None:
    """Tear down the IDF default event loop."""
    rc = _delete_default()
    if rc != 0:
        raise EhHostError("esp_event_loop_delete_default", rc)


def handler_register(event_base: c_char_p,
                     event_id: int,
                     handler,
                     ctx=None) -> None:
    """Register a Python `handler(ctx, base, event_id, data_ptr)`
    against `(event_base, event_id)`.

    `handler` must accept (ctx, base, event_id, data_ptr) where
    `data_ptr` is a c_void_p the user can `cast()` to the relevant
    payload struct (e.g. `cast(data_ptr, POINTER(HeartbeatPayload))`).

    Pass `event_id=ESP_EVENT_ANY_ID` to subscribe to every id under
    `event_base`.

    The CFUNCTYPE wrapper is retained for the registration's lifetime;
    `handler_unregister` releases it.
    """
    cb = _HANDLER_T(handler)
    rc = _register(event_base, event_id, cb, ctx)
    if rc != 0:
        raise EhHostError("esp_event_handler_register", rc)
    # Key by (base value bytes + id + python handler id) — the same
    # Python callable can be registered against multiple bases/ids.
    base_val = event_base.value if isinstance(event_base, c_char_p) else event_base
    _LIVE_HANDLERS[(base_val, event_id, id(handler))] = cb


def handler_unregister(event_base: c_char_p,
                       event_id: int,
                       handler) -> None:
    """Reverse of `handler_register`."""
    base_val = event_base.value if isinstance(event_base, c_char_p) else event_base
    cb = _LIVE_HANDLERS.pop((base_val, event_id, id(handler)), None)
    if cb is None:
        # Nothing registered under this key — nothing to do.
        return
    rc = _unregister(event_base, event_id, cb)
    if rc != 0:
        raise EhHostError("esp_event_handler_unregister", rc)


def cast_payload(data_ptr, struct_t):
    """Helper: cast a `void*` event_data to the typed payload struct."""
    return cast(data_ptr, POINTER(struct_t)).contents
