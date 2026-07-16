"""Network-split — DHCP/DNS lease bridging between CP lwIP and host kernel.

`get_status()` reads the CP's current DHCP/DNS snapshot synchronously;
`set_status()` pushes a host-originated lease into the CP.  Inbound
DHCP/DNS lease updates ride the host event base (slot
``ESP_HOSTED_EVENT_NW_SPLIT_STATUS``) on the IDF default loop;
subscribe via `eh_host.event` to receive them.
"""

import socket as _socket
import struct as _struct

from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_int,
    c_uint8,
    c_uint32,
)

from eh_host._lib import _opt_bind, _must
from eh_host.core import EhHostError
from eh_host.event import (
    NwSplitStatusPayload as NwSplitStatus,
    ip4_addr_to_str,
    ip_addr_to_str,
)


def ip6_addr_to_str(ip6) -> str:
    """Render a raw `esp_ip6_addr_t` (no type tag) as text."""
    raw = b"".join(_struct.pack("=I", w & 0xffffffff) for w in ip6.addr)
    return _socket.inet_ntop(_socket.AF_INET6, raw)


# ── interface ids (mirror IDF wifi_interface_t) ─────────────────────
WIFI_IF_STA = 0
WIFI_IF_AP  = 1


# `eh_host_nw_split_status_t` is a C typedef of
# `esp_hosted_event_nw_split_status_t`, so reuse the same ctypes mirror.

_get_status = _opt_bind("eh_host_nw_split_get_status",
                        [POINTER(NwSplitStatus)], c_int)

_set_status = _opt_bind("eh_host_nw_split_set_status",
                        [c_uint32, c_uint8, c_uint8,
                         c_char_p, c_char_p, c_char_p,
                         c_uint8, c_char_p, c_uint8],
                        c_int)

_register   = _opt_bind("eh_host_nw_split_register_event_handlers",   [], c_int)
_unregister = _opt_bind("eh_host_nw_split_unregister_event_handlers", [], c_int)


def get_status() -> dict:
    """Read the current DHCP/DNS lease state from the CP.

    Returns a dict with keys:
      ``link_up``, ``dhcp_up``        — bool
      ``has_ipv4``                    — bool; True if ipv4_* are valid
      ``ipv4``, ``netmask``, ``gw``   — dotted-quad strings (when has_ipv4)
      ``has_ipv6``                    — bool (reserved; False on v1 wire)
      ``ipv6``, ``ipv6_gateway``      — strings (when has_ipv6; empty today)
      ``dns_main``, ``dns_backup``    — strings or None (None when type==0)

    Raises `EhHostError` on transport or CP-side error.
    """
    out = NwSplitStatus()
    rc = _must(_get_status, "eh_host_nw_split_get_status", byref(out))
    if rc != 0:
        raise EhHostError("eh_host_nw_split_get_status", rc)

    return {
        "link_up":      bool(out.link_up),
        "dhcp_up":      bool(out.dhcp_up),
        "has_ipv4":     bool(out.has_ipv4),
        "ipv4":         ip4_addr_to_str(out.ipv4.ip.addr)      if out.has_ipv4 else None,
        "netmask":      ip4_addr_to_str(out.ipv4.netmask.addr) if out.has_ipv4 else None,
        "gw":           ip4_addr_to_str(out.ipv4.gw.addr)      if out.has_ipv4 else None,
        "has_ipv6":     bool(out.has_ipv6),
        "ipv6":         ip6_addr_to_str(out.ipv6)         if out.has_ipv6 else None,
        "ipv6_gateway": ip6_addr_to_str(out.ipv6_gateway) if out.has_ipv6 else None,
        "dns_main":     ip_addr_to_str(out.dns_main)   if out.dns_main.type   != 0 else None,
        "dns_backup":   ip_addr_to_str(out.dns_backup) if out.dns_backup.type != 0 else None,
    }


def set_status(iface: int,
               link_up: int,
               dhcp_up: int,
               dhcp_ip: str | None,
               dhcp_nm: str | None,
               dhcp_gw: str | None,
               dns_up: int,
               dns_ip: str | None,
               dns_type: int) -> None:
    """Push a host-originated lease into the CP.

    `iface` is a wifi-interface index (0=STA, 1=SoftAP).  The IP /
    netmask / gateway / dns_ip params are dotted-quad strings (or
    `None` to leave the field empty on the wire).  `dns_type` selects
    main / backup / fallback per the upstream proto.

    Mirror of upstream `rpc_set_dhcp_dns_status` (rpc_wrap.h:143-145).
    Raises `EhHostError` on transport or CP-side error.
    """
    def _b(s):
        if s is None:
            return None
        if isinstance(s, str):
            return s.encode("ascii")
        return s

    rc = _must(_set_status, "eh_host_nw_split_set_status",
               iface, link_up, dhcp_up,
               _b(dhcp_ip), _b(dhcp_nm), _b(dhcp_gw),
               dns_up, _b(dns_ip), dns_type)
    if rc != 0:
        raise EhHostError("eh_host_nw_split_set_status", rc)


def register_event_handlers() -> None:
    rc = _must(_register, "eh_host_nw_split_register_event_handlers")
    if rc != 0:
        raise EhHostError("eh_host_nw_split_register_event_handlers", rc)


def unregister_event_handlers() -> None:
    rc = _must(_unregister, "eh_host_nw_split_unregister_event_handlers")
    if rc != 0:
        raise EhHostError("eh_host_nw_split_unregister_event_handlers", rc)
