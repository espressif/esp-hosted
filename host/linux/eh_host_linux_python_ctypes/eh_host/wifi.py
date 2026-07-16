"""WiFi feature — STA lifecycle + scan + event ids.

Wraps the C surface in
`host/features/eh_host_feat_rpc/include/eh_host_wifi.h`.  Covers
STA + scan + softAP introspection + dual-band + protocol /
bandwidth / channel control + country settings + iTWT config.

Two opaque buffer wrappers carry IDF-layout structs whose layout is
sensitive to IDF version (`wifi_config_t`, `wifi_ap_record_t`):
`make_sta_config(...)` builds the STA half of `wifi_config_t` using
pinned offsets validated against `host/compat/include/esp_wifi_types.h`.
When the linked libeh_host.so is rebuilt against a different IDF
that shifts these offsets, bump `_WIFI_CFG_T_SIZE` /
`_WIFI_AP_RECORD_T_SIZE` and the offset constants accordingly —
there's no DWARF introspection here.

Header-only deferral (typedef conflict with IDF's
esp_wifi_types_generic.h):
    set_scan_parameters / get_scan_parameters
"""

from ctypes import (
    POINTER,
    Structure,
    addressof,
    byref,
    c_bool,
    c_char,
    c_char_p,
    c_int,
    c_int8,
    c_int32,
    c_uint8,
    c_uint16,
    c_uint32,
    c_void_p,
    cast,
    memset,
)

from eh_host._lib import _LIB
from eh_host.core import EhHostError


# ---- enums (mirrors esp_wifi_types.h / esp_wifi_types_generic.h) ---

# wifi_mode_t
WIFI_MODE_NULL    = 0
WIFI_MODE_STA     = 1
WIFI_MODE_AP      = 2
WIFI_MODE_APSTA   = 3

# wifi_interface_t
WIFI_IF_STA       = 0
WIFI_IF_AP        = 1

# wifi_auth_mode_t
WIFI_AUTH_OPEN              = 0
WIFI_AUTH_WEP               = 1
WIFI_AUTH_WPA_PSK           = 2
WIFI_AUTH_WPA2_PSK          = 3
WIFI_AUTH_WPA_WPA2_PSK      = 4
WIFI_AUTH_WPA2_ENTERPRISE   = 5
WIFI_AUTH_WPA3_PSK          = 6
WIFI_AUTH_WPA2_WPA3_PSK     = 7

# wifi_ps_type_t
WIFI_PS_NONE       = 0
WIFI_PS_MIN_MODEM  = 1
WIFI_PS_MAX_MODEM  = 2

# wifi protocol bitfield (set_protocol / get_protocol)
WIFI_PROTOCOL_11B   = 0x01
WIFI_PROTOCOL_11G   = 0x02
WIFI_PROTOCOL_11N   = 0x04
WIFI_PROTOCOL_LR    = 0x08
WIFI_PROTOCOL_11A   = 0x10
WIFI_PROTOCOL_11AX  = 0x20

# wifi_bandwidth_t (set_bandwidth / get_bandwidth)
WIFI_BW_HT20 = 1
WIFI_BW_HT40 = 2

# wifi_storage_t (set_storage)
WIFI_STORAGE_FLASH = 0
WIFI_STORAGE_RAM   = 1

# wifi_band_mode_t (set_band_mode / get_band_mode)
WIFI_BAND_MODE_2G_ONLY = 1
WIFI_BAND_MODE_5G_ONLY = 2
WIFI_BAND_MODE_AUTO    = 3   # 2G + 5G

# wifi_band_t (set_band / get_band)
WIFI_BAND_2G = 1
WIFI_BAND_5G = 2

# wifi_second_chan_t (set_channel / get_channel)
WIFI_SECOND_CHAN_NONE  = 0
WIFI_SECOND_CHAN_ABOVE = 1
WIFI_SECOND_CHAN_BELOW = 2

# WIFI_EVENT ids — register handlers via eh_host.event.handler_register
# against eh_host.event.WIFI_EVENT.  Values mirror the IDF wifi_event_id_t.
WIFI_EVENT_WIFI_READY            = 0
WIFI_EVENT_SCAN_DONE             = 1
WIFI_EVENT_STA_START             = 2
WIFI_EVENT_STA_STOP              = 3
WIFI_EVENT_STA_CONNECTED         = 4
WIFI_EVENT_STA_DISCONNECTED      = 5
WIFI_EVENT_STA_AUTHMODE_CHANGE   = 6
WIFI_EVENT_AP_START              = 12
WIFI_EVENT_AP_STOP               = 13
WIFI_EVENT_AP_STACONNECTED       = 14
WIFI_EVENT_AP_STADISCONNECTED    = 15
WIFI_EVENT_HOME_CHANNEL_CHANGE   = 40


# ---- wifi_config_t (IDF-layout) -----------------------------------

# Layout pin: validated against host/compat/include/esp_wifi_types.h
# (180 bytes total; ssid @ 0; password @ 32; threshold.authmode @ 120).
# If the linked libeh_host.so was built against a different IDF that
# shifts these offsets, bump these constants accordingly.
_WIFI_CFG_T_SIZE       = 180
_OFF_SSID              = 0
_OFF_PASSWORD          = 32
_OFF_THRESHOLD_AUTHMODE = 120

# wifi_ap_record_t — size in host/compat/include/esp_wifi_types.h is
# ~80 bytes; bump to 100 for headroom across IDF versions.  Returned
# raw bytes; callers can decode if they need fields.
_WIFI_AP_RECORD_T_SIZE = 100


class WifiConfig(Structure):
    """Opaque 180-byte buffer matching `wifi_config_t`.  Build it via
    `make_sta_config(...)` rather than poking fields directly."""

    _fields_ = [("_raw", c_uint8 * _WIFI_CFG_T_SIZE)]


def make_sta_config(ssid: bytes,
                    password: bytes,
                    authmode: int = WIFI_AUTH_WPA2_PSK) -> WifiConfig:
    """Build a `wifi_config_t` populated for STA mode.

    `ssid` and `password` are bytes (≤ 31 / ≤ 63 chars; one slot
    each is reserved for NUL).  `authmode` defaults to
    `WIFI_AUTH_WPA2_PSK` (matches upstream IDF station example).
    """
    if isinstance(ssid, str):
        ssid = ssid.encode()
    if isinstance(password, str):
        password = password.encode()
    if len(ssid) > 31:
        raise ValueError(f"ssid too long: {len(ssid)} > 31")
    if len(password) > 63:
        raise ValueError(f"password too long: {len(password)} > 63")

    cfg = WifiConfig()
    memset(cast(cfg._raw, c_void_p), 0, _WIFI_CFG_T_SIZE)
    for i, b in enumerate(ssid):
        cfg._raw[_OFF_SSID + i] = b
    for i, b in enumerate(password):
        cfg._raw[_OFF_PASSWORD + i] = b
    cfg._raw[_OFF_THRESHOLD_AUTHMODE] = authmode
    return cfg


# ---- ctypes signatures --------------------------------------------

_init = _LIB.eh_host_wifi_init
_init.argtypes = [c_void_p]
_init.restype  = c_int

_deinit = _LIB.eh_host_wifi_deinit
_deinit.argtypes = []
_deinit.restype  = c_int

_start = _LIB.eh_host_wifi_start
_start.argtypes = []
_start.restype  = c_int

_stop = _LIB.eh_host_wifi_stop
_stop.argtypes = []
_stop.restype  = c_int

_connect = _LIB.eh_host_wifi_connect
_connect.argtypes = []
_connect.restype  = c_int

_disconnect = _LIB.eh_host_wifi_disconnect
_disconnect.argtypes = []
_disconnect.restype  = c_int

_set_mode = _LIB.eh_host_wifi_set_mode
_set_mode.argtypes = [c_int]
_set_mode.restype  = c_int

_set_config = _LIB.eh_host_wifi_set_config
_set_config.argtypes = [c_int, POINTER(WifiConfig)]
_set_config.restype  = c_int

_scan_start = _LIB.eh_host_wifi_scan_start
_scan_start.argtypes = [c_void_p, c_bool]  # cfg can be NULL
_scan_start.restype  = c_int

_scan_get_ap_num = _LIB.eh_host_wifi_scan_get_ap_num
_scan_get_ap_num.argtypes = [POINTER(c_uint16)]
_scan_get_ap_num.restype  = c_int

_clear_ap_list = _LIB.eh_host_wifi_clear_ap_list
_clear_ap_list.argtypes = []
_clear_ap_list.restype  = c_int

_restore = _LIB.eh_host_wifi_restore
_restore.argtypes = []
_restore.restype  = c_int

_get_mode = _LIB.eh_host_wifi_get_mode
_get_mode.argtypes = [POINTER(c_int)]
_get_mode.restype  = c_int

_get_config = _LIB.eh_host_wifi_get_config
_get_config.argtypes = [c_int, c_void_p]
_get_config.restype  = c_int

_sta_get_ap_info = _LIB.eh_host_wifi_sta_get_ap_info
_sta_get_ap_info.argtypes = [c_void_p]
_sta_get_ap_info.restype  = c_int

_scan_stop = _LIB.eh_host_wifi_scan_stop
_scan_stop.argtypes = []
_scan_stop.restype  = c_int

_set_ps = _LIB.eh_host_wifi_set_ps
_set_ps.argtypes = [c_int]
_set_ps.restype  = c_int

_get_ps = _LIB.eh_host_wifi_get_ps
_get_ps.argtypes = [POINTER(c_int)]
_get_ps.restype  = c_int

_set_max_tx_power = _LIB.eh_host_wifi_set_max_tx_power
_set_max_tx_power.argtypes = [c_int8]
_set_max_tx_power.restype  = c_int

_get_max_tx_power = _LIB.eh_host_wifi_get_max_tx_power
_get_max_tx_power.argtypes = [POINTER(c_int8)]
_get_max_tx_power.restype  = c_int

_set_country_code = _LIB.eh_host_wifi_set_country_code
_set_country_code.argtypes = [c_char_p, c_bool]
_set_country_code.restype  = c_int

_get_country_code = _LIB.eh_host_wifi_get_country_code
_get_country_code.argtypes = [c_char_p]
_get_country_code.restype  = c_int

# ---- P5.2 wrappers (16) — pack/decode wiring pending in V1_MCU ext;
# C wrapper bubbles the absent proto pack/decode as ESP_FAIL today.

_clear_fast_connect = _LIB.eh_host_wifi_clear_fast_connect
_clear_fast_connect.argtypes = []
_clear_fast_connect.restype  = c_int

_deauth_sta = _LIB.eh_host_wifi_deauth_sta
_deauth_sta.argtypes = [c_uint16]
_deauth_sta.restype  = c_int

_ap_get_sta_aid = _LIB.eh_host_wifi_ap_get_sta_aid
_ap_get_sta_aid.argtypes = [POINTER(c_uint8), POINTER(c_uint16)]
_ap_get_sta_aid.restype  = c_int

_sta_get_aid = _LIB.eh_host_wifi_sta_get_aid
_sta_get_aid.argtypes = [POINTER(c_uint16)]
_sta_get_aid.restype  = c_int

_sta_get_rssi = _LIB.eh_host_wifi_sta_get_rssi
_sta_get_rssi.argtypes = [POINTER(c_int)]
_sta_get_rssi.restype  = c_int

_sta_get_negotiated_phymode = _LIB.eh_host_wifi_sta_get_negotiated_phymode
_sta_get_negotiated_phymode.argtypes = [POINTER(c_int)]
_sta_get_negotiated_phymode.restype  = c_int

_set_protocol = _LIB.eh_host_wifi_set_protocol
_set_protocol.argtypes = [c_int, c_uint8]
_set_protocol.restype  = c_int

_get_protocol = _LIB.eh_host_wifi_get_protocol
_get_protocol.argtypes = [c_int, POINTER(c_uint8)]
_get_protocol.restype  = c_int

_set_bandwidth = _LIB.eh_host_wifi_set_bandwidth
_set_bandwidth.argtypes = [c_int, c_int]
_set_bandwidth.restype  = c_int

_get_bandwidth = _LIB.eh_host_wifi_get_bandwidth
_get_bandwidth.argtypes = [c_int, POINTER(c_int)]
_get_bandwidth.restype  = c_int

_set_channel = _LIB.eh_host_wifi_set_channel
_set_channel.argtypes = [c_uint8, c_int]
_set_channel.restype  = c_int

_get_channel = _LIB.eh_host_wifi_get_channel
_get_channel.argtypes = [POINTER(c_uint8), POINTER(c_int)]
_get_channel.restype  = c_int

_set_inactive_time = _LIB.eh_host_wifi_set_inactive_time
_set_inactive_time.argtypes = [c_int, c_uint16]
_set_inactive_time.restype  = c_int

_get_inactive_time = _LIB.eh_host_wifi_get_inactive_time
_get_inactive_time.argtypes = [c_int, POINTER(c_uint16)]
_get_inactive_time.restype  = c_int

_set_storage = _LIB.eh_host_wifi_set_storage
_set_storage.argtypes = [c_int]
_set_storage.restype  = c_int

_set_band = _LIB.eh_host_wifi_set_band
_set_band.argtypes = [c_int]
_set_band.restype  = c_int

_get_band = _LIB.eh_host_wifi_get_band
_get_band.argtypes = [POINTER(c_int)]
_get_band.restype  = c_int

_set_band_mode = _LIB.eh_host_wifi_set_band_mode
_set_band_mode.argtypes = [c_int]
_set_band_mode.restype  = c_int

_get_band_mode = _LIB.eh_host_wifi_get_band_mode
_get_band_mode.argtypes = [POINTER(c_int)]
_get_band_mode.restype  = c_int

_set_okc_support = _LIB.eh_host_wifi_set_okc_support
_set_okc_support.argtypes = [c_bool]
_set_okc_support.restype  = c_int

_scan_get_ap_record = _LIB.eh_host_wifi_scan_get_ap_record
_scan_get_ap_record.argtypes = [c_void_p]
_scan_get_ap_record.restype  = c_int


# ---- public Pythonic wrappers --------------------------------------

def init() -> None:
    """Bring the wifi feature up.  CP firmware uses its own defaults;
    `cfg` is accepted as NULL on this stack (see api_wifi.c)."""
    rc = _init(None)
    if rc != 0:
        raise EhHostError("eh_host_wifi_init", rc)


def deinit() -> None:
    rc = _deinit()
    if rc != 0:
        raise EhHostError("eh_host_wifi_deinit", rc)


def start() -> None:
    rc = _start()
    if rc != 0:
        raise EhHostError("eh_host_wifi_start", rc)


def stop() -> None:
    rc = _stop()
    if rc != 0:
        raise EhHostError("eh_host_wifi_stop", rc)


def connect() -> None:
    rc = _connect()
    if rc != 0:
        raise EhHostError("eh_host_wifi_connect", rc)


def disconnect() -> None:
    rc = _disconnect()
    if rc != 0:
        raise EhHostError("eh_host_wifi_disconnect", rc)


def set_mode(mode: int) -> None:
    rc = _set_mode(mode)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_mode", rc)


def set_config(iface: int, cfg: WifiConfig) -> None:
    """Apply a `WifiConfig` to `iface` (`WIFI_IF_STA` / `WIFI_IF_AP`)."""
    rc = _set_config(iface, cfg)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_config", rc)


def scan_start(block: bool = True) -> None:
    """Start a scan (cfg=NULL → default).  `block=True` blocks until done."""
    rc = _scan_start(None, block)
    if rc != 0:
        raise EhHostError("eh_host_wifi_scan_start", rc)


def scan_get_ap_num() -> int:
    out = c_uint16(0)
    rc = _scan_get_ap_num(out)
    if rc != 0:
        raise EhHostError("eh_host_wifi_scan_get_ap_num", rc)
    return out.value


def clear_ap_list() -> None:
    rc = _clear_ap_list()
    if rc != 0:
        raise EhHostError("eh_host_wifi_clear_ap_list", rc)


def restore() -> None:
    """Restore CP-side wifi state to factory defaults."""
    rc = _restore()
    if rc != 0:
        raise EhHostError("eh_host_wifi_restore", rc)


def get_mode() -> int:
    """Read the current `wifi_mode_t` (e.g. WIFI_MODE_STA)."""
    out = c_int(0)
    rc = _get_mode(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_mode", rc)
    return out.value


def get_config(iface: int) -> bytes:
    """Read the `wifi_config_t` for `iface` as raw bytes.

    Returned as bytes (not a typed struct) because the IDF
    `wifi_config_t` layout is sensitive to IDF version; callers that
    need fields can decode against `host/compat/include/esp_wifi_types.h`
    using the offsets pinned in this module.
    """
    buf = (c_uint8 * _WIFI_CFG_T_SIZE)()
    rc = _get_config(iface, buf)
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_config", rc)
    return bytes(buf)


def sta_get_ap_info() -> bytes:
    """Read the `wifi_ap_record_t` for the connected AP as raw bytes.

    Like `get_config`, the struct layout is IDF-version-sensitive; the
    raw bytes are returned for callers that want to decode against the
    pinned `wifi_ap_record_t` definition.
    """
    buf = (c_uint8 * _WIFI_AP_RECORD_T_SIZE)()
    rc = _sta_get_ap_info(buf)
    if rc != 0:
        raise EhHostError("eh_host_wifi_sta_get_ap_info", rc)
    return bytes(buf)


def scan_stop() -> None:
    """Cancel an in-progress scan."""
    rc = _scan_stop()
    if rc != 0:
        raise EhHostError("eh_host_wifi_scan_stop", rc)


def set_ps(ps_type: int) -> None:
    """Set the wifi power-save mode (`WIFI_PS_NONE` / `_MIN_MODEM` / `_MAX_MODEM`)."""
    rc = _set_ps(ps_type)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_ps", rc)


def get_ps() -> int:
    """Read the current `wifi_ps_type_t`."""
    out = c_int(0)
    rc = _get_ps(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_ps", rc)
    return out.value


def set_max_tx_power(power: int) -> None:
    """Set the maximum TX power (signed int8; units = 0.25 dBm)."""
    rc = _set_max_tx_power(power)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_max_tx_power", rc)


def get_max_tx_power() -> int:
    """Read the current maximum TX power (signed int8)."""
    out = c_int8(0)
    rc = _get_max_tx_power(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_max_tx_power", rc)
    return out.value


def set_country_code(country: str, ieee80211d_enabled: bool = False) -> None:
    """Set the 2-letter ISO country code (e.g. \"US\", \"CN\").

    Pass `ieee80211d_enabled=True` to honour 802.11d-derived country
    info from associated APs.
    """
    rc = _set_country_code(country.encode("ascii"), ieee80211d_enabled)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_country_code", rc)


def get_country_code() -> str:
    """Read the 2-letter ISO country code currently in effect."""
    buf = (c_char * 3)()
    rc = _get_country_code(buf)
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_country_code", rc)
    return buf.value.decode("ascii", errors="replace")


# ---- P5.2 wrappers (16) — see module docstring for deferred list ----

def clear_fast_connect() -> None:
    """Clear cached fast-connect / PMKSA state on the CP."""
    rc = _clear_fast_connect()
    if rc != 0:
        raise EhHostError("eh_host_wifi_clear_fast_connect", rc)


def deauth_sta(aid: int) -> None:
    """De-authenticate a connected STA on the SoftAP by association id.

    `aid=0` deauths every connected STA.
    """
    rc = _deauth_sta(aid)
    if rc != 0:
        raise EhHostError("eh_host_wifi_deauth_sta", rc)


def ap_get_sta_aid(mac: bytes) -> int:
    """Return the association id (AID) of a STA connected to the SoftAP.

    `mac` must be a 6-byte MAC address.
    """
    if len(mac) != 6:
        raise ValueError(f"mac must be 6 bytes, got {len(mac)}")
    mac_buf = (c_uint8 * 6)(*mac)
    out = c_uint16(0)
    rc = _ap_get_sta_aid(mac_buf, byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_ap_get_sta_aid", rc)
    return out.value


def sta_get_aid() -> int:
    """Return this STA's association id with its associated AP."""
    out = c_uint16(0)
    rc = _sta_get_aid(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_sta_get_aid", rc)
    return out.value


def sta_get_rssi() -> int:
    """Return the RSSI (dBm) of the AP this STA is associated with."""
    out = c_int(0)
    rc = _sta_get_rssi(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_sta_get_rssi", rc)
    return out.value


def sta_get_negotiated_phymode() -> int:
    """Return the `wifi_phy_mode_t` negotiated with the associated AP."""
    out = c_int(0)
    rc = _sta_get_negotiated_phymode(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_sta_get_negotiated_phymode", rc)
    return out.value


def set_protocol(iface: int, protocol: int) -> None:
    """Set the wifi-protocol bitmap (`WIFI_PROTOCOL_*`) on `iface`."""
    rc = _set_protocol(iface, protocol)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_protocol", rc)


def get_protocol(iface: int) -> int:
    """Read the wifi-protocol bitmap (`WIFI_PROTOCOL_*`) from `iface`."""
    out = c_uint8(0)
    rc = _get_protocol(iface, byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_protocol", rc)
    return out.value


def set_bandwidth(iface: int, bw: int) -> None:
    """Set bandwidth (`WIFI_BW_HT20` / `WIFI_BW_HT40`) on `iface`."""
    rc = _set_bandwidth(iface, bw)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_bandwidth", rc)


def get_bandwidth(iface: int) -> int:
    """Read the bandwidth (`WIFI_BW_*`) from `iface`."""
    out = c_int(0)
    rc = _get_bandwidth(iface, byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_bandwidth", rc)
    return out.value


def set_channel(primary: int, second: int = WIFI_SECOND_CHAN_NONE) -> None:
    """Set the primary channel (1-14) and secondary-channel selector
    (`WIFI_SECOND_CHAN_*`)."""
    rc = _set_channel(primary, second)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_channel", rc)


def get_channel() -> tuple[int, int]:
    """Return `(primary, second)` — the primary channel (1-14) and the
    secondary-channel selector (`WIFI_SECOND_CHAN_*`)."""
    primary = c_uint8(0)
    second  = c_int(0)
    rc = _get_channel(byref(primary), byref(second))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_channel", rc)
    return primary.value, second.value


def set_inactive_time(iface: int, sec: int) -> None:
    """Set the inactive-timeout (seconds) on `iface`.  0 disables."""
    rc = _set_inactive_time(iface, sec)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_inactive_time", rc)


def get_inactive_time(iface: int) -> int:
    """Read the inactive-timeout (seconds) from `iface`."""
    out = c_uint16(0)
    rc = _get_inactive_time(iface, byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_inactive_time", rc)
    return out.value


def set_storage(storage: int) -> None:
    """Set wifi config persistence (`WIFI_STORAGE_RAM` / `_FLASH`)."""
    rc = _set_storage(storage)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_storage", rc)


def set_band(band: int) -> None:
    """Set the active band (`WIFI_BAND_2G` / `WIFI_BAND_5G`)."""
    rc = _set_band(band)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_band", rc)


def get_band() -> int:
    """Read the active band (`WIFI_BAND_*`)."""
    out = c_int(0)
    rc = _get_band(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_band", rc)
    return out.value


def set_band_mode(mode: int) -> None:
    """Set band mode (`WIFI_BAND_MODE_2G_ONLY` / `_5G_ONLY` / `_AUTO`)."""
    rc = _set_band_mode(mode)
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_band_mode", rc)


def get_band_mode() -> int:
    """Read the band mode (`WIFI_BAND_MODE_*`)."""
    out = c_int(0)
    rc = _get_band_mode(byref(out))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_band_mode", rc)
    return out.value


def set_okc_support(enable: bool) -> None:
    """Enable / disable opportunistic key caching (OKC) on the STA."""
    rc = _set_okc_support(bool(enable))
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_okc_support", rc)


def scan_get_ap_record() -> bytes:
    """Read the next `wifi_ap_record_t` from the scan results as raw
    bytes.  Same shape as `sta_get_ap_info`; callers decode against the
    pinned `wifi_ap_record_t` definition.
    """
    buf = (c_uint8 * _WIFI_AP_RECORD_T_SIZE)()
    rc = _scan_get_ap_record(buf)
    if rc != 0:
        raise EhHostError("eh_host_wifi_scan_get_ap_record", rc)
    return bytes(buf)


# ---- P5.7 wrappers — wired through V1_MCU ext (struct + array RPCs) -

# IDF wifi_country_t layout (host/compat/include/esp_wifi_types.h:56-62):
#   char    cc[3];          @ 0
#   uint8_t schan;          @ 3
#   uint8_t nchan;          @ 4
#   int8_t  max_tx_power;   @ 5
#   wifi_country_policy_t (int enum) policy;  @ 8 (with 2-byte padding)
# sizeof = 12.

class WifiCountry(Structure):
    """Mirror of IDF `wifi_country_t`."""

    _fields_ = [
        ("cc",           c_char * 3),
        ("schan",        c_uint8),
        ("nchan",        c_uint8),
        ("max_tx_power", c_int8),
        ("policy",       c_int),
    ]


# wifi_protocols_t / wifi_bandwidths_t — both two scalars, no padding.

class WifiProtocols(Structure):
    """Mirror of IDF `wifi_protocols_t` (2.4 GHz + 5 GHz protocol bitmap)."""

    _fields_ = [
        ("ghz_2g", c_uint16),
        ("ghz_5g", c_uint16),
    ]


class WifiBandwidths(Structure):
    """Mirror of IDF `wifi_bandwidths_t` (2.4 GHz + 5 GHz bandwidth)."""

    _fields_ = [
        ("ghz_2g", c_int),
        ("ghz_5g", c_int),
    ]


class WifiTwtConfig(Structure):
    """Mirror of IDF `wifi_twt_config_t`."""

    _fields_ = [
        ("post_wakeup_event",     c_bool),
        ("twt_enable_keep_alive", c_bool),
    ]


# wifi_sta_list_t carries a target-dependent array (ESP_WIFI_MAX_CONN_NUM
# = 4 / 10 / 15).  Pinned conservative buffer size that fits all targets:
# 15 × sizeof(wifi_sta_info_t = 12 bytes typical) + sizeof(int num) = ~184.
# Bump headroom to 256.
_WIFI_STA_LIST_T_SIZE = 256


_set_country = _LIB.eh_host_wifi_set_country
_set_country.argtypes = [POINTER(WifiCountry)]
_set_country.restype  = c_int

_get_country = _LIB.eh_host_wifi_get_country
_get_country.argtypes = [POINTER(WifiCountry)]
_get_country.restype  = c_int

_ap_get_sta_list = _LIB.eh_host_wifi_ap_get_sta_list
_ap_get_sta_list.argtypes = [c_void_p]
_ap_get_sta_list.restype  = c_int

_scan_get_ap_records = _LIB.eh_host_wifi_scan_get_ap_records
_scan_get_ap_records.argtypes = [POINTER(c_uint16), c_void_p]
_scan_get_ap_records.restype  = c_int

_set_protocols = _LIB.eh_host_wifi_set_protocols
_set_protocols.argtypes = [c_int, POINTER(WifiProtocols)]
_set_protocols.restype  = c_int

_get_protocols = _LIB.eh_host_wifi_get_protocols
_get_protocols.argtypes = [c_int, POINTER(WifiProtocols)]
_get_protocols.restype  = c_int

_set_bandwidths = _LIB.eh_host_wifi_set_bandwidths
_set_bandwidths.argtypes = [c_int, POINTER(WifiBandwidths)]
_set_bandwidths.restype  = c_int

_get_bandwidths = _LIB.eh_host_wifi_get_bandwidths
_get_bandwidths.argtypes = [c_int, POINTER(WifiBandwidths)]
_get_bandwidths.restype  = c_int

_sta_twt_config = _LIB.eh_host_wifi_sta_twt_config
_sta_twt_config.argtypes = [POINTER(WifiTwtConfig)]
_sta_twt_config.restype  = c_int


def set_country(cc: str,
                schan: int = 1,
                nchan: int = 13,
                max_tx_power: int = 20,
                policy: int = 0) -> None:
    """Set the full `wifi_country_t` (cc + channel range + max TX power
    + policy).  `policy` is `wifi_country_policy_t` (0=auto, 1=manual)."""
    cc_b = cc.encode("ascii")
    if len(cc_b) > 3:
        raise ValueError(f"cc must be ≤ 3 chars, got {cc!r}")
    country = WifiCountry(cc=cc_b, schan=schan, nchan=nchan,
                          max_tx_power=max_tx_power, policy=policy)
    rc = _set_country(byref(country))
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_country", rc)


def get_country() -> dict:
    """Read the full `wifi_country_t` as a dict."""
    country = WifiCountry()
    rc = _get_country(byref(country))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_country", rc)
    return {
        "cc":           country.cc.decode("ascii", errors="replace"),
        "schan":        country.schan,
        "nchan":        country.nchan,
        "max_tx_power": country.max_tx_power,
        "policy":       country.policy,
    }


def ap_get_sta_list() -> bytes:
    """Read the SoftAP-side `wifi_sta_list_t` as raw bytes.  Layout is
    target-sensitive (`ESP_WIFI_MAX_CONN_NUM` varies by chip); callers
    decode against the pinned `wifi_sta_list_t` definition.
    """
    buf = (c_uint8 * _WIFI_STA_LIST_T_SIZE)()
    rc = _ap_get_sta_list(buf)
    if rc != 0:
        raise EhHostError("eh_host_wifi_ap_get_sta_list", rc)
    return bytes(buf)


def scan_get_ap_records(max_n: int) -> tuple[int, bytes]:
    """Read up to `max_n` `wifi_ap_record_t` entries.  Returns
    `(actual_n, raw_bytes)` where `raw_bytes` is `max_n × _WIFI_AP_RECORD_T_SIZE`
    bytes; only the first `actual_n × _WIFI_AP_RECORD_T_SIZE` are valid.
    Caller decodes against the pinned `wifi_ap_record_t` definition.
    """
    if max_n <= 0:
        raise ValueError(f"max_n must be > 0, got {max_n}")
    n = c_uint16(max_n)
    buf = (c_uint8 * (_WIFI_AP_RECORD_T_SIZE * max_n))()
    rc = _scan_get_ap_records(byref(n), buf)
    if rc != 0:
        raise EhHostError("eh_host_wifi_scan_get_ap_records", rc)
    return n.value, bytes(buf)


def set_protocols(iface: int, ghz_2g: int, ghz_5g: int = 0) -> None:
    """Set the per-band protocol bitmap on `iface`.  `ghz_2g` and
    `ghz_5g` are `WIFI_PROTOCOL_*` bitmaps."""
    p = WifiProtocols(ghz_2g=ghz_2g, ghz_5g=ghz_5g)
    rc = _set_protocols(iface, byref(p))
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_protocols", rc)


def get_protocols(iface: int) -> tuple[int, int]:
    """Return `(ghz_2g, ghz_5g)` — per-band protocol bitmaps on `iface`."""
    p = WifiProtocols()
    rc = _get_protocols(iface, byref(p))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_protocols", rc)
    return p.ghz_2g, p.ghz_5g


def set_bandwidths(iface: int, ghz_2g: int, ghz_5g: int = 0) -> None:
    """Set per-band bandwidth on `iface`.  Each is `WIFI_BW_HT20` /
    `WIFI_BW_HT40`."""
    b = WifiBandwidths(ghz_2g=ghz_2g, ghz_5g=ghz_5g)
    rc = _set_bandwidths(iface, byref(b))
    if rc != 0:
        raise EhHostError("eh_host_wifi_set_bandwidths", rc)


def get_bandwidths(iface: int) -> tuple[int, int]:
    """Return `(ghz_2g, ghz_5g)` per-band bandwidths on `iface`."""
    b = WifiBandwidths()
    rc = _get_bandwidths(iface, byref(b))
    if rc != 0:
        raise EhHostError("eh_host_wifi_get_bandwidths", rc)
    return b.ghz_2g, b.ghz_5g


def sta_twt_config(post_wakeup_event: bool = False,
                   twt_enable_keep_alive: bool = False) -> None:
    """Configure STA-side iTWT post-wakeup-event + keep-alive flags."""
    c = WifiTwtConfig(post_wakeup_event=post_wakeup_event,
                      twt_enable_keep_alive=twt_enable_keep_alive)
    rc = _sta_twt_config(byref(c))
    if rc != 0:
        raise EhHostError("eh_host_wifi_sta_twt_config", rc)
