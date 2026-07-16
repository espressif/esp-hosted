from eh_host.core import init, init_linux_default, deinit, EhHostError
from eh_host.system import (
    config_heartbeat,
    get_cp_fw_version,
    get_mac,
    set_mac,
    FwVer,
)
from eh_host import event
from eh_host import log

__all__ = [
    "init",
    "init_linux_default",
    "deinit",
    "EhHostError",
    "config_heartbeat",
    "get_cp_fw_version",
    "get_mac",
    "set_mac",
    "FwVer",
    "event",
    "log",
]

for _modname in (
    "wifi", "bt", "ext_coex", "gpio_exp", "heartbeat",
    "mem_monitor", "nw_split", "ota", "peer_data",
    "wifi_dpp", "wifi_ent", "wifi_itwt",
):
    try:
        _mod = __import__("eh_host." + _modname, fromlist=[_modname])
        globals()[_modname] = _mod
        __all__.append(_modname)
    except (AttributeError, OSError):
        pass
