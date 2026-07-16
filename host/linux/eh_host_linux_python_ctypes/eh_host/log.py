import os
import sys
import time

_LEVELS = ("E", "W", "I", "D", "V")
_LEVEL_MAX = _LEVELS.index(os.environ.get("EH_HOST_LOG_LEVEL", "I").upper()
                           if os.environ.get("EH_HOST_LOG_LEVEL", "I").upper() in _LEVELS
                           else "I")

_COLORS = {
    "E": "\033[0;31m",
    "W": "\033[0;33m",
    "I": "\033[0;32m",
    "D": "",
    "V": "",
}
_RESET = "\033[0m"
_START_NS = time.monotonic_ns()


def _ms() -> int:
    return (time.monotonic_ns() - _START_NS) // 1_000_000


def _log(level: str, tag: str, fmt: str, *args) -> None:
    if _LEVELS.index(level) > _LEVEL_MAX:
        return
    msg = (fmt % args) if args else fmt
    stream = sys.stderr if level in ("E", "W") else sys.stdout
    print(f"{_COLORS[level]}{level} ({_ms()}) {tag}: {msg}{_RESET}",
          file=stream, flush=True)


def ESP_LOGE(tag: str, fmt: str, *args) -> None: _log("E", tag, fmt, *args)
def ESP_LOGW(tag: str, fmt: str, *args) -> None: _log("W", tag, fmt, *args)
def ESP_LOGI(tag: str, fmt: str, *args) -> None: _log("I", tag, fmt, *args)
def ESP_LOGD(tag: str, fmt: str, *args) -> None: _log("D", tag, fmt, *args)
def ESP_LOGV(tag: str, fmt: str, *args) -> None: _log("V", tag, fmt, *args)
