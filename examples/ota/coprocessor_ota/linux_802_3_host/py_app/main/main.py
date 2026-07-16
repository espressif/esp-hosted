#!/usr/bin/env python3
"""ota / coprocessor_ota · linux_802_3 · host_py_app — eh_host ctypes demo.

Streams a local CP firmware image to the coprocessor (begin → write*
→ end → activate). Pass the firmware path as argv[1]."""

import sys
from pathlib import Path

import eh_host
from eh_host import ota, system


CHUNK_BYTES = 4096


def _stream_file(path: Path) -> int:
    total = 0
    with path.open("rb") as fp:
        while True:
            buf = fp.read(CHUNK_BYTES)
            if not buf:
                break
            ota.write(buf)
            total += len(buf)
    return total


def main() -> int:
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <cp.bin>", file=sys.stderr)
        return 2
    image = Path(sys.argv[1])
    if not image.is_file():
        print(f"error: {image} is not a regular file", file=sys.stderr)
        return 2

    eh_host.init_linux_default()
    try:
        ver = system.get_cp_fw_version()
        print(f"slave fw version: {ver.major1}.{ver.minor1}.{ver.patch1}")

        print("ota_begin")
        ota.begin()
        print(f"streaming {image} ({image.stat().st_size} B) in {CHUNK_BYTES}-B chunks")
        sent = _stream_file(image)
        print(f"streamed {sent} bytes")
        print("ota_end")
        ota.end()
        print("ota_activate (CP will reboot into the new image)")
        ota.activate()
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        eh_host.deinit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
