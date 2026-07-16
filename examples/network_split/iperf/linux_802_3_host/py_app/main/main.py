#!/usr/bin/env python3
"""network_split/iperf · linux_802_3 · host_py_app — eh_host ctypes demo.

Reads the network-split status from the CP; use the C app for the
full iperf traffic test."""

import sys

import eh_host
from eh_host import nw_split, system


def main() -> int:
    eh_host.init_linux_default()
    try:
        fw = system.get_cp_fw_version()
        print(f"CP firmware: {fw.major1}.{fw.minor1}.{fw.patch1}")

        try:
            status = nw_split.get_status()
            print("Network-split status snapshot:")
            for key, value in status.items():
                print(f"  {key}: {value}")
        except eh_host.EhHostError as status_err:
            print(f"network-split status query: {status_err}", file=sys.stderr)
            print(
                "(CP-side network_split feature must be enabled — "
                "see cp/sdkconfig.defaults)",
                file=sys.stderr,
            )
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        eh_host.deinit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
