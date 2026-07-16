#!/usr/bin/env python3
"""system/hosted_events · linux_802_3 · host_py_app — eh_host ctypes demo.

Configures the CP heartbeat (source of EH_HOST_EVENT_CP_HEARTBEAT)
and prints the firmware version."""

import sys
import time

import eh_host
from eh_host import system


def main() -> int:
    eh_host.init_linux_default()
    try:
        # Configure CP heartbeat at 5s cadence (drives EH_HOST_EVENT_CP_HEARTBEAT).
        system.config_heartbeat(enable=True, duration_sec=5)
        print("Heartbeat configured (enable=True, duration=5s).")

        fw = system.get_cp_fw_version()
        print(
            f"CP firmware: "
            f"{fw.major1}.{fw.minor1}.{fw.patch1}"
            f" (revision={fw.revision_no} prerelease={fw.prerelease}"
            f" build={fw.build_no})"
        )

        print("Sleeping 10s — CP emits heartbeat events; observe via")
        print("`dmesg -w` or the host log to see the wire traffic.")
        time.sleep(10)

        system.config_heartbeat(enable=False, duration_sec=0)
        print("Heartbeat disabled.")
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        eh_host.deinit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
