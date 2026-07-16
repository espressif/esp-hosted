#!/usr/bin/env python3
"""network_split/station · linux_802_3 · host_py_app — eh_host ctypes demo.

Wi-Fi station bring-up with network-split. Override SSID / password
via EH_WIFI_SSID / EH_WIFI_PASSWORD env vars."""

import os
import sys
import time

import eh_host
from eh_host import nw_split, system, wifi


WIFI_MODE_STA = 1


def main() -> int:
    ssid = os.environ.get("EH_WIFI_SSID", "myssid").encode()
    password = os.environ.get("EH_WIFI_PASSWORD", "mypassword").encode()

    eh_host.init_linux_default()
    try:
        fw = system.get_cp_fw_version()
        print(f"CP firmware: {fw.major1}.{fw.minor1}.{fw.patch1}")

        wifi.init()
        wifi.set_mode(WIFI_MODE_STA)
        cfg = wifi.make_sta_config(ssid, password)
        wifi.set_config(0, cfg)  # WIFI_IF_STA = 0
        wifi.start()
        try:
            wifi.connect()
        except eh_host.EhHostError as connect_err:
            print(f"connect: {connect_err}", file=sys.stderr)
        time.sleep(3)

        try:
            status = nw_split.get_status()
            print("Network-split status:")
            for key, value in status.items():
                print(f"  {key}: {value}")
        except eh_host.EhHostError as status_err:
            print(
                f"network-split status query: {status_err}", file=sys.stderr
            )

        wifi.disconnect()
        wifi.stop()
        wifi.deinit()
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        eh_host.deinit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
