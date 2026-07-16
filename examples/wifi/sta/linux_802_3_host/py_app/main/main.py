#!/usr/bin/env python3
"""wifi/sta · linux_802_3 · host_py_app — eh_host ctypes demo.

Override SSID / password / retry count via env:
  EH_WIFI_SSID, EH_WIFI_PASSWORD, EH_WIFI_MAX_RETRY."""

import os
import sys
import threading
from ctypes import c_char_p

import eh_host
from eh_host import event, wifi

WIFI_MODE_STA              = 1
IP_EVENT_STA_GOT_IP        = 0


def main() -> int:
    ssid     = os.environ.get("EH_WIFI_SSID", "myssid").encode()
    password = os.environ.get("EH_WIFI_PASSWORD", "mypassword").encode()
    max_retry = int(os.environ.get("EH_WIFI_MAX_RETRY", "5"))

    state = {"retry": 0, "result": None}
    done = threading.Event()

    wifi_base = event.WIFI_EVENT.value
    ip_base   = event.IP_EVENT.value

    def on_event(ctx, base, event_id, data_ptr):
        if base == wifi_base and event_id == wifi.WIFI_EVENT_STA_START:
            wifi.connect()
        elif base == wifi_base and event_id == wifi.WIFI_EVENT_STA_DISCONNECTED:
            if state["retry"] < max_retry:
                state["retry"] += 1
                print(f"retry {state['retry']}/{max_retry} to connect to the AP")
                wifi.connect()
            else:
                state["result"] = "fail"
                done.set()
        elif base == ip_base and event_id == IP_EVENT_STA_GOT_IP:
            state["result"] = "ok"
            done.set()

    eh_host.init_linux_default()
    event.loop_create_default()
    event.handler_register(event.WIFI_EVENT, event.ESP_EVENT_ANY_ID, on_event)
    event.handler_register(event.IP_EVENT,   IP_EVENT_STA_GOT_IP,    on_event)

    try:
        wifi.init()
        wifi.set_mode(WIFI_MODE_STA)
        cfg = wifi.make_sta_config(ssid, password)
        wifi.set_config(0, cfg)
        wifi.start()
        print(f"Wi-Fi started; waiting for {ssid.decode()!r}")

        done.wait()
        if state["result"] == "ok":
            print(f"connected to ap SSID={ssid.decode()!r}")
        else:
            print(f"failed to connect to SSID={ssid.decode()!r}", file=sys.stderr)

        wifi.disconnect()
        wifi.stop()
        wifi.deinit()
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        event.handler_unregister(event.WIFI_EVENT, event.ESP_EVENT_ANY_ID, on_event)
        event.handler_unregister(event.IP_EVENT,   IP_EVENT_STA_GOT_IP,    on_event)
        eh_host.deinit()

    return 0 if state["result"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
