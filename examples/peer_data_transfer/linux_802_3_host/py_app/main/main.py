#!/usr/bin/env python3
"""peer_data_transfer / echo · linux_802_3 · host_py_app — eh_host ctypes demo.

Sends three peer-data requests and listens for echoes on
EH_HOST_EVENT_PEER_DATA_RX."""

import ctypes
import sys
import time

import eh_host
from eh_host import event, peer_data


MSG_ID_CAT, MSG_ID_MEOW = 1, 2
MSG_ID_DOG, MSG_ID_WOOF = 3, 4
MSG_ID_HUMAN, MSG_ID_HELLO = 5, 6


_received = []


def _on_peer_data(_ctx, _event_base, _event_id, event_data):
    payload = event.cast_payload(event_data, event.PeerDataPayload)
    body = bytes(
        ctypes.cast(payload.data, ctypes.POINTER(ctypes.c_uint8 * payload.len))[0]
    )
    _received.append((payload.msg_id, body))
    print(f"  <- echo msg_id={payload.msg_id} len={payload.len}")


def _send_and_wait(msg_id: int, payload: bytes, timeout_s: float = 2.0) -> None:
    print(f"-> send msg_id={msg_id} len={len(payload)}")
    peer_data.send(msg_id, payload)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _received and _received[-1][0] == msg_id + 1:
            return
        time.sleep(0.05)
    print(f"  (timed out waiting for echo of msg_id={msg_id})", file=sys.stderr)


def main() -> int:
    eh_host.init_linux_default()
    try:
        event.loop_create_default()
        peer_data.register_event_thunks()
        event.handler_register(
            event.ESP_HOSTED_EVENT,
            event.ESP_HOSTED_EVENT_PEER_DATA_RX,
            _on_peer_data,
            None,
        )

        _send_and_wait(MSG_ID_CAT, b"meow"[:1] * 16)
        _send_and_wait(MSG_ID_DOG, b"woof" * 256)
        _send_and_wait(MSG_ID_HUMAN, b"hello" * 1000)

        print(f"received {len(_received)} echo(s)")

        event.handler_unregister(
            event.ESP_HOSTED_EVENT, event.ESP_HOSTED_EVENT_PEER_DATA_RX, _on_peer_data
        )
        peer_data.unregister_event_thunks()
        event.loop_delete_default()
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        eh_host.deinit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
