"""Host power-save + wake — substrate-agnostic, capability-gated.

Verifies the power-save PATH on any bench that has a host-wake GPIO (the 'wake'
cap): connect Wi-Fi -> host deep-sleep + CP light-sleep (ENTRY, always checked) ->
external packet wakes the host via the CP (WAKE).

The wake STIMULUS differs by substrate but the body doesn't branch on substrate:
the emu bench hands back a `net` stimulus; a serial-HW bench on the AP's network
has the runner send a real packet to the host's IP. A board without the wake GPIO
(no 'wake' cap, e.g. the Core board) skips — power-save is board-dependent.
"""
import os
import re
import socket
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

PS_EXAMPLE = 'power_save/host+cp/network_split__host_deep_sleep_cp_light_sleep'
PS_FAIL = FATAL_PATTERNS + ['bring-up timed out', 'Firmware abort', 'CORRUPT HEAP']


def _shoot(ip, port=22):
    """Fire a TCP SYN at the sleeping host's IP — the CP (awake) sees the priority
    packet and asserts the wake GPIO. HW analogue of the emu Net stimulus."""
    try:
        s = socket.socket()
        s.settimeout(2)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
        s.connect((ip, port))
        s.sendall(b"wakeup-host")
        s.close()
    except OSError:
        pass


@pytest.mark.power_save
@pytest.mark.sanity  # sanity: representative host power-save cycle
def test_host_power_save(bench, bench_caps, wifi_ap):
    if "wake" not in bench_caps:
        pytest.skip("board has no host-wake GPIO ('wake' cap) — power-save N/A here")
    b = bench(PS_EXAMPLE, "esp_host", "sdio", wake=True, timeout="200s")
    host, cp = b["host"], b["cp"]

    # ── bring-up + Wi-Fi connect ──
    assert eh_test_expect(host, r"host>", fail=PS_FAIL, timeout=45).ok, "CLI ready"
    # Creds are substrate-appropriate (see the wifi_ap fixture): the emu's modeled
    # SoftAP on emu, the real AP on hw/rpi. Config-driven — never hardcoded here.
    ssid, pw = (wifi_ap["ssid"], wifi_ap["password"]) if wifi_ap else ("myssid", "mypassword")
    host.write(f"sta {ssid} {pw}")
    r = eh_test_expect(host, r"IP_EVENT_STA_GOT_IP", fail=PS_FAIL, timeout=60)
    assert r.ok, f"GOT_IP: {r.matched}"
    ip_buf = (host.pexpect_proc.before or b"").decode("utf-8", "replace")

    # ── ENTRY: host deep-sleep -> CP light-sleep ──
    host.write("host_power_save")
    r = eh_test_expect(cp, r"Light sleep ENABLED", fail=PS_FAIL, timeout=45)
    assert r.ok, f"CP light-sleep on host power-save: {r.matched}"

    # ── WAKE: emu provides a stimulus; on HW send a packet to the host's real IP ──
    net = b.get("net")
    if net is not None:
        net.wake()
    else:
        m = re.search(r"(\d{1,3}(?:\.\d{1,3}){3})", ip_buf)
        if not m or m.group(1).startswith("0."):
            pytest.skip("power-save ENTRY verified; no host IP parsed for the wake packet")
        _shoot(m.group(1))
    r = eh_test_expect(host, r"Host woke up from power save", fail=PS_FAIL, timeout=45)
    assert r.ok, f"host wake: {r.matched}"
