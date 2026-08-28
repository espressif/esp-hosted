"""
Test: Host power save — CP light-sleep + wake-on-packet.

CP:   power_save/network_split/host_deep_sleep_cp_light_sleep (ESP32-C6)
Host: power_save/network_split/host_deep_sleep_cp_light_sleep (ESP32-P4)

Verifies the whole wake surface:
  - WiFi connect via CLI (routing up)
  - host_power_save -> host deep-sleep, CP light-sleep
  - external packet on a host priority port wakes the host via the CP
  - link comes back cleanly (the bug surface we fixed)

Authored identically to the HW suite: same infra.expect_helper, same
dut[0]/dut[1] handles. Only the bench provider (conftest) differs — here it
is the C6+P4 esp-emu pair instead of serial-attached boards.
"""

import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

PS_FAIL = FATAL_PATTERNS + [
    'bring-up timed out',
    'Firmware abort',
    'Instruction access fault',
    'CORRUPT HEAP',
]


@pytest.mark.power_save
@pytest.mark.parametrize('rep', [1, 2])  # -n 2 runs two independent benches in parallel
class TestHostPowerSave:

    def test_cp_light_sleep_wake_on_packet(self, dut, net, rep):
        host, cp = dut[0], dut[1]

        # ── Stage 1: bring-up + WiFi connect ──
        r = eh_test_expect(host, r'host>', timeout=40)
        assert r.ok, f'CLI ready: {r.matched}'

        host.write('sta myssid mypassword')
        r = eh_test_expect(host, r'IP_EVENT_STA_GOT_IP', fail=PS_FAIL, timeout=60)
        assert r.ok, f'GOT_IP: {r.matched}'

        # ── Stage 2: host deep-sleep -> CP light-sleep ──
        host.write('host_power_save')
        r = eh_test_expect(cp, r'Light sleep ENABLED', fail=PS_FAIL, timeout=45)
        assert r.ok, f'CP light-sleep: {r.matched}'

        # ── Stage 3: external packet wakes the host via the CP ──
        net.wake()

        r = eh_test_expect(host, r'Host woke up from power save', fail=PS_FAIL, timeout=45)
        assert r.ok, f'Host wake: {r.matched}'
        r = eh_test_expect(host, r'slave chip id', fail=PS_FAIL, timeout=30)
        assert r.ok, f'Transport re-init: {r.matched}'
        r = eh_test_expect(
            cp, r'host  woke up|Host power save off - device fully ready',
            fail=PS_FAIL, timeout=30)
        assert r.ok, f'CP wake ack: {r.matched}'
        # (the _bench teardown runs settle_check — post-success crash catch)

# NOTE: the wake-flood regression (used to hit the START..CONNECTED RX race
# ~1/6) lives in test_power_save_flood.py; the admit-on-CONNECTED gate closes it
# (30/30 clean) — see decision-journal S7.
