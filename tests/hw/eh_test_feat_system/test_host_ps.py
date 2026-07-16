"""
Test: Host Power Save — automatic shutdown/wake cycle.

CP:   extensions/host_power_save (ESP32-C6)
Host: host_shuts_down_slave_to_power_save (ESP32-P4)

Host app runs autonomous power save cycles:
  1. Connect WiFi
  2. Deinit esp-hosted (shutdown slave)
  3. Wait 5s in low-power
  4. Reinit + reconnect WiFi
  5. Repeat
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, FATAL_PATTERNS

PS_FAIL = FATAL_PATTERNS + [
    'transport failure',
    'HEARTBEAT timeout',
    'eh_cp_deinit failed',
]


@pytest.mark.host_ps
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestHostPowerSave:

    def test_host_ps_cycle(self, dut):
        """Transport -> WiFi -> shutdown slave -> wake -> reconnect."""
        host, cp = dut[0], dut[1]

        # ── Boot + transport ──
        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        # ── Initial WiFi connect ──
        r = eh_test_expect(host, r'Got IP|got ip|IP_EVENT', fail=PS_FAIL, timeout=20)
        assert r.ok, f'WiFi connect: {r.matched}'

        # ── Power save cycle 1: shutdown slave ──
        r = eh_test_expect_exact(host, 'ShutDown slave to save power', fail=PS_FAIL, timeout=15)
        assert r.ok, f'PS cycle start: {r.matched}'

        r = eh_test_expect_exact(host, 'de-initialized', fail=PS_FAIL, timeout=10)
        assert r.ok, f'Slave deinit: {r.matched}'

        # ── Wake up + reinit ──
        r = eh_test_expect_exact(host, 'Reinitializing', fail=PS_FAIL, timeout=10)
        assert r.ok, f'Reinit: {r.matched}'

        r = eh_test_expect_exact(host, 'Transport is ready', fail=PS_FAIL, timeout=20)
        assert r.ok, f'Transport restored: {r.matched}'

        # ── WiFi reconnect ──
        r = eh_test_expect(host, r'Got IP|got ip|IP_EVENT', fail=PS_FAIL, timeout=20)
        assert r.ok, f'WiFi reconnect: {r.matched}'

        # ── Cycle complete ──
        r = eh_test_expect_exact(host, 'recovery sequence complete', fail=PS_FAIL, timeout=10)
        assert r.ok, f'Cycle complete: {r.matched}'
