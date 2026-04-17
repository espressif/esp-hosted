"""
Test: WiFi connect + events. Requires iPhone hotspot.
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS

WIFI_FAIL = FATAL_PATTERNS + [
    'connect to the AP fail',
    'Failed to connect to SSID',
    'HEARTBEAT timeout',
    'transport failure',
]


@pytest.mark.wifi
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestWifiConnect:

    def test_wifi_connect_flow(self, dut):
        """Transport → FW ver → heartbeat → STA → connect → GOT_IP."""
        host, cp = dut[0], dut[1]

        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        r = eh_test_expect_exact(host, 'FW Version', timeout=5)
        assert r.ok, f'FW version: {r.matched}'

        r = eh_test_expect_exact(host, 'heartbeat timer started', timeout=10)
        assert r.ok, f'Heartbeat: {r.matched}'

        r = eh_test_expect_exact(host, 'wifi station started', timeout=10)
        assert r.ok, f'STA started: {r.matched}'

        r = eh_test_expect_exact(host, 'esp_wifi_remote_connect', timeout=10)
        assert r.ok, f'Connect attempt: {r.matched}'

        r = eh_test_expect_exact(host, 'Station mode: Connected', fail=WIFI_FAIL, timeout=15)
        assert r.ok, f'Connected: {r.matched}'

        # HEARTBEAT timeout can fire during WiFi reconnect — don't treat as fatal here
        r = eh_test_expect(host, r'got ip|GOT_IP|IPv4|sta ip:', fail=FATAL_PATTERNS, timeout=15)
        assert r.ok, f'GOT_IP: {r.matched}'
