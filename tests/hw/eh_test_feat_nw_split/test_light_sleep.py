"""
Test: Light Sleep — CP enters light sleep, transport still works.

CP:   extensions/light_sleep (ESP32-C6)
Host: host_network_split__power_save (ESP32-P4)

Verifies CP with PM/light_sleep enabled still maintains transport
and WiFi connectivity.
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, FATAL_PATTERNS
from infra.config import eh_test_config_load

LS_FAIL = FATAL_PATTERNS + [
    'connect to the AP fail',
    'transport failure',
]


@pytest.mark.light_sleep
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestLightSleep:

    def test_light_sleep_transport(self, dut):
        """Transport works with CP in light sleep mode."""
        host, cp = dut[0], dut[1]
        cfg = eh_test_config_load()

        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        # Wait for CLI prompt
        r = eh_test_expect(host, r'iperf>|Steps to test', timeout=10)
        assert r.ok, f'CLI ready: {r.matched}'

        # Connect WiFi via CLI — validates transport works with CP light sleep
        host.write(f'sta_connect {cfg.wifi_ssid} {cfg.wifi_password}\n')

        r = eh_test_expect(host, r'WIFI_CONNECT_START|Connecting to', fail=LS_FAIL, timeout=10)
        assert r.ok, f'Connect started: {r.matched}'

        r = eh_test_expect(host, r'Got IP|got ip|IP_EVENT|STA_GOT_IP', fail=LS_FAIL, timeout=20)
        assert r.ok, f'GOT_IP: {r.matched}'

        # Ping to verify data path works through light-sleeping CP
        host.write('ping 8.8.8.8 -c 3\n')

        r = eh_test_expect(host, r'bytes from|time=|seq=', fail=LS_FAIL, timeout=15)
        assert r.ok, f'Ping reply: {r.matched}'
