"""
Test: Network Split — basic connectivity via CLI.

CP:   extensions/network_split/station (ESP32-C6)
Host: host_network_split__power_save (ESP32-P4)

Verifies:
  - Transport handshake
  - WiFi connect via CLI
  - Ping through CP (packet routing works)
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, FATAL_PATTERNS
from infra.config import eh_test_config_load

NW_FAIL = FATAL_PATTERNS + [
    'connect to the AP fail',
    'Failed to connect',
    'HEARTBEAT timeout',
    'transport failure',
]


@pytest.mark.nw_split
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestNwSplitBasic:

    def test_nw_split_connect_and_ping(self, dut):
        """Transport -> CLI sta_connect -> GOT_IP -> ping (proves routing)."""
        host, cp = dut[0], dut[1]
        cfg = eh_test_config_load()

        # ── Stage 1: Transport handshake ──
        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        # Wait for CLI prompt
        r = eh_test_expect(host, r'iperf>|Steps to test', timeout=10)
        assert r.ok, f'CLI ready: {r.matched}'

        # ── Stage 2: WiFi connect via CLI ──
        host.write(f'sta_connect {cfg.wifi_ssid} {cfg.wifi_password}\n')

        r = eh_test_expect(host, r'WIFI_CONNECT_START|Connecting to', fail=NW_FAIL, timeout=10)
        assert r.ok, f'Connect started: {r.matched}'

        r = eh_test_expect(host, r'Got IP|got ip|IP_EVENT|STA_GOT_IP', fail=NW_FAIL, timeout=20)
        assert r.ok, f'GOT_IP: {r.matched}'

        # ── Stage 3: Ping through CP (verifies NW split routing) ──
        # ICMP echo requests route to CP, replies route to BOTH stacks
        host.write('ping 8.8.8.8 -c 3\n')

        r = eh_test_expect(host, r'bytes from|time=|seq=', fail=NW_FAIL, timeout=15)
        assert r.ok, f'Ping reply: {r.matched}'
