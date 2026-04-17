"""
Test: Peer Data Transfer (Custom RPC echo).
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS


@pytest.mark.peer_data
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestPeerData:

    def test_peer_data_echo(self, dut):
        """Full sequence test."""
        host, cp = dut[0], dut[1]

        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        # Wait for CP features to finish init — peer_data handler registration
        # happens in auto_feat_init_task after transport is up. Host sends
        # immediately but CP may not have registered the handler yet.
        r = eh_test_expect(host, r'Coprocessor Boot-up|callbacks registered', timeout=10)
        assert r.ok, f'CP ready: {r.matched}'

        r = eh_test_expect(host, r'MEOW.*Verified, all OK', timeout=15)
        assert r.ok, f'CAT echo: {r.matched}'

        r = eh_test_expect(host, r'WOOF.*Verified, all OK', timeout=10)
        assert r.ok, f'DOG echo: {r.matched}'

        r = eh_test_expect(host, r'HELLO.*Verified, all OK', timeout=10)
        assert r.ok, f'HUMAN echo: {r.matched}'

        r = eh_test_expect(host, r'GHOST|No.*handler|overflow|unregistered', timeout=10)
        assert r.ok, f'GHOST unregistered: {r.matched}'
