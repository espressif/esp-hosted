"""
Test: Real OTA update via partition method.
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS


@pytest.mark.system
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestOta:

    def test_ota_full_cycle(self, dut):
        """Full sequence test."""
        host, cp = dut[0], dut[1]

        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        r = eh_test_expect_exact(host, 'Starting slave OTA update', timeout=10)
        assert r.ok, f'OTA starts: {r.matched}'

        r = eh_test_expect(host, r'Found partition.*slave_fw', timeout=5)
        assert r.ok, f'Partition found: {r.matched}'

        r = eh_test_expect_exact(host, 'Partition OTA completed successfully', timeout=90)
        assert r.ok, f'OTA transfer: {r.matched}'

        r = eh_test_expect_exact(host, 'New firmware activated', timeout=10)
        assert r.ok, f'Activated: {r.matched}'

        r = eh_test_expect_exact(host, 'Restarting host to resync', timeout=10)
        assert r.ok, f'Host resync: {r.matched}'
