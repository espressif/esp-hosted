"""
Test: Boot handshake + WiFi scan + FW version.

CP:   minimal/wifi (ESP32-C6)
Host: host_minimal_test (ESP32-P4)
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS


@pytest.mark.boot
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestBootHandshake:

    def test_full_boot_sequence(self, dut):
        """Boot → transport → FW version → WiFi scan → complete."""
        host, cp = dut[0], dut[1]

        # PREREQ: CP boots
        r = eh_test_verify_cp_boot(cp, timeout=10)
        assert r.ok, f'CP boot: {r.matched}'

        # PREREQ: Transport handshake
        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        # FW version query
        r = eh_test_expect_exact(host, 'TEST_PASS', timeout=10)
        assert r.ok, f'FW version: {r.matched}'

        # WiFi scan
        r = eh_test_expect_exact(host, 'WiFi scan found', timeout=15)
        assert r.ok, f'WiFi scan: {r.matched}'

        # All complete
        r = eh_test_expect_exact(host, 'TESTS COMPLETE', timeout=5)
        assert r.ok, f'Complete: {r.matched}'
