"""
Test: GPIO Expander via RPC.
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS


@pytest.mark.gpio
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestGpio:

    def test_gpio_operations(self, dut):
        """Full sequence test."""
        host, cp = dut[0], dut[1]

        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        r = eh_test_expect_exact(host, 'GPIO toggled successfully', timeout=10)
        assert r.ok, f'GPIO toggle: {r.matched}'

        r = eh_test_expect(host, r'GPIO.*level', timeout=5)
        assert r.ok, f'GPIO read: {r.matched}'
