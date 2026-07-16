"""
Test: BT/BLE via NimBLE VHCI.
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS


@pytest.mark.bt
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestBt:

    def test_ble_nimble_vhci(self, dut):
        """Full sequence test."""
        host, cp = dut[0], dut[1]

        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        r = eh_test_expect_exact(host, 'HCI over SDIO', timeout=5)
        assert r.ok, f'BT capability: {r.matched}'

        r = eh_test_expect_exact(host, 'Host BT Support: Enabled', timeout=5)
        assert r.ok, f'VHCI enabled: {r.matched}'

        r = eh_test_expect_exact(host, 'BLE Host Task Started', timeout=15)
        assert r.ok, f'NimBLE started: {r.matched}'

        r = eh_test_expect_exact(host, 'GAP procedure initiated: advertise', timeout=15)
        assert r.ok, f'BLE advertising: {r.matched}'
