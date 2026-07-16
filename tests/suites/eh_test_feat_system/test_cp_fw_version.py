"""get CP firmware version — the transport bring-up smoke, substrate-agnostic.

Runs on ANY substrate via the shared `bench` factory: emu sweeps sdio+uart; a HW
bench runs whatever wire it's physically provisioned for and SKIPs the rest. Same
body, no substrate branches — the Target contract's whole point.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out', 'chip_id mismatch', 'refusing']


@pytest.mark.system
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: fw-version handshake on the reliable wire
    'uart', 'spi_fd', 'spi_hd'])
class TestCpFwVersion:

    def test_get_cp_fw_version(self, bench, transport):
        b = bench('system/get_cp_fw_version', 'mcu_host', transport)
        host = b['host']

        # transport handshake (the transport-specific bit under test)
        r = eh_test_expect(host, r'slave chip id: 0x0d', fail=FAIL, timeout=45)
        assert r.ok, f'[{transport}] transport up: {r.matched}'

        # RPC round-trip: host queries the CP firmware version
        r = eh_test_expect(host, r'CP firmware: \d+\.\d+\.\d+', fail=FAIL, timeout=30)
        assert r.ok, f'[{transport}] RPC get_cp_fw_version: {r.matched}'
