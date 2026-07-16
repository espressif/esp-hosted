"""
Test: Linux userspace host gets the CP firmware version, over the emulated C6.

Substrate = linux-shim (real Linux vserial host, no kernel, no mock) <-> shim <->
esp-emu C6 UART bridge. Same authoring surface as the emu/HW suites.
"""
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['chip_id mismatch', 'could not connect']


@pytest.mark.system
class TestLinuxCpFwVersion:

    def test_get_cp_fw_version(self, dut):
        host = dut[0]
        r = eh_test_expect(host, r'CP firmware: \d+\.\d+\.\d+', fail=FAIL, timeout=45)
        assert r.ok, f'linux host RPC get_cp_fw_version: {r.matched}'
