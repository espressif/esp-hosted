"""
Test: Memory Monitor via RPC.
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS


@pytest.mark.mem_monitor
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestMemMonitor:

    def test_mem_monitor_stats(self, dut):
        """Full sequence test."""
        host, cp = dut[0], dut[1]

        r = eh_test_verify_transport(host, timeout=15)
        assert r.ok, f'Transport: {r.matched}'

        r = eh_test_expect_exact(host, 'Current Co-processor Mem Info', timeout=10)
        assert r.ok, f'Mem info header: {r.matched}'

        # Output order: header → config → interval → heap table
        r = eh_test_expect(host, r'mem monitoring config.*Enabled', timeout=5)
        assert r.ok, f'Monitor enabled: {r.matched}'

        r = eh_test_expect_exact(host, 'reporting interval', timeout=5)
        assert r.ok, f'Interval set: {r.matched}'

        r = eh_test_expect_exact(host, 'Internal DMA', timeout=5)
        assert r.ok, f'Heap values: {r.matched}'
