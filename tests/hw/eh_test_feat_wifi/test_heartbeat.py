"""
Test: Heartbeat event flow.
"""

import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, eh_test_expect_exact, eh_test_verify_transport, eh_test_verify_cp_boot, FATAL_PATTERNS


@pytest.mark.system
@pytest.mark.parametrize('count', [2], indirect=True)
@pytest.mark.parametrize('target', ['esp32p4|esp32c6'], indirect=True)
class TestHeartbeat:

    def test_heartbeat_flow(self, dut):
        """Wait for periodic HEARTBEAT event.

        Note: When run after wifi_connect, boot/transport events are already
        consumed. Only check for the periodic heartbeat tick which repeats.
        """
        host, cp = dut[0], dut[1]

        r = eh_test_expect_exact(host, 'HEARTBEAT', timeout=65)
        assert r.ok, f'HB tick event: {r.matched}'
