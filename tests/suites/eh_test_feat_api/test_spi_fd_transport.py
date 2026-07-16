"""Real-hardware SPI full-duplex transport validation: P4 host <--SPI-FD--> C6 CP.

Ground-truth check for the SPI-FD path on silicon. Reaching the api_exerciser
'ready' line means the whole transport worked end-to-end on real hardware:
  - the host clocks its caps out (MOSI) and the CP receives them,
  - the CP's caps + ESPInit come back (MISO) and the host processes them,
  - RPC is negotiated.
A control-plane RPC round-trip then proves data flows both ways at runtime.

Run on the real P4-Function-EV bench:
    python3 tools/eh.py test hw          # EH_SUBSTRATE=hw -> SerialTarget

Requires the bench to declare it is SPI-wired: add "spi_fd" to `transports` in
tests/lab.local.json (the board symbol in host_board_sdkconfig auto-selects the
board's SPI pins + reset line). A bench that is not SPI-wired SKIPs.

This same test also runs on the emu substrate (`eh.py test emu`), so it is the
exact A/B: pass on hw + fail on emu => the emulator's SPI model is at fault, not
the firmware.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out']
EXAMPLE = 'system/api_exerciser'


@pytest.mark.system
@pytest.mark.retired("Fully covered by test_api_exerciser[spi_fd]: its first "
                     "assertion is 'EH api_exerciser ready' (= this bring-up), "
                     "on the same substrates.")
def test_spi_fd_bringup(bench):
    """P4<->C6 SPI-FD reaches api_exerciser 'ready' — bidirectional transport up."""
    b = bench(EXAMPLE, 'mcu_host', 'spi_fd', timeout='150s')
    r = eh_test_expect(b['host'], r'EH api_exerciser ready', fail=FAIL, timeout=120)
    assert r.ok, f'SPI-FD bring-up to ready: {r.matched}'


@pytest.mark.system
@pytest.mark.retired("Fully covered by test_api_exerciser[spi_fd]: it asserts "
                     "'ready' then the same sys_fw_version round-trip plus 11 "
                     "more RPCs, on the same substrates.")
def test_spi_fd_rpc_roundtrip(bench):
    """A control-plane RPC (sys_fw_version) round-trips over SPI-FD: the request
    leaves the host (MOSI) and the CP's response returns (MISO)."""
    b = bench(EXAMPLE, 'mcu_host', 'spi_fd', timeout='150s')
    host = b['host']
    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=120)
    assert r.ok, f'ready: {r.matched}'
    host.write('sys_fw_version')
    r = eh_test_expect(host, r'EH rc=0 cmd=sys_fw_version ver=\d+\.\d+\.\d+',
                       fail=FAIL, timeout=20)
    assert r.ok, f'sys_fw_version round-trip: {r.matched}'
