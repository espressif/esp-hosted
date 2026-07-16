"""SPI half-duplex transport validation: P4 host <--SPI-HD--> C6 CP.

Ground-truth check for the SPI-HD path. Reaching the api_exerciser 'ready' line
means the half-duplex transport worked end-to-end:
  - the host reads SLAVE_READY (0xEE) over RDBUF, opens the datapath (WRBUF
    DATAPATH_ON), then the CP asserts data-ready and the host drains the CP's
    startup event + caps over RDDMA,
  - the host clocks its own caps out over WRDMA,
  - RPC is negotiated.
A control-plane RPC round-trip then proves data flows both ways at runtime.

Parametrized over the data-line count: plain `spi_hd` (Kconfig default = 4-line),
plus explicit 1/2/4-line builds. Each is a distinct FW build (distinct CONFIG
overlay -> distinct build-cache key) and its own emu launch.

Runs on the emu substrate (`eh.py test emu`) and on real HW (`eh.py test hw`)
when the bench declares "spi_hd" (or spi_hd_1/2/4) in tests/lab.local.json. On a
substrate that cannot provide the wire, the bench factory SKIPs; on the emu, if
the resolved esp-emu lacks --hosted-spi-hd it SKIPs too.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out']
EXAMPLE = 'system/api_exerciser'

# Plain 'spi_hd' == Kconfig default (4-line); the numbered variants pin the
# data-line count explicitly on both CP and host.
LINE_MODES = ['spi_hd', 'spi_hd_1', 'spi_hd_2', 'spi_hd_4']


@pytest.mark.system
@pytest.mark.retired("Subsumed by test_spi_hd_rpc_roundtrip below: its first "
                     "assertion is the same 'ready' bring-up, then an RPC.")
@pytest.mark.parametrize('transport', LINE_MODES)
def test_spi_hd_bringup(bench, transport):
    """P4<->C6 SPI-HD reaches api_exerciser 'ready' — half-duplex transport up."""
    b = bench(EXAMPLE, 'mcu_host', transport, timeout='150s')
    r = eh_test_expect(b['host'], r'EH api_exerciser ready', fail=FAIL, timeout=120)
    assert r.ok, f'[{transport}] SPI-HD bring-up to ready: {r.matched}'


# The 4-line configs ('spi_hd' default and explicit 'spi_hd_4') are covered by
# test_api_exerciser[spi_hd]; only the 1- and 2-line configs are unique here.
_HD_RPC_MODES = [
    pytest.param('spi_hd',   marks=pytest.mark.retired(
        "4-line default == test_api_exerciser[spi_hd].")),
    'spi_hd_1',
    'spi_hd_2',
    pytest.param('spi_hd_4', marks=pytest.mark.retired(
        "Explicit 4-line == default; covered by test_api_exerciser[spi_hd].")),
]


@pytest.mark.system
@pytest.mark.parametrize('transport', _HD_RPC_MODES)
def test_spi_hd_rpc_roundtrip(bench, transport):
    """A control-plane RPC (sys_fw_version) round-trips over SPI-HD: the request
    leaves the host (WRDMA) and the CP's response returns (RDDMA). Unique
    coverage = the 1-/2-line data-line configs (4-line is the exerciser's)."""
    b = bench(EXAMPLE, 'mcu_host', transport, timeout='150s')
    host = b['host']
    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=120)
    assert r.ok, f'[{transport}] ready: {r.matched}'
    host.write('sys_fw_version')
    r = eh_test_expect(host, r'EH rc=0 cmd=sys_fw_version ver=\d+\.\d+\.\d+',
                       fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] sys_fw_version round-trip: {r.matched}'
