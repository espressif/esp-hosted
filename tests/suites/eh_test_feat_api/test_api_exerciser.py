"""API-exerciser (B2): drive the native eh_host_* surface over the console and
assert the uniform result contract

    EH rc=<int> cmd=<name> [k=v ...]

Each scenario is DATA here (a console command + expected line), not per-scenario
firmware. Firmware: examples/system/api_exerciser. Substrate-agnostic: same body
on emu and HW via the shared `bench` factory.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out']
EXAMPLE = 'system/api_exerciser'


@pytest.mark.system
@pytest.mark.parametrize('transport', [
    # sdio/uart/spi_hd: the full ~40-RPC corpus now runs in test_control_plane.
    # spi_fd stays here as the short, reliable spi_fd API/wire smoke — spi_fd is
    # excluded from the heavier control_plane sweep (see that file's note).
    pytest.param('sdio',   marks=pytest.mark.retired("full corpus in test_control_plane[sdio]")),
    pytest.param('uart',   marks=pytest.mark.retired("full corpus in test_control_plane[uart]")),
    'spi_fd',
    pytest.param('spi_hd', marks=pytest.mark.retired("full corpus in test_control_plane[spi_hd]")),
])
@pytest.mark.second_chance
def test_api_exerciser(bench, transport):
    b = bench(EXAMPLE, 'mcu_host', transport, timeout='120s')
    host = b['host']

    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=60)
    assert r.ok, f'[{transport}] ready: {r.matched}'

    # system: CP firmware version
    host.write('sys_fw_version')
    r = eh_test_expect(host, r'EH rc=0 cmd=sys_fw_version ver=\d+\.\d+\.\d+', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] sys_fw_version: {r.matched}'

    # system: STA MAC readback
    host.write('sys_get_mac sta')
    r = eh_test_expect(host, r'EH rc=0 cmd=sys_get_mac mac=([0-9a-f]{2}:){5}[0-9a-f]{2}',
                       fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] sys_get_mac: {r.matched}'

    # wifi: power-save exact round-trip (set -> get returns the set value)
    host.write('wifi_set_ps 2')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_set_ps', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_set_ps: {r.matched}'
    host.write('wifi_get_ps')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_ps ps=2', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_get_ps round-trip: {r.matched}'

    # wifi: country round-trip
    host.write('wifi_set_country US 0')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_set_country', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_set_country: {r.matched}'
    host.write('wifi_get_country')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_country cc=US', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_get_country round-trip: {r.matched}'

    # wifi: mode round-trip (STA is idempotent — proves set/get agree)
    host.write('wifi_set_mode 1')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_set_mode', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_set_mode: {r.matched}'
    host.write('wifi_get_mode')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_mode mode=1', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_get_mode round-trip: {r.matched}'

    # wifi: getters return well-formed values on a started stack
    host.write('wifi_get_max_tx_power')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_max_tx_power power=-?\d+', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_get_max_tx_power: {r.matched}'
    host.write('wifi_get_protocol sta')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_protocol proto=0x[0-9a-f]+', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_get_protocol: {r.matched}'
    host.write('wifi_get_bandwidth sta')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_bandwidth bw=\d+', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_get_bandwidth: {r.matched}'

    # negative: a missing arg is rejected with a non-zero rc, no crash
    host.write('wifi_set_ps')
    r = eh_test_expect(host, r'EH rc=-1 cmd=wifi_set_ps err=USAGE', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] wifi_set_ps usage guard: {r.matched}'
