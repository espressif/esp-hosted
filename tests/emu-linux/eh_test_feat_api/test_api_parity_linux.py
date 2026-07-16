"""MCU<->Linux parity: the SAME api_exerciser commands + result contract, driven
on the Linux userspace host (vserial <-> shim <-> emu C6) as on the mcu bench.
Proves the shared eh_api_cmd corpus + the posix esp_console port behave
identically across host flavors — "supported on one → supported on the other".
(System surface here; wifi/gpio/coex are feat-gated and enabled per host.)"""

import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['could not connect', 'chip_id mismatch']


@pytest.mark.system
@pytest.mark.sanity  # sanity: linux (802.3) substrate parity
def test_api_exerciser_linux(linux_bench):
    host = linux_bench('system/api_exerciser')['host']

    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=45)
    assert r.ok, f'linux ready: {r.matched}'

    # Cross-host parity: the SAME exerciser commands + EH result contract the mcu
    # bench asserts, now driven on the Linux host (control-plane RPC over vserial).
    host.write('sys_fw_version')
    r = eh_test_expect(host, r'EH rc=0 cmd=sys_fw_version ver=\d+\.\d+\.\d+', fail=FAIL, timeout=30)
    assert r.ok, f'linux sys_fw_version: {r.matched}'

    host.write('sys_get_mac sta')
    r = eh_test_expect(host, r'EH rc=0 cmd=sys_get_mac mac=([0-9a-f]{2}:){5}[0-9a-f]{2}',
                       fail=FAIL, timeout=30)
    assert r.ok, f'linux sys_get_mac: {r.matched}'

    # wifi control-plane round-trip over vserial (FEAT_WIFI enabled on linux too)
    host.write('wifi_set_ps 2')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_set_ps', fail=FAIL, timeout=30)
    assert r.ok, f'linux wifi_set_ps: {r.matched}'
    host.write('wifi_get_ps')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_ps ps=2', fail=FAIL, timeout=30)
    assert r.ok, f'linux wifi_get_ps round-trip: {r.matched}'
