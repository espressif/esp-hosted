"""More B2 wifi coverage as data: max_tx_power + mode round-trips, softap config
round-trip, bssid-pinned marshalling, the NULL-arg negative guard, and scan.
Substrate-agnostic via the shared `bench` factory."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out']
EX = 'system/api_exerciser'


def _ready(bench, t):
    host = bench(EX, 'mcu_host', t, timeout='120s')['host']
    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=60)
    assert r.ok, f'[{t}] ready: {r.matched}'
    return host


def _ok(host, t, cmd, extra=''):
    host.write(cmd)
    pat = r'EH rc=0 cmd=' + cmd.split()[0] + ((' ' + extra) if extra else '')
    r = eh_test_expect(host, pat, fail=FAIL, timeout=20)
    assert r.ok, f'[{t}] {cmd}: {r.matched}'


@pytest.mark.system
@pytest.mark.retired("subsumed by test_control_plane — scalar set/get (sdio/uart/spi_hd)")
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_fd'])
def test_wifi_scalar_more(bench, transport):
    h = _ready(bench, transport)
    _ok(h, transport, 'wifi_set_max_tx_power 40')
    h.write('wifi_get_max_tx_power')
    r = eh_test_expect(h, r'EH rc=0 cmd=wifi_get_max_tx_power power=\d+', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] get_max_tx_power: {r.matched}'
    _ok(h, transport, 'wifi_set_mode 3'); _ok(h, transport, 'wifi_get_mode', 'mode=3')
    _ok(h, transport, 'wifi_set_mode 1'); _ok(h, transport, 'wifi_get_mode', 'mode=1')


@pytest.mark.system
@pytest.mark.retired("subsumed by test_control_plane — ap config round-trip (sdio/uart/spi_hd)")
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_fd'])
def test_softap_config_roundtrip(bench, transport):
    h = _ready(bench, transport)
    _ok(h, transport, 'wifi_set_mode 3')  # APSTA so AP config applies
    for c in ('wifi_cfg_reset', 'wifi_cfg_set ap_ssid apnet',
              'wifi_cfg_set ap_channel 6', 'wifi_set_config ap'):
        _ok(h, transport, c)
    h.write('wifi_get_config ap')
    r = eh_test_expect(h, r'EH rc=0 cmd=wifi_get_config ssid=apnet', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] softap config round-trip: {r.matched}'


@pytest.mark.system
@pytest.mark.retired("subsumed by test_control_plane — bssid marshalling + null-arg guard (sdio/uart/spi_hd)")
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_fd'])
def test_sta_bssid_and_null_guard(bench, transport):
    h = _ready(bench, transport)
    for c in ('wifi_cfg_reset', 'wifi_cfg_set sta_ssid pinned',
              'wifi_cfg_set sta_bssid aa:bb:cc:dd:ee:ff', 'wifi_set_config sta'):
        _ok(h, transport, c)
    # NULL-arg guard: host returns INVALID_ARG (rc != 0, err field present)
    h.write('wifi_set_config_null')
    r = eh_test_expect(h, r'EH rc=-?[1-9]\d* cmd=wifi_set_config_null err=\w+', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] null-arg guard: {r.matched}'


@pytest.mark.system
@pytest.mark.retired("subsumed by test_control_plane — scan clear/count/start/stop/dump (sdio/uart/spi_hd)")
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_fd'])
def test_scan(bench, transport):
    h = _ready(bench, transport)
    _ok(h, transport, 'wifi_clear_ap_list')
    h.write('wifi_scan_get_ap_num')
    r = eh_test_expect(h, r'EH rc=0 cmd=wifi_scan_get_ap_num num=\d+', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] scan_get_ap_num: {r.matched}'
    # scan itself: emu scan support varies, HW does a real scan — assert DRIVABLE
    # (well-formed, no crash) so the one body holds on every substrate.
    h.write('wifi_scan_start 1')
    r = eh_test_expect(h, r'EH rc=-?\d+ cmd=wifi_scan_start', fail=FAIL, timeout=30)
    assert r.ok, f'[{transport}] scan_start drivable: {r.matched}'
