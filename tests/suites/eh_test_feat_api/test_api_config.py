"""wifi struct-config round-trip (B2: sta_config_roundtrip): stage a config via
field-setters, apply it, read it back, compare — all as data over the console,
no per-scenario firmware. Exercises eh_host_wifi_set_config/get_config."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out']


@pytest.mark.system
@pytest.mark.retired("subsumed by test_control_plane — sta config round-trip (sdio/uart/spi_hd)")
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_fd'])
def test_wifi_sta_config_roundtrip(bench, transport):
    b = bench('system/api_exerciser', 'mcu_host', transport, timeout='120s')
    host = b['host']

    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=60)
    assert r.ok, f'[{transport}] ready: {r.matched}'

    for cmd in ('wifi_cfg_reset',
                'wifi_cfg_set sta_ssid testnet',
                'wifi_cfg_set sta_channel 6',
                'wifi_set_config sta'):
        host.write(cmd)
        r = eh_test_expect(host, r'EH rc=0 cmd=' + cmd.split()[0], fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] {cmd}: {r.matched}'

    # read it back — the SSID we staged must survive set→get
    host.write('wifi_get_config sta')
    r = eh_test_expect(host, r'EH rc=0 cmd=wifi_get_config ssid=testnet', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] sta config round-trip: {r.matched}'
