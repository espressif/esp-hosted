"""Coprocessor OTA — emulator, self-contained via LittleFS.

The host flashes a new CP firmware image over the hosted link (begin → write* →
end). To keep it network-free on the emu, we use the example's LittleFS OTA
method: the CP app image is staged into an on-device LittleFS `storage` partition
at build time and the host reads it straight from flash — no HTTPS server.

The image is the SAME CP app the bench just built (app-only .bin), injected into
the host build's LittleFS source dir by the bench (BenchSpec.cp_app_to_host) —
sandbox-safe (it lands only in the COW build copy). Reference: examples/ota/
coprocessor_ota with CONFIG_OTA_METHOD_LITTLEFS.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out', 'OTA failed with error',
                         'No .bin files found']
EX = 'ota/coprocessor_ota'


@pytest.mark.ota
@pytest.mark.second_chance
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: OTA verified on sdio (uart=134s)
    'uart', 'spi_fd', 'spi_hd',
])
def test_ota_littlefs(bench, transport):
    """Positive: the host reads the staged CP image from its LittleFS partition and
    OTAs the coprocessor over the link — begin/write/end complete with no network.

    Checked PER-PHASE (host-init → transfer-start → complete → activate) so a hang
    fails fast at the phase that stalled instead of waiting out one long timeout with
    no clue where it stuck. Only the transfer itself keeps a long wait — the firmware
    logs no intermediate progress and UART is ~140s — and even that is scoped tight on
    the fast wires so a stall there fails in 60s (not 240s) off UART."""
    b = bench(EX, 'mcu_host', transport, timeout='300s',
              overlay=['CONFIG_OTA_METHOD_LITTLEFS=y'],
              cp_app_to_host='components/ota_littlefs/slave_fw_bin/cp_app.bin')
    host = b['host']
    # 1. host-side hosted link up
    r = eh_test_expect(host, r'ESP-Hosted initialized successfully', fail=FAIL, timeout=30)
    assert r.ok, f'[{transport}] OTA host-init: {r.matched}'
    # 2. image found + verified in LittleFS, transfer begins (fails fast on missing/bad image)
    r = eh_test_expect(host, r'Starting OTA from LittleFS', fail=FAIL, timeout=30)
    assert r.ok, f'[{transport}] OTA transfer-start (image staged/verified): {r.matched}'
    # 3. the transfer — no progress logging, UART ~140s so it needs the long wait; the
    #    fast wires (~25s) fail a stall in 60s instead of 240s
    xfer_timeout = 200 if transport == 'uart' else 60
    r = eh_test_expect(host, r'OTA completed successfully', fail=FAIL, timeout=xfer_timeout)
    assert r.ok, f'[{transport}] OTA transfer/complete: {r.matched}'
    # 4. new image activated on the slave
    r = eh_test_expect(host, r'New firmware activated', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] OTA activate: {r.matched}'
