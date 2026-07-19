"""SDIO software-aggregation (SW_AGGR) — smoke + host-first OTA migration drill.

SW_AGGR is negotiated at boot via the 0x1B buf-config TLV (mode==SW_AGGR both
directions); a stream/packet CP keeps the host on its unchanged rel-2 path
with a one-time warning. These tests pin BOTH sides of that contract on the
emu, sdio-only (the datapath is SDIO-specific by design).

The migration drill approximates a field rel-2 CP with a STREAM-mode build
(wire-equivalent; it advertises 0x1B mode=STREAM where a true rel-2 CP sends
no 0x1B — both take the same host fallback branch. The TLV-absent variant
needs a pre-rel-3 binary and runs on the HW bench instead).
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out', 'malformed SDIO buf-config',
                         'cannot allocate', 'refusing']

SW_AGGR_CP_OVL = ['CONFIG_EH_TRANSPORT_CP_SDIO_MODE_SW_AGGR=y']  # == default (explicit)
STREAM_CP_OVL = ['CONFIG_EH_TRANSPORT_CP_SDIO_MODE_STREAM=y']    # the rel-2-like wire
# multi-frame trace markers are compiled OUT in production — tests opt in
AGGR_TRACE_CP_OVL = SW_AGGR_CP_OVL + ['CONFIG_EH_TRANSPORT_CP_SDIO_AGGR_TRACE=y']
AGGR_TRACE_HOST_OVL = ['CONFIG_ESP_HOSTED_HOST_SDIO_AGGR_TRACE=y']


@pytest.mark.system
@pytest.mark.sanity
def test_sdio_sw_aggr_smoke(bench):
    """SW_AGGR CP + default host: negotiation selects the aggregating datapath
    and an RPC round-trip works over it (init event E2H, request H2E,
    response E2H — all through the aggregate paths)."""
    b = bench('system/get_cp_fw_version', 'mcu_host', 'sdio',
              cp_overlay=SW_AGGR_CP_OVL)
    host, cp = b['host'], b['cp']

    r = eh_test_expect(cp, r'SDIO datapath mode: SW_AGGR', fail=FATAL_PATTERNS, timeout=45)
    assert r.ok, f'CP booted the SW_AGGR datapath: {r.matched}'

    r = eh_test_expect(host, r'SDIO SW_AGGR negotiated', fail=FAIL, timeout=45)
    assert r.ok, f'host selected SW_AGGR from 0x1B: {r.matched}'

    r = eh_test_expect(host, r'CP firmware: \d+\.\d+\.\d+', fail=FAIL, timeout=30)
    assert r.ok, f'RPC round-trip over SW_AGGR: {r.matched}'


@pytest.mark.system
def test_sdio_stream_back_compat_warns(bench):
    """H3/D9: a non-aggregating (stream) CP keeps the host on the unchanged
    stream path — one warning, link fully functional."""
    b = bench('system/get_cp_fw_version', 'mcu_host', 'sdio',
              cp_overlay=STREAM_CP_OVL)   # SW_AGGR is the default now — force stream
    host = b['host']

    r = eh_test_expect(host, r'CP without SDIO SW_AGGR', fail=FAIL, timeout=45)
    assert r.ok, f'D9 back-compat warning: {r.matched}'

    r = eh_test_expect(host, r'CP firmware: \d+\.\d+\.\d+', fail=FAIL, timeout=30)
    assert r.ok, f'RPC round-trip on the stream fallback: {r.matched}'


@pytest.mark.ota
@pytest.mark.second_chance
def test_sdio_sw_aggr_ota_migration(bench):
    """Host-first migration drill (plan C5 gate), in rollout order:
    phase 1 = OTA sanity on the OLD wire (rel-3 host + stream CP: warning,
    link works, OTA itself completes over the stream path), THEN
    phase 2/3 = the CP reboots into SW_AGGR and the host switches live.
    Real multi-frame aggregation is proven separately by
    test_sdio_sw_aggr_bulk_really_aggregates (small frames here would ride
    the single-frame fast path / latency bypass and prove nothing)."""
    b = bench('ota/coprocessor_ota', 'mcu_host', 'sdio', timeout='300s',
              overlay=['CONFIG_OTA_METHOD_LITTLEFS=y'],
              cp_overlay=STREAM_CP_OVL,        # boot CP = rel-2-like stream wire
              cp_ota_overlay=SW_AGGR_CP_OVL,   # staged image = the new default
              cp_app_to_host='components/ota_littlefs/slave_fw_bin/cp_app.bin')
    host, cp = b['host'], b['cp']

    # Phase 1: new host + stream CP = rel-2-compatible mode + one warning.
    r = eh_test_expect(host, r'CP without SDIO SW_AGGR',
                       fail=FATAL_PATTERNS + ['bring-up timed out'], timeout=60)
    assert r.ok, f'phase1 back-compat warning: {r.matched}'

    # Phase 2: host OTAs the CP with the staged SW_AGGR image over the link.
    r = eh_test_expect(host, r'OTA completed successfully',
                       fail=FATAL_PATTERNS + ['OTA failed with error',
                                              'No .bin files found'], timeout=240)
    assert r.ok, f'phase2 OTA to SW_AGGR image: {r.matched}'

    # Phase 3: CP reboots into SW_AGGR; host re-negotiates and switches live.
    r = eh_test_expect(cp, r'SDIO datapath mode: SW_AGGR', fail=FATAL_PATTERNS, timeout=90)
    assert r.ok, f'phase3 CP rebooted into SW_AGGR: {r.matched}'

    r = eh_test_expect(host, r'SDIO SW_AGGR negotiated',
                       fail=FATAL_PATTERNS + ['malformed SDIO buf-config'], timeout=90)
    assert r.ok, f'phase3 host switched to SW_AGGR: {r.matched}'


@pytest.mark.system
@pytest.mark.second_chance
def test_sdio_raw_tp_cli(bench):
    """Raw throughput test stays trivially triggerable (kmod-parity): the
    api_exerciser CLI starts/stops the flood in both directions over the
    SW_AGGR wire, with per-second kbps stats on both ends."""
    b = bench('system/api_exerciser', 'mcu_host', 'sdio', timeout='180s')
    host, cp = b['host'], b['cp']

    r = eh_test_expect(host, r'SDIO SW_AGGR negotiated', fail=FAIL, timeout=45)
    assert r.ok, f'negotiated: {r.matched}'

    # Console REPL comes up AFTER transport bring-up — writing earlier drops
    # the command bytes on the floor (same gate as the api_exerciser suites).
    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=45)
    assert r.ok, f'console ready: {r.matched}'

    # RX: CP floods E2H — host must report non-zero Rx kbps
    host.write('raw_tp start rx')
    r = eh_test_expect(host, r'EH rc=0 cmd=raw_tp', fail=FAIL, timeout=15)
    assert r.ok, f'raw_tp start rx accepted: {r.matched}'
    r = eh_test_expect(host, r'sec Tx:\d+ Rx:[1-9]\d* Kbps', fail=FAIL, timeout=30)
    assert r.ok, f'host counts E2H flood: {r.matched}'

    # TX: host floods H2E — CP must report non-zero Rx kbps
    host.write('raw_tp start tx')
    r = eh_test_expect(host, r'EH rc=0 cmd=raw_tp', fail=FAIL, timeout=15)
    assert r.ok, f'raw_tp start tx accepted: {r.matched}'
    r = eh_test_expect(cp, r'sec\s+Rx: [1-9][0-9.]* Tx: [0-9.]+ kbps', fail=FATAL_PATTERNS, timeout=40)
    assert r.ok, f'CP counts H2E flood: {r.matched}'

    # STOP: both ends quiesce without error
    host.write('raw_tp stop')
    r = eh_test_expect(host, r'EH rc=0 cmd=raw_tp', fail=FAIL, timeout=15)
    assert r.ok, f'raw_tp stop accepted: {r.matched}'


@pytest.mark.system
@pytest.mark.second_chance
def test_sdio_sw_aggr_bulk_really_aggregates(bench):
    """REAL aggregation proof, CP->host direction: peer-data makes the CP batch
    its large, fragmented responses, so the CP packs >1 frame per SDIO transfer
    and the host de-aggregates >1 sub-frame — pinning the packer/walker on frames
    that genuinely share one transfer (not the single-frame fast path or the
    <=256 B latency bypass). (Throughput is a HW-bench gate — emu proves function.)

    host->CP is intentionally NOT asserted here: eh_host_peer_data_send() is a
    SYNCHRONOUS RPC round-trip, so the host never queues >1 frame and host-TX
    packing simply can't be produced by this example (asserting it made the test
    flaky). host->CP bulk packing needs an async flood that genuinely queues —
    see test_sdio_sw_aggr_host_tx_flood (raw_tp TX)."""
    b = bench('peer_data_transfer', 'mcu_host', 'sdio', timeout='180s',
              cp_overlay=AGGR_TRACE_CP_OVL, overlay=AGGR_TRACE_HOST_OVL)
    host, cp = b['host'], b['cp']

    r = eh_test_expect(host, r'SDIO SW_AGGR negotiated', fail=FAIL, timeout=45)
    assert r.ok, f'negotiated: {r.matched}'

    # CP->host multi-frame markers (one-time INFO logs) — reliably produced.
    r = eh_test_expect(cp, r'sdio aggr tx: packed \d+ frames', fail=FATAL_PATTERNS, timeout=120)
    assert r.ok, f'CP TX really aggregated (CP->host >1/transfer): {r.matched}'
    r = eh_test_expect(host, r'sdio aggr rx: \d+ sub-frames', fail=FAIL, timeout=120)
    assert r.ok, f'host RX really de-aggregated (CP->host >1/transfer): {r.matched}'

    # and the peer-data run itself must pass over the aggregating wire
    r = eh_test_expect(host, r'Test Summary', fail=FAIL, timeout=150)
    assert r.ok, f'peer_data_transfer completed over SW_AGGR: {r.matched}'
