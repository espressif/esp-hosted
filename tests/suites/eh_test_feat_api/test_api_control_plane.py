"""Consolidated control-plane sweep for the native eh_host_* surface.

One emu/HW bench per transport drives the ENTIRE non-conflicting command corpus
(system + wifi scalars + struct-config + scan + gpio + coex) against the uniform
result contract `EH rc=<int> cmd=<name> [k=v ...]`. Folding these into a single
bench (was 8 separate tests) cuts ~8x bench bring-ups per transport and widens
coverage — every command is exercised on every transport in one shot.

Assertion strength:
  * scalars whose value is stable → get→set-different→get(matches)→revert(get) round-trip,
  * clamped/constrained scalars (tx_power) → set(ok)+get(well-formed),
  * scan / gpio / coex → sequenced, cap-aware (loopback / wired) per the bench caps.

Connect-dependent STA queries live in test_wifi_connected (separate bench: connect
mutates wifi state, so it cannot share the not-connected sweep's bench).
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS
from targets.base import CAP_COEX_WIRED, CAP_GPIO_LOOPBACK, CAP_WIFI_ITWT

FAIL = FATAL_PATTERNS + ['bring-up timed out']
EX = 'system/api_exerciser'
# CP GPIO for the loopback round-trip. Must dodge every transport's own CP-side
# pins so the driven pin is never a live bus/handshake/data line:
#   spi_fd  HS=3 DR=4  bus MOSI7 MISO2 CLK6 CS10
#   spi_hd  4-line D0=7 D1=2 D2=5 D3=4 CLK6 CS10 DR=11
#   sdio    bus 18-23 ;  uart console 16/17 ;  reset 12/54
# GPIO 14 is free on all of them. (GPIO 5 is spi_hd's D2 — would fight the SPI bus;
# GPIO 4 is the spi_fd DR line — read the DR state, not the driven level.)
GPIO_PIN = 14


def _ready(bench, t, wake=False):
    # wake=True asks the factory for a net-provisioned bench (emu: --net + the
    # `myssid` softap, caps gains `wifi_ap`) so a STA can actually associate.
    b = bench(EX, 'mcu_host', t, timeout='120s', wake=wake)
    r = eh_test_expect(b['host'], r'EH api_exerciser ready', fail=FAIL, timeout=60)
    assert r.ok, f'[{t}] ready: {r.matched}'
    return b['host'], b['caps']


def _ok(host, t, cmd, extra=''):
    """Assert the command returns rc=0 (+ optional trailing field pattern)."""
    host.write(cmd)
    pat = r'EH rc=0 cmd=' + cmd.split()[0] + ((' ' + extra) if extra else '')
    r = eh_test_expect(host, pat, fail=FAIL, timeout=20)
    assert r.ok, f'[{t}] {cmd}: {r.matched}'


def _drivable(host, t, cmd):
    """Assert the command produces a well-formed line (any rc, no crash)."""
    host.write(cmd)
    r = eh_test_expect(host, r'EH rc=-?\d+ cmd=' + cmd.split()[0], fail=FAIL, timeout=30)
    assert r.ok, f'[{t}] {cmd} drivable: {r.matched}'


def _get_is(host, t, get_cmd, field, val):
    """Assert a getter returns rc=0 with field=val."""
    base = get_cmd.split()[0]
    host.write(get_cmd)
    r = eh_test_expect(host, rf'EH rc=0 cmd={base}.*{field}={val}(\b|$)', fail=FAIL, timeout=20)
    assert r.ok, f'[{t}] {get_cmd} expected {field}={val}: {r.matched}'


def _roundtrip(host, t, get_cmd, field, set_fmt, v1, v2):
    """Prove the setter changes state and the getter reflects it, non-destructively:
    set v1 → get(v1) → set v2 → get(v2) → set v1 → get(v1) (revert to baseline v1)."""
    for v in (v1, v2, v1):
        _ok(host, t, set_fmt.format(v))
        _get_is(host, t, get_cmd, field, v)


# The full ~40-RPC sweep runs on all four wires. spi_fd used to wedge under this
# volume — an EMULATOR-ONLY deadlock (never firmware: HS/DR are physical GPIO lines
# on real HW): the esp-emu SPI bridge multiplexed async HS/DR GPIO frames + SPI
# responses on one unix socket, and under load the CP-emu's send buffer filled while
# the host-emu drained lazily → the handshake assert never reached the host. Fixed in
# esp-emu (spi_bridge: HS/DR now ride a dedicated second socket — <path>.gpio —
# separate from SPI data, so a handshake edge can never queue behind a bulk response;
# mirrors the real wiring where HS/DR are their own pins read off-bus).
@pytest.mark.system
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: full RPC sweep on the reliable wire
    'uart', 'spi_hd', 'spi_fd',
])
@pytest.mark.second_chance
def test_control_plane(bench, transport):
    host, caps = _ready(bench, transport)
    t = transport

    # ── system ──────────────────────────────────────────────────────────
    _ok(host, t, 'sys_fw_version', r'ver=\d+\.\d+\.\d+')
    _ok(host, t, 'sys_get_mac sta', r'mac=([0-9a-f]{2}:){5}[0-9a-f]{2}')
    _ok(host, t, 'sys_get_mac ap', r'mac=([0-9a-f]{2}:){5}[0-9a-f]{2}')
    # set_mac needs the iface stopped; on a started stack it may reject — assert
    # the RPC path is well-formed either way (getter above is the strict check).
    _drivable(host, t, 'sys_set_mac sta 02:00:00:11:22:33')
    _ok(host, t, 'sys_app_desc', r'proj=\S* ver=\d+\.\d+\.\d+')  # proj may be empty on the CP

    # ── wifi scalar round-trips (set v1→get→set v2→get→revert v1) ─────────
    _roundtrip(host, t, 'wifi_get_ps', 'ps', 'wifi_set_ps {}', 0, 2)
    _roundtrip(host, t, 'wifi_get_country', 'cc', 'wifi_set_country {} 0', 'US', 'JP')
    _roundtrip(host, t, 'wifi_get_inactive_time sta', 'sec', 'wifi_set_inactive_time sta {}', 5, 10)
    # setter + getter both exercised; the radio clamps/normalizes the value, so
    # assert the getter is well-formed rather than an exact echo of what was set.
    _drivable(host, t, 'wifi_set_channel 6');         _ok(host, t, 'wifi_get_channel', r'chan=\d+')
    # protocol/bandwidth setters are radio-reconfig HARD STOPS — moved to the end of
    # this test (see the tail) so their CP stall can't starve the RPCs below them.
    _ok(host, t, 'wifi_set_max_tx_power 40');         _ok(host, t, 'wifi_get_max_tx_power', r'power=-?\d+')
    _drivable(host, t, 'wifi_set_band 1');            _ok(host, t, 'wifi_get_band', r'band=\d+')
    _drivable(host, t, 'wifi_set_band_mode 1');       _ok(host, t, 'wifi_get_band_mode', r'band_mode=\d+')

    # negative: a missing required arg is rejected (rc!=0, err=USAGE), no crash
    host.write('wifi_set_ps')
    r = eh_test_expect(host, r'EH rc=-1 cmd=wifi_set_ps err=USAGE', fail=FAIL, timeout=20)
    assert r.ok, f'[{t}] wifi_set_ps usage guard: {r.matched}'

    # ── wifi struct-config round-trips ────────────────────────────────────
    # STA config (mode stays STA): staged SSID must survive set→get.
    _ok(host, t, 'wifi_set_mode 1'); _ok(host, t, 'wifi_get_mode', 'mode=1')
    for c in ('wifi_cfg_reset', 'wifi_cfg_set sta_ssid testnet',
              'wifi_cfg_set sta_channel 6', 'wifi_set_config sta'):
        _ok(host, t, c)
    _ok(host, t, 'wifi_get_config sta', 'ssid=testnet')
    # bssid-pinned STA config marshals.
    for c in ('wifi_cfg_reset', 'wifi_cfg_set sta_ssid pinned',
              'wifi_cfg_set sta_bssid aa:bb:cc:dd:ee:ff', 'wifi_set_config sta'):
        _ok(host, t, c)
    # AP config needs APSTA so the AP iface is live; SSID must survive set→get.
    _ok(host, t, 'wifi_set_mode 3'); _ok(host, t, 'wifi_get_mode', 'mode=3')
    for c in ('wifi_cfg_reset', 'wifi_cfg_set ap_ssid apnet',
              'wifi_cfg_set ap_channel 6', 'wifi_set_config ap'):
        _ok(host, t, c)
    _ok(host, t, 'wifi_get_config ap', 'ssid=apnet')
    # AP-side getters (APSTA live): well-formed with zero associated STAs on emu —
    # sta_list returns num=0; get_sta_aid for an absent STA rejects cleanly.
    # (wifi_deauth_sta is a hard stop — see the tail — so it isn't run mid-sweep.)
    _ok(host, t, 'wifi_ap_get_sta_list', r'num=\d+')
    _drivable(host, t, 'wifi_ap_get_sta_aid 24:0a:c4:00:00:09')
    _ok(host, t, 'wifi_set_mode 1')  # back to STA for a sane end state
    # NULL-arg guard: host-side returns non-zero + err field, never crashes.
    host.write('wifi_set_config_null')
    r = eh_test_expect(host, r'EH rc=-?[1-9]\d* cmd=wifi_set_config_null err=\w+', fail=FAIL, timeout=20)
    assert r.ok, f'[{t}] null-arg guard: {r.matched}'

    # ── scan (sequenced: clear → count → start → stop → dump) ─────────────
    _ok(host, t, 'wifi_clear_ap_list')
    _ok(host, t, 'wifi_scan_get_ap_num', r'num=\d+')
    _drivable(host, t, 'wifi_scan_start 1')   # emu scan support varies; HW scans for real
    _drivable(host, t, 'wifi_scan_stop')
    _drivable(host, t, 'wifi_scan_dump')

    # ── misc wifi (drivable: storage set, restore, idempotent disconnect) ─
    _ok(host, t, 'wifi_set_storage 0')
    _drivable(host, t, 'wifi_disconnect')

    # ── gpio expander (loopback round-trip where the bench models it) ─────
    loopback = CAP_GPIO_LOOPBACK in caps
    _ok(host, t, 'gpio_init')
    _ok(host, t, f'gpio_set_direction {GPIO_PIN} 2')  # OUTPUT
    for level in (1, 0):
        _ok(host, t, f'gpio_set_level {GPIO_PIN} {level}')
        pat = (rf'EH rc=0 cmd=gpio_get_level level={level}' if loopback
               else r'EH rc=0 cmd=gpio_get_level level=[01]')
        host.write(f'gpio_get_level {GPIO_PIN}')
        r = eh_test_expect(host, pat, fail=FAIL, timeout=20)
        assert r.ok, f'[{t}] gpio_get_level ({"loopback" if loopback else "drivable"}): {r.matched}'
    _ok(host, t, f'gpio_input_enable {GPIO_PIN}')
    _ok(host, t, f'gpio_set_pull_mode {GPIO_PIN} 1')
    _ok(host, t, f'gpio_reset_pin {GPIO_PIN}')

    # ── wifi iTWT (individual TWT; auto-enabled on HE-capable slaves) ──────
    # No AP/TWT peer on emu, so the setup family may reject — assert each RPC
    # round-trips cleanly (well-formed line, any rc, no crash).
    if CAP_WIFI_ITWT in caps:
        _drivable(host, t, 'wifi_itwt_setup')
        _drivable(host, t, 'wifi_itwt_suspend 0 100')
        _drivable(host, t, 'wifi_itwt_probe 500')
        _drivable(host, t, 'wifi_itwt_offset 0')
        _drivable(host, t, 'wifi_itwt_teardown 0')
        _drivable(host, t, 'wifi_itwt_flow_status')


    # ── external coex (config succeeds only where the coex wires exist) ───
    wired = CAP_COEX_WIRED in caps
    _ok(host, t, 'coex_init')
    for cmd in ('coex_set_work_mode 0', 'coex_set_grant_delay 10',
                'coex_set_validate_high 1', 'coex_disable'):
        if wired:
            _ok(host, t, cmd)
        else:
            _drivable(host, t, cmd)

    # ── radio-reconfig / AP HARD STOPS, run LAST ──────────────────────────
    # wifi_set_protocol / wifi_set_bandwidth kick off a phymode + iTWT reconfig,
    # and wifi_deauth_sta drives the AP deauth path — each blocks the CP's RPC
    # handler for ~40s under emulation (no real radio to complete the op). Placed
    # at the very end so that stall can never starve an upstream RPC; drive the
    # path only, tolerant of the emu rc.
    _drivable(host, t, 'wifi_set_protocol sta 0x0b')
    _drivable(host, t, 'wifi_set_bandwidth sta 1')
    _drivable(host, t, 'wifi_deauth_sta 1')


_SPI_ASSOC_FLAKE = ("emu-under-load: post-assoc wifi_sta_get_ap_info times out (rc=-1, "
    "'no response ... msg_id=294') ~1/3 on SPI under --jobs auto — the emulated C6, "
    "CPU-starved by oversubscription, stalls its RPC/app tasks after assoc while the "
    "SPI transport stays alive (HS-traced). The emu WFI wall-pacing fix (handle_wfi "
    "gated on gpspi_slave.has_bridge()) did NOT make it reliably pass — a clean 3x "
    "verification still failed 1/3 with the identical signature. NOT a transport/"
    "handshake/data-path drop. Assoc over SPI is proven on all 4 wires by "
    "wifi_sta_connect + nw_split_host_ip. allow_fail (non-blocking) + second_chance "
    "(isolated retry usually recovers it) until the emu-load starvation is actually "
    "resolved (or regression runs at non-oversubscribed concurrency).")


@pytest.mark.system
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: connect path on the reliable wire
    'uart',
    'spi_hd',
    'spi_fd',
])
@pytest.mark.second_chance
@pytest.mark.xdist_group("emu_heavy")  # serialize: spi_fd assoc flakes under parallel load
def test_wifi_connected(bench, transport):
    """Connect to the bench's AP, then assert the connect-only STA queries return
    live data. Sequence: stage sta config → set_config → connect → poll a query
    until associated → assert ap_info/rssi/aid → disconnect. Gated on the bench
    advertising an associable AP (emu virtual AP / HW creds); else skip."""
    host, caps = _ready(bench, transport, wake=True)
    t = transport
    # Cap-gate: only benches that advertise an associable AP can run this. Instant
    # skip (no wasted polling) when absent — e.g. a HW bench with no AP wired.
    if 'wifi_ap' not in caps:
        pytest.skip(f'[{t}] bench has no associable AP (no wifi_ap cap)')

    ssid = os.environ.get('EH_TEST_AP_SSID', 'myssid')
    pw = os.environ.get('EH_TEST_AP_PASS', 'mypassword')

    _ok(host, t, 'wifi_set_mode 1')
    for c in ('wifi_cfg_reset', f'wifi_cfg_set sta_ssid {ssid}',
              f'wifi_cfg_set sta_password {pw}', 'wifi_set_config sta'):
        _ok(host, t, c)
    _ok(host, t, 'wifi_connect')

    # Association is async — WAIT for the STA_CONNECTED event the host surfaces on
    # its console, rather than busy-polling wifi_sta_get_ap_info. Each poll is a
    # full RPC round-trip; over the slow spi_fd emu link under parallel
    # load a poll loop keeps both emulators spinning on the bus and starves the
    # co-scheduled jobs' CPU (and the very assoc it waits on). Blocking on the
    # event lets the host idle (emu WFI-skips) so other jobs get the core.
    ev_to = 100 if t.startswith('spi') else 30  # SPI assoc on the emu is slow/variable
    r = eh_test_expect(host, r'EH event wifi_sta_connected', fail=FAIL, timeout=ev_to)
    assert r.ok, f'[{t}] STA never signalled connected to {ssid}: {r.matched}'

    # Connected — one query returns live AP data. A couple of retries absorb any
    # event→query settle, but this is NOT the old 20x poll-through-assoc loop.
    associated = False
    for _ in range(3):
        host.write('wifi_sta_get_ap_info')
        if eh_test_expect(host, rf'EH rc=0 cmd=wifi_sta_get_ap_info ssid={ssid}',
                          fail=FAIL, timeout=5).ok:
            associated = True
            break
    assert associated, f'[{t}] ap_info did not report {ssid} after connect event'

    # The remaining connect-only queries exercise their RPC paths; the emu models
    # only some (rssi returns N/A there), so assert well-formed rather than rc=0.
    _drivable(host, t, 'wifi_sta_get_rssi')
    _drivable(host, t, 'wifi_sta_get_aid')
    _drivable(host, t, 'wifi_sta_get_negotiated_phymode')
    _ok(host, t, 'wifi_disconnect')
