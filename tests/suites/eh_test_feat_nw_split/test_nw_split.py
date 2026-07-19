"""Network-split (802.3) — connect-ownership matrix on the emulator.

The coprocessor owns the Wi-Fi radio; host and CP each run their own LWIP stack
sharing ONE IP with the port space split (host range -> host stack, CP-local
range + unmatched -> CP stack).

But *who triggers the connect* is a separate axis with several valid owners.
The earlier suite tested only the "host is always the controller" shape and did
NOT pin the config, so multiple owners connected at once -> assoc churn
(StaDisconnected reason=8 ASSOC_LEAVE, repeated got-ip). The old "SPI assoc
~100s" budget was almost certainly compensating for that churn. See
.meta2/changes/nw-split-connect-ownership/RCA.md.

Each test here pins EXACTLY ONE connect owner via overlay, is named for that
ownership, asserts the IP-arrival timing that ownership implies, and then proves
the link with a REAL ping (>=1 ICMP reply) — not just a `got ip:` log line.

Owners:
  CP feature auto   — CP connects on STA_START; host just receives the shared IP.
  host CLI          — host console `sta_connect`; nobody auto-connects.
  host feature auto — host nw_split feature connects on STA_START (creds at build).
  host app example  — the station example's own connect handler drives it.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out', 'connect to the AP fail']

IPF = 'network_split/iperf'      # host idle by default (CLI / feature driven)
STA = 'network_split/station'    # host app is itself the connect owner
IP = r'\d+\.\d+\.\d+\.\d+'
GW = '192.168.4.1'               # emu --net user gateway == CP SoftAP gw
ECHO_PORT = 49999                # host range (49152-61439) -> host stack
CP_PORT = 61500                  # CP-local range (61440-65535) -> coprocessor stack

# Connect-owner knobs — pin exactly one esp_wifi_connect() driver per test.
CP_AUTO_ON = 'CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START=y'
CP_AUTO_OFF = 'CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START=n'
HOST_AUTO_ON = 'CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_CONNECT_ON_START=y'
HOST_AUTO_OFF = 'CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_CONNECT_ON_START=n'

# Timeouts: assoc is seconds on a single-owner link (the old 120-240s budgets
# compensated for multi-owner churn — see RCA §7). Kept tight so a real hang
# fails fast instead of hiding; EH_EXPECT_SCALE widens them on emu under load.
T_CONSOLE, T_IP, T_NOIP, T_PING = 30, 45, 8, 20

# Known host-side emu flake, orthogonal to connect-ownership: the single-shot TCP
# echo probe runs through the P4 host's user-mode net (smoltcp) and its
# connect/echo intermittently drops (~20-40% even idle, jobs=1). Host-side; the
# split-routing LOGIC is transport-agnostic and fully proven on sdio/uart/spi_hd.
_SPI_ECHO_FLAKE = ("emu user-net single-shot TCP echo flake (~20-40% even idle) — "
    "host-side, orthogonal to connect-ownership. allow_fail (non-blocking) + "
    "second_chance (isolated retry usually recovers it).")


def _creds(ap):
    """Build-time STA creds overlay for whichever side owns the connect."""
    return [f'CONFIG_EH_EXAMPLE_WIFI_SSID="{ap["ssid"]}"',
            f'CONFIG_EH_EXAMPLE_WIFI_PASSWORD="{ap["password"]}"']


def _assert_ping(host, transport, gw=GW):
    """Real-traffic proof: ICMP to the gw, assert >=1 reply — the link carries
    data, not just a got-ip log. ~6 pings over 3 s."""
    host.write(f'ping {gw} -c 6 -i 0.5')
    r = eh_test_expect(host, r'\d+ packets transmitted, [1-9]\d* received',
                       fail=FAIL, timeout=T_PING)
    assert r.ok, f'[{transport}] ping {gw}: no ICMP reply (link not real): {r.matched}'


# ── Owner 1: CP feature auto-connect — host just RECEIVES the shared IP ──────
@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),
    'uart', 'spi_hd', 'spi_fd'])
@pytest.mark.second_chance
def test_nwsplit_cp_autoconnect_host_receives_ip(bench, wifi_ap, transport):
    """CP owns Wi-Fi (auto-connect on STA_START); the host does NOTHING and still
    obtains the split's shared IP. First-class scenario the earlier skip hid:
    the host stays idle yet gets an IP because the CP connected."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    b = bench(IPF, 'mcu_host', transport, timeout='150s', wake=True,
              overlay=[HOST_AUTO_OFF],
              cp_overlay=[CP_AUTO_ON, *_creds(wifi_ap)])
    host = b['host']

    # Gate on the console prompt (always first), NOT 'awaiting sta_connect': when
    # the CP owns the connect it can deliver the IP before the host finishes
    # wifi_init_sta, so got-ip may precede 'awaiting' (order not guaranteed).
    r = eh_test_expect(host, r'network-split-iperf>', fail=FAIL, timeout=T_CONSOLE)
    assert r.ok, f'[{transport}] console ready: {r.matched}'
    # No host action at all (idle example, host auto OFF, no CLI) -> IP still
    # arrives, delivered by the CP's connect over the split.
    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=T_IP)
    assert r.ok, f'[{transport}] host received CP-owned IP (no host action): {r.matched}'
    _assert_ping(host, transport)


# ── Owner 2: host CLI — nobody auto-connects; console drives it ──────────────
@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),
    'uart', 'spi_hd', 'spi_fd'])
@pytest.mark.second_chance
def test_nwsplit_host_cli_connect(bench, wifi_ap, transport):
    """Host CLI owns the connect: CP auto OFF, host feature auto OFF, so NO IP
    until the console `sta_connect`. The negative ("no IP before connect") now
    holds on emu because the CP no longer auto-connects — that was the real bug
    the earlier tap_available skip masked."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    b = bench(IPF, 'mcu_host', transport, timeout='150s', wake=True,
              overlay=[HOST_AUTO_OFF], cp_overlay=[CP_AUTO_OFF])
    host = b['host']

    r = eh_test_expect(host, r'awaiting sta_connect', fail=FAIL, timeout=T_CONSOLE)
    assert r.ok, f'[{transport}] wifi idle at boot: {r.matched}'
    # Bounded negative: no owner is connecting, so no IP may appear yet.
    r = eh_test_expect(host, rf'got ip:{IP}', fail=[], timeout=T_NOIP)
    assert not r.ok, f'[{transport}] must NOT get IP before CLI connect (saw {r.matched})'
    host.write(f'sta_connect {wifi_ap["ssid"]} {wifi_ap["password"]}')
    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=T_IP)
    assert r.ok, f'[{transport}] CLI sta_connect -> host got IP: {r.matched}'
    _assert_ping(host, transport)


# ── Owner 3: host nw_split feature auto-connect (no CLI) ─────────────────────
@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),
    'uart', 'spi_hd', 'spi_fd'])
@pytest.mark.second_chance
def test_nwsplit_host_feature_autoconnect(bench, wifi_ap, transport):
    """Host feature owns the connect: CP auto OFF, host feature auto ON with creds
    provisioned at build. The host obtains the IP on STA_START with NO CLI — this
    exercises eh_host_feat_nw_split:on_sta_start unambiguously (CP not racing)."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    b = bench(IPF, 'mcu_host', transport, timeout='150s', wake=True,
              overlay=[HOST_AUTO_ON, *_creds(wifi_ap)],
              cp_overlay=[CP_AUTO_OFF])
    host = b['host']

    r = eh_test_expect(host, r'network-split-iperf>', fail=FAIL, timeout=T_CONSOLE)
    assert r.ok, f'[{transport}] console ready: {r.matched}'
    # Host feature auto-connects on STA_START — IP with NO CLI command.
    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=T_IP)
    assert r.ok, f'[{transport}] host-feature auto-connect -> IP (no CLI): {r.matched}'
    _assert_ping(host, transport)


# ── Owner 4: host application example drives the connect (station example) ───
@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [pytest.param('sdio', marks=pytest.mark.sanity)])
@pytest.mark.second_chance
def test_nwsplit_host_station_example_connect(bench, wifi_ap, transport):
    """Host application owns the connect: the station example's own event handler
    connects on STA_START and retries on disconnect. CP auto OFF so the app is the
    sole owner (no churn). Station is console-less, so host `got ip:` is the
    functional proof; the ping-level proof is carried by the iperf owners above."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    b = bench(STA, 'mcu_host', transport, timeout='150s', wake=True,
              overlay=[HOST_AUTO_OFF, *_creds(wifi_ap)],
              cp_overlay=[CP_AUTO_OFF])
    host = b['host']

    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=T_IP)
    assert r.ok, f'[{transport}] station app connect -> host got IP: {r.matched}'


# ── Port-class routing: host-range -> host stack, CP-local -> CP stack ───────
@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),
    'uart', 'spi_hd', 'spi_fd'])  # spi_fd was allow_fail: fixed by esp-emu gpspi_slave
    # HS/DR wall-time re-broadcast gate; blanket @second_chance below covers contention.
@pytest.mark.second_chance
@pytest.mark.xdist_group("emu_heavy")  # serialize heavy spi_fd echo under parallel load
def test_nwsplit_port_routing_host_vs_cp(bench, wifi_ap, transport):
    """Single owner (host CLI), then prove the 802.3 split routes by port class:
    a host-range port reaches the HOST echo server; a CP-local port does not.
    Setting CP_AUTO_OFF makes the host CLI the sole owner — this removes the
    reason=8 churn that failed port_routing[spi_hd] in run 20260718-074553."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    b = bench(IPF, 'mcu_host', transport, timeout='150s', wake=True,
              overlay=[HOST_AUTO_OFF, 'CONFIG_EXAMPLE_NW_SPLIT_ECHO_SERVER=y',
                       f'CONFIG_EXAMPLE_NW_SPLIT_ECHO_PORT={ECHO_PORT}'],
              cp_overlay=[CP_AUTO_OFF],
              hostfwd=(ECHO_PORT, CP_PORT))
    host, net = b['host'], b['net']

    r = eh_test_expect(host, r'network-split-iperf>', fail=FAIL, timeout=T_CONSOLE)
    assert r.ok, f'[{transport}] console ready: {r.matched}'
    r = eh_test_expect(host, r'awaiting sta_connect', fail=FAIL, timeout=T_CONSOLE)
    assert r.ok, f'[{transport}] wifi ready: {r.matched}'
    host.write(f'sta_connect {wifi_ap["ssid"]} {wifi_ap["password"]}')
    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=T_IP)
    assert r.ok, f'[{transport}] host got IP: {r.matched}'
    r = eh_test_expect(host, rf'echo server listening on host port {ECHO_PORT}',
                       fail=FAIL, timeout=T_CONSOLE)
    assert r.ok, f'[{transport}] host echo server up: {r.matched}'

    payload = b'nw-split-routing-probe'
    resp = net.request(ECHO_PORT, payload)
    assert payload in resp, f'[{transport}] host-range :{ECHO_PORT} echo — got {resp!r}'
    resp = net.request(CP_PORT, payload)
    assert resp == b'', f'[{transport}] CP-local :{CP_PORT} must not be host-served — got {resp!r}'
