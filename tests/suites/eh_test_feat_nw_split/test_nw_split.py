"""Network-split (802.3) functional routing — emulator.

The coprocessor owns Wi-Fi; host and CP each run their own LWIP stack sharing ONE
IP with the port space split (host-reserved + host range → host stack, CP-local
range + unmatched → CP stack; ICMP echo-req → CP, reply → BOTH).

Core positive (this file): the 802.3 split delivers the DHCP-assigned address to
BOTH stacks — the CP (the DHCP client) logs `sta ip:` and the host logs `got ip:`.
Uses the minimal `network_split/station` example as reference (pure auto-connect).

Per-port-class server/client routing (IP layer) and ping (ICMP layer) are the HW
suite's job (tests/hw/eh_test_feat_nw_split) — on emu there is no app server to
answer a client and the per-packet verdict is logged only at VERBOSE.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out', 'connect to the AP fail']
EX = 'network_split/station'
IP = r'\d+\.\d+\.\d+\.\d+'


@pytest.mark.nw_split
@pytest.mark.wifi
# STA assoc over SPI is ~100s on the emu (vs ~6s sdio/uart) — hence the 180s budget;
# second_chance re-runs isolated if parallel load starves it.
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_hd', 'spi_fd'])
@pytest.mark.second_chance
def test_nw_split_host_ip_via_cp(bench, wifi_ap, transport):
    """Positive: the coprocessor owns Wi-Fi (associates to the AP) and the 802.3
    split delivers an IP to the HOST stack — which has no radio of its own. Proving
    the host obtains a DHCP address purely through the CP-owned link is the core
    network-split positive. (The CP netif shares that same IP by construction; the
    HW suite dumps the CP-side status via its CLI — the emu CP example exposes none.)"""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    ovl = [f'CONFIG_EXAMPLE_WIFI_SSID="{wifi_ap["ssid"]}"',
           f'CONFIG_EXAMPLE_WIFI_PASSWORD="{wifi_ap["password"]}"',
           'CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY=100']
    b = bench(EX, 'mcu_host', transport, timeout='240s', wake=True, overlay=ovl)
    host, cp = b['host'], b['cp']

    # CP owns the radio and associates. SPI assoc is slow AND variable on the emu
    # (~100-120s+ under load), so give it the generous budget that keeps assoc
    # reliable there instead of flaking at a ~120s boundary.
    rc = eh_test_expect(cp, rf'connected with {wifi_ap["ssid"]}', fail=FAIL,
                        timeout=180 if transport.startswith('spi') else 120)
    assert rc.ok, f'[{transport}] CP associate: {rc.matched}'
    # The split hands the host stack a DHCP address over that CP-owned link.
    rh = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=60)
    assert rh.ok, f'[{transport}] host got IP via split: {rh.matched}'


ECHO_PORT = 49999   # host range (49152-61439) → routes to the host stack
CP_PORT = 61500     # CP-local range (61440-65535) → routes to the coprocessor stack


# NOT the wifi_connected[spi] clock-starvation issue (that emu bug is fixed — see
# esp-emulator handle_wfi_bridge_slave; wifi_connected now runs on all four wires).
# The remaining blocker here is a SEPARATE, pre-existing flake in the single-shot
# TCP echo probe: it is a short-lived connection through the P4 host's user-mode
# net (smoltcp) — the host firmware comes up fully (got IP + "echo server
# listening") but the probe's connect/echo intermittently drops (~20-40% even
# idle, jobs=1). That path is on the host side and untouched by the CP-slave WFI
# fix; tightening/loosening the slave idle-pace does not change the rate. Port-
# split LOGIC is fully covered on sdio/uart (transport-agnostic classifier); basic
# assoc/DHCP over SPI is proven by wifi_sta_connect + nw_split_host_ip (all 4 wires).
_SPI_ECHO_FLAKE = ("emu user-net single-shot TCP echo flake (~20-40% even idle) — "
    "host-side, orthogonal to the (now-fixed) CP clock-starvation; see module note. "
    "allow_fail (non-blocking) + second_chance (isolated retry usually recovers it).")


@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: port-split proof
    'uart',
    'spi_hd',  # verified green 5/5 isolated — the flake is spi_fd-ONLY, not all-SPI
    # spi_fd echo flakes under parallel load: second_chance re-runs it isolated
    # (usually recovers → green ↻); allow_fail keeps a both-fail non-blocking (red ⊗).
    pytest.param('spi_fd', marks=[pytest.mark.allow_fail(reason=_SPI_ECHO_FLAKE),
                                  pytest.mark.second_chance]),
])
@pytest.mark.second_chance
@pytest.mark.xdist_group("emu_heavy")  # serialize: spi_fd echo flakes under parallel load
def test_nw_split_port_routing(bench, wifi_ap, transport):
    """IP-layer port split: with one shared IP, a client to a HOST-range port is
    answered by the host stack (a default-off echo server runs there), while a
    CP-local port is routed to the coprocessor (no server → refused). Same IP,
    different port class → different stack. Uses the iperf example (has a console).

    Positive: host-range port echoes.  Negative: CP-local port is not host-served."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    b = bench('network_split/iperf', 'mcu_host', transport, timeout='180s', wake=True,
              overlay=['CONFIG_EXAMPLE_NW_SPLIT_ECHO_SERVER=y',
                       f'CONFIG_EXAMPLE_NW_SPLIT_ECHO_PORT={ECHO_PORT}'],
              hostfwd=(ECHO_PORT, CP_PORT))
    host, net = b['host'], b['net']

    r = eh_test_expect(host, r'network-split-iperf>', fail=FAIL, timeout=90)
    assert r.ok, f'[{transport}] console ready: {r.matched}'
    # Wi-Fi comes up init-only (no boot auto-connect); wait until it's ready
    # before driving the CLI connect, else sta_connect races esp_wifi_start.
    r = eh_test_expect(host, r'awaiting sta_connect', fail=FAIL, timeout=60)
    assert r.ok, f'[{transport}] wifi ready: {r.matched}'
    host.write(f'sta_connect {wifi_ap["ssid"]} {wifi_ap["password"]}')
    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=90)
    assert r.ok, f'[{transport}] host got IP: {r.matched}'
    r = eh_test_expect(host, rf'echo server listening on host port {ECHO_PORT}',
                       fail=FAIL, timeout=30)
    assert r.ok, f'[{transport}] host echo server up: {r.matched}'

    # Positive: a host-range port reaches the HOST stack's echo server.
    payload = b'nw-split-routing-probe'
    resp = net.request(ECHO_PORT, payload)
    assert payload in resp, f'[{transport}] host-range :{ECHO_PORT} echo — got {resp!r}'
    # Negative: a CP-local port is routed to the coprocessor (no server there),
    # so the host echo never sees it — connection refused / no data.
    resp = net.request(CP_PORT, payload)
    assert resp == b'', f'[{transport}] CP-local :{CP_PORT} must not be host-served — got {resp!r}'


@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: console-connect proof
    'uart',                                          # the wire the boot-race crashed on
])
@pytest.mark.second_chance
def test_nw_split_iperf_cli_connect(bench, wifi_ap, transport, tap_available, substrate):
    """Console-connect model (iperf example): unlike the station example (pure
    auto-connect, covered by host_ip_via_cp), the iperf example is CLI-driven —
    it must NOT auto-connect at boot and must connect ONLY via `sta_connect`.

    Regression guard for the boot-auto-connect that raced the console command
    and crashed over uart. Asserts BOTH halves of the contract:
      1. Wi-Fi comes up idle — no connection before the CLI command is issued.
      2. `sta_connect` from the CLI brings the split's shared IP to the host."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    # Contract 1b below ("no IP before sta_connect") needs the emu to model a real
    # AP (association-gated DHCP), which requires TAP networking. Without CAP_NET_ADMIN
    # the emu falls back to --net user (smoltcp NAT) and leases the netif an IP at
    # boot, so the negative assertion cannot hold. The positive path is covered by
    # nw_split_port_routing / iperf_auto_connect.
    if substrate == 'emu' and not tap_available:
        pytest.skip(f'[{transport}] needs TAP-modeled AP (emu --net user NAT-leases at boot)')
    b = bench('network_split/iperf', 'mcu_host', transport, timeout='150s', wake=True)
    host = b['host']

    r = eh_test_expect(host, r'network-split-iperf>', fail=FAIL, timeout=90)
    assert r.ok, f'[{transport}] console ready: {r.matched}'
    # Contract 1a: Wi-Fi is up but idle (init-only, no auto-connect).
    r = eh_test_expect(host, r'awaiting sta_connect', fail=FAIL, timeout=60)
    assert r.ok, f'[{transport}] wifi idle at boot: {r.matched}'
    # Contract 1b: bounded negative — no IP is obtained before the CLI connect.
    r = eh_test_expect(host, rf'got ip:{IP}', fail=[], timeout=8)
    assert not r.ok, f'[{transport}] must NOT connect before sta_connect (saw {r.matched})'
    # Contract 2: the CLI connect brings the shared IP to the host stack.
    host.write(f'sta_connect {wifi_ap["ssid"]} {wifi_ap["password"]}')
    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=90)
    assert r.ok, f'[{transport}] sta_connect -> host got IP: {r.matched}'


@pytest.mark.nw_split
@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: framework auto-connect
    'uart',
])
@pytest.mark.second_chance
def test_nw_split_iperf_auto_connect(bench, wifi_ap, transport):
    """Framework (host) auto-connect path — closes the gap where nothing
    exercised eh_host_feat_nw_split:on_sta_start. With the Hosted Kconfig flag
    STA_AUTO_CONNECT_ON_START=y and credentials provisioned at build, the HOST
    feature auto-connects on WIFI_EVENT_STA_START with NO CLI command. CP-side
    auto-connect is disabled here so the connect is unambiguously host-driven
    (the CP path is separately covered by host_ip_via_cp / the station example).

    Verifies the network actually COMES UP (host obtains the split's shared IP),
    not merely that the node didn't crash."""
    if not wifi_ap:
        pytest.skip(f'[{transport}] bench has no associable AP')
    b = bench('network_split/iperf', 'mcu_host', transport, timeout='150s', wake=True,
              overlay=['CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_CONNECT_ON_START=y',
                       f'CONFIG_EXAMPLE_WIFI_SSID="{wifi_ap["ssid"]}"',
                       f'CONFIG_EXAMPLE_WIFI_PASSWORD="{wifi_ap["password"]}"'],
              cp_overlay=['CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START=n'])
    host = b['host']

    r = eh_test_expect(host, r'network-split-iperf>', fail=FAIL, timeout=90)
    assert r.ok, f'[{transport}] console ready: {r.matched}'
    # Host feature auto-connects on STA_START (creds provisioned, CP auto-connect
    # OFF) — the host obtains the shared IP with NO CLI sta_connect.
    r = eh_test_expect(host, rf'got ip:{IP}', fail=FAIL, timeout=90)
    assert r.ok, f'[{transport}] host-feature auto-connect -> host got IP (no CLI): {r.matched}'
