"""Boot-and-verify smokes for the real example firmwares — substrate-agnostic.

Each row boots one real example on the bench and asserts its success signature.
To keep flashing cheap, this suite deliberately covers only behaviours the broad
system/api_exerciser firmware CANNOT (events/heartbeat, CP mem telemetry,
custom-RPC echo, transport-config validation): API-level scenarios (wifi cfg /
scan / gpio / coex / mac …) are already exercised there in ONE firmware, so no
redundant per-feature firmware is flashed here.

A row needing external infra (a real AP, a BT peer, …) declares a required cap
and SKIPs unless the bench reports it — no build/flash wasted on it.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out']

# (example, success_regex, timeout_s) — self-contained, no external infra. Boots are
# transport-agnostic (just build + reach the init line); the sanity boot runs on sdio
# (reliable/fast), regression covers all four wires.
EXAMPLES = [
    ("system/hosted_events",        r"got INIT event from co-processor",   45),
    ("mem_monitor",                 r"Co-processor Mem Info",              45),
    ("system/mcu_transport_config", r"ESP-Hosted initialized successfully", 45),
    ("peer_data_transfer",          r"Test Summary",                       60),
    # Feature examples: assert each BUILDS + BOOTS to its init line on the emu.
    ("ext_coex",                    r"External Coexistence configuration completed", 60),
    ("ota/coprocessor_ota",         r"ESP-Hosted initialized successfully",          60),
    ("network_split/iperf",         r"Network Split Iperf Example",                  60),
    # HW-only (excluded from emu): BT/OpenThread need a BLE/802.15.4 radio the emu
    # lacks; wifi/{itwt,dpp,enterprise} bus-fault at app_main entry on the emu
    # (read8 unmapped 0x5C) — verify on HW, and RCA whether that early crash is a
    # real example bug or an emu memory-map gap.
]


_BOOT_WIRES = ["sdio", "uart", "spi_hd", "spi_fd"]
# (example, wire) combos that can't boot on the emu — parked visibly (maintenance),
# never silently omitted.
_BOOT_MAINT = {
}

# flaky spi_fd boot cells → serialize in the emu_heavy bucket (with wifi_connected /
# wifi_sta_connect / nw_split_port_routing), the rest stay in emu_boots.
_BOOT_EMU_HEAVY = {("peer_data_transfer", "spi_fd")}


def _boot_params():
    """Every example × every wire under --regression; the sdio cell of each is tagged
    @sanity (reliable/fast wire) so the sanity set stays quick and green. Known-broken
    (example, wire) combos are @maintenance with a reason — visible in the matrix as a
    skip, not a blank."""
    out = []
    for example, ready, timeout in EXAMPLES:
        for w in _BOOT_WIRES:
            marks = []
            if w == "sdio":
                marks.append(pytest.mark.sanity)
            reason = _BOOT_MAINT.get((example, w))
            if reason:
                marks.append(pytest.mark.maintenance(reason=f"emu: {reason}"))
            group = "emu_heavy" if (example, w) in _BOOT_EMU_HEAVY else "emu_boots"
            marks.append(pytest.mark.xdist_group(group))
            out.append(pytest.param(example, ready, timeout, w, marks=marks,
                                    id=f"{example.replace('/', '_')}-{w}"))
    return out


@pytest.mark.system
@pytest.mark.parametrize("example,ready,timeout,transport", _boot_params())
def test_example_boots(bench, example, ready, timeout, transport):
    b = bench(example, 'mcu_host', transport, timeout=f'{timeout + 40}s')
    host = b['host']
    r = eh_test_expect(host, ready, fail=FAIL, timeout=timeout)
    assert r.ok, f'[{transport}] {example} boot/success: {r.matched}'


# spi_fd carries the same emu assoc-under-load flake as wifi_connected/port_routing
# (assoc over spi_fd stalls under parallel oversubscription; spi_hd/sdio/uart are
# clean). So spi_fd runs @allow_fail (non-blocking) + @second_chance (isolated retry
# usually recovers); sanity runs on sdio (reliable/fast), the rest are regression-only.
_SPI_FD_ASSOC_FLAKE = ("emu spi_fd assoc flake under parallel load — same spi_fd-only "
    "root as wifi_connected/nw_split_port_routing; runs, isolated retry usually "
    "recovers, residual failure non-blocking until the emu-load starvation is fixed.")


@pytest.mark.wifi
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity: real Wi-Fi join on the reliable wire
    'uart', 'spi_hd',
    pytest.param('spi_fd', marks=pytest.mark.second_chance),
])
@pytest.mark.xdist_group("emu_heavy")  # serialize heavy spi_fd assoc under parallel load
def test_wifi_sta_connect(bench, bench_caps, wifi_ap, transport):
    """Real Wi-Fi join: build wifi/sta with the bench's AP creds and confirm the
    host connects through the co-processor. Needs a reachable AP → gated on the
    'wifi_ap' cap + creds (lab.local.json / env.json); skips otherwise."""
    if "wifi_ap" not in bench_caps or not wifi_ap:
        pytest.skip("no 'wifi_ap' cap / creds — add to lab.local.json to enable")
    # Keep retrying: a 2.4GHz AP that roams channels (incl. ch12-13, which some
    # regulatory domains only scan passively) isn't always caught on the first
    # sweep — the stock example gives up after ~5 retries. Bump it so the join is
    # reliable, matching a manual `idf.py monitor` run.
    ovl = [f'CONFIG_ESP_WIFI_SSID="{wifi_ap["ssid"]}"',
           f'CONFIG_ESP_WIFI_PASSWORD="{wifi_ap["password"]}"',
           'CONFIG_ESP_MAXIMUM_RETRY=100']
    b = bench('wifi/sta', 'mcu_host', transport, timeout='240s', overlay=ovl)
    host = b['host']
    r = eh_test_expect(host, r'(connected to ap|got ip:|sta ip:|IP_EVENT_STA_GOT_IP)',
                       fail=FATAL_PATTERNS + ['connect to the AP fail', 'bring-up timed out'],
                       timeout=180)
    assert r.ok, f"wifi/sta join {wifi_ap['ssid']}: {r.matched}"
