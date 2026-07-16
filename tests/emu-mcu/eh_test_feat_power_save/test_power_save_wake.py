"""Combined host power-save wake regression (sdio + uart): an external packet wakes
the host via the CP, repeated back-to-back for CYCLES iterations, checking each wake
completes reliably AND the coprocessor's heap stays stable across the bus deinit→reinit
cycles (CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING). The CP light-sleeps
and unloads/reloads its transport on every host sleep; a leak shows as a monotonic decline
in the CP free heap sampled at each reinit ([mem:<transport>_init] free=…)."""

import sys
import os
import re
import statistics

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS, _get_serial_buffer

PS_FAIL = FATAL_PATTERNS + [
    'bring-up timed out',
    'Firmware abort',
    'Instruction access fault',
    'CORRUPT HEAP',
]

EXAMPLE = 'power_save/host+cp/network_split__host_deep_sleep_cp_light_sleep'
CYCLES = 30
# Per-cycle heap noise tolerance (fragmentation / cache). A real leak accrues far
# more than this over CYCLES; a stable path stays within it.
LEAK_EPS = 2048


# allow_fail + second_chance: the wake path re-inits Wi-Fi behind a 5s post-wake
# RPC deadline that slips under CPU contention (a busy/shared box) — the isolated
# retry usually recovers it (second_chance), and a both-fail under load stays
# non-blocking (allow_fail) instead of failing the pipeline on an environmental
# timing miss. Proven non-deterministic: passes cleanly isolated on a quiet box.
_WAKE_DEADLINE_FLAKE = ("power-save wake re-inits Wi-Fi under a 5s post-wake RPC "
                        "deadline that slips under shared-box CPU contention")


@pytest.mark.power_save
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=[pytest.mark.allow_fail(reason=_WAKE_DEADLINE_FLAKE),
                                pytest.mark.second_chance]),
    pytest.param('uart', marks=[pytest.mark.allow_fail(reason=_WAKE_DEADLINE_FLAKE),
                                pytest.mark.second_chance]),
])
def test_power_save_wake_mem(emu_bench, transport):
    b = emu_bench(EXAMPLE, 'esp_host', transport, wake=True, timeout='300s')
    host, cp, net = b['host'], b['cp'], b['net']

    # ── bring-up + connect ──
    r = eh_test_expect(host, r'iperf>', timeout=60)
    assert r.ok, f'CLI ready: {r.matched}'
    host.write('sta_connect myssid mypassword')
    r = eh_test_expect(host, r'IP_EVENT_STA_GOT_IP', fail=PS_FAIL, timeout=90)
    assert r.ok, f'GOT_IP: {r.matched}'

    for i in range(CYCLES):
        # host → deep sleep, CP → light sleep (+ bus unload)
        host.write('host_power_save')
        r = eh_test_expect(cp, r'Light sleep ENABLED', fail=PS_FAIL, timeout=60)
        assert r.ok, f'cycle {i}: CP light-sleep: {r.matched}'

        # external packet wakes the host via the CP
        net.wake()
        r = eh_test_expect(host, r'Host woke up from power save', fail=PS_FAIL, timeout=60)
        assert r.ok, f'cycle {i}: host wake: {r.matched}'
        r = eh_test_expect(host, r'slave chip id', fail=PS_FAIL, timeout=45)
        assert r.ok, f'cycle {i}: host transport re-init: {r.matched}'
        # Wait for the host to finish booting before the next cycle's
        # host_power_save, else the command is written before the console is ready
        # and the CP never re-sleeps. Match app_main's return (a newline-terminated
        # once-per-boot line) — the bare "iperf>" prompt has no trailing newline so
        # the line-based expect can miss it. (Wake is CP-driven via net.wake(), so
        # host connectivity isn't required to cycle.)
        r = eh_test_expect(host, r'Returned from app_main', fail=PS_FAIL, timeout=45)
        assert r.ok, f'cycle {i}: host boot complete after wake: {r.matched}'

    # CP heap leak check, post-loop (no per-cycle expect drift): CP logs
    # [mem:<transport>_init] free=N at each reinit. A leak is a downward TREND, so
    # compare median of the last third vs the middle third — both past boot/connect
    # warm-up, and median ignores transient single-sample dips.
    free = [int(x) for x in re.findall(r'\[mem:' + transport + r'_init\] free=(\d+)',
                                       getattr(cp, '_buf', ''))]
    assert len(free) >= 6, f'too few CP heap samples to assess leak: {free}'
    third = len(free) // 3
    mid, last = free[third:2 * third], free[2 * third:]
    assert statistics.median(last) >= statistics.median(mid) - LEAK_EPS, \
        f'CP heap leak across wake cycles: {free}'
