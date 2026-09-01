"""Combined host power-save wake regression (sdio + uart): an external packet wakes
the host via the CP, repeated back-to-back for CYCLES iterations, checking each wake
completes reliably AND the coprocessor's heap stays stable across the bus deinit→reinit
cycles (CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING). The CP light-sleeps
and unloads/reloads its transport on every host sleep; a leak shows as a decline in the
CP free heap sampled once per cycle at the same quiescent point ([mem:ps_cycle] free=…),
plus a fall in min-ever-free, which fragmentation cannot produce."""

import sys
import os
import re
import statistics

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

PS_FAIL = FATAL_PATTERNS + [
    'bring-up timed out',
    'Firmware abort',
    'Instruction access fault',
    'CORRUPT HEAP',
]

EXAMPLE = 'power_save/host+cp/network_split__host_deep_sleep_cp_light_sleep'
# Raise for a soak run: EH_PS_WAKE_CYCLES=120 … --timeout=3000. A wake bug that
# shows up once in 30 needs more than 30 cycles to call fixed.
CYCLES = int(os.environ.get('EH_PS_WAKE_CYCLES', '30'))
# Per-cycle heap noise tolerance (fragmentation / cache). A real leak accrues far
# more than this over CYCLES; a stable path stays within it.
LEAK_EPS = 2048


# allow_fail is GONE from both cells, deliberately. It used to blame "shared-box CPU
# contention", which turned out to be wrong twice over: the late-cycle failures were
# the emulator being killed at its 300 s lifetime (raised below), and the uart ones
# were the CP light-sleeping in the middle of its own wake transaction — powering
# down the UART it had just re-initialised. Both are code/harness bugs, so a failure
# here has to block. second_chance stays: it re-runs isolated and takes that as the
# final outcome, which surfaces genuine load sensitivity without hiding a failure.
# Kept as a record, not a marker. baa530d09 documented a THIRD uart wake bug here
# and it deserves not to be lost just because its allow_fail went away:
#   post-wake wake-packet delivery called netif->input before the netif was
#   attached - instruction access fault, MEPC=0, RA in wlanif_input
#   (esp_netif/lwip/netif/wlanif.c: netif->input(p, netif)), i.e. a NULL function
#   pointer. The coprocessor holds the wake packet and writes it the instant the
#   host is up, so it could beat lwIP's input attach. It was recorded as
#   deterministic, 2/2 isolated.
# It no longer reproduces ON sdio/uart: 180 cycles show no such fault. The example
# now adopts the coprocessor's existing link instead of re-associating, so the
# netif is never torn down and its input stays attached - which plausibly removed
# it there. But it is NOT gone: spi_fd reproduced this exact signature
# (Instruction access fault, MEPC=0x00000000, RA in wlanif_input) on its first
# ever wake run, downstream of the CP reset described below. So the underlying
# NULL-input window is still real; sdio/uart merely stopped entering it.

# spi_fd and spi_hd were parked here as "host deep-sleep wake is NOT implemented
# on SPI". Both are unparked: each passes 6/6 wake cycles. Recorded because the
# park listed only two of the causes and the rest are worth not re-discovering:
#   - targets/emu.py wired no CP->host wake GPIO on the SPI branch at all, and the
#     pin is transport-dependent (CP out 2, but 9 on spi_hd; host in 6 / 4).
#   - eh_frame_decode() returned EH_FRAME_DUMMY for a zero-length frame BEFORE
#     populating flags. The host's power-save announcement is header-only, so any
#     wire reading flags after decode (both SPI ones) silently lost it; uart only
#     worked because it reads the raw header first.
#   - neither host SPI backend had the wakeup-reason resume path sdio/uart have,
#     so on wake the host reset the coprocessor and wiped its Wi-Fi/RPC state.
#   - spi_fd's slave-up event was given exactly once, at CP core init, and the
#     host's tx_ready gate is flipped by nothing else - so a rebooted host waited
#     out eh_host_core_bringup()'s 5000 ms and aborted in esp_wifi_init(). The
#     coprocessor now re-arms it on ESP_POWER_SAVE_OFF.
#   - spi_hd tears its slave down for the sleep, so the host must redo
#     wait_ready/DR-ISR/open_datapath and recreate its rx task, and the two
#     cumulative flow-control counters have to be zeroed on both sides at the
#     same instant or the host computes a negative transfer size.


@pytest.mark.power_save
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=[pytest.mark.second_chance]),
    pytest.param('uart', marks=[pytest.mark.second_chance]),
    pytest.param('spi_fd', marks=[pytest.mark.second_chance]),
    pytest.param('spi_hd', marks=[pytest.mark.second_chance]),
])
def test_power_save_wake_mem(emu_bench, transport):
    # Emulator lifetime, wall clock — derived from CYCLES so the soak knob above
    # stays coherent. Measured ~13 s per sleep/wake cycle; 20 s leaves margin.
    # This was a flat 300 s, under the ~390 s that 30 cycles actually need, so the
    # emulator was killed mid-run and the harness reported it as a wake timeout.
    # That is why failures always landed late (cycles 22-29) and moved with machine
    # load. pytest.ini's 900 s cap covers the default 30; raise --timeout to soak.
    # EH_CP_EXTRA_OVL lets a diagnostic run add CP sdkconfig lines (e.g. a trace
    # facility) without editing the checked-in defaults.
    cp_ovl = [x for x in os.environ.get('EH_CP_EXTRA_OVL', '').split(',') if x]
    b = emu_bench(EXAMPLE, 'esp_host', transport, wake=True,
                  timeout=f'{90 + CYCLES * 20}s', cp_extra_ovl=cp_ovl)
    host, cp, net = b['host'], b['cp'], b['net']

    # Cycle failures are COLLECTED, not asserted, so the heap check below always
    # runs. It used to sit after this loop behind four asserts, so one transient
    # cycle failure hid the leak entirely — measured: a -5636 byte trend went
    # unreported because cycle 24 timed out first.
    #
    # The two bring-up checks are collected for the same reason: they used to be
    # bare asserts, so a bring-up failure aborted the function BEFORE the serial
    # dump below, and the run left no dutlog at all - exactly when one is most
    # wanted. Collect, skip the cycles, still fail.
    failures = []

    # ── bring-up + connect ──
    r = eh_test_expect(host, r'host>', timeout=60)
    if not r.ok:
        failures.append(f'CLI ready: {r.matched}')
    else:
        # Control defaults to the console, so the example waits to be told.
        host.write('sta myssid mypassword')
        r = eh_test_expect(host, r'IP_EVENT_STA_GOT_IP', fail=PS_FAIL, timeout=90)
        if not r.ok:
            failures.append(f'GOT_IP: {r.matched}')

    for i in range(0 if failures else CYCLES):
        # host → deep sleep, CP → light sleep (+ bus unload)
        host.write('host_power_save')
        r = eh_test_expect(cp, r'Light sleep ENABLED', fail=PS_FAIL, timeout=60)
        if not r.ok:
            failures.append(f'cycle {i}: CP light-sleep: {r.matched}')
            break

        # external packet wakes the host via the CP
        net.wake()
        r = eh_test_expect(host, r'Host woke up from power save', fail=PS_FAIL, timeout=60)
        if not r.ok:
            failures.append(f'cycle {i}: host wake: {r.matched}')
            break
        r = eh_test_expect(host, r'slave chip id', fail=PS_FAIL, timeout=45)
        if not r.ok:
            failures.append(f'cycle {i}: host transport re-init: {r.matched}')
            break
        # Wait for the host to finish booting before the next cycle's
        # host_power_save, else the command is written before the console is ready
        # and the CP never re-sleeps. Match app_main's return (a newline-terminated
        # once-per-boot line) — the bare "host>" prompt has no trailing newline so
        # the line-based expect can miss it. (Wake is CP-driven via net.wake(), so
        # host connectivity isn't required to cycle.)
        r = eh_test_expect(host, r'Returned from app_main', fail=PS_FAIL, timeout=45)
        if not r.ok:
            failures.append(f'cycle {i}: host boot complete after wake: {r.matched}')
            break

    # CP heap. Sampled from [mem:ps_cycle], which the example logs once per cycle
    # in the host-power-save prepare callback — host idle, bus still up. That point
    # recurs identically every cycle, so a decline there is a real leak.
    #
    # This used to read [mem:<transport>_init] instead, sampled mid-bring-up. That
    # point carries whatever traffic buffers are transiently outstanding, so it
    # drifts by kilobytes on its own: a measured run showed that series falling
    # 100532 -> 93124 (~1 kB/cycle, "a leak") while free heap at the quiescent
    # point was flat within 40 B and the CP heap-trace live-record count did not
    # grow at all. The old check was measuring traffic, not a leak.
    samples = [(int(f), int(m)) for f, m in
               re.findall(r'\[mem:ps_cycle\] free=(\d+) min=(\d+)',
                          getattr(cp, '_buf', ''))]
    if len(samples) < 6:
        failures.append(f'too few CP heap samples to assess leak: {samples}')
    else:
        free = [f for f, _ in samples]
        third = len(free) // 3
        mid, last = free[third:2 * third], free[2 * third:]
        delta = int(statistics.median(last) - statistics.median(mid))
        # Reported, never asserted on. min-ever-free is a high-WATER-MARK, not a
        # leak signal: it only records the deepest transient dip ever seen, so it
        # ratchets down a step whenever a rarer, burstier moment comes along and
        # then sits flat. Measured over 30 cycles it fell in three steps (~cycle
        # 15/17/25) while free heap at this same point moved by 40 B total —
        # nothing retained, just a deeper peak. Asserting on it fails clean runs.
        min_drop = samples[third][1] - samples[-1][1]
        print(f'[heap] {transport} samples={len(free)} median delta={delta:+d} '
              f'min-ever drop={min_drop} (tolerance {LEAK_EPS}) series={free}')
        if delta < -LEAK_EPS:
            failures.append(f'CP heap leak across wake cycles: median delta '
                            f'{delta:+d} over {len(free)} samples: {free}')

    # Keep the serial streams on EVERY run, pass or fail. They used to be dropped
    # unless the test failed, so a rare intermittent wake bug burned a run each time
    # it did NOT reproduce - there was nothing left to compare a good cycle against.
    outdir = os.environ.get('EH_DUT_LOG_DIR',
                            os.path.join(os.path.dirname(__file__), '..', '..',
                                         '.work', 'dutlogs'))
    os.makedirs(outdir, exist_ok=True)
    stamp = f'{transport}-{os.getpid()}-{"FAIL" if failures else "PASS"}'
    for name, dut in (('cp', cp), ('host', host)):
        path = os.path.join(outdir, f'{stamp}-{name}.log')
        with open(path, 'w') as fh:
            fh.write(getattr(dut, '_buf', ''))
        print(f'[dut-log] {path}')

    if failures:
        for name, dut in (('cp', cp), ('host', host)):
            tail = getattr(dut, '_buf', '').splitlines()[-400:]
            print(f'\n───── {name} tail ({len(tail)} lines) ─────')
            for line in tail:
                print(f'  {line}')

    assert not failures, '; '.join(failures)
