"""CP shut-down / cold-boot cycle soak — power_save/cp/shut_down_cp_when_unused.

The host holds the CP in reset when networking is idle, then cold-boots it: each
cycle does eh_host_deinit -> drive EN low -> EN high -> eh_host_init +
connect_to_slave -> re-init Wi-Fi -> reconnect. This is the use-case gate:

  * deinit/init must fully clean up so a FRESH CP is brought back each cycle
    (every cycle must re-reach GOT_IP), and
  * host memory must not grow across cycles (no leak / UAF / IMR / IMW).

Memory correctness is asserted on the example's byte-exact `[Mem after cycle N]`
free-heap at a fixed point each cycle: steady-state cycles must stay flat. The
build runs with comprehensive heap poisoning, so a use-after-free / overflow trips
`CORRUPT HEAP` (in the fail set) rather than passing silently.
"""
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

CYCLES = 10
STEADY_FROM = 5           # ignore warm-up cycles; assert flatness from here
LEAK_TOL_BYTES = 512      # steady-state drift budget (measured drift is 0; real leak was ~148 B/cy)

FAIL = FATAL_PATTERNS + ['CORRUPT HEAP', 'Firmware abort', 'abort()']


@pytest.mark.power_save
@pytest.mark.parametrize('transport', [
    pytest.param('sdio', marks=pytest.mark.sanity),  # sanity rep on the reliable wire
    'uart', 'spi_hd',                                 # regression: soak every bus's teardown/re-init
    # spi_fd passes standalone (3/3 isolated) but is non-blocking under the full
    # --jobs 16 sweep: 16 tests × (host+CP) = 32 emu on 32 cores starves the CP so a
    # reconnect RPC exceeds the host firmware's FIXED 5000ms timeout (ESP_ERROR_CHECK
    # → abort). EH_EXPECT_SCALE widens test waits but can't touch that in-firmware
    # timeout; @second_chance can't help since its retry runs on the still-loaded box.
    # Emu-under-load artifact, not a product bug — kept visible (red ⊗) until the emu
    # is fast enough under full parallelism (or the soak is serialized off the sweep).
    pytest.param('spi_fd', marks=pytest.mark.allow_fail(
        reason="emu CPU-starved under --jobs 16 exceeds firmware 5s RPC timeout; passes isolated")),
])
@pytest.mark.second_chance
def test_cp_shutdown_cold_boot_cycles(bench, transport):
    # Poisoning on so a stray UAF/IMR/IMW during teardown/re-init aborts (CORRUPT HEAP).
    b = bench("power_save/cp/shut_down_cp_when_unused", "mcu_host", transport,
              timeout="300s", overlay=["CONFIG_HEAP_POISONING_COMPREHENSIVE=y"])
    host, cp = b["host"], b["cp"]

    # Initial cold boot + first association.
    assert eh_test_expect(cp, r"shut_down_cp coprocessor ready", fail=FAIL, timeout=90).ok, "CP first boot"
    assert eh_test_expect(host, r"Got IP address", fail=FAIL, timeout=120).ok, "initial GOT_IP"

    free_by_cycle = {}
    for n in range(1, CYCLES + 1):
        assert eh_test_expect(host, rf"CYCLE {n}:", fail=FAIL, timeout=120).ok, f"cycle {n} start"

        # Byte-exact free heap at the fixed post-teardown point (printed for n>1).
        # The wrapper only exposes `.before`, so bracket the number: match up to
        # "Free: ", then match ", Min-Ever" — the digits land in `.before` between.
        if n >= 2:
            assert eh_test_expect(host, rf"Mem after cycle {n}\] Free: ", fail=FAIL, timeout=30).ok, \
                f"cycle {n} heap checkpoint"
            if eh_test_expect(host, r", Min-Ever", fail=FAIL, timeout=5).ok:
                between = (host.pexpect_proc.before or b"").decode("utf-8", "replace")
                m = re.search(r"(\d+)", between)
                if m:
                    free_by_cycle[n] = int(m.group(1))

        # The CP must actually have been held in reset and cold-booted (not a no-op).
        assert eh_test_expect(host, r"Slave is down", fail=FAIL, timeout=60).ok, f"cycle {n} CP held in reset"
        assert eh_test_expect(cp, r"shut_down_cp coprocessor ready", fail=FAIL, timeout=90).ok, f"cycle {n} CP cold-boot"

        # Full recovery on the fresh CP: deinit/init restored everything -> IP again.
        assert eh_test_expect(host, r"Got IP address", fail=FAIL, timeout=120).ok, f"cycle {n} reconnect"

    # Memory correctness: steady-state cycles must not lose heap (no per-cycle leak).
    steady = {n: v for n, v in free_by_cycle.items() if n >= STEADY_FROM}
    assert len(steady) >= 3, f"too few heap samples: {free_by_cycle}"
    first, last = steady[min(steady)], steady[max(steady)]
    drift = first - last
    assert drift <= LEAK_TOL_BYTES, (
        f"host heap leak across cold-boot cycles: dropped {drift} B from cycle "
        f"{min(steady)} to {max(steady)} (per-cycle samples: {steady})")
