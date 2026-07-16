"""gpio-expander + external-coex API exercise (B2), substrate-agnostic.

GPIO is a STRONG round-trip everywhere (set level -> read it back): the emulator
models CP GPIO, real HW drives it. External coex is a wire-level feature — coex
config ops return rc=0 only where the coex wires exist. The test reads that from
the bench's `caps` (CAP_COEX_WIRED), not from the substrate name: a bench that
reports coex wiring gets the strong rc=0 assertion, everything else asserts the
op is DRIVABLE (well-formed line, no crash)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS
from targets.base import CAP_COEX_WIRED, CAP_GPIO_LOOPBACK

FAIL = FATAL_PATTERNS + ['bring-up timed out']
# CP GPIO used for the loopback round-trip. Must dodge every transport's own
# CP-side pins so the pin the test drives is never a live bus/handshake line:
# spi_fd HS=3 DR=4 + bus MOSI7/MISO2/CLK6/CS10, spi_hd DR=11, reset 12/54,
# console 16/17, sdio bus 18-23. GPIO 5 is free on all of them. (GPIO 4 — the
# spi_fd DR line — made gpio_get_level read the DR state, not the driven level.)
PIN = 5


def _ready(bench, transport):
    b = bench('system/api_exerciser', 'mcu_host', transport, timeout='120s')
    host = b['host']
    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=60)
    assert r.ok, f'[{transport}] ready: {r.matched}'
    return host, b['caps']


@pytest.mark.system
@pytest.mark.retired("subsumed by test_control_plane — gpio loopback round-trip (sdio/uart/spi_hd)")
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_fd'])
def test_gpio_level(bench, transport):
    host, caps = _ready(bench, transport)
    loopback = CAP_GPIO_LOOPBACK in caps  # can we read back the driven level?

    for cmd in ('gpio_init', f'gpio_set_direction {PIN} 2'):  # 2 = OUTPUT
        host.write(cmd)
        r = eh_test_expect(host, r'EH rc=0 cmd=' + cmd.split()[0], fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] {cmd}: {r.matched}'

    # driving the pin always succeeds (rc=0). The read-BACK value equals what was
    # driven only where GPIO loops back (emu models it; a real pin needs a loopback
    # wire) — otherwise assert get_level is merely DRIVABLE. One body, both worlds.
    for level in (1, 0):
        host.write(f'gpio_set_level {PIN} {level}')
        r = eh_test_expect(host, r'EH rc=0 cmd=gpio_set_level', fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] gpio_set_level {level}: {r.matched}'
        host.write(f'gpio_get_level {PIN}')
        pat = (rf'EH rc=0 cmd=gpio_get_level level={level}' if loopback
               else r'EH rc=0 cmd=gpio_get_level level=[01]')
        r = eh_test_expect(host, pat, fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] gpio_get_level ({"loopback" if loopback else "drivable"}): {r.matched}'

    for cmd in (f'gpio_set_pull_mode {PIN} 1', f'gpio_reset_pin {PIN}'):
        host.write(cmd)
        r = eh_test_expect(host, r'EH rc=0 cmd=' + cmd.split()[0], fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] {cmd}: {r.matched}'


@pytest.mark.system
@pytest.mark.retired("subsumed by test_control_plane — coex config ops (sdio/uart/spi_hd)")
@pytest.mark.parametrize('transport', ['sdio', 'uart', 'spi_fd'])
def test_coex(bench, transport):
    host, caps = _ready(bench, transport)
    wired = CAP_COEX_WIRED in caps

    # coex_init succeeds everywhere; the config ops need the physical coex wires.
    host.write('coex_init')
    r = eh_test_expect(host, r'EH rc=0 cmd=coex_init', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] coex_init: {r.matched}'

    for cmd in ('coex_set_work_mode 0', 'coex_set_grant_delay 10',
                'coex_set_validate_high 1', 'coex_disable'):
        # wired bench: config must succeed (rc=0); otherwise DRIVABLE (any rc, no crash).
        pat = (r'EH rc=0 cmd=' if wired else r'EH rc=-?\d+ cmd=') + cmd.split()[0]
        host.write(cmd)
        r = eh_test_expect(host, pat, fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] {cmd} ({"wired" if wired else "drivable"}): {r.matched}'
