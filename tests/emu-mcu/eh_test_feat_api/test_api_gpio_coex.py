"""gpio-expander + external-coex API exercise (B2).

The emulator models CP GPIO, so gpio is a STRONG round-trip (set level -> read it
back). External coex is a wire-level feature: coex_init succeeds but the config
ops return ESP_FAIL on the emu (they need the physical coex wires / gpio-pin
setup), so coex is asserted as DRIVABLE only (well-formed line, no crash) — the
rc=0 assertion belongs on HW."""

import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # tests/
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

FAIL = FATAL_PATTERNS + ['bring-up timed out']
PIN = 4


def _ready(emu_bench, transport):
    b = emu_bench('system/api_exerciser', 'mcu_host', transport, timeout='120s')
    host = b['host']
    r = eh_test_expect(host, r'EH api_exerciser ready', fail=FAIL, timeout=60)
    assert r.ok, f'[{transport}] ready: {r.matched}'
    return host


@pytest.mark.system
@pytest.mark.parametrize('transport', ['sdio', 'uart'])
def test_gpio_level_roundtrip(emu_bench, transport):
    host = _ready(emu_bench, transport)

    for cmd in ('gpio_init', f'gpio_set_direction {PIN} 2'):  # 2 = OUTPUT
        host.write(cmd)
        r = eh_test_expect(host, r'EH rc=0 cmd=' + cmd.split()[0], fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] {cmd}: {r.matched}'

    # drive high -> read back high, drive low -> read back low
    for level in (1, 0):
        host.write(f'gpio_set_level {PIN} {level}')
        r = eh_test_expect(host, r'EH rc=0 cmd=gpio_set_level', fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] gpio_set_level {level}: {r.matched}'
        host.write(f'gpio_get_level {PIN}')
        r = eh_test_expect(host, rf'EH rc=0 cmd=gpio_get_level level={level}', fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] gpio_get_level=={level} round-trip: {r.matched}'

    for cmd in (f'gpio_set_pull_mode {PIN} 1', f'gpio_reset_pin {PIN}'):
        host.write(cmd)
        r = eh_test_expect(host, r'EH rc=0 cmd=' + cmd.split()[0], fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] {cmd}: {r.matched}'


@pytest.mark.system
@pytest.mark.parametrize('transport', ['sdio', 'uart'])
def test_coex_drivable(emu_bench, transport):
    host = _ready(emu_bench, transport)

    # coex_init succeeds; the config ops need wire setup absent on the emu, so we
    # assert only that each call is DRIVABLE (well-formed result, no crash).
    host.write('coex_init')
    r = eh_test_expect(host, r'EH rc=0 cmd=coex_init', fail=FAIL, timeout=20)
    assert r.ok, f'[{transport}] coex_init: {r.matched}'

    for cmd in ('coex_set_work_mode 0', 'coex_set_grant_delay 10',
                'coex_set_validate_high 1', 'coex_disable'):
        host.write(cmd)
        r = eh_test_expect(host, r'EH rc=-?\d+ cmd=' + cmd.split()[0], fail=FAIL, timeout=20)
        assert r.ok, f'[{transport}] {cmd} drivable: {r.matched}'
