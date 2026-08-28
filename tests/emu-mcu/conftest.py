"""
ESP-Hosted Emulator Test Infrastructure — conftest.py

Thin pytest adapters over the emu Target provider (`targets/emu.py`). They give
the SAME `dut` handle surface pytest-embedded gives HW tests, so an emu test
reads identically to its HW twin:

    host, cp = dut[0], dut[1]
    r = eh_test_expect(host, r'host>', timeout=40)

Convention (identical to HW): dut[0] = Host (ESP32-P4), dut[1] = CP/Slave (C6).
The launch logic lives in EmuTarget; these fixtures only choose a BenchSpec and
own the setup/teardown lifetime.
"""
import os
import sys

import pytest

# Add tests/ to sys.path so emu tests share infra/ + targets/ with HW tests.
_tests_dir = os.path.join(os.path.dirname(__file__), '..')
if _tests_dir not in sys.path:
    sys.path.insert(0, _tests_dir)

from targets.base import BenchSpec   # noqa: E402
from targets.emu import EmuTarget    # noqa: E402

_PS_EXAMPLE = 'power_save/host+cp/network_split__host_deep_sleep_cp_light_sleep'


@pytest.fixture(scope='session', autouse=True)
def _prebuild_ahead():
    """Speculative: warm the build cache concurrently before tests fan out, so a
    first run overlaps builds instead of building serially per test. Lock-guarded
    (safe under --jobs); best-effort (lazy build in the bench is the fallback)."""
    from infra import build_fw
    ex = 'system/get_cp_fw_version'
    build_fw.prebuild([
        (ex, 'cp', 'esp32c6', []),
        (ex, 'mcu_host', 'esp32p4', []),
        (ex, 'cp', 'esp32c6', ['CONFIG_EH_TRANSPORT_CP_UART=y']),
        (ex, 'mcu_host', 'esp32p4', ['CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART=y']),
    ])


@pytest.fixture
def worker_id(request):
    """xdist worker id ('gw0'…), or 'master' without -n; makes sockets unique."""
    return getattr(request.config, 'workerinput', {}).get('workerid', 'master')


@pytest.fixture
def _bench(lab_tmp, worker_id):
    """The SDIO power-save bench (C6+P4, wake-on-packet) behind the dut/net
    fixtures — one specific BenchSpec of the emu Target."""
    b = EmuTarget().make(
        BenchSpec(example=_PS_EXAMPLE, host_role='esp_host', transport='sdio',
                  wake=True, timeout='200s'),
        worker_id=worker_id, lab_tmp=lab_tmp)
    yield {'host': b.host, 'cp': b.cp, 'net': b.net}
    b.down()


@pytest.fixture
def dut(_bench):
    """[host, cp] — same shape/order as pytest-embedded's multi-dut on HW."""
    return [_bench['host'], _bench['cp']]


@pytest.fixture
def net(_bench):
    return _bench['net']


@pytest.fixture
def emu_bench(lab_tmp, worker_id):
    """Factory: launch a C6 CP + P4 host emu pair for examples/<example>/<host_role>
    over <transport> (sdio|uart). transport is a *parameter* — same test, swap the
    wire. All benches are reaped at teardown (fault-isolated per bench)."""
    target = EmuTarget()
    benches = []

    def _make(example, host_role, transport, wake=False, timeout='60s',
              cp_extra_ovl=(), extra_ovl=()):
        b = target.make(
            BenchSpec(example=example, host_role=host_role, transport=transport,
                      wake=wake, timeout=timeout,
                      cp_extra_ovl=tuple(cp_extra_ovl), extra_ovl=tuple(extra_ovl)),
            worker_id=worker_id, lab_tmp=lab_tmp)
        benches.append(b)
        out = {'host': b.host, 'cp': b.cp}
        if b.net is not None:
            out['net'] = b.net
        return out

    yield _make
    for b in benches:
        b.down()
