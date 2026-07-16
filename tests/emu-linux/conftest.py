"""
ESP-Hosted Linux substrate — conftest.py

Thin pytest adapter over the linux Target provider (`targets/linux.py`), which
runs the REAL Linux userspace host (vserial, no kernel module, no mock) against
an emulated C6 CP via the vserial↔eh_frame shim over a PTY. The `dut` reads
identically to the emu/HW suites — config, not mock (real read()/write() on a
real PTY).
"""
import os
import sys

import pytest

_tests_dir = os.path.join(os.path.dirname(__file__), '..')
if _tests_dir not in sys.path:
    sys.path.insert(0, _tests_dir)

from targets.base import BenchSpec      # noqa: E402
from targets.linux import LinuxTarget   # noqa: E402


@pytest.fixture
def worker_id(request):
    return getattr(request.config, 'workerinput', {}).get('workerid', 'master')


@pytest.fixture
def linux_bench(lab_tmp, worker_id):
    """Factory: CP(c6, uart-emu) + shim + Linux host for examples/<example>.
    Returns {'host','cp'} — the Linux host process behind the Dut surface."""
    target = LinuxTarget()
    benches = []

    def _make(example='system/get_cp_fw_version'):
        b = target.make(
            BenchSpec(example=example, host_role='linux_802_3_host/c_app', transport='uart'),
            worker_id=worker_id, lab_tmp=lab_tmp)
        benches.append(b)
        return {'host': b.host, 'cp': b.cp}

    yield _make
    for b in benches:
        b.down()


@pytest.fixture
def dut(linux_bench):
    """Default single-DUT: the Linux host for get_cp_fw_version over the shim."""
    b = linux_bench('system/get_cp_fw_version')
    return [b['host']]
