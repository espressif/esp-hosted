"""Per-test log archiver — freezes each run's CP/host/build logs into the record.

The bench fixtures tee each emulator/host subprocess to `lab_tmp/*.log` (the
per-test scratch under the run dir). At teardown this copies them into the frozen
record `runs/<ts>/logs/<test>/` (`.FAIL`-suffixed on failure) next to `live.log`,
so cp/host/build logs sit together for whoever reads the error.

Huge logs are head+tail capped (a wedged CP can spew 100s of MB of EMFILE spam) so
the record stays readable. Full logs remain in the scratch dir until the run dir
is cleaned.
"""
import logging
import os
import shutil
from pathlib import Path

import pytest

from infra import lab

_CAP_LINES = 4000    # keep first/last this-many when a log is oversized
_CAP_BYTES = 4 << 20  # ...triggered above this size (4 MiB)


def pytest_configure(config):
    # Mirror the 'eh' step trace ([build]/[emu]/[expect]/[cmd]) into the shared
    # live.log from EVERY process — controller AND each xdist worker. Without this
    # the trace runs on workers and never reaches live.log, so `tail -f live.log`
    # goes blind during builds under --jobs. O_APPEND single-line writes stay
    # atomic across processes; the reporter opens the same file in append mode.
    run_dir = os.environ.get("EH_RUN_DIR")
    if not run_dir:
        return
    live = Path(run_dir) / "live.log"
    eh = logging.getLogger("eh")
    if any(getattr(h, "_eh_live", False) for h in eh.handlers):
        return
    try:
        Path(run_dir).mkdir(parents=True, exist_ok=True)
        h = logging.FileHandler(live, mode="a")
    except OSError:
        return
    h.setFormatter(logging.Formatter("%(asctime)s  %(message)s", "%H:%M:%S"))
    h.setLevel(logging.INFO)
    h._eh_live = True
    eh.addHandler(h)
    eh.setLevel(logging.INFO)


def _copy_capped(src: Path, dst: Path):
    try:
        if src.stat().st_size <= _CAP_BYTES:
            shutil.copyfile(src, dst); return
    except OSError:
        return
    # Oversized: head + tail, with an elision marker in between.
    head, tail = [], []
    try:
        with open(src, "r", errors="replace") as fh:
            for i, line in enumerate(fh):
                if i < _CAP_LINES:
                    head.append(line)
                else:
                    tail.append(line)
                    if len(tail) > _CAP_LINES:
                        tail.pop(0)
        elided = max(0, (i + 1) - len(head) - len(tail))
        with open(dst, "w") as out:
            out.writelines(head)
            out.write(f"\n... [eh: {elided} lines elided — see full log in tmp] ...\n\n")
            out.writelines(tail)
    except OSError:
        pass


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    # Stash each phase's report so the fixture can tell pass from fail at teardown.
    rep = (yield).get_result()
    setattr(item, f"_eh_rep_{rep.when}", rep)


@pytest.fixture(autouse=True)
def _eh_logcap(request, lab_tmp):
    """Archive this test's bench logs after it runs. Autouse + lab_tmp so it
    shares the exact scratch dir the bench fixtures write their logs into."""
    yield
    if not os.environ.get("EH_RUN_DIR"):
        return
    logs = sorted(Path(lab_tmp).glob("*.log"))
    if not logs:
        return
    setup = getattr(request.node, "_eh_rep_setup", None)
    call = getattr(request.node, "_eh_rep_call", None)
    failed = bool((setup and setup.failed) or (call and call.failed))
    dest = lab.test_logs_dir(request.node.nodeid, failed)
    try:
        dest.mkdir(parents=True, exist_ok=True)
    except OSError:
        return
    for lg in logs:
        _copy_capped(lg, dest / lg.name)
