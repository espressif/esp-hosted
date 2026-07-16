"""Single source of truth for the test-lab filesystem layout.

Everything a run touches lives under ONE sandbox-bindable root, `tests/.work`
(bwrap binds this RW; the rest of the FS is read-only), split by lifecycle:

    tests/.work/
    ├─ cache/<config-hash>/   reused across runs (built firmware); disposable
    ├─ sock/                  emu Unix sockets (kept short + shallow — sun_path
    │                         is capped ~108 bytes, so NOT under runs/<ts>/…)
    └─ runs/<ts>/             per-run, fresh:
        ├─ scratch/<test>/    live writable working files (COW flash, raw logs)
        ├─ logs/<test>/       the frozen record (archived cp/host/build logs + seq.md)
        └─ report.md · live.log · events.jsonl

`run_dir()` is pinned by eh.py via $EH_RUN_DIR so the xdist controller and every
worker agree; consumers here derive the rest from it.
"""
import os
import shutil
import socket
import time
import zlib
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
WORK = _REPO / "tests" / ".work"
CACHE = WORK / "cache"
SOCK_DIR = WORK / "sock"
RUNS = WORK / "runs"

RETENTION_DEFAULT_DAYS = 30.0


def sanitize(nodeid: str) -> str:
    return (nodeid.replace("::", ".").replace("/", ".")
            .replace("[", "_").replace("]", ""))


def run_dir() -> Path:
    d = os.environ.get("EH_RUN_DIR")
    return Path(d) if d else RUNS / "adhoc"


def test_logs_dir(nodeid: str, failed: bool = False) -> Path:
    """The frozen per-test record dir (cp/host/build logs + seq.md)."""
    return run_dir() / "logs" / (sanitize(nodeid) + (".FAIL" if failed else ""))


def test_scratch(nodeid: str) -> Path:
    """Live per-test writable scratch (COW flash, raw logs) — under the bound root
    so a sandboxed emu can write it; discardable (the record lives in logs/)."""
    d = run_dir() / "scratch" / sanitize(nodeid)
    d.mkdir(parents=True, exist_ok=True)
    return d


def sock_path(worker: str, tag: str = "emu") -> str:
    """A short, shallow socket path (Unix sun_path ~108-byte cap forbids the deep
    runs/<ts>/… tree) that still sits under the sandbox-bound root."""
    SOCK_DIR.mkdir(parents=True, exist_ok=True)
    return str(SOCK_DIR / f"eh_{tag}_{worker}_{os.getpid()}.sock")


# ── Per-bench resource allocation ────────────────────────────────────────────
# Every bench pair must own a UNIQUE hostfwd TCP port and Unix socket, or two
# pairs collide with "Address already in use". Uniqueness is layered:
#
#   run   → each eh.py RUN gets a disjoint high-band window, seeded from
#           EH_RUN_DIR (set once by eh.py, shared by every xdist worker of the
#           run, unique per run). A FIXED low base was the bug behind the 2313
#           collision: every concurrent run/rerun (and every user on a shared
#           box) started at the same port, so their blocks overlapped.
#   worker → striped by _PORT_STRIDE within the run window (workers are separate
#           processes; the shared seed keeps their bases disjoint).
#   bench  → monotonic slot within THIS process, never reused (also side-steps
#           TCP TIME_WAIT).
#
# The band sits above privileged/well-known ports and BELOW the Linux ephemeral
# range (ip_local_port_range default 32768+), so the kernel's outbound-connection
# allocator never steals a test port. The base is a stable property fixed at
# import (process start), not re-derived per bind. _port_free() remains a final
# guard against a cross-run straggler landing in an aliased window.
_PORT_LO = 10000
_PORT_HI = 32000                                   # < 32768 ephemeral floor
_PORT_STRIDE = 100                                 # ports per xdist worker
_WORKERS_PER_RUN = 16                              # worker blocks per run window
_WORKER_SPAN = _WORKERS_PER_RUN * _PORT_STRIDE     # 1600
_RUN_SLOTS = (_PORT_HI - _PORT_LO) // _WORKER_SPAN  # disjoint run windows

# crc32 (not hash()) — hash() of str is per-process randomized, which would give
# each xdist worker a different base and break worker striping.
_run_seed = os.environ.get("EH_RUN_DIR") or str(os.getppid())
_PORT_BASE = _PORT_LO + (zlib.crc32(_run_seed.encode()) % _RUN_SLOTS) * _WORKER_SPAN
_bench_slot = 0


def _worker_num(worker: str) -> int:
    n = int(worker[2:]) if worker.startswith("gw") and worker[2:].isdigit() else 0
    return n % _WORKERS_PER_RUN   # keep every worker inside this run's window


def _port_free(p: int) -> bool:
    """True if 127.0.0.1:p can be bound right now (no live bind, no TIME_WAIT).
    No SO_REUSEADDR — mirror the emu's plain bind so we skip a port it couldn't take."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("127.0.0.1", p))
            return True
        except OSError:
            return False


def alloc_bench(worker: str) -> dict:
    """Reserve a fresh {slot, port} for one bench pair. `slot` uniquifies the
    socket path; `port` is the hostfwd bind. Call once per bench in make().

    The slot-derived port is unique within THIS run, but a lingering emu from a
    killed/overlapping run (or another eh.py process) can still hold it — so probe
    and skip any port not actually free, avoiding a spurious 'Address already in
    use' at emu launch. We check the base port AND the next few (the bench binds a
    small contiguous block: wake→22 at `port`, hostfwd at port+1+i)."""
    global _bench_slot
    n = _worker_num(worker)
    for _ in range(_PORT_STRIDE):  # bounded: at most one lap of this worker's block
        slot = _bench_slot
        _bench_slot += 1
        port = _PORT_BASE + n * _PORT_STRIDE + slot % _PORT_STRIDE
        if all(_port_free(port + k) for k in range(4)):
            return {"slot": slot, "port": port}
    # Exhausted the block (every candidate held) — fall back to the raw slot port;
    # the emu will surface the real bind error rather than us looping forever.
    return {"slot": _bench_slot, "port": _PORT_BASE + n * _PORT_STRIDE + _bench_slot % _PORT_STRIDE}


def retention_days() -> float:
    """Days to keep run reports + build-cache entries. Default 30; any run
    overrides via $EH_RETENTION_DAYS (0 or negative disables the sweep)."""
    v = os.environ.get("EH_RETENTION_DAYS")
    if v:
        try:
            return float(v)
        except ValueError:
            pass
    return RETENTION_DEFAULT_DAYS


def sweep_expired(days=None) -> int:
    """Prune runs/ and cache/ entries whose mtime is older than `days` (default
    retention_days()). One stat per top-level entry, so it's cheap to run at every
    startup. Never removes the current run ($EH_RUN_DIR). A cache dir's mtime is
    bumped on every reuse (build_fw), so 'old' means genuinely unused. Returns the
    count removed."""
    days = retention_days() if days is None else days
    if days <= 0:
        return 0
    cutoff = time.time() - days * 86400.0
    keep = os.environ.get("EH_RUN_DIR")
    keep = str(Path(keep)) if keep else None
    removed = 0
    for base in (RUNS, CACHE):
        if not base.is_dir():
            continue
        for d in base.iterdir():
            if keep and str(d) == keep:
                continue
            try:
                if d.stat().st_mtime < cutoff:
                    shutil.rmtree(d, ignore_errors=True)
                    removed += 1
            except OSError:
                pass
    return removed
