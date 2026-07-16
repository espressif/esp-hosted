"""Event-bus reporter — the transparency/responsiveness seam.

The engine (pytest) emits a typed event stream; reporters consume it. Per run
(`runs/<ts>/`) this plugin writes:
  * `events.jsonl` — machine stream any sink can tail (WS/UI).
  * `live.log`     — HUMAN, tailable line-per-event (▶ start / ✓ pass / ✗ fail),
                     so even under `--jobs` (where xdist hides the live step
                     trace) you can `tail -f` it to see exactly what's running
                     and what's stuck.
  * `report.md`    — on finish: summary + mermaid timeline.

Events: session_start · test_start · test_end{outcome,duration} · session_end.
"""
import json
import sys
import time
from pathlib import Path

from infra import lab

_RUNS = lab.RUNS  # tests/.work/runs
_MARK = {"passed": "✓ PASS", "failed": "✗ FAIL", "skipped": "⚪ SKIP"}


class _Bus:
    def __init__(self):
        self.dir = None
        self.fh = None       # events.jsonl
        self.live = None     # human live.log
        self.events = []
        self.t0 = None
        self.console = False  # mirror per-test lines to the console (batch mode)
        self.regression = False  # full-matrix run (--regression) vs default sanity

    def emit(self, **ev):
        ev["t"] = time.time()
        self.events.append(ev)
        if self.fh:
            self.fh.write(json.dumps(ev) + "\n")
            self.fh.flush()

    def line(self, msg):
        stamped = f"{time.strftime('%H:%M:%S')}  {msg}\n"
        if self.live:
            self.live.write(stamped)
            self.live.flush()
        if self.console:
            # One atomic line per event, emitted only by the single xdist
            # controller — so under --jobs the per-test names/results show live
            # WITHOUT the interleaved multi-line spew of N parallel workers.
            sys.stderr.write(stamped)
            sys.stderr.flush()


_bus = _Bus()


def pytest_configure(config):
    # Only the xdist controller (no workerinput) owns the stream/report; workers
    # forward their reports to it, so we still see every test exactly once.
    if hasattr(config, "workerinput"):
        return
    # Retention: prune runs/ + cache/ past their expiry (default 30d,
    # $EH_RETENTION_DAYS) before this run's dir is created — the "scan on startup".
    try:
        n = lab.sweep_expired()
        if n:
            print(f"[eh] retention: pruned {n} expired run/cache dir(s) "
                  f"(older than {lab.retention_days():.0f}d; set EH_RETENTION_DAYS to change)")
    except Exception:  # noqa: BLE001 — housekeeping must never break a run
        pass
    # eh.py pins one run dir for the controller + all xdist workers (so the live
    # log and per-test log archive co-locate); fall back to a fresh ts dir when
    # pytest is invoked directly.
    import os
    env_dir = os.environ.get("EH_RUN_DIR")
    ts = Path(env_dir).name if env_dir else time.strftime("%Y%m%d-%H%M%S")
    _bus.dir = Path(env_dir) if env_dir else _RUNS / ts
    _bus.dir.mkdir(parents=True, exist_ok=True)
    _bus.fh = open(_bus.dir / "events.jsonl", "w")
    # Append (not truncate): xdist workers append their step trace to this same
    # file (see infra.eh_logcap); O_APPEND keeps single-line writes atomic across
    # processes. The run dir is unique per run, so there's nothing to truncate.
    _bus.live = open(_bus.dir / "live.log", "a")
    _bus.t0 = time.time()
    # Batch/parallel run (xdist -n N) hides the per-step log_cli trace, so mirror
    # the per-test start/result lines to the console (see _Bus.line). Sequential
    # runs already stream the full trace, so leave console off to avoid dup output.
    _bus.console = bool(getattr(config.option, "numprocesses", 0) or 0)
    _bus.regression = bool(config.getoption("regression", default=False))
    _bus.emit(event="session_start", ts=ts)
    _bus.line("session start")


def pytest_report_header(config):
    # Shows at the top of every run so the user knows where to watch — even
    # under --jobs, where the per-step [build]/[expect]/[cmd] trace is hidden.
    if _bus.dir:
        return (f"eh: live progress → tail -f {_bus.dir}/live.log   "
                f"(per-step trace shows inline when run WITHOUT --jobs)")
    return None


def pytest_runtest_logstart(nodeid, location):
    if _bus.fh:
        _bus.emit(event="test_start", nodeid=nodeid)
        _bus.line(f"▶ START {nodeid}")


def pytest_runtest_logreport(report):
    if not _bus.fh:
        return
    if report.when == "call" or (report.when == "setup" and report.outcome != "passed"):
        dur = round(getattr(report, "duration", 0.0), 3)
        # Marker names ride in report.keywords; stamp the two lifecycle markers on the
        # event so any consumer (matrix, eh.py second_chance merge) can classify the
        # outcome without re-collecting.
        kw = getattr(report, "keywords", {})
        _bus.emit(event="test_end", nodeid=report.nodeid, outcome=report.outcome,
                  duration=dur, when=report.when,
                  allow_fail=("allow_fail" in kw), second_chance=("second_chance" in kw))
        msg = f"{_MARK.get(report.outcome, report.outcome)} {report.nodeid} ({dur:.1f}s)"
        if report.outcome == "failed":
            # Point at the archived per-test logs so a failure can be assessed
            # mid-run without hunting — cp/host build + runtime logs live here.
            msg += f"  → {_bus.dir}/logs/{lab.sanitize(report.nodeid)}.FAIL/"
        _bus.line(msg)


def pytest_sessionfinish(session, exitstatus):
    if not _bus.fh:
        return
    _bus.emit(event="session_end", exitstatus=int(exitstatus))
    _bus.line(f"session end (exit {int(exitstatus)})")
    # Under eh.py's two-phase orchestration the merged (post-second_chance) matrix is
    # rendered by eh.py after both phases; skip the per-invocation one so a soon-to-be-
    # recovered flake isn't shown as a hard ✗. Direct pytest runs still print here.
    import os
    if not os.environ.get("EH_DEFER_MATRIX"):
        _print_matrix()
    _write_report()
    _bus.fh.close()
    if _bus.live:
        _bus.live.close()
    print(f"\n[eh] run trace: {_bus.dir}/report.md   live: {_bus.dir}/live.log   "
          f"events: {_bus.dir}/events.jsonl")


_TRANSPORTS = ["sdio", "spi_fd", "spi_hd", "uart"]
_ANSI = {"grey": "\033[90m", "green": "\033[32m", "red": "\033[31m",
         "yellow": "\033[33m", "bold": "\033[1m", "off": "\033[0m"}


def _scen_col(nodeid):
    """(scenario, transport-column) for a nodeid. The transport is the whole param
    (sdio/uart/spi_fd/spi_hd[_N]) or the suffix of a compound id like
    "<example>-<wire>" (the boot cases, e.g. network_split_iperf-uart). spi_hd_{1,2,4}
    fold to spi_hd; a non-transport param (rep number / none / linux) sits under
    sdio, the default emu wire."""
    tail = nodeid.split("::")[-1].split("@")[0]
    fn, _, rest = tail.partition("[")
    param = rest.rstrip("]").lower()
    tok = param.rsplit("-", 1)[-1] if "-" in param else param
    if tok in ("sdio", "uart", "spi_fd"):
        col = tok
    elif tok.startswith("spi_hd"):
        col = "spi_hd"
    else:
        col = "sdio"
    if param and "boot" in fn:              # keep each example boot as its own row
        fn = f"{fn}[{param}]"
    return fn, col


# Effective per-cell STATES (outcome after applying the two lifecycle markers):
#   passed / skipped / failed — as before
#   tolerated — an allow_fail test that ran & FAILED: red ⊗ (distinct from a hard ✗),
#               non-blocking — "ran, failed, fix later"
#   recovered — a second_chance test that failed in parallel then PASSED isolated:
#               green ↻ (flaky-but-green) — distinct from a clean ✓
_STATE_GLYPH = {"passed": ("green", "✓"), "failed": ("red", "✗"),
                "skipped": ("yellow", "–"), "tolerated": ("red", "⊗"),
                "recovered": ("green", "↻")}
# Cell-fold severity when several params collapse onto one (scenario,transport) cell:
# a hard fail dominates a tolerated fail dominates skip dominates recovered/pass.
_STATE_RANK = {"passed": 0, "recovered": 1, "skipped": 2, "tolerated": 3, "failed": 4}


def _state_of(outcome, allow_fail=False, recovered=False):
    """Map a raw outcome + markers to an effective matrix state."""
    if recovered:
        return "recovered"
    if outcome == "failed":
        return "tolerated" if allow_fail else "failed"
    return outcome  # passed / skipped


def _grid_from_states(items):
    """items: iterable of (nodeid, state) → (grid{scen:{col:state}}, ordered scen list)."""
    grid, order = {}, []
    for nid, st in items:
        scen, col = _scen_col(nid)
        row = grid.setdefault(scen, {})
        if scen not in order:
            order.append(scen)
        if col not in row or _STATE_RANK[st] > _STATE_RANK[row[col]]:
            row[col] = st
    return grid, order


def _render_matrix(grid, order, regression):
    color = sys.stderr.isatty()

    def c(txt, key):
        return f"{_ANSI[key]}{txt}{_ANSI['off']}" if color else txt

    def cell(state):
        # fixed 8-wide column; visible content is 2 glyphs, centred by hand so the
        # ANSI codes (zero visible width) don't skew Python's str formatting.
        if state is None:
            return " " * 8
        col_key, g = _STATE_GLYPH[state]
        return "   " + c("✓", "grey") + c(g, col_key) + "   "  # grey ran-tick + result

    wscen = max([len("scenario")] + [len(s) for s in order]) + 2
    title = "Regression matrix" if regression else "Sanity test performed"
    hdr = f"{'scenario':<{wscen}}" + "".join(f"{t:^8}" for t in _TRANSPORTS)
    lines = ["", c(f"{title}  (grey ✓ = ran · green ✓ pass · red ✗ fail · yellow – skip · "
                   f"red ⊗ allow_fail · green ↻ recovered 2nd-chance)", "bold"),
             hdr, "─" * (wscen + 8 * len(_TRANSPORTS))]
    for scen in order:
        cells = "".join(cell(grid[scen].get(t)) for t in _TRANSPORTS)
        lines.append(f"{scen:<{wscen}}{cells}")
    out = "\n".join(lines) + "\n"
    sys.stderr.write(out)
    sys.stderr.flush()
    if _bus.live and not _bus.live.closed:
        import re
        _bus.live.write(re.sub(r"\033\[[0-9]+m", "", out))
        _bus.live.flush()


def _print_matrix():
    """End-of-run coverage matrix: scenarios × transports. Each cell that ran gets
    a grey ✓ (ran) followed by the outcome glyph. Shows at a glance that the set
    touches every wire without duplicating."""
    ends = [e for e in _bus.events if e["event"] == "test_end"]
    if not ends:
        return
    items = [(e["nodeid"], _state_of(e["outcome"], e.get("allow_fail", False)))
             for e in ends]
    grid, order = _grid_from_states(items)
    _render_matrix(grid, order, _bus.regression)


def _load_ends(run_dir):
    """Read the test_end events out of a run's events.jsonl (controller-owned, one
    per test). Tolerant of a missing/partial file so a merge never crashes a run."""
    p = Path(run_dir) / "events.jsonl"
    out = []
    if not p.exists():
        return out
    for line in p.read_text().splitlines():
        try:
            ev = json.loads(line)
        except ValueError:
            continue
        if ev.get("event") == "test_end":
            out.append(ev)
    return out


def summarize(run_dir, retry_dir=None, regression=False, elapsed=None, prewarm=None):
    """eh.py's authoritative post-run render: merge the parallel phase (run_dir) with
    the optional isolated second_chance re-run (retry_dir, whose result is FINAL),
    print the merged matrix + a grouped truthful summary, and return the failure
    classification so eh.py can compute the pipeline exit code:

        {"recovered": [...], "allow_fail": [...], "blocking": [...]}

    Exit is non-zero iff 'blocking' is non-empty (a FINAL failure that is not allow_fail)."""
    def _key(n):
        # xdist_group runs carry a "nodeid@group" display suffix; the -n0 retry
        # emits the bare nodeid. Key both phases on the bare id or the retry's
        # pass can never flip the parallel-phase failure it recovered.
        i = n.rfind("@")
        return n[:i] if i > n.rfind("]") else n

    final = {}
    for e in _load_ends(run_dir):
        final[_key(e["nodeid"])] = {"nodeid": e["nodeid"], "outcome": e["outcome"],
                              "allow_fail": e.get("allow_fail", False),
                              "second_chance": e.get("second_chance", False),
                              "recovered": False}
    if retry_dir:
        for e in _load_ends(retry_dir):
            k = _key(e["nodeid"])
            f = final.setdefault(k, {"nodeid": e["nodeid"],
                                               "allow_fail": e.get("allow_fail", False),
                                               "second_chance": True, "recovered": False})
            # the isolated re-run is the FINAL word; a failed→passed flip is "recovered".
            if e["outcome"] == "passed" and final.get(k, {}).get("outcome") == "failed":
                f["recovered"] = True
            f["outcome"] = e["outcome"]

    if not final:  # nothing ran (e.g. --collect-only / -k with no match): no matrix
        return {"recovered": [], "allow_fail": [], "blocking": []}
    items = [(f["nodeid"], _state_of(f["outcome"], f["allow_fail"], f["recovered"]))
             for f in final.values()]
    grid, order = _grid_from_states(items)
    _render_matrix(grid, order, regression)

    recovered = sorted(f["nodeid"] for f in final.values() if f["recovered"])
    allow_fail = sorted(f["nodeid"] for f in final.values()
                        if f["outcome"] == "failed" and f["allow_fail"])
    blocking = sorted(f["nodeid"] for f in final.values()
                      if f["outcome"] == "failed" and not f["allow_fail"])
    counts = {"passed": sum(1 for f in final.values()
                            if f["outcome"] == "passed" and not f["recovered"]),
              "recovered": len(recovered), "allow_fail": len(allow_fail),
              "blocking": len(blocking),
              "skipped": sum(1 for f in final.values() if f["outcome"] == "skipped")}
    _print_groups(recovered, allow_fail, blocking, run_dir, counts, elapsed, prewarm)
    return {"recovered": recovered, "allow_fail": allow_fail, "blocking": blocking}


def _print_groups(recovered, allow_fail, blocking, run_dir, counts, elapsed=None, prewarm=None):
    """Truthful end-of-run grouping + a single closing verdict line (always printed).
    Every not-clean-green outcome is shown in its own labelled bucket so no failure is
    ever hidden by the allow_fail/second_chance path; the verdict states the overall
    GREEN/RED (pipeline exit) so the run never ends ambiguously."""
    color = sys.stderr.isatty()

    def c(txt, key):
        return f"{_ANSI[key]}{txt}{_ANSI['off']}" if color else txt

    def paths(n):
        """Log dir(s) for a failed nodeid: the phase-1 archive, plus the isolated
        second_chance retry archive if it exists (the cleaner one to debug from)."""
        slug = lab.sanitize(n)
        out = [f"      → {run_dir}/logs/{slug}.FAIL/"]
        retry = Path(run_dir) / "second_chance" / "logs" / f"{slug}.FAIL"
        if retry.is_dir():
            out.append(f"      → {retry}/   (isolated retry — clean repro)")
        return out

    lines = []
    if recovered:
        lines.append(c("recovered on 2nd chance (flaky — passed isolated):", "bold"))
        for n in recovered:
            lines.append(f"  {c('↻', 'green')} {n}")
            lines.append(f"      → {run_dir}/logs/{lab.sanitize(n)}.FAIL/   (phase-1 flake log)")
    if allow_fail:
        lines.append(c("allow_fail (non-blocking, fix later):", "bold"))
        for n in allow_fail:
            lines.append(f"  {c('⊗', 'red')} {n} FAILED")
            lines += paths(n)
    if blocking:
        lines.append(c("BLOCKING failures (pipeline red):", "bold"))
        for n in blocking:
            lines.append(f"  {c('✗', 'red')} {n} FAILED")
            lines += paths(n)
    # Always: one closing verdict line, so the run has an unambiguous end state.
    # Only surface the categories that actually have entries — a clean run reads
    # "N passed → GREEN", with no 0-count recovered/allow_fail/blocking/skipped noise;
    # the detail groups above already print only when non-empty.
    green = not blocking
    parts = [f"{counts['passed']} passed"]
    if counts['recovered']:  parts.append(f"{counts['recovered']} recovered ↻")
    if counts['allow_fail']: parts.append(f"{counts['allow_fail']} allow_fail ⊗")
    if counts['blocking']:   parts.append(f"{counts['blocking']} blocking ✗")
    if counts['skipped']:    parts.append(f"{counts['skipped']} skipped –")
    tally = " · ".join(parts)
    if elapsed:
        def _hms(sec):
            s = int(round(sec))
            return f"{s // 60}:{s % 60:02d}" if s >= 60 else f"{s}s"
        if prewarm:
            # account BOTH phases: headline is total wall time, with the split shown
            tally += f" in {_hms(prewarm + elapsed)} (prewarm {_hms(prewarm)} + run {_hms(elapsed)})"
        else:
            tally += f" in {_hms(elapsed)}"
    verdict = c("GREEN", "green") if green else c("RED", "red")
    lines.append(f"{c('eh verdict:', 'bold')} {tally}  →  {verdict}")
    out = "\n" + "\n".join(lines) + "\n"
    sys.stderr.write(out)
    sys.stderr.flush()
    try:
        import re
        live = Path(run_dir) / "live.log"
        with open(live, "a") as fh:
            fh.write(re.sub(r"\033\[[0-9]+m", "", out))
    except OSError:
        pass


def _write_report():
    ends = [e for e in _bus.events if e["event"] == "test_end"]
    starts = {e["nodeid"]: e["t"] for e in _bus.events if e["event"] == "test_start"}
    passed = sum(1 for e in ends if e["outcome"] == "passed")
    failed = sum(1 for e in ends if e["outcome"] == "failed")
    skipped = sum(1 for e in ends if e["outcome"] == "skipped")
    total = time.time() - _bus.t0

    md = ["# Test run trace", "",
          f"**{passed} passed · {failed} failed · {skipped} skipped** "
          f"in {total:.1f}s", "",
          f"Detailed logs per test: `{_bus.dir}/logs/<test>/` — each holds "
          f"`{{cp,host}}_build.log` (firmware build), `{{cp,host}}.log` (runtime), "
          f"and `seq.md` (sequence diagram). Reused build cache: `{lab.CACHE}`.", "",
          "| test | outcome | duration | logs |", "|---|---|---|---|"]
    for e in ends:
        nid = e["nodeid"].split("::", 1)[-1]
        mark = {"passed": "✅", "failed": "❌", "skipped": "⚪"}.get(e["outcome"], e["outcome"])
        d = lab.sanitize(e["nodeid"]) + (".FAIL" if e["outcome"] == "failed" else "")
        md.append(f"| `{nid}` | {mark} | {e['duration']:.2f}s | [`logs/{d}/`](logs/{d}/) |")
    md += ["", "```mermaid", "gantt", "  dateFormat X", "  axisFormat %Ss",
           "  title Test timeline"]
    # one section = the whole run; each test a bar from its start (rel s) for its duration
    md.append("  section run")
    for i, e in enumerate(ends):
        nid = e["nodeid"].split("::")[-1].replace(":", "_")[:36]
        s = int(max(starts.get(e["nodeid"], _bus.t0) - _bus.t0, 0))
        dur = max(int(round(e["duration"])), 1)
        tag = "done" if e["outcome"] == "passed" else ("active" if e["outcome"] == "skipped" else "crit")
        md.append(f"  {nid} :{tag}, t{i}, {s}, {dur}s")
    md += ["```", ""]
    (_bus.dir / "report.md").write_text("\n".join(md))
