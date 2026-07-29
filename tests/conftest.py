# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Root conftest for all ESP-Hosted test substrates (emu / hw / linux).

Puts tests/ on sys.path (so suites share infra/) and registers the event-bus
reporter for every run. Per-substrate conftests (emu/, linux/, hw/) nest under this.

Sanity distribution — the default run (`eh.py test emu`) executes only the
`@pytest.mark.sanity` cell of each scenario; `--regression` runs the FULL matrix
— every scenario on every wire. Sanity picks are spread so the set touches all
four transports once, with the slowest tests on faster wires. Keep this table in
sync when adding/moving a mark (the reporter prints the live matrix at end-of-run).
S = sanity wire · R = also under --regression · maint = @maintenance (reasoned) ·
sc = @second_chance (runs and BLOCKS like any wire; a parallel-load flake is re-run
isolated — green ↻ if it recovers, a genuine both-times fail is a real ✗):

    scenario                          sdio  spi_fd  spi_hd  uart
    ------------------------------------------------------------
    control_plane (full RPC sweep)     R      S      R      R
    cp_fw_version                      R      R      R      S
    nw_split_host_ip                   R      R      R      R
    nw_split_port_routing              S     sc      R      R     spi_fd load-flake (retry recovers)
    ota_littlefs (slowest)             S      R      R      R
    wifi_connected                     R     sc      R      S     spi_fd load-flake (retry recovers)
    spi_hd_transport (HD-specific)     .      .      S      .
    api_exerciser (control_plane dup)  .      S      .      .     other wires @retired
    power_save (wake path)             S      .      .      R
    example_boots (every example ×4)   R      R      R      R
    wifi_sta_connect                   R      S      R      R
    linux (802.3 substrate)            —      —      —      —     own substrate; S in emu-linux

Full matrix under --regression: every transport-agnostic scenario runs on ALL four
wires. The only cells NOT run are @maintenance (reasoned) or @retired
(api_exerciser dup) — visible and reasoned, never silently omitted. spi_fd now GATES
like every other wire (no allow_fail): its systematic break — the emu defaulted the
CP CS pin to GPIO 10 while the FUNC_BOARD CP uses 18, so the CP never armed a
transaction and every spi_fd RPC timed out — is FIXED (emu.py now passes
`--hosted-spi-cs-gpio`). What remains is a parallel-load flake, so spi_fd cells carry
@second_chance only: a phase-1 failure is re-run ISOLATED (-n0) and that is final —
recovered shows green ↻; a genuine both-times fail is a real ✗ that blocks.

Two emu characteristics this design accommodates rather than dodges: (1) under
`--jobs auto` host-CPU oversubscription the emulated C6 is starved, so the P4 host —
running its firmware ~50x real-time — burns the 5s RPC-response window in ~100ms of
wall-clock and times out a CP that just wasn't OS-co-scheduled in that sliver. The
esp-emulator throttle fix (`maybe_throttle` now paces the P4 MASTER to wall-clock while
the C6 slave stays flat-out) targets that skew and largely resolves it (parallel runs
mostly fully clean); the residual is external CPU contention on the shared dev box,
which @second_chance's isolated retry recovers. Basic assoc over SPI is also proven on
all four wires by `wifi_sta_connect` + `nw_split_host_ip`. Assoc-heavy + all spi_fd
sweeps are serialized via the `emu_heavy` xdist loadgroup; (2) the ×4 example boots share an
`emu_boots` group so the breadth sweep serializes instead of flooding the 15-way run
and starving the timing-sensitive power-save tests (which are themselves marginally
flaky under full-load oversubscription — e.g. power_save_wake_mem[sdio] bring-up).
"""
import os
import sys

_tests_dir = os.path.dirname(__file__)
if _tests_dir not in sys.path:
    sys.path.insert(0, _tests_dir)

import pytest  # noqa: E402

# The engine emits a typed event stream; the reporter consumes it (JSONL + trace).
# eh_logcap archives each test's CP/host logs next to the run's live.log.
pytest_plugins = ("infra.eh_reporter", "infra.eh_logcap")


def pytest_addoption(parser):
    parser.addoption(
        "--regression", action="store_true", default=False,
        help="run the FULL matrix (every test on every transport). Default runs "
             "only @pytest.mark.sanity — one representative per scenario, spread "
             "across transports so the set covers all wires without duplicating.")
    parser.addoption(
        "--regression-exclude-sanity", action="store_true", default=False,
        help="run the full matrix MINUS the @sanity representatives (only the extra "
             "regression coverage). Pair with a prior default (sanity) run to split "
             "the full matrix into two non-overlapping passes.")


def pytest_collection_modifyitems(config, items):
    """Lifecycle + selection markers (all function- and per-param granular via
    pytest.param(..., marks=pytest.mark.<m>)):

    - @pytest.mark.retired(reason="...") — DESELECTED: never runs, never in the
      pass/fail stats. For a case fully covered elsewhere, kept as a record.
    - @pytest.mark.maintenance(reason="...") — SKIPPED: shown as skipped
      'maintenance: <reason>'. Temporarily parked while its support is fixed.
    - @pytest.mark.sanity — the DEFAULT set. Without --regression, only sanity
      items run (one representative (test, transport) per scenario, distributed
      so the set exercises every transport once); the rest are deselected. With
      --regression the full matrix runs.
    - @pytest.mark.allow_fail(reason="...") — RUNS normally, but a FAILURE is
      NON-blocking: eh.py keeps the pipeline exit 0 and reports it truthfully in
      its own 'allow_fail (non-blocking, fix later)' summary group + a red ⊗ matrix
      glyph (distinct from a hard ✗). For known-flaky cells kept visible until fixed.
    - @pytest.mark.second_chance — RUNS in the parallel phase; if it FAILS there,
      eh.py re-runs it ISOLATED (-n0, one at a time — see tools/eh.py
      _run_second_chance) and takes that as the FINAL outcome. Recovered-on-retry
      shows as a green ↻ ('recovered on 2nd chance'); a both-fail is a real failure.
      For tests that pass reliably alone but flake under parallel emu load.
      AUTO-APPLIED to every spi_fd cell below (the emu spi_fd data/RPC flake is
      transport-wide under load, not test-specific), so no per-test marking is needed
      for spi_fd; also settable explicitly for non-spi_fd cells (e.g. power_save[sdio]).

    allow_fail + second_chance are enforced by eh.py post-run (they carry through to
    the events.jsonl the reporter writes); collection here just leaves them running.
    A cell may carry BOTH (second_chance decides the FINAL outcome, THEN allow_fail
    decides whether it blocks) — but spi_fd now carries second_chance ONLY, so a
    genuine spi_fd failure blocks like every other wire."""
    regression = config.getoption("regression", default=False)
    excl_sanity = config.getoption("regression_exclude_sanity", default=False)
    retired, keep = [], []
    for item in items:
        if item.get_closest_marker("retired"):
            retired.append(item)
            continue
        m = item.get_closest_marker("maintenance")
        if m:
            reason = m.kwargs.get("reason") or (m.args[0] if m.args else "") or "parked"
            item.add_marker(pytest.mark.skip(reason=f"maintenance: {reason}"))
        # spi_fd's systematic break (the emu CP CS-pin: emu defaulted CS to GPIO 10
        # while FUNC_BOARD uses 18, so the CP never armed a transaction) is FIXED, so
        # spi_fd now GATES like every other wire — NO allow_fail. What remains is
        # transient CPU-starvation skew under parallel load on a shared box (the P4
        # host advances its firmware 5s RPC timer while the CP emu is briefly
        # OS-descheduled). second_chance (isolated retry) recovers that env noise;
        # a genuine both-times failure is a real ✗ that blocks. spi_fd cells are also
        # serialized into the one emu_heavy worker to cut concurrent emu-pairs.
        cs = getattr(item, "callspec", None)
        if cs and "spi_fd" in cs.params.values():
            item.add_marker(pytest.mark.xdist_group("emu_heavy"))
            item.add_marker(pytest.mark.second_chance)
        keep.append(item)
    deselected = list(retired)
    if excl_sanity:
        # Full matrix MINUS the sanity reps (the complement of a default sanity run).
        sanity = [it for it in keep if it.get_closest_marker("sanity")]
        deselected += sanity
        keep = [it for it in keep if not it.get_closest_marker("sanity")]
    elif not regression:
        # Default run: keep only the sanity representatives.
        non_sanity = [it for it in keep if not it.get_closest_marker("sanity")]
        deselected += non_sanity
        keep = [it for it in keep if it.get_closest_marker("sanity")]
    if deselected:
        config.hook.pytest_deselected(items=deselected)
    items[:] = keep


@pytest.fixture
def lab_tmp(request):
    """Per-test writable scratch UNDER the sandbox-bound root (tests/.work/runs/
    <ts>/scratch/<test>/) — replaces pytest's tmp_path so a sandboxed emu can
    write its COW flash and raw logs. eh_logcap archives the record into logs/."""
    from infra import lab
    return lab.test_scratch(request.node.nodeid)


@pytest.fixture(scope="session")
def substrate():
    """Which substrate this session runs on — chosen at RUNTIME by EH_SUBSTRATE
    (set by `eh.py test <substrate>`), default 'emu'. This is the single seam
    where sw-vs-hw is decided; nothing downstream hardcodes it."""
    return os.environ.get("EH_SUBSTRATE", "emu")


@pytest.fixture
def worker_id(request):
    """xdist worker id ('gw0'…) or 'master'; keeps per-bench resources unique."""
    return getattr(request.config, "workerinput", {}).get("workerid", "master")


@pytest.fixture
def wifi_ap(substrate):
    """AP credentials for STA-connect tests — substrate-appropriate. On the emu the
    AP is the emulator's fixed modeled SoftAP (EMU_SOFTAP), NOT the physical AP: the
    emu can only associate to what its wifi model advertises. On real benches
    (hw/rpi) it's lab.local.json 'wifi_ap' (machine-local) else tests/env.json.
    None if no AP is configured."""
    if substrate == "emu":
        from targets.emu import EMU_SOFTAP
        return EMU_SOFTAP
    import json
    from infra import lab
    for f in ("lab.local.json", "env.json"):
        try:
            ap = json.loads((lab._REPO / "tests" / f).read_text()).get("wifi_ap")
        except (OSError, ValueError):
            ap = None
        if ap and ap.get("ssid"):
            return ap
    return None


@pytest.fixture(scope="session")
def tap_available():
    """True if this process can create a TAP interface (needs CAP_NET_ADMIN).
    The emu models a real AP only with TAP; without it it falls back to --net
    user (smoltcp NAT), which leases the STA netif an IP at boot — before any
    Wi-Fi association. Tests whose contract depends on association-gated IP
    (e.g. 'no IP before connect') must skip when this is False."""
    import os, fcntl, struct
    TUNSETIFF, IFF_TAP, IFF_NO_PI = 0x400454ca, 0x0002, 0x1000
    try:
        fd = os.open("/dev/net/tun", os.O_RDWR)
    except OSError:
        return False
    try:
        fcntl.ioctl(fd, TUNSETIFF, struct.pack("16sH", b"ehtapprobe%d", IFF_TAP | IFF_NO_PI))
        return True
    except OSError:
        return False
    finally:
        try:
            os.close(fd)
        except OSError:
            pass


@pytest.fixture
def bench_caps(substrate):
    """The substrate's capability set WITHOUT provisioning a bench — so a test can
    skip on a missing cap (e.g. needs a real AP) before paying a build+flash."""
    if substrate == "hw":
        from targets.serial import SerialTarget
        return SerialTarget().caps
    if substrate == "rpi":
        from targets.rpi import RpiTarget
        return RpiTarget().caps
    if substrate == "linux":
        from targets.linux import LinuxTarget
        return LinuxTarget().caps
    from targets.emu import EmuTarget
    return EmuTarget().caps


@pytest.fixture
def bench(substrate, lab_tmp, worker_id):
    """The one bench factory the shared corpus calls — substrate-agnostic.

        b = bench('system/api_exerciser', 'mcu_host', transport)
        host, caps = b['host'], b['caps']

    The provider is selected by `substrate` (emu | hw); it reports its own
    `transports` (a test asking for an unprovided wire SKIPs) and stamps runtime
    `caps` on the bench so bodies branch on capability, never on substrate name.
    Benches made here are reaped at test end."""
    from targets.base import BenchSpec
    if substrate == "hw":
        from targets.serial import SerialTarget
        target = SerialTarget()
    elif substrate == "rpi":
        from targets.rpi import RpiTarget
        target = RpiTarget()
    elif substrate == "linux":
        from targets.linux import LinuxTarget
        target = LinuxTarget()
    else:
        from targets.emu import EmuTarget
        target = EmuTarget()

    made = []

    def _make(example, host_role="mcu_host", transport="sdio", timeout="60s",
              wake=False, overlay=None, cp_overlay=None, cp_ota_overlay=None,
              hostfwd=(), cp_app_to_host=None, pre_launch=None):
        if transport not in target.transports:
            pytest.skip(f"{substrate} bench provides no transport={transport}")
        # pre_launch: a callback run AFTER firmware is built but BEFORE the DUTs
        # launch. Lets a test stand up an external peer (e.g. Bumble) whose connect
        # deadline must cover only device bring-up, not the (possibly minutes-long,
        # cold-cache) build. Only the emu target honours it; passed only when set so
        # other targets' signatures are untouched.
        extra = {"pre_launch": pre_launch} if pre_launch is not None else {}
        b = target.make(
            BenchSpec(example=example, host_role=host_role, transport=transport,
                      wake=wake, timeout=timeout, extra_ovl=tuple(overlay or ()),
                      cp_extra_ovl=tuple(cp_overlay or ()),
                      cp_ota_ovl=tuple(cp_ota_overlay or ()),
                      hostfwd=tuple(hostfwd), cp_app_to_host=cp_app_to_host),
            worker_id=worker_id, lab_tmp=lab_tmp, **extra)
        made.append(b)
        out = {"host": b.host, "cp": b.cp, "caps": b.caps}
        if b.net is not None:
            out["net"] = b.net
        return out

    yield _make
    for b in made:
        b.down()
