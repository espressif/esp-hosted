#!/usr/bin/env python3
"""esplab runner — entity-driven ESP-Hosted test engine (build + emulate + assert).

Everything is an entity with properties (see tests.json): a suite, scenario, or
case is the same shape — behaviour comes from props, not type. A *scenario* is
one emulator run: build the coprocessor (C6) + host (P4) firmware, launch the
bench (the emulator pair over the SDIO bridge with the setup's reset/wake GPIOs
and networking), drive any stimulus (console injections on host and/or cp), and
capture both serial streams. Its *cases* are named assertions on that capture:
each case must see every `expect` substring and must NOT see any `absent` (crash)
marker within `timeout_s`. positive vs negative is just the `kind` property — a
negative case simply expects the graceful-failure line. `support` gates
execution: supported runs; unsupported/tbd are catalogued but skipped.

Loosely coupled by design: standalone CLI *and* importable by the esplab HTTP
server, no UI dependency either way.

  python3 runner.py list                       # the suite/scenario/case tree
  python3 runner.py run wifi_sta               # a suite, scenario, or case id
  python3 runner.py run sta_connect sta_no_ap  # several
  python3 runner.py run --all                  # every supported scenario -> coverage.md
  python3 runner.py run --all --include-unsupported
  python3 runner.py run sta_connect --rebuild   # force a clean rebuild (else reuse cache)

Env: ESP_EMU_BIN (esp-emu path), IDF_PATH (ESP-IDF path).
"""
import hashlib
import json
import os
import pathlib
import re
import shutil
import socket
import subprocess
import threading
import time

HERE = pathlib.Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
CATALOG = HERE / 'tests.json'
WORK = REPO_ROOT / 'tests' / '.work'  # scratch build workspace (shared with the
# pytest framework's build_fw; the sandbox-bindable root). Examples stay read-only.

SLAVE_WARMUP_S = 3        # let the C6 slave bind its socket before the host connects
LOG_TAIL = 6000           # max lines kept in memory per stream (full logs go to disk)
CP_SDIO_SYM = 'CONFIG_EH_TRANSPORT_CP_SDIO'
HOST_SDIO_SYM = 'CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO'

# Dependency resolution via eh.py (eh.conf override -> .deps -> $IDF_PATH for
# IDF; eh.conf -> .deps for emu). No hardcoded paths: use ESP_EMU_BIN / IDF_PATH,
# ./install.sh (--with-emu for esp-emu), or `eh.py set-idf-path` / `set-esp-emu`.
try:
    import importlib.util as _ilu
    _spec = _ilu.spec_from_file_location('eh', REPO_ROOT / 'tools' / 'eh.py')
    _eh = _ilu.module_from_spec(_spec); _spec.loader.exec_module(_eh)
except Exception:
    _eh = None


def emu_bin() -> pathlib.Path:
    env = os.environ.get('ESP_EMU_BIN')
    if env:
        return pathlib.Path(env)
    if _eh is not None:
        d = _eh.resolve_emu_dir()
        if d is not None:
            return _eh.emu_binary(d)
    raise RuntimeError("esp-emu not found: run ./install.sh --with-emu, set "
                       "ESP_EMU_BIN, or `eh.py set-esp-emu <dir>`")


def idf_path() -> pathlib.Path:
    env = os.environ.get('IDF_PATH')
    if env:
        return pathlib.Path(env)
    if _eh is not None:
        p = _eh.resolve_idf()
        if p is not None:
            return p
    raise RuntimeError("ESP-IDF not found: run ./install.sh, set IDF_PATH, "
                       "or `eh.py set-idf-path <dir>`")


def _bwrap_works() -> bool:
    """True if bubblewrap is installed AND user namespaces actually work here."""
    if not shutil.which('bwrap'):
        return False
    try:
        r = subprocess.run(['bwrap', '--ro-bind', '/', '/', '--dev-bind', '/dev', '/dev',
                            '--proc', '/proc', 'true'], capture_output=True, timeout=10)
        return r.returncode == 0
    except Exception:
        return False


def _absolutize_overrides(src: pathlib.Path, proj: pathlib.Path) -> None:
    """Rewrite relative `override_path:` in the scratch copy's idf_component.yml
    files to absolute paths resolved against the ORIGINAL example location, so the
    copy still points at the real in-repo components (e.g. esp_hosted at repo root)
    instead of a path that only made sense from the example's own depth."""
    def fix(m):
        val = m.group(3).strip()
        if not val or val.startswith('/'):
            return m.group(0)
        return f'{m.group(1)}"{(yml_src.parent / val).resolve()}"'
    for yml_src in src.rglob('idf_component.yml'):
        yml_dst = proj / yml_src.relative_to(src)
        if not yml_dst.is_file():
            continue
        text = yml_dst.read_text()
        new = re.sub(r'(override_path:\s*)([\'"]?)([^\'"\n]+)\2', fix, text)
        if new != text:
            yml_dst.write_text(new)


# ── catalog (entity tree) ────────────────────────────────────────────────────

def load_catalog() -> dict:
    return json.loads(CATALOG.read_text())


def scenarios() -> list:
    """Flatten suites→scenarios, injecting suite id/label and defaults onto each."""
    cat = load_catalog()
    out = []
    for suite in cat['suites']:
        for sc in suite['scenarios']:
            sc = dict(sc)
            sc['suite'] = suite['id']
            sc['suite_label'] = suite['label']
            out.append(sc)
    return out


def find_scenarios(token: str) -> list:
    """Resolve an id token to scenarios: a suite id, a scenario id, or a case id."""
    cat = load_catalog()
    # suite id → all its scenarios
    for suite in cat['suites']:
        if suite['id'] == token:
            return [s for s in scenarios() if s['suite'] == token]
    # scenario id
    hits = [s for s in scenarios() if s['id'] == token]
    if hits:
        return hits
    # case id → the owning scenario
    for s in scenarios():
        if any(c['id'] == token for c in s.get('cases', [])):
            return [s]
    return []


def preflight() -> list:
    problems = []
    if not emu_bin().is_file():
        problems.append(f'emulator binary not found at {emu_bin()} (set ESP_EMU_BIN or build esp-emu)')
    if not (idf_path() / 'export.sh').is_file():
        problems.append(f'ESP-IDF export.sh not found under {idf_path()} (set IDF_PATH)')
    return problems


# ── observable run state ─────────────────────────────────────────────────────

class Stream:
    """Bounded, thread-safe line buffer that also tees to a file."""
    def __init__(self, path: pathlib.Path | None = None):
        self.lines: list[str] = []
        self._lock = threading.Lock()
        self._fh = path.open('w') if path else None

    def write(self, line: str):
        with self._lock:
            self.lines.append(line)
            if len(self.lines) > LOG_TAIL:
                del self.lines[0]
            if self._fh:
                self._fh.write(line + '\n'); self._fh.flush()

    def text(self) -> str:
        with self._lock:
            return '\n'.join(self.lines)

    def close(self):
        if self._fh:
            self._fh.close()


class ScenarioRun:
    """Run state for one scenario; serialisable for the HTTP layer."""
    def __init__(self, sc: dict, defaults: dict, logdir: pathlib.Path):
        self.sc = sc
        self.id = sc['id']
        self.support = sc.get('support', 'supported')
        self.status = 'pending'      # pending|building|emulating|pass|fail|skip
        self.detail = ''
        self.started = None
        self.ended = None
        self.phases = {k: {'status': 'pending', 'detail': ''} for k in ('build_cp', 'build_host', 'emulate')}
        self.cases = [
            {'id': c['id'], 'label': c['label'], 'kind': c.get('kind', sc.get('kind', 'positive')),
             'status': 'pending', 'detail': ''}
            for c in sc.get('cases', [])
        ]
        self.build_log = Stream(logdir / f'{self.id}.build.log')
        self.host_log = Stream(logdir / f'{self.id}.host.log')
        self.cp_log = Stream(logdir / f'{self.id}.cp.log')
        self.procs: list = []      # live bench processes (filled at launch, reaped at teardown)
        self.threads: list = []    # log-pump threads
        self.sock = None           # bridge socket path for this run

    def to_dict(self) -> dict:
        return {
            'id': self.id, 'suite': self.sc['suite'], 'label': self.sc['label'],
            'kind': self.sc.get('kind', 'positive'), 'category': self.sc.get('category', 'functional'),
            'support': self.support, 'status': self.status, 'detail': self.detail,
            'started': self.started, 'ended': self.ended,
            'dur_s': round(self.ended - self.started, 1) if self.started and self.ended else None,
            'phases': self.phases, 'cases': self.cases,
            'logs': {'build': self.build_log.text(), 'host': self.host_log.text(), 'cp': self.cp_log.text()},
        }

    def close(self):
        for s in (self.build_log, self.host_log, self.cp_log):
            s.close()


# ── the engine ───────────────────────────────────────────────────────────────

class Runner:
    def __init__(self, rebuild=False, logroot: pathlib.Path | None = None, stop_event=None, sandbox=''):
        # rebuild=False (default) reuses a prior build for the same config — the big
        # speedup; rebuild=True forces a clean build (use when example source changed).
        self.rebuild = rebuild
        self.builds: dict[str, dict] = {}   # config-key → build result, built once per run
        self.cat = load_catalog()
        self.defaults = self.cat.get('defaults', {})
        # One run-reports root shared with the pytest framework: tests/.work/runs/<ts>/.
        self.logroot = logroot or (REPO_ROOT / 'tests' / '.work' / 'runs' / time.strftime('%Y%m%d-%H%M%S'))
        self.logroot.mkdir(parents=True, exist_ok=True)
        self.stop_event = stop_event
        # Optional sandbox: 'bwrap' isolates build/emulate in a bubblewrap namespace
        # (private /tmp, repo read-only except the scratch workspace) so the main
        # system stays untouched. Off by default; degrades gracefully to native.
        self.sandbox = sandbox if sandbox and sandbox != 'none' else ''
        if self.sandbox == 'bwrap' and not _bwrap_works():
            self._log('  ! sandbox=bwrap requested but bubblewrap is unavailable/blocked — running native')
            self.sandbox = ''

    def stopped(self) -> bool:
        return bool(self.stop_event and self.stop_event.is_set())

    def _bwrap_prefix(self, rw_paths: list, net: bool) -> list:
        """Build a bubblewrap argv prefix: whole root read-only, private /tmp + /dev
        + /proc, with `rw_paths` re-bound read-write (later binds override the RO
        root). `net=False` drops the network namespace entirely (the emulator's
        --net user is in-process, so emulation needs no host network)."""
        if not self.sandbox:
            return []
        argv = ['bwrap', '--die-with-parent', '--unshare-pid',
                '--ro-bind', '/', '/', '--dev-bind', '/dev', '/dev',
                '--proc', '/proc', '--tmpfs', '/tmp']
        for p in rw_paths:
            sp = str(p)
            if os.path.exists(sp):
                argv += ['--bind', sp, sp]
        if not net:
            argv += ['--unshare-net']
        return argv

    # -- build one side (with optional sdkconfig overlay) ---------------------

    def _result(self, proj: pathlib.Path, sdio_symbol: str) -> dict:
        merged = proj / 'build' / 'merged_flash.bin'
        elfs = [p for p in (proj / 'build').glob('*.elf')]
        if not merged.is_file() or not elfs:
            return {'ok': False, 'err': 'build produced no merged_flash.bin / app elf'}
        sdk = (proj / 'sdkconfig').read_text() if (proj / 'sdkconfig').is_file() else ''
        if sdio_symbol and f'{sdio_symbol}=y' not in sdk:
            return {'ok': False, 'err': f'{sdio_symbol} not set — bench bridges SDIO only'}
        app_bin = pathlib.Path(str(elfs[0])).with_suffix('.bin')
        return {'ok': True, 'merged': str(merged), 'elf': str(elfs[0]),
                'app_bin': str(app_bin) if app_bin.is_file() else '', 'err': ''}

    def build_side(self, ex_dir: str, target: str, overlay: list, sdio_symbol: str, log: Stream, inject=None) -> dict:
        src = REPO_ROOT / ex_dir
        if not src.is_dir():
            return {'ok': False, 'err': f'missing dir {ex_dir}'}
        overlay = overlay or []
        # Build identity = dir + target + overlay + injected files. Identical config
        # builds ONCE and is reused everywhere (the shared CP across a suite's
        # scenarios, and across re-runs). A changed overlay → a new key → a rebuild.
        key = hashlib.sha1(('|'.join([ex_dir, target, *sorted(overlay), *(d for _, d in (inject or []))])).encode()).hexdigest()[:8]
        if key in self.builds:
            log.write(f'↺ reusing build (cached this run): {ex_dir} [{target}] — no rebuild')
            return self.builds[key]
        proj = WORK / (ex_dir.replace('/', '__') + '__' + key)
        if self.rebuild and proj.exists():
            shutil.rmtree(proj, ignore_errors=True)
        # reuse a prior valid build for this exact config — no rebuild at all
        if not self.rebuild and (proj / 'build' / 'merged_flash.bin').is_file():
            res = self._result(proj, sdio_symbol)
            if res['ok']:
                log.write(f'↺ reusing cached build {proj.name}')
                self.builds[key] = res
                return res
        # The example is a read-only input: build in an isolated scratch copy so the
        # example tree is never written to (no build/, sdkconfig, overlay, deps, bin).
        if not proj.exists():
            proj.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(src, proj, ignore=shutil.ignore_patterns(
                'build', 'managed_components', 'dependencies.lock',
                'sdkconfig', 'sdkconfig.old', '*.bin'))
            _absolutize_overrides(src, proj)
        for asrc, rel in (inject or []):
            if not os.path.isfile(asrc):
                return {'ok': False, 'err': f'inject source missing: {asrc}'}
            dst = proj / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(asrc, dst)
        if overlay:
            (proj / '.esplab_overlay.defaults').write_text('\n'.join(overlay) + '\n')
            sdkdefaults = 'sdkconfig.defaults;.esplab_overlay.defaults'
        else:
            (proj / '.esplab_overlay.defaults').unlink(missing_ok=True)
            sdkdefaults = 'sdkconfig.defaults'
        script = (
            f'. "{idf_path()}/export.sh" >/dev/null 2>&1 || exit 90; '
            f'cd "{proj}" || exit 91; '
            f'idf.py -D SDKCONFIG_DEFAULTS="{sdkdefaults}" set-target {target} build && '
            f'idf.py merge-bin -o merged_flash.bin'
        )
        home = pathlib.Path.home()
        prefix = self._bwrap_prefix([WORK, home / '.espressif', home / '.cache'], net=True)
        rc = self._stream(prefix + ['bash', '-lc', script], log)
        if rc != 0:
            return {'ok': False, 'err': self._first_error(log)}
        res = self._result(proj, sdio_symbol)
        if res['ok']:
            self.builds[key] = res
        return res

    # -- lifecycle: suite + scenario setup/teardown (generic, reused) ---------
    # The bench (a pair of emulator processes + a bridge socket) is the resource
    # that must be brought up clean and torn down completely. These hooks are the
    # common, generic fixture code shared by every scenario in every suite.

    def suite_setup(self, suite_id: str, label: str = '') -> None:
        """Once before a suite's scenarios run. No global process kills (that would
        disturb a concurrently-running sweep) — each scenario owns its own bench."""
        self._log(f'\n■ suite {label or suite_id} — setup')

    def suite_teardown(self, suite_id: str) -> None:
        """Once after a suite's scenarios. Each scenario already reaps its own bench
        in scenario_teardown; this is the symmetric close-out / extension point."""
        self._log(f'■ suite {suite_id} — teardown')

    def scenario_setup(self, sc: dict, run: ScenarioRun) -> None:
        """Before each scenario: a fresh, isolated starting point. CODE+CONFIG come
        from build_side (a per-config scratch copy of the example, reused across
        scenarios with the same config); here we reset runtime state and assign a
        unique bridge socket, clearing any stale socket file so no run bleeds in."""
        run.started = time.time()
        run.procs.clear(); run.threads.clear()
        # When sandboxed, /tmp is a private tmpfs per process, so the bridge socket
        # must live in the shared, bound scratch workspace instead; native runs keep
        # it in /tmp. (Both stay well under the 108-char AF_UNIX path limit.)
        if self.sandbox:
            WORK.mkdir(parents=True, exist_ok=True)
            run.sock = str(WORK / f'eh_{run.id}_{os.getpid()}.sock')
        else:
            run.sock = f'/tmp/esplab_{run.id}_{os.getpid()}.sock'
        try:
            os.unlink(run.sock)
        except OSError:
            pass

    def scenario_teardown(self, sc: dict, run: ScenarioRun) -> None:
        """After each scenario, ALWAYS (even on a build/emulate error): stop the
        bench, drop the socket, stamp the end time. Leaves nothing behind."""
        self._stop_bench(run)
        if run.ended is None:
            run.ended = time.time()

    def _stop_bench(self, run: ScenarioRun) -> None:
        """Reap the bench — terminate both emulator processes, join the log pumps,
        remove the socket. Idempotent: safe to call from emulate() and again from
        scenario_teardown()."""
        for p in run.procs:
            self._kill(p)
        for t in run.threads:
            try:
                t.join(2)
            except RuntimeError:
                pass
        run.procs.clear(); run.threads.clear()
        if run.sock:
            try:
                os.unlink(run.sock)
            except OSError:
                pass

    # -- launch the bench, drive stimulus, evaluate cases ---------------------

    def emulate(self, sc: dict, setup: dict, cp: dict, host: dict, run: ScenarioRun) -> None:
        timeout_s = max([c.get('assert', {}).get('timeout_s', self.defaults.get('timeout_s', 150))
                         for c in sc.get('cases', [])] + [self.defaults.get('timeout_s', 150)])
        self._launch_bench(sc, setup, cp, host, run, timeout_s)
        self._poll_bench(sc, run, timeout_s)
        self._stop_bench(run)          # stop the readers so the logs are final before asserting
        self._evaluate(sc, run)

    def _launch_bench(self, sc, setup, cp, host, run, timeout_s) -> None:
        stim = sc.get('stimulus', {})
        net_stim = stim.get('net', [])
        # the bridge socket lives in the scratch workspace (a bound-RW dir) so both
        # sandboxed instances share it; --unshare-net is fine (Unix socket ≠ network)
        host_prefix = self._bwrap_prefix([WORK], net=False)
        # the CP owns --net user; an external wake-packet stimulus reaches it via a
        # hostfwd loopback port, so the CP must share the host net namespace.
        cp_prefix = self._bwrap_prefix([WORK], net=bool(net_stim))
        # ── C6 coprocessor (slave) — starts first, registered for teardown ──
        cp_argv = [str(emu_bin()), '--chip', setup['cp_target'],
                   '--firmware', cp['merged'], '--elf', cp['elf'],
                   '--hosted', f'bridge:slave:{run.sock}']
        net_spec = setup.get('net')
        if net_spec:
            for ns in net_stim:  # QEMU-style hostfwd: external loopback port → guest dst port
                proto, guest_port = ns.get('proto', 'udp'), int(ns['host_port'])
                ext_port = int(ns.get('ext_port', guest_port))  # ext stays unprivileged
                net_spec += f',hostfwd={proto}:127.0.0.1:{ext_port}-:{guest_port}'
            cp_argv += ['--net', net_spec]
        if setup.get('wake_gpio') is not None:
            cp_argv += ['--hosted-wake-gpio', str(setup['wake_gpio'])]
        cp_argv += self._inject_args(stim.get('cp', []))
        slave = subprocess.Popen(cp_prefix + cp_argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        run.procs.append(slave); run.threads.append(self._tee(slave, run.cp_log))
        time.sleep(SLAVE_WARMUP_S)
        # ── P4 host ──
        host_argv = [str(emu_bin()), '--chip', setup['host_target'],
                     '--firmware', host['merged'], '--elf', host['elf'],
                     '--hosted', f'bridge:host:{run.sock}']
        if setup.get('reset_gpio') is not None:
            host_argv += ['--hosted-reset-gpio', str(setup['reset_gpio'])]
        host_argv += ['--timeout', f'{timeout_s}s']
        host_argv += self._inject_args(stim.get('host', []))
        host_p = subprocess.Popen(host_prefix + host_argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        run.procs.append(host_p); run.threads.append(self._tee(host_p, run.host_log))

    def _poll_bench(self, sc, run, timeout_s) -> None:
        host_p = run.procs[-1]
        wants = set()       # all `expect` substrings across cases
        absent = set(self.defaults.get('absent', []))   # fault/forbidden markers
        for c in sc.get('cases', []):
            wants.update(c.get('assert', {}).get('expect', []))
            absent.update(c.get('assert', {}).get('absent', []))
        net_stim = sc.get('stimulus', {}).get('net', [])
        # times each stimulus has fired. one-shot fires once on first sighting;
        # repeat fires once per NEW occurrence of its marker (i.e. once per cycle).
        fired = [0] * len(net_stim)
        # once every expect is seen, keep watching settle_s for a late fault before
        # declaring success — a flow that completes then panics (e.g. a post-wake
        # crash) must not pass just because the success markers arrived first.
        settle_s = sc.get('settle_s', self.defaults.get('settle_s', 4))
        deadline = time.time() + timeout_s + 20
        satisfied_at = None
        while host_p.poll() is None and time.time() < deadline and not self.stopped():
            cp_text, host_text = run.cp_log.text(), run.host_log.text()
            combined = host_text + '\n' + cp_text
            for i, ns in enumerate(net_stim):
                marker = ns.get('after_cp') or ns.get('after_host')
                if not marker:
                    continue
                hay = cp_text if 'after_cp' in ns else host_text
                target = hay.count(marker) if ns.get('repeat') else (1 if marker in hay else 0)
                while fired[i] < target:
                    self._send_net_packet(ns, run.cp_log)   # may itself be a burst
                    fired[i] += 1
            if any(a in combined for a in absent):
                break                       # fault marker → stop now; _evaluate fails it
            if wants and all(w in combined for w in wants):
                if satisfied_at is None:
                    satisfied_at = time.time()
                elif time.time() - satisfied_at >= settle_s:
                    break
            time.sleep(0.3)

    @staticmethod
    def _send_net_packet(ns: dict, log) -> None:
        """Fire the external wake packet(s) at a host-owned port via the CP's
        user-net hostfwd — the bench analogue of the example's
        host_wakeup_demo_using_test_packet helper. Targets a host-reserved
        priority port (e.g. TCP 22) so network-split forwards it to the host and
        the coprocessor drives the host-wakeup GPIO.

        `burst` (default 1) sends N packets `burst_gap_ms` apart in one call, so a
        single trigger can span a timing window (e.g. straddle the CP's light-sleep
        teardown to race a wake against it) instead of a lone packet."""
        proto = ns.get('proto', 'udp')
        guest_port = int(ns['host_port'])
        ext_port = int(ns.get('ext_port', guest_port))
        payload = ns.get('payload', 'Hello, UDP!').encode()
        burst = max(1, int(ns.get('burst', 1)))
        gap = float(ns.get('burst_gap_ms', 0)) / 1000.0
        sent = 0
        for k in range(burst):
            try:
                if proto == 'udp':
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.sendto(payload, ('127.0.0.1', ext_port))
                else:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(2); s.connect(('127.0.0.1', ext_port)); s.sendall(payload)
                s.close()
                sent += 1
            except OSError:
                # host asleep/rebooting → no SYN-ACK; the routed SYN already
                # reached the CP and triggered the wake decision.
                pass
            if gap and k < burst - 1:
                time.sleep(gap)
        if log:
            tail = f' x{burst} ({sent} delivered)' if burst > 1 else ''
            log.write(f'[esplab] external {proto} wake packet{tail} → 127.0.0.1:{ext_port} (guest :{guest_port})')

    def _evaluate(self, sc, run) -> None:
        combined = run.host_log.text() + '\n' + run.cp_log.text()
        absent_default = self.defaults.get('absent', [])
        npass = 0
        for cr, cdef in zip(run.cases, sc.get('cases', [])):
            a = cdef.get('assert', {})
            expect = a.get('expect', [])
            absent = a.get('absent', absent_default)
            missing = [e for e in expect if e not in combined]
            crash = next((x for x in absent if x in combined), None)
            if crash:
                cr['status'], cr['detail'] = 'fail', f'crash marker seen: "{crash}"'
            elif missing:
                cr['status'], cr['detail'] = 'fail', f'missing: {missing[:2]}'
            else:
                cr['status'], cr['detail'] = 'pass', '; '.join(expect)[:80]
                npass += 1
        run.status = 'pass' if npass == len(run.cases) and run.cases else 'fail'
        if run.status != 'pass':
            run.detail = f'{npass}/{len(run.cases)} cases passed'
        run.phases['emulate'] = {'status': run.status, 'detail': run.detail}

    # -- one full scenario ----------------------------------------------------

    def run_scenario(self, sc: dict, run: ScenarioRun, force_unsupported=False) -> None:
        if run.support != 'supported' and not force_unsupported:
            run.started = run.started or time.time()
            run.status, run.detail = 'skip', run.support
            for cr in run.cases:
                cr['status'], cr['detail'] = 'skip', run.support
            run.ended = time.time()
            return
        self.scenario_setup(sc, run)
        try:
            setup = self.cat['setups'][sc['setup']]
            b = sc['build']
            # CP (esp32c6)
            run.status = 'building'; run.phases['build_cp']['status'] = 'running'
            cp = self.build_side(b['cp_dir'], setup['cp_target'], b.get('cp_overlay', []), CP_SDIO_SYM, run.build_log)
            run.phases['build_cp'] = {'status': 'pass' if cp['ok'] else 'fail', 'detail': cp.get('err', '')}
            if not cp['ok']:
                self._fail_all(run, f'cp build: {cp["err"]}'); return
            # Host (esp32p4) — optionally embed the freshly-built CP app image into the
            # host build (coprocessor-OTA: the host streams this image to the CP).
            inject = []
            if b.get('embed_cp_app'):
                if not cp.get('app_bin'):
                    self._fail_all(run, 'embed_cp_app: CP build produced no app .bin'); return
                inject.append((cp['app_bin'], b['embed_cp_app']))
            run.phases['build_host']['status'] = 'running'
            host = self.build_side(b['host_dir'], setup['host_target'], b.get('host_overlay', []), HOST_SDIO_SYM, run.build_log, inject=inject)
            run.phases['build_host'] = {'status': 'pass' if host['ok'] else 'fail', 'detail': host.get('err', '')}
            if not host['ok']:
                self._fail_all(run, f'host build: {host["err"]}'); return
            # Emulate
            run.status = 'emulating'; run.phases['emulate']['status'] = 'running'
            self.emulate(sc, setup, cp, host, run)
        finally:
            self.scenario_teardown(sc, run)

    def _fail_all(self, run: ScenarioRun, detail: str):
        run.status, run.detail, run.ended = 'fail', detail, time.time()
        for cr in run.cases:
            if cr['status'] == 'pending':
                cr['status'], cr['detail'] = 'fail', detail

    # -- subprocess helpers ---------------------------------------------------

    @staticmethod
    def _log(msg: str) -> None:
        print(msg, flush=True)

    @staticmethod
    def _inject_args(rules: list) -> list:
        argv = []
        for r in rules:
            argv += ['--inject-on', r['on'], '--inject', r['inject']]
        return argv

    def _stream(self, argv: list, stream: Stream, timeout=1800) -> int:
        p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in p.stdout:
            stream.write(line.rstrip('\n'))
        try:
            return p.wait(timeout)
        except subprocess.TimeoutExpired:
            self._kill(p); return 124

    def _tee(self, proc: subprocess.Popen, stream: Stream) -> threading.Thread:
        def pump():
            for line in proc.stdout:
                stream.write(line.rstrip('\n'))
        th = threading.Thread(target=pump, daemon=True); th.start(); return th

    @staticmethod
    def _kill(p: subprocess.Popen):
        if p.poll() is None:
            p.terminate()
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()

    @staticmethod
    def _first_error(stream: Stream) -> str:
        # strong, unambiguous failure markers first (avoid false hits like
        # "error.c.obj" — a filename containing the word "error")
        strong = re.compile(r'fatal error:|CMake Error|FATAL_ERROR|ninja: build stopped|'
                            r'Compilation failed because|undefined reference|: error:|'
                            r'No such file or directory|Error \d+', re.I)
        for ln in stream.lines:
            if strong.search(ln):
                return ln.strip()[:200]
        for ln in stream.lines:               # fallback: a FAILED: target line
            if ln.startswith('FAILED:'):
                return ln.strip()[:200]
        return 'build failed (see log)'


# ── run jobs (HTTP-facing) ───────────────────────────────────────────────────

class RunJob:
    def __init__(self, scs: list, rebuild=False, include_unsupported=False, sandbox=''):
        self.id = time.strftime('run-%Y%m%d-%H%M%S')
        self._stop = threading.Event()
        self.runner = Runner(rebuild=rebuild, stop_event=self._stop, sandbox=sandbox)
        self.include_unsupported = include_unsupported
        self.runs = [ScenarioRun(sc, self.runner.defaults, self.runner.logroot) for sc in scs]
        self.status = 'queued'
        self.started = None
        self.ended = None
        self._thread = threading.Thread(target=self._work, daemon=True)

    def start(self):
        self.status = 'running'; self.started = time.time(); self._thread.start()

    def stop(self):
        self._stop.set()

    def _work(self):
        cur_suite = None
        for run in self.runs:
            if self._stop.is_set():
                run.status = 'skip'; run.detail = 'stopped by user'; continue
            sid = run.sc.get('suite')
            if sid != cur_suite:
                if cur_suite is not None:
                    self.runner.suite_teardown(cur_suite)
                self.runner.suite_setup(sid, run.sc.get('suite_label', ''))
                cur_suite = sid
            self.runner.run_scenario(run.sc, run, force_unsupported=self.include_unsupported)
            run.close()
        if cur_suite is not None:
            self.runner.suite_teardown(cur_suite)
        self.ended = time.time()
        self.status = 'stopped' if self._stop.is_set() else 'done'

    def to_dict(self) -> dict:
        runs = [r.to_dict() for r in self.runs]
        tally = {k: sum(1 for r in runs if r['status'] == k) for k in ('pass', 'fail', 'skip')}
        return {'id': self.id, 'status': self.status, 'started': self.started, 'ended': self.ended,
                'logroot': str(self.runner.logroot),
                'summary': {**tally, 'total': len(runs)}, 'scenarios': runs}


RUNS: dict[str, RunJob] = {}


def start_run(ids=None, rebuild=False, include_unsupported=False, sandbox='') -> dict:
    if preflight():
        return {'error': '; '.join(preflight())}
    if ids:
        scs, seen = [], set()
        for tok in ids:
            for s in find_scenarios(tok):
                if s['id'] not in seen:
                    seen.add(s['id']); scs.append(s)
        if not scs:
            return {'error': f'no suite/scenario/case matched: {ids}'}
    else:
        scs = scenarios()
    job = RunJob(scs, rebuild=rebuild, include_unsupported=include_unsupported, sandbox=sandbox)
    RUNS[job.id] = job
    job.start()
    return {'runId': job.id}


def get_run(run_id: str) -> dict | None:
    job = RUNS.get(run_id)
    return job.to_dict() if job else None


def stop_run(run_id: str) -> bool:
    job = RUNS.get(run_id)
    if job:
        job.stop(); return True
    return False


# ── CLI ───────────────────────────────────────────────────────────────────────

def _cli():
    import argparse
    ap = argparse.ArgumentParser(prog='runner', description='ESP-Hosted C6+P4 entity test engine')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list')
    cp = sub.add_parser('catalog')
    cp.add_argument('-o', '--out', default=str(HERE / 'TEST_CATALOG.md'))
    rp = sub.add_parser('run')
    rp.add_argument('ids', nargs='*')
    rp.add_argument('--all', action='store_true')
    rp.add_argument('--include-unsupported', action='store_true')
    rp.add_argument('--rebuild', action='store_true', help='force a clean rebuild (default reuses cached builds per config)')
    rp.add_argument('--sandbox', choices=['none', 'bwrap'], default='none',
                    help='isolate build+emulate in a bubblewrap namespace (main system untouched)')
    args = ap.parse_args()

    if args.cmd == 'list':
        cat = load_catalog()
        mark = {'supported': '✓'}
        for suite in cat['suites']:
            print(f'\n■ {suite["id"]:<16} {suite["label"]}')
            for sc in suite['scenarios']:
                m = mark.get(sc.get('support', 'supported'), '·')
                kind = 'neg' if sc.get('kind') == 'negative' else 'pos'
                print(f'   {m} {sc["id"]:<26} [{kind}/{sc.get("category","functional")}] {sc.get("support")}')
        return

    if args.cmd == 'catalog':
        pathlib.Path(args.out).write_text(catalog_md())
        print(f'wrote {args.out}')
        return

    problems = preflight()
    if problems:
        print('preflight failed:'); [print('  -', p) for p in problems]; raise SystemExit(2)

    if args.all:
        scs = scenarios()
    else:
        scs, seen = [], set()
        for tok in args.ids:
            hits = find_scenarios(tok)
            if not hits:
                raise SystemExit(f'no suite/scenario/case matched: {tok}')
            for s in hits:
                if s['id'] not in seen:
                    seen.add(s['id']); scs.append(s)

    runner = Runner(rebuild=args.rebuild, sandbox=args.sandbox)
    print(f'logs: {runner.logroot}{"  [sandbox: bwrap]" if runner.sandbox else ""}')
    results = []
    cur_suite = None
    for sc in scs:
        if sc.get('suite') != cur_suite:
            if cur_suite is not None:
                runner.suite_teardown(cur_suite)
            runner.suite_setup(sc.get('suite'), sc.get('suite_label', ''))
            cur_suite = sc.get('suite')
        run = ScenarioRun(sc, runner.defaults, runner.logroot)
        print(f'\n=== [{sc["suite"]}] {sc["id"]} ({sc.get("support")}) ===', flush=True)
        runner.run_scenario(sc, run, force_unsupported=args.include_unsupported)
        run.close()
        icon = {'pass': 'PASS', 'fail': 'FAIL', 'skip': 'SKIP'}.get(run.status, run.status.upper())
        for cr in run.cases:
            ci = {'pass': '✓', 'fail': '✗', 'skip': '·'}.get(cr['status'], '?')
            print(f'    {ci} {cr["id"]:<22} {cr.get("detail","")[:70]}')
        print(f'--- {sc["id"]}: {icon}  {run.detail}', flush=True)
        results.append(run)
    if cur_suite is not None:
        runner.suite_teardown(cur_suite)

    print('\n================ summary ================')
    for r in results:
        print(f'  {r.status.upper():<5} [{r.sc["suite"]}] {r.id}')
    npass = sum(1 for r in results if r.status == 'pass')
    print(f'  {npass}/{len(results)} scenarios passed')
    if args.all:
        print(f'coverage report: {write_coverage(results, runner.logroot)}')


def catalog_md() -> str:
    """Render the whole entity catalogue (tests.json) as a Markdown report with
    summary, mermaid hierarchy/status diagrams, and per-suite case tables. Supported
    scenarios are the verified-green set (every supported scenario passes the bench);
    tbd/unsupported carry their reason. Single source of truth — never drifts."""
    cat = load_catalog()
    sc_all = scenarios()
    def bucket(s): return s.get('support', 'supported').split(':')[0]
    def reason(s): return (s.get('support', 'supported').split(':', 1) + [''])[1]
    n_sup = sum(1 for s in sc_all if bucket(s) == 'supported')
    n_tbd = sum(1 for s in sc_all if bucket(s) == 'tbd')
    n_uns = sum(1 for s in sc_all if bucket(s) == 'unsupported')
    n_case = sum(len(s.get('cases', [])) for s in sc_all)
    badge = {'supported': '✅ pass', 'tbd': '🟡 tbd', 'unsupported': '⚪ unsupported'}
    L = []
    L.append('# ESP-Hosted Test Catalogue\n')
    L.append(f'Bench: **C6 {cat["targets"]["cp"]} ⇄ P4 {cat["targets"]["host"]}** over SDIO '
             f'(esp-emulator). Generated by `runner.py catalog` from `tests.json` — single '
             f'source of truth for the CLI and the UI.\n')
    L.append('## Summary\n')
    L.append('| Status | Scenarios | Meaning |')
    L.append('|---|---:|---|')
    L.append(f'| ✅ supported (pass) | {n_sup} | runs on the bench; passing in the last verified sweep (0 failing) |')
    L.append(f'| 🟡 tbd | {n_tbd} | catalogued; needs a bench/harness/emulator capability first (reason given) |')
    L.append(f'| ⚪ unsupported | {n_uns} | needs external HW/infra the emulator inherently lacks (reason given) |')
    L.append(f'| **total** | **{len(sc_all)}** | across **{len(cat["suites"])}** suites · **{n_case}** assertion cases |\n')
    # entity model
    L.append('## Entity model\n')
    L.append('```mermaid\nflowchart TD\n'
             '  C["Catalogue"] --> SE["setups: cp/host target, bridge, reset/wake GPIO"]\n'
             '  C --> SU["Suites"]\n  SU --> SC["Scenarios — one emulator run"]\n'
             '  SC --> CA["Cases — expect present, absent not, within timeout"]\n'
             '  SC -. props .-> P["kind pos/neg · category · support supported/tbd/unsupported"]\n```\n')
    # status pie
    L.append('## Status at a glance\n')
    L.append(f'```mermaid\npie showData title Scenario support ({len(sc_all)} total)\n'
             f'  "supported / pass" : {n_sup}\n  "tbd" : {n_tbd}\n  "unsupported" : {n_uns}\n```\n')
    # suite map
    L.append('## Suites\n')
    lines = ['```mermaid', 'flowchart LR', '  root([Catalogue])']
    for su in cat['suites']:
        sc = su['scenarios']
        sup = sum(1 for s in sc if s.get('support', 'supported') == 'supported')
        lines.append(f'  root --> {su["id"]}["{su["label"]}<br/>{sup}/{len(sc)} green"]')
    lines.append('```')
    L.append('\n'.join(lines) + '\n')
    # per-suite tables
    L.append('## Scenarios & cases\n')
    for su in cat['suites']:
        L.append(f'### {su["label"]}  &nbsp;`{su["id"]}`\n')
        L.append('| Scenario | Kind | Category | Status | Cases (assertions) | Reason / note |')
        L.append('|---|---|---|---|---|---|')
        for s in su['scenarios']:
            kind = 'negative' if s.get('kind') == 'negative' else 'positive'
            cases = '<br/>'.join(
                f'`{c["id"]}` — ' + ' & '.join(f'"{e}"' for e in c.get('assert', {}).get('expect', []))
                for c in s.get('cases', [])) or '—'
            rsn = reason(s).replace('|', '\\|') or ('all cases verified on the bench' if bucket(s) == 'supported' else '')
            L.append(f'| `{s["id"]}` | {kind} | {s.get("category","functional")} | '
                     f'{badge[bucket(s)]} | {cases} | {rsn} |')
        L.append('')
    return '\n'.join(L) + '\n'


def write_coverage(results: list, logroot: pathlib.Path) -> pathlib.Path:
    tally = {k: sum(1 for r in results if r.status == k) for k in ('pass', 'fail', 'skip')}
    icon = {'pass': '✅', 'fail': '❌', 'skip': '⚪'}
    lines = [
        '# ESP-Hosted C6↔P4 test coverage', '',
        f'Generated by `tools/esplab/runner.py`. Logs: `{logroot}`', '',
        f'**{tally["pass"]} pass · {tally["fail"]} fail · {tally["skip"]} skip/unsupported** '
        f'of {len(results)} scenarios.', '',
        '| Suite | Scenario | Kind | Result | Detail |', '|---|---|---|---|---|',
    ]
    for r in sorted(results, key=lambda r: (r.status != 'fail', r.status != 'pass', r.sc['suite'], r.id)):
        detail = (r.detail or '; '.join(c['detail'] for c in r.cases if c['status'] == 'pass'))[:80].replace('|', '\\|')
        lines.append(f'| {r.sc["suite"]} | `{r.id}` | {r.sc.get("kind","pos")} | '
                     f'{icon.get(r.status, r.status)} {r.status} | {detail} |')
    lines += ['', '## Skipped (unsupported / tbd)', '']
    for r in results:
        if r.status == 'skip':
            lines.append(f'- `{r.id}` — {r.support}')
    out = HERE / 'coverage.md'
    out.write_text('\n'.join(lines) + '\n')
    return out


if __name__ == '__main__':
    _cli()
