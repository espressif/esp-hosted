#!/usr/bin/env python3
"""
ESP-Hosted Test Config UI -- browser-based configuration for env.json.

Launch:
    python tests/tools/eh_test_ui.py
    # Opens http://localhost:8321 in default browser

Features:
    - Edit hardware ports, WiFi, paths, build strategy
    - Toggle test suites on/off
    - Probe connected hardware
    - View/clear flash cache
    - Browse test run artifacts
    - Run tests with live SSE output streaming

Zero dependencies beyond Python stdlib.
"""

import glob
import http.server
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

PORT = 8321

# ── Path constants ─────────────────────────────────────────────────────
TESTS_DIR = Path(__file__).resolve().parent.parent  # tests/
ENV_FILE = TESTS_DIR / 'env.json'
ENV_EXAMPLE = TESTS_DIR / 'env.json.example'
MATRIX_FILE = TESTS_DIR / 'hw' / 'eh_test_matrix.json'
RUNNER_SCRIPT = TESTS_DIR / 'hw' / 'eh_test_runner.py'
FLASH_CACHE_FILE = Path('/tmp/eh_test_flash_cache.json')

# Add tests/ to path so we can import infra modules
sys.path.insert(0, str(TESTS_DIR))


# ── Config helpers ─────────────────────────────────────────────────────

def load_config():
    for path in [ENV_FILE, ENV_EXAMPLE]:
        if path.exists():
            with open(path) as f:
                return json.load(f)
    return {}


def save_config(data):
    with open(ENV_FILE, 'w') as f:
        json.dump(data, f, indent=2)
        f.write('\n')


def load_matrix():
    if MATRIX_FILE.exists():
        with open(MATRIX_FILE) as f:
            return json.load(f)
    return {'pairs': {}}


def load_flash_cache():
    if FLASH_CACHE_FILE.exists():
        try:
            with open(FLASH_CACHE_FILE) as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError):
            return {}
    return {}


def load_artifacts_manifest():
    cfg = load_config()
    art_dir = Path(os.path.expanduser(cfg.get('artifacts', {}).get('dir', '/tmp/esp_hosted_test_logs')))
    manifest_path = art_dir / 'manifest.json'
    if manifest_path.exists():
        try:
            with open(manifest_path) as f:
                return json.load(f), str(art_dir)
        except (json.JSONDecodeError, OSError):
            pass
    return {'runs': []}, str(art_dir)


def list_serial_ports():
    ports = []
    for pattern in ['/dev/cu.usbserial-*', '/dev/cu.usbmodem*',
                    '/dev/ttyUSB*', '/dev/ttyACM*']:
        ports.extend(glob.glob(pattern))
    return sorted(ports)


# ── ANSI strip ─────────────────────────────────────────────────────────
_ANSI_RE = re.compile(r'\x1b\[[0-9;]*[mGKHJ]')


def strip_ansi(s):
    return _ANSI_RE.sub('', s)


# ── Background state classes ───────────────────────────────────────────

class ProbeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = 'idle'  # idle, running, done
        self.result = None

    def start(self, host_port, slave_port):
        with self.lock:
            if self.status == 'running':
                return False
            self.status = 'running'
            self.result = None
        t = threading.Thread(target=self._worker, args=(host_port, slave_port), daemon=True)
        t.start()
        return True

    def _worker(self, host_port, slave_port):
        try:
            from infra.hardware import eh_test_hw_probe_device
            host = eh_test_hw_probe_device(host_port)
            slave = eh_test_hw_probe_device(slave_port)
            with self.lock:
                self.result = {'host': host, 'slave': slave}
                self.status = 'done'
        except Exception as e:
            with self.lock:
                self.result = {'error': str(e)}
                self.status = 'done'

    def get(self):
        with self.lock:
            return {'status': self.status, 'result': self.result}


# Regex patterns for parsing runner output
_RX_TEST_START = re.compile(r'^\[(\d+)/(\d+)\]\s+(\S+)\s+--\s+(.+)$')
_RX_PHASE = re.compile(r'^\s{2}(\w[^:]*):\s*(.+)$')
_RX_TEST_RESULT = re.compile(r'(\d+)p/(\d+)f.*\(([0-9.]+)s\)')
_RX_LOG_PATH = re.compile(r'Log:\s*(\S+)')
_RX_SUMMARY_PASS = re.compile(r'(\d+)\s+passed')
_RX_SUMMARY_FAIL = re.compile(r'(\d+)\s+failed')


class RunState:
    """Tracks a running test execution with full output buffering,
    per-test state parsing, and late-connecting SSE client support."""

    MAX_BUFFERED_LINES = 10000  # Cap on line buffer size

    def __init__(self):
        self.lock = threading.Lock()
        self.process = None
        self.running = False
        self.suites = []
        self.started_at = None
        self.started_at_ts = None
        self.exit_code = None
        self.sse_clients = []  # list of queue.Queue

        # Full output buffer (so late-connecting clients replay history)
        self.lines = []

        # Per-test state: {test_name: {status, index, total, description, duration, phases, log_path, ...}}
        self.tests = {}
        self.current_test = None  # name of currently-running test
        self.test_order = []  # insertion order
        self.run_id = None  # timestamp ID from artifacts manager

    def reset(self):
        self.lines = []
        self.tests = {}
        self.test_order = []
        self.current_test = None
        self.exit_code = None
        self.run_id = None

    def add_client(self, q, replay=True):
        """Add an SSE client. If replay, sends buffered history first."""
        with self.lock:
            if replay:
                # Send snapshot event with current state
                snapshot = {
                    'type': 'snapshot',
                    'lines': list(self.lines),
                    'tests': dict(self.tests),
                    'test_order': list(self.test_order),
                    'current_test': self.current_test,
                    'running': self.running,
                }
                try:
                    q.put_nowait(('snapshot', snapshot))
                except queue.Full:
                    pass
            self.sse_clients.append(q)

    def remove_client(self, q):
        with self.lock:
            if q in self.sse_clients:
                self.sse_clients.remove(q)

    def _broadcast_event(self, evt_type, payload):
        """Send a structured event to all SSE clients."""
        with self.lock:
            dead = []
            for q in self.sse_clients:
                try:
                    q.put_nowait((evt_type, payload))
                except queue.Full:
                    dead.append(q)
            for q in dead:
                self.sse_clients.remove(q)

    def _append_line(self, line):
        """Add a line to the buffer and parse state."""
        with self.lock:
            self.lines.append(line)
            if len(self.lines) > self.MAX_BUFFERED_LINES:
                self.lines = self.lines[-self.MAX_BUFFERED_LINES:]

        self._parse_line(line)
        self._broadcast_event('line', line)

    def _parse_line(self, line):
        """Extract per-test state from runner output lines."""
        # Test start: [N/M] test_name -- description
        m = _RX_TEST_START.match(line)
        if m:
            idx, total, name, desc = m.groups()
            with self.lock:
                # Mark previous test as complete if still running
                if self.current_test and self.tests.get(self.current_test, {}).get('status') == 'running':
                    prev = self.tests[self.current_test]
                    prev['status'] = 'completed'
                    if not prev.get('duration'):
                        prev['duration'] = time.time() - prev.get('started_at', time.time())
                # Start new test
                self.tests[name] = {
                    'name': name,
                    'index': int(idx),
                    'total': int(total),
                    'description': desc,
                    'status': 'running',
                    'phases': [],
                    'log_lines': [],
                    'started_at': time.time(),
                    'duration': 0,
                    'test_duration': None,
                    'passed': 0,
                    'failed': 0,
                    'log_path': None,
                }
                if name not in self.test_order:
                    self.test_order.append(name)
                self.current_test = name
            self._broadcast_event('test_start', self.tests[name])
            return

        # Run ID discovery (artifact path)
        if 'Retained for' in line or '/esp_hosted_test_logs/' in line:
            m2 = re.search(r'/(\d{8}_\d{6})', line)
            if m2:
                with self.lock:
                    self.run_id = m2.group(1)

        # Phase line: "  Build: ..." or "  Flash: ..."
        m = _RX_PHASE.match(line)
        if m and self.current_test:
            phase_name, phase_msg = m.groups()
            phase_name = phase_name.strip()
            phase_msg = phase_msg.strip()
            with self.lock:
                t = self.tests.get(self.current_test)
                if t:
                    t['phases'].append({'name': phase_name, 'msg': phase_msg})
                    # Extract test result from "Test: 1p/0f, 1w (8.5s)"
                    if phase_name == 'Test':
                        res = _RX_TEST_RESULT.search(phase_msg)
                        if res:
                            p, f, dur = res.groups()
                            t['passed'] = int(p)
                            t['failed'] = int(f)
                            t['test_duration'] = float(dur)
                            # Keep duration as total elapsed (build+flash+test)
                            t['duration'] = time.time() - t.get('started_at', time.time())
                            t['status'] = 'passed' if int(f) == 0 and int(p) > 0 else 'failed'
                    elif phase_name == 'Build' and 'failed' in phase_msg.lower():
                        t['status'] = 'failed'
                        t['duration'] = time.time() - t.get('started_at', time.time())
                    elif phase_name == 'Flash' and 'failed' in phase_msg.lower():
                        t['status'] = 'failed'
                        t['duration'] = time.time() - t.get('started_at', time.time())
            self._broadcast_event('test_update', self.tests.get(self.current_test))
            return

        # Capture log path
        m = _RX_LOG_PATH.search(line)
        if m and self.current_test:
            with self.lock:
                t = self.tests.get(self.current_test)
                if t:
                    t['log_path'] = m.group(1)
            return

        # Append to current test's log buffer
        if self.current_test:
            with self.lock:
                t = self.tests.get(self.current_test)
                if t and len(t['log_lines']) < 500:
                    t['log_lines'].append(line)

    def start(self, cmd, suites):
        with self.lock:
            if self.running:
                return False, 'A test run is already in progress'
            self.running = True
            self.suites = suites
            self.started_at = time.strftime('%Y-%m-%d %H:%M:%S')
            self.started_at_ts = time.time()
            self.reset()

        # Subprocess env: force unbuffered output so we see lines in real time.
        # Without this, Python's stdout block-buffers when piped (8KB chunks),
        # making it look like "no output" until the process exits.
        env = dict(os.environ)
        env['PYTHONUNBUFFERED'] = '1'
        env['FORCE_COLOR'] = '0'   # Don't emit ANSI if we're going to strip it
        env['NO_COLOR'] = '1'

        try:
            self.process = subprocess.Popen(
                cmd, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                preexec_fn=os.setsid,
                cwd=str(TESTS_DIR / 'hw'),
                env=env,
            )
        except Exception as e:
            with self.lock:
                self.running = False
            return False, str(e)

        threading.Thread(target=self._reader, daemon=True).start()
        return True, 'Started'

    def _reader(self):
        """Read subprocess output. Uses readline() explicitly so partial lines
        still get flushed. PYTHONUNBUFFERED=1 ensures upstream flushes on each line."""
        try:
            while True:
                line = self.process.stdout.readline()
                if not line:
                    break
                self._append_line(strip_ansi(line.rstrip('\n')))
            self.process.wait()
        except Exception as e:
            self._append_line(f'[reader error] {e}')
        with self.lock:
            self.exit_code = self.process.returncode if self.process else -1
            self.running = False
            if self.current_test and self.tests.get(self.current_test, {}).get('status') == 'running':
                t = self.tests[self.current_test]
                t['status'] = 'completed'
                if not t.get('duration'):
                    t['duration'] = time.time() - t.get('started_at', time.time())
        self._broadcast_event('done', {'exit_code': self.exit_code})

    def cancel(self):
        with self.lock:
            if not self.running or not self.process:
                return False
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
        return True

    def get_status(self):
        with self.lock:
            return {
                'running': self.running,
                'suites': self.suites,
                'started_at': self.started_at,
                'exit_code': self.exit_code,
                'run_id': self.run_id,
            }

    def get_state(self):
        """Full snapshot for /api/run/state (for page reload / reconnect)."""
        with self.lock:
            return {
                'running': self.running,
                'started_at': self.started_at,
                'exit_code': self.exit_code,
                'suites': self.suites,
                'run_id': self.run_id,
                'current_test': self.current_test,
                'test_order': list(self.test_order),
                'tests': {k: dict(v) for k, v in self.tests.items()},
                'line_count': len(self.lines),
            }


# ── HTTP Handler ───────────────────────────────────────────────────────

class ConfigUIHandler(http.server.BaseHTTPRequestHandler):
    probe_state = ProbeState()
    run_state = RunState()

    def log_message(self, fmt, *args):
        pass  # Suppress default access logs

    def _read_body(self):
        length = int(self.headers.get('Content-Length', 0))
        return self.rfile.read(length) if length else b''

    def _json_response(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)

    def _text_response(self, text, status=200, content_type='text/plain'):
        body = text.encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status, msg):
        self._json_response({'error': msg}, status)

    # ── Routing ────────────────────────────────────────────────────────

    def do_GET(self):
        p = self.path.split('?')[0]
        if p == '/':
            self._serve_ui()
        elif p == '/api/config':
            self._json_response(load_config())
        elif p == '/api/matrix':
            self._api_get_matrix()
        elif p == '/api/ports':
            self._json_response(list_serial_ports())
        elif p == '/api/flash-cache':
            self._json_response(load_flash_cache())
        elif p == '/api/artifacts':
            self._api_get_artifacts()
        elif p.startswith('/api/artifacts/'):
            self._api_get_artifact_file(p)
        elif p == '/api/hardware/probe':
            self._json_response(self.probe_state.get())
        elif p == '/api/run/status':
            self._json_response(self.run_state.get_status())
        elif p == '/api/run/state':
            self._json_response(self.run_state.get_state())
        elif p == '/api/run/stream':
            self._api_sse_stream()
        else:
            self._error(404, 'Not found')

    def do_PUT(self):
        if self.path == '/api/config':
            if self.run_state.running:
                self._error(409, 'Cannot save config while tests are running')
                return
            try:
                data = json.loads(self._read_body())
                save_config(data)
                self._json_response(data)
            except (json.JSONDecodeError, OSError) as e:
                self._error(400, str(e))
        else:
            self._error(404, 'Not found')

    def do_POST(self):
        p = self.path.split('?')[0]
        if p == '/api/hardware/probe':
            self._api_post_probe()
        elif p == '/api/run':
            self._api_post_run()
        elif p == '/api/run/cancel':
            ok = self.run_state.cancel()
            self._json_response({'cancelled': ok})
        else:
            self._error(404, 'Not found')

    def do_DELETE(self):
        if self.path == '/api/flash-cache':
            try:
                if FLASH_CACHE_FILE.exists():
                    FLASH_CACHE_FILE.unlink()
                self._json_response({'deleted': True})
            except OSError as e:
                self._error(500, str(e))
        else:
            self._error(404, 'Not found')

    # ── API implementations ────────────────────────────────────────────

    def _serve_ui(self):
        body = HTML_PAGE.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)

    def _api_get_matrix(self):
        matrix = load_matrix()
        cfg = load_config()
        wifi_configured = bool(cfg.get('wifi_ap', {}).get('ssid'))
        suites_cfg = cfg.get('suites', {})
        overrides = cfg.get('suite_overrides', {})

        result = {}
        for name, pair in matrix.get('pairs', {}).items():
            requires_wifi = 'wifi_ap' in pair.get('requires', [])
            result[name] = {
                'description': pair.get('description', ''),
                'requires': pair.get('requires', []),
                'enabled': suites_cfg.get(name, True),
                'runnable': not requires_wifi or wifi_configured,
                'cp': pair.get('cp', ''),
                'host': pair.get('host', ''),
                'tests': pair.get('tests', []),
                'overrides': overrides.get(name, {}),
                'has_pre_flash': bool(pair.get('pre_flash')),
            }
        self._json_response(result)

    def _api_get_artifacts(self):
        manifest, art_dir = load_artifacts_manifest()
        self._json_response({'runs': manifest.get('runs', []), 'dir': art_dir})

    def _api_get_artifact_file(self, path):
        """Routes:
        /api/artifacts/<run_id>/files       → JSON list of files in run dir
        /api/artifacts/<run_id>/report      → parsed report.json
        /api/artifacts/<run_id>/<filename>  → raw file content
        """
        parts = path.strip('/').split('/')
        if len(parts) < 4:
            self._error(400, 'Invalid path')
            return
        run_id = parts[2]
        filename = parts[3]
        if '..' in run_id or '..' in filename:
            self._error(400, 'Invalid path')
            return
        cfg = load_config()
        art_dir = Path(os.path.expanduser(cfg.get('artifacts', {}).get('dir', '/tmp/esp_hosted_test_logs')))
        run_dir = art_dir / run_id
        if not run_dir.exists():
            self._error(404, 'Run not found')
            return

        if filename == 'files':
            # List files + per-pair subdirectories with per-test-function logs
            files = []
            subdirs = {}
            try:
                for f in sorted(run_dir.iterdir()):
                    if f.is_file():
                        files.append({
                            'name': f.name,
                            'size': f.stat().st_size,
                            'mtime': f.stat().st_mtime,
                        })
                    elif f.is_dir():
                        sub_files = []
                        for sf in sorted(f.iterdir()):
                            if sf.is_file():
                                sub_files.append({
                                    'name': sf.name,
                                    'size': sf.stat().st_size,
                                    'mtime': sf.stat().st_mtime,
                                })
                        subdirs[f.name] = sub_files
            except OSError as e:
                self._error(500, str(e))
                return
            self._json_response({'run_id': run_id, 'files': files, 'subdirs': subdirs})
            return

        if filename == 'report':
            report_path = run_dir / 'report.json'
            if not report_path.exists():
                self._error(404, 'No report.json')
                return
            try:
                with open(report_path) as f:
                    self._json_response(json.load(f))
            except (OSError, json.JSONDecodeError) as e:
                self._error(500, str(e))
            return

        # Raw file content — supports subdir/filename path
        # parts beyond [3] are additional path segments for subdir files
        path_parts = parts[3:]
        if any('..' in p for p in path_parts):
            self._error(400, 'Invalid filename')
            return
        fpath = run_dir.joinpath(*path_parts)
        # Ensure final path is still under run_dir (defense in depth)
        try:
            fpath.resolve().relative_to(run_dir.resolve())
        except ValueError:
            self._error(400, 'Invalid path')
            return
        if not fpath.exists() or not fpath.is_file():
            self._error(404, 'File not found')
            return
        try:
            content = fpath.read_text(errors='replace')
            self._text_response(content)
        except OSError as e:
            self._error(500, str(e))

    def _api_post_probe(self):
        cfg = load_config()
        hw = cfg.get('hardware', {})
        host_port = hw.get('host_port', '')
        slave_port = hw.get('slave_port', '')
        if not host_port or not slave_port:
            self._error(400, 'Ports not configured')
            return
        ok = self.probe_state.start(host_port, slave_port)
        if ok:
            self._json_response({'status': 'running'})
        else:
            self._error(409, 'Probe already running')

    def _api_post_run(self):
        try:
            body = json.loads(self._read_body())
        except json.JSONDecodeError:
            body = {}

        suites = body.get('suites', [])
        skip_build = body.get('skip_build', False)
        skip_flash = body.get('skip_flash', False)
        retry_failed = body.get('retry_failed', False)

        # Ensure IDF env is available — source export.sh via the runner
        cmd = f'{sys.executable} {RUNNER_SCRIPT}'
        if suites:
            cmd += f' --suite {",".join(suites)}'
        if skip_build:
            cmd += ' --skip-build'
        if skip_flash:
            cmd += ' --skip-flash'
        if retry_failed:
            cmd += ' --retry-failed'

        ok, msg = self.run_state.start(cmd, suites)
        if ok:
            self._json_response({'status': 'started', 'suites': suites})
        else:
            self._error(409, msg)

    def _api_sse_stream(self):
        """SSE stream with event types:
        - snapshot: full state on connect (replays line buffer + test states)
        - line: new output line
        - test_start, test_update: per-test state changes
        - done: run finished
        """
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()

        q = queue.Queue(maxsize=20000)
        self.run_state.add_client(q, replay=True)
        try:
            while True:
                try:
                    evt = q.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b': heartbeat\n\n')
                    self.wfile.flush()
                    continue
                evt_type, payload = evt
                data = json.dumps(payload) if not isinstance(payload, str) else payload
                if evt_type == 'line':
                    self.wfile.write(f'data: {data}\n\n'.encode())
                else:
                    self.wfile.write(f'event: {evt_type}\ndata: {data}\n\n'.encode())
                self.wfile.flush()
                if evt_type == 'done':
                    break
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.run_state.remove_client(q)


# ── Embedded HTML/CSS/JS ───────────────────────────────────────────────

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP-Hosted Test Config</title>
<style>
:root {
  --bg: #1a1b26; --bg2: #24283b; --bg3: #2f3348;
  --fg: #c0caf5; --fg2: #a9b1d6; --dim: #565f89;
  --green: #9ece6a; --red: #f7768e; --yellow: #e0af68;
  --blue: #7aa2f7; --cyan: #7dcfff; --orange: #ff9e64;
  --border: #3b4261;
}
* { margin: 0; padding: 0; box-sizing: border-box; }
body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', system-ui, sans-serif;
  background: var(--bg); color: var(--fg);
  min-height: 100vh; display: flex; flex-direction: column;
}

/* Top bar with tabs */
header {
  background: var(--bg2); border-bottom: 1px solid var(--border);
  padding: 0 24px; display: flex; align-items: center;
  height: 56px; flex-shrink: 0;
  position: sticky; top: 0; z-index: 10;
}
header h1 { font-size: 16px; color: var(--fg); margin-right: 32px; }
header .tabs { display: flex; gap: 4px; }
header .tab {
  padding: 8px 16px; cursor: pointer; color: var(--fg2);
  font-size: 14px; border-radius: 6px; transition: all 0.15s;
  background: transparent; border: none;
}
header .tab:hover { background: var(--bg3); color: var(--fg); }
header .tab.active { background: var(--blue); color: var(--bg); }
header .spacer { flex: 1; }
header .run-indicator { font-size: 12px; color: var(--dim); }
header .run-indicator.active { color: var(--green); }

main { flex: 1; display: flex; overflow: hidden; }

/* Config tab layout */
.cfg-layout { display: flex; width: 100%; }
nav.sidebar {
  width: 180px; background: var(--bg2); border-right: 1px solid var(--border);
  padding: 16px 0; flex-shrink: 0;
  overflow-y: auto;
}
nav.sidebar a {
  display: block; padding: 8px 16px; color: var(--fg2); text-decoration: none;
  font-size: 13px; border-left: 3px solid transparent; transition: all 0.15s;
  cursor: pointer;
}
nav.sidebar a:hover { background: var(--bg3); color: var(--fg); }
nav.sidebar a.active { border-left-color: var(--blue); color: var(--blue); background: var(--bg3); }

.cfg-content {
  flex: 1; padding: 24px 32px; max-width: 900px;
  overflow-y: auto;
}

/* Tests tab layout */
.tests-layout { width: 100%; display: flex; flex-direction: column; overflow: hidden; }
.tests-toolbar {
  padding: 16px 24px; background: var(--bg2); border-bottom: 1px solid var(--border);
}
.tests-body {
  flex: 1; overflow-y: auto; padding: 16px 24px;
  display: grid; grid-template-columns: 1fr 1fr; gap: 16px;
}
@media (max-width: 1100px) { .tests-body { grid-template-columns: 1fr; } }

.tests-body .left, .tests-body .right {
  display: flex; flex-direction: column; gap: 8px; min-width: 0;
}

h1, h2 { font-weight: 500; }
h1 { font-size: 18px; margin-bottom: 16px; }
h2 { font-size: 14px; color: var(--dim); text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 8px; }

/* Cards (config) */
.card {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 8px;
  padding: 20px; margin-bottom: 20px;
}
.card h3 {
  font-size: 14px; color: var(--blue); margin-bottom: 14px;
  text-transform: uppercase; letter-spacing: 0.5px;
}

/* Form elements */
label { display: block; font-size: 12px; color: var(--dim); margin-bottom: 4px; }
input[type="text"], input[type="password"], input[type="number"], select, textarea {
  width: 100%; padding: 8px 10px; background: var(--bg); border: 1px solid var(--border);
  border-radius: 4px; color: var(--fg); font-size: 13px;
  font-family: 'SF Mono', 'Fira Code', monospace;
  outline: none; transition: border-color 0.15s;
}
input:focus, select:focus, textarea:focus { border-color: var(--blue); }
textarea { resize: vertical; min-height: 60px; }
.field { margin-bottom: 12px; }
.row { display: flex; gap: 12px; }
.row .field { flex: 1; }

/* Buttons */
button {
  padding: 8px 16px; border-radius: 4px; border: 1px solid var(--border);
  background: var(--bg3); color: var(--fg); cursor: pointer; font-size: 13px;
  transition: all 0.15s;
}
button:hover:not(:disabled) { background: var(--border); }
button.primary { background: var(--blue); color: var(--bg); border-color: var(--blue); }
button.primary:hover:not(:disabled) { opacity: 0.9; }
button.danger { background: var(--red); color: var(--bg); border-color: var(--red); }
button.danger:hover:not(:disabled) { opacity: 0.9; }
button:disabled { opacity: 0.4; cursor: not-allowed; }
button.small { padding: 4px 10px; font-size: 11px; }

/* Badges */
.badge {
  display: inline-block; padding: 2px 8px; border-radius: 10px;
  font-size: 11px; font-weight: 600;
}
.badge.green { background: var(--green); color: var(--bg); }
.badge.yellow { background: var(--yellow); color: var(--bg); }
.badge.red { background: var(--red); color: var(--bg); }
.badge.dim { background: var(--bg3); color: var(--dim); }
.badge.blue { background: var(--blue); color: var(--bg); }

.radio-group { display: flex; gap: 8px; margin-top: 4px; flex-wrap: wrap; }
.radio-group label {
  display: flex; align-items: center; gap: 4px;
  font-size: 13px; color: var(--fg2); cursor: pointer;
}
.radio-group input { accent-color: var(--blue); }

/* Suite toggles */
.suite-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
.suite-item {
  display: flex; align-items: center; gap: 10px;
  padding: 8px 12px; background: var(--bg); border-radius: 4px;
  border: 1px solid var(--border);
}
.suite-item.disabled { opacity: 0.4; }
.suite-item .name { font-size: 13px; font-weight: 600; color: var(--fg); }
.suite-item .desc { font-size: 11px; color: var(--dim); }
.suite-item .info { flex: 1; min-width: 0; }
.toggle { position: relative; width: 36px; height: 20px; flex-shrink: 0; }
.toggle input { display: none; }
.toggle span {
  position: absolute; inset: 0; background: var(--bg3); border-radius: 10px;
  cursor: pointer; transition: 0.2s;
}
.toggle span::after {
  content: ''; position: absolute; width: 16px; height: 16px;
  left: 2px; top: 2px; background: var(--dim); border-radius: 50%;
  transition: 0.2s;
}
.toggle input:checked + span { background: var(--blue); }
.toggle input:checked + span::after { left: 18px; background: white; }

table { width: 100%; border-collapse: collapse; font-size: 13px; }
th { text-align: left; color: var(--dim); font-size: 11px; padding: 6px 8px; border-bottom: 1px solid var(--border); }
td { padding: 6px 8px; border-bottom: 1px solid var(--border); font-family: monospace; font-size: 12px; }

/* ── Test Execution tab ──────────────────────────────────────────── */
.tests-toolbar .status-line {
  display: flex; align-items: center; gap: 12px; margin-bottom: 8px;
  flex-wrap: wrap;
}
.tests-toolbar .run-status {
  font-size: 14px; font-weight: 600; color: var(--fg);
}
.tests-toolbar .run-stats { color: var(--dim); font-size: 13px; }
.tests-toolbar .progress-bar {
  height: 4px; background: var(--bg3); border-radius: 2px; overflow: hidden; margin-top: 6px;
}
.tests-toolbar .progress-fill {
  height: 100%; background: var(--blue); transition: width 0.3s;
}
.tests-toolbar .controls {
  display: flex; gap: 8px; margin-top: 12px; flex-wrap: wrap; align-items: center;
}
.tests-toolbar .controls label {
  display: flex; align-items: center; gap: 4px; margin: 0; font-size: 13px; color: var(--fg2); cursor: pointer;
}

/* Test card */
.test-card {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 6px;
  overflow: hidden; transition: border-color 0.2s;
}
.test-card.running { border-color: var(--blue); box-shadow: 0 0 0 1px var(--blue); }
.test-card.passed { border-color: #3d5a30; }
.test-card.failed { border-color: #5a3030; }

.test-header {
  padding: 10px 14px; display: flex; align-items: center; gap: 10px;
  cursor: pointer; user-select: none; transition: background 0.15s;
}
.test-header:hover { background: var(--bg3); }

.test-status-icon {
  width: 20px; height: 20px; flex-shrink: 0;
  display: flex; align-items: center; justify-content: center;
  border-radius: 50%;
}
.test-status-icon.pending { background: var(--bg3); color: var(--dim); }
.test-status-icon.running { background: var(--blue); color: var(--bg); animation: spin 2s linear infinite; }
.test-status-icon.passed { background: var(--green); color: var(--bg); }
.test-status-icon.failed { background: var(--red); color: var(--bg); }
.test-status-icon.skipped { background: var(--dim); color: var(--bg); }
@keyframes spin { from { transform: rotate(0deg); } to { transform: rotate(360deg); } }

.test-meta { flex: 1; min-width: 0; }
.test-name {
  font-size: 14px; font-weight: 600; color: var(--fg);
  display: flex; align-items: center; gap: 8px;
}
.test-name .index { color: var(--dim); font-weight: 400; font-size: 12px; }
.test-desc { font-size: 12px; color: var(--fg2); margin-top: 2px; }
.test-duration { font-size: 12px; color: var(--dim); font-family: monospace; flex-shrink: 0; }

.test-body {
  border-top: 1px solid var(--border); padding: 10px 14px;
  background: var(--bg); display: none;
}
.test-body.open { display: block; }

.phase-list { margin-bottom: 10px; }
.phase-item {
  font-size: 12px; padding: 3px 0; color: var(--fg2);
  display: flex; gap: 6px;
}
.phase-item .label { color: var(--blue); font-weight: 600; min-width: 50px; }
.phase-item.failed .label { color: var(--red); }

.test-log {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 4px;
  padding: 8px 10px; max-height: 200px; overflow-y: auto;
  font-family: 'SF Mono', 'Fira Code', monospace;
  font-size: 11px; white-space: pre-wrap; word-break: break-all;
  color: var(--fg2); line-height: 1.5;
}

.test-actions { display: flex; gap: 6px; margin-top: 8px; }
.test-actions a {
  color: var(--cyan); font-size: 11px; text-decoration: none;
  padding: 3px 8px; background: var(--bg2); border: 1px solid var(--border);
  border-radius: 3px;
}
.test-actions a:hover { background: var(--bg3); }

.test-config-panel {
  margin-top: 10px; padding: 8px 10px;
  background: var(--bg2); border: 1px solid var(--border); border-radius: 4px;
  display: none;
}
.test-config-panel.open { display: block; }
.test-config-panel .row { margin-bottom: 4px; font-size: 11px; }
.test-config-panel .row span.k { color: var(--dim); min-width: 80px; display: inline-block; }
.test-config-panel .row span.v { color: var(--fg); font-family: monospace; }

/* Raw output tail */
.output-box {
  background: var(--bg); border: 1px solid var(--border); border-radius: 4px;
  padding: 12px; max-height: 300px; overflow-y: auto;
  font-family: 'SF Mono', 'Fira Code', monospace;
  font-size: 11px; line-height: 1.5; white-space: pre-wrap; word-break: break-all;
  color: var(--fg2);
}

/* Artifacts */
.run-entry {
  padding: 8px 0; border-bottom: 1px solid var(--border); font-size: 13px;
}
.run-entry a { color: var(--cyan); text-decoration: none; }
.run-entry a:hover { text-decoration: underline; }

.btn-row { display: flex; gap: 8px; margin-top: 16px; }
.btn-row.right { justify-content: flex-end; }

.unsaved { color: var(--yellow); font-size: 12px; margin-left: 8px; display: none; }
.unsaved.show { display: inline; }

.check-row { display: flex; align-items: center; gap: 6px; margin-bottom: 8px; font-size: 13px; }
.check-row input { accent-color: var(--blue); }

.probe-result { margin-top: 12px; }
.probe-device {
  display: flex; align-items: center; gap: 8px; padding: 6px 0; font-size: 13px;
}
.probe-device .dot { width: 8px; height: 8px; border-radius: 50%; }

.toast {
  position: fixed; bottom: 20px; right: 20px; padding: 10px 20px;
  background: var(--green); color: var(--bg); border-radius: 6px;
  font-size: 13px; font-weight: 600; opacity: 0; transition: opacity 0.3s;
  pointer-events: none; z-index: 100;
}
.toast.show { opacity: 1; }
.toast.error { background: var(--red); }

/* Hidden */
.hidden { display: none !important; }
</style>
</head>
<body>

<header>
  <h1>ESP-HOSTED</h1>
  <div class="tabs">
    <button class="tab active" data-tab="config" onclick="switchTab('config')">Configuration</button>
    <button class="tab" data-tab="tests" onclick="switchTab('tests')">Test Execution</button>
  </div>
  <div class="spacer"></div>
  <div class="run-indicator" id="run-indicator">Idle</div>
</header>

<main>
  <!-- CONFIG TAB -->
  <div id="tab-config" class="cfg-layout">
    <nav class="sidebar">
      <a class="side-link active" onclick="scrollToSection('hardware')">Hardware</a>
      <a class="side-link" onclick="scrollToSection('wifi')">WiFi AP</a>
      <a class="side-link" onclick="scrollToSection('paths')">Paths</a>
      <a class="side-link" onclick="scrollToSection('build')">Build &amp; Order</a>
      <a class="side-link" onclick="scrollToSection('flash')">Flash</a>
      <a class="side-link" onclick="scrollToSection('suites')">Test Suites</a>
      <a class="side-link" onclick="scrollToSection('artifacts')">Artifacts</a>
    </nav>
    <div class="cfg-content">
      <h1>Test Environment Config <span class="unsaved" id="unsaved-indicator">* unsaved changes</span></h1>

      <div class="card" id="sec-hardware">
        <h3>Hardware</h3>
        <div class="row">
          <div class="field"><label>Host Port</label><select id="host_port" onchange="markDirty()"></select></div>
          <div class="field"><label>Slave Port</label><select id="slave_port" onchange="markDirty()"></select></div>
        </div>
        <div class="row">
          <div class="field"><label>Board</label>
            <select id="board" onchange="markDirty()">
              <option value="p4_c6_core_board">p4_c6_core_board</option>
              <option value="p4_c5_core_board">p4_c5_core_board</option>
            </select>
          </div>
          <div class="field"><label>Flash Baud</label>
            <select id="flash_baud" onchange="markDirty()">
              <option value="460800">460800</option>
              <option value="921600">921600</option>
              <option value="2000000">2000000</option>
            </select>
          </div>
        </div>
        <div class="btn-row">
          <button onclick="probeHardware()" id="probe-btn">Probe Hardware</button>
        </div>
        <div class="probe-result" id="probe-result" style="display:none"></div>
      </div>

      <div class="card" id="sec-wifi">
        <h3>WiFi AP <span class="badge" id="wifi-badge"></span></h3>
        <div class="row">
          <div class="field"><label>SSID</label>
            <input type="text" id="wifi_ssid" onchange="markDirty(); updateWifiBadge()"></div>
          <div class="field"><label>Password</label>
            <input type="password" id="wifi_password" onchange="markDirty()"></div>
        </div>
      </div>

      <div class="card" id="sec-paths">
        <h3>Paths</h3>
        <div class="field"><label>IDF Path</label><input type="text" id="idf_path" onchange="markDirty()"></div>
        <div class="field"><label>CP Base</label><input type="text" id="cp_base" onchange="markDirty()"></div>
        <div class="field"><label>Host Base</label><input type="text" id="host_base" onchange="markDirty()"></div>
        <div class="field"><label>Workspace Directory</label>
          <input type="text" id="workspace_dir" onchange="markDirty()" placeholder="(default: tests/workspace)"></div>
      </div>

      <div class="card" id="sec-build">
        <h3>Build &amp; Test Order</h3>
        <div class="field"><label>Build Strategy</label>
          <div class="radio-group">
            <label><input type="radio" name="strategy" value="reuse" onchange="markDirty()"> Reuse (fastest)</label>
            <label><input type="radio" name="strategy" value="clean" onchange="markDirty()"> Clean (safest)</label>
            <label><input type="radio" name="strategy" value="fallback" onchange="markDirty()"> Fallback</label>
          </div>
        </div>
        <div class="field"><label>SDKConfig Optimizations (one per line)</label>
          <textarea id="sdkconfig_opts" rows="3" onchange="markDirty()"></textarea></div>
        <div class="field"><label>Test Order</label>
          <div class="radio-group">
            <label><input type="radio" name="test_order" value="auto" onchange="markDirty()"> Auto (minimize flash transitions)</label>
            <label><input type="radio" name="test_order" value="alphabetical" onchange="markDirty()"> Alphabetical</label>
            <label><input type="radio" name="test_order" value="manual" onchange="markDirty()"> Manual (matrix order)</label>
          </div>
        </div>
      </div>

      <div class="btn-row right">
        <button class="primary" onclick="saveConfig()" id="save-btn">Save Configuration</button>
      </div>

      <div class="card" id="sec-flash">
        <h3>Flash Optimization</h3>
        <div class="field"><label>Strategy</label>
          <div class="radio-group">
            <label><input type="radio" name="flash_strategy" value="sequential" onchange="markDirty()"> Sequential (safest)</label>
            <label><input type="radio" name="flash_strategy" value="parallel" onchange="markDirty()"> Parallel (fastest)</label>
            <label><input type="radio" name="flash_strategy" value="staggered" onchange="markDirty()"> Staggered</label>
          </div>
        </div>
        <div class="row">
          <div class="field"><label>Timeout Multiplier</label><input type="text" id="flash_timeout_multiplier" onchange="markDirty()"></div>
          <div class="field"><label>Overhead (s)</label><input type="text" id="flash_overhead_s" onchange="markDirty()"></div>
          <div class="field"><label>Compression Ratio</label><input type="text" id="flash_compression_ratio" onchange="markDirty()"></div>
        </div>
        <div class="check-row">
          <input type="checkbox" id="flash_use_diff" onchange="markDirty()">
          <label for="flash_use_diff">Use <code>--diff-with</code> (sector-level diff flash)</label>
        </div>
        <div class="check-row">
          <input type="checkbox" id="flash_trust_content" onchange="markDirty()">
          <label for="flash_trust_content">Trust flash content (skip MD5 on unchanged sectors)</label>
        </div>
        <div class="check-row">
          <input type="checkbox" id="flash_force_full" onchange="markDirty()">
          <label for="flash_force_full"><strong>Force full flash</strong> (disable hash-based skip — always flash bootloader + partition table + app)</label>
        </div>

        <h3 style="margin-top:20px">Cache &amp; Calibration</h3>
        <div id="flash-cache-content"></div>
        <div class="btn-row">
          <button onclick="refreshFlashCache()">Refresh</button>
          <button class="danger" onclick="clearFlashCache()">Clear Cache</button>
        </div>
      </div>

      <div class="card" id="sec-suites">
        <h3>Test Suites <span class="badge dim" id="suite-count"></span></h3>
        <div class="btn-row" style="margin-bottom:12px">
          <button onclick="toggleAllSuites(true)">Enable All</button>
          <button onclick="toggleAllSuites(false)">Disable All</button>
        </div>
        <div class="suite-grid" id="suite-grid"></div>
      </div>

      <div class="card" id="sec-artifacts">
        <h3>Artifacts &amp; Log Retention</h3>
        <div class="field">
          <label>Retention mode</label>
          <div class="radio-group">
            <label><input type="radio" name="log_retention" value="checkpoints_on_pass" onchange="markDirty()"> Checkpoints on pass (default)</label>
            <label><input type="radio" name="log_retention" value="fail_only" onchange="markDirty()"> Fail only (legacy)</label>
            <label><input type="radio" name="log_retention" value="full_always" onchange="markDirty()"> Full always</label>
            <label><input type="radio" name="log_retention" value="none" onchange="markDirty()"> None</label>
          </div>
          <p style="color:var(--dim); font-size:11px; margin-top:4px">
            <b>Checkpoints on pass</b>: failures get full logs; passes get a compact expect-match trace.<br>
            <b>Fail only</b>: only failures saved. <b>Full always</b>: every pair saves full logs. <b>None</b>: nothing saved.
          </p>
        </div>
        <h3 style="margin-top:20px">Recent Test Runs</h3>
        <div id="artifacts-content"></div>
      </div>
    </div>
  </div>

  <!-- TESTS TAB -->
  <div id="tab-tests" class="tests-layout hidden">
    <div class="tests-toolbar">
      <div class="status-line">
        <div class="run-status" id="tests-status">Idle</div>
        <div class="run-stats" id="tests-stats"></div>
      </div>
      <div class="progress-bar"><div class="progress-fill" id="tests-progress" style="width:0"></div></div>
      <div class="controls">
        <input type="text" id="tests-filter" placeholder="suites (e.g. boot_wifi,gpio — empty = all enabled)" style="flex:1; min-width:240px;">
        <label><input type="checkbox" id="tests-skip-build"> Skip build</label>
        <label><input type="checkbox" id="tests-skip-flash"> Skip flash</label>
        <button class="primary" onclick="startRun(false)" id="tests-run-btn">Run Tests</button>
        <button onclick="startRun(true)" id="tests-retry-btn">Retry Failed</button>
        <button class="danger" onclick="cancelRun()" id="tests-cancel-btn" disabled>Cancel</button>
      </div>
    </div>
    <div class="tests-body">
      <div class="left">
        <h2>Test Progress</h2>
        <div id="tests-list"></div>
      </div>
      <div class="right">
        <h2>Raw Output</h2>
        <div class="output-box" id="tests-output"></div>
      </div>
    </div>
  </div>
</main>

<div class="toast" id="toast"></div>

<script>
// ── Global state ──────────────────────────────────────────────────────
let config = {};
let matrix = {};
let dirty = false;
let eventSource = null;
let testStates = {};  // {name: state}
let testOrder = [];

// ── Tab switching ─────────────────────────────────────────────────────
function switchTab(name) {
  document.querySelectorAll('header .tab').forEach(t =>
    t.classList.toggle('active', t.dataset.tab === name));
  document.getElementById('tab-config').classList.toggle('hidden', name !== 'config');
  document.getElementById('tab-tests').classList.toggle('hidden', name !== 'tests');
}

function scrollToSection(id) {
  document.querySelectorAll('.side-link').forEach(a => a.classList.remove('active'));
  event.target.classList.add('active');
  const el = document.getElementById('sec-' + id);
  if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

// ── Init ──────────────────────────────────────────────────────────────
async function init() {
  await Promise.all([
    loadConfig(), loadMatrix(), loadPorts(),
    refreshFlashCache(), loadArtifacts()
  ]);
  await loadInitialRunState();
  connectSSE();
}

async function loadInitialRunState() {
  try {
    const state = await fetchJson('/api/run/state');
    testStates = state.tests || {};
    testOrder = state.test_order || [];
    renderTests();
    updateRunUI(state.running, state);
  } catch (e) {}
}

// ── Config CRUD ───────────────────────────────────────────────────────
async function loadConfig() {
  config = await fetchJson('/api/config');
  populateForm();
}

function populateForm() {
  const hw = config.hardware || {};
  const wifi = config.wifi_ap || {};
  const paths = config.paths || {};
  const build = config.build || {};
  const flash = config.flash || {};

  setSelectVal('host_port', hw.host_port || '');
  setSelectVal('slave_port', hw.slave_port || '');
  setSelectVal('board', hw.board || 'p4_c6_core_board');
  setSelectVal('flash_baud', String(hw.flash_baud || 2000000));

  document.getElementById('wifi_ssid').value = wifi.ssid || '';
  document.getElementById('wifi_password').value = wifi.password || '';
  updateWifiBadge();

  document.getElementById('idf_path').value = paths.idf_path || '~/esp-idf';
  document.getElementById('cp_base').value = paths.cp_base || '';
  document.getElementById('host_base').value = paths.host_base || '';
  document.getElementById('workspace_dir').value = build.workspace_dir || '';

  checkRadio('strategy', build.strategy || 'fallback');
  const opts = build.sdkconfig_optimizations ||
    ['CONFIG_SPIRAM_MEMTEST=n', 'CONFIG_BOOTLOADER_SKIP_VALIDATE_ON_POWER_ON=y'];
  document.getElementById('sdkconfig_opts').value = opts.join('\n');

  checkRadio('test_order', config.test_order || 'alphabetical');
  checkRadio('log_retention', (config.artifacts || {}).log_retention || 'checkpoints_on_pass');
  checkRadio('flash_strategy', flash.strategy || 'sequential');
  document.getElementById('flash_timeout_multiplier').value = flash.timeout_multiplier ?? 2.0;
  document.getElementById('flash_overhead_s').value = flash.overhead_s ?? 3.0;
  document.getElementById('flash_compression_ratio').value = flash.compression_ratio ?? 0.65;
  document.getElementById('flash_use_diff').checked = !!flash.use_diff;
  document.getElementById('flash_trust_content').checked = !!flash.trust_flash_content;
  document.getElementById('flash_force_full').checked = !!flash.force_full_flash;

  dirty = false;
  document.getElementById('unsaved-indicator').classList.remove('show');
}

function checkRadio(name, value) {
  const el = document.querySelector(`input[name="${name}"][value="${value}"]`);
  if (el) el.checked = true;
}

function gatherForm() {
  const suites = {};
  document.querySelectorAll('.suite-toggle').forEach(cb => {
    suites[cb.dataset.suite] = cb.checked;
  });
  const optsText = document.getElementById('sdkconfig_opts').value.trim();
  const opts = optsText ? optsText.split('\n').map(s => s.trim()).filter(Boolean) : [];
  const baud = parseInt(document.getElementById('flash_baud').value);
  return {
    hardware: {
      host_port: document.getElementById('host_port').value,
      slave_port: document.getElementById('slave_port').value,
      board: document.getElementById('board').value,
      flash_baud: baud,
    },
    wifi_ap: {
      ssid: document.getElementById('wifi_ssid').value,
      password: document.getElementById('wifi_password').value,
    },
    paths: {
      idf_path: document.getElementById('idf_path').value,
      cp_base: document.getElementById('cp_base').value,
      host_base: document.getElementById('host_base').value,
    },
    build: {
      strategy: document.querySelector('input[name="strategy"]:checked')?.value || 'fallback',
      workspace_dir: document.getElementById('workspace_dir').value || undefined,
      sdkconfig_optimizations: opts.length ? opts : undefined,
    },
    flash: {
      strategy: document.querySelector('input[name="flash_strategy"]:checked')?.value || 'sequential',
      baud: baud,
      timeout_multiplier: parseFloat(document.getElementById('flash_timeout_multiplier').value) || 2.0,
      overhead_s: parseFloat(document.getElementById('flash_overhead_s').value) || 3.0,
      compression_ratio: parseFloat(document.getElementById('flash_compression_ratio').value) || 0.65,
      use_diff: document.getElementById('flash_use_diff').checked,
      trust_flash_content: document.getElementById('flash_trust_content').checked,
      force_full_flash: document.getElementById('flash_force_full').checked,
    },
    test_order: document.querySelector('input[name="test_order"]:checked')?.value || 'alphabetical',
    suite_overrides: config.suite_overrides || {},
    artifacts: {
      ...(config.artifacts || {}),
      log_retention: document.querySelector('input[name="log_retention"]:checked')?.value || 'checkpoints_on_pass',
    },
    suites: suites,
  };
}

async function saveConfig() {
  const data = gatherForm();
  Object.keys(data).forEach(k => { if (data[k] === undefined) delete data[k]; });
  try {
    await fetch('/api/config', {
      method: 'PUT', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(data),
    });
    config = data;
    dirty = false;
    document.getElementById('unsaved-indicator').classList.remove('show');
    toast('Configuration saved');
  } catch (e) { toast('Save failed: ' + e.message, true); }
}

function markDirty() {
  dirty = true;
  document.getElementById('unsaved-indicator').classList.add('show');
}

// ── Ports ─────────────────────────────────────────────────────────────
async function loadPorts() {
  const ports = await fetchJson('/api/ports');
  ['host_port', 'slave_port'].forEach(id => {
    const sel = document.getElementById(id);
    const currentVal = sel.value || (config.hardware || {})[id] || '';
    sel.innerHTML = '';
    const allPorts = new Set([...ports]);
    if (currentVal) allPorts.add(currentVal);
    [...allPorts].sort().forEach(p => {
      const opt = document.createElement('option');
      opt.value = p; opt.textContent = p;
      sel.appendChild(opt);
    });
    sel.value = currentVal;
  });
}

function updateWifiBadge() {
  const badge = document.getElementById('wifi-badge');
  const ssid = document.getElementById('wifi_ssid').value;
  if (ssid) { badge.textContent = 'Configured'; badge.className = 'badge green'; }
  else { badge.textContent = 'Not configured'; badge.className = 'badge yellow'; }
}

// ── Matrix / Suites ───────────────────────────────────────────────────
async function loadMatrix() {
  matrix = await fetchJson('/api/matrix');
  renderSuites();
}

function renderSuites() {
  const grid = document.getElementById('suite-grid');
  grid.innerHTML = '';
  const entries = Object.entries(matrix);
  document.getElementById('suite-count').textContent = `${entries.length} suites`;
  for (const [name, info] of entries) {
    const item = document.createElement('div');
    item.className = 'suite-item' + (info.runnable ? '' : ' disabled');
    const badges = info.requires.map(r =>
      `<span class="badge ${info.runnable ? 'dim' : 'red'}">${r}</span>`).join(' ');
    item.innerHTML = `
      <label class="toggle">
        <input type="checkbox" class="suite-toggle" data-suite="${name}"
               ${info.enabled ? 'checked' : ''} ${info.runnable ? '' : 'disabled'}
               onchange="markDirty()">
        <span></span>
      </label>
      <div class="info">
        <div class="name">${name} ${badges}</div>
        <div class="desc">${info.description}</div>
      </div>`;
    grid.appendChild(item);
  }
}

function toggleAllSuites(state) {
  document.querySelectorAll('.suite-toggle').forEach(cb => {
    if (!cb.disabled) cb.checked = state;
  });
  markDirty();
}

// ── Flash cache ───────────────────────────────────────────────────────
async function refreshFlashCache() {
  const cache = await fetchJson('/api/flash-cache');
  const el = document.getElementById('flash-cache-content');
  const keys = Object.keys(cache);
  if (keys.length === 0) {
    el.innerHTML = '<p style="color:var(--dim);font-size:13px">No cache entries</p>';
    return;
  }
  let html = '<table><tr><th>Port</th><th>App</th><th>Layout</th><th>Last Flash</th><th>Throughput</th></tr>';
  for (const [port, val] of Object.entries(cache)) {
    const isObj = typeof val === 'object';
    const app = (isObj ? (val.app || '-') : String(val)).slice(0, 8);
    const layout = (isObj ? (val.layout || '-') : '-').slice(0, 8);
    const lastFlash = isObj && val.last_flash_s ? `${val.last_flash_s}s` : '-';
    const throughput = isObj && val.measured_throughput_bps
      ? `${(val.measured_throughput_bps / 1024).toFixed(0)} KB/s` : '-';
    html += `<tr><td>${port}</td><td>${app}...</td><td>${layout}...</td><td>${lastFlash}</td><td>${throughput}</td></tr>`;
  }
  html += '</table>';
  el.innerHTML = html;
}

async function clearFlashCache() {
  if (!confirm('Clear flash cache? Next run will do full flash.')) return;
  await fetch('/api/flash-cache', {method: 'DELETE'});
  toast('Flash cache cleared');
  refreshFlashCache();
}

// ── Artifacts ─────────────────────────────────────────────────────────
async function loadArtifacts() {
  const data = await fetchJson('/api/artifacts');
  const el = document.getElementById('artifacts-content');
  const runs = data.runs || [];
  if (runs.length === 0) {
    el.innerHTML = '<p style="color:var(--dim);font-size:13px">No test runs recorded</p>';
    return;
  }
  let html = '';
  for (const run of runs.slice(-10).reverse()) {
    const status = run.all_pass ? '<span class="badge green">PASS</span>'
                                : '<span class="badge red">FAIL</span>';
    html += `<div class="run-entry">
      ${status} <strong>${run.run_id}</strong> &mdash;
      ${run.total_passed || 0}p/${run.total_failed || 0}f
      &nbsp;<a onclick="viewRun('${run.run_id}')">files &raquo;</a>
      <span id="files-${run.run_id}" style="display:none"></span>
    </div>`;
  }
  el.innerHTML = html;
}

async function viewRun(runId) {
  const el = document.getElementById('files-' + runId);
  if (el.style.display === 'none') {
    const data = await fetchJson(`/api/artifacts/${runId}/files`);
    const files = data.files || [];
    const subdirs = data.subdirs || {};
    const fileLinks = files.map(f =>
      `<a href="/api/artifacts/${runId}/${f.name}" target="_blank">${f.name}</a>`
    ).join(', ');
    let subdirHtml = '';
    for (const [name, subFiles] of Object.entries(subdirs)) {
      const subLinks = subFiles.map(sf =>
        `<a href="/api/artifacts/${runId}/${name}/${sf.name}" target="_blank">${sf.name}</a>`
      ).join(', ');
      subdirHtml += `<div style="padding-left:16px;color:var(--dim);font-size:12px">└ ${name}/: ${subLinks}</div>`;
    }
    el.innerHTML = ' &mdash; ' + fileLinks + subdirHtml;
    el.style.display = 'inline-block';
  } else {
    el.style.display = 'none';
  }
}

// ── Hardware probe ────────────────────────────────────────────────────
async function probeHardware() {
  const btn = document.getElementById('probe-btn');
  const res = document.getElementById('probe-result');
  btn.disabled = true; btn.textContent = 'Probing...';
  res.style.display = 'block';
  res.innerHTML = '<span style="color:var(--dim)">Detecting...</span>';
  await fetch('/api/hardware/probe', {method: 'POST'});
  const poll = setInterval(async () => {
    const state = await fetchJson('/api/hardware/probe');
    if (state.status === 'done') {
      clearInterval(poll);
      btn.disabled = false; btn.textContent = 'Probe Hardware';
      renderProbeResult(state.result);
    }
  }, 500);
}

function renderProbeResult(result) {
  const el = document.getElementById('probe-result');
  if (result.error) { el.innerHTML = `<div style="color:var(--red)">${result.error}</div>`; return; }
  let html = '';
  for (const [role, dev] of [['Host', result.host], ['Slave', result.slave]]) {
    const ok = dev && dev.connected;
    const color = ok ? 'var(--green)' : 'var(--red)';
    const text = ok ? dev.chip : (dev?.error || 'Not found');
    html += `<div class="probe-device"><div class="dot" style="background:${color}"></div><strong>${role}:</strong> ${text}</div>`;
  }
  el.innerHTML = html;
}

// ── Tests tab: execution ──────────────────────────────────────────────
function getStatusIcon(status) {
  const icons = {
    pending: '<span class="test-status-icon pending">&#9675;</span>',
    running: '<span class="test-status-icon running">&#9696;</span>',
    passed: '<span class="test-status-icon passed">&#10003;</span>',
    failed: '<span class="test-status-icon failed">&#10007;</span>',
    skipped: '<span class="test-status-icon skipped">&#8212;</span>',
    completed: '<span class="test-status-icon passed">&#10003;</span>',
  };
  return icons[status] || icons.pending;
}

function renderTests() {
  const list = document.getElementById('tests-list');

  // Merge: show all suites from matrix as pending if not started,
  // in the order they appear in matrix (or optimized order when running)
  const enabled = Object.entries(matrix).filter(([n, i]) => i.enabled && i.runnable).map(([n]) => n);
  const orderToShow = testOrder.length > 0 ? testOrder :
    (enabled.length > 0 ? enabled : Object.keys(matrix));

  if (orderToShow.length === 0) {
    list.innerHTML = '<p style="color:var(--dim);font-size:13px">No enabled test suites</p>';
    updateStatsBar();
    return;
  }

  list.innerHTML = '';
  for (const name of orderToShow) {
    const t = testStates[name];
    const matrixInfo = matrix[name] || {};
    const status = t?.status || 'pending';
    const duration = t?.duration ? `${t.duration.toFixed(1)}s` : (t?.started_at ? computeElapsed(t.started_at) : '');
    const idx = t ? `${t.index}/${t.total}` : '';

    const card = document.createElement('div');
    card.className = 'test-card ' + status;
    card.id = 'test-' + name;

    const phases = (t?.phases || []).map(p => {
      const failed = p.msg.toLowerCase().includes('fail') ? ' failed' : '';
      return `<div class="phase-item${failed}"><span class="label">${p.name}:</span><span>${escapeHtml(p.msg)}</span></div>`;
    }).join('');

    const logLines = (t?.log_lines || []).slice(-50).join('\n');

    card.innerHTML = `
      <div class="test-header" onclick="toggleTestCard('${name}')">
        ${getStatusIcon(status)}
        <div class="test-meta">
          <div class="test-name">
            ${idx ? `<span class="index">${idx}</span>` : ''}${name}
            <span class="badge dim">${matrixInfo.cp || '?'} / ${matrixInfo.host || '?'}</span>
          </div>
          <div class="test-desc">${matrixInfo.description || t?.description || ''}</div>
        </div>
        <div class="test-duration">${duration}</div>
      </div>
      <div class="test-body" id="test-body-${name}">
        ${phases ? `<div class="phase-list">${phases}</div>` : '<div style="color:var(--dim);font-size:12px">No phases yet</div>'}
        ${logLines ? `<div class="test-log">${escapeHtml(logLines)}</div>` : ''}
        <div class="test-actions">
          ${t?.log_path ? `<a href="${logPathToUrl(t.log_path)}" target="_blank">Full log</a>` : ''}
          <a onclick="toggleTestConfig('${name}')">Matrix config</a>
        </div>
        <div class="test-config-panel" id="test-cfg-${name}">
          <div class="row"><span class="k">CP:</span><span class="v">${matrixInfo.cp || '?'}</span></div>
          <div class="row"><span class="k">Host:</span><span class="v">${matrixInfo.host || '?'}</span></div>
          <div class="row"><span class="k">Tests:</span><span class="v">${(matrixInfo.tests || []).join(', ')}</span></div>
          <div class="row"><span class="k">Requires:</span><span class="v">${(matrixInfo.requires || []).join(', ') || '-'}</span></div>
          <div class="row"><span class="k">Pre-flash:</span><span class="v">${matrixInfo.has_pre_flash ? 'yes (OTA-style)' : 'no'}</span></div>
          <div class="row"><span class="k">Enabled:</span><span class="v">
            <label style="cursor:pointer"><input type="checkbox" onchange="toggleSuiteFromTest('${name}', this.checked)" ${matrixInfo.enabled ? 'checked' : ''}> enabled</label>
          </span></div>
          <div class="row" style="margin-top:6px; padding-top:6px; border-top:1px solid var(--border)">
            <span class="k">Flash strategy override:</span>
            <span class="v">
              <select onchange="setSuiteOverride('${name}', 'flash', 'strategy', this.value)">
                <option value="">(use global: ${(config.flash || {}).strategy || 'sequential'})</option>
                <option value="sequential" ${(matrixInfo.overrides?.flash?.strategy === 'sequential') ? 'selected' : ''}>sequential</option>
                <option value="parallel" ${(matrixInfo.overrides?.flash?.strategy === 'parallel') ? 'selected' : ''}>parallel</option>
                <option value="staggered" ${(matrixInfo.overrides?.flash?.strategy === 'staggered') ? 'selected' : ''}>staggered</option>
              </select>
            </span>
          </div>
          <div class="row">
            <span class="k">Use diff:</span>
            <span class="v">
              <select onchange="setSuiteOverride('${name}', 'flash', 'use_diff', this.value)">
                <option value="">(use global)</option>
                <option value="true" ${(matrixInfo.overrides?.flash?.use_diff === true) ? 'selected' : ''}>true</option>
                <option value="false" ${(matrixInfo.overrides?.flash?.use_diff === false) ? 'selected' : ''}>false</option>
              </select>
            </span>
          </div>
        </div>
      </div>`;
    list.appendChild(card);
  }
  updateStatsBar();
}

function computeElapsed(ts) {
  const s = (Date.now() / 1000 - ts);
  return s > 0 ? `${s.toFixed(0)}s` : '';
}

function toggleTestCard(name) {
  const body = document.getElementById('test-body-' + name);
  if (body) body.classList.toggle('open');
}

function toggleTestConfig(name) {
  event.stopPropagation();
  const panel = document.getElementById('test-cfg-' + name);
  if (panel) panel.classList.toggle('open');
}

async function toggleSuiteFromTest(name, enabled) {
  event.stopPropagation();
  // Update config.suites and persist
  if (!config.suites) config.suites = {};
  config.suites[name] = enabled;
  await persistConfig();
  if (matrix[name]) matrix[name].enabled = enabled;
  toast(`${name} ${enabled ? 'enabled' : 'disabled'}`);
}

async function setSuiteOverride(name, section, key, value) {
  event.stopPropagation();
  if (!config.suite_overrides) config.suite_overrides = {};
  if (!config.suite_overrides[name]) config.suite_overrides[name] = {};
  if (!config.suite_overrides[name][section]) config.suite_overrides[name][section] = {};
  // Empty value → remove override (use global)
  if (value === '') {
    delete config.suite_overrides[name][section][key];
    if (Object.keys(config.suite_overrides[name][section]).length === 0) {
      delete config.suite_overrides[name][section];
    }
    if (Object.keys(config.suite_overrides[name]).length === 0) {
      delete config.suite_overrides[name];
    }
  } else {
    // Coerce type
    let v = value;
    if (value === 'true') v = true;
    else if (value === 'false') v = false;
    config.suite_overrides[name][section][key] = v;
  }
  // Update local matrix for immediate UI refresh
  if (matrix[name]) matrix[name].overrides = config.suite_overrides[name] || {};
  await persistConfig();
  toast(`${name}: ${section}.${key} = ${value || '(global)'}`);
}

async function persistConfig() {
  // Save config WITHOUT gatherForm() so we preserve the runtime changes
  await fetch('/api/config', {
    method: 'PUT', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(config),
  });
}

function updateStatsBar() {
  const testsArr = Object.values(testStates);
  const passed = testsArr.filter(t => t.status === 'passed').length;
  const failed = testsArr.filter(t => t.status === 'failed').length;
  const running = testsArr.filter(t => t.status === 'running').length;
  const total = testOrder.length || Object.keys(testStates).length;
  const done = passed + failed;
  document.getElementById('tests-stats').textContent =
    total > 0 ? `${done}/${total} complete · ${passed} passed · ${failed} failed` : '';
  document.getElementById('tests-progress').style.width =
    total > 0 ? `${(done / total * 100).toFixed(1)}%` : '0';
}

function logPathToUrl(p) {
  // Convert /tmp/esp_hosted_test_logs/<run_id>/<file>.log to /api/artifacts/<run_id>/<file>.log
  const m = p.match(/\/([\d_]+)\/([^\/]+)$/);
  return m ? `/api/artifacts/${m[1]}/${m[2]}` : '#';
}

function escapeHtml(s) {
  return (s || '').replace(/[&<>"']/g, c => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  }[c]));
}

// ── Run control ───────────────────────────────────────────────────────
async function startRun(retryFailed) {
  const suitesStr = document.getElementById('tests-filter').value.trim();
  const suites = suitesStr ? suitesStr.split(',').map(s => s.trim()).filter(Boolean) : [];
  const body = {
    suites, retry_failed: !!retryFailed,
    skip_build: document.getElementById('tests-skip-build').checked,
    skip_flash: document.getElementById('tests-skip-flash').checked,
  };
  const resp = await fetch('/api/run', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body),
  });
  if (!resp.ok) {
    const err = await resp.json();
    toast(err.error || 'Failed to start', true);
    return;
  }
  // Reset local state; server broadcasts new events as they arrive.
  testStates = {}; testOrder = [];
  document.getElementById('tests-output').textContent = '';
  renderTests();
  updateRunUI(true);
  // Ensure SSE connection is live; reconnect if needed
  if (!eventSource || eventSource.readyState === 2 /* CLOSED */) {
    connectSSE();
  }
  // Switch to Tests tab so output is visible
  switchTab('tests');
}

async function cancelRun() {
  if (!confirm('Cancel running tests?')) return;
  await fetch('/api/run/cancel', {method: 'POST'});
  toast('Cancellation requested');
}

function updateRunUI(running, status) {
  document.getElementById('tests-run-btn').disabled = running;
  document.getElementById('tests-retry-btn').disabled = running;
  document.getElementById('tests-cancel-btn').disabled = !running;
  document.getElementById('save-btn').disabled = running;

  const ind = document.getElementById('run-indicator');
  const stat = document.getElementById('tests-status');
  if (running) {
    ind.textContent = 'Running...';
    ind.classList.add('active');
    stat.textContent = 'Running';
  } else {
    ind.classList.remove('active');
    if (status?.exit_code === 0) {
      ind.textContent = 'Last run: PASS';
      stat.textContent = 'Complete';
    } else if (status?.exit_code != null) {
      ind.textContent = `Last run: FAIL (${status.exit_code})`;
      stat.textContent = `Complete (exit ${status.exit_code})`;
    } else {
      ind.textContent = 'Idle';
      stat.textContent = 'Idle';
    }
  }
}

// ── SSE ──────────────────────────────────────────────────────────────
function connectSSE() {
  if (eventSource) eventSource.close();
  eventSource = new EventSource('/api/run/stream');
  const output = document.getElementById('tests-output');

  eventSource.addEventListener('snapshot', (e) => {
    const snap = JSON.parse(e.data);
    testStates = snap.tests || {};
    testOrder = snap.test_order || [];
    output.textContent = (snap.lines || []).join('\n');
    output.scrollTop = output.scrollHeight;
    renderTests();
    updateRunUI(snap.running);
  });

  eventSource.onmessage = (e) => {
    // Default event = raw line
    output.textContent += e.data + '\n';
    if (output.textContent.length > 200000) {
      output.textContent = output.textContent.slice(-150000);
    }
    output.scrollTop = output.scrollHeight;
  };

  eventSource.addEventListener('test_start', (e) => {
    const t = JSON.parse(e.data);
    testStates[t.name] = t;
    if (!testOrder.includes(t.name)) testOrder.push(t.name);
    renderTests();
  });

  eventSource.addEventListener('test_update', (e) => {
    const t = JSON.parse(e.data);
    testStates[t.name] = t;
    renderTests();
  });

  eventSource.addEventListener('done', (e) => {
    const status = JSON.parse(e.data);
    eventSource.close(); eventSource = null;
    updateRunUI(false, status);
    loadArtifacts();
  });

  eventSource.onerror = () => {
    // Auto-reconnect handled by browser EventSource
  };
}

// ── Helpers ──────────────────────────────────────────────────────────
async function fetchJson(url) {
  try { const r = await fetch(url); return await r.json(); }
  catch (e) { return {}; }
}

function setSelectVal(id, val) {
  const sel = document.getElementById(id);
  if (val && ![...sel.options].some(o => o.value === val)) {
    const opt = document.createElement('option');
    opt.value = val; opt.textContent = val;
    sel.insertBefore(opt, sel.firstChild);
  }
  sel.value = val;
}

function toast(msg, isError) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = 'toast show' + (isError ? ' error' : '');
  setTimeout(() => el.className = 'toast', 2500);
}

document.addEventListener('keydown', (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key === 's') { e.preventDefault(); saveConfig(); }
});

window.addEventListener('beforeunload', (e) => {
  if (dirty) { e.preventDefault(); e.returnValue = ''; }
});

// Periodic refresh of "elapsed" on running tests
setInterval(() => {
  const anyRunning = Object.values(testStates).some(t => t.status === 'running');
  if (anyRunning) renderTests();
}, 1000);

init();
</script>
</body>
</html>
"""


# ── Main ───────────────────────────────────────────────────────────────

def main():
    if '--help' in sys.argv or '-h' in sys.argv:
        print(f'Usage: {sys.argv[0]} [--port PORT]')
        print(f'  ESP-Hosted test configuration UI (web-based)')
        print(f'  Default port: {PORT}')
        sys.exit(0)

    # Handle Ctrl+C gracefully
    def shutdown(signum, frame):
        if ConfigUIHandler.run_state.process:
            try:
                os.killpg(os.getpgid(ConfigUIHandler.run_state.process.pid), signal.SIGTERM)
            except (ProcessLookupError, OSError):
                pass
        print('\nShutting down...')
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    class QuietServer(http.server.ThreadingHTTPServer):
        """Suppress noisy tracebacks for common benign client disconnects."""
        allow_reuse_address = True

        def handle_error(self, request, client_address):
            import sys as _sys
            exc = _sys.exc_info()[1]
            if isinstance(exc, (ConnectionResetError, BrokenPipeError, ConnectionAbortedError)):
                return  # expected when SSE clients navigate/close
            super().handle_error(request, client_address)

    server = QuietServer(('', PORT), ConfigUIHandler)
    url = f'http://localhost:{PORT}'
    print(f'ESP-Hosted Config UI: {url}')
    print(f'  Config: {ENV_FILE}')
    print(f'  Matrix: {MATRIX_FILE}')
    print(f'  Press Ctrl+C to stop\n')

    # Auto-open browser (slight delay for server to start)
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    server.serve_forever()


if __name__ == '__main__':
    main()
