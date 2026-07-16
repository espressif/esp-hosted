"""Shared process-backed DUT handle for emulated / userspace benches.

Exposes the pytest-embedded Dut surface the HW helpers use (expect / expect_exact
/ pexpect_proc.before / write) over a subprocess (an esp-emu instance, or a Linux
host binary). One implementation, reused by the emu and linux substrate conftests.
"""
import logging
import os
import re
import subprocess
import threading
import time

import pytest

_log = logging.getLogger("eh")  # streamed live by pytest log_cli


def emu_bin():
    """Resolve the esp-emu binary via eh.py (override → .deps → legacy); skip the
    test if it isn't built. Shared by every emu-backed bench."""
    import importlib.util
    from infra import lab
    spec = importlib.util.spec_from_file_location("eh", lab._REPO / "tools" / "eh.py")
    eh = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(eh)
    d = eh.resolve_emu_dir()
    if not d:
        pytest.skip("esp-emu not found (run ./install.sh --with-emu or eh.py set-esp-emu)")
    b = eh.emu_binary(d)
    if not b.exists():
        pytest.skip(f"esp-emu not built at {b} (cargo build --release)")
    return b


class _Pexpect:
    """Minimal stand-in for pytest-embedded's pexpect_proc: exposes `.before`
    (unconsumed buffer as bytes) which infra.expect_helper reads on timeout."""
    def __init__(self, dut):
        self._dut = dut

    @property
    def before(self):
        return self._dut._unconsumed().encode('utf-8', errors='replace')


class EmuDut:
    """One subprocess behind the pytest-embedded Dut surface (expect / expect_exact
    / pexpect_proc.before / write). A background thread tees stdout untouched to
    `logpath` and into an in-memory buffer; expect() advances a consume cursor so
    `pexpect_proc.before` reflects only bytes since the last match (forward-only).

    esp-emu's own env_logger lines ([LEVEL module] …) are kept out of the expect
    buffer (a real serial DUT never sees them) but still teed raw to disk."""

    _EMU_LOG = re.compile(r'^\[(?:INFO|WARN|ERROR|DEBUG|TRACE)\b')

    def __init__(self, name, argv, logpath, env=None):
        self.name = name
        self._buf = ''
        self._cursor = 0
        self._lock = threading.Lock()
        self._fh = open(logpath, 'w')
        self.pexpect_proc = _Pexpect(self)
        _log.info("[emu] launch %s", name)
        self.proc = subprocess.Popen(
            argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout:
            self._fh.write(line)
            self._fh.flush()
            if self._EMU_LOG.match(line):
                continue
            with self._lock:
                self._buf += line

    def _unconsumed(self):
        with self._lock:
            return self._buf[self._cursor:]

    def write(self, s):
        _log.info("[cmd %s] %s", self.name, s.rstrip('\n'))
        self.proc.stdin.write(s if s.endswith('\n') else s + '\n')
        self.proc.stdin.flush()

    def _expect(self, success, not_matching, timeout, exact):
        fails = [not_matching] if isinstance(not_matching, str) else list(not_matching or [])
        wants = [success] if isinstance(success, str) else list(success)
        end = time.time() + timeout
        while time.time() < end and self.proc.poll() is None:
            text = self._unconsumed()
            for fp in fails:
                if (fp in text) if exact else re.search(fp, text):
                    raise RuntimeError(f'not_matching pattern hit: {fp!r}')
            for w in wants:
                idx = text.find(w) if exact else None
                m = None if exact else re.search(w, text)
                if (exact and idx >= 0) or m:
                    hit_end = (idx + len(w)) if exact else m.end()
                    with self._lock:
                        self._cursor += hit_end
                    return w
            time.sleep(0.05)
        raise TimeoutError(f'[{self.name}] timeout ({timeout}s) waiting for {success!r}')

    def expect(self, success, not_matching=None, timeout=15):
        return self._expect(success, not_matching, timeout, exact=False)

    def expect_exact(self, success, not_matching=None, timeout=15):
        return self._expect(success, not_matching, timeout, exact=True)

    def fatal(self):
        """First unambiguous crash signature in the full log, or None. Used for
        the post-success settle check (a test can pass its expects, then the
        firmware panics a moment later)."""
        with self._lock:
            text = self._buf
        for p in _FATAL:
            if p in text:
                return p
        return None

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self._fh.close()


# Full-buffer late-crash signatures — see infra.signatures for why bare 'panic'
# is excluded from this (settle) list but present in the expect list.
from infra.signatures import SETTLE_FATAL as _FATAL


def settle_check(*duts, window=None):
    """Post-success crash catch: wait a short settle window, then fail if any dut's
    log shows a crash signature. Window from EH_SETTLE_SEC (default 0.5s; 0 skips)."""
    import os
    if window is None:
        window = float(os.environ.get('EH_SETTLE_SEC', '0.5'))
    if window > 0:
        time.sleep(window)
    for d in duts:
        p = d.fatal()
        if p:
            raise AssertionError(f'[{d.name}] late crash after test passed: {p!r}')
