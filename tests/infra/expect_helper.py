"""
Smart expect — leverages pytest-embedded's native capabilities.

Uses:
  - dut.expect(pattern, not_matching=fail_patterns) for bifurcation
  - dut.expect_exact() for literal string matching (faster)
  - Minimal timeouts, fast-fail on crash/error patterns
  - Post-timeout diagnostics: scans serial buffer for common failure
    signatures so the error message says *why* it timed out
"""

import os
import re
import sys
import time


# ── Checkpoint logging ─────────────────────────────────────────────────
# When EH_CHECKPOINT_LOG env var is set to a file path, every expect call
# appends a one-line summary to that file.  The runner sets this per-pair,
# and the artifact manager decides whether to keep/discard the file based
# on the configured log_retention mode.

_CHECKPOINT_LOG_ENV = 'EH_CHECKPOINT_LOG'


def _log_checkpoint(wanted, elapsed, ok, matched_text=None):
    path = os.environ.get(_CHECKPOINT_LOG_ENV)
    if not path:
        return
    status = 'MATCH' if ok else 'FAIL '
    w = wanted if isinstance(wanted, str) else repr(wanted)
    w = w.replace('\n', ' ').replace('\r', ' ')
    if len(w) > 80:
        w = w[:77] + '...'
    ts = time.strftime('%H:%M:%S')
    line = f'{ts} {status} [{elapsed:6.2f}s] want={w!r}'
    if not ok and matched_text:
        mt = str(matched_text).replace('\n', ' ')[:120]
        line += f' got={mt!r}'
    try:
        with open(path, 'a') as f:
            f.write(line + '\n')
    except OSError:
        pass


class EhTestExpectResult:
    __slots__ = ('ok', 'matched', 'elapsed')

    def __init__(self, ok, matched, elapsed):
        self.ok = ok
        self.matched = matched
        self.elapsed = elapsed

    def __repr__(self):
        return f'<{"OK" if self.ok else "FAIL"} "{self.matched}" {self.elapsed:.1f}s>'


FATAL_PATTERNS = [
    'Guru Meditation Error',
    'assert failed',
    'panic',
    'abort()',
    'LoadProhibited',
    'StoreProhibited',
    'Illegal instruction',
    'Stack smashing',
]

TRANSPORT_FAIL_PATTERNS = [
    'sdmmc_card_init failed',
    'card init failed',
    'ESP-Hosted link not yet up',
    'ensure_slave_bus_ready failed',
]

# Diagnostic patterns: checked on timeout to explain *why* it timed out.
# Each entry: (search_string, human-readable diagnosis)
_TIMEOUT_DIAGNOSTICS = [
    ('not bootable',
     'Boot loop: app partition not bootable (corrupt flash? flash size mismatch?)'),
    ('invalid segment length',
     'Boot loop: corrupt app image (truncated binary or wrong flash geometry)'),
    ('No bootable app partitions',
     'Boot loop: no bootable partition found'),
    ('rst:0x3 (SW_SYS_RESET)',
     'Device in reset loop'),
    ('WIFI_EVENT_STA_DISCONNECTED',
     None),  # handled specially — extract reason code
    ('connect to the AP fail',
     'WiFi: connect retries exhausted — AP unreachable or wrong credentials'),
    ('Partition.*not found',
     None),  # handled specially — extract partition name
    ('Cannot establish a connection to the component registry',
     'Build env: component registry unreachable (no internet?)'),
    ('OTA failed',
     'OTA procedure failed'),
    ('host power save is started',
     'CP console silent — likely host in deep sleep, SDIO down, CP output stalled'),
    ('Inform slave: Host PS start',
     'CP console silent — likely host in deep sleep, SDIO down, CP output stalled'),
]


def _diagnose_timeout(buf, elapsed, wanted):
    """Scan the post-expect buffer (forward-only) for failure signatures.

    Caller must pass ONLY the content captured during this expect call —
    not the full session history — so stale messages from earlier phases
    don't trigger false diagnostics.
    See eh_test_expect: it computes `post[pre_len:]` before calling this."""
    if not buf:
        return f'Timeout ({elapsed:.1f}s): {wanted}'

    buf_lower = buf.lower()

    # Check each diagnostic pattern
    for pattern, diagnosis in _TIMEOUT_DIAGNOSTICS:
        if pattern.lower() in buf_lower:
            if diagnosis:
                return f'{diagnosis} [wanted: {wanted}]'

            # Special case: WiFi disconnect — extract reason code
            if 'DISCONNECTED' in pattern:
                m = re.search(r'reason[:\s]+(\d+)', buf)
                reasons = {
                    '2': 'AUTH_EXPIRE', '15': 'FOURWAY_HANDSHAKE_TIMEOUT',
                    '201': 'NO_AP_FOUND', '202': 'AUTH_FAIL',
                    '203': 'ASSOC_FAIL', '204': 'HANDSHAKE_TIMEOUT',
                }
                if m:
                    code = m.group(1)
                    name = reasons.get(code, 'unknown')
                    return f'WiFi disconnect loop: reason {code} ({name}) [wanted: {wanted}]'
                return f'WiFi disconnect loop [wanted: {wanted}]'

            # Special case: partition not found — extract name
            if 'not found' in pattern.lower():
                m = re.search(r"[Pp]artition '(\w+)' not found", buf)
                if m:
                    return f"Partition '{m.group(1)}' not found [wanted: {wanted}]"

    return f'Timeout ({elapsed:.1f}s): {wanted}'


def _get_serial_buffer(dut):
    """Extract current pexpect `before` buffer as a string.

    `before` contains data read by pexpect up to (but not matching) the
    current pattern.  Across multiple timeouts without a match, this buffer
    accumulates — which is why callers must snapshot its length BEFORE the
    expect and diagnose only the delta (forward-only)."""
    try:
        buf = getattr(dut.pexpect_proc, 'before', b'') or b''
        if isinstance(buf, bytes):
            buf = buf.decode('utf-8', errors='replace')
        return buf
    except Exception:
        return ''


def _expect_impl(dut, success, fail, timeout, use_exact):
    """Shared implementation for expect / expect_exact.

    Forward-only semantics: captures buffer length before the call, and
    diagnoses only the content that arrived DURING this expect.  Stale
    data from earlier expects never contaminates the diagnosis."""
    if fail is None:
        fail = FATAL_PATTERNS

    # Snapshot buffer length BEFORE the call — diagnostics only look at
    # content captured during this specific expect.
    pre_len = len(_get_serial_buffer(dut))

    start = time.time()
    try:
        if use_exact:
            dut.expect_exact(success, not_matching=fail, timeout=timeout)
        else:
            dut.expect(success, not_matching=fail, timeout=timeout)
        elapsed = time.time() - start
        _log_checkpoint(success, elapsed, True)
        return EhTestExpectResult(True, success, elapsed)
    except Exception as e:
        elapsed = time.time() - start
        err = str(e)
        if 'not_matching' in err.lower() or 'unexpected' in err.lower():
            _log_checkpoint(success, elapsed, False, f'FATAL: {err[:80]}')
            return EhTestExpectResult(False, f'FATAL: {err[:200]}', elapsed)

        # Forward-only: only content that arrived during THIS expect
        post = _get_serial_buffer(dut)
        recent = post[pre_len:] if len(post) >= pre_len else post

        for fp in fail:
            if fp.lower() in recent.lower():
                _log_checkpoint(success, elapsed, False, f'FATAL: {fp}')
                return EhTestExpectResult(False, f'FATAL: {fp}', elapsed)
        diag = _diagnose_timeout(recent, elapsed, success)
        _log_checkpoint(success, elapsed, False, diag)
        return EhTestExpectResult(False, diag, elapsed)


def eh_test_expect(dut, success, fail=None, timeout=15):
    """Native pytest-embedded expect with fail-pattern bifurcation.
    Forward-only timeout diagnostics (see _expect_impl)."""
    return _expect_impl(dut, success, fail, timeout, use_exact=False)


def eh_test_expect_exact(dut, success, fail=None, timeout=15):
    """Same as eh_test_expect but literal match (faster, no regex)."""
    return _expect_impl(dut, success, fail, timeout, use_exact=True)


def eh_test_verify_transport(host_dut, timeout=15):
    return eh_test_expect_exact(
        host_dut, 'Identified slave',
        fail=FATAL_PATTERNS + TRANSPORT_FAIL_PATTERNS,
        timeout=timeout
    )


def eh_test_verify_cp_boot(cp_dut, timeout=15):
    # Matches both old CP ("Start Data Path") and new V1 CP ("Open Data Path")
    return eh_test_expect(cp_dut, r'(Start|Open) Data Path', fail=FATAL_PATTERNS, timeout=timeout)
