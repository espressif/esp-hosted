"""
Flash orchestration for dual-DUT (host P4 + slave C6).

Optimization layers (all configurable via flash_cfg):
  1. Hash-based skip: if binary + layout unchanged → just reset (no flash)
  2. App-only flash: layout unchanged but app changed → skip bootloader + PT
  3. esptool --diff-with: sector-level diff against last-flashed binary
  4. Flash strategies:
     - sequential: CP then host (current, safest)
     - parallel: both simultaneously in threads (fastest, ~35% savings)
     - staggered: CP first, host delayed so both finish together
  5. Calibrated timing: measures actual throughput, uses it next run

Safety invariant: host always uses --after no_reset during flash. Host doesn't
boot until the runner's explicit _just_reset(host_port). Therefore parallel
flash completion order is irrelevant — host cannot reset CP mid-flash.
"""

import hashlib
import json
import os
import shutil
import subprocess
import threading
import time

# Default flash baud (fallback if not in flash_cfg)
FLASH_BAUD = int(os.environ.get('FLASH_BAUD', '2000000'))

# Cache file — persists hashes + calibration data across runs
_FLASH_CACHE_FILE = '/tmp/eh_test_flash_cache.json'

# Strategy constants
STRATEGY_SEQUENTIAL = 'sequential'
STRATEGY_PARALLEL = 'parallel'
STRATEGY_STAGGERED = 'staggered'
VALID_STRATEGIES = (STRATEGY_SEQUENTIAL, STRATEGY_PARALLEL, STRATEGY_STAGGERED)

# Timing defaults
DEFAULT_OVERHEAD_S = 3.0
DEFAULT_COMPRESSION_RATIO = 0.65
DEFAULT_TIMEOUT_MULTIPLIER = 2.0
MIN_TIMEOUT_S = 30

# Filename for diff-flash source binary (copied after each flash)
_LAST_FLASHED_NAME = '.last_flashed.bin'


# ── Shell helpers ──────────────────────────────────────────────────────

def _run(cmd, cwd=None, timeout=120):
    try:
        r = subprocess.run(
            cmd, shell=True, cwd=cwd,
            capture_output=True, text=True, timeout=timeout
        )
        return r.returncode == 0, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return False, 'Timeout'
    except Exception as e:
        return False, str(e)


def _get_idf_python():
    env_path = os.environ.get('IDF_PYTHON_ENV_PATH', '')
    p = os.path.join(env_path, 'bin', 'python')
    return p if os.path.exists(p) else 'python'


def _esptool(port, chip, args, cwd=None, timeout=120, baud=None):
    """Run esptool with given args."""
    py = _get_idf_python()
    b = baud or FLASH_BAUD
    cmd = f'{py} -m esptool --chip {chip} --port {port} -b {b} {args}'
    return _run(cmd, cwd=cwd, timeout=timeout)


# ── Cache with calibration ─────────────────────────────────────────────

def _load_flash_cache():
    try:
        with open(_FLASH_CACHE_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _save_flash_cache(cache):
    try:
        with open(_FLASH_CACHE_FILE, 'w') as f:
            json.dump(cache, f, indent=2)
    except OSError:
        pass


def _cached_field(cache, key, field):
    """Get a field from cache entry. Handles legacy (plain string) entries."""
    entry = cache.get(key)
    if isinstance(entry, dict):
        return entry.get(field)
    # Legacy cache: plain string = app hash
    return entry if field == 'app' else None


def _record_calibration(cache, port_key, elapsed_s, total_bytes, overhead_s, bin_path=None):
    """Record measured throughput + last binary path after successful flash."""
    entry = cache.get(port_key) or {}
    if not isinstance(entry, dict):
        entry = {'app': entry}
    transfer_time = max(elapsed_s - overhead_s, 0.1)
    if total_bytes > 0:
        entry['last_flash_s'] = round(elapsed_s, 2)
        entry['bytes_flashed'] = total_bytes
        entry['measured_throughput_bps'] = int(total_bytes / transfer_time)
    if bin_path:
        entry['last_bin_path'] = bin_path
    cache[port_key] = entry


# ── Hash helpers ───────────────────────────────────────────────────────

def _hash_build_dir(build_dir):
    """Hash app binary to detect app changes."""
    if not os.path.isdir(build_dir):
        return None
    app_bin = None
    for f in os.listdir(build_dir):
        if f.endswith('.bin') and not f.startswith('partition') and not f.startswith('bootloader'):
            app_bin = os.path.join(build_dir, f)
            break
    if not app_bin or not os.path.exists(app_bin):
        return None
    h = hashlib.md5()
    with open(app_bin, 'rb') as f:
        for chunk in iter(lambda: f.read(8192), b''):
            h.update(chunk)
    return h.hexdigest()


def _hash_layout(build_dir):
    """Hash partition table + flash geometry. Layout change → full flash needed."""
    if not os.path.isdir(build_dir):
        return None
    h = hashlib.md5()
    pt_bin = os.path.join(build_dir, 'partition_table', 'partition-table.bin')
    if os.path.exists(pt_bin):
        with open(pt_bin, 'rb') as f:
            h.update(f.read())
    fa = os.path.join(build_dir, 'flash_args')
    if os.path.exists(fa):
        with open(fa) as f:
            first_line = f.readline().strip()
        h.update(first_line.encode())
    return h.hexdigest()


# ── Flash-args parsing ─────────────────────────────────────────────────

def _parse_flash_args(build_dir):
    """Parse flash_args → list of (offset, bin_name) tuples."""
    flash_args = os.path.join(build_dir, 'flash_args')
    if not os.path.exists(flash_args):
        return []
    with open(flash_args) as f:
        tokens = f.read().strip().split()
    pairs = []
    i = 0
    while i < len(tokens):
        if tokens[i].startswith('0x') and i + 1 < len(tokens) and not tokens[i+1].startswith('-'):
            pairs.append((tokens[i], tokens[i+1]))
            i += 2
        else:
            i += 1
    return pairs


def _app_only_flash_args(build_dir):
    """Get just the app binary's (offset, name). Skips BL, PT, OTA data."""
    for offset, name in _parse_flash_args(build_dir):
        if 'bootloader' not in name and 'partition' not in name and 'ota_data' not in name:
            return (offset, name)
    return None


def _get_flash_bytes(build_dir, app_only=False):
    """Total bytes to flash (for timing estimation)."""
    if not os.path.isdir(build_dir):
        return 0
    if app_only:
        pair = _app_only_flash_args(build_dir)
        if pair:
            bin_path = os.path.join(build_dir, pair[1])
            return os.path.getsize(bin_path) if os.path.exists(bin_path) else 0
        return 0
    total = 0
    for _, bin_name in _parse_flash_args(build_dir):
        bin_path = os.path.join(build_dir, bin_name)
        if os.path.exists(bin_path):
            total += os.path.getsize(bin_path)
    return total


def _get_app_bin_path(build_dir):
    """Absolute path to app binary (for --diff-with tracking)."""
    pair = _app_only_flash_args(build_dir)
    if pair:
        p = os.path.join(build_dir, pair[1])
        return p if os.path.exists(p) else None
    return None


# ── Timing estimation ──────────────────────────────────────────────────

def _estimate_flash_time(total_bytes, baud, overhead_s=DEFAULT_OVERHEAD_S,
                         compression_ratio=DEFAULT_COMPRESSION_RATIO,
                         throughput_bps=None):
    """Estimate flash time.

    Formula: time = overhead + (bytes * compression_ratio) / throughput
    - If throughput_bps given (from calibration), use it
    - Else use theoretical: baud / 10 bytes/sec
    """
    if total_bytes == 0:
        return overhead_s
    if throughput_bps and throughput_bps > 0:
        return overhead_s + (total_bytes * compression_ratio) / throughput_bps
    byte_rate = baud / 10  # 10 bits per byte (8 data + start + stop)
    return overhead_s + (total_bytes * compression_ratio) / byte_rate


def _calibrated_estimate(cache, port_key, total_bytes, baud, overhead_s, compression_ratio):
    """Use calibrated throughput if available."""
    throughput = _cached_field(cache, port_key, 'measured_throughput_bps')
    source = 'calibrated' if throughput else 'theoretical'
    est = _estimate_flash_time(total_bytes, baud, overhead_s, compression_ratio, throughput)
    return est, source


def _compute_stagger_delay(cp_est, host_est, max_delay=10.0):
    """Start host after delay so both flashes finish together.
    If CP is longer → delay host. Otherwise → no delay (host is the longer one)."""
    delay = max(0, cp_est - host_est)
    return min(delay, max_delay)


# ── Core flash operations ──────────────────────────────────────────────

def _just_reset(port):
    """Reset device without flashing."""
    py = _get_idf_python()
    ok, out = _run(
        f'{py} -m esptool --port {port} --before default_reset --after hard_reset run',
        timeout=10
    )
    return ok


def put_host_in_bootloader(host_port):
    """Put host in bootloader mode. Retries once on failure (serial port race)."""
    py = _get_idf_python()
    cmd = (f'{py} -m esptool --port {host_port} '
           f'--before default_reset --after no_reset run')
    ok, out = _run(cmd, timeout=15)
    if not ok:
        time.sleep(2)
        ok, out = _run(cmd, timeout=15)
        if not ok:
            return False, f'Failed to put host in bootloader: {out[-200:]}'
    time.sleep(0.3)
    return True, 'Host in bootloader'


def _build_diff_args(build_dir, app_only, bin_name, diff_path, trust_content):
    """Build --diff-with / --trust-flash-content flags for esptool."""
    if not diff_path or not os.path.exists(diff_path):
        return ''
    # --diff-with is applied to the file being flashed. Must reference by same
    # name as the file arg. We pass relative path if diff is in same dir.
    flags = f'--diff-with {diff_path} '
    if trust_content:
        flags += '--trust-flash-content '
    return flags


def _flash_device(port, chip, build_dir, app_only=False, force=False,
                  after='no_reset', baud=None, timeout=120,
                  diff_path=None, trust_content=False):
    """Flash a device. Returns (ok, msg, bytes_flashed)."""
    force_flag = '--force ' if force else ''
    total_bytes = _get_flash_bytes(build_dir, app_only)

    if app_only:
        app_pair = _app_only_flash_args(build_dir)
        if app_pair:
            offset, bin_name = app_pair
            diff_flags = _build_diff_args(build_dir, True, bin_name, diff_path, trust_content)
            ok, out = _esptool(
                port, chip,
                f'--before default_reset --after {after} write_flash {force_flag}'
                f'--compress {diff_flags}{offset} {bin_name}',
                cwd=build_dir, timeout=timeout, baud=baud
            )
            if ok:
                return True, 'app flashed', total_bytes

    flash_args = os.path.join(build_dir, 'flash_args')
    if not os.path.exists(flash_args):
        return False, f'flash_args not found in {build_dir}', 0

    # Full flash — diff-with doesn't apply to @flash_args mode
    ok, out = _esptool(
        port, chip,
        f'--before default_reset --after {after} write_flash {force_flag}--compress @flash_args',
        cwd=build_dir, timeout=timeout, baud=baud
    )
    if not ok:
        return False, f'Flash failed: {out[-300:]}', 0
    return True, 'flashed', total_bytes


def pre_flash_action(port, chip, address, file_path, baud=None):
    """Write a binary to a specific flash address (e.g., OTA partition)."""
    if not os.path.exists(file_path):
        return False, f'File not found: {file_path}'

    ok, out = _esptool(
        port, chip,
        f'--before default_reset --after no_reset write_flash --force {address} {file_path}',
        cwd=os.path.dirname(file_path) or None, timeout=120, baud=baud
    )
    if not ok:
        return False, f'Pre-flash failed: {out[-300:]}'
    return True, f'Wrote {os.path.basename(file_path)} to {address}'


def _save_last_flashed(build_dir, app_only):
    """Copy app binary to .last_flashed.bin for next run's --diff-with."""
    if not app_only:
        # For full flash, we'd need all binaries — just track the app for now
        pair = _app_only_flash_args(build_dir)
    else:
        pair = _app_only_flash_args(build_dir)
    if not pair:
        return None
    src = os.path.join(build_dir, pair[1])
    if not os.path.exists(src):
        return None
    dst = os.path.join(build_dir, _LAST_FLASHED_NAME)
    try:
        shutil.copy2(src, dst)
        return dst
    except OSError:
        return None


# ── Parallel flash internals ───────────────────────────────────────────

class _FlashResult:
    """Thread-safe container for flash result."""
    __slots__ = ('ok', 'msg', 'elapsed', 'bytes_flashed')

    def __init__(self):
        self.ok = False
        self.msg = ''
        self.elapsed = 0.0
        self.bytes_flashed = 0


def _flash_thread(result, port, chip, build_dir, app_only, force, after,
                  baud, timeout, diff_path, trust_content):
    """Target function for flash thread. Writes to result object."""
    t = time.time()
    try:
        ok, msg, n_bytes = _flash_device(
            port, chip, build_dir,
            app_only=app_only, force=force, after=after,
            baud=baud, timeout=timeout,
            diff_path=diff_path, trust_content=trust_content
        )
        result.ok = ok
        result.msg = msg
        result.bytes_flashed = n_bytes
    except Exception as e:
        result.ok = False
        result.msg = f'Exception: {e}'
    result.elapsed = time.time() - t


# ── Strategy implementations ───────────────────────────────────────────

def _flash_sequential(params, log):
    """CP then host, sequential. Uses --after hard_reset on CP, no_reset on host."""
    cp = params
    # CP flash
    if cp['need_cp_flash']:
        t = time.time()
        ok, msg, n_bytes = _flash_device(
            cp['slave_port'], cp['slave_chip'], cp['cp_build_dir'],
            app_only=cp['cp_app_only'], force=False, after='hard_reset',
            baud=cp['baud'], timeout=cp['cp_timeout'],
            diff_path=cp['cp_diff_path'], trust_content=cp['trust_content']
        )
        elapsed = time.time() - t
        log.append(f'[2] CP {msg} ({elapsed:.1f}s)')
        if not ok:
            return False, None, None
        cp_result = _FlashResult()
        cp_result.ok = ok; cp_result.msg = msg; cp_result.elapsed = elapsed; cp_result.bytes_flashed = n_bytes
    else:
        cp_result = None
        log.append('[2] CP unchanged, skip')

    # Pre-flash actions (between CP and host)
    for action in cp.get('pre_flash_actions') or []:
        t = time.time()
        port = cp['host_port'] if action.get('port') == 'host' else cp['slave_port']
        chip = cp['host_chip'] if action.get('port') == 'host' else cp['slave_chip']
        file_path = action['file'].format(cp_build=cp['cp_build_dir'])
        ok, msg = pre_flash_action(port, chip, action['address'], file_path, baud=cp['baud'])
        log.append(f'[3] {msg} ({time.time()-t:.1f}s)')
        if not ok:
            return False, cp_result, None

    # Host flash
    if cp['need_host_flash']:
        t = time.time()
        ok, msg, n_bytes = _flash_device(
            cp['host_port'], cp['host_chip'], cp['host_build_dir'],
            app_only=cp['host_app_only'], force=True, after='no_reset',
            baud=cp['baud'], timeout=cp['host_timeout'],
            diff_path=cp['host_diff_path'], trust_content=cp['trust_content']
        )
        elapsed = time.time() - t
        log.append(f'[4] Host {msg} ({elapsed:.1f}s)')
        if not ok:
            return False, cp_result, None
        host_result = _FlashResult()
        host_result.ok = ok; host_result.msg = msg; host_result.elapsed = elapsed; host_result.bytes_flashed = n_bytes
    else:
        host_result = None
        log.append('[4] Host unchanged, hold in bootloader')
        put_host_in_bootloader(cp['host_port'])

    return True, cp_result, host_result


def _flash_parallel(params, strategy, log):
    """Parallel or staggered. Both use --after no_reset (host doesn't boot)."""
    cp = params
    need_cp = cp['need_cp_flash']
    need_host = cp['need_host_flash']

    # If only one side needs flashing, fall back to sequential
    if not (need_cp and need_host):
        log.append('[2-4] Only one side needs flash — using sequential path')
        return _flash_sequential(params, log)

    # Estimate times for stagger delay
    cp_est = _estimate_flash_time(
        _get_flash_bytes(cp['cp_build_dir'], cp['cp_app_only']),
        cp['baud'], cp['overhead_s'], cp['compression_ratio'],
        cp.get('cp_throughput_bps'))
    host_est = _estimate_flash_time(
        _get_flash_bytes(cp['host_build_dir'], cp['host_app_only']),
        cp['baud'], cp['overhead_s'], cp['compression_ratio'],
        cp.get('host_throughput_bps'))

    log.append(f'[est] CP ~{cp_est:.1f}s, Host ~{host_est:.1f}s')

    # Shared timeout for both threads
    max_est = max(cp_est, host_est)
    join_timeout = max(max_est * cp['timeout_multiplier'], MIN_TIMEOUT_S)

    cp_result = _FlashResult()
    host_result = _FlashResult()

    # Both use --after no_reset so host won't boot and reset CP
    cp_thread = threading.Thread(
        target=_flash_thread,
        args=(cp_result, cp['slave_port'], cp['slave_chip'], cp['cp_build_dir'],
              cp['cp_app_only'], False, 'no_reset',
              cp['baud'], int(join_timeout),
              cp['cp_diff_path'], cp['trust_content']),
        daemon=True
    )
    host_thread = threading.Thread(
        target=_flash_thread,
        args=(host_result, cp['host_port'], cp['host_chip'], cp['host_build_dir'],
              cp['host_app_only'], True, 'no_reset',
              cp['baud'], int(join_timeout),
              cp['host_diff_path'], cp['trust_content']),
        daemon=True
    )

    t_start = time.time()
    if strategy == STRATEGY_STAGGERED:
        stagger = _compute_stagger_delay(cp_est, host_est)
        log.append(f'[stagger] CP starts first, host delayed {stagger:.1f}s')
        cp_thread.start()
        if stagger > 0:
            time.sleep(stagger)
        host_thread.start()
    else:
        cp_thread.start()
        host_thread.start()

    cp_thread.join(timeout=join_timeout)
    host_thread.join(timeout=join_timeout)
    elapsed = time.time() - t_start

    # Handle timeouts (thread still alive)
    if cp_thread.is_alive():
        cp_result.ok = False
        cp_result.msg = f'Timeout ({join_timeout:.0f}s)'
    if host_thread.is_alive():
        host_result.ok = False
        host_result.msg = f'Timeout ({join_timeout:.0f}s)'

    log.append(f'[2] CP {cp_result.msg} ({cp_result.elapsed:.1f}s)')
    log.append(f'[4] Host {host_result.msg} ({host_result.elapsed:.1f}s)')
    log.append(f'[parallel] Total {elapsed:.1f}s '
               f'(vs ~{cp_result.elapsed + host_result.elapsed:.1f}s sequential)')

    # Fallback: if either failed, retry sequentially (USB contention suspected)
    if not cp_result.ok or not host_result.ok:
        which = 'both' if (not cp_result.ok and not host_result.ok) else \
                'CP' if not cp_result.ok else 'Host'
        log.append(f'[fallback] {which} parallel flash failed — retrying sequentially')
        return _flash_sequential(params, log)

    # Pre-flash actions AFTER both flashes completed (both in bootloader)
    for action in cp.get('pre_flash_actions') or []:
        t = time.time()
        port = cp['host_port'] if action.get('port') == 'host' else cp['slave_port']
        chip = cp['host_chip'] if action.get('port') == 'host' else cp['slave_chip']
        file_path = action['file'].format(cp_build=cp['cp_build_dir'])
        ok, msg = pre_flash_action(port, chip, action['address'], file_path, baud=cp['baud'])
        log.append(f'[3] {msg} ({time.time()-t:.1f}s)')
        if not ok:
            return False, cp_result, host_result

    return True, cp_result, host_result


# ── Main entry point ───────────────────────────────────────────────────

def eh_test_flash_pair(host_port, slave_port, host_chip, slave_chip,
                       cp_build_dir, host_build_dir,
                       pre_flash_actions=None, flash_cfg=None):
    """Flash CP + host with configurable strategy.

    flash_cfg keys (all optional):
      strategy: 'sequential' | 'parallel' | 'staggered'
      baud: int
      timeout_multiplier: float
      overhead_s: float
      compression_ratio: float
      use_diff: bool
      trust_flash_content: bool
      force_full_flash: bool  — skip hash-based optimization, always full flash
    """
    log = []
    total_start = time.time()

    cfg = flash_cfg or {}
    strategy = cfg.get('strategy', STRATEGY_SEQUENTIAL)
    if strategy not in VALID_STRATEGIES:
        log.append(f'[warn] Unknown strategy "{strategy}", using sequential')
        strategy = STRATEGY_SEQUENTIAL
    baud = cfg.get('baud', FLASH_BAUD)
    multiplier = cfg.get('timeout_multiplier', DEFAULT_TIMEOUT_MULTIPLIER)
    overhead_s = cfg.get('overhead_s', DEFAULT_OVERHEAD_S)
    compression_ratio = cfg.get('compression_ratio', DEFAULT_COMPRESSION_RATIO)
    use_diff = cfg.get('use_diff', False)
    trust_content = cfg.get('trust_flash_content', False)
    force_full = cfg.get('force_full_flash', False)

    # Load cache
    cache = _load_flash_cache()
    cp_hash = _hash_build_dir(cp_build_dir)
    host_hash = _hash_build_dir(host_build_dir)
    cp_layout = _hash_layout(cp_build_dir)
    host_layout = _hash_layout(host_build_dir)

    cp_port_key = f'cp:{slave_port}'
    host_port_key = f'host:{host_port}'

    if force_full:
        # Bypass all hash-based optimization — always flash everything
        cp_unchanged = False
        host_unchanged = False
        log.append('[force] force_full_flash: skipping hash-based optimization')
    else:
        # Skip check
        cp_unchanged = (cp_hash and _cached_field(cache, cp_port_key, 'app') == cp_hash
                        and cp_layout and _cached_field(cache, cp_port_key, 'layout') == cp_layout
                        and not pre_flash_actions)
        host_unchanged = (host_hash and _cached_field(cache, host_port_key, 'app') == host_hash
                          and host_layout and _cached_field(cache, host_port_key, 'layout') == host_layout
                          and not pre_flash_actions)

    if cp_unchanged and host_unchanged:
        log.append('[skip] Binaries unchanged, reset only')
        _just_reset(host_port)
        log.append(f'[total] Flash: {time.time()-total_start:.1f}s')
        return True, log

    # App-only possible only when layout unchanged (disabled by force_full)
    if force_full:
        cp_app_only = False
        host_app_only = False
    else:
        cp_layout_same = (cp_layout and _cached_field(cache, cp_port_key, 'layout') == cp_layout)
        host_layout_same = (host_layout and _cached_field(cache, host_port_key, 'layout') == host_layout)
        cp_app_only = (cp_layout_same and not cp_unchanged)
        host_app_only = (host_layout_same and not host_unchanged)

    need_cp_flash = not cp_unchanged
    need_host_flash = not host_unchanged

    # Compute timeouts based on estimated flash time
    def _timeout_for(bd, app_only, port_key):
        n_bytes = _get_flash_bytes(bd, app_only)
        est, _ = _calibrated_estimate(cache, port_key, n_bytes, baud, overhead_s, compression_ratio)
        return max(int(est * multiplier), MIN_TIMEOUT_S)

    cp_timeout = _timeout_for(cp_build_dir, cp_app_only, cp_port_key)
    host_timeout = _timeout_for(host_build_dir, host_app_only, host_port_key)

    # Diff-flash: use previously-flashed binary if available
    def _diff_path(bd, app_only, port_key):
        if not use_diff or not app_only:
            return None
        last = _cached_field(cache, port_key, 'last_bin_path')
        if last and os.path.exists(last):
            return last
        return None

    cp_diff_path = _diff_path(cp_build_dir, cp_app_only, cp_port_key)
    host_diff_path = _diff_path(host_build_dir, host_app_only, host_port_key)

    # Step 1: Put host in bootloader (required before CP flash)
    if need_cp_flash or strategy != STRATEGY_SEQUENTIAL:
        t = time.time()
        ok, msg = put_host_in_bootloader(host_port)
        log.append(f'[1] {msg} ({time.time()-t:.1f}s)')
        if not ok:
            return False, log

    # Package params for strategy functions
    params = {
        'host_port': host_port, 'slave_port': slave_port,
        'host_chip': host_chip, 'slave_chip': slave_chip,
        'cp_build_dir': cp_build_dir, 'host_build_dir': host_build_dir,
        'pre_flash_actions': pre_flash_actions,
        'cp_app_only': cp_app_only, 'host_app_only': host_app_only,
        'need_cp_flash': need_cp_flash, 'need_host_flash': need_host_flash,
        'baud': baud, 'overhead_s': overhead_s,
        'compression_ratio': compression_ratio,
        'timeout_multiplier': multiplier,
        'cp_timeout': cp_timeout, 'host_timeout': host_timeout,
        'cp_diff_path': cp_diff_path, 'host_diff_path': host_diff_path,
        'trust_content': trust_content,
        'cp_throughput_bps': _cached_field(cache, cp_port_key, 'measured_throughput_bps'),
        'host_throughput_bps': _cached_field(cache, host_port_key, 'measured_throughput_bps'),
    }

    # Run the chosen strategy
    if strategy == STRATEGY_SEQUENTIAL:
        ok, cp_result, host_result = _flash_sequential(params, log)
    else:
        ok, cp_result, host_result = _flash_parallel(params, strategy, log)

    if not ok:
        log.append(f'[total] Flash: {time.time()-total_start:.1f}s (FAILED)')
        return False, log

    # Update cache with calibration + diff source
    if cp_result and cp_result.ok:
        cp_bin = _get_app_bin_path(cp_build_dir)
        # Save a copy for next run's --diff-with
        saved = _save_last_flashed(cp_build_dir, cp_app_only) if use_diff else None
        entry = cache.get(cp_port_key) or {}
        if not isinstance(entry, dict):
            entry = {}
        entry['app'] = cp_hash
        entry['layout'] = cp_layout
        cache[cp_port_key] = entry
        _record_calibration(cache, cp_port_key, cp_result.elapsed,
                            cp_result.bytes_flashed, overhead_s,
                            bin_path=saved or cp_bin)

    if host_result and host_result.ok:
        host_bin = _get_app_bin_path(host_build_dir)
        saved = _save_last_flashed(host_build_dir, host_app_only) if use_diff else None
        entry = cache.get(host_port_key) or {}
        if not isinstance(entry, dict):
            entry = {}
        entry['app'] = host_hash
        entry['layout'] = host_layout
        cache[host_port_key] = entry
        _record_calibration(cache, host_port_key, host_result.elapsed,
                            host_result.bytes_flashed, overhead_s,
                            bin_path=saved or host_bin)

    _save_flash_cache(cache)
    log.append(f'[total] Flash: {time.time()-total_start:.1f}s')
    return True, log
