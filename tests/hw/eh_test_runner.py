#!/usr/bin/env python3
"""
ESP-Hosted Hardware Test Matrix Runner.

Usage:
    python eh_test_runner.py                          # Auto-detect HW, run all
    python eh_test_runner.py --suite boot_wifi,gpio   # Run specific suites
    python eh_test_runner.py --mode on-target         # Require hardware
    python eh_test_runner.py --mode simulated         # Simulated tests only
    python eh_test_runner.py --skip-build             # Use existing builds
    python eh_test_runner.py --skip-flash             # Skip flashing + building
    python eh_test_runner.py --list                   # List available suites
    python eh_test_runner.py --json results.json      # JSON report
    python eh_test_runner.py --dry-run                # Show plan, don't execute

Workspace: builds go to tests/workspace/<suite>/{cp,host}/ — examples stay clean.
"""

import argparse
import os
import re
import subprocess
import sys
import time
import json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from infra.hardware import eh_test_hw_probe_setup
from infra.flasher import eh_test_flash_pair
from infra.builder import eh_test_build_project, eh_test_build_apply_patch
from infra.reporter import EhTestReport, eh_test_print_pair_header, eh_test_print_phase, C
from infra.config import eh_test_config_load
from infra.artifacts import EhTestArtifactManager
from infra.backtrace import eh_test_decode_backtrace

SCRIPT_DIR = Path(__file__).parent
MATRIX_FILE = SCRIPT_DIR / 'eh_test_matrix.json'


def load_matrix():
    with open(MATRIX_FILE) as f:
        return json.load(f)


def ensure_idf_env(idf_path):
    """Source IDF export.sh if idf.py isn't already on PATH."""
    if os.system('which idf.py >/dev/null 2>&1') == 0:
        return

    idf_path = os.path.expanduser(idf_path)
    export_sh = os.path.join(idf_path, 'export.sh')
    if not os.path.exists(export_sh):
        print(f'ERROR: IDF not found at {idf_path}. Set paths.idf_path in env.json')
        sys.exit(1)

    print(f'  Sourcing IDF from {idf_path}...')
    cmd = f'bash -c ". {export_sh} >/dev/null 2>&1 && env -0"'
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, timeout=30)
        if result.returncode != 0:
            print(f'ERROR: Failed to source {export_sh}')
            sys.exit(1)
        for entry in result.stdout.split(b'\x00'):
            entry = entry.decode('utf-8', errors='replace')
            if '=' in entry:
                key, _, val = entry.partition('=')
                if key and not key.startswith(('BASH_FUNC', '_=')):
                    os.environ[key] = val
    except subprocess.TimeoutExpired:
        print(f'ERROR: Timeout sourcing {export_sh}')
        sys.exit(1)


def run_pytest(hw_cfg, test_files, host_ws, cp_ws, junit_path=None):
    """
    Run pytest-embedded with serial service (no esptool chip-detect).
    We flash externally, so we just need serial port monitoring.
    Returns (passed, failed, duration, output, fail_lines, warn_count).
    """
    host_port = os.environ.get('HOST_PORT', hw_cfg['host_port'])
    slave_port = os.environ.get('SLAVE_PORT', hw_cfg['slave_port'])

    # Ensure workspace build dirs exist (idf-ci build_dir fixture requires them)
    os.makedirs(os.path.join(host_ws, 'build'), exist_ok=True)
    os.makedirs(os.path.join(cp_ws, 'build'), exist_ok=True)

    cmd = (
        f'pytest {" ".join(test_files)} '
        f'--embedded-services serial '
        f'--count 2 '
        f'--port "{host_port}|{slave_port}" '
        f'--target {hw_cfg["host_target"]},{hw_cfg["slave_target"]} '
        f'--app-path "{host_ws}|{cp_ws}" '
        f'-v'
    )
    if junit_path:
        cmd += f' --junitxml={junit_path}'

    start = time.time()
    output_lines = []
    fail_lines = []
    in_failure_block = False
    try:
        proc = subprocess.Popen(
            cmd, shell=True, cwd=SCRIPT_DIR,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1
        )
        for line in proc.stdout:
            line = line.rstrip()
            output_lines.append(line)

            # Capture panic decoded file path
            if 'panic_decoded' in line:
                fail_lines.append(line.strip())

            # Track failure blocks
            if '>       assert' in line or 'AssertionError' in line:
                in_failure_block = True
            if in_failure_block:
                fail_lines.append(line.strip())
                if line.strip() == '' or 'short test summary' in line:
                    in_failure_block = False

            # Capture device crash/error lines
            if any(k in line for k in ['Guru Meditation', 'assert failed',
                                        'panic', 'abort()',
                                        'FatalError', 'ERROR at setup',
                                        'ERROR at teardown']):
                fail_lines.append(line.strip())

        proc.wait(timeout=300)
    except subprocess.TimeoutExpired:
        proc.kill()
        return 0, 1, time.time() - start, '\n'.join(output_lines), ['Timeout'], 0
    duration = time.time() - start

    output = '\n'.join(output_lines)
    m_pass = re.search(r'(\d+) passed', output)
    m_fail = re.search(r'(\d+) failed', output)
    m_err = re.search(r'(\d+) error', output)
    m_warn = re.search(r'(\d+) warning', output)
    passed = int(m_pass.group(1)) if m_pass else 0
    failed = int(m_fail.group(1)) if m_fail else 0
    errors = int(m_err.group(1)) if m_err else 0
    failed += errors
    warn_count = int(m_warn.group(1)) if m_warn else 0

    return passed, failed, duration, output, fail_lines, warn_count


def _extract_failure_summary(fail_lines, pytest_output):
    """Extract concise failure reason from captured lines and full output."""
    # Priority 1: Crash/panic
    for fl in fail_lines:
        for k in ['Guru Meditation', 'assert failed', 'panic', 'abort()']:
            if k in fl:
                return f'CRASH: {fl[:100]}'

    # Priority 2: esptool/connection errors
    for fl in fail_lines:
        if 'FatalError' in fl:
            return fl[:100]

    # Priority 3: Assertion failures
    for fl in fail_lines:
        if 'assert ' in fl or 'AssertionError' in fl:
            return fl[:100]

    # Priority 4: ERROR at setup/teardown
    for fl in fail_lines:
        if 'ERROR at setup' in fl or 'ERROR at teardown' in fl:
            return fl[:100]

    # Priority 5: Last meaningful line from pytest output
    for line in reversed(pytest_output.split('\n')):
        line = line.strip()
        if line and 'warning' not in line.lower() and '===' not in line:
            return line[:100]

    return 'Unknown failure'


LAST_RUN_FILE = SCRIPT_DIR.parent / 'workspace' / '.last_run.json'


def optimize_order(pairs):
    """Reorder pairs to minimize flash transitions between consecutive tests.

    Greedy algorithm: pick next test that shares the most firmware with the
    previous test. Prefers: same CP+host > same CP > same host > neither.

    Tests with pre_flash_actions always need a host flash — handled naturally
    by the flash cache (pre_flash_actions disables unchanged-skip).
    """
    ordered = []
    remaining = dict(pairs)
    cur_cp, cur_host = None, None

    while remaining:
        def score(item):
            _, pcfg = item
            same_cp = pcfg['cp'] == cur_cp
            same_host = pcfg['host'] == cur_host
            if same_cp and same_host:
                return 0
            if same_cp:
                return 1
            if same_host:
                return 2
            return 3
        best_name, best_cfg = min(remaining.items(), key=score)
        ordered.append((best_name, best_cfg))
        cur_cp, cur_host = best_cfg['cp'], best_cfg['host']
        del remaining[best_name]

    return dict(ordered)


def _save_last_run(failed_pairs, workspace_root):
    """Save failed pairs for --retry-failed."""
    data = {
        'failed': list(failed_pairs),
        'workspace': str(workspace_root),
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    os.makedirs(os.path.dirname(LAST_RUN_FILE), exist_ok=True)
    with open(LAST_RUN_FILE, 'w') as f:
        json.dump(data, f, indent=2)


def _load_last_run():
    """Load failed pairs from previous run."""
    if not os.path.exists(LAST_RUN_FILE):
        return None
    with open(LAST_RUN_FILE) as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser(description='ESP-Hosted HW Test Matrix Runner')
    parser.add_argument('--suite', help='Comma-separated test suite names (e.g. boot_wifi,gpio)')
    parser.add_argument('--mode', choices=['on-target', 'simulated', 'auto'], default='auto')
    parser.add_argument('--skip-build', action='store_true', help='Use existing builds')
    parser.add_argument('--skip-flash', action='store_true', help='Skip flashing, run tests only')
    parser.add_argument('--retry-failed', action='store_true',
                        help='Retry only failed suites from last run (reuses workspace)')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--order', choices=['auto', 'alphabetical', 'manual'],
                        help='Test ordering: auto=minimize flash transitions, '
                             'alphabetical=sorted, manual=matrix order. '
                             'Default from env.json test_order.')
    parser.add_argument('--json', help='JSON report output path')
    parser.add_argument('--junit', help='JUnit XML output directory')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    if args.skip_flash:
        args.skip_build = True

    cfg = eh_test_config_load()
    ensure_idf_env(cfg.idf_path)

    matrix = load_matrix()
    hw_cfg = {
        'host_port': cfg.host_port,
        'slave_port': cfg.slave_port,
        'host_target': cfg.get_host_target(),
        'slave_target': cfg.get_slave_target(),
    }

    os.environ['FLASH_BAUD'] = str(cfg.flash_baud)

    pairs = matrix['pairs']

    if args.list:
        for name, pcfg in pairs.items():
            req = pcfg.get('requires', [])
            enabled = cfg.is_suite_enabled(name)
            skip_reason = ''
            if not enabled:
                skip_reason = ' [DISABLED in env.json]'
            elif 'wifi_ap' in req and not cfg.wifi_configured:
                skip_reason = ' [needs wifi_ap in env.json]'
            print(f'  {"  " if not enabled else ""}{name:20s} -- {pcfg["description"]}{skip_reason}')
        return 0

    # Clear stale retry state on fresh runs
    if not args.retry_failed and os.path.exists(LAST_RUN_FILE):
        os.remove(LAST_RUN_FILE)

    # --retry-failed: load failed pairs from last run, reuse workspace
    if args.retry_failed:
        last = _load_last_run()
        if not last or not last.get('failed'):
            print('ERROR: No previous failed run found. Run without --retry-failed first.')
            return 1
        print(f'\nRetrying {len(last["failed"])} failed suites from {last["timestamp"]}:')
        print(f'  {", ".join(last["failed"])}')
        selected = set(last['failed'])
        pairs = {k: v for k, v in pairs.items() if k in selected}
        # Reuse existing workspace builds — only rebuild if explicitly asked
        if not args.skip_build:
            args.skip_build = True
            print('  (reusing workspace builds, use without --skip-build to force rebuild)')
    elif args.suite:
        selected = set(args.suite.split(','))
        pairs = {k: v for k, v in pairs.items() if k in selected}
    else:
        pairs = {k: v for k, v in pairs.items() if cfg.is_suite_enabled(k)}

    if not pairs:
        print('ERROR: No matching/enabled suites')
        return 1

    runnable = {}
    for name, pcfg in pairs.items():
        reqs = pcfg.get('requires', [])
        if 'wifi_ap' in reqs and not cfg.wifi_configured:
            print(f'  SKIP {name}: requires wifi_ap (configure in env.json)')
            continue
        runnable[name] = pcfg
    pairs = runnable

    # Apply test ordering (CLI arg overrides env.json)
    order = args.order or cfg.test_order
    if order == 'auto' and len(pairs) > 1:
        pairs = optimize_order(pairs)
        print(f'  Order: auto (optimized for {len(pairs)} suites)')
    elif order == 'alphabetical':
        pairs = dict(sorted(pairs.items()))

    cp_base = cfg.cp_base
    host_base = cfg.host_base
    workspace_root = cfg.workspace_dir

    # ── Hardware Probe ───────────────────────────────────────────────
    hw_available = False
    if args.skip_flash:
        # Skip probe when --skip-flash: devices are running, can't esptool-connect
        print('\nSkipping HW probe (--skip-flash: devices already running)')
        hw_available = True
    elif args.mode in ('on-target', 'auto'):
        print('\nProbing hardware...')
        ok, msg, _, _ = eh_test_hw_probe_setup(cfg.host_port, cfg.slave_port)
        if ok:
            print(f'  OK: {msg}')
            hw_available = True
        else:
            print(f'  NOT FOUND: {msg}')
            if args.mode == 'on-target':
                print('ERROR: --mode on-target requires hardware. Aborting.')
                return 1
            print('  Falling back to simulated mode.')

    if args.mode == 'simulated' or (args.mode == 'auto' and not hw_available):
        print('\nSimulated mode -- no hardware tests.')
        print('Simulated tests not yet implemented.')
        return 0

    if args.dry_run:
        print(f'\nDry run (workspace: {workspace_root}):')
        for name, pcfg in pairs.items():
            print(f'  {name}: build {pcfg["cp"]} + {pcfg["host"]}, flash, run {pcfg["tests"]}')
        return 0

    print(f'  Workspace: {workspace_root}')

    # ── Artifacts ────────────────────────────────────────────────────
    artifacts = EhTestArtifactManager(
        artifact_dir=cfg.artifact_dir,
        retention_days=cfg.artifact_retention_days,
        log_retention=cfg.log_retention,
    )
    artifacts.init()

    # ── Execute ──────────────────────────────────────────────────────
    report = EhTestReport()

    for pair_name, pair_cfg in pairs.items():
        cp_path = os.path.join(cp_base, pair_cfg['cp'])
        host_path = os.path.join(host_base, pair_cfg['host'])

        # Workspace dirs — all build artifacts isolated here
        cp_ws = os.path.join(workspace_root, pair_name, 'cp')
        host_ws = os.path.join(workspace_root, pair_name, 'host')

        suite_start = time.time()
        pair_idx = list(pairs.keys()).index(pair_name) + 1
        eh_test_print_pair_header(pair_idx, len(pairs), pair_name, pair_cfg['description'])

        def subst(lines):
            if not lines:
                return None
            return [l.format(wifi_ssid=cfg.wifi_ssid, wifi_password=cfg.wifi_password)
                    for l in lines]

        # ── Build ────────────────────────────────────────────────────
        cp_build_dir = os.path.join(cp_ws, 'build')
        host_build_dir = os.path.join(host_ws, 'build')

        if not args.skip_build:
            cp_extra = (cfg.build_sdkconfig_optimizations or []) + (subst(pair_cfg.get('cp_sdkconfig_extra')) or [])
            ok, msg, cp_build_dir = eh_test_build_project(
                cp_path, hw_cfg['slave_target'],
                sdkconfig_extra=cp_extra,
                strategy=cfg.build_strategy,
                workspace_dir=cp_ws)
            if not ok:
                eh_test_print_phase('Build', f'CP failed: {msg[:60]}', ok=False)
                report.add_pair(pair_name, pair_cfg['description'], 'BUILD_FAIL', error=msg)
                continue
            cp_ok = msg

            host_extra = (cfg.build_sdkconfig_optimizations or []) + (subst(pair_cfg.get('host_sdkconfig_extra')) or [])
            ok, msg, host_build_dir = eh_test_build_project(
                host_path, hw_cfg['host_target'],
                sdkconfig_extra=host_extra,
                strategy=cfg.build_strategy,
                workspace_dir=host_ws)
            if not ok:
                eh_test_print_phase('Build', f'Host failed: {msg[:60]}', ok=False)
                report.add_pair(pair_name, pair_cfg['description'], 'BUILD_FAIL', error=msg)
                continue

            eh_test_print_phase('Build', f'CP {cp_ok.lower()}, Host {msg.lower()}')

            # Patch managed component (in host project dir, not workspace)
            for patch in matrix.get('managed_component_patches', []):
                ok, pmsg = eh_test_build_apply_patch(
                    host_path, patch['find'],
                    patch['old'], patch['new']
                )
                if ok:
                    # Rebuild with workspace flags after patch
                    idf_flags = f'-B {host_build_dir}' if host_ws else ''
                    r = subprocess.run(
                        f'idf.py {idf_flags} build',
                        shell=True, cwd=host_path,
                        capture_output=True, text=True, timeout=600
                    )
                    eh_test_print_phase('Patch', f'{pmsg}, rebuilt' if r.returncode == 0 else f'{pmsg}, rebuild FAILED')

        # ── Flash ────────────────────────────────────────────────────
        if not args.skip_flash:
            ok, flash_log = eh_test_flash_pair(
                cfg.host_port, cfg.slave_port,
                hw_cfg['host_target'], hw_cfg['slave_target'],
                cp_build_dir, host_build_dir,
                pre_flash_actions=pair_cfg.get('pre_flash'),
                flash_cfg=cfg.get_flash_cfg(pair_name)
            )
            total_line = [l for l in flash_log if '[total]' in l]
            eh_test_print_phase('Flash',
                total_line[0].replace('[total] ', '') if total_line else 'done',
                ok=ok)
            if not ok:
                for entry in flash_log:
                    print(f'    {C.DIM}{entry}{C.RESET}')
                report.add_pair(pair_name, pair_cfg['description'], 'FLASH_FAIL',
                                error=flash_log[-1], log=flash_log)
                continue
        else:
            eh_test_print_phase('Flash', 'skipped (--skip-flash)')

        # Reset both devices just before pytest — ensures pytest captures all boot output.
        # CP may have been flashed with --after hard_reset seconds ago (already booted),
        # while host was skip-flashed (still in bootloader). Resetting both ensures
        # synchronized boot and a clean SDIO/SPI handshake.
        from infra.flasher import _just_reset
        _just_reset(cfg.slave_port)   # CP reset first
        time.sleep(0.1)
        _just_reset(cfg.host_port)    # then host — CP is ready when host boots
        time.sleep(0.3)

        # ── Test ─────────────────────────────────────────────────────
        junit_path = None
        if args.junit:
            os.makedirs(args.junit, exist_ok=True)
            junit_path = os.path.join(args.junit, f'{pair_name}.xml')

        # Checkpoint log: captured by eh_test_expect* via EH_CHECKPOINT_LOG env var.
        # Fresh file per pair; artifact manager saves or discards per retention mode.
        checkpoint_log = os.path.join(host_ws, 'checkpoints.log')
        try:
            if os.path.exists(checkpoint_log):
                os.remove(checkpoint_log)
        except OSError:
            pass
        os.environ['EH_CHECKPOINT_LOG'] = checkpoint_log

        passed, failed, duration, pytest_output, fail_lines, warn_count = run_pytest(
            hw_cfg, pair_cfg['tests'], host_ws, cp_ws, junit_path
        )

        status = 'PASS' if failed == 0 and passed > 0 else 'FAIL'
        warn_str = f', {warn_count}w' if warn_count else ''
        eh_test_print_phase('Test',
            f'{passed}p/{failed}f{warn_str} ({duration:.1f}s)',
            ok=(status == 'PASS'))

        # Decode crash backtraces — inline replace addresses + summary
        if 'Guru Meditation' in pytest_output:
            pytest_output, crash_summary = eh_test_decode_backtrace(
                pytest_output, cp_build_dir, host_build_dir,
                cp_chip=hw_cfg['slave_target'],
                host_chip=hw_cfg['host_target'])
            for line in crash_summary:
                print(f'    {C.ORANGE}{line}{C.RESET}')

        # Inline failure summary
        if status == 'FAIL':
            summary = _extract_failure_summary(fail_lines, pytest_output)
            print(f'    {C.RED}{summary}{C.RESET}')
            if artifacts.run_dir:
                log_path = artifacts.run_dir / f'{pair_name}.log'
                print(f'    {C.DIM}Log: {log_path}{C.RESET}')

        artifacts.save_pair_log(pair_name, status, pytest_output,
                                checkpoint_log_path=checkpoint_log)

        # Allow serial ports to fully close before next test's flash step.
        # pytest-embedded's serial teardown can be async — without this,
        # the next test's put_host_in_bootloader() races with the stale
        # serial handle and fails with termios errors.
        time.sleep(1.0)

        suite_duration = time.time() - suite_start
        report.add_pair(pair_name, pair_cfg['description'], status,
                        tests_passed=passed, tests_failed=failed,
                        duration_s=suite_duration)

    # ── Report ───────────────────────────────────────────────────────
    report.print_summary()
    artifacts.finalize(report)

    # Save failed pairs for --retry-failed
    failed_names = [p['name'] for p in report.pairs if 'FAIL' in p['status']]
    if failed_names:
        _save_last_run(failed_names, workspace_root)
        print(f'\n  {C.DIM}Retry failed: python eh_test_runner.py --retry-failed{C.RESET}')

    if args.json:
        report.to_json(args.json)
        print(f'\nJSON report: {args.json}')

    return 0 if report.all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
