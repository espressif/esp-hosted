"""
Test artifact management — retain failure logs, discard pass logs.

Artifact directory layout:
    <artifact_root>/
        manifest.json          # index of all runs
        <run_id>/              # one dir per run (timestamp-based)
            report.json        # test report
            <pair_name>.log    # pytest stdout for failed pairs

On each test suite start, the manager:
  1. Loads manifest.json (creates if missing)
  2. Prunes stale entries: removes dirs older than retention_days
  3. Removes orphan entries (dir missing on disk)
  4. Saves updated manifest

After test run:
  - PASS pairs: logs discarded (not stored)
  - FAIL pairs: full pytest output saved
  - Report always saved
"""

import json
import os
import shutil
from datetime import datetime, timedelta
from pathlib import Path

DEFAULT_ARTIFACT_DIR = '/tmp/esp_hosted_test_logs'
DEFAULT_RETENTION_DAYS = 7
DEFAULT_LOG_RETENTION = 'checkpoints_on_pass'

# Retention modes:
#   fail_only           — save full logs only for failed pairs
#   checkpoints_on_pass — save full logs for failures + checkpoint trace
#                         for passes (default)
#   full_always         — save full logs for every pair
#   none                — discard everything
VALID_RETENTION_MODES = ('fail_only', 'checkpoints_on_pass', 'full_always', 'none')


class EhTestArtifactManager:
    """Manages test log artifacts with automatic pruning."""

    def __init__(self, artifact_dir=None, retention_days=None, log_retention=None):
        self.artifact_dir = Path(artifact_dir or DEFAULT_ARTIFACT_DIR)
        self.retention_days = retention_days if retention_days is not None else DEFAULT_RETENTION_DAYS
        self.log_retention = log_retention if log_retention in VALID_RETENTION_MODES else DEFAULT_LOG_RETENTION
        self.manifest_path = self.artifact_dir / 'manifest.json'
        self.manifest = {'runs': []}
        self.run_id = None
        self.run_dir = None

    # ── Lifecycle ────────────────────────────────────────────────────

    def init(self):
        """Call at start of test suite. Loads manifest, prunes old runs."""
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        self._load_manifest()
        self._prune()
        self._save_manifest()

        # Create run directory
        self.run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = self.artifact_dir / self.run_id
        self.run_dir.mkdir(exist_ok=True)

    def finalize(self, report):
        """Call after test suite completes. Saves report, updates manifest."""
        if not self.run_dir:
            return

        # Save report
        report_path = self.run_dir / 'report.json'
        report.to_json(str(report_path))

        # If all passed, remove the run dir (no failures to retain)
        has_failures = any(
            'FAIL' in p['status'] for p in report.pairs
        )

        entry = {
            'run_id': self.run_id,
            'timestamp': datetime.now().isoformat(),
            'total_passed': report.total_passed,
            'total_failed': report.total_failed,
            'all_pass': report.all_pass,
        }

        # Keep the run dir if:
        #  - there are failures (for full logs), OR
        #  - retention is checkpoints_on_pass or full_always (for compact trace)
        keep_dir = has_failures or self.log_retention in ('checkpoints_on_pass', 'full_always')

        if keep_dir:
            entry['retained'] = True
            entry['expire_date'] = (
                datetime.now() + timedelta(days=self.retention_days)
            ).isoformat()
            self.manifest['runs'].append(entry)
            label = 'Failure logs' if has_failures else 'Pass checkpoints'
            print(f'\n  {label}: {self.run_dir}')
            print(f'  Retained for {self.retention_days} days')
        else:
            shutil.rmtree(self.run_dir, ignore_errors=True)
            self.run_dir = None

        self._save_manifest()

    # ── Per-pair + per-test logging ──────────────────────────────────

    def save_pair_log(self, pair_name, status, pytest_output, checkpoint_log_path=None):
        """Save pytest output for a pair, respecting the log_retention mode.

        Files produced (depending on mode):
          <pair_name>.log                 — full pytest output
          <pair_name>.checkpoints.log     — compact expect trace
          <pair_name>/<func>.log          — per-test-function slice

        Modes:
          none                → save nothing
          fail_only           → save full + per-func only on failure
          checkpoints_on_pass → failure: full + per-func; pass: checkpoints
          full_always         → save full + per-func every time
        """
        if not self.run_dir or self.log_retention == 'none':
            return

        is_fail = 'FAIL' in status
        mode = self.log_retention

        # Decide what to save
        save_full = is_fail or mode == 'full_always'
        save_checkpoints = (
            not is_fail and mode in ('checkpoints_on_pass', 'full_always')
        ) or (is_fail and mode == 'full_always')

        if save_full:
            log_path = self.run_dir / f'{pair_name}.log'
            with open(log_path, 'w') as f:
                f.write(pytest_output)

            # Per-test-function split (only meaningful when we saved full)
            try:
                slices = _split_pytest_output_by_test(pytest_output)
                if len(slices) > 1:
                    sub_dir = self.run_dir / pair_name
                    sub_dir.mkdir(exist_ok=True)
                    for func_name, content in slices.items():
                        if not func_name:
                            continue
                        safe = func_name.replace('/', '_').replace(':', '_')
                        with open(sub_dir / f'{safe}.log', 'w') as f:
                            f.write(content)
            except Exception:
                pass

        if save_checkpoints and checkpoint_log_path and os.path.exists(checkpoint_log_path):
            try:
                dst = self.run_dir / f'{pair_name}.checkpoints.log'
                shutil.copy2(checkpoint_log_path, dst)
            except OSError:
                pass

    # ── Internal ─────────────────────────────────────────────────────

    def _load_manifest(self):
        if self.manifest_path.exists():
            try:
                with open(self.manifest_path) as f:
                    self.manifest = json.load(f)
            except (json.JSONDecodeError, KeyError):
                self.manifest = {'runs': []}
        else:
            self.manifest = {'runs': []}

    def _save_manifest(self):
        with open(self.manifest_path, 'w') as f:
            json.dump(self.manifest, f, indent=2)

    def _prune(self):
        """Remove expired and orphaned entries."""
        now = datetime.now()
        kept = []

        for entry in self.manifest.get('runs', []):
            run_dir = self.artifact_dir / entry['run_id']

            # Remove orphan entries (dir missing on disk)
            if not run_dir.exists():
                continue

            # Check expiry
            expire_str = entry.get('expire_date')
            if expire_str:
                try:
                    expire = datetime.fromisoformat(expire_str)
                    if now > expire:
                        shutil.rmtree(run_dir, ignore_errors=True)
                        print(f'  Pruned expired logs: {entry["run_id"]}')
                        continue
                except ValueError:
                    pass

            kept.append(entry)

        # Also clean up any orphan directories not in manifest
        if self.artifact_dir.exists():
            manifest_ids = {e['run_id'] for e in kept}
            for child in self.artifact_dir.iterdir():
                if child.is_dir() and child.name not in manifest_ids:
                    # Unknown dir — check if it looks like a run dir (YYYYMMDD_HHMMSS)
                    if len(child.name) == 15 and child.name[8] == '_':
                        shutil.rmtree(child, ignore_errors=True)

        self.manifest['runs'] = kept


# ── pytest output splitter ─────────────────────────────────────────────

import re as _re

_PYTEST_TEST_LINE = _re.compile(
    r'^[\w/]+\.py::(\w+::)?(\w+)(\[[^\]]*\])?\s+(PASSED|FAILED|SKIPPED|ERROR)'
)
_PYTEST_FAILURE_HEADER = _re.compile(r'^_{5,}\s+(\w+)\.(\w+)(\[[^\]]*\])?\s+_{5,}$')


def _split_pytest_output_by_test(output):
    """Split pytest output into per-test-function chunks.

    Pytest output has two landmarks per test function:
      1. Collection/run line: `path/to/test.py::ClassName::test_func[...] PASSED`
      2. Failure block header: `______ ClassName.test_func[...] ______`

    Strategy: use the failure block headers (they clearly delimit each
    test's failure details).  Pass-only tests have no detailed block, so
    we just capture their one-line status.

    Returns: dict { 'test_func_name': 'chunk of output' }
    """
    lines = output.split('\n')
    result = {}
    current_test = None
    current_lines = []
    header_lines = []  # session header (before any test)
    in_header = True

    for line in lines:
        # Detect failure block header
        m = _PYTEST_FAILURE_HEADER.match(line)
        if m:
            if current_test and current_lines:
                result[current_test] = '\n'.join(current_lines)
            current_test = m.group(2)
            current_lines = [line]
            in_header = False
            continue

        # Detect end of a test block (short summary / warnings / etc.)
        if line.startswith('===') or line.startswith('---') and current_test:
            if 'short test summary' in line.lower() or 'FAILED' in line:
                if current_test and current_lines:
                    result[current_test] = '\n'.join(current_lines)
                    current_test = None
                    current_lines = []

        if current_test:
            current_lines.append(line)
        elif in_header:
            header_lines.append(line)

    # Flush last block
    if current_test and current_lines:
        result[current_test] = '\n'.join(current_lines)

    # Prepend session header to each file so they're self-contained
    if result and header_lines:
        header = '\n'.join(header_lines[:30])  # First 30 lines = session info
        result = {k: header + '\n\n' + v for k, v in result.items()}

    return result
