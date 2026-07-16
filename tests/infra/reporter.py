"""
Report generation — ANSI-colored terminal summary + JSON export.
"""

import json
import os
import sys
from datetime import datetime


# ── ANSI Colors ─────────────────────────────────────────────────────────

def _supports_color():
    if os.environ.get('NO_COLOR'):
        return False
    if os.environ.get('FORCE_COLOR'):
        return True
    return hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()


class C:
    """ANSI escape codes. Empty strings when color is off."""
    _on = _supports_color()
    RESET  = '\033[0m'  if _on else ''
    BOLD   = '\033[1m'  if _on else ''
    DIM    = '\033[2m'  if _on else ''
    GREEN  = '\033[32m' if _on else ''
    RED    = '\033[31m' if _on else ''
    YELLOW = '\033[33m' if _on else ''
    CYAN   = '\033[36m' if _on else ''
    ORANGE = '\033[38;5;208m' if _on else ''
    GRAY   = '\033[90m' if _on else ''
    WHITE  = '\033[97m' if _on else ''


# ── Public helpers for runner ───────────────────────────────────────────

def eh_test_print_pair_header(idx, total, pair_name, description):
    """Compact pair header: [3/12] gpio -- GPIO set/get/toggle via RPC"""
    print(f'\n{C.BOLD}[{idx}/{total}]{C.RESET} '
          f'{C.CYAN}{pair_name}{C.RESET} '
          f'{C.DIM}-- {description}{C.RESET}')


def eh_test_print_phase(phase, msg, ok=True):
    """Print a phase result inline: Build: ok | Flash: 28s"""
    color = C.GREEN if ok else C.RED
    print(f'  {C.DIM}{phase}:{C.RESET} {color}{msg}{C.RESET}')


def eh_test_print_test_line(line):
    """Print a streamed test output line with color."""
    if 'PASSED' in line:
        print(f'    {C.GREEN}{line}{C.RESET}')
    elif 'FAILED' in line:
        print(f'    {C.RED}{line}{C.RESET}')
    elif any(k in line for k in ['AssertionError', 'assert ', 'E   ',
                                  'short test summary']):
        print(f'    {C.YELLOW}{line}{C.RESET}')
    elif any(k in line for k in ['FATAL', 'Timeout', 'ERROR', 'error']):
        print(f'    {C.RED}{line}{C.RESET}')
    else:
        print(f'    {C.DIM}{line}{C.RESET}')


# ── Report class ────────────────────────────────────────────────────────

class EhTestReport:
    """Accumulates test results and generates reports."""

    def __init__(self):
        self.pairs = []
        self.start_time = datetime.now()

    def add_pair(self, name, description, status, tests_passed=0,
                 tests_failed=0, duration_s=0, error=None, log=None):
        self.pairs.append({
            'name': name,
            'description': description,
            'status': status,
            'tests_passed': tests_passed,
            'tests_failed': tests_failed,
            'duration_s': round(duration_s, 1),
            'error': error,
            'log': log or [],
        })

    @property
    def total_passed(self):
        return sum(p['tests_passed'] for p in self.pairs)

    @property
    def total_failed(self):
        return sum(p['tests_failed'] for p in self.pairs)

    @property
    def all_pass(self):
        return self.total_failed == 0 and self.total_passed > 0

    def print_summary(self):
        elapsed = (datetime.now() - self.start_time).total_seconds()

        print(f'\n{C.BOLD}{"═" * 62}{C.RESET}')
        print(f'  {C.BOLD}ESP-HOSTED TEST REPORT{C.RESET}'
              f'  {C.DIM}{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}{C.RESET}')
        print(f'{C.BOLD}{"═" * 62}{C.RESET}')

        for p in self.pairs:
            total_tests = p['tests_passed'] + p['tests_failed']

            if p['status'] == 'PASS':
                tag = f'{C.GREEN}{C.BOLD}PASS{C.RESET}'
            elif 'FAIL' in p['status']:
                tag = f'{C.RED}{C.BOLD}FAIL{C.RESET}'
            else:
                tag = f'{C.YELLOW}{C.BOLD}SKIP{C.RESET}'

            tests = f'{p["tests_passed"]}/{total_tests}' if total_tests > 0 else ' - '
            dur = f'{p["duration_s"]:.0f}s' if p['duration_s'] > 0 else ' - '

            print(f'  {tag} {p["name"]:<18s} '
                  f'{tests:>4s}  {C.DIM}{dur:>4s}{C.RESET}  '
                  f'{C.DIM}{p["description"]}{C.RESET}')

            if p.get('error'):
                # Show error concisely — first line only, point to log
                err_line = p['error'].split('|')[0].strip()[:90]
                print(f'       {C.RED}└─ {err_line}{C.RESET}')
                if '|' in p['error']:
                    log_path = p['error'].split('|')[-1].strip()
                    if 'Log:' in log_path:
                        print(f'       {C.DIM}   {log_path}{C.RESET}')

        print(f'{C.BOLD}{"─" * 62}{C.RESET}')

        p_color = C.GREEN if self.total_passed > 0 else C.DIM
        f_color = C.RED if self.total_failed > 0 else C.DIM
        elapsed_min = elapsed / 60
        print(f'  {p_color}{self.total_passed} passed{C.RESET}, '
              f'{f_color}{self.total_failed} failed{C.RESET}'
              f'  {C.DIM}|  {elapsed_min:.1f}m ({elapsed:.0f}s){C.RESET}')
        print(f'{C.BOLD}{"═" * 62}{C.RESET}')

    def to_json(self, path=None):
        data = {
            'timestamp': self.start_time.isoformat(),
            'total_passed': self.total_passed,
            'total_failed': self.total_failed,
            'all_pass': self.all_pass,
            'pairs': self.pairs,
        }
        if path:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
        return data
