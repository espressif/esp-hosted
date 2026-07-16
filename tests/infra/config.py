"""
Test configuration loader.

Reads env.json (user-local, gitignored) and provides typed access.
Falls back to env.json.example if env.json doesn't exist.

Usage:
    from infra.config import eh_test_config_load
    cfg = eh_test_config_load()
    print(cfg.host_port, cfg.wifi_ssid)
"""

import json
import os
from pathlib import Path

TESTS_DIR = Path(__file__).parent.parent
ENV_FILE = TESTS_DIR / 'env.json'
ENV_EXAMPLE = TESTS_DIR / 'env.json.example'


class EhTestConfig:
    """Merged test configuration from env.json."""

    def __init__(self, data):
        hw = data.get('hardware', {})
        self.host_port = os.environ.get('HOST_PORT', hw.get('host_port', '/dev/cu.usbserial-120'))
        self.slave_port = os.environ.get('SLAVE_PORT', hw.get('slave_port', '/dev/cu.usbserial-1301'))
        self.board = hw.get('board', 'p4_c6_core_board')
        wifi = data.get('wifi_ap', {})
        self.wifi_ssid = wifi.get('ssid', '')
        self.wifi_password = wifi.get('password', '')
        self.wifi_configured = bool(self.wifi_ssid)

        paths = data.get('paths', {})
        # Mode picks the source for the HOST/MCU example. CP is always this-repo.
        #   "this_repo" (default) — examples/<feat>/<scen>/mcu/{host,esp_host}/
        #   "legacy"             — paths.host_base + pair["host_legacy"]
        # CLI --mode flag overrides via $EH_TEST_MODE.
        self.path_mode = os.environ.get('EH_TEST_MODE', paths.get('mode', 'this_repo'))
        if self.path_mode not in ('this_repo', 'legacy'):
            raise ValueError(f"paths.mode must be 'this_repo' or 'legacy', got '{self.path_mode}'")
        # this_repo_root: auto-detect (3 levels up from this file: tests/infra/) or explicit.
        repo_root = paths.get('this_repo_root', 'auto')
        if repo_root == 'auto':
            self.this_repo_root = str(Path(__file__).resolve().parents[2])
        else:
            self.this_repo_root = os.path.expanduser(repo_root)
        # Legacy host base — only consulted when mode == "legacy".
        # Default uses the in-repo _upstream_mcu symlink so legacy mode works
        # out-of-box for anyone who has a sibling esp-hosted-mcu checkout.
        host_base_raw = paths.get('host_base', '')
        if not host_base_raw or host_base_raw == 'auto':
            self.host_base = str(Path(self.this_repo_root) / '_upstream_mcu' / 'examples')
        else:
            self.host_base = os.path.expanduser(host_base_raw)
        # cp_base anchors Kconfig discovery (find_cp_kconfigs walks up
        # from this dir to the first ancestor that contains
        # `coprocessor/`).  Defaults to <this_repo>/examples — that
        # walks up to repo root which has coprocessor/.  Override only
        # when pointing at an external CP checkout.
        cp_base_raw = paths.get('cp_base', '')
        if not cp_base_raw or cp_base_raw == 'auto':
            self.cp_base = str(Path(self.this_repo_root) / 'examples')
        else:
            self.cp_base = os.path.expanduser(cp_base_raw)
        self.idf_path = os.path.expanduser(paths.get('idf_path', '~/esp-idf'))

        build = data.get('build', {})
        # "reuse"    — incremental build, fastest (default)
        # "clean"    — always rm -rf build/sdkconfig before building
        # "fallback" — try incremental, clean-rebuild on failure
        self.build_strategy = build.get('strategy', 'fallback')

        # Flash configuration (new: strategy, calibration, diff flash)
        flash = data.get('flash', {})
        # Flash strategy: "sequential" (default), "parallel", "staggered"
        self.flash_strategy = flash.get('strategy', 'sequential')
        # Baud priority: $FLASH_BAUD env > flash.baud > hardware.flash_baud > 2000000
        self.flash_baud = int(os.environ.get('FLASH_BAUD',
            flash.get('baud', hw.get('flash_baud', 2000000))))
        self.flash_timeout_multiplier = float(flash.get('timeout_multiplier', 2.0))
        self.flash_overhead_s = float(flash.get('overhead_s', 3.0))
        self.flash_compression_ratio = float(flash.get('compression_ratio', 0.65))
        self.flash_use_diff = bool(flash.get('use_diff', False))
        self.flash_trust_content = bool(flash.get('trust_flash_content', False))
        self.flash_force_full = bool(flash.get('force_full_flash', False))

        # Test ordering: "auto" (minimize flash transitions), "alphabetical", "manual"
        self.test_order = data.get('test_order', 'alphabetical')

        # Per-suite overrides: {suite_name: {flash: {...}, ...}}
        # Allows overriding flash config (strategy, diff) for specific tests
        self.suite_overrides = data.get('suite_overrides', {})

        # Workspace: all build artifacts go here (isolates example source dirs)
        # Default: tests/workspace/ (relative to tests dir)
        ws = build.get('workspace_dir', '')
        if ws:
            self.workspace_dir = os.path.expanduser(ws)
        else:
            self.workspace_dir = str(TESTS_DIR / 'workspace')

        # Test-build sdkconfig optimizations — user preferences (Layer 2)
        self.build_sdkconfig_optimizations = build.get('sdkconfig_optimizations', [
            'CONFIG_SPIRAM_MEMTEST=n',
            'CONFIG_BOOTLOADER_SKIP_VALIDATE_ON_POWER_ON=y',
        ])

        # Board auto-config: separate modes for host and slave
        self.host_board_config_mode = build.get('host_board_config', 'auto')
        self.slave_board_config_mode = build.get('slave_board_config', 'auto')
        self.host_board_sdkconfig = build.get('host_board_sdkconfig', [])
        self.slave_board_sdkconfig = build.get('slave_board_sdkconfig', [])

        artifacts = data.get('artifacts', {})
        self.artifact_dir = os.path.expanduser(
            artifacts.get('dir', '/tmp/esp_hosted_test_logs'))
        self.artifact_retention_days = int(artifacts.get('retention_days', 7))
        # Log retention mode: what's kept per test pair
        #  'fail_only'           — only failures get full logs (legacy)
        #  'checkpoints_on_pass' — failures get full logs, passes get compact
        #                          checkpoint trace (default, best of both)
        #  'full_always'         — every pair keeps full pytest output
        #  'none'                — discard everything
        self.log_retention = artifacts.get('log_retention', 'checkpoints_on_pass')

        self.suites = data.get('suites', {})

    def is_suite_enabled(self, suite_name):
        return self.suites.get(suite_name, True)

    def resolve_cp_path(self, pair_cfg):
        """CP example dir for a matrix pair. Always this-repo."""
        sub = pair_cfg.get('cp')
        if not sub:
            return None
        return str(Path(self.this_repo_root) / sub)

    def resolve_host_path(self, pair_cfg):
        """Host/MCU example dir; mode picks this-repo vs legacy mcu6."""
        if self.path_mode == 'legacy':
            sub = pair_cfg.get('host_legacy', pair_cfg.get('host'))
            if not sub:
                return None
            return os.path.join(self.host_base, sub)
        # this_repo
        sub = pair_cfg.get('host_this_repo', pair_cfg.get('host'))
        if not sub:
            return None
        p = Path(self.this_repo_root) / sub
        if p.is_dir():
            return str(p)
        # mcu_host ↔ esp_host swap fallback.
        s = str(p)
        for a, b in (('/mcu_host', '/esp_host'), ('/esp_host', '/mcu_host')):
            if s.endswith(a):
                alt = Path(s[:-len(a)] + b)
                if alt.is_dir():
                    return str(alt)
        return s    # caller surfaces the missing-dir error

    def describe_paths(self):
        """One-line human summary for runner / UI banner."""
        if self.path_mode == 'legacy':
            return f"mode=legacy  host_base={self.host_base}  this_repo={self.this_repo_root}"
        return f"mode=this_repo  root={self.this_repo_root}"

    def get_flash_cfg(self, suite_name=None):
        """Return flash config dict for eh_test_flash_pair().
        If suite_name given and suite_overrides[suite_name].flash exists,
        those values override the globals."""
        base = {
            'strategy': self.flash_strategy,
            'baud': self.flash_baud,
            'timeout_multiplier': self.flash_timeout_multiplier,
            'overhead_s': self.flash_overhead_s,
            'compression_ratio': self.flash_compression_ratio,
            'use_diff': self.flash_use_diff,
            'trust_flash_content': self.flash_trust_content,
            'force_full_flash': self.flash_force_full,
        }
        if suite_name:
            override = self.suite_overrides.get(suite_name, {}).get('flash', {})
            base.update({k: v for k, v in override.items() if v is not None})
        return base

    def get_host_target(self):
        b = self.board.lower()
        if 'p4' in b:
            return 'esp32p4'
        return 'esp32'

    def get_slave_target(self):
        b = self.board.lower()
        if 'c61' in b:
            return 'esp32c61'
        if 'c6' in b:
            return 'esp32c6'
        if 'c5' in b:
            return 'esp32c5'
        if 'c3' in b:
            return 'esp32c3'
        if 'c2' in b:
            return 'esp32c2'
        return 'esp32'


def eh_test_config_load():
    """Load from env.json, fall back to example."""
    for path in [ENV_FILE, ENV_EXAMPLE]:
        if path.exists():
            if path == ENV_EXAMPLE:
                print('  NOTE: No env.json found. Using env.json.example.')
                print('  Copy tests/env.json.example -> tests/env.json and customize.')
            with open(path) as f:
                return EhTestConfig(json.load(f))
    return EhTestConfig({})
