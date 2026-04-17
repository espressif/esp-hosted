#!/usr/bin/env python3
"""
ESP-Hosted example build orchestrator.

Design goals:
  - Single source of truth for target/transport/example compatibility
  - Shell-agnostic: pluggable runners for bash, fish, or custom shells
  - Parallel builds with proper skip/pass/fail reporting
  - Functions < 30 lines (Clean Code standard)

Usage:
  python3 build_examples.py --build-targets=esp32c6
  python3 build_examples.py --build-targets=esp32c2,esp32c6 --build-transports=sdio,spi-fd
  python3 build_examples.py --build-targets=esp32c6 --build-examples=minimal/wifi,minimal/bt
  python3 build_examples.py --build-targets=all --build-transports=all --shell=fish
  python3 build_examples.py --build-targets=esp32c2 --dry-run
  python3 build_examples.py --list-compat
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional


# ═══════════════════════════════════════════════════════════════════════════════
# Compatibility matrix — single source of truth
# ═══════════════════════════════════════════════════════════════════════════════

ALL_TARGETS = [
    "esp32", "esp32c2", "esp32c3", "esp32c5", "esp32c6", "esp32c61",
    "esp32s2", "esp32s3", "esp32h2", "esp32h4",
]


class Transport(Enum):
    SDIO   = "sdio"
    SPI_FD = "spi-fd"
    SPI_HD = "spi-hd"
    UART   = "uart"


TRANSPORT_TARGETS: dict[Transport, set[str]] = {
    Transport.SDIO:   {"esp32", "esp32c5", "esp32c6", "esp32c61"},
    Transport.SPI_FD: set(ALL_TARGETS),
    Transport.SPI_HD: {"esp32s2", "esp32s3"},
    Transport.UART:   set(ALL_TARGETS),
}

DEFAULT_TRANSPORT: dict[str, Transport] = {
    "esp32": Transport.SDIO, "esp32c2": Transport.SPI_FD,
    "esp32c3": Transport.SPI_FD, "esp32c5": Transport.SDIO,
    "esp32c6": Transport.SDIO, "esp32c61": Transport.SDIO,
    "esp32s2": Transport.SPI_FD, "esp32s3": Transport.SPI_FD,
    "esp32h2": Transport.SPI_FD, "esp32h4": Transport.SPI_FD,
}

TRANSPORT_KCONFIG: dict[Transport, str] = {
    Transport.SDIO:   "ESP_HOSTED_TRANSPORT_CP_SDIO",
    Transport.SPI_FD: "ESP_HOSTED_TRANSPORT_CP_SPI",
    Transport.SPI_HD: "ESP_HOSTED_TRANSPORT_CP_SPI_HD",
    Transport.UART:   "ESP_HOSTED_TRANSPORT_CP_UART",
}

NO_WIFI_TARGETS = {"esp32h2"}

# Example path substring → excluded chips.
# Verified against IDF Kconfig + soc_caps.
EXAMPLE_EXCLUDE: dict[str, set[str]] = {
    "network_split":   {"esp32c2", "esp32c3"},
    "minimal/bt":      {"esp32s2"},
    "wifi_enterprise": {"esp32c2"},
    "wifi_dpp":        {"esp32c2"},
    "wifi_itwt":       {"esp32", "esp32c2", "esp32c3", "esp32s2", "esp32s3"},
    "ext_coex":        {"esp32", "esp32c2"},
}


# ═══════════════════════════════════════════════════════════════════════════════
# CompatMatrix — query helpers
# ═══════════════════════════════════════════════════════════════════════════════

class CompatMatrix:

    @staticmethod
    def get_default_transport(target: str) -> Transport:
        return DEFAULT_TRANSPORT.get(target, Transport.SPI_FD)

    @staticmethod
    def get_supported_transports(target: str) -> list[Transport]:
        return [t for t in Transport if target in TRANSPORT_TARGETS.get(t, set())]

    @staticmethod
    def should_skip_example(example_rel: str, target: str) -> Optional[str]:
        if target in NO_WIFI_TARGETS:
            return f"{target} has no WiFi"
        for pattern, excluded in EXAMPLE_EXCLUDE.items():
            if pattern in example_rel and target in excluded:
                return f"{pattern} not supported on {target}"
        return None

    @staticmethod
    def resolve_targets(spec: str) -> list[str]:
        if spec == "all":
            return list(ALL_TARGETS)
        targets = [t.strip() for t in spec.split(",")]
        unknown = [t for t in targets if t not in ALL_TARGETS]
        if unknown:
            print(f"WARNING: Unknown targets: {unknown}", file=sys.stderr)
        return targets

    @staticmethod
    def resolve_transports(spec: str, target: str) -> list[Transport]:
        if spec == "default":
            return [CompatMatrix.get_default_transport(target)]
        if spec == "all":
            return CompatMatrix.get_supported_transports(target)
        result = []
        for name in (n.strip() for n in spec.split(",")):
            try:
                t = Transport(name)
            except ValueError:
                print(f"WARNING: Unknown transport: {name}", file=sys.stderr)
                continue
            if target in TRANSPORT_TARGETS.get(t, set()):
                result.append(t)
        return result


# ═══════════════════════════════════════════════════════════════════════════════
# Shell runners — pluggable build invocation
# ═══════════════════════════════════════════════════════════════════════════════

class ShellRunner:
    def __init__(self, idf_path: Optional[str] = None):
        self.idf_path = idf_path or os.environ.get(
            "IDF_PATH", os.path.expanduser("~/esp-idf"))

    def run(self, cmd: str, cwd: str, timeout: int = 600) -> tuple[int, str]:
        raise NotImplementedError

    def idf_build(self, project_dir: str, target: str,
                  fullclean: bool = True, timeout: int = 600,
                  build_dir: Optional[str] = None) -> tuple[int, str]:
        raise NotImplementedError


class BashRunner(ShellRunner):

    def run(self, cmd: str, cwd: str, timeout: int = 600) -> tuple[int, str]:
        try:
            r = subprocess.run(["bash", "-c", cmd], cwd=cwd,
                               capture_output=True, text=True, timeout=timeout)
            return r.returncode, r.stdout + r.stderr
        except subprocess.TimeoutExpired:
            return 1, f"TIMEOUT after {timeout}s"
        except Exception as e:
            return 1, str(e)

    def idf_build(self, project_dir: str, target: str,
                  fullclean: bool = True, timeout: int = 600,
                  build_dir: Optional[str] = None) -> tuple[int, str]:
        b_flag = f"-B {build_dir}" if build_dir else ""
        idf = f"idf.py {b_flag}"
        parts = [f'source "{self.idf_path}/export.sh" > /dev/null 2>&1']
        if fullclean:
            parts.append(f"{idf} fullclean 2>&1 || true")
            parts.append(f"IDF_TARGET={target} {idf} set-target {target} 2>&1")
        parts.append(f"IDF_TARGET={target} {idf} build 2>&1")
        return self.run(" && ".join(parts), cwd=project_dir, timeout=timeout)


class FishRunner(ShellRunner):

    def run(self, cmd: str, cwd: str, timeout: int = 600) -> tuple[int, str]:
        try:
            r = subprocess.run(["fish", "-c", cmd], cwd=cwd,
                               capture_output=True, text=True, timeout=timeout)
            return r.returncode, r.stdout + r.stderr
        except subprocess.TimeoutExpired:
            return 1, f"TIMEOUT after {timeout}s"
        except Exception as e:
            return 1, str(e)

    def idf_build(self, project_dir: str, target: str,
                  fullclean: bool = True, timeout: int = 600,
                  build_dir: Optional[str] = None) -> tuple[int, str]:
        b_flag = f"-B {build_dir}" if build_dir else ""
        idf = f"idf.py {b_flag}"
        parts = [f'source "{self.idf_path}/export.fish" > /dev/null 2>&1']
        if fullclean:
            parts.append(f"{idf} fullclean 2>&1; or true")
            parts.append(f"set -x IDF_TARGET {target}; and {idf} set-target {target} 2>&1")
        parts.append(f"set -x IDF_TARGET {target}; and {idf} build 2>&1")
        return self.run("; ".join(parts), cwd=project_dir, timeout=timeout)


SHELL_RUNNERS: dict[str, type[ShellRunner]] = {
    "bash": BashRunner, "fish": FishRunner,
}


# ═══════════════════════════════════════════════════════════════════════════════
# Build result
# ═══════════════════════════════════════════════════════════════════════════════

class Status(Enum):
    OK = "OK"; FAIL = "FAIL"; SKIP = "SKIP"


@dataclass
class BuildResult:
    example: str
    target: str
    transport: str = ""
    status: Status = Status.SKIP
    duration_s: float = 0.0
    warnings: int = 0
    reason: str = ""
    log_path: str = ""


# ═══════════════════════════════════════════════════════════════════════════════
# Discovery + detection
# ═══════════════════════════════════════════════════════════════════════════════

SKIP_DIRS = {"/build/", "/managed_components/", "/components/", "/main/"}


def discover_examples(examples_root: Path) -> list[Path]:
    projects = []
    for cmake in examples_root.rglob("CMakeLists.txt"):
        d = cmake.parent
        rel = str(d.relative_to(examples_root))
        if any(s in rel for s in SKIP_DIRS):
            continue
        if (d / "sdkconfig.defaults").exists() and (d / "main" / "CMakeLists.txt").exists():
            projects.append(d)
    return sorted(projects)


def detect_transport(project_dir: str, target: str) -> str:
    for name in [f"sdkconfig.defaults.{target}", "sdkconfig.defaults"]:
        path = os.path.join(project_dir, name)
        if not os.path.exists(path):
            continue
        with open(path) as f:
            content = f.read()
        for transport, kconfig in TRANSPORT_KCONFIG.items():
            if f"CONFIG_{kconfig}=y" in content:
                return transport.value
    return CompatMatrix.get_default_transport(target).value


# ═══════════════════════════════════════════════════════════════════════════════
# Project-dir helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _repo_root() -> str:
    return str(Path(__file__).resolve().parents[2])


def _ensure_component_symlink(project_dir: str, repo_root: str):
    """Create components/esp_hosted -> <repo_root> symlink if missing.

    Uses a relative path so the link survives repo moves.
    """
    comp_dir = Path(project_dir) / "components"
    link = comp_dir / "esp_hosted"
    if link.is_symlink() or link.exists():
        return
    comp_dir.mkdir(exist_ok=True)
    target = os.path.relpath(repo_root, comp_dir)
    link.symlink_to(target)


# IDF transient files that litter example source directories.
# ── ANSI colors ──────────────────────────────────────────────────────────────
class C:
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    CYAN    = "\033[96m"
    DIM     = "\033[2m"
    BOLD    = "\033[1m"
    RESET   = "\033[0m"


_TRANSIENT_PATTERNS = ["build", "sdkconfig", "sdkconfig.old",
                       "managed_components", "dependencies.lock"]

# Directories that are always transient (safe to delete without confirmation)
_TRANSIENT_DIRS = {"build", "managed_components", "__pycache__"}

# Repo-wide transients: scanned from repo root, not just examples_root
# __pycache__ lives mostly in tests/ so needs wider scope.
_REPO_WIDE_TRANSIENT_DIRS = {"__pycache__"}
_REPO_WIDE_SKIP_DIRS = {".git", "workspace", "managed_components", "build",
                        "node_modules", ".venv", "venv"}

# Directories that MAY contain user code — require explicit confirmation
_CONFIRM_DIRS = {"components"}


def _find_symlinks(directory: Path) -> list[Path]:
    """Find symlinks inside a directory (non-recursive, top-level only)."""
    if not directory.is_dir():
        return []
    return [child for child in directory.iterdir() if child.is_symlink()]


def _find_transient_dirs(examples_root: Path) -> tuple[list[Path], list[Path]]:
    """Find transient dirs and component dirs separately.
    Returns (auto_clean_dirs, confirm_dirs)."""
    auto = []
    confirm = []
    for pattern in _TRANSIENT_DIRS:
        for hit in examples_root.rglob(pattern):
            if not hit.is_dir():
                continue
            # Skip dirs nested inside other transients (build/managed_components/...)
            if any(p.name in _TRANSIENT_DIRS for p in hit.parents
                   if p != examples_root):
                continue
            auto.append(hit)
    for pattern in _CONFIRM_DIRS:
        for hit in examples_root.rglob(pattern):
            if not hit.is_dir():
                continue
            if any(p.name in _TRANSIENT_DIRS for p in hit.parents):
                continue
            confirm.append(hit)
    return sorted(auto), sorted(confirm)


def _find_transient_files(examples_root: Path) -> list[Path]:
    """Find transient files (sdkconfig, dependencies.lock, etc.)."""
    files = []
    for pattern in ("sdkconfig", "sdkconfig.old", "dependencies.lock"):
        for hit in examples_root.rglob(pattern):
            if hit.is_file() and not any(
                    p.name in _TRANSIENT_DIRS for p in hit.parents):
                files.append(hit)
    return sorted(files)


def _print_dir_tree(dirs: list[Path], examples_root: Path, label: str,
                    warn: bool = False):
    """Print directory list with color coding."""
    color = C.YELLOW if warn else C.CYAN
    print(f"\n{color}{C.BOLD}{label} ({len(dirs)}){C.RESET}")
    for d in dirs:
        rel = d.relative_to(examples_root)
        symlinks = _find_symlinks(d)
        sym_info = ""
        if symlinks:
            targets = [f"{C.DIM}{s.name} -> {os.readlink(s)}{C.RESET}" for s in symlinks]
            sym_info = f"  {C.CYAN}[symlinks: {', '.join(targets)}{C.CYAN}]{C.RESET}"
        print(f"  {C.RED}{rel}/{C.RESET}{sym_info}")


def _find_repo_wide_pycache(repo_root: Path) -> list[Path]:
    """Find __pycache__ dirs across repo (skipping .git, workspace, etc.)."""
    hits = []
    for d in repo_root.rglob("__pycache__"):
        if not d.is_dir():
            continue
        # Skip if any parent is in the skip list
        if any(p.name in _REPO_WIDE_SKIP_DIRS for p in d.parents
               if p != repo_root):
            continue
        hits.append(d)
    return sorted(hits)


def clean_repo_wide_pycache(repo_root: Path) -> int:
    """Remove __pycache__ dirs from entire repo (tests/, tools/, etc.)."""
    hits = _find_repo_wide_pycache(repo_root)
    if not hits:
        return 0
    _print_dir_tree(hits, repo_root, "Python __pycache__ (repo-wide)")
    for d in hits:
        shutil.rmtree(d, ignore_errors=True)
    print(f"  {C.GREEN}✓ Removed {len(hits)} __pycache__ dirs.{C.RESET}")
    return len(hits)


def clean_example_transients(examples_root: Path, confirm: bool = True) -> int:
    """Remove IDF build transients + component dirs from example dirs.

    - build/, managed_components/: deleted automatically
    - sdkconfig, sdkconfig.old, dependencies.lock: deleted automatically
    - components/: shown with symlink details, requires user typing 'YES'
    - Symlinks inside components/ are unlinked (NOT followed/deleted)
    """
    auto_dirs, confirm_dirs = _find_transient_dirs(examples_root)
    transient_files = _find_transient_files(examples_root)

    if not auto_dirs and not confirm_dirs and not transient_files:
        print("Nothing to clean.")
        return 0

    removed = 0

    # Phase 1: Auto-clean transient dirs + files
    if auto_dirs or transient_files:
        _print_dir_tree(auto_dirs, examples_root, "Transient directories (auto-clean)")
        print(f"\n  {C.DIM}+ {len(transient_files)} transient files "
              f"(sdkconfig, dependencies.lock, etc.){C.RESET}")
        for d in auto_dirs:
            shutil.rmtree(d)
            removed += 1
        for f in transient_files:
            f.unlink()
            removed += 1
        print(f"  {C.GREEN}✓ Removed {removed} items.{C.RESET}")

    # Phase 2: Component dirs — require explicit 'YES'
    if confirm_dirs:
        _print_dir_tree(confirm_dirs, examples_root,
                        "⚠️  Component directories (MAY contain your code)", warn=True)
        print(f"\n  {C.RED}{C.BOLD}These directories will be DELETED.{C.RESET}")
        print(f"  {C.DIM}Symlinks inside will be unlinked "
              f"(target repos are NOT deleted).{C.RESET}")

        if confirm:
            ans = input(f"\n  Type '{C.RED}YES{C.RESET}' to delete, "
                        f"anything else to skip: ").strip()
            if ans != "YES":
                print(f"  {C.YELLOW}Skipped component directories.{C.RESET}")
                return removed

        for d in confirm_dirs:
            for sym in _find_symlinks(d):
                sym.unlink()
            if d.exists():
                shutil.rmtree(d)
            removed += 1
        print(f"  {C.GREEN}✓ Removed {len(confirm_dirs)} component directories.{C.RESET}")

    return removed


# ═══════════════════════════════════════════════════════════════════════════════
# Build single
# ═══════════════════════════════════════════════════════════════════════════════


def build_single(project_dir: str, target: str, shell: str,
                 idf_path: str, fullclean: bool, output_dir: str) -> BuildResult:
    repo_root = _repo_root()
    _ensure_component_symlink(project_dir, repo_root)
    rel = os.path.relpath(project_dir, repo_root)
    transport = detect_transport(project_dir, target)

    skip = CompatMatrix.should_skip_example(rel, target)
    if skip:
        return BuildResult(rel, target, transport, Status.SKIP, reason=skip)

    short = rel.replace("fg/cp/examples/", "").replace("/", "__")
    log_path = os.path.join(output_dir, f"{short}__{target}.log")

    # Place build artifacts under output_dir mirroring example layout:
    #   <output_dir>/builds/<example_short>/<target>/
    build_dir = os.path.join(output_dir, "builds", short, target)
    os.makedirs(build_dir, exist_ok=True)

    return _do_build(project_dir, target, shell, idf_path, fullclean,
                     rel, transport, log_path, build_dir)


def _do_build(project_dir, target, shell, idf_path, fullclean,
              rel, transport, log_path, build_dir) -> BuildResult:
    runner = SHELL_RUNNERS.get(shell, BashRunner)(idf_path=idf_path)
    t0 = time.time()
    code, output = runner.idf_build(project_dir, target, fullclean=fullclean,
                                    build_dir=build_dir)
    elapsed = time.time() - t0

    Path(log_path).write_text(output)
    warnings = output.count("warning:")
    ok = code == 0 and "Project build complete" in output

    return BuildResult(rel, target, transport,
                       Status.OK if ok else Status.FAIL,
                       elapsed, warnings, log_path=log_path)


# ═══════════════════════════════════════════════════════════════════════════════
# Reporting
# ═══════════════════════════════════════════════════════════════════════════════

def _print_result_line(r: BuildResult):
    dur = f"{r.duration_s:.0f}s" if r.duration_s else "-"
    warn = str(r.warnings) if r.status != Status.SKIP else "-"
    tr = r.transport or "-"
    extra = f"  ({r.reason})" if r.status == Status.SKIP \
        else f"  -> {r.log_path}" if r.status == Status.FAIL else ""
    print(f"  {r.status.value:<8} {r.target:<12} {tr:<8} {dur:>5}  {warn:>5}  {r.example}{extra}")


def print_report(results: list[BuildResult], output_dir: str):
    print(f"\n{'=' * 80}")
    print(f"  Build summary (logs in {output_dir})")
    print(f"{'=' * 80}")
    print(f"  {'STATUS':<8} {'TARGET':<12} {'TRANSPORT':<8} {'DUR':>5}  {'WARN':>5}  PATH")
    print("-" * 80)
    for r in sorted(results, key=lambda x: (x.status.value, x.target, x.example)):
        _print_result_line(r)
    _print_totals(results)


def _print_totals(results: list[BuildResult]):
    ok = sum(1 for r in results if r.status == Status.OK)
    fail = sum(1 for r in results if r.status == Status.FAIL)
    skip = sum(1 for r in results if r.status == Status.SKIP)
    print(f"-" * 80)
    print(f"  Total: OK={ok}  FAIL={fail}  SKIP={skip}\n")
    if fail:
        print("  Failed builds:")
        for r in results:
            if r.status == Status.FAIL:
                print(f"    {r.example} [{r.target}] -> {r.log_path}")
        print()


def save_json_report(results: list[BuildResult], output_dir: Path):
    with open(output_dir / "report.json", "w") as f:
        json.dump([{
            "example": r.example, "target": r.target, "transport": r.transport,
            "status": r.status.value, "duration_s": round(r.duration_s, 1),
            "warnings": r.warnings, "reason": r.reason, "log_path": r.log_path,
        } for r in results], f, indent=2)


# ═══════════════════════════════════════════════════════════════════════════════
# Log rotation
# ═══════════════════════════════════════════════════════════════════════════════

def rotate_logs(output_dir: Path):
    old = output_dir.with_name(output_dir.name + ".old")
    if output_dir.exists():
        if old.exists():
            shutil.rmtree(old)
        output_dir.rename(old)
    output_dir.mkdir(parents=True, exist_ok=True)


# ═══════════════════════════════════════════════════════════════════════════════
# Artifact export
# ═══════════════════════════════════════════════════════════════════════════════

def _md5_file(path: Path) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b''):
            h.update(chunk)
    return h.hexdigest()


def _get_idf_version(idf_path: str) -> Optional[str]:
    ver_cmake = Path(idf_path) / "tools" / "cmake" / "version.cmake"
    if not ver_cmake.exists():
        return None
    content = ver_cmake.read_text()
    parts = {}
    for line in content.splitlines():
        for key in ("IDF_VERSION_MAJOR", "IDF_VERSION_MINOR", "IDF_VERSION_PATCH"):
            if f"set({key}" in line:
                val = line.split(key)[-1].strip().rstrip(")").strip()
                parts[key] = val
    if parts:
        return ".".join(parts.get(k, "?") for k in
                        ("IDF_VERSION_MAJOR", "IDF_VERSION_MINOR", "IDF_VERSION_PATCH"))
    return None


def _get_git_commit(repo_root: Path) -> Optional[str]:
    try:
        r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(repo_root),
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else None
    except Exception:
        return None


def _get_hosted_version(repo_root: Path) -> Optional[str]:
    yml = repo_root / "idf_component.yml"
    if not yml.exists():
        return None
    for line in yml.read_text().splitlines():
        if line.strip().startswith("version:"):
            return line.split(":", 1)[1].strip().strip('"')
    return None


def _collect_extra_info(repo_root: Path, idf_path: str) -> dict:
    info = {}
    v = _get_idf_version(idf_path)
    if v:
        info["idf_version"] = v
    c = _get_git_commit(repo_root)
    if c:
        info["eh_commit"] = c
    v = _get_hosted_version(repo_root)
    if v:
        info["eh_version"] = v
    return info


def _parse_flash_args(build_dir: Path) -> list[tuple[str, str]]:
    fa = build_dir / "flash_args"
    if not fa.exists():
        return []
    tokens = fa.read_text().strip().split()
    pairs = []
    i = 0
    while i < len(tokens):
        if tokens[i].startswith("0x") and i + 1 < len(tokens) \
                and not tokens[i + 1].startswith("-"):
            pairs.append((tokens[i], tokens[i + 1]))
            i += 2
        else:
            i += 1
    return pairs


def _copy_bins(build_dir: Path, dest: Path, flash_pairs: list) -> list:
    copied = []
    for offset, bin_name in flash_pairs:
        src = build_dir / bin_name
        if src.exists():
            shutil.copy2(src, dest / src.name)
            copied.append((offset, src.name))
    return copied


def _write_md5sums(dest: Path):
    with open(dest / "md5sums.txt", "w") as mf:
        for f in sorted(dest.glob("*.bin")):
            mf.write(f"{_md5_file(f)}  {f.name}\n")


def _write_flash_command(dest: Path, target: str, short: str, parts: list):
    with open(dest / "flash_command.txt", "w") as fc:
        fc.write(f"# Flash command for {short} on {target}\n")
        fc.write(f"esptool.py --chip {target} -b 460800 "
                 f"--before default_reset --after hard_reset "
                 f"write_flash --flash_mode dio --flash_freq 80m \\\n")
        for offset, name in parts:
            fc.write(f"  {offset} {name} \\\n")
        fc.write("\n")


def _write_extra_info(dest: Path, repo_root: Path, idf_path: str, project_dir: str):
    extra_dir = dest / "extra_info"
    extra_dir.mkdir(exist_ok=True)
    for key, val in _collect_extra_info(repo_root, idf_path).items():
        (extra_dir / f"{key}.txt").write_text(val + "\n")
    # Copy generated sdkconfig for cross-reference
    # Check build_dir (already resolved by caller) and in-source fallback
    for candidate in [dest.parent.parent / "config" / "sdkconfig.h",
                      Path(project_dir) / "build" / "config" / "sdkconfig.h",
                      Path(project_dir) / "sdkconfig"]:
        if candidate.exists():
            shutil.copy2(candidate, extra_dir / "sdkconfig")
            break


def export_artifacts(result: BuildResult, project_dir: str, target: str,
                     export_root: Path, repo_root: Path, idf_path: str,
                     output_dir: Optional[Path] = None) -> int:
    short = result.example.replace("fg/cp/examples/", "")
    short_flat = short.replace("/", "__")

    # Try output_dir build layout first, fall back to in-source build/
    if output_dir:
        build_dir = output_dir / "builds" / short_flat / target
    else:
        build_dir = Path(project_dir) / "build"
    if not build_dir.exists():
        # Fallback to in-source for backwards compat
        build_dir = Path(project_dir) / "build"
    if not build_dir.exists():
        return 0

    dest = export_root / short / target
    dest.mkdir(parents=True, exist_ok=True)

    flash_pairs = _parse_flash_args(build_dir)
    copied = _copy_bins(build_dir, dest, flash_pairs)
    _write_md5sums(dest)
    _write_flash_command(dest, target, short, copied)
    _write_extra_info(dest, repo_root, idf_path, project_dir)
    return len(copied)


# ═══════════════════════════════════════════════════════════════════════════════
# CLI — argument parsing
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description="ESP-Hosted example build orchestrator",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    _skip_targets = "--list-compat" in sys.argv or "--clean" in sys.argv
    p.add_argument("--build-targets", required=not _skip_targets,
                   metavar="TARGETS",
                   help="Comma-separated chip targets, or 'all' (e.g. --build-targets=esp32c6)")
    p.add_argument("--build-transports", default="default", metavar="TRANSPORTS",
                   help="sdio,spi-fd,spi-hd,uart,all,default (e.g. --build-transports=sdio)")
    p.add_argument("--build-examples", default="all", metavar="EXAMPLES",
                   help="Comma-separated example substrings, or 'all'")
    p.add_argument("--build-incremental", action="store_true",
                   help="Skip idf.py fullclean")
    p.add_argument("--jobs", type=int, default=4, metavar="N",
                   help="Parallel build jobs (default: 4)")
    p.add_argument("--filter", default="", metavar="REGEX",
                   help="Regex filter on example path")
    p.add_argument("--build-dir", default="build_logs", metavar="DIR",
                   help="Build output directory for artifacts, logs, and report (default: build_logs/)")
    p.add_argument("--export-fw-bins-dir", default=None, metavar="DIR",
                   help="Export firmware binaries + metadata per example/target")
    p.add_argument("--shell", default="bash", metavar="SHELL",
                   choices=sorted(SHELL_RUNNERS.keys()),
                   help="Shell for idf.py: bash, fish (default: bash)")
    p.add_argument("--idf-path", default=None, metavar="PATH",
                   help="ESP-IDF path (default: ~/esp-idf or $IDF_PATH)")
    p.add_argument("--no-rotate", action="store_true",
                   help="Don't rotate build_logs -> build_logs.old")
    p.add_argument("--list-compat", action="store_true",
                   help="Print compatibility matrix and exit")
    p.add_argument("--dry-run", action="store_true",
                   help="Show what would be built without building")
    p.add_argument("--clean", action="store_true",
                   help="Remove IDF build transients (build/, sdkconfig, managed_components, "
                        "dependencies.lock) from all example dirs and exit")
    return p.parse_args()


# ═══════════════════════════════════════════════════════════════════════════════
# CLI — list-compat
# ═══════════════════════════════════════════════════════════════════════════════

def cmd_list_compat():
    print("\nTransport compatibility:")
    for t in Transport:
        print(f"  {t.value:<8} -> {', '.join(sorted(TRANSPORT_TARGETS[t]))}")
    print("\nDefault transport per chip:")
    for chip in ALL_TARGETS:
        print(f"  {chip:<12} -> {DEFAULT_TRANSPORT.get(chip, Transport.SPI_FD).value}")
    print("\nExample exclusions:")
    for pat, chips in EXAMPLE_EXCLUDE.items():
        print(f"  {pat:<30} NOT on {', '.join(sorted(chips))}")
    print(f"\nNo-WiFi chips (skip all): {', '.join(sorted(NO_WIFI_TARGETS))}")


# ═══════════════════════════════════════════════════════════════════════════════
# CLI — setup
# ═══════════════════════════════════════════════════════════════════════════════

def setup_output(args, repo_root: Path) -> Path:
    output_dir = Path(args.build_dir)
    if not output_dir.is_absolute():
        output_dir = repo_root / output_dir
    if not args.no_rotate and not args.dry_run:
        rotate_logs(output_dir)
    else:
        output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def collect_projects(args, examples_root: Path, repo_root: Path) -> list[Path]:
    projects = discover_examples(examples_root)
    if args.build_examples != "all":
        subs = [s.strip() for s in args.build_examples.split(",")]
        projects = [p for p in projects if any(s in str(p) for s in subs)]
    if args.filter:
        pat = re.compile(args.filter)
        projects = [p for p in projects if pat.search(str(p.relative_to(repo_root)))]
    return projects


def make_jobs(projects: list[Path], targets: list[str]) -> list[tuple[str, str]]:
    return [(str(p), t) for p in projects for t in targets]


# ═══════════════════════════════════════════════════════════════════════════════
# CLI — execute
# ═══════════════════════════════════════════════════════════════════════════════

def print_live(result: BuildResult):
    if result.status == Status.SKIP:
        print(f"[SKIP] {result.example} [{result.target}] ({result.reason})")
    elif result.status == Status.OK:
        print(f"[OK]   {result.example} [{result.target}] "
              f"({result.duration_s:.0f}s, {result.warnings} warnings)")
    else:
        print(f"[FAIL] {result.example} [{result.target}] "
              f"({result.duration_s:.0f}s, see {result.log_path})")


def execute_builds(jobs, args, output_dir, idf_path, repo_root) -> list[BuildResult]:
    results = []
    fullclean = not args.build_incremental
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures = {
            pool.submit(build_single, p, t, args.shell,
                        idf_path, fullclean, str(output_dir)): (p, t)
            for p, t in jobs
        }
        for future in as_completed(futures):
            p, t = futures[future]
            try:
                result = future.result()
            except Exception as e:
                rel = os.path.relpath(p, repo_root)
                result = BuildResult(rel, t, "", Status.FAIL, reason=str(e))
            results.append(result)
            print_live(result)
    return results


def do_export(results, jobs, args, repo_root, idf_path, output_dir):
    export_root = Path(args.export_fw_bins_dir)
    if not export_root.is_absolute():
        export_root = repo_root / export_root
    export_root.mkdir(parents=True, exist_ok=True)

    exported = 0
    for r in results:
        if r.status != Status.OK:
            continue
        n = export_artifacts(r, str(repo_root / r.example), r.target,
                             export_root, repo_root, idf_path,
                             output_dir=output_dir)
        if n:
            exported += 1
    if exported:
        print(f"\n  Exported {exported} build(s) to {export_root}")


# ═══════════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    args = parse_args()

    if args.list_compat:
        cmd_list_compat()
        return

    repo_root = Path(__file__).resolve().parents[2]
    examples_root = repo_root / "fg" / "cp" / "examples"

    if args.clean:
        n = clean_example_transients(examples_root, confirm=True)
        n += clean_repo_wide_pycache(repo_root)
        print(f"\nTotal cleaned: {n} items.")
        return

    output_dir = setup_output(args, repo_root)
    idf_path = args.idf_path or os.environ.get("IDF_PATH", os.path.expanduser("~/esp-idf"))
    targets = CompatMatrix.resolve_targets(args.build_targets)

    projects = collect_projects(args, examples_root, repo_root)
    if not projects:
        print("No examples matched.", file=sys.stderr)
        return

    jobs = make_jobs(projects, targets)
    print(f"==> {len(projects)} example(s) x {len(targets)} target(s) = {len(jobs)} build(s)")
    print(f"==> Targets: {', '.join(targets)}")
    print(f"==> Transports: {args.build_transports}")
    print(f"==> Shell: {args.shell}, Jobs: {args.jobs}, Fullclean: {not args.build_incremental}")
    print(f"==> Output: {output_dir}\n")

    if args.dry_run:
        for p, t in jobs:
            rel = os.path.relpath(p, repo_root)
            skip = CompatMatrix.should_skip_example(rel, t)
            print(f"  {'[SKIP: ' + skip + ']' if skip else '[BUILD]'} {rel} [{t}]")
        return

    results = execute_builds(jobs, args, output_dir, idf_path, repo_root)

    if args.export_fw_bins_dir:
        do_export(results, jobs, args, repo_root, idf_path, output_dir)

    print_report(results, str(output_dir))
    save_json_report(results, output_dir)

    sys.exit(1 if any(r.status == Status.FAIL for r in results) else 0)


if __name__ == "__main__":
    main()
