"""
Build orchestration — build CP + host firmware, apply patches.

Build strategies (configured via env.json "build.strategy"):
  "reuse"    — incremental build, skip clean (fastest)
  "clean"    — always clean build (rm -rf build/sdkconfig)
  "fallback" — try incremental, auto-retry with clean on failure (default)

Workspace isolation:
  When workspace_dir is provided, all build artifacts (build/, sdkconfig)
  go there. Source project dir is read-only — no pollution.
  Uses IDF's -B and -DSDKCONFIG= flags.
"""

import os
import subprocess
import re
import glob
from pathlib import Path


def _repo_root() -> str:
    """Resolve repo root from this file's location (tests/infra/builder.py).
    Used only as a fallback when a project is not inside a git repo."""
    return str(Path(__file__).resolve().parents[2])


def _find_repo_root(project_dir: str) -> str:
    """Walk up from project_dir to find the containing git repo root.

    CP and Host projects live in different repos (esp_hosted.new vs esp_hosted_mcu6),
    so the symlink target must be derived from the project's own location, not
    from where this file lives.
    """
    p = Path(project_dir).resolve()
    for parent in [p] + list(p.parents):
        if (parent / ".git").exists():
            return str(parent)
    return _repo_root()


def _project_uses_managed_esp_hosted(project_dir: str) -> bool:
    """True if main/idf_component.yml declares espressif/esp_hosted as a dependency.

    Managed-component projects get esp_hosted from the IDF component registry into
    managed_components/ — a local components/esp_hosted symlink would collide.
    """
    yml = Path(project_dir) / "main" / "idf_component.yml"
    if not yml.exists():
        return False
    try:
        return "espressif/esp_hosted" in yml.read_text()
    except OSError:
        return False


def _ensure_component_symlink(project_dir: str):
    """Validate and (if appropriate) create components/esp_hosted symlink.

    Rule: a local components/esp_hosted symlink must point INTO the same repo
    as the project. A symlink pointing to a different repo is a stale/buggy
    artifact (e.g. from an earlier buggy helper run) and is removed.

    Creation policy:
    - Project without `espressif/esp_hosted` in main/idf_component.yml
      (CP-style projects in esp_hosted.new): auto-create symlink to repo root.
      These projects have `REQUIRES esp_hosted` and need local discovery.
    - Project with managed-component declaration (host examples): do NOT
      auto-create. Developer may intentionally maintain a local-source
      override (pointing inside their own repo) or leave IDF to fetch the
      managed copy. Either is respected as long as the override is intra-repo.
    """
    project = Path(project_dir).resolve()
    comp_dir = project / "components"
    link = comp_dir / "esp_hosted"
    project_repo = Path(_find_repo_root(project_dir)).resolve()

    if link.is_symlink():
        target = (link.parent / os.readlink(link)).resolve()
        try:
            target.relative_to(project_repo)
            # Intra-repo symlink — respect it (intentional local-source override).
            return
        except ValueError:
            # Cross-repo target — stale/buggy, remove it.
            link.unlink()
    elif link.exists():
        # Real file or directory — leave it alone.
        return

    # No symlink present. Auto-create only for local-source (non-managed) projects.
    if _project_uses_managed_esp_hosted(project_dir):
        return
    comp_dir.mkdir(exist_ok=True)
    target = os.path.relpath(project_repo, comp_dir)
    link.symlink_to(target)


def _extract_build_error(build_dir, raw_output):
    """Extract meaningful error from build output + log files.
    Shows the actual compiler error, not just 'ninja failed'."""
    # Find error lines from build output (compiler errors)
    errors = []
    for l in raw_output.split('\n'):
        ls = l.strip()
        if 'error:' in ls.lower() and not ls.startswith('warning'):
            # Strip long paths to just filename:line
            # /very/long/path/to/file.c:429:37: error: msg → file.c:429: error: msg
            if ': error:' in ls:
                parts = ls.split(': error:', 1)
                path_part = parts[0]
                # Get just filename:line from path
                fname = os.path.basename(path_part.split(':')[0]) if ':' in path_part else path_part
                linenum = path_part.split(':')[1] if ':' in path_part and len(path_part.split(':')) > 1 else ''
                short = f'{fname}:{linenum}: error:{parts[1]}'
                errors.append(short[:150])
            else:
                errors.append(ls[:150])

    if errors:
        return '\n    '.join(errors[:3])
    elif 'cmake error' in raw_output.lower():
        return 'CMake configuration failed'
    else:
        # Last non-empty lines
        lines = [l.strip() for l in raw_output.strip().split('\n') if l.strip()]
        return lines[-1][:150] if lines else 'Unknown error'


def _run(cmd, cwd=None, timeout=600):
    r = subprocess.run(
        cmd, shell=True, cwd=cwd,
        capture_output=True, text=True, timeout=timeout
    )
    return r.returncode == 0, r.stdout + r.stderr


def eh_test_build_apply_sdkconfig_extra(target_dir, extra_lines):
    """
    Write extra sdkconfig lines to a separate test overlay file.
    Uses IDF's SDKCONFIG_DEFAULTS to chain: original defaults + test overlay.
    """
    if not extra_lines:
        return None
    overlay_path = os.path.join(target_dir, 'sdkconfig.test_overlay')
    with open(overlay_path, 'w') as f:
        f.write('# Auto-generated test overlay — do not commit\n')
        for line in extra_lines:
            f.write(line + '\n')
    return overlay_path


def _setup_sdkconfig_defaults_env(app_path, target, overlay_path=None):
    """Set SDKCONFIG_DEFAULTS env var to chain defaults files.
    IDF uses semicolons to separate multiple defaults files."""
    files = []
    base = os.path.join(app_path, 'sdkconfig.defaults')
    if os.path.exists(base):
        files.append(base)
    chip_specific = os.path.join(app_path, f'sdkconfig.defaults.{target}')
    if os.path.exists(chip_specific):
        files.append(chip_specific)
    if overlay_path and os.path.exists(overlay_path):
        files.append(overlay_path)
    if files:
        os.environ['SDKCONFIG_DEFAULTS'] = ';'.join(files)


def _clean_workspace(build_dir, sdkconfig):
    """Remove workspace build artifacts completely — forces fresh cmake."""
    if os.path.exists(build_dir):
        import shutil
        shutil.rmtree(build_dir, ignore_errors=True)
    if os.path.exists(sdkconfig):
        os.remove(sdkconfig)


def _idf_set_target(app_path, target, idf_flags=''):
    """Run idf.py set-target with optional workspace flags.
    -D cmake defines are stripped — set-target doesn't forward them."""
    st_flags = ' '.join(f for f in idf_flags.split() if not f.startswith('-D'))
    ok, out = _run(f'idf.py {st_flags} set-target {target}', cwd=app_path)
    if not ok:
        return False, f'set-target failed: {out[-300:]}'
    return True, ''


def _idf_build(app_path, idf_flags='', timeout=600):
    """Run idf.py build with optional workspace flags."""
    return _run(f'idf.py {idf_flags} build', cwd=app_path, timeout=timeout)


def eh_test_build_project(app_path, target, sdkconfig_extra=None,
                          strategy='fallback', workspace_dir=None):
    """
    Build an IDF project for the given target.

    Args:
        app_path: path to the IDF project (CMakeLists.txt location) — read-only
        target: IDF target (esp32c6, esp32p4, etc)
        sdkconfig_extra: list of config lines to append via overlay
        strategy: "reuse" | "clean" | "fallback"
        workspace_dir: if set, all build artifacts go here (isolates source)

    Returns: (success: bool, message: str, build_dir: str)
    """
    # Ensure components/esp_hosted symlink exists in the project source tree
    # (required because CP/Host examples have `REQUIRES esp_hosted` in main/CMakeLists.txt)
    _ensure_component_symlink(app_path)

    # Determine paths — workspace or in-tree
    if workspace_dir:
        os.makedirs(workspace_dir, exist_ok=True)
        build_dir = os.path.join(workspace_dir, 'build')
        sdkconfig = os.path.join(workspace_dir, 'sdkconfig')
        idf_flags = f'-B {build_dir} -DSDKCONFIG={sdkconfig}'
    else:
        build_dir = os.path.join(app_path, 'build')
        sdkconfig = os.path.join(app_path, 'sdkconfig')
        idf_flags = ''

    # Write test overlay (to workspace, not source dir)
    overlay_dir = workspace_dir or app_path
    overlay = eh_test_build_apply_sdkconfig_extra(overlay_dir, sdkconfig_extra) if sdkconfig_extra else None

    # Set SDKCONFIG_DEFAULTS chaining: source defaults + chip-specific + test overlay
    _setup_sdkconfig_defaults_env(app_path, target, overlay)

    if strategy == 'clean':
        _clean_workspace(build_dir, sdkconfig)
        ok, msg = _idf_set_target(app_path, target, idf_flags)
        if not ok:
            return False, msg, build_dir
        ok, out = _idf_build(app_path, idf_flags)
        if not ok:
            return False, _extract_build_error(build_dir, out), build_dir
        return True, 'Build OK (clean)', build_dir

    # "reuse" or "fallback" — try incremental first
    needs_retarget = True
    if os.path.exists(sdkconfig):
        with open(sdkconfig) as f:
            content = f.read()
        if f'CONFIG_IDF_TARGET="{target}"' in content:
            needs_retarget = False
            if sdkconfig_extra:
                needs_retarget = True

    if needs_retarget:
        _clean_workspace(build_dir, sdkconfig)
        ok, msg = _idf_set_target(app_path, target, idf_flags)
        if not ok:
            return False, msg, build_dir

    ok, out = _idf_build(app_path, idf_flags)
    if ok:
        return True, 'Build OK', build_dir

    if strategy == 'reuse':
        return False, _extract_build_error(build_dir, out), build_dir

    # "fallback" — retry with clean build
    print('  Build failed, retrying with clean build...')
    _clean_workspace(build_dir, sdkconfig)
    ok, msg = _idf_set_target(app_path, target, idf_flags)
    if not ok:
        return False, msg, build_dir
    ok, out = _idf_build(app_path, idf_flags)
    if not ok:
        return False, _extract_build_error(build_dir, out), build_dir
    return True, 'Build OK (clean retry)', build_dir


def eh_test_build_apply_patch(app_path, find_str, old_block, new_block):
    """
    Patch a managed component file (e.g., fix assert(len < 64)).
    Searches recursively under managed_components/ in the project dir.

    Returns: (patched: bool, message: str)
    """
    mc_dir = os.path.join(app_path, 'managed_components')
    if not os.path.exists(mc_dir):
        return False, 'No managed_components directory'

    patched = False
    for root, dirs, files in os.walk(mc_dir):
        for fname in files:
            if not fname.endswith('.c'):
                continue
            fpath = os.path.join(root, fname)
            with open(fpath) as f:
                content = f.read()
            if find_str in content:
                content = content.replace(old_block.strip(), new_block.strip())
                with open(fpath, 'w') as f:
                    f.write(content)
                patched = True

    if patched:
        return True, 'Patched'
    return False, 'No files needed patching'
