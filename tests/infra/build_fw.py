"""Firmware build cache — the copy-on-write build layer for the test framework.

Principle (see .meta2/changes/eh-testlab-framework/execution-model.md §2):
  * Most suites share ONE binary and never rebuild it.
  * A suite whose sdkconfig `overlay` changes the resolved config gets its OWN
    binary, keyed by a content hash — the base is never mutated (COW level 1).
  * Per-run writable flash is a reflink copy (COW level 2): parallel runs and NVS
    writes never touch the cached base.

Cheap and reused across the whole run; safe to call concurrently for distinct keys.
"""
import fcntl
import hashlib
import importlib.util
import logging
import os
import shutil
import subprocess
import time
from pathlib import Path

from infra import lab  # single source of truth for the .work/ layout

_REPO = lab._REPO
_WORK = lab.WORK      # sandbox-bindable root; cache/ = reused build artifacts
_CACHE = lab.CACHE
_EH = _REPO / "tools" / "eh.py"

_log = logging.getLogger("eh")  # streamed live by pytest log_cli


def _env_float(key, default):
    try:
        return float(os.environ.get(key) or default)
    except (TypeError, ValueError):
        return float(default)


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _idf_path() -> str:
    eh = _load("eh", _EH)
    p = eh.resolve_idf()
    if not p:
        raise RuntimeError("ESP-IDF not found (run ./install.sh or eh.py set-idf-path)")
    return str(p)


def _idf_path_opt():
    """IDF path or None — the Linux (posix) host build needs no IDF (builds with
    the system toolchain), so an IDF-less host (RPi/imx/yocto) must not be a hard
    error there."""
    try:
        return _idf_path()
    except RuntimeError:
        return None


def _config_hash(example_rel: str, role: str, target: str, overlay) -> str:
    parts = [example_rel, role, target, *sorted(overlay or [])]
    return hashlib.sha1("|".join(parts).encode()).hexdigest()[:8]


# Two classes of build input, because they need different rebuild actions:
#  - CODE: a .c/.h edit → fast incremental `idf.py build`.
#  - CFG: a CMakeLists/Kconfig/sdkconfig.defaults/overlay change → a full
#    reconfigure (`set-target`), because `idf.py build` won't re-read defaults
#    once sdkconfig exists (the bug that left new Kconfig feats uncompiled).
_CODE_EXTS = {".c", ".cpp", ".cc", ".cxx", ".h", ".hpp", ".hh", ".inc",
              ".s", ".S", ".ld"}
_CFG_EXTS = {".cmake", ".csv", ".yml", ".yaml"}
_CFG_NAMES = {"CMakeLists.txt", "Kconfig", "Kconfig.projbuild", "idf_component.yml"}
_SRC_SKIP_DIRS = {"build", "managed_components", ".git", "__pycache__",
                  "node_modules", "dist", ".work"}


def _classify(fn: str):
    ext = os.path.splitext(fn)[1]
    if ext in _CODE_EXTS:
        return "src"
    # `Kconfig*` catches orsource'd sub-files (Kconfig.host.spi, Kconfig.cp.spi_hd,
    # Kconfig.ext, …), not just the literal "Kconfig" — else a per-transport Kconfig
    # edit silently reuses a stale sdkconfig on a same-overlay cached build.
    if (fn in _CFG_NAMES or fn.startswith("Kconfig") or ext in _CFG_EXTS
            or fn.startswith("sdkconfig.defaults")):
        return "cfg"
    return None


def _fingerprint_roots(example_rel: str, role: str):
    """The source trees a build compiles: the example role dir, the example's
    SIBLING common/ (shared across the example's roles, e.g. api_exerciser/common),
    the role's component tree, and the shared repo common/ + port/ trees. A change
    under any of these must invalidate the cache (missing the example-sibling
    common/ left a stale binary after editing examples/<ex>/common/*.c)."""
    src = _REPO / "examples" / example_rel / role
    ex_common = _REPO / "examples" / example_rel / "common"
    component = "coprocessor" if role == "cp" else "host"
    return [src, ex_common, _REPO / component, _REPO / "common", _REPO / "port"]


def _fingerprints(example_rel: str, role: str, target: str, overlay, inject=None):
    """Return (src_fp, cfg_fp): content-cheap fingerprints (path+mtime_ns+size)
    over the build's trees, split by class. One walk. A false positive (mtime
    touched, same content) costs a fast rebuild; a false negative (skip a real
    change) is what the wide roots prevent. Overlay lines + injected files
    (e.g. a staged CP OTA image) fold into cfg_fp so a changed input rebuilds."""
    hs, hc = hashlib.sha1(), hashlib.sha1()
    seed = "|".join([example_rel, role, target]).encode()
    hs.update(seed)
    hc.update(seed + b"|" + "|".join(sorted(overlay or [])).encode())
    for dest, srcpath in sorted((inject or {}).items()):
        try:
            st = os.stat(srcpath)
            hc.update(f"|inject:{dest}:{st.st_mtime_ns}:{st.st_size}".encode())
        except OSError:
            hc.update(f"|inject:{dest}:missing".encode())
    for root in _fingerprint_roots(example_rel, role):
        if not root.is_dir():
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in _SRC_SKIP_DIRS]
            for fn in filenames:
                cls = _classify(fn)
                if not cls:
                    continue
                fp = os.path.join(dirpath, fn)
                try:
                    st = os.stat(fp)
                except OSError:
                    continue
                rec = f"{os.path.relpath(fp, _REPO)}:{st.st_mtime_ns}:{st.st_size}".encode()
                (hs if cls == "src" else hc).update(rec)
    return hs.hexdigest(), hc.hexdigest()


def build(example_rel: str, role: str, target: str, overlay=None, inject=None) -> dict:
    """Build (or reuse from cache) firmware for examples/<example_rel>/<role> at
    <target>, with optional sdkconfig <overlay> lines. Returns
    {'merged', 'elf', 'dir'}. Cache hit needs BOTH a matching config (same
    example/role/target/overlay => same scratch dir) AND a matching source
    fingerprint — so a source edit rebuilds (incrementally) instead of silently
    reusing a stale binary. A different overlay => a new binary; base untouched.

    `inject` = {dest_rel: abs_src_file}: files staged into the COW build copy
    after the source copy (e.g. a CP app image dropped into the host's LittleFS
    OTA source dir). Sandbox-safe — writes land only under the cache copy — and
    fold into the cache key so a changed injected file rebuilds."""
    overlay = list(overlay or [])
    inject = dict(inject or {})
    src = _REPO / "examples" / example_rel / role
    if not (src / "CMakeLists.txt").is_file():
        raise FileNotFoundError(f"no such example role: {src}")
    key = _config_hash(example_rel, role, target, overlay)
    proj = _CACHE / f"{example_rel.replace('/', '__')}__{role}__{key}"
    common_src = _REPO / "examples" / example_rel / "common"
    has_common = common_src.is_dir()
    # A sibling common/ shared across roles needs a nested scratch (proj/<role>
    # + proj/common) so the role CMake's ../../common resolves in the COW copy
    # exactly as in-tree; without common/, the role dir IS the scratch (flat).
    workdir = (proj / role) if has_common else proj
    merged = workdir / "build" / "merged_flash.bin"
    srcfp_file = proj / ".srcfp"
    cfgfp_file = proj / ".cfgfp"
    src_fp, cfg_fp = _fingerprints(example_rel, role, target, overlay, inject)

    def _hit():
        elf = next(workdir.glob("build/*.elf"), None) if workdir.exists() else None
        if (merged.is_file() and elf and srcfp_file.is_file() and cfgfp_file.is_file()
                and srcfp_file.read_text().strip() == src_fp
                and cfgfp_file.read_text().strip() == cfg_fp):
            try:
                os.utime(proj)  # mark last-used so retention keeps active caches
            except OSError:
                pass
            return {"merged": str(merged), "elf": str(elf), "bin": None,
                    "dir": str(workdir), "build_log": str(workdir / "build.log")}
        return None

    hit = _hit()
    if hit:
        _log.info("[build] %s/%s [%s]%s — cache hit", example_rel, role, target,
                  f" overlay={'+'.join(overlay)}" if overlay else "")
        return hit

    # Per-config lock: parallel workers (--jobs N) or prebuild threads that need
    # the SAME config serialize here — one builds, the rest wait then reuse — so
    # a shared scratch build is never corrupted. Distinct configs build freely.
    locks = _WORK / ".locks"
    locks.mkdir(parents=True, exist_ok=True)
    with open(locks / f"{proj.name}.lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        # re-check under the lock: another worker may have finished while we waited
        hit = _hit()
        if hit:
            return hit

        # COW level 1: build in an isolated scratch copy so the example stays a
        # read-only input; absolutise override_paths so the copy still points at
        # the in-repo components. Rebuild action depends on WHAT changed:
        #  - config changed (or fresh) → `set-target` re-reads SDKCONFIG_DEFAULTS
        #    and does a clean reconfigure (a new Kconfig feat must actually
        #    compile in — `idf.py build` alone won't re-read defaults).
        #  - source only → refresh the copied sources and `idf.py build`
        #    INCREMENTALLY; component trees compile live via override_paths.
        runner = _load("esplab_runner", _REPO / "tools" / "esplab" / "runner.py")
        ignore = shutil.ignore_patterns(
            "build", "managed_components", "dependencies.lock",
            "sdkconfig", "sdkconfig.old", "*.bin")
        fresh = not workdir.exists()
        cfg_changed = fresh or not cfgfp_file.is_file() or \
            cfgfp_file.read_text().strip() != cfg_fp
        if fresh:
            workdir.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(src, workdir, ignore=ignore)
        else:
            shutil.copytree(src, workdir, ignore=ignore, dirs_exist_ok=True)
        if has_common:
            shutil.copytree(common_src, proj / "common", ignore=ignore, dirs_exist_ok=True)
        # Stage injected files into the COW copy (idempotent — re-applied each build,
        # since an incremental copytree of `src` never carries them). Sandbox-safe.
        for dest, srcpath in inject.items():
            d = workdir / dest
            d.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(srcpath, d)
        runner._absolutize_overrides(src, workdir)
        # gcov variant: instrument the SCRATCH copy only (never the real example).
        # Re-applied every build because an incremental copytree restores the
        # pristine CMakeLists; the inject is idempotent (marker-guarded).
        if _GCOV_SENTINEL in overlay:
            _inject_gcov_instrument(workdir / "CMakeLists.txt")
        defaults = "sdkconfig.defaults"
        if overlay:
            (workdir / ".ovl.defaults").write_text("\n".join(overlay) + "\n")
            defaults = "sdkconfig.defaults;.ovl.defaults"
        bt = _env_float("EH_BUILD_TIMEOUT", 1800)  # per-firmware build-phase cap
        build_log = workdir / "build.log"          # tee'd; the run report links here

        def _run_build(clean):
            """One build pass. clean=True → drop the (possibly wedged) build/ and
            reconfigure from defaults via set-target; clean=False → incremental
            `idf.py build`. Returns the subprocess rc (raises only on the timeout cap)."""
            if clean:
                # A build interrupted mid-flight (Ctrl-C, OOM, kill) — or one
                # configured under a different IDF python-env — leaves a build/ that
                # `set-target`'s fullclean REFUSES to delete; drop it ourselves so the
                # reconfigure self-heals.
                shutil.rmtree(workdir / "build", ignore_errors=True)
                cmd = (f'idf.py -D SDKCONFIG_DEFAULTS="{defaults}" '
                       f'set-target {target} build')
            else:
                cmd = 'idf.py build'  # source-only => incremental
            idf = _idf_path()
            script = (
                # Build in a clean IDF env: drop any Python-venv / IDF activation
                # inherited from the caller (the test runner's own venv, or a
                # different IDF sourced in the shell), and pin IDF_PATH to the
                # resolved IDF so export.sh activates *that* one — not whatever
                # was active. Otherwise idf.py runs against the wrong venv.
                f'unset VIRTUAL_ENV IDF_PYTHON_ENV_PATH PYTHONHOME PYTHONPATH; '
                f'export IDF_PATH="{idf}"; '
                f'. "{idf}/export.sh" >/dev/null 2>&1 || exit 90; '
                # ccache always on: the N transport variants of an example share
                # ~all TUs, so after the first cold build the rest (and every
                # post-edit rebuild) are compiler-cache hits. IDF gates ccache on
                # this env var; unset it defaults -DCCACHE_ENABLE=0.
                f'export IDF_CCACHE_ENABLE=1; '
                f'cd "{workdir}" || exit 91; '
                f'{cmd} && idf.py merge-bin -o merged_flash.bin'
            )
            _log.info("[build] %s/%s [%s]%s — %s%s (may take minutes on a first build)…",
                      example_rel, role, target,
                      f" overlay={'+'.join(overlay)}" if overlay else "",
                      "clean reconfigure" if clean else "incremental",
                      " (fresh)" if fresh else "")
            with open(build_log, "w") as blf:
                return subprocess.run(["bash", "-lc", script], timeout=bt,
                                      stdout=blf, stderr=subprocess.STDOUT).returncode

        t0 = time.time()
        try:
            rc = _run_build(clean=cfg_changed)
            if rc != 0:
                # One clean-rebuild retry before we give up: an incremental build over
                # a wedged/stale scratch (classic: "project configured with a different
                # python, run idf.py fullclean") fails, but a from-scratch reconfigure
                # succeeds. A genuine code/config break fails the retry too → terminate
                # rather than run tests against a broken or stale binary.
                _log.warning("[build] %s/%s [%s] build failed — one clean-rebuild retry",
                             example_rel, role, target)
                rc = _run_build(clean=True)
        except subprocess.TimeoutExpired:
            raise RuntimeError(
                f"build phase exceeded EH_BUILD_TIMEOUT={bt:.0f}s: "
                f"{example_rel}/{role} [{target}] (raise it or check the build); "
                f"log: {build_log}")
        if rc != 0:
            tail = ""
            try:
                tail = build_log.read_text(errors="replace")[-2000:]
            except Exception:
                pass
            if "No module named" in tail or "not spawned within an ESP-IDF" in tail:
                raise RuntimeError(
                    f"ESP-IDF Python env not ready for {example_rel}/{role} [{target}] "
                    f"— idf.py couldn't import its venv packages. Finish the install "
                    f"(./install.sh) or repair tools, then retry. See {build_log}")
            # Inline the log tail into the exception: on CI the container is torn
            # down after the job, so the tee'd build.log is otherwise unreachable —
            # this puts the real compiler/cmake error in the pytest FAILURES section.
            raise RuntimeError(f"firmware build failed (clean-rebuild retry also failed): "
                               f"{example_rel}/{role} [{target}] — see {build_log}\n"
                               f"----- build.log tail -----\n{tail.strip() or '(empty)'}\n"
                               f"----- end build.log -----")
        elf = next(workdir.glob("build/*.elf"))
        srcfp_file.write_text(src_fp)  # re-fingerprint AFTER a good build
        cfgfp_file.write_text(cfg_fp)
        _log.info("[build] %s/%s [%s] — done in %.0fs", example_rel, role, target, time.time() - t0)
        return {"merged": str(merged), "elf": str(elf), "bin": None,
                "dir": str(workdir), "build_log": str(build_log)}


def stash_logs(dest_dir, **named) -> None:
    """Copy each build result's build.log into dest_dir as <name>_build.log, so a
    test's build + cp + host logs co-locate in the run report dir for drill-down."""
    for name, fw in named.items():
        bl = (fw or {}).get("build_log")
        if bl and os.path.exists(bl):
            try:
                shutil.copyfile(bl, os.path.join(str(dest_dir), f"{name}_build.log"))
            except OSError:
                pass


def prebuild(specs) -> None:
    """Speculative prebuild-ahead: build several (example, role, target, overlay)
    specs concurrently before tests fan out, so the first run overlaps builds
    instead of stalling per-suite. Each build is lock-guarded, so this is safe
    even when xdist workers call it at once. Failures are swallowed — the lazy
    build in the bench fixture is the fallback."""
    from concurrent.futures import ThreadPoolExecutor
    specs = list(specs)
    if not specs:
        return
    workers = min(len(specs), max(2, (os.cpu_count() or 2) - 1))
    with ThreadPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(build, *s) for s in specs]
        for f in futs:
            try:
                f.result()
            except Exception:
                pass


def slave_overlay(cp_target):
    """Bench-derived sdkconfig overlay that tells the host which CP chip it
    drives, so esp_wifi_remote exports the right slave SoC caps (e.g.
    SLAVE_SOC_WIFI_SUPPORT_5G for a C5). The chip is a bench property (the CP the
    bench actually has), injected as an overlay — NOT pinned per example. Applied
    over the example's neutral default; the choice's later value wins."""
    if not cp_target:
        return []
    return [f"CONFIG_SLAVE_IDF_TARGET_{cp_target.upper()}=y"]


# --- on-target gcov (app_trace GCOV over USB-Serial-JTAG) --------------------
# A comment line (harmless in sdkconfig.defaults) that rides in the overlay so it
# folds into the config hash → the instrumented firmware gets its OWN scratch dir
# and never clobbers the normal build; build() detects it to inject --coverage.
_GCOV_SENTINEL = "# EH_GCOV_INSTRUMENT"
_GCOV_MARK = "EH gcov instrumentation"
# Instrument ONLY ESP-Hosted components (eh_*, msg_codec, main) — whole-project
# --coverage overflows DRAM by ~700KB (gcov counters for all of IDF/WiFi/mbedTLS
# land in .dram0.data). Third-party/IDF stay uninstrumented; the aggregator
# filters generated files (gen_v2.c) out anyway. Appended AFTER project() because
# that's when the component lib targets exist.
_GCOV_CMAKE = (
    f"\n# --- {_GCOV_MARK} (scratch copy only) ---\n"
    "idf_build_get_property(_eh_cov_comps BUILD_COMPONENTS)\n"
    "foreach(_eh_c IN LISTS _eh_cov_comps)\n"
    '  if(_eh_c MATCHES "^eh_" OR _eh_c STREQUAL "msg_codec" OR _eh_c STREQUAL "main")\n'
    "    idf_component_get_property(_eh_lib ${_eh_c} COMPONENT_LIB)\n"
    "    if(_eh_lib)\n"
    "      target_compile_options(${_eh_lib} PRIVATE --coverage)\n"
    "      target_link_libraries(${_eh_lib} INTERFACE --coverage)\n"
    "    endif()\n"
    "  endif()\n"
    "endforeach()\n\n")


def gcov_overlay():
    """Overlay that turns a CP/MCU firmware into a coverage build: enables the
    app_trace GCOV path (dumped later by OpenOCD `esp gcov` over USB-Serial-JTAG)
    and flags build() to whole-project-instrument the scratch copy. Union the
    resulting .gcda with the Linux-host gcov for one full-codebase number."""
    return [_GCOV_SENTINEL,
            "CONFIG_APPTRACE_ENABLE=y",
            "CONFIG_APPTRACE_DEST_JTAG=y",
            "CONFIG_APPTRACE_GCOV_ENABLE=y",
            "CONFIG_APPTRACE_LOCK_ENABLE=y"]


def _inject_gcov_instrument(cml):
    """Append per-component --coverage AFTER project() in the scratch CMakeLists
    (component lib targets only exist post-project()). Idempotent; only ever
    touches the COW copy, never the real example."""
    if not cml.is_file():
        return
    txt = cml.read_text()
    if _GCOV_MARK in txt:
        return
    cml.write_text(txt.rstrip() + "\n" + _GCOV_CMAKE)


def build_linux_host(example_rel: str, role: str, overlay=None) -> dict:
    """Build (or reuse) a Linux userspace host binary (posix target) OUT-OF-TREE.

    The example is a read-only input: it's COW-copied to a per-config scratch
    (config-hash keyed, sdkconfig excluded from the copy) so the generated
    sdkconfig lives in OUR workspace, never inside the example — no stale in-tree
    sdkconfig to fight the layered defaults. Optional ordered <overlay> sdkconfig
    lines (bench/suite/case layers) apply on top via EH_SDKCONFIG_OVERLAY. Same
    (example, role, overlay) + unchanged sources => cache hit, no rebuild.

    Mirrors build() (the ESP path) but drives the posix hosted build via eh.py."""
    overlay = list(overlay or [])
    target = "posix"
    src = _REPO / "examples" / example_rel / role
    if not (src / "CMakeLists.txt").is_file():
        raise FileNotFoundError(f"no such linux host: {src}")
    key = _config_hash(example_rel, role, target, overlay)
    proj = _CACHE / f"{example_rel.replace('/', '__')}__{role}__{key}"
    common_src = _REPO / "examples" / example_rel / "common"
    has_common = common_src.is_dir()
    workdir = (proj / role) if has_common else proj
    srcfp_file, cfgfp_file = proj / ".srcfp", proj / ".cfgfp"
    src_fp, cfg_fp = _fingerprints(example_rel, role, target, overlay)

    def _bin():
        if not workdir.exists():
            return None
        for p in sorted(workdir.glob("build/**/*")):
            if p.is_file() and os.access(p, os.X_OK) and (
                    "c_app" in p.name or "py_app" in p.name):
                return p
        return None

    def _hit():
        b = _bin()
        if (b and srcfp_file.is_file() and cfgfp_file.is_file()
                and srcfp_file.read_text().strip() == src_fp
                and cfgfp_file.read_text().strip() == cfg_fp):
            try:
                os.utime(proj)
            except OSError:
                pass
            return {"merged": None, "elf": None, "bin": str(b),
                    "dir": str(workdir), "build_log": str(workdir / "build.log")}
        return None

    hit = _hit()
    if hit:
        _log.info("[build-linux] %s/%s%s — cache hit", example_rel, role,
                  f" overlay={'+'.join(overlay)}" if overlay else "")
        return hit

    locks = _WORK / ".locks"
    locks.mkdir(parents=True, exist_ok=True)
    with open(locks / f"{proj.name}.lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        hit = _hit()
        if hit:
            return hit
        runner = _load("esplab_runner", _REPO / "tools" / "esplab" / "runner.py")
        # Exclude sdkconfig* + runtime artifacts from the copy → the scratch's
        # sdkconfig is generated fresh from defaults+overlay (never inherited),
        # and root-owned run leftovers (.nvs from a sudo'd host, staged bin/)
        # don't break the copy or pollute the workspace.
        ignore = shutil.ignore_patterns(
            "build", "managed_components", "dependencies.lock",
            "sdkconfig", "sdkconfig.old", "*.bin", ".nvs", "bin", ".eh.target")
        fresh = not workdir.exists()
        if fresh:
            workdir.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(src, workdir, ignore=ignore)
        else:
            shutil.copytree(src, workdir, ignore=ignore, dirs_exist_ok=True)
        if has_common:
            shutil.copytree(common_src, proj / "common", ignore=ignore, dirs_exist_ok=True)
        runner._absolutize_overrides(src, workdir)
        env_overlay = ""
        if overlay:
            ovl = workdir / ".ovl.defaults"
            ovl.write_text("\n".join(overlay) + "\n")
            env_overlay = f'export EH_SDKCONFIG_OVERLAY="{ovl}"; '
        build_log = workdir / "build.log"
        idf = _idf_path_opt()  # optional: posix host builds with the system toolchain
        idf_src = f'. "{idf}/export.sh" >/dev/null 2>&1; ' if idf else ''
        script = (
            f'export EH_PATH="{_REPO}"; '
            f'{idf_src}'
            f'{env_overlay}'
            f'cd "{workdir}" || exit 91; '
            f'python3 "{_EH}" set-target posix && python3 "{_EH}" build'
        )
        def _run_linux_build(clean):
            if clean:  # drop a wedged build/ so set-target reconfigures from scratch
                shutil.rmtree(workdir / "build", ignore_errors=True)
            _log.info("[build-linux] %s/%s%s — %s%s…", example_rel, role,
                      f" overlay={'+'.join(overlay)}" if overlay else "",
                      "clean rebuild" if clean else "building",
                      " (fresh)" if fresh else "")
            with open(build_log, "w") as blf:
                return subprocess.run(["bash", "-lc", script], stdout=blf,
                                      stderr=subprocess.STDOUT).returncode

        rc = _run_linux_build(clean=False)
        if rc != 0:
            # One clean-rebuild retry (same policy as the ESP build path): a wedged
            # scratch self-heals; a real code/config break fails again → terminate.
            _log.warning("[build-linux] %s/%s build failed — one clean-rebuild retry",
                         example_rel, role)
            rc = _run_linux_build(clean=True)
        if rc != 0:
            raise RuntimeError(f"linux host build failed (clean-rebuild retry also failed): "
                               f"{example_rel}/{role} — see {build_log}")
        b = _bin()
        if not b:
            raise RuntimeError(f"linux host build produced no binary: {workdir}")
        srcfp_file.write_text(src_fp)
        cfgfp_file.write_text(cfg_fp)
        return {"bin": str(b), "dir": str(workdir), "build_log": str(build_log)}


def cow_flash(merged: str, dst_dir) -> str:
    """Per-run writable flash image (COW level 2). reflink where the fs supports it
    (btrfs/xfs: instant, zero extra disk), else a plain copy. The cached base is
    never written to, so runs are isolated and reproducible. The dest name is
    derived from the source so distinct images (e.g. cp vs host) never collide in
    the same dir."""
    src = Path(merged)
    tag = hashlib.sha1(str(src.resolve()).encode()).hexdigest()[:8]
    dst = Path(dst_dir) / f"flash_{tag}.bin"
    # reflink is a Linux (btrfs/xfs) fast-path; `cp --reflink` doesn't exist on
    # macOS `cp` (errors out). Try it, fall back to a portable copy everywhere else.
    try:
        subprocess.run(["cp", "--reflink=auto", merged, str(dst)],
                       check=True, capture_output=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        shutil.copyfile(merged, str(dst))
    return str(dst)
