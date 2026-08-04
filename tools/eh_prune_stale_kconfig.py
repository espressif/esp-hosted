#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Prune stale Kconfig assignments from example `sdkconfig.defaults*` files, driven
by the ACTUAL "unknown kconfig symbol" warnings emitted by a build / kconfgen run.

Why evidence-driven (not a hardcoded symbol list): the same legacy symbol can be
live on one target and dead on another. e.g. `CONFIG_ESP32_DEFAULT_CPU_FREQ_240`
is auto-renamed and honored on esp32, but `..._120` has no rename on esp32c2 and
is silently dropped there. A symbol IDF still understands NEVER appears as
"unknown", so this tool never touches it -> no behaviour change, no guessing.

It only edits target-overlay files (`sdkconfig.defaults.<target>`), because those
are used by exactly one chip, so a symbol reported unknown there is unknown for
that file, period. Shared base `sdkconfig.defaults` hits are listed for manual
review (a base symbol may still be valid on some other target) unless --allow-base.

Usage:
  # capture a run (serial keeps warnings on stdout):
  JOBS=1 DOCKER='sudo docker' bash tools/ci_local_build_matrix.sh 2>&1 | tee /tmp/eh.log
  python3 tools/eh_prune_stale_kconfig.py /tmp/eh.log            # dry-run
  python3 tools/eh_prune_stale_kconfig.py /tmp/eh.log --apply    # edit files
  # JOBS>1 keeps per-cell logs in a temp dir; point at that dir instead:
  python3 tools/eh_prune_stale_kconfig.py <logdir> --apply
"""
import argparse
import os
import re
import sys
from collections import defaultdict

WARN = re.compile(r"unknown kconfig symbol '([^']+)' assigned to '[^']*' in (\S+)")
OVERLAY = re.compile(r"sdkconfig\.defaults\.[A-Za-z0-9_]+$")


def read_input(path):
    if path and os.path.isdir(path):
        buf = []
        for root, _, files in os.walk(path):
            for f in files:
                try:
                    buf.append(open(os.path.join(root, f), errors="ignore").read())
                except OSError:
                    pass
        return "\n".join(buf)
    return open(path, errors="ignore").read() if path else sys.stdin.read()


def repo_rel(p):
    for pre in ("/esp_hosted/", "/repo/"):
        if p.startswith(pre):
            return p[len(pre):]
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", help="build log file or dir (default: stdin)")
    ap.add_argument("--apply", action="store_true", help="edit files (default: dry-run)")
    ap.add_argument("--allow-base", action="store_true",
                    help="also prune shared base sdkconfig.defaults (off by default)")
    args = ap.parse_args()

    hits = defaultdict(set)  # repo-relative file -> {symbols}
    for sym, path in WARN.findall(read_input(args.log)):
        p = repo_rel(path)
        if "sdkconfig.defaults" not in os.path.basename(p):
            continue  # ignore generated files (override-result.sdkconfig, /tmp/kconfgen_*)
        hits[p].add(sym)

    if not hits:
        print("No 'unknown kconfig symbol' warnings found in input.")
        return 0

    removed_total, overlay_files, skipped_base = 0, 0, {}
    for path in sorted(hits):
        if not OVERLAY.search(path) and not args.allow_base:
            skipped_base[path] = sorted(hits[path])
            continue
        if not os.path.isfile(path):
            print(f"!! not found (skipped): {path}")
            continue
        wanted = {f"CONFIG_{s}" for s in hits[path]}
        kept, removed = [], []
        for line in open(path):
            key = line.strip().lstrip("#").strip().split("=", 1)[0].strip()
            (removed if key in wanted else kept).append(line)
        if removed:
            overlay_files += 1
            removed_total += len(removed)
            print(f"\n{path}  (-{len(removed)})")
            for r in removed:
                print(f"    - {r.rstrip()}")
            if args.apply:
                with open(path, "w") as fh:
                    fh.writelines(kept)

    if skipped_base:
        print("\nSKIPPED shared base files (review manually, or --allow-base):")
        for p, syms in skipped_base.items():
            print(f"    {p}: {syms}")

    print(f"\n{'APPLIED' if args.apply else 'DRY-RUN'}: "
          f"{removed_total} line(s) across {overlay_files} overlay file(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
