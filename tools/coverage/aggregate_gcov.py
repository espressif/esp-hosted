#!/usr/bin/env python3
"""Aggregate gcov line coverage of ESP-Hosted's OWN code across one or more
instrumented build dirs (union by source file+line), excluding generated
protobuf and vendored submodules. Merges runs from ANY substrate (emu-mcu /
emu-linux / hw-linux) — pass every build-cov dir; coverage is the union.

Usage: aggregate_gcov.py <repo_root> <build_dir> [<build_dir> ...]
"""
import subprocess, json, os, sys

# Only count ESP-Hosted's own source; drop generated + submodules + framework.
INCLUDE = ('/host/', '/port/', '/common/')
EXCLUDE = ('managed_components', '/vendor/', 'esp-idf', '/build/', 'build-cov',
           'third_party', 'protobuf-c', 'gen_v1', 'gen_v2', '.pb-c',
           'esp_wifi_remote/esp_wifi_remote')  # the esp_wifi_remote submodule


def collect(build_dirs):
    files = {}  # path -> {'cov': set, 'tot': set}   (union across all builds)
    dec = json.JSONDecoder()
    for bd in build_dirs:
        for g in subprocess.run(['find', bd, '-name', '*.gcda'],
                                capture_output=True, text=True).stdout.split():
            out = subprocess.run(['gcov', '--json-format', '--stdout', g],
                                 capture_output=True, text=True).stdout.strip()
            i = 0
            while i < len(out):
                try:
                    obj, i = dec.raw_decode(out, i)
                except Exception:
                    break
                while i < len(out) and out[i] in ' \n\r\t':
                    i += 1
                for f in obj.get('files', []):
                    fp = f['file']
                    if not any(k in fp for k in INCLUDE):
                        continue
                    if any(k in fp for k in EXCLUDE):
                        continue
                    d = files.setdefault(fp, {'cov': set(), 'tot': set()})
                    for L in f.get('lines', []):
                        d['tot'].add(L['line_number'])
                        if L.get('count', 0) > 0:
                            d['cov'].add(L['line_number'])
    return files


def main():
    repo, build_dirs = sys.argv[1], sys.argv[2:]
    files = collect(build_dirs)
    agg = {}
    for fp, d in files.items():
        b = next(k.strip('/') for k in INCLUDE if k in fp)
        a = agg.setdefault(b, [0, 0]); a[0] += len(d['cov']); a[1] += len(d['tot'])
    tc = sum(len(d['cov']) for d in files.values())
    tt = sum(len(d['tot']) for d in files.values())
    print(f"=== ESP-Hosted line coverage ({len(files)} files, "
          f"{len(build_dirs)} build dir(s)) ===")
    for b, (c, t) in sorted(agg.items()):
        print(f"  {b:8} {c:5}/{t:<5} {100*c/max(1,t):5.1f}%")
    print(f"  {'TOTAL':8} {tc:5}/{tt:<5} {100*tc/max(1,tt):5.1f}%")
    low = sorted((len(d['cov'])/max(1, len(d['tot'])), fp, len(d['cov']), len(d['tot']))
                 for fp, d in files.items())
    print("=== darkest (burn-down targets) ===")
    for pct, fp, c, t in low[:15]:
        print(f"  {100*pct:5.1f}%  {c:4}/{t:<4} {fp.replace(repo+'/','')}")


if __name__ == '__main__':
    main()
