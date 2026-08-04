#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Generate registry example READMEs from the top-level example README's markers.

Publish-time only (run in the upload_component CI step); generated files are NOT
committed. The top-level README is the single source of truth and stays eh.py /
git-only. See docs/design/registry-example-readme-generation.md.

Marker vocabulary (HTML comments — invisible in normal markdown render).
Consistent shape: `<!-- <scope>-<edge> -->` and `<!-- <scope>-ignore-<edge> -->`,
scope in {common, coprocessor, esp_host}, edge in {start, stop}.

  ROUTING (include content into an output):
    <!-- common-start --> ...       <!-- common-stop -->        -> both cp + host
    <!-- coprocessor-start --> ...  <!-- coprocessor-stop -->   -> cp/README.md
    <!-- esp_host-start --> ...     <!-- esp_host-stop -->      -> <host>/README.md

  EXCLUSION (drop a fragment from specific output(s), even inside a routed block):
    <!-- common-ignore-start --> ...      <!-- common-ignore-stop -->      drop from BOTH (git-only)
    <!-- coprocessor-ignore-start --> ... <!-- coprocessor-ignore-stop --> drop from cp only
    <!-- esp_host-ignore-start --> ...    <!-- esp_host-ignore-stop -->    drop from host only

  cp/README.md      = common + coprocessor   (minus common-ignore / coprocessor-ignore)
  <host>/README.md  = common + esp_host      (minus common-ignore / esp_host-ignore)
                      <host> = mcu_host/ if present else esp_host/
Content outside every routing block (e.g. the Linux flow) is ignored.
`eh.py` -> `idf.py` in the generated files.
"""
import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
EXAMPLES = REPO / "examples"

_MARK = re.compile(
    r"^\s*<!--\s*(common|coprocessor|esp_host)(-ignore)?-(start|stop)\s*-->\s*$"
)
_PROV = "<!-- generated from ../README.md — edit that file, not this one -->"


def _collect(text):
    """Return (rows, has_cp, has_host); rows = [(region, ignore_frozenset, line)]
    for every content line inside a routing block."""
    rows = []
    region = None
    ignore = set()
    block = 0                # bumps on every routing-block start
    has_cp = has_host = False
    for line in text.splitlines():
        m = _MARK.match(line)
        if m:
            scope, is_ignore, edge = m.group(1), m.group(2), m.group(3)
            if is_ignore:
                ignore.add(scope) if edge == "start" else ignore.discard(scope)
            else:
                region = scope if edge == "start" else None
                if edge == "start":
                    block += 1
                    has_cp = has_cp or scope == "coprocessor"
                    has_host = has_host or scope == "esp_host"
            continue
        if region is not None:
            rows.append((region, block, frozenset(ignore), line))
    return rows, has_cp, has_host


def _body(rows, keep_regions, drop_scopes):
    """Concatenate kept blocks in document order, blank-separating distinct blocks."""
    out, prev_block = [], None
    for region, block, ig, line in rows:
        if region not in keep_regions or (ig & drop_scopes):
            continue
        if prev_block is not None and block != prev_block and out and out[-1].strip():
            out.append("")
        out.append(line)
        prev_block = block
    return out


def slice_readme(text):
    """Return (cp_body, host_body, has_cp, has_host)."""
    rows, has_cp, has_host = _collect(text)
    cp = _body(rows, {"common", "coprocessor"}, {"common", "coprocessor"})
    host = _body(rows, {"common", "esp_host"}, {"common", "esp_host"})
    return cp, host, has_cp, has_host


def _trim(lines):
    lines = list(lines)
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    return lines


def title_of(text):
    for line in text.splitlines():
        if line.startswith("# "):
            t = re.sub(r"\(`[^`]*`\)", "", line[2:]).replace("`", "").strip()
            return t or "Example"
    return "Example"


def render(title, suffix, body):
    out = [f"# {title} — {suffix}", ""] + _trim(body) + ["", _PROV, ""]
    return re.sub(r"\beh\.py\b", "idf.py", "\n".join(out))


def _strip_code(text):
    text = re.sub(r"```.*?```", "", text, flags=re.DOTALL)
    return re.sub(r"`[^`]*`", "", text)


def lint(text):
    prose = _strip_code(text)
    warns = []
    if re.search(r"</?(?:table|tr|td|th|div|span|img|a|h[1-6]|br|p|ul|ol|li)\b", prose, re.I):
        warns.append("raw HTML (won't render on the registry)")
    if re.search(r"\]\(\.\.?/", prose):
        warns.append("relative link (dead on the registry)")
    if re.search(r"\beh\.py\b", text):
        warns.append("residual eh.py")
    return warns


def host_dir(example):
    for name in ("mcu_host", "esp_host"):
        if (example / name).is_dir():
            return name
    return None


def process(example, dry_run, out_dir=None):
    text = (example / "README.md").read_text()
    if not any(_MARK.match(l) for l in text.splitlines()):
        return []  # not opted in
    cp_body, host_body, has_cp, has_host = slice_readme(text)
    title = title_of(text)
    out = []
    if has_cp:
        # cp/ and any cp_<variant>/ (e.g. cp_wifi) all publish the coprocessor block.
        for role in sorted(example.iterdir()):
            if role.is_dir() and (role.name == "cp" or role.name.startswith("cp_")):
                out.append((role / "README.md", render(title, "Coprocessor", cp_body)))
    if has_host:
        hd = host_dir(example)
        if hd:
            out.append((example / hd / "README.md", render(title, "Host", host_body)))
    results = []
    for path, content in out:
        # --out mirrors the in-repo path under a review dir; publish writes in place.
        dest = (out_dir / path.relative_to(REPO)) if out_dir else path
        if dry_run:
            print(f"\n===== {path.relative_to(REPO)} =====\n{content}")
        else:
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_text(content)
        results.append((dest, lint(content)))
    return results


def top_readmes():
    for readme in sorted(EXAMPLES.rglob("README.md")):
        d = readme.parent
        parts = d.relative_to(EXAMPLES).parts
        if "managed_components" in parts or "components" in parts:
            continue
        if (d / "cp").is_dir() or (d / "mcu_host").is_dir() or (d / "esp_host").is_dir():
            yield d


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-run", action="store_true", help="print, don't write")
    ap.add_argument("--out", help="write generated files under this review dir "
                    "(mirrors repo paths) instead of in place")
    ap.add_argument("--example", help="one example dir (relative to repo root)")
    args = ap.parse_args()

    out_dir = Path(args.out).resolve() if args.out else None
    targets = [REPO / args.example] if args.example \
        else list(top_readmes())

    total = warned = 0
    for ex in targets:
        for path, warns in process(ex, args.dry_run, out_dir):
            total += 1
            rel = path.relative_to(out_dir if out_dir else REPO)
            if warns:
                warned += 1
                print(f"WARN {rel}: {'; '.join(warns)}", file=sys.stderr)
            else:
                print(f"ok   {rel}", file=sys.stderr)
    print(f"\n{total} file(s) generated, {warned} with warnings", file=sys.stderr)
    return 1 if (warned and not args.dry_run) else 0


if __name__ == "__main__":
    sys.exit(main())
