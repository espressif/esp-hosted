#!/usr/bin/env python3
"""Strip named local-component dependency stanzas from an idf_component.yml.

Registry publish copies each shared repo-local component into the example's
components/ (IDF auto-discovers it there). But the registry strips
override_path server-side, leaving a bare `name: {version: '*'}` that the
component manager version-solves against the registry — where these repo-local
components do not exist — so resolution fails ("no versions match *") before
the local components/ copy is ever considered. Removing the declared
dependency lets IDF satisfy the CMake requirement from components/ instead.

Publish/CI-only: the committed manifests keep override_path for local git
builds. Robust to indentation; removes the key line and its indented children.
"""
import re
import sys

_KEY_RE = re.compile(r"^(\s+)([A-Za-z0-9_./-]+):\s*$")


def strip_deps(path, deps):
    with open(path) as f:
        lines = f.read().splitlines(keepends=True)
    out, i, n, removed = [], 0, len(lines), []
    while i < n:
        m = _KEY_RE.match(lines[i])
        if m and m.group(2) in deps:
            indent = len(m.group(1))
            removed.append(m.group(2))
            i += 1
            while i < n:
                stripped = lines[i].strip()
                cur_indent = len(lines[i]) - len(lines[i].lstrip())
                if not stripped or cur_indent <= indent:
                    break
                i += 1
            continue
        out.append(lines[i])
        i += 1
    with open(path, "w") as f:
        f.write("".join(out))
    return removed


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit("usage: strip_bundled_deps.py <idf_component.yml> <dep> [dep ...]")
    got = strip_deps(sys.argv[1], set(sys.argv[2:]))
    print(f"{sys.argv[1]}: stripped {got or '(none present)'}")
