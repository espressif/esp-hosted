#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
#
# CI gate: every example conforms to the Example Structure & Publishing Spec
# (.meta2/changes/example-structure-spec/proposal.md). No exceptions.
#
# Enforces (per scenario):
#   - scenario root <= 3 path segments below examples/
#   - published role dirs are cp / mcu_host / esp_host (+ cp_<variant>); each has
#     main/idf_component.yml; no committed role README
#   - a scenario has cp AND exactly one host (mcu_host | esp_host)
#   - scenario README: exists, has a title and routing markers for the published
#     roles present; routed content is registry-safe (no raw HTML, no relative links)
#   - linux_802_3_host is GitHub-only: it must NOT carry main/idf_component.yml

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
EXAMPLES = REPO / "examples"

PUBLISHED_HOST_ROLES = {"mcu_host", "esp_host"}
LINUX_ROLE = "linux_802_3_host"
SKID = ("build", "managed_components", "components", "common_components")

_HTML = re.compile(r"</?(?:table|tr|td|th|div|span|img|a|h[1-6]|br|p|ul|ol|li)\b", re.I)
_RELLINK = re.compile(r"\]\(\.\.?/")


def is_published_role(name):
    return name in PUBLISHED_HOST_ROLES or name == "cp" or name.startswith("cp_")


def skipped(path):
    return any(p in SKID for p in path.relative_to(EXAMPLES).parts)


def published_role_projects():
    """Yield (project_dir, role_name, scenario_dir) for each cp/host role."""
    for yml in EXAMPLES.rglob("main/idf_component.yml"):
        proj = yml.parent.parent
        if skipped(proj):
            continue
        yield proj, proj.name, proj.parent


_MARK_LINE = re.compile(
    r"^\s*<!--\s*(common|coprocessor|esp_host)(-ignore)?-(start|stop)\s*-->\s*$")


def routed(text):
    """Return (scopes_seen, routed_text) — only content inside non-ignore
    routing blocks, which is what actually reaches the registry."""
    scopes, out, region = set(), [], None
    for line in text.splitlines():
        m = _MARK_LINE.match(line)
        if m:
            if not m.group(2):  # not an -ignore- marker
                region = m.group(1) if m.group(3) == "start" else None
                if region:
                    scopes.add(region)
            continue
        if region is not None:
            out.append(line)
    return scopes, "\n".join(out)


def _strip_code(text):
    text = re.sub(r"```.*?```", "", text, flags=re.DOTALL)
    return re.sub(r"`[^`]*`", "", text)


def check_readme(scenario, roles, add):
    rel = scenario.relative_to(REPO)
    readme = scenario / "README.md"
    if not readme.is_file():
        add(f"{rel}: missing scenario README.md")
        return
    text = readme.read_text(errors="ignore")
    if not any(l.startswith("# ") for l in text.splitlines()):
        add(f"{rel}/README.md: no title (`# ...`)")
    scopes, routed_text = routed(text)
    if not scopes:
        add(f"{rel}/README.md: no routing markers (not opted into generation)")
    else:
        has_cp = any(r == "cp" or r.startswith("cp_") for r in roles)
        if has_cp and not (scopes & {"common", "coprocessor"}):
            add(f"{rel}/README.md: cp role has no routed content (need common or coprocessor block)")
        if (roles & PUBLISHED_HOST_ROLES) and not (scopes & {"common", "esp_host"}):
            add(f"{rel}/README.md: host role has no routed content (need common or esp_host block)")
    prose = _strip_code(routed_text)  # registry-safe check applies to ROUTED content only
    if _HTML.search(prose):
        add(f"{rel}/README.md: raw HTML inside routed block (won't render on the registry)")
    if _RELLINK.search(prose):
        add(f"{rel}/README.md: relative link inside routed block (dead on the registry)")


def main():
    violations = []
    add = violations.append
    scenarios = {}  # scenario_dir -> set(role_name)

    for proj, role, scenario in published_role_projects():
        relp = proj.relative_to(REPO)
        if not is_published_role(role):
            add(f"{relp}: unknown/non-conforming role dir '{role}' "
                f"(expected cp / mcu_host / esp_host under a scenario)")
            continue
        depth = len(scenario.relative_to(EXAMPLES).parts)
        if depth > 3:
            add(f"{relp}: scenario root '{scenario.relative_to(EXAMPLES)}' is "
                f"{depth} deep (max 3)")
        if (proj / "README.md").is_file():
            add(f"{relp}/README.md: committed role README (must be publish-generated)")
        scenarios.setdefault(scenario, set()).add(role)

    for scenario, roles in sorted(scenarios.items()):
        rel = scenario.relative_to(REPO)
        if not any(r == "cp" or r.startswith("cp_") for r in roles):
            add(f"{rel}: scenario has no cp/ role")
        hosts = roles & PUBLISHED_HOST_ROLES
        if len(hosts) != 1:
            add(f"{rel}: scenario must have exactly one host role, found {sorted(hosts) or 'none'}")
        check_readme(scenario, roles, add)

    # Linux role must NOT be an IDF component (no main/idf_component.yml).
    for linux in EXAMPLES.rglob(LINUX_ROLE):
        if not linux.is_dir() or skipped(linux):
            continue
        for yml in linux.rglob("main/idf_component.yml"):
            add(f"{yml.relative_to(REPO)}: linux_802_3_host must be GitHub-only "
                f"(no main/idf_component.yml)")

    if violations:
        print(f"FAIL: {len(violations)} example-structure violation(s):")
        for v in violations:
            print(f"  {v}")
        print("\nSee .meta2/changes/example-structure-spec/proposal.md")
        return 1
    print("OK: all examples conform to the structure spec.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
