#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Design test for .gitlab-ci.yml + tools/ci/decide_build.sh — NOT a code test.

Guards the pipeline *design* documented in docs/design/pipeline_design.md so an
edit can't silently break routing. It:

  1. asserts structure (jobs exist; PRE vs POST rules; deploy is branch-only;
     regression is off MRs; PRE CP build covers all four transports),
  2. asserts every build job wires the change-scope gate: it calls
     `decide_build.sh <cp|host>` for the correct side and then `eh_setup`,
  3. runs decide_build.sh on representative diffs (via EH_TEST_FILES) and checks
     it BUILDs/SKIPs correctly — including "version header + docs only -> SKIP".

Run:  python3 tools/ci/test_pipeline_rules.py   (exits non-zero on any failure)
"""
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
CI = ROOT / ".gitlab-ci.yml"
DECIDE = ROOT / "tools" / "ci" / "decide_build.sh"

# build job -> which side it must gate on
CP_JOBS = ["build_cp_super", "build_cp_super_ext", "build_cp_specials"]
HOST_JOBS = ["build_host_v55_p4", "build_host_v55_h2",
             "build_host_latest_p4", "build_host_v60_p4", "build_host_v551_p4"]
PRE_JOBS = ["build_cp_super", "build_host_v55_p4", "build_host_v55_h2"]

_checks = []


def check(name, ok, detail=""):
    _checks.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail and not ok else ""))


def is_mr_rule(job):
    return any("merge_request_event" in str(r) for r in job.get("rules", []))


def script_text(job):
    return "\n".join(job.get("script", []))


def decide(side, files):
    """Run decide_build.sh; return 'BUILD' (exit 0) or 'SKIP' (exit 1)."""
    r = subprocess.run(["bash", str(DECIDE), side],
                       env={"EH_TEST_FILES": "\n".join(files), "PATH": "/usr/bin:/bin"},
                       capture_output=True, text=True)
    return "BUILD" if r.returncode == 0 else "SKIP"


GIT_ENV = {"PATH": "/usr/bin:/bin", "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t",
           "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}
BASE_FILES = ["coprocessor/eh_cp_core/src/eh_cp_core.c", "host/features/x.c",
              "common/eh_frame/src/eh_frame.c",
              "common/eh_common/include/eh_common_fw_version.h",
              "docs/changelog.md", "idf_component.yml",
              "examples/wifi/enterprise/mcu_host/main/main.c",
              "examples/wifi/sta/README.md"]


def git_scenarios():
    """Exercise the REAL git-diff paths (not EH_TEST_FILES): a temp repo, real
    commits, decide_build.sh via the branch base (CI_COMMIT_BEFORE_SHA) and the
    MR base (origin/<target>...HEAD)."""
    def g(td, *a, env=None):
        subprocess.run(["git", *a], cwd=td, env=env or GIT_ENV, check=True, capture_output=True)

    def touch(td, rels):
        for f in rels:
            p = Path(td) / f
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text((p.read_text() if p.exists() else "") + "x\n")

    def decide_rc(td, side, env):
        return subprocess.run(["bash", str(DECIDE), side], cwd=td,
                              env={**GIT_ENV, **env}, capture_output=True, text=True).returncode

    scenarios = [  # changed files, side, expect
        (["common/eh_common/include/eh_common_fw_version.h", "docs/changelog.md",
          "idf_component.yml"], "cp",   "SKIP"),
        (["common/eh_common/include/eh_common_fw_version.h", "docs/changelog.md"], "host", "SKIP"),
        (["examples/wifi/sta/README.md"],                    "host", "SKIP"),
        (["coprocessor/eh_cp_core/src/eh_cp_core.c"],        "cp",   "BUILD"),
        (["coprocessor/eh_cp_core/src/eh_cp_core.c"],        "host", "SKIP"),
        (["host/features/x.c"],                              "host", "BUILD"),
        (["common/eh_frame/src/eh_frame.c"],                 "cp",   "BUILD"),
        (["common/eh_common/include/eh_common_fw_version.h",
          "coprocessor/eh_cp_core/src/eh_cp_core.c"],        "cp",   "BUILD"),
    ]

    # --- branch path (CI_COMMIT_BEFORE_SHA..HEAD) ---
    with tempfile.TemporaryDirectory() as td:
        g(td, "init", "-q", "-b", "main", ".")
        touch(td, BASE_FILES); g(td, "add", "-A"); g(td, "commit", "-qm", "base")
        for i, (changes, side, exp) in enumerate(scenarios):
            touch(td, changes); g(td, "commit", "-qam", f"s{i}")
            before = subprocess.run(["git", "rev-parse", "HEAD~1"], cwd=td, env=GIT_ENV,
                                    capture_output=True, text=True).stdout.strip()
            rc = decide_rc(td, side, {"CI_COMMIT_BEFORE_SHA": before})
            got = "BUILD" if rc == 0 else "SKIP"
            check(f"[git/branch] {side} {changes[0].split('/')[0]}.. -> {exp}", got == exp, f"got {got}")

    # --- CRITICAL-2: a pure rename must not hide a source deletion ---
    # `git mv coprocessor/x.c docs/x.c` with rename detection ON emits only the
    # (ignored) destination; the cp side would wrongly SKIP despite losing a .c.
    with tempfile.TemporaryDirectory() as td:
        g(td, "init", "-q", "-b", "main", ".")
        touch(td, ["coprocessor/eh_cp_core/src/eh_cp_core.c", *BASE_FILES])
        g(td, "add", "-A"); g(td, "commit", "-qm", "base")
        before = subprocess.run(["git", "rev-parse", "HEAD"], cwd=td, env=GIT_ENV,
                                capture_output=True, text=True).stdout.strip()
        g(td, "mv", "coprocessor/eh_cp_core/src/eh_cp_core.c", "docs/eh_cp_core.c")
        g(td, "commit", "-qam", "archive src under docs")
        rc = decide_rc(td, "cp", {"CI_COMMIT_BEFORE_SHA": before})
        check("[git/rename] cp coprocessor->docs move -> BUILD (not hidden)", rc == 0,
              f"got {'BUILD' if rc == 0 else 'SKIP'}")

    # --- MR path (origin/<target>...HEAD) via a real bare origin ---
    with tempfile.TemporaryDirectory() as td, tempfile.TemporaryDirectory() as origin:
        g(td, "init", "-q", "-b", "main", ".")
        touch(td, BASE_FILES); g(td, "add", "-A"); g(td, "commit", "-qm", "base")
        g(origin, "init", "-q", "--bare", ".")
        g(td, "remote", "add", "origin", origin); g(td, "push", "-q", "origin", "main")
        g(td, "checkout", "-q", "-b", "feat")
        touch(td, ["host/features/x.c"]); g(td, "commit", "-qam", "feat")
        env = {"CI_MERGE_REQUEST_TARGET_BRANCH_NAME": "main"}
        check("[git/MR] host-only host -> BUILD", decide_rc(td, "host", env) == 0)
        check("[git/MR] host-only cp -> SKIP",   decide_rc(td, "cp", env) == 1)


def main():
    d = yaml.safe_load(CI.read_text())          # dangling anchor would raise
    check("YAML parses (no dangling anchors)", True)
    check("decide_build.sh exists", DECIDE.exists())

    for j in [*CP_JOBS, *HOST_JOBS, "premerge_check", "pre_commit",
              "pipeline_rules_test", "regression", "deploy_master_github"]:
        check(f"job exists: {j}", j in d, f"missing {j}")

    # --- every build job wires the gate for the right side, then eh_setup ---
    for j in CP_JOBS:
        s = script_text(d[j])
        check(f"{j} gates on 'decide_build.sh cp'", "decide_build.sh cp" in s)
        check(f"{j} calls eh_setup after the gate", "eh_setup" in s)
    for j in HOST_JOBS:
        s = script_text(d[j])
        check(f"{j} gates on 'decide_build.sh host'", "decide_build.sh host" in s)
        check(f"{j} calls eh_setup after the gate", "eh_setup" in s)

    # --- PRE vs POST rule assignment ---
    for j in PRE_JOBS:
        check(f"{j} is PRE (MR rule)", is_mr_rule(d[j]))
    for j in [*CP_JOBS, *HOST_JOBS]:
        if j not in PRE_JOBS:
            check(f"{j} is POST (no MR rule)", not is_mr_rule(d[j]))

    # --- deploy branch-only; regression off MRs ---
    dep = str(d["deploy_master_github"].get("rules", []))
    check("deploy: not on MRs", "merge_request_event" not in dep)
    check("deploy: master + unified_release", "master" in dep and "unified_release" in dep)
    check("deploy: not on release/*", "^release" not in dep)  # 'unified_release' contains 'release'
    check("regression: off MRs", not is_mr_rule(d["regression"]))

    # --- PRE CP build covers all four transports ---
    buses = {c.get("BUS", "") for c in d["build_cp_super"]["parallel"]["matrix"]}
    have = {"SDIO" if b == "" else b.split("CONFIG_EH_TRANSPORT_CP_")[1].split("=")[0] for b in buses}
    for t in ("SDIO", "SPI", "SPI_HD", "UART"):
        check(f"PRE covers transport {t}", t in have, f"have={sorted(have)}")

    # --- decision-script behavior (the heart of the change-scope policy) ---
    VER = "common/eh_common/include/eh_common_fw_version.h"
    scenarios = [
        # files, side, expect
        ([VER, "docs/changelog.md", "idf_component.yml"], "cp",   "SKIP"),
        ([VER, "docs/changelog.md", "idf_component.yml"], "host", "SKIP"),
        (["examples/wifi/sta/README.md"],                 "host", "SKIP"),
        (["coprocessor/eh_cp_core/src/eh_cp_core.c"],     "cp",   "BUILD"),
        (["coprocessor/eh_cp_core/src/eh_cp_core.c"],     "host", "SKIP"),
        (["host/features/eh_host_feat_wifi/x.c"],         "host", "BUILD"),
        (["host/features/eh_host_feat_wifi/x.c"],         "cp",   "SKIP"),
        (["common/eh_frame/src/eh_frame.c"],              "cp",   "BUILD"),
        (["common/eh_frame/src/eh_frame.c"],              "host", "BUILD"),
        ([VER, "coprocessor/x.c"],                        "cp",   "BUILD"),
        (["examples/wifi/enterprise/mcu_host/main/main.c"], "host", "BUILD"),
        # CRITICAL-1: an example's CP firmware must build the cp side (build_cp_specials
        # gates ext_coex/cp + ota/coprocessor_ota/cp on `decide_build.sh cp`).
        (["examples/ext_coex/cp/main/main.c"],            "cp",   "BUILD"),
        (["examples/ext_coex/cp/main/main.c"],            "host", "SKIP"),
        (["examples/wifi/sta/cp/main/main.c"],            "cp",   "BUILD"),
        (["examples/wifi/sta/cp/main/main.c"],            "host", "SKIP"),
        (["examples/power_save/cp/shut_down_cp_when_unused/cp/main/main.c"], "cp", "BUILD"),
        # cp variant role dirs (cp_wifi / cp_streaming) are still cp.
        (["examples/wifi/sta/cp_streaming/main/main.c"],  "cp",   "BUILD"),
        # Shared example infra (no role dir) affects BOTH sides.
        (["examples/common_components/esp_hosted_examples_common/sta.c"], "cp",   "BUILD"),
        (["examples/common_components/esp_hosted_examples_common/sta.c"], "host", "BUILD"),
        (["examples/system/api_exerciser/common/eh_api_cmd.c"],          "cp",   "BUILD"),
        # MEDIUM: README*/LICENSE*/CHANGELOG* prefix must not swallow real source.
        (["host/x/READMEgen.c"],                          "host", "BUILD"),
        (["coprocessor/CHANGELOG_parser.c"],              "cp",   "BUILD"),
    ]
    for files, side, exp in scenarios:
        got = decide(side, files)
        check(f"decide[{side}] {files[0]}... -> {exp}", got == exp, f"got {got}")

    # --- failure paths (the two places the gate could lie) ---
    # (a) eh_build must read idf-build-apps' status, not tee's.
    bs = "\n".join(d[".build"].get("before_script", []))
    check(".build eh_build uses PIPESTATUS[0] (not tee's status)", "PIPESTATUS[0]" in bs)
    check(".build defines eh_setup (install deferred past the skip gate)", "eh_setup()" in bs)
    r = subprocess.run(["bash", "-c", "false | tee /dev/null >/dev/null; echo ${PIPESTATUS[0]}"],
                       capture_output=True, text=True)
    check("a failed pipeline is observable via PIPESTATUS[0]", r.stdout.strip() == "1",
          f"got '{r.stdout.strip()}'")

    # (b) decide_build.sh must BUILD (never SKIP) when it can't compute a diff.
    base_env = {"PATH": "/usr/bin:/bin"}
    with tempfile.TemporaryDirectory() as td:
        rr = subprocess.run(["bash", str(DECIDE), "cp"], cwd=td, env=base_env,
                            capture_output=True, text=True)
        check("decide: no diff base -> BUILD (conservative)", rr.returncode == 0, f"rc={rr.returncode}")
        subprocess.run(["git", "init", "-q", "-b", "main", "."], cwd=td, env=base_env)
        rr2 = subprocess.run(["bash", str(DECIDE), "cp"], cwd=td,
                            env={**base_env, "CI_COMMIT_BEFORE_SHA": "deadbeef" * 5},
                            capture_output=True, text=True)
        check("decide: unresolvable base -> BUILD (conservative)", rr2.returncode == 0, f"rc={rr2.returncode}")

    # --- real git-diff paths (branch + MR), not just EH_TEST_FILES ---
    git_scenarios()

    n_fail = _checks.count(False)
    print(f"\n{len(_checks)} checks, {n_fail} failed")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
