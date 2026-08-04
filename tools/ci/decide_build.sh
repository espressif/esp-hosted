#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
#
# Decide whether the <side> (cp|host) build is needed for the current change set.
# The single source of truth for pipeline change-scoping (see
# docs/design/pipeline_design.md). GitLab rules:changes can't express "ignore
# this one generated header", so the build jobs call this instead and self-skip.
#
#   exit 0  -> BUILD this side
#   exit 1  -> SKIP  this side (nothing it can affect changed)
# Reasoning is printed to stderr so it lands in the job log.
#
# Change set:
#   - $EH_TEST_FILES (newline-separated) if set  -> used verbatim (unit tests)
#   - else an MR diff vs the target branch
#   - else the branch push range ($CI_COMMIT_BEFORE_SHA..HEAD)
#   - else: cannot tell -> BUILD (conservative; never under-build)
#
# Files that must NOT, on their own, trigger a build (docs + generated version
# headers + the top-level manifest): a version bump or a README edit is not a
# source change. Everything else is classified CP / host / shared.
set -u

side="${1:?usage: decide_build.sh cp|host}"

if [ -n "${EH_TEST_FILES:-}" ]; then
    files="$EH_TEST_FILES"
else
    # Pick the diff base. CRUCIAL: any inability to compute a real diff must fall
    # through to BUILD (never SKIP) — an empty/failed diff is indistinguishable
    # from "nothing relevant changed", so we must not treat it as a skip signal.
    if [ -n "${CI_MERGE_REQUEST_TARGET_BRANCH_NAME:-}" ]; then
        git fetch --no-tags -q origin "$CI_MERGE_REQUEST_TARGET_BRANCH_NAME" 2>/dev/null || true
        base="origin/${CI_MERGE_REQUEST_TARGET_BRANCH_NAME}"; spec="${base}...HEAD"
    elif [ -n "${CI_COMMIT_BEFORE_SHA:-}" ] && \
         [ "$CI_COMMIT_BEFORE_SHA" != "0000000000000000000000000000000000000000" ]; then
        base="$CI_COMMIT_BEFORE_SHA"; spec="${base} HEAD"
    else
        echo "[$side] no diff base available -> BUILD (conservative)" >&2; exit 0
    fi
    if ! git rev-parse --verify -q "${base}^{commit}" >/dev/null 2>&1; then
        echo "[$side] base '$base' does not resolve -> BUILD (conservative)" >&2; exit 0
    fi
    # --no-renames: with rename detection ON, `git mv coprocessor/x.c docs/x.c`
    # emits ONLY the (ignored) destination, hiding that a .c left the CP tree.
    # Force add+delete so the source path always appears and is classified.
    # shellcheck disable=SC2086  # $spec is 'A...HEAD' (1 token) or 'A HEAD' (2) on purpose
    if ! files="$(git diff --name-only --no-renames $spec 2>/dev/null)"; then
        echo "[$side] git diff ($spec) failed -> BUILD (conservative)" >&2; exit 0
    fi
fi

# Ignore list: docs, and the two generated fw-version headers + root manifest.
# Basenames are END-anchored ([.]|$) so a real source file whose name merely
# STARTS with README/LICENSE/CHANGELOG (e.g. READMEgen.c) is NOT ignored.
IGNORE='(^|/)README([.]|$)|[.]md$|^docs/|^LICENSES?/|(^|/)LICENSE([.]|$)|(^|/)CHANGELOG([.]|$)|^idf_component[.]yml$'
IGNORE="$IGNORE"'|(^|/)eh_common_fw_version[.]h$|(^|/)esp_hosted_host_fw_ver[.]h$'

real="$(printf '%s\n' "$files" | grep -vE "$IGNORE" | sed '/^[[:space:]]*$/d' || true)"

echo "[$side] change set (excluding version headers + docs):" >&2
printf '%s\n' "${real:-  <none>}" | sed 's/^/    /' >&2

if [ -z "$real" ]; then
    echo "[$side] only version/docs changed -> SKIP" >&2
    exit 1
fi

# Shared trees affect every target.
SHARED='^(common|port|tools)/|^CMakeLists[.]txt$|^Kconfig|^[.]gitlab-ci[.]yml$|^[.]gitmodules$'
m() { printf '%s\n' "$real" | grep -qE "$1"; }

# examples/ is per-role: examples/<feat>/<scenario>/<role>/ where <role> is a CP
# role (cp, cp_wifi, cp_streaming), a host role (mcu_host, esp_host,
# linux_802_3_host), or NONE — files under examples/ outside any role dir are
# shared example infra (examples/common_components/, .../<ex>/common/) and affect
# BOTH sides. So an examples/*/cp change is a CP build (this is why build_cp_specials,
# which builds examples/*/cp, gates correctly on `decide_build.sh cp`).
EX_CP='^examples/.*/cp(_[a-z0-9]+)?/'
EX_HOST='^examples/.*/(mcu_host|esp_host|linux_802_3_host)/'
EX_ANY_ROLE='/(cp(_[a-z0-9]+)?|mcu_host|esp_host|linux_802_3_host)/'
# true iff some examples/ file is NOT under any recognised role dir (shared infra).
ex_shared() { printf '%s\n' "$real" | grep -E '^examples/' | grep -qvE "$EX_ANY_ROLE"; }

if m "$SHARED"; then echo "[$side] shared code changed -> BUILD" >&2; exit 0; fi
if ex_shared; then echo "[$side] shared example infra changed -> BUILD" >&2; exit 0; fi
case "$side" in
    cp)   m '^coprocessor/' && { echo "[cp] coprocessor/ changed -> BUILD"       >&2; exit 0; }
          m "$EX_CP"        && { echo "[cp] examples/*/cp changed -> BUILD"       >&2; exit 0; } ;;
    host) m '^host/'        && { echo "[host] host/ changed -> BUILD"             >&2; exit 0; }
          m "$EX_HOST"      && { echo "[host] examples host-role changed -> BUILD" >&2; exit 0; } ;;
    *)    echo "unknown side: $side" >&2; exit 0 ;;   # unknown -> build (safe)
esac

echo "[$side] no ${side}-relevant source change -> SKIP" >&2
exit 1
