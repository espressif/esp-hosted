#!/usr/bin/env bash
# check_override_path_depth.sh — fail if any examples/ idf_component.yml
# carries an `override_path` whose `..` count doesn't match the file's
# distance from the repo root.
#
# Background: every `examples/<feature>/.../main/idf_component.yml` that
# pulls in this repo as the `esp_hosted` IDF component does so via
# `override_path: "../../..."`. The number of `..` segments must equal
# the number of dirs between the .yml and the repo root.  When it
# doesn't, IDF resolves the dependency to the wrong directory (or
# silently picks up the registry version) and the example becomes a
# tarball — configures or builds against the wrong sources.
#
# CI gate.  Returns 0 if every override_path matches; 1 with a per-file
# report otherwise.

set -euo pipefail

repo="$(cd "$(dirname "$0")/../.." && pwd)"
cd "${repo}"

bad=0
# Skip generated / pulled directories: managed_components/ (IDF
# component-manager downloads) and build/ (CMake build trees) are
# never tracked and carry their own (unrelated) override_path values
# that don't apply to our gating.
while IFS= read -r f; do
    case "$f" in
        */managed_components/*) continue ;;
        */build/*)              continue ;;
    esac
    # Number of dirs between idf_component.yml and repo root = number
    # of components in dirname(f) (with `tr / '\n' | wc -l`).
    dirs=$(dirname "$f" | tr / '\n' | wc -l)
    op=$(grep override_path "$f" | head -1 \
        | sed -e 's/.*override_path: *//' -e 's/"//g' -e "s/'//g" -e 's/[[:space:]]*$//')
    [ -z "$op" ] && continue
    # Number of ".." segments (slash-separated).
    segs=$(echo "$op" | awk -F/ '{print NF}')
    if [ "$segs" != "$dirs" ]; then
        echo "FAIL  $f"
        echo "      override_path = $op  (has $segs '..' segments)"
        echo "      file is $dirs dirs below repo root → expected $dirs '..' segments"
        bad=$((bad + 1))
    fi
done < <(grep -rl override_path examples --include="idf_component.yml" 2>/dev/null)

if [ "$bad" -gt 0 ]; then
    echo ""
    echo "${bad} idf_component.yml file(s) have a wrong override_path depth."
    echo "Each '..' segment in override_path must match the file's distance"
    echo "to the repo root (count the dirs between the .yml and esp_hosted/)."
    exit 1
fi

echo "All override_path depths consistent."
