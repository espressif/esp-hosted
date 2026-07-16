#!/usr/bin/env bash
# build_all_host_examples.sh — Build MCU host firmware examples
# (under examples/**/mcu/{host,esp_host}/) across host SoCs.
#
# Pairs with build_all_examples.sh, which is CP-only.  Host and CP
# build worlds differ:
#   - role:    CONFIG_ESP_HOSTED_HOST=y vs CONFIG_ESP_HOSTED_CP=y
#   - chips:   esp32p4 / esp32h2 vs the CP coprocessor SoCs
#   - flags:   the host *consumes* a transport (it picks a bus), the
#              CP example *advertises* one (it picks an interface).
# So a single driver would have been a tangle; mirroring keeps each
# script narrow.
#
# Usage:
#   ./tests/tools/build_all_host_examples.sh --build-targets TARGETS [OPTIONS]
#
#   --build-targets TARGETS   Comma-separated host targets (MANDATORY).
#                             Use "all" for every host target in
#                             ALL_HOST_TARGETS below (esp32p4 only by
#                             default — h2 needs per-example SPI
#                             overlays to drop the SDIO requirement).
#   --build-examples EXAMPLES Comma-separated path-substring filters,
#                             or "all" (default: "all").
#   --build-incremental       Reuse existing build/sdkconfig/managed_components.
#                             Default is a full wipe before each (project,target).
#   --jobs N                  Parallel jobs (default: 4).
#   --filter PAT              Additional grep -E filter on example path.
#   --build-dir DIR           Output dir (default: build_logs/host).
#   --help                    Show this help.
#
# Skip rules below capture KNOWN-incompatible (example, target) pairs.
# Don't over-exclude — let IDF surface fresh failures explicitly.

set -uo pipefail

# ══════════════════════════════════════════════════════════════════════════════
# Compatibility matrix
# ══════════════════════════════════════════════════════════════════════════════

# Host SoCs this script knows about.  Mirrors ALL_HOST_TARGETS in
# tests/tools/build_examples.py.  H2 is included so explicit
# `--build-targets esp32h2` works (with the heads-up below); "all"
# resolves to ALL_HOST_TARGETS (P4 today; H2 once SPI overlays exist).
ALL_HOST_TARGETS=(esp32p4)
KNOWN_HOST_TARGETS=(esp32p4 esp32h2)

# ══════════════════════════════════════════════════════════════════════════════
# Defaults
# ══════════════════════════════════════════════════════════════════════════════
JOBS=4
FILTER=""
OUT_DIR="build_logs/host"
TARGETS=""
EXAMPLES="all"
FULLCLEAN=1

# ══════════════════════════════════════════════════════════════════════════════
# Argument parsing (long options only)
# ══════════════════════════════════════════════════════════════════════════════
usage() {
    awk 'NR>1 && /^$/{exit} NR>1{sub(/^# ?/,""); print}' "$0" >&2
    exit "${1:-0}"
}

_optval() { echo "${1#*=}"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-targets=*)    TARGETS="$(_optval "$1")"; shift ;;
        --build-targets)      TARGETS="$2"; shift 2 ;;
        --build-examples=*)   EXAMPLES="$(_optval "$1")"; shift ;;
        --build-examples)     EXAMPLES="$2"; shift 2 ;;
        --build-incremental)  FULLCLEAN=0; shift ;;
        --jobs=*)             JOBS="$(_optval "$1")"; shift ;;
        --jobs)               JOBS="$2"; shift 2 ;;
        --filter=*)           FILTER="$(_optval "$1")"; shift ;;
        --filter)             FILTER="$2"; shift 2 ;;
        --build-dir=*)        OUT_DIR="$(_optval "$1")"; shift ;;
        --build-dir)          OUT_DIR="$2"; shift 2 ;;
        --help)               usage 0 ;;
        *)  echo "ERROR: Unknown option: $1" >&2; usage 2 ;;
    esac
done

if [[ -z "$TARGETS" ]]; then
    echo "ERROR: --build-targets is mandatory" >&2
    echo "       Use --build-targets all (=${ALL_HOST_TARGETS[*]})" >&2
    echo "       or pick from: ${KNOWN_HOST_TARGETS[*]}" >&2
    exit 2
fi

# Resolve "all" to ALL_HOST_TARGETS
if [[ "$TARGETS" == "all" ]]; then
    IFS=',' read -r -a TARGET_LIST <<< "$(IFS=,; echo "${ALL_HOST_TARGETS[*]}")"
else
    IFS=',' read -r -a TARGET_LIST <<< "$TARGETS"
fi
TARGETS_CSV="$(IFS=,; echo "${TARGET_LIST[*]}")"

# Warn on unknown targets
for tgt in "${TARGET_LIST[@]}"; do
    found=0
    for known in "${KNOWN_HOST_TARGETS[@]}"; do
        [[ "$tgt" == "$known" ]] && { found=1; break; }
    done
    [[ $found -eq 0 ]] && echo "WARNING: '$tgt' not in KNOWN_HOST_TARGETS — proceeding anyway" >&2
done

# ══════════════════════════════════════════════════════════════════════════════
# Locate repo
# ══════════════════════════════════════════════════════════════════════════════
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
# Script lives at tests/tools/, repo root is three levels up.
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
EXAMPLES_ROOT="$REPO_ROOT/examples"

case "$OUT_DIR" in /*) ;; *) OUT_DIR="$REPO_ROOT/$OUT_DIR" ;; esac
mkdir -p "$OUT_DIR"
SUMMARY="$OUT_DIR/summary.txt"
: > "$SUMMARY"
SKIP_LOG="$OUT_DIR/skipped.txt"
: > "$SKIP_LOG"

# ══════════════════════════════════════════════════════════════════════════════
# Source IDF
# ══════════════════════════════════════════════════════════════════════════════
if ! command -v idf.py >/dev/null 2>&1; then
    [[ -f "$HOME/esp-idf/export.sh" ]] && source "$HOME/esp-idf/export.sh" >/dev/null 2>&1
fi
if ! command -v idf.py >/dev/null 2>&1; then
    echo "ERROR: idf.py not found. Source ESP-IDF env first." >&2; exit 2
fi

# ══════════════════════════════════════════════════════════════════════════════
# Skip rules — KNOWN-incompatible (example path-substring, target chips).
# Empty rules = "doesn't apply on any chip".  Add lines as new
# incompatibilities surface; don't preemptively exclude.
# ══════════════════════════════════════════════════════════════════════════════
SKIP_RULES_FILE="$OUT_DIR/.skip_rules"
cat > "$SKIP_RULES_FILE" << 'RULES'
# pattern                                 chips...
# ---- H2 host: no SDIO master, no Wi-Fi, no BT classic ----
# Until per-example SPI overlays exist, h2 only builds the bus-agnostic
# system/utility examples.  Add lines under "/" patterns to opt back in.
mcu_host                                  esp32h2
esp_host                              esp32h2
RULES

# ══════════════════════════════════════════════════════════════════════════════
# Discover host examples
# ══════════════════════════════════════════════════════════════════════════════
mapfile -t ALL_PROJECTS < <(
    find "$EXAMPLES_ROOT" \
         \( -path "*/mcu_host" -o -path "*/esp_host" \) \
         -type d \
         -not -path "*/build/*" \
         -not -path "*/managed_components/*" \
         2>/dev/null \
      | while read -r d; do
            [[ -f "$d/CMakeLists.txt" \
            && -f "$d/sdkconfig.defaults" \
            && -f "$d/main/CMakeLists.txt" ]] && echo "$d"
        done | sort -u
)

# Apply --filter and --build-examples
PROJECTS=()
for p in "${ALL_PROJECTS[@]}"; do
    rel="${p#"$REPO_ROOT/"}"
    if [[ -n "$FILTER" ]] && ! [[ "$rel" =~ $FILTER ]]; then continue; fi
    if [[ "$EXAMPLES" != "all" ]]; then
        matched=0
        IFS=',' read -r -a ex_list <<< "$EXAMPLES"
        for ex in "${ex_list[@]}"; do
            [[ "$rel" == *"$ex"* ]] && { matched=1; break; }
        done
        [[ $matched -eq 0 ]] && continue
    fi
    PROJECTS+=("$p")
done

if [[ ${#PROJECTS[@]} -eq 0 ]]; then
    echo "No host examples matched." >&2; exit 0
fi

# ══════════════════════════════════════════════════════════════════════════════
# Print plan
# ══════════════════════════════════════════════════════════════════════════════
echo "==> Discovered ${#PROJECTS[@]} host example(s)"
echo "==> Targets: ${TARGET_LIST[*]}"
echo "==> Jobs: $JOBS, Fullclean: $FULLCLEAN"
echo "==> Logs: $OUT_DIR"
echo

# ══════════════════════════════════════════════════════════════════════════════
# Build worker
# ══════════════════════════════════════════════════════════════════════════════
build_one() {
    local proj="$1"
    local rel="${proj#"$REPO_ROOT/"}"
    local base_id; base_id="$(echo "$rel" | sed 's|^examples/||; s|/|__|g')"

    local TARGET_LIST_LOCAL=()
    IFS=',' read -r -a TARGET_LIST_LOCAL <<< "$TARGETS_CSV"

    for tgt in "${TARGET_LIST_LOCAL[@]}"; do
        local id="${base_id}__${tgt}"
        local log="$OUT_DIR/$id.log"
        local start end dur status warnings
        local skip_reason=""

        # ── Rule-based skip ───────────────────────────────────────
        if [[ -f "$SKIP_RULES_FILE" ]]; then
            while IFS=' ' read -r pattern chips; do
                [[ "$pattern" == "#"* || -z "$pattern" ]] && continue
                if [[ "$rel" == *"$pattern"* && " $chips " == *" $tgt "* ]]; then
                    skip_reason="$pattern not supported on $tgt"
                    break
                fi
            done < "$SKIP_RULES_FILE"
        fi

        if [[ -n "$skip_reason" ]]; then
            printf '%-8s %-12s %4ds  W=%-3s  %s\n' "SKIP" "$tgt" "0" "-" "$rel" >>"$SUMMARY"
            echo "$tgt  $rel  ($skip_reason)" >>"$SKIP_LOG"
            echo "[SKIP] $rel [$tgt] ($skip_reason)"
            continue
        fi

        # ── Build ─────────────────────────────────────────────────
        start=$(date +%s)

        {
            echo "=== build_one (host): $rel ==="
            echo "target=$tgt"
            echo

            # Wipe everything that can leak state across targets.
            # idf.py fullclean leaves sdkconfig + dependencies.lock behind;
            # both bind the project to a specific chip / managed-component
            # set and poison the next iteration.  --build-incremental opts
            # out and reuses whatever is on disk.
            if [[ "$FULLCLEAN" -eq 1 ]]; then
                rm -rf "$proj/build" \
                       "$proj/sdkconfig" \
                       "$proj/sdkconfig.old" \
                       "$proj/managed_components" \
                       "$proj/dependencies.lock" \
                       "$proj/main/dependencies.lock"
            fi

            ( cd "$proj" && IDF_TARGET="$tgt" idf.py set-target "$tgt" \
                && IDF_TARGET="$tgt" idf.py build )
        } >"$log" 2>&1

        status=$?
        end=$(date +%s); dur=$((end - start))
        warnings=$(grep -c "warning:" "$log" 2>/dev/null || echo 0)

        if [[ $status -eq 0 ]] && grep -q "Project build complete" "$log"; then
            printf '%-8s %-12s %4ds  W=%-3s  %s\n' "OK" "$tgt" "$dur" "$warnings" "$rel" >>"$SUMMARY"
            echo "[OK]   $rel [$tgt] (${dur}s, $warnings warnings)"
        else
            printf '%-8s %-12s %4ds  W=%-3s  %s\n' "FAIL" "$tgt" "$dur" "$warnings" "$rel" >>"$SUMMARY"
            echo "[FAIL] $rel [$tgt] (${dur}s, see $log)"
        fi
    done
}
export -f build_one
export REPO_ROOT OUT_DIR SUMMARY SKIP_LOG TARGETS_CSV FULLCLEAN SKIP_RULES_FILE

# ══════════════════════════════════════════════════════════════════════════════
# Parallel execution
# ══════════════════════════════════════════════════════════════════════════════
printf '%s\n' "${PROJECTS[@]}" \
    | xargs -I{} -P "$JOBS" bash -c 'build_one "$@"' _ {}

# ══════════════════════════════════════════════════════════════════════════════
# Summary
# ══════════════════════════════════════════════════════════════════════════════
echo
echo "============================================================"
echo "Host build summary (logs in $OUT_DIR)"
echo "============================================================"
printf '%-8s %-12s %5s   %-5s  %s\n' STATUS TARGET DUR WARN PATH
echo "------------------------------------------------------------"
sort "$SUMMARY"
echo "------------------------------------------------------------"
# grep -c always prints a number; swallow the exit code on no match.
ok=$(grep -c '^OK ' "$SUMMARY" || true)
fail=$(grep -c '^FAIL ' "$SUMMARY" || true)
skip=$(grep -c '^SKIP ' "$SUMMARY" || true)
echo "Totals: OK=$ok  FAIL=$fail  SKIP=$skip"
[[ $fail -gt 0 ]] && exit 1 || exit 0
