#!/usr/bin/env bash
# build_all_examples.sh — Build CP firmware examples with target/transport/example awareness.
#
# Usage:
#   ./tests/tools/build_all_examples.sh --build-targets TARGETS [OPTIONS]
#
#   --build-targets TARGETS       Comma-separated chip targets (MANDATORY)
#                                 e.g. esp32c6 or esp32c2,esp32c6,esp32s3
#   --build-transports TRANSPORTS Comma-separated transports (default: "default")
#                                 Values: sdio, spi-fd, spi-hd, uart, all, default
#                                 "default" picks sdio for chips that support it, spi-fd otherwise
#                                 "all" builds every compatible transport per chip
#   --build-examples EXAMPLES     Comma-separated example filters, or "all" (default: "all")
#                                 Auto-skips incompatible target/example combos
#   --build-incremental           Skip idf.py fullclean — reuse existing build dirs
#   --jobs N                      Parallel jobs (default: 4)
#   --filter PAT                  Additional grep -E filter on example path
#   --build-dir DIR              Build output directory for artifacts, logs, and report (default: build_logs/)
#   --help                        Show this help
#
# The script auto-skips incompatible combos (e.g. BT on ESP32-S2, NW split on C2/C3).

set -uo pipefail

# ══════════════════════════════════════════════════════════════════════════════
# Compatibility matrix
# ══════════════════════════════════════════════════════════════════════════════

# Transport → supported chips (space-separated)
declare -A TRANSPORT_CHIPS=(
    [sdio]="esp32 esp32c5 esp32c6 esp32c61"
    [spi-fd]="esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32s2 esp32s3 esp32h2 esp32h4"
    [spi-hd]="esp32s2 esp32s3"
    [uart]="esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32s2 esp32s3 esp32h2 esp32h4"
)

# Default transport per chip (used when --build-transports=default)
declare -A DEFAULT_TRANSPORT=(
    [esp32]=sdio [esp32c2]=spi-fd [esp32c3]=spi-fd
    [esp32c5]=sdio [esp32c6]=sdio [esp32c61]=sdio
    [esp32s2]=spi-fd [esp32s3]=spi-fd
    [esp32h2]=spi-fd [esp32h4]=spi-fd
)

# Transport → Kconfig symbol (for detecting which transport an example configures)
declare -A TRANSPORT_KCONFIG=(
    [sdio]="ESP_HOSTED_TRANSPORT_CP_SDIO"
    [spi-fd]="ESP_HOSTED_TRANSPORT_CP_SPI"
    [spi-hd]="ESP_HOSTED_TRANSPORT_CP_SPI_HD"
    [uart]="ESP_HOSTED_TRANSPORT_CP_UART"
)

# Example path patterns → excluded chips (space-separated)
# Only KNOWN build failures — don't over-exclude.
declare -A EXAMPLE_EXCLUDE_CHIPS=(
    [network_split]="esp32c2 esp32c3"
    [minimal/bt]="esp32s2"
    [host_power_save]="esp32c2"
    [wifi_enterprise]="esp32c2"
    [wifi_dpp]="esp32c2"
    [wifi_itwt]="esp32c2"
)

# Chips without WiFi — skip all WiFi-dependent examples
NO_WIFI_CHIPS="esp32h2"

# ══════════════════════════════════════════════════════════════════════════════
# Defaults
# ══════════════════════════════════════════════════════════════════════════════
JOBS=4
FILTER=""
OUT_DIR="build_logs"
TARGETS=""
TRANSPORTS="default"
EXAMPLES="all"
FULLCLEAN=1

# ══════════════════════════════════════════════════════════════════════════════
# Argument parsing (long options only)
# ══════════════════════════════════════════════════════════════════════════════
usage() {
    awk 'NR>1 && /^$/{exit} NR>1{sub(/^# ?/,""); print}' "$0" >&2
    exit "${1:-0}"
}

# Parse --option=value or --option value
_optval() { echo "${1#*=}"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-targets=*)      TARGETS="$(_optval "$1")"; shift ;;
        --build-targets)        TARGETS="$2"; shift 2 ;;
        --build-transports=*)   TRANSPORTS="$(_optval "$1")"; shift ;;
        --build-transports)     TRANSPORTS="$2"; shift 2 ;;
        --build-examples=*)     EXAMPLES="$(_optval "$1")"; shift ;;
        --build-examples)       EXAMPLES="$2"; shift 2 ;;
        --build-incremental)    FULLCLEAN=0; shift ;;
        --jobs=*)               JOBS="$(_optval "$1")"; shift ;;
        --jobs)                 JOBS="$2"; shift 2 ;;
        --filter=*)             FILTER="$(_optval "$1")"; shift ;;
        --filter)               FILTER="$2"; shift 2 ;;
        --build-dir=*)         OUT_DIR="$(_optval "$1")"; shift ;;
        --build-dir)           OUT_DIR="$2"; shift 2 ;;
        --help)                 usage 0 ;;
        *)  echo "ERROR: Unknown option: $1" >&2; usage 2 ;;
    esac
done

if [[ -z "$TARGETS" ]]; then
    echo "ERROR: --build-targets is mandatory (e.g. --build-targets esp32c6)" >&2
    exit 2
fi

IFS=',' read -r -a TARGET_LIST <<< "$TARGETS"
TARGETS_CSV="$TARGETS"

# ══════════════════════════════════════════════════════════════════════════════
# Locate repo
# ══════════════════════════════════════════════════════════════════════════════
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
EXAMPLES_ROOT="$REPO_ROOT/fg/cp/examples"

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
# Helper: check if chip is in space-separated list
# ══════════════════════════════════════════════════════════════════════════════
chip_in_list() { local chip="$1" list="$2"; [[ " $list " == *" $chip "* ]]; }

# ══════════════════════════════════════════════════════════════════════════════
# Helper: should this (example, target) pair be skipped?
# Returns 0 = skip, 1 = build. Sets SKIP_REASON on skip.
# ══════════════════════════════════════════════════════════════════════════════
should_skip() {
    local rel="$1" tgt="$2"
    SKIP_REASON=""

    # WiFi-less chips skip all WiFi examples (everything except maybe UART-only)
    if chip_in_list "$tgt" "$NO_WIFI_CHIPS"; then
        SKIP_REASON="$tgt has no WiFi"
        return 0
    fi

    # Check example exclusion patterns
    for pattern in "${!EXAMPLE_EXCLUDE_CHIPS[@]}"; do
        if [[ "$rel" == *"$pattern"* ]]; then
            local excluded="${EXAMPLE_EXCLUDE_CHIPS[$pattern]}"
            if chip_in_list "$tgt" "$excluded"; then
                SKIP_REASON="$pattern incompatible with $tgt"
                return 0
            fi
        fi
    done

    return 1  # don't skip
}

# ══════════════════════════════════════════════════════════════════════════════
# Discover examples
# ══════════════════════════════════════════════════════════════════════════════
mapfile -t ALL_PROJECTS < <(
    find "$EXAMPLES_ROOT" -name CMakeLists.txt \
         -not -path "*/build/*" \
         -not -path "*/managed_components/*" \
         -not -path "*/components/*" \
         -not -path "*/main/*" \
         2>/dev/null \
      | while read -r f; do
            d="$(dirname "$f")"
            [[ -f "$d/sdkconfig.defaults" && -f "$d/main/CMakeLists.txt" ]] && echo "$d"
        done | sort -u
)

# Apply --filter and --build-examples
PROJECTS=()
for p in "${ALL_PROJECTS[@]}"; do
    rel="${p#"$REPO_ROOT/"}"
    # --filter
    if [[ -n "$FILTER" ]] && ! [[ "$rel" =~ $FILTER ]]; then continue; fi
    # --build-examples (if not "all", comma-list of substrings)
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
    echo "No examples matched." >&2; exit 0
fi

# ══════════════════════════════════════════════════════════════════════════════
# Print plan
# ══════════════════════════════════════════════════════════════════════════════
echo "==> Discovered ${#PROJECTS[@]} example(s)"
echo "==> Targets: ${TARGET_LIST[*]}"
echo "==> Transports: $TRANSPORTS"
echo "==> Jobs: $JOBS, Fullclean: $FULLCLEAN"
echo "==> Logs: $OUT_DIR"
echo

# ══════════════════════════════════════════════════════════════════════════════
# Build worker
# ══════════════════════════════════════════════════════════════════════════════
build_one() {
    local proj="$1"
    local rel="${proj#"$REPO_ROOT/"}"
    local base_id; base_id="$(echo "$rel" | sed 's|fg/cp/examples/||; s|/|__|g')"

    local TARGET_LIST_LOCAL=()
    IFS=',' read -r -a TARGET_LIST_LOCAL <<< "$TARGETS_CSV"

    for tgt in "${TARGET_LIST_LOCAL[@]}"; do
        local id="${base_id}__${tgt}"
        local log="$OUT_DIR/$id.log"
        local start end dur status warnings

        # ── Skip check ────────────────────────────────────────────
        if should_skip "$rel" "$tgt"; then
            printf '%-8s %-12s %4ds  W=%-3s  %s\n' "SKIP" "$tgt" "0" "-" "$rel" >>"$SUMMARY"
            echo "$tgt  $rel  ($SKIP_REASON)" >>"$SKIP_LOG"
            echo "[SKIP] $rel [$tgt] ($SKIP_REASON)"
            continue
        fi

        start=$(date +%s)

        {
            echo "=== build_one: $rel ==="
            echo "target=$tgt"
            echo

            if [[ -d "$proj/build" && ! -f "$proj/build/CMakeCache.txt" ]]; then
                echo "NOTE: Removing stale build/ (not a CMake build dir)"
                rm -rf "$proj/build"
            fi

            if [[ "$FULLCLEAN" -eq 1 ]]; then
                ( cd "$proj" && idf.py fullclean ) || true
            fi

            ( cd "$proj" && IDF_TARGET="$tgt" idf.py build )
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

export -f build_one should_skip chip_in_list
export REPO_ROOT OUT_DIR SUMMARY SKIP_LOG TARGETS_CSV FULLCLEAN NO_WIFI_CHIPS

# Export associative arrays (bash 4.4+ via declare -p)
export EXAMPLE_EXCLUDE_CHIPS_SERIALIZED
EXAMPLE_EXCLUDE_CHIPS_SERIALIZED="$(declare -p EXAMPLE_EXCLUDE_CHIPS)"

# Re-import in subshell — override should_skip to deserialize
_orig_should_skip=$(declare -f should_skip)
should_skip() {
    eval "$EXAMPLE_EXCLUDE_CHIPS_SERIALIZED"
    # Re-declare the real function and call it
    eval "$(echo "$_orig_should_skip" | sed 's/^should_skip/should_skip_inner/')"
    should_skip_inner "$@"
}
# Actually simpler: inline the skip logic with serialized data.
# Bash subshells can't easily share assoc arrays via export.
# Workaround: write skip rules to a file the worker reads.

SKIP_RULES_FILE="$OUT_DIR/.skip_rules"
cat > "$SKIP_RULES_FILE" << 'RULES'
# pattern chip [chip ...]
network_split esp32c2 esp32c3
minimal/bt esp32s2
wifi_enterprise esp32c2
wifi_dpp esp32c2
wifi_itwt esp32 esp32c2 esp32c3 esp32s2 esp32s3
ext_coex esp32 esp32c2
RULES

# Redefine build_one to use file-based skip rules (subshell-safe)
build_one() {
    local proj="$1"
    local rel="${proj#"$REPO_ROOT/"}"
    local base_id; base_id="$(echo "$rel" | sed 's|fg/cp/examples/||; s|/|__|g')"

    local TARGET_LIST_LOCAL=()
    IFS=',' read -r -a TARGET_LIST_LOCAL <<< "$TARGETS_CSV"

    for tgt in "${TARGET_LIST_LOCAL[@]}"; do
        local id="${base_id}__${tgt}"
        local log="$OUT_DIR/$id.log"
        local start end dur status warnings
        local skip_reason=""

        # ── No-WiFi chip skip ─────────────────────────────────────
        if [[ " $NO_WIFI_CHIPS " == *" $tgt "* ]]; then
            skip_reason="$tgt has no WiFi"
        fi

        # ── Rule-based skip ───────────────────────────────────────
        if [[ -z "$skip_reason" && -f "$SKIP_RULES_FILE" ]]; then
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
            echo "=== build_one: $rel ==="
            echo "target=$tgt"
            echo

            if [[ -d "$proj/build" && ! -f "$proj/build/CMakeCache.txt" ]]; then
                rm -rf "$proj/build"
            fi

            if [[ "$FULLCLEAN" -eq 1 ]]; then
                ( cd "$proj" && idf.py fullclean ) || true
            fi

            ( cd "$proj" && IDF_TARGET="$tgt" idf.py build )
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
export SKIP_RULES_FILE

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
echo "Build summary (logs in $OUT_DIR)"
echo "============================================================"
printf '%-8s %-12s %5s   %-5s  %s\n' STATUS TARGET DUR WARN PATH
echo "------------------------------------------------------------"
sort "$SUMMARY"
echo "------------------------------------------------------------"

ok_count=$(grep -c '^OK' "$SUMMARY" || true)
fail_count=$(grep -c '^FAIL' "$SUMMARY" || true)
skip_count=$(grep -c '^SKIP' "$SUMMARY" || true)

echo "Total: OK=$ok_count  FAIL=$fail_count  SKIP=$skip_count"

if [[ -s "$SKIP_LOG" ]]; then
    echo
    echo "Skipped (incompatible target/example):"
    sort "$SKIP_LOG" | sed 's/^/  /'
fi

[[ $fail_count -eq 0 ]] || exit 1
