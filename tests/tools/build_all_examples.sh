#!/usr/bin/env bash
# build_all_examples.sh — Build CP firmware examples with
# target / transport / example awareness.
#
# Layout: walks `examples/**/{mcu,linux_802_3}/cp/` (CP-side projects
# only).  Host projects under `*/mcu_host/` / `*/esp_host/` build
# with different chip targets (P4 / H2) and aren't covered here —
# add a sibling build_all_host.sh if needed.
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
#   --build-incremental           Reuse existing build/sdkconfig/managed_components.
#                                 Default is a full wipe before each (project,target).
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

# Transport → Kconfig symbol (for detecting which transport an example configures).
# Keys live in coprocessor/eh_cp_transport/Kconfig.ext (`choice EH_TRANSPORT_CP_INTERFACE`).
declare -A TRANSPORT_KCONFIG=(
    [sdio]="EH_TRANSPORT_CP_SDIO"
    [spi-fd]="EH_TRANSPORT_CP_SPI"
    [spi-hd]="EH_TRANSPORT_CP_SPI_HD"
    [uart]="EH_TRANSPORT_CP_UART"
)

# Example path patterns → excluded chips (space-separated)
# Only KNOWN build failures — don't over-exclude.  Patterns are
# substring-matched against the path relative to repo root, so use
# and matches union: a path may hit several keys (e.g. a classic-BT
# example matches both `bluetooth/` and its scenario key).
declare -A EXAMPLE_EXCLUDE_CHIPS=(
    [network_split]="esp32c2 esp32c3"
    [bluetooth/]="esp32s2"
    [classic_bt_discovery]="esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32s2 esp32s3"
    [bt_hid_mouse]="esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32s2 esp32s3"
    [power_save]="esp32c2"
    [wifi/enterprise]="esp32c2"
    [wifi/dpp]="esp32c2"
    [wifi/itwt]="esp32c2 esp32 esp32c3 esp32s2 esp32s3"
    [ext_coex]="esp32 esp32c2"
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
FULLCLEAN=1   # default: wipe.  --build-incremental flips to 0.

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
    # CP-side projects live under examples/<feat>/<scenario>/{mcu,linux_802_3}/cp/.
    # Restrict the walk to those — host projects (mcu_host, esp_host)
    # use different chip targets and aren't built by this script.
    find "$EXAMPLES_ROOT" \
         \( -path "*/cp" -o -path "*/cp" \) \
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
# Skip rules — written to a file so xargs subshells can read them.
# Bash assoc arrays don't survive `export`, so we serialise to disk.
# ══════════════════════════════════════════════════════════════════════════════
SKIP_RULES_FILE="$OUT_DIR/.skip_rules"
cat > "$SKIP_RULES_FILE" << 'RULES'
# pattern chip [chip ...]   (path-substring match against examples/<rel>)
network_split        esp32c2 esp32c3
bluetooth/           esp32s2
classic_bt_discovery esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32s2 esp32s3
bt_hid_mouse         esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32s2 esp32s3
power_save           esp32c2
wifi/enterprise      esp32c2
wifi/dpp             esp32c2
wifi/itwt            esp32 esp32c2 esp32c3 esp32s2 esp32s3
ext_coex             esp32 esp32c2
RULES

# ══════════════════════════════════════════════════════════════════════════════
# Build worker — runs once per project, iterates targets internally.
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

            # Wipe everything that can leak state across targets.
            # idf.py fullclean leaves sdkconfig + dependencies.lock behind;
            # both bind the project to a specific chip / managed-component
            # set and poison the next iteration.  Explicit rm is the only
            # way to guarantee a fresh start.  --build-incremental opts
            # out and reuses whatever is on disk, untouched.
            if [[ "$FULLCLEAN" -eq 1 ]]; then
                rm -rf "$proj/build" \
                       "$proj/sdkconfig" \
                       "$proj/sdkconfig.old" \
                       "$proj/managed_components" \
                       "$proj/dependencies.lock" \
                       "$proj/main/dependencies.lock"
            fi

            # set-target regenerates sdkconfig from sdkconfig.defaults
            # for the requested chip; without it `idf.py build` aborts
            # with "sdkconfig was generated for target X, IDF_TARGET=Y".
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
# xargs spawns fresh `bash -c` subshells; build_one needs all of these.
export REPO_ROOT OUT_DIR SUMMARY SKIP_LOG TARGETS_CSV FULLCLEAN NO_WIFI_CHIPS SKIP_RULES_FILE

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
