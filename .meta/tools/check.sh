#!/usr/bin/env bash
# check.sh — validate ESP-Hosted spec tag system integrity
#
# Usage:
#   .meta/tools/check.sh            # hard checks only
#   .meta/tools/check.sh --soft     # hard + soft warnings
#
# Hard checks (exit 1 on any failure):
#   1. Registry: no duplicate codes
#   2. Per-file: every .o tag has exactly one matching .c tag
#   3. Global: no duplicate tag IDs across repo
#   4. Consistency: tag prefix resolves to the file it lives in
#   5. @ref: every referenced tag exists somewhere
#
# Soft checks (--soft, warnings only):
#   S1. lessons.md has no unreviewed [PENDING] items
#   S2. AGENT_CONTEXT.md ≤ 80 lines
#   S3. verified specs with last_verified > 60 days old

set -euo pipefail

if (( BASH_VERSINFO[0] < 4 )); then
    echo "ERROR: bash 4+ required. Install: brew install bash" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPECS_DIR="$(cd "$SCRIPT_DIR/../specs" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
REG_FILE="$SCRIPT_DIR/tag_registry.conf"
SOFT="${1:-}"

ERRORS=0
WARNINGS=0

# ── registry ──────────────────────────────────────────────────────────────────

declare -A REG   # code → name

load_registry() {
    while IFS= read -r line; do
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        [[ -z "${line// }" ]] && continue
        if [[ "$line" =~ ^([a-z][a-z0-9]*)[[:space:]]*=[[:space:]]*(.+)$ ]]; then
            local code="${BASH_REMATCH[1]}"
            local name="${BASH_REMATCH[2]}"
            name="${name#"${name%%[! ]*}"}"   # ltrim
            name="${name%"${name##*[! ]}"}"   # rtrim
            REG["$code"]="$name"
        fi
    done < "$REG_FILE"
}

# Resolve tag prefix → absolute path.
# Input:  sp.co.ve-rd  or  sp-ac  or  fg.cp.src-cor
# Output: /abs/path/to/file  OR  ERR_UNKNOWN:code
resolve() {
    local tag="$1"
    local dir_part="${tag%%-*}"          # before first '-'
    local file_part="${tag##*-}"         # after last '-'
    local file_code="${file_part%%.*}"   # strip section components

    local path="$REPO_ROOT"
    IFS='.' read -ra codes <<< "$dir_part"
    for code in "${codes[@]}"; do
        local mapped="${REG[$code]:-}"
        [[ -z "$mapped" ]] && { echo "ERR_UNKNOWN:$code"; return 0; }
        path="$path/$mapped"
    done

    local fname="${REG[$file_code]:-}"
    [[ -z "$fname" ]] && { echo "ERR_UNKNOWN:$file_code"; return 0; }
    echo "$path/$fname"
}

# ── helpers ───────────────────────────────────────────────────────────────────

fail() { printf "  FAIL: %s\n" "$*" >&2; (( ERRORS++ )) || true; }
warn() { printf "  WARN: %s\n" "$*";    (( WARNINGS++ )) || true; }
pass() { printf "  ok:   %s\n" "$*"; }

# Files to check: specs (excl. legacy) + source code
spec_files()  { find "$SPECS_DIR" -name "*.md" -not -path "*/legacy/*" -print0; }
src_files()   { find "$REPO_ROOT/fg" "$REPO_ROOT/ng" -not -path "*/build/*" \( -name "*.c" -o -name "*.h" \) -print0 2>/dev/null || true; }
all_files()   { spec_files; src_files; }

# Tag base pattern: lowercase, dirs joined by '.', '-', file, optional '.section' parts
TAG_PAT='[a-z][a-z0-9.]*-[a-z][a-z0-9.]*'

# ── check 1: registry uniqueness ──────────────────────────────────────────────

check1_registry() {
    echo "Check 1: Registry uniqueness"
    local -A seen; local dup=0
    while IFS= read -r line; do
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        [[ -z "${line// }" ]] && continue
        if [[ "$line" =~ ^([a-z][a-z0-9]*)[[:space:]]*= ]]; then
            local code="${BASH_REMATCH[1]}"
            if [[ -n "${seen[$code]:-}" ]]; then
                fail "Duplicate registry code: '$code'"; (( dup++ )) || true
            fi
            seen["$code"]=1
        fi
    done < "$REG_FILE"
    [[ $dup -eq 0 ]] && pass "all codes unique"
}

# ── check 2: open/close matching per file ─────────────────────────────────────

check2_matching() {
    echo "Check 2: Tag open/close matching (per file)"
    local total=0 bad=0

    while IFS= read -r -d '' file; do
        local rel="${file#"$REPO_ROOT/"}"
        declare -A op=() cl=()

        while IFS= read -r line; do
            if [[ "$line" =~ \<\!--[[:space:]]%%[[:space:]]($TAG_PAT)\.o[[:space:]]%% ]]; then
                local t="${BASH_REMATCH[1]}"
                op["$t"]=$(( ${op["$t"]:-0} + 1 )); (( total++ )) || true
            fi
            if [[ "$line" =~ \<\!--[[:space:]]%%[[:space:]]($TAG_PAT)\.c[[:space:]]%% ]]; then
                local t="${BASH_REMATCH[1]}"
                cl["$t"]=$(( ${cl["$t"]:-0} + 1 ))
            fi
        done < "$file"

        for t in "${!op[@]}"; do
            [[ "${op[$t]}" != "${cl[$t]:-0}" ]] && {
                fail "$rel: '$t' — ${op[$t]} open, ${cl[$t]:-0} close"; (( bad++ )) || true; }
        done
        for t in "${!cl[@]}"; do
            [[ -z "${op[$t]:-}" ]] && {
                fail "$rel: '$t' — close with no open"; (( bad++ )) || true; }
        done
        unset op cl
    done < <(all_files)

    [[ $bad -eq 0 ]] && pass "$total open tags, all matched"
}

# ── check 3: global tag uniqueness ────────────────────────────────────────────

check3_uniqueness() {
    echo "Check 3: Global tag uniqueness"
    declare -A seen_in; local dup=0

    while IFS= read -r -d '' file; do
        local rel="${file#"$REPO_ROOT/"}"
        while IFS= read -r line; do
            if [[ "$line" =~ \<\!--[[:space:]]%%[[:space:]]($TAG_PAT)\.o[[:space:]]%% ]]; then
                local t="${BASH_REMATCH[1]}"
                if [[ -n "${seen_in[$t]:-}" ]]; then
                    fail "Duplicate tag '$t': ${seen_in[$t]} and $rel"; (( dup++ )) || true
                else
                    seen_in["$t"]="$rel"
                fi
            fi
        done < "$file"
    done < <(all_files)

    [[ $dup -eq 0 ]] && pass "all tag IDs globally unique"
}

# ── check 4: tag prefix → file path consistency ───────────────────────────────

check4_paths() {
    echo "Check 4: Tag prefix → file path"
    local bad=0

    while IFS= read -r -d '' file; do
        local rel="${file#"$REPO_ROOT/"}"
        while IFS= read -r line; do
            if [[ "$line" =~ \<\!--[[:space:]]%%[[:space:]]($TAG_PAT)\.[oc][[:space:]]%% ]]; then
                local tag="${BASH_REMATCH[1]}"
                [[ "$tag" == *"@ref"* ]] && continue
                # Extract file-level prefix (strip section components after file code)
                local dp="${tag%%-*}"
                local fp="${tag##*-}"; fp="${fp%%.*}"
                local expected; expected="$(resolve "${dp}-${fp}")"
                if [[ "$expected" == ERR_* ]]; then
                    fail "$rel: '$tag' — $expected"; (( bad++ )) || true
                elif [[ "$expected" != "$file" ]]; then
                    fail "$rel: '$tag' resolves to $expected"; (( bad++ )) || true
                fi
            fi
        done < "$file"
    done < <(spec_files)

    [[ $bad -eq 0 ]] && pass "all tag prefixes match file paths"
}

# ── check 5: @ref targets exist ───────────────────────────────────────────────

check5_refs() {
    echo "Check 5: @ref targets exist"
    declare -A known
    while IFS= read -r -d '' file; do
        while IFS= read -r line; do
            [[ "$line" =~ \<\!--[[:space:]]%%[[:space:]]($TAG_PAT)\.o[[:space:]]%% ]] \
                && known["${BASH_REMATCH[1]}"]=1
        done < "$file"
    done < <(all_files)

    local bad=0
    while IFS= read -r -d '' file; do
        local rel="${file#"$REPO_ROOT/"}"
        while IFS= read -r line; do
            if [[ "$line" =~ \<\!--[[:space:]]%%[[:space:]]@ref:[[:space:]]($TAG_PAT)[[:space:]]%% ]]; then
                local ref="${BASH_REMATCH[1]}"
                [[ -z "${known[$ref]:-}" ]] && {
                    fail "$rel: @ref '$ref' — tag not found"; (( bad++ )) || true; }
            fi
        done < "$file"
    done < <(spec_files)

    [[ $bad -eq 0 ]] && pass "all @ref targets exist"
}

# ── soft checks ───────────────────────────────────────────────────────────────

check_soft() {
    echo "Soft checks"

    # S1: pending lessons
    local lf="$SPECS_DIR/tasks/lessons.md"
    if [[ -f "$lf" ]]; then
        local n; n=$(grep -cP '\*\*Status\*\*:.*\[PENDING|\bStatus:\s+\[PENDING' "$lf" 2>/dev/null || echo 0)
        (( n > 0 )) && warn "lessons.md: $n [PENDING] item(s) need user review" \
                     || pass "no pending lessons"
    fi

    # S2: AGENT_CONTEXT.md size
    local ac="$SPECS_DIR/AGENT_CONTEXT.md"
    [[ -f "$ac" ]] && {
        local lines; lines=$(wc -l < "$ac" | tr -d ' ')
        (( lines > 80 )) && warn "AGENT_CONTEXT.md: $lines lines (target ≤80)" \
                          || pass "AGENT_CONTEXT.md: $lines lines"
    }

    # S3: last_verified age
    local today; today=$(date +%Y-%m-%d)
    local ts_today; ts_today=$(date -j -f "%Y-%m-%d" "$today" "+%s" 2>/dev/null \
                               || date -d "$today" "+%s" 2>/dev/null || echo 0)

    while IFS= read -r -d '' file; do
        local rel="${file#"$REPO_ROOT/"}"
        local ds; ds=$(grep -m1 'last_verified:' "$file" 2>/dev/null \
                       | sed 's/.*last_verified:[[:space:]]*//' | tr -d ' \r' || true)
        if [[ -n "$ds" && "$ds" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}$ ]]; then
            local ts_s; ts_s=$(date -j -f "%Y-%m-%d" "$ds" "+%s" 2>/dev/null \
                               || date -d "$ds" "+%s" 2>/dev/null || echo 0)
            if [[ "$ts_today" -gt 0 && "$ts_s" -gt 0 ]]; then
                local age=$(( (ts_today - ts_s) / 86400 ))
                if (( age > 60 )); then
                    warn "$rel: last_verified=$ds (${age}d) — confirm vs code"
                fi
            fi
        fi
    done < <(find "$SPECS_DIR"/*/verified -name "*.md" -print0 2>/dev/null)
}

# ── main ──────────────────────────────────────────────────────────────────────

echo "=== ESP-Hosted spec check ==="
echo ""
load_registry
check1_registry;  echo ""
check2_matching;  echo ""
check3_uniqueness; echo ""
check4_paths;     echo ""
check5_refs
[[ "$SOFT" == "--soft" ]] && { echo ""; check_soft; }
echo ""
if (( ERRORS > 0 )); then
    echo "RESULT: FAILED — $ERRORS error(s)${WARNINGS:+, $WARNINGS warning(s)}"
    exit 1
else
    echo "RESULT: OK${WARNINGS:+ — $WARNINGS warning(s)}"
    exit 0
fi
