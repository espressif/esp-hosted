#!/usr/bin/env bash
# ctx.sh — context assembler for ESP-Hosted spec system
#
# Usage:
#   ctx.sh --tag TAG_ID                     extract a named section by tag
#   ctx.sh --zone always|context|all FILE   filter file by loading preference
#   ctx.sh --ref-resolve FILE               expand @ref: markers inline
#   ctx.sh --zone context --ref-resolve F   combine: filter + expand refs
#
# Loading preferences:
#   always   — agent loads every session (contracts, invariants)
#   context  — agent loads when working in this area (detail, call flows)
#   ignore   — human reading only; agents skip (history, rejected ideas)
#
# Tag format:
#   <!-- %% sp.co.ve-rd.dc.o %% - always -->   open
#   <!-- %% sp.co.ve-rd.dc.c %% -->            close
#   <!-- %% @ref: sp.co.ve-rd.dc %% -->        cross-reference
#
# Examples:
#   ctx.sh --tag sp.co.ve-rd.dc
#   ctx.sh --zone always specs/coprocessor/verified/rpc_dispatch.md
#   ctx.sh --ref-resolve specs/system/verified/architecture.md
#   ctx.sh --zone context --ref-resolve specs/coprocessor/verified/rpc_dispatch.md

set -euo pipefail

if (( BASH_VERSINFO[0] < 4 )); then
    echo "ERROR: bash 4+ required. Install: brew install bash" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPECS_DIR="$(cd "$SCRIPT_DIR/../specs" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
REG_FILE="$SCRIPT_DIR/tag_registry.conf"

declare -A REG

load_registry() {
    while IFS= read -r line; do
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        [[ -z "${line// }" ]] && continue
        if [[ "$line" =~ ^([a-z][a-z0-9]*)[[:space:]]*=[[:space:]]*(.+)$ ]]; then
            local code="${BASH_REMATCH[1]}"
            local name="${BASH_REMATCH[2]}"
            name="${name#"${name%%[! ]*}"}"
            name="${name%"${name##*[! ]}"}"
            REG["$code"]="$name"
        fi
    done < "$REG_FILE"
}

# Resolve tag prefix → absolute file path.
# Strips section component before looking up file code.
# Exits with error on unknown code (never silently returns empty path).
resolve() {
    local tag="$1"
    local dir_part="${tag%%-*}"
    local file_part="${tag##*-}"
    local file_code="${file_part%%.*}"

    local path="$REPO_ROOT"
    IFS='.' read -ra codes <<< "$dir_part"
    for code in "${codes[@]}"; do
        local mapped="${REG[$code]:-}"
        if [[ -z "$mapped" ]]; then
            echo "ERROR: unknown code '$code' in tag '$tag'" >&2
            exit 1
        fi
        path="$path/$mapped"
    done

    local fname="${REG[$file_code]:-}"
    if [[ -z "$fname" ]]; then
        echo "ERROR: unknown file code '$file_code' in tag '$tag'" >&2
        exit 1
    fi
    echo "$path/$fname"
}

# Extract content between TAG.o and TAG.c markers in FILE.
# Fails loudly if tag not found.
extract_tag() {
    local tag="$1" file="$2"
    local content
    content=$(awk -v t="$tag" '
        $0 ~ "<!-- %% " t "\\.o %%" { found=1; in_block=1; next }
        $0 ~ "<!-- %% " t "\\.c %%" { in_block=0; next }
        in_block { print }
    ' "$file")
    if [[ -z "$content" ]]; then
        echo "ERROR: tag '$tag' not found or empty in '$file'" >&2
        exit 1
    fi
    echo "$content"
}

# Filter FILE content by loading preference zone.
# always  → output only 'always' blocks
# context → output 'always' + 'context' blocks
# all     → output everything (no filtering)
filter_zone() {
    local zone="$1" file="$2"
    if [[ "$zone" == "all" ]]; then
        cat "$file"
        return
    fi
    awk -v zone="$zone" '
        /<!-- %% [a-z][a-z0-9.]*-[a-z][a-z0-9.]*\.o %% - / {
            pref = "context"
            if ($0 ~ / - always /)  pref = "always"
            else if ($0 ~ / - context /) pref = "context"
            else if ($0 ~ / - ignore /)  pref = "ignore"
            if (zone == "always")  printing = (pref == "always")  ? 1 : 0
            if (zone == "context") printing = (pref != "ignore")  ? 1 : 0
            if (zone == "all")     printing = 1
            next
        }
        /<!-- %% [a-z][a-z0-9.]*-[a-z][a-z0-9.]*\.c %% -->/ {
            printing = 0; next
        }
        printing { print }
    ' "$file"
}

# Expand @ref: markers in FILE, inlining the referenced section.
# Exits with error if any @ref target is missing.
resolve_refs() {
    local file="$1"
    while IFS= read -r line; do
        if [[ "$line" =~ \<\!--[[:space:]]%%[[:space:]]@ref:[[:space:]]([a-z][a-z0-9.]*-[a-z][a-z0-9.]*)[[:space:]]%% ]]; then
            local ref="${BASH_REMATCH[1]}"
            local ref_file
            ref_file="$(resolve "$ref")"
            if [[ ! -f "$ref_file" ]]; then
                echo "ERROR: @ref '$ref' → '$ref_file' does not exist" >&2
                exit 1
            fi
            extract_tag "$ref" "$ref_file"
        else
            echo "$line"
        fi
    done < "$file"
}

# ── argument parsing ──────────────────────────────────────────────────────────

MODE=""
TAG_ID=""
ZONE="all"
REF_RESOLVE=0
FILES=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --tag)         MODE="tag";  TAG_ID="$2"; shift 2 ;;
        --zone)        ZONE="$2";               shift 2 ;;
        --ref-resolve) REF_RESOLVE=1;            shift   ;;
        --help|-h)
            sed -n '2,/^$/p' "$0"   # print header comment
            exit 0
            ;;
        *)             FILES+=("$1");            shift   ;;
    esac
done

load_registry

# ── dispatch ─────────────────────────────────────────────────────────────────

if [[ "$MODE" == "tag" ]]; then
    [[ -z "$TAG_ID" ]] && { echo "ERROR: --tag requires a tag ID" >&2; exit 1; }
    file="$(resolve "$TAG_ID")"
    [[ ! -f "$file" ]] && { echo "ERROR: '$TAG_ID' → '$file' does not exist" >&2; exit 1; }
    extract_tag "$TAG_ID" "$file"

elif [[ ${#FILES[@]} -gt 0 ]]; then
    for file in "${FILES[@]}"; do
        [[ ! -f "$file" ]] && { echo "ERROR: file not found: $file" >&2; exit 1; }
        if (( REF_RESOLVE )); then
            tmp="$(mktemp)"
            resolve_refs "$file" > "$tmp"
            filter_zone "$ZONE" "$tmp"
            rm -f "$tmp"
        else
            filter_zone "$ZONE" "$file"
        fi
    done

else
    echo "Usage: ctx.sh [--tag TAG | --zone ZONE | --ref-resolve] FILE..." >&2
    exit 1
fi
