#!/usr/bin/env bash

set -euo pipefail

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <setup.sh path> <setup.ps1 path>"
    exit 2
fi

SETUP_SH="$1"
SETUP_PS1="$2"

# Determine the base commit for the MR
TARGET_BRANCH="${CI_MERGE_REQUEST_TARGET_BRANCH_NAME:-master}"
git fetch origin "$TARGET_BRANCH" --depth=1
BASE_COMMIT=$(git merge-base HEAD "origin/$TARGET_BRANCH")

SH_CHANGED=0
PS1_CHANGED=0

if ! git diff --quiet "$BASE_COMMIT" HEAD -- "$SETUP_SH"; then
    SH_CHANGED=1
fi

if ! git diff --quiet "$BASE_COMMIT" HEAD -- "$SETUP_PS1"; then
    PS1_CHANGED=1
fi

if [ "$SH_CHANGED" -ne "$PS1_CHANGED" ]; then
    echo "ERROR: Sync mismatch between $SETUP_SH and $SETUP_PS1"
    if [ "$SH_CHANGED" -eq 1 ]; then
        echo "$SETUP_SH was modified, but $SETUP_PS1 was not."
    else
        echo "$SETUP_PS1 was modified, but $SETUP_SH was not."
    fi
    echo "Please ensure both setup scripts are updated to maintain parity."
    exit 1
fi

echo "Setup scripts are in sync (both changed or both unchanged)."
exit 0
