#!/usr/bin/env bash

set -euo pipefail

if [ -z "${HOSTED_NG_PATH:-}" ]; then
    echo "HOSTED_NG_PATH must be set before running this script"
    exit 1
fi

FAILURES=0

function check_md5() {
    local host_filename="$1"
    local fw_filename="$2"

    local md5_from_host
    local md5_from_fw

    md5_from_host=$(md5sum "$host_filename" | cut -c 1-7)
    md5_from_fw=$(md5sum "$fw_filename" | cut -c 1-7)

    echo "Comparing MD5 checksums for:"
    echo "  $host_filename (host) -> $md5_from_host"
    echo "  $fw_filename (firmware) -> $md5_from_fw"

    if [ "$md5_from_host" != "$md5_from_fw" ]; then
        echo "  error: MD5 mismatch!"
        FAILURES=$((FAILURES + 1))
    fi
}

function check_version_update() {
    local network_adapter_dir="${HOSTED_NG_PATH}/esp/esp_driver/network_adapter"
    local version_file="${HOSTED_NG_PATH}/esp/esp_driver/network_adapter/main/include/esp_fw_version.h"

    # Determine the base commit for the MR
    local target_branch="${CI_MERGE_REQUEST_TARGET_BRANCH_NAME:-master}"
    local base_commit
    base_commit=$(git merge-base HEAD "origin/$target_branch")

    # Check if there are changes in network_adapter_dir in the MR range
    if git diff --quiet "$base_commit" HEAD -- "$network_adapter_dir"; then
        echo "No changes in ${network_adapter_dir}, skipping version check."
    else
        # Check if version_file was also updated
        if ! git diff --quiet "$base_commit" HEAD -- "$version_file"; then
            echo "Change detected in firmware, and version has been updated."
        else
            echo "error: version must be updated when changes are made in firmware"
            FAILURES=$((FAILURES + 1))
        fi
    fi
}

echo "Checking if version in host matches version in firmware"
check_md5 "${HOSTED_NG_PATH}/esp/esp_driver/network_adapter/main/include/esp_fw_version.h" \
          "${HOSTED_NG_PATH}/host/include/esp_fw_version.h"

echo "Checking if interface in host matches interface in firmware"
check_md5 "${HOSTED_NG_PATH}/esp/esp_driver/network_adapter/main/include/adapter.h" \
          "${HOSTED_NG_PATH}/host/include/adapter.h"

echo "Checking if version file was updated with network adapter changes"
check_version_update

if [ $FAILURES -gt 0 ]; then
    echo "$FAILURES check(s) failed."
    exit 1
else
    echo "All checks passed."
fi
