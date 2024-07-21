#!/usr/bin/env bash

set -euo pipefail

if [ -z "${HOSTED_NG_PATH:-}" ]; then
    echo "HOSTED_NG_PATH must be set before running this script"
    exit 1
fi

echo "Checking if version in host matches version in firmware"

FAILURES=0

function check_md5()
{
    HOST_FILENAME=$1
    FW_FILENAME=$2

    MD5_FROM_HOST=$(md5sum ${HOST_FILENAME} | cut -c 1-7)
    MD5_FROM_FW=$(md5sum ${FW_FILENAME} | cut -c 1-7)

    echo "  ${MD5_FROM_HOST} - from host file"
    echo "  ${MD5_FROM_FW} - from firmware file"
    if [ "${MD5_FROM_HOST}" != "${MD5_FROM_FW}" ]; then
        echo "  error: MD5 mismatch!"
        FAILURES=$(($FAILURES+1))
    fi
}

check_md5 ${HOSTED_NG_PATH}/esp/esp_driver/network_adapter/main/include/esp_fw_version.h ${HOSTED_NG_PATH}/host/include/esp_fw_version.h

if [ $FAILURES -gt 0 ]; then
    exit 1
fi
