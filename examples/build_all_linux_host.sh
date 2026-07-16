#!/usr/bin/env bash
# build_all_linux_host.sh — per-project loop runner for Linux user-space examples.

set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "${here}/.." && pwd)"
hosted="${repo}/tools/eh.py"
export EH_PATH="${repo}"

examples=(
    "system/get_cp_fw_version/linux_802_3_host/c_app"
    "system/get_cp_fw_version/linux_802_3_host/py_app"
    "system/hosted_events/linux_802_3_host/c_app"
    "system/hosted_events/linux_802_3_host/py_app"
    "wifi/sta/linux_802_3_host/c_app"
    "wifi/sta/linux_802_3_host/py_app"
    "network_split/iperf/linux_802_3_host/c_app"
    "network_split/iperf/linux_802_3_host/py_app"
    "network_split/station/linux_802_3_host/c_app"
    "network_split/station/linux_802_3_host/py_app"
    "ext_coex/linux_802_3_host/c_app"
    "ext_coex/linux_802_3_host/py_app"
    "gpio_expander/linux_802_3_host/c_app"
    "gpio_expander/linux_802_3_host/py_app"
    "ota/coprocessor_ota/linux_802_3_host/c_app"
    "ota/coprocessor_ota/linux_802_3_host/py_app"
    "peer_data_transfer/echo/linux_802_3_host/c_app"
    "peer_data_transfer/echo/linux_802_3_host/py_app"
)

passed=0
failed=0
skipped=0
fail_list=()
skip_list=()

echo ""
for rel in "${examples[@]}"; do
    dir="${here}/${rel}"
    if [ ! -d "${dir}" ]; then
        printf "%-60s \e[33mSKIP\e[0m  dir missing\n" "${rel}"
        skipped=$((skipped + 1))
        skip_list+=("${rel} (dir missing)")
        continue
    fi
    if ! grep -q "hosted_project\.cmake" "${dir}/CMakeLists.txt" 2>/dev/null; then
        printf "%-60s \e[33mSKIP\e[0m  not IDF-shape\n" "${rel}"
        skipped=$((skipped + 1))
        skip_list+=("${rel} (legacy flat shape)")
        continue
    fi

    log="${dir}/build/build.log"
    mkdir -p "${dir}/build"
    if (cd "${dir}" && python3 "${hosted}" set-target posix >"${log}" 2>&1 \
            && python3 "${hosted}" build >>"${log}" 2>&1); then
        case "${rel}" in
            *host_py_app)
                artefact="${dir}/build/eh_libeh_host/libeh_host.so"
                if [ -f "${artefact}" ]; then
                    size=$(ls -lh "${artefact}" | awk '{print $5}')
                    printf "%-60s \e[32mPASS\e[0m  %s (%s)\n" "${rel}" "${artefact#${repo}/}" "${size}"
                    passed=$((passed + 1))
                else
                    printf "%-60s \e[31mFAIL\e[0m  build succeeded but no libeh_host.so\n" "${rel}"
                    failed=$((failed + 1))
                    fail_list+=("${rel} (no .so)")
                fi
                ;;
            *)
                bin="$(find "${dir}/build" -maxdepth 4 -type f -executable \
                        -name '*_c_app' -print -quit 2>/dev/null || true)"
                if [ -n "${bin}" ]; then
                    size=$(ls -lh "${bin}" | awk '{print $5}')
                    printf "%-60s \e[32mPASS\e[0m  %s (%s)\n" "${rel}" "${bin#${repo}/}" "${size}"
                    passed=$((passed + 1))
                else
                    printf "%-60s \e[31mFAIL\e[0m  build succeeded but no _c_app binary\n" "${rel}"
                    failed=$((failed + 1))
                    fail_list+=("${rel} (no binary)")
                fi
                ;;
        esac
    else
        printf "%-60s \e[31mFAIL\e[0m  see ${log#${repo}/}\n" "${rel}"
        failed=$((failed + 1))
        fail_list+=("${rel}")
    fi
done

echo ""
echo "${passed} passed, ${failed} failed, ${skipped} skipped (of ${#examples[@]} examples)."
if [ ${failed} -gt 0 ]; then
    echo ""
    echo "Failures:"
    for entry in "${fail_list[@]}"; do
        echo "  - ${entry}"
    done
    exit 1
fi
if [ ${skipped} -gt 0 ]; then
    echo ""
    echo "Skipped:"
    for entry in "${skip_list[@]}"; do
        echo "  - ${entry}"
    done
fi
