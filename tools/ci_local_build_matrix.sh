#!/usr/bin/env bash
#
# SPDX-License-Identifier: Apache-2.0
#
# Local reproduction of the build job in .github/workflows/build_and_checks.yml.
# Builds our OWN esp_hosted examples inside the same espressif/idf containers,
# with the same EXTRA_CFLAGS (esp_wifi_remote / normal IDF component level), so
# the CI can be validated on a beefy machine BEFORE committing/pushing.
#
# Each example resolves esp_hosted via its own idf_component.yml override_path
# (relative to the example dir -> repo root) — no symlink or manifest surgery.
#
# Requires: docker (with enough disk for the IDF images) + this repo checked out.
#
# Usage:
#   tools/ci_local_build_matrix.sh                              # full spread (console + log)
#   tools/ci_local_build_matrix.sh --config-only               # kconfig pass only (no compile)
#   tools/ci_local_build_matrix.sh [flags] <idf_ver> <example_path> <target> [override]
#     flags: --config-only|-c   kconfig/configure pass only (no compile), for the pruner
#            --console          write output to console only
#            --log              write output to the log file only
#            --both             write output to console AND log file (default)
#     e.g. tools/ci_local_build_matrix.sh release-v5.5 examples/wifi/sta/cp esp32c6
#          tools/ci_local_build_matrix.sh v5.5.1 examples/wifi/apsta/cp esp32c3 CONFIG_EH_TRANSPORT_CP_SPI=y
#   Session output is logged to /tmp/eh_ci_build_matrix.log (overwritten each run;
#   override path with EH_CI_LOG=/path). Then e.g.:
#          tools/eh_prune_stale_kconfig.py /tmp/eh_ci_build_matrix.log

set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Shared, persistent ccache across all cells (and re-runs) so builds become
# CPU-bound instead of re-compiling cold every container. ccache is concurrency-
# and multi-version-safe, so JOBS>1 cells can share one dir.
EH_CCACHE_DIR="${EH_CCACHE_DIR:-${HOME}/.cache/eh_ci_ccache}"
mkdir -p "${EH_CCACHE_DIR}"

# Total CPU budget for the whole matrix; leaves ~25% of cores free by default so
# the box stays usable. Split across concurrent cells (see PER_CPUS below).
NPROC="$(nproc)"
EH_MAX_CPUS="${EH_MAX_CPUS:-$(( NPROC * 3 / 4 ))}"
[ "${EH_MAX_CPUS}" -lt 1 ] && EH_MAX_CPUS=1
PER_CPUS="${EH_MAX_CPUS}"

# idf_ver|path|target|override  (override empty => example default transport, SDIO)
CELLS=(
  # --- co-processor builds (chipset + transport spread) ---
  "release-v5.5|examples/wifi/sta/cp|esp32c6|"
  "latest|examples/wifi/iperf/cp|esp32c5|"
  "v5.5.1|examples/wifi/apsta/cp|esp32c3|CONFIG_EH_TRANSPORT_CP_SPI=y"
  "release-v5.5|examples/wifi/scan/cp|esp32c2|CONFIG_EH_TRANSPORT_CP_SPI_HD=y"
  "latest|examples/wifi/softap/cp|esp32s3|CONFIG_EH_TRANSPORT_CP_UART=y"
  "v5.5|examples/wifi/dpp/cp|esp32c6|"
  "release-v6.0|examples/wifi/itwt/cp|esp32c5|"
  "release-v6.0|examples/network_split/station/cp|esp32c61|"
  "release-v5.5|examples/network_split/iperf/cp|esp32c6|"
  "release-v5.5|examples/bluetooth/esp_hosted_nimble/bleprph_gatt/cp|esp32c6|"
  "latest|examples/bluetooth/nimble_uart/bleprph_host_only_uart/cp|esp32|"
  "latest|examples/openthread/cli/cp|esp32h2|"
  "release-v5.5|examples/ota/coprocessor_ota/cp|esp32|"
  "v5.5.1|examples/ext_coex/cp|esp32c6|"
  "release-v5.5|examples/gpio_expander/cp|esp32c3|"
  "latest|examples/mem_monitor/cp|esp32c2|"
  "release-v6.0|examples/peer_data_transfer/cp|esp32c6|"
  "release-v5.5|examples/system/api_exerciser/cp|esp32c6|"
  "v5.5.1|examples/power_save/cp/shut_down_cp_when_unused/cp|esp32c6|"
  # --- host builds (p4 + h2) ---
  "latest|examples/wifi/iperf/mcu_host|esp32p4|"
  "release-v5.5|examples/wifi/sta/mcu_host|esp32h2|"
  "release-v6.0|examples/network_split/iperf/mcu_host|esp32p4|"
  "release-v5.5|examples/wifi/enterprise/mcu_host|esp32p4|"
  "v5.5.1|examples/power_save/host/network_split__host_deep_sleep/esp_host|esp32p4|"
  "latest|examples/bluetooth/esp_hosted_nimble/bleprph_wifi_coex/mcu_host|esp32p4|"
  "release-v5.5|examples/bluetooth/nimble_uart/bleprph_host_only_uart/mcu_host|esp32h2|"
  "release-v5.5|examples/gpio_expander/mcu_host|esp32p4|"
  "latest|examples/openthread/cli/esp_host|esp32p4|"
  "release-v6.0|examples/ota/coprocessor_ota/mcu_host|esp32p4|"
  "v5.5.1|examples/system/mcu_transport_config/mcu_host|esp32p4|"
  "release-v6.0|examples/mcu_hosted_sdio_sdmmc_combined/mcu_host|esp32p4|"
)

# Build recipe run INSIDE the idf container. $1=example path $2=target $3=override(may be empty)
inner() {
  cat <<'INNER'
set -euo pipefail
EXAMPLE="$1"; TARGET="$2"; OVERRIDE="${3:-}"; CONFIG_ONLY="${4:-0}"
# The espressif/idf image already ships all toolchains, so skip the slow
# per-cell install.sh --enable-ci (it only pip-installs Python CI deps, which
# aren't needed to build). Just activate. Warning strictness comes from IDF's
# own flags + EXTRA_CFLAGS, not from install.sh.
. "${IDF_PATH}/export.sh"
export IDF_CCACHE_ENABLE=1
# Match esp_wifi_remote / normal IDF component level: -Werror=all + the classes
# that block downstream. The heavier esp_hosted-only set is opt-in via
# ESP_HOSTED_CI_PEDANTIC=1 (kept out here to get CI green first).
export EXTRA_CFLAGS="-DIDF_CI_BUILD -Werror=deprecated-declarations -Werror=unused-variable -Werror=unused-but-set-variable -Werror=unused-function"
export EXTRA_CXXFLAGS="${EXTRA_CFLAGS}"
# Container runs as root; repo is bind-mounted and owned by the host user.
git config --global --add safe.directory /esp_hosted
cd /esp_hosted
# Hermetic like fresh CI: drop any dependencies.lock/managed_components left by a
# prior run (a stale lock bakes an absolute override path and breaks re-resolution).
rm -f "${EXAMPLE}/dependencies.lock"
rm -rf "${EXAMPLE}/managed_components"
if [ "${CONFIG_ONLY}" = "1" ]; then
    # kconfig/configure pass only — no compilation. Emits the same "unknown
    # kconfig symbol" warnings as a full build, in seconds. Feed the output to
    # tools/eh_prune_stale_kconfig.py.
    idf.py -C "${EXAMPLE}" -B "${EXAMPLE}/build" -DIDF_TARGET="${TARGET}" reconfigure
else
    command -v idf-build-apps >/dev/null || pip install idf-build-apps
    OVR=""
    [ -n "${OVERRIDE}" ] && OVR="--override-sdkconfig-items=${OVERRIDE}"
    idf-build-apps build -p "${EXAMPLE}" --target "${TARGET}" ${OVR} -vv
fi
INNER
}

run_cell() {
  local ver="$1" example="$2" target="$3" override="${4:-}"
  echo "==================================================================="
  echo ">>> idf:${ver}  ${example}  ${target}  ${override:+[$override]}"
  echo "==================================================================="
  # Mount at /esp_hosted (NOT /repo): IDF resolves the esp_hosted component by the
  # override dir's basename during requirement expansion, so the dir must be named it.
  ${DOCKER:-docker} run --rm \
    --cpus="${PER_CPUS}" \
    -v "${REPO_ROOT}":/esp_hosted \
    -v "${EH_CCACHE_DIR}":/root/.cache/ccache \
    -e HOME=/root -e CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-10G}" \
    "espressif/idf:${ver}" bash -lc "$(inner)" _ "${example}" "${target}" "${override}" "${CONFIG_ONLY:-0}"
}

# Flags parsed here so the positional single-cell args (ver example target
# [override]) still work after them. --config-only runs the kconfig/configure
# pass only (no compile), for tools/eh_prune_stale_kconfig.py.
CONFIG_ONLY=0
OUTPUT_MODE=both
while [ "$#" -gt 0 ]; do
  case "${1:-}" in
    --config-only|-c) CONFIG_ONLY=1; shift ;;
    --console)        OUTPUT_MODE=console; shift ;;
    --log)            OUTPUT_MODE=log; shift ;;
    --both)           OUTPUT_MODE=both; shift ;;
    *) break ;;
  esac
done

# Session output: console / log / both (default). Log file is truncated (older
# run replaced) each invocation; override the path with EH_CI_LOG.
LOG_FILE="${EH_CI_LOG:-/tmp/eh_ci_build_matrix.log}"
if [ "${OUTPUT_MODE}" != "console" ]; then
  echo "[matrix] logging session output to ${LOG_FILE} (mode: ${OUTPUT_MODE})" >&2
fi
case "${OUTPUT_MODE}" in
  log)  exec >"${LOG_FILE}" 2>&1 ;;
  both) exec > >(tee "${LOG_FILE}") 2>&1 ;;
  # console: no redirect
esac

if [ "$#" -ge 3 ]; then
  run_cell "$1" "$2" "$3" "${4:-}"; exit $?
fi

# JOBS>1 builds that many cells at once. The EH_MAX_CPUS budget is split evenly
# across them (--cpus per container), so total CPU stays bounded regardless of
# JOBS. JOBS=1 (default) keeps the original live, streaming, single-stream output.
JOBS="${JOBS:-1}"
if [ "${JOBS}" -gt 1 ]; then
  PER_CPUS=$(( EH_MAX_CPUS / JOBS ))
  [ "${PER_CPUS}" -lt 1 ] && PER_CPUS=1
fi
declare -a PASS=() FAIL=()

if [ "${JOBS}" -le 1 ]; then
  for cell in "${CELLS[@]}"; do
    IFS='|' read -r ver ex tg ovr <<< "${cell}"
    if run_cell "${ver}" "${ex}" "${tg}" "${ovr}"; then PASS+=("${ex}@${tg}/${ver}"); else FAIL+=("${ex}@${tg}/${ver}"); fi
  done
else
  LOGDIR="$(mktemp -d)"
  echo "Running ${#CELLS[@]} cells, ${JOBS} at a time. Per-cell logs under ${LOGDIR}/"
  declare -a PIDS=() LABELS=() LOGS=()
  idx=0
  for cell in "${CELLS[@]}"; do
    IFS='|' read -r ver ex tg ovr <<< "${cell}"
    label="${ex}@${tg}/${ver}"; log="${LOGDIR}/${idx}.log"
    echo ">>> [start] ${label}"
    run_cell "${ver}" "${ex}" "${tg}" "${ovr}" >"${log}" 2>&1 &
    PIDS+=("$!"); LABELS+=("${label}"); LOGS+=("${log}"); idx=$((idx + 1))
    while [ "$(jobs -rp | wc -l)" -ge "${JOBS}" ]; do sleep 1; done
  done
  for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}"; then PASS+=("${LABELS[$i]}"); echo "PASS  ${LABELS[$i]}"
    else FAIL+=("${LABELS[$i]}"); echo "FAIL  ${LABELS[$i]}  (log: ${LOGS[$i]})"; fi
  done
fi

echo ""
echo "################### SUMMARY ###################"
echo "PASS (${#PASS[@]}):"; printf '  %s\n' "${PASS[@]:-<none>}"
echo "FAIL (${#FAIL[@]}):"; printf '  %s\n' "${FAIL[@]:-<none>}"
[ "${#FAIL[@]}" -eq 0 ]
