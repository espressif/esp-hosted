#!/bin/bash
# ESP-Hosted Hardware Test Runner
#
# Usage:
#   ./run_tests.sh                    # Run all tests (this_repo source)
#   ./run_tests.sh -m boot            # Run boot tests only
#   ./run_tests.sh --src legacy       # Use external mcu6 host examples
#   ./run_tests.sh --build-only       # Build but don't flash/test
#
# Source switch:
#   --src this_repo (default) → host from examples/<feat>/<scen>/mcu/{host,esp_host}/
#   --src legacy              → host from $EH_TEST_HOST_BASE / paths.host_base (mcu6 repo)
#   Or set EH_TEST_MODE=legacy in env.
#
# Prerequisites:
#   - IDF environment sourced (. ~/esp-idf/export.sh)
#   - pytest-embedded installed (pip install pytest-embedded pytest-embedded-serial pytest-embedded-serial-esp pytest-embedded-idf)
#   - ESP32-P4 on /dev/cu.usbserial-120 (host)
#   - ESP32-C6 on /dev/cu.usbserial-11301 (slave/CP)

set -e

HOST_PORT="${HOST_PORT:-/dev/cu.usbserial-120}"
SLAVE_PORT="${SLAVE_PORT:-/dev/cu.usbserial-11301}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ESP_HOSTED_NEW="$(cd "$SCRIPT_DIR/../.." && pwd)"
ESP_HOSTED_MCU="${ESP_HOSTED_MCU:-$HOME/code/esp_hosted_mcu6}"

# Parse args
PYTEST_ARGS=()
BUILD_ONLY=false
SRC=""
i=0
args=("$@")
while [ $i -lt ${#args[@]} ]; do
    arg="${args[i]}"
    case "$arg" in
        --build-only)
            BUILD_ONLY=true
            ;;
        --src)
            i=$((i+1))
            SRC="${args[i]}"
            ;;
        --src=*)
            SRC="${arg#--src=}"
            ;;
        *)
            PYTEST_ARGS+=("$arg")
            ;;
    esac
    i=$((i+1))
done

# Source override → exported for runner / config.py to pick up.
if [ -n "$SRC" ]; then
    export EH_TEST_MODE="$SRC"
fi
echo "[INFO] Example source: ${EH_TEST_MODE:-this_repo}"

echo "=== ESP-Hosted Hardware Test ==="
echo "Host port:  $HOST_PORT (ESP32-P4)"
echo "Slave port: $SLAVE_PORT (ESP32-C6)"
echo ""

# Trick: put P4 in bootloader so C6 SDIO lines are free during CP flash
put_p4_in_boot() {
    echo "[INFO] Putting P4 in bootloader mode..."
    python -m esptool --port "$HOST_PORT" --before default-reset --after no-reset run 2>/dev/null || true
    sleep 1
}

if [ "$BUILD_ONLY" = true ]; then
    echo "[INFO] Build-only mode"
    exit 0
fi

echo "[INFO] Running pytest-embedded..."
cd "$SCRIPT_DIR"

pytest \
    --embedded-services serial \
    --count 2 \
    --port "$HOST_PORT|$SLAVE_PORT" \
    -v -s \
    "${PYTEST_ARGS[@]}"
