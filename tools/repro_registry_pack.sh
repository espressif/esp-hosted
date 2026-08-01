#!/usr/bin/env bash
# Reproduce, locally, the EXACT component the CI publish job uploads to the
# ESP Component Registry — so you can inspect it and dry-run the upload.
#
# It mirrors .github/workflows/upload_component.yml step-for-step:
#   1. isolated checkout (worktree) at a ref, submodules recursive (like CI)
#   2. bundle shared local components into examples (override_path is stripped
#      server-side, so each example must carry its local deps)
#   3. generate registry example READMEs
#   4. stage docs/changelog.md as root CHANGELOG.md
#   5. `compote component pack` — the exact archive the action uploads
#
# Your working tree is never touched (all work happens in the worktree).
#
# Usage:
#   . ~/esp-idf/export.sh            # compote comes from idf-component-manager
#   bash tools/repro_registry_pack.sh [REF]     # REF defaults to HEAD
#
# Then to reproduce SERVER-side example processing WITHOUT publishing:
#   cd <STAGE printed below>
#   compote component upload --namespace espressif --name esp_hosted --dry-run
set -euo pipefail

REPO="$(git rev-parse --show-toplevel)"
REF="${1:-HEAD}"
STAGE="${STAGE:-/tmp/eh-registry-stage}"

echo "=== [1/5] isolated checkout of $REF at $STAGE (submodules recursive) ==="
git -C "$REPO" worktree remove --force "$STAGE" 2>/dev/null || true
rm -rf "$STAGE"
git -C "$REPO" worktree add --detach "$STAGE" "$REF"
git -C "$STAGE" submodule update --init --recursive
cd "$STAGE"

echo "=== [2/5] bundle shared example components (verbatim from the workflow) ==="
# esp_hosted_examples_common -> CP examples
mkdir -p examples/network_split/station/cp/components
cp -r examples/common_components/esp_hosted_examples_common examples/network_split/station/cp/components/
mkdir -p examples/network_split/iperf/cp/components
cp -r examples/common_components/esp_hosted_examples_common examples/network_split/iperf/cp/components/
mkdir -p examples/power_save/host/network_split__host_deep_sleep/cp/components
cp -r examples/common_components/esp_hosted_examples_common examples/power_save/host/network_split__host_deep_sleep/cp/components/
mkdir -p examples/power_save/host+cp/network_split__host_deep_sleep_cp_light_sleep/cp/components
cp -r examples/common_components/esp_hosted_examples_common examples/power_save/host+cp/network_split__host_deep_sleep_cp_light_sleep/cp/components/
# esp_hosted_examples_common -> network_split mcu_host examples
mkdir -p examples/network_split/station/mcu_host/components
cp -r examples/common_components/esp_hosted_examples_common examples/network_split/station/mcu_host/components/
mkdir -p examples/network_split/iperf/mcu_host/components
cp -r examples/common_components/esp_hosted_examples_common examples/network_split/iperf/mcu_host/components/
# ---- Bluetooth ----
# The NimBLE/Bluedroid hosted-HCI stack ports are internal to esp_hosted (built
# from examples/common_components/eh_host_* by the root CMakeLists), so BT
# examples need no port bundling. nimble_uart/bleprph_host_only_uart carries
# nimble_peripheral_utils in-tree (mcu_host/components/). Only the Wi-Fi-coex
# example still bundles the shared examples_common helper:
mkdir -p examples/bluetooth/esp_hosted_nimble/bleprph_wifi_coex/mcu_host/components
cp -r examples/common_components/esp_hosted_examples_common examples/bluetooth/esp_hosted_nimble/bleprph_wifi_coex/mcu_host/components/

echo "=== [2b] strip bundled local-component deps (override_path is stripped server-side; ==="
echo "===       leftover bare deps fail version-solving — the copy above satisfies CMake) ==="
for e in \
  examples/network_split/station/cp \
  examples/network_split/iperf/cp \
  examples/power_save/host/network_split__host_deep_sleep/cp \
  examples/power_save/host+cp/network_split__host_deep_sleep_cp_light_sleep/cp \
  examples/network_split/station/mcu_host \
  examples/network_split/iperf/mcu_host \
  examples/bluetooth/esp_hosted_nimble/bleprph_wifi_coex/mcu_host ; do
  python3 tools/ci/strip_bundled_deps.py "$e/main/idf_component.yml" \
    esp_hosted_examples_common
done

echo "=== [3/5] generate registry example READMEs ==="
python3 tools/gen_registry_readmes.py

echo "=== [4/5] stage changelog at component root ==="
cp docs/changelog.md CHANGELOG.md

echo "=== [5/5] pack the component (components: esp_hosted:.) ==="
compote component pack --name esp_hosted --version 3.0.1

echo
echo "=== ARCHIVE ==="
ls -la dist/
echo
echo "=== example projects the registry will validate (count + list) ==="
tar tzf dist/esp_hosted_*.tgz | grep -E 'examples/.*/main/idf_component.yml$' | sort | tee /tmp/eh-archive-examples.txt | wc -l

echo
echo "Next steps:"
echo "  # reproduce SERVER-side example processing WITHOUT publishing:"
echo "  cd $STAGE && compote component upload --namespace espressif --name esp_hosted --dry-run"
echo "  # cleanup when done:"
echo "  git -C $REPO worktree remove --force $STAGE"
