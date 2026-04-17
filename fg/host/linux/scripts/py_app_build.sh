#!/bin/bash

# SPDX-License-Identifier: GPL-2.0-only
#
# Build ESP Hosted Python User Space Applications (pure Python)
# Python extension depends on RPC library, which will be built in a separate build dir.

log() { echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"; }
warn() { echo "[$(date +'%Y-%m-%d %H:%M:%S')] **Warn** $1"; }

SCRIPT_DIR=$(pwd)
ESP_ROOT=$(realpath $SCRIPT_DIR/..)
USER_SPACE_DIR=$ESP_ROOT/user_space
PY_DEMO_DIR=$USER_SPACE_DIR/python_demo_app
RPC_SRC_DIR=$(realpath $ESP_ROOT/../components/esp_hosted_rpc_lib)
PY_BUILD_DIR=$SCRIPT_DIR/build_py
RPC_BUILD_DIR=$PY_BUILD_DIR/control_lib_build

mkdir -p "$PY_BUILD_DIR"
mkdir -p "$RPC_BUILD_DIR"

# ---------------------------
# Build RPC library in out-of-source build dir
# ---------------------------
build_esp_hosted_rpc_lib() {
    log "Building RPC library..."
    cd "$RPC_BUILD_DIR"
    cmake "$RPC_SRC_DIR" -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    if [ ! -f "$RPC_BUILD_DIR/libesp_hosted_rpc.so" ]; then
        log "ERROR: Failed to build RPC library"
        exit 1
    fi
    cd "$SCRIPT_DIR"
}

# ---------------------------
# Python scripts are ready to run
# ---------------------------
print_run_instructions() {
    echo ""
    echo "Python demo scripts are ready in '$PY_DEMO_DIR'."
    echo "To run, make sure RPC library is found in LD_LIBRARY_PATH:"
    echo "  sudo LD_LIBRARY_PATH=$RPC_BUILD_DIR python3 $PY_DEMO_DIR/test.py"
    echo ""
}

# ---------------------------
# Main
# ---------------------------
build_esp_hosted_rpc_lib
log "--- Python setup finished ---"
print_run_instructions

