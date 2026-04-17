#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Espressif Systems Wireless LAN device driver
# Build C User Space Applications Script
#
# SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
#

log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

warn() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] **Warn** $1"
}

log_enter() {
    log "Entering ${FUNCNAME[1]}"
}

log_exit() {
    log "Exiting ${FUNCNAME[1]}"
}

# ---------------------------
# Paths (portable)
# ---------------------------
SCRIPT_DIR=$(pwd)
ESP_ROOT=$(realpath $SCRIPT_DIR/..)
USER_SPACE_DIR=$ESP_ROOT/user_space
C_DEMO_DIR=$USER_SPACE_DIR/c_demo_app
RPC_LIB_DIR=$ESP_ROOT/../components/control_lib

BUILD_DIR=$SCRIPT_DIR/build
BIN_DIR=$SCRIPT_DIR/bin

# Clean previous builds
rm -rf "$BUILD_DIR" "$BIN_DIR"
mkdir -p "$BUILD_DIR" "$BIN_DIR"

# ---------------------------
# Build RPC library
# ---------------------------
#build_esp_hosted_rpc_lib() {
#    log_enter
#    if [ ! -d "$RPC_LIB_DIR" ]; then
#        log "ERROR: RPC library folder not found: $RPC_LIB_DIR"
#        exit 1
#    fi
#
#    cd "$RPC_LIB_DIR"
#    [ -d build ] && rm -rf build
#    mkdir build
#    cd build
#
#    cmake .. -DCMAKE_BUILD_TYPE=Release
#    make -j$(nproc)
#
#    LIB_PATH="$RPC_LIB_DIR/build/libesp_hosted_rpc.a"
#    if [ ! -f "$LIB_PATH" ]; then
#        log "ERROR: Failed to build libesp_hosted_rpc.a, exiting"
#        exit 1
#    fi
#
#    # Copy library to central build folder
#    cp "$LIB_PATH" "$BUILD_DIR/"
#    cd "$SCRIPT_DIR"
#    log_exit
#}

# ---------------------------
# Build C demo applications
# ---------------------------
build_c_demo_app() {
    log_enter

    # Check if replxx library is available
    if [ ! -f /usr/include/replxx.h ] && [ ! -f /usr/local/include/replxx.h ]; then
        warn "replxx library not found. Skipping hosted_shell build."
        warn "Install replxx from https://github.com/AmokHuginnsson/replxx"
    fi

    cd "$C_DEMO_DIR"

    # Clean and build in local BUILD_DIR
    cmake . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    if ! make -C "$BUILD_DIR" -j$(nproc); then
        log "Failed to build C demo apps"
        exit 1
    fi

    # Copy binaries to BIN_DIR
    for bin in "$BUILD_DIR"/*.out "$BUILD_DIR"/hosted_daemon; do
        [ -f "$bin" ] && cp -v "$bin" "$BIN_DIR/"
    done

    cd "$SCRIPT_DIR"
    log_exit
}

# ---------------------------
# Usage / Arguments
# ---------------------------
usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "This script builds the ESP Hosted C user space applications including:"
    echo "  - ESP Hosted RPC library (libesp_hosted_rpc.a)"
    echo "  - C demo applications"
    echo "  - hosted_shell (if replxx library is available)"
    echo ""
    echo "Options:"
    echo "  --help, -h                   Show this help message"
    echo ""
}

parse_arguments() {
    while [ "$1" != "" ]; do
        case $1 in
            --help | -h )
                usage
                exit 0
                ;;
            *)
                log "$1 : unknown option"
                usage
                exit 1
                ;;
        esac
        shift
    done
}

# ---------------------------
# Script main
# ---------------------------
parse_arguments "$@"

log "Building ESP Hosted C user space applications"
log ""

#build_esp_hosted_rpc_lib
build_c_demo_app

log "--- C user space applications build finished ---"
log ""
log "Built applications:"
log "  - ESP Hosted RPC library: $BUILD_DIR/libesp_hosted_rpc.a"
log "  - C demo apps in: $BIN_DIR"
log ""
log "You can now run the C demo applications directly from '$BIN_DIR'."
log ""
log "Run one demo C app from:"
log " [1] Recommended app - hosted shell:"
log "  sudo $BIN_DIR/hosted_shell.out"
log " [2] Minimal app:"
log "  sudo $BIN_DIR/test.out"
log " [3] Daemon app:"
log "  sudo $BIN_DIR/hosted_daemon.out"
log "-----------------------------------"
