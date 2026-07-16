/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * msg_codec.h — Public abstraction header for the message codec layer.
 *
 * DESIGN RULE:
 *   No component in this codebase should ever directly #include
 *   <protobuf-c/protobuf-c.h>.  All protobuf-c consumers MUST go through
 *   this header.  msg_codec is the single owner of the protobuf-c vendored
 *   library and is responsible for version management and portability.
 *
 * Usage (in any component that needs protobuf-c types/macros):
 *   #include "msg_codec.h"
 *
 * Note on generated pb-c files:
 *   Files produced by protoc-gen-c (*.pb-c.h / *.pb-c.c) include
 *   <protobuf-c/protobuf-c.h> via this header's re-export below.
 *   That path resolves because msg_codec exports "protobuf-c/" as a PUBLIC
 *   include directory.  No other component should vendor or re-export
 *   protobuf-c headers independently.
 */

#pragma once

/* Re-export the full protobuf-c runtime through the msg_codec abstraction. */
#include <protobuf-c/protobuf-c.h>
