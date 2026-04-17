# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(eh_proto_linux_v1 C)

# Create the library target with -fPIC for shared library compatibility
add_library(ehrcp_linux_protobuf STATIC
    src/eh_config.pb-c.c
    src/protobuf-c/protobuf-c/protobuf-c.c
)

target_include_directories(ehrcp_linux_protobuf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/protobuf-c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/protobuf-c/protobuf-c
)

# Enable position-independent code for shared library compatibility
set_target_properties(ehrcp_linux_protobuf PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
