# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(eh_proto_linux_v1 C)

# Create the library target with -fPIC for shared library compatibility
set(PROTOBUF_C_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../modules/common/serializers/third_party/msg_codec/protobuf-c)

add_library(ehrcp_linux_protobuf STATIC
    src/esp_hosted_config.pb-c.c
    ${PROTOBUF_C_DIR}/protobuf-c/protobuf-c.c
)

target_include_directories(ehrcp_linux_protobuf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROTOBUF_C_DIR}
    ${PROTOBUF_C_DIR}/protobuf-c
)

# Enable position-independent code for shared library compatibility
set_target_properties(ehrcp_linux_protobuf PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
