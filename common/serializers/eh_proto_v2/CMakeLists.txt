# SPDX-License-Identifier: Apache-2.0
# eh_proto_v2 — Rpc protobuf (V2, formerly MCU V1)

set(srcs "")
# Compile the V2 protobuf whenever V2 RPC is in use — on the CP (any CP type
# that selected RPC V2: MCU *or* the new Linux 802.3) or on the host. Keying on
# the V2 feature readiness (not CP_FOR_MCU) is what lets CP_FOR_LINUX_802_3 + V2
# link; the old CP_FOR_MCU gate assumed V2 ⟺ MCU, which is no longer true.
if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V2_READY OR CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2_READY)
    list(APPEND srcs
        "src/gen_v2.c"
        "src/eh_rpc_id_map_v2_asserts.c"
    )
endif()
# Stub TU so add_library doesn't fail when no sources are gated in.
if(NOT srcs)
    set(stub "${CMAKE_CURRENT_BINARY_DIR}/eh_proto_v2_empty.c")
    file(WRITE "${stub}" "/* empty stub */\n")
    list(APPEND srcs "${stub}")
endif()

add_library(eh_proto_v2 STATIC ${srcs})

target_include_directories(eh_proto_v2
    PUBLIC
        "include"
)

target_link_libraries(eh_proto_v2
    PUBLIC
        eh_common
        msg_codec
)
