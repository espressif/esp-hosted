# SPDX-License-Identifier: Apache-2.0
# eh_proto_v1 — CtrlMsg protobuf (Linux FG V1)

set(srcs "src/eh_proto_v1_stub.c")
if(CONFIG_ESP_HOSTED_CP_FOR_LINUX)
    list(APPEND srcs
        "src/gen_v1.c"
        "src/eh_rpc_id_map_v1_asserts.c"
    )
endif()

add_library(eh_proto_v1 STATIC ${srcs})

target_include_directories(eh_proto_v1
    PUBLIC
        "include"
)

target_link_libraries(eh_proto_v1
    PUBLIC
        eh_common
        msg_codec
)
