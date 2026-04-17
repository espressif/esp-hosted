# SPDX-License-Identifier: Apache-2.0
# eh_proto_linux_v1 — CtrlMsg protobuf (Linux FG V1)
# Moved from: components/coprocessor/features/eh_cp_feat_rpc_ext_linux_pbuf/


set(srcs "src/eh_proto_linux_v1_stub.c")
if(CONFIG_ESP_HOSTED_CP_FOR_LINUX)
    list(APPEND srcs
        "src/eh_config.pb-c.c"
        "src/eh_rpc_id_map_v1_fg_asserts.c"
    )
endif()

add_library(eh_proto_linux_v1 STATIC ${srcs})

target_include_directories(eh_proto_linux_v1
    PUBLIC
        "include"
)

target_link_libraries(eh_proto_linux_v1
    PUBLIC
        eh_common
        msg_codec
)
