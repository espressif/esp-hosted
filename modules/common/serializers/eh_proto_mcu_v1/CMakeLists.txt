# SPDX-License-Identifier: Apache-2.0
# eh_proto_mcu_v1 — Rpc protobuf (MCU V1)
# Moved from: components/coprocessor/features/eh_cp_feat_rpc_ext_mcu_pbuf/


set(srcs "src/eh_proto_mcu_v1_stub.c")
if(CONFIG_ESP_HOSTED_CP_FOR_MCU)
    list(APPEND srcs
        "src/eh_rpc.pb-c.c"
        "src/eh_rpc_id_map_v1_mcu_asserts.c"
    )
endif()

add_library(eh_proto_mcu_v1 STATIC ${srcs})

target_include_directories(eh_proto_mcu_v1
    PUBLIC
        "include"
)

target_link_libraries(eh_proto_mcu_v1
    PUBLIC
        eh_common
        msg_codec
)
