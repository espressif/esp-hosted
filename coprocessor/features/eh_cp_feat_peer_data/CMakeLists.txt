# SPDX-License-Identifier: Apache-2.0

add_library(eh_cp_feat_peer_data STATIC
    "src/eh_cp_feat_peer_data.c"
)

target_include_directories(eh_cp_feat_peer_data
    PUBLIC
        "include"
)

target_link_libraries(eh_cp_feat_peer_data
    PUBLIC
        eh_cp_core
)

if(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_AUTO_INIT)
    target_link_options(eh_cp_feat_peer_data INTERFACE
    "-Wl,--undefined=eh_cp_feat_peer_data_init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_peer_data)
endif()
