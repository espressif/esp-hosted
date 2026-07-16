
add_library(eh_cp_feat_rpc STATIC
    "src/eh_cp_feat_rpc.c"
    "src/eh_cp_feat_rpc_registries.c"
    "src/eh_cp_feat_rpc_ll.c"
    "src/eh_cp_feat_rpc_serial.c"
)

target_include_directories(eh_cp_feat_rpc
    PUBLIC
        "include"
    PRIVATE
        "src"
        "$ENV{IDF_PATH}/components/protocomm/src/common"
)

target_link_libraries(eh_cp_feat_rpc
    PUBLIC
        eh_cp_core
        eh_common
        eh_tlv
        idf::protocomm
        idf::driver
        idf::esp_timer
        idf::esp_common
)

if(CONFIG_ESP_HOSTED_CP_FEAT_RPC_AUTO_INIT)
    target_link_options(eh_cp_feat_rpc INTERFACE
    "-Wl,--undefined=eh_cp_feat_rpc_set_endpoints")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_rpc)
endif()
