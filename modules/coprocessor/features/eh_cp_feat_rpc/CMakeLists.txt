# eh_cp_feat_rpc — protocomm / pserial transport extension

add_library(eh_cp_feat_rpc STATIC
    "src/eh_cp_feat_rpc.c"
    "src/eh_cp_feat_rpc_registries.c"
    "src/eh_cp_feat_rpc_ll.c"
    "src/eh_cp_feat_rpc_serial.c"
    #"src/eh_cp_rpc.c"
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
        idf::protocomm
        idf::driver
        idf::esp_timer
        idf::esp_common
)
