if(NOT CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_READY)
    return()
endif()

set(srcs
    "src/eh_cp_feat_host_ps.c"
    "src/eh_cp_feat_host_ps_internal.c"
    "src/eh_cp_feat_host_ps_worker.c"
)

add_library(eh_cp_feat_host_ps STATIC ${srcs})
target_include_directories(eh_cp_feat_host_ps
    PUBLIC
        "include"
)
target_link_libraries(eh_cp_feat_host_ps
    PUBLIC
        eh_cp_core
        eh_frame
        idf::esp_common
        idf::esp_timer
)
if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
    target_link_libraries(eh_cp_feat_host_ps PUBLIC eh_cp_feat_wifi)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY)
    target_link_libraries(eh_cp_feat_host_ps PUBLIC eh_cp_feat_nw_split)
endif()
if(CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_AUTO_INIT)
    target_link_options(eh_cp_feat_host_ps INTERFACE
    "-Wl,--undefined=eh_cp_feat_host_ps_cp__init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_host_ps)
endif()
