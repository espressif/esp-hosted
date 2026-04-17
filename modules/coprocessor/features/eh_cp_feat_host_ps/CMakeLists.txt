# eh_cp_feat_host_ps — host power-save extension

if(CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_READY)
    set(srcs
        "src/eh_cp_feat_host_ps.c"
        "src/eh_cp_feat_host_ps_internal.c"
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
else()
    add_library(eh_cp_feat_host_ps INTERFACE)
    target_include_directories(eh_cp_feat_host_ps
        INTERFACE
            "include"
    )
endif()
