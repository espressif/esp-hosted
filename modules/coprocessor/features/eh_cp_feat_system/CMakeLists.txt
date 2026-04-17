# eh_cp_feat_system — System feature extension

if(CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY)
    set(srcs
        "src/eh_cp_feat_system.c"
    )

    add_library(eh_cp_feat_system STATIC ${srcs})
    target_include_directories(eh_cp_feat_system
        PUBLIC
            "include"
    )
    target_link_libraries(eh_cp_feat_system
        PUBLIC
            eh_cp_core
            eh_common
            idf::esp_common
            idf::esp_event
            idf::freertos
    )
else()
    add_library(eh_cp_feat_system INTERFACE)
    target_include_directories(eh_cp_feat_system
        INTERFACE
            "include"
    )
endif()
