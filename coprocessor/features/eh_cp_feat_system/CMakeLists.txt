if(NOT CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY)
    return()
endif()

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
if(CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_AUTO_INIT)
    target_link_options(eh_cp_feat_system INTERFACE
    "-Wl,--undefined=eh_cp_feat_system_init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_system)
endif()
