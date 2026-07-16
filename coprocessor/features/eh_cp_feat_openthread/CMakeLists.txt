if(NOT CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD_READY)
    return()
endif()

add_library(eh_cp_feat_openthread STATIC
    "src/eh_cp_feat_openthread.c"
)
target_include_directories(eh_cp_feat_openthread PUBLIC "include")
target_link_libraries(eh_cp_feat_openthread PUBLIC
    eh_cp_core
    eh_common
    idf::esp_common
    idf::openthread
    idf::vfs)

if(CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD_AUTO_INIT)
    target_link_options(eh_cp_feat_openthread INTERFACE
        "-Wl,--undefined=eh_cp_feat_openthread_init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_openthread)
endif()
