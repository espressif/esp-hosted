if(CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_READY)
    add_library(eh_cp_feat_debug STATIC
        "src/eh_cp_feat_debug.c"
        "src/eh_cp_feat_debug_heap_stats.c")
    target_include_directories(eh_cp_feat_debug PUBLIC "include")
    target_link_libraries(eh_cp_feat_debug PUBLIC eh_cp_core eh_common idf::esp_common idf::heap)

    if(CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_AUTO_INIT)
        target_link_options(eh_cp_feat_debug INTERFACE
        "-Wl,--undefined=eh_cp_feat_debug_init")
    endif()
else()
    add_library(eh_cp_feat_debug INTERFACE)
    target_include_directories(eh_cp_feat_debug INTERFACE "include")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_debug)
endif()
