if(NOT CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX_READY)
    return()
endif()

add_library(eh_cp_feat_cp_ext_coex STATIC "src/eh_cp_feat_cp_ext_coex.c")
target_include_directories(eh_cp_feat_cp_ext_coex PUBLIC "include")
target_link_libraries(eh_cp_feat_cp_ext_coex PUBLIC eh_cp_core eh_common idf::esp_common)
if(CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX_AUTO_INIT)
    target_link_options(eh_cp_feat_cp_ext_coex INTERFACE
    "-Wl,--undefined=eh_cp_feat_cp_ext_coex_init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_cp_ext_coex)
endif()
