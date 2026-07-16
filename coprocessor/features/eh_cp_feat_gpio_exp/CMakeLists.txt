if(NOT CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_READY)
    return()
endif()

add_library(eh_cp_feat_gpio_exp STATIC
    "src/eh_cp_feat_gpio_exp.c"
)
target_include_directories(eh_cp_feat_gpio_exp
    PUBLIC "include"
)
target_link_libraries(eh_cp_feat_gpio_exp
    PUBLIC eh_cp_core eh_common idf::esp_common idf::esp_driver_gpio
)
if(CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_AUTO_INIT)
    target_link_options(eh_cp_feat_gpio_exp INTERFACE
    "-Wl,--undefined=eh_cp_feat_gpio_exp_init")
endif()

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_feat_gpio_exp)
endif()
