# eh_cp_feat_gpio_exp — GPIO Expander feature

if(CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_READY)
    add_library(eh_cp_feat_gpio_exp STATIC
        "src/eh_cp_feat_gpio_exp.c"
    )
    target_include_directories(eh_cp_feat_gpio_exp
        PUBLIC "include"
    )
    target_link_libraries(eh_cp_feat_gpio_exp
        PUBLIC eh_cp_core eh_common idf::esp_common idf::esp_driver_gpio
    )
else()
    add_library(eh_cp_feat_gpio_exp INTERFACE)
    target_include_directories(eh_cp_feat_gpio_exp INTERFACE "include")
endif()
