# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP_READY)
    return()
endif()

add_library(eh_host_feat_gpio_exp STATIC
    "src/eh_host_feat_gpio_exp.c"
)

target_include_directories(eh_host_feat_gpio_exp
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_gpio_exp
    PUBLIC
        eh_host_port_api
        eh_common
)

if(CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP_AUTO_INIT)
    target_link_options(eh_host_feat_gpio_exp INTERFACE
        "-Wl,--undefined=eh_host_feat_gpio_exp_init")
endif()


target_link_libraries(eh_host_feat_gpio_exp PRIVATE eh_host_autoinit)

target_sources(eh_host_feat_gpio_exp PRIVATE
    "src/eh_host_cp_gpio.c"
    "src/eh_host_feat_gpio_exp_v2_req.c"
    "src/eh_host_feat_gpio_exp_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_gpio_exp PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)
