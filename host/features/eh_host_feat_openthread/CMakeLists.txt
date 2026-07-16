# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD_READY)
    return()
endif()

add_library(eh_host_feat_openthread STATIC
    "src/eh_host_feat_openthread.c"
)

target_include_directories(eh_host_feat_openthread
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_openthread
    PUBLIC
        eh_host_port_api
        eh_common
)

if(CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD_AUTO_INIT)
    target_link_options(eh_host_feat_openthread INTERFACE
        "-Wl,--undefined=eh_host_feat_openthread_init")
endif()

target_link_libraries(eh_host_feat_openthread PRIVATE eh_host_autoinit)

target_sources(eh_host_feat_openthread PRIVATE
    "src/eh_host_openthread.c"
)
target_link_libraries(eh_host_feat_openthread PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)
