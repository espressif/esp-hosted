# SPDX-License-Identifier: Apache-2.0

if(NOT CONFIG_ESP_HOSTED_HOST_FEAT_MEM_MONITOR_READY)
    return()
endif()

add_library(eh_host_feat_mem_monitor STATIC
    "src/eh_host_feat_mem_monitor.c"
)

target_include_directories(eh_host_feat_mem_monitor
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_feat_mem_monitor
    PUBLIC
        eh_host_feat_rpc
        eh_host_port_api
        eh_common
    PRIVATE
        eh_host_config
)

if(CONFIG_ESP_HOSTED_HOST_FEAT_MEM_MONITOR_AUTO_INIT)
    target_link_options(eh_host_feat_mem_monitor INTERFACE
        "-Wl,--undefined=eh_host_feat_mem_monitor_init")
endif()


target_link_libraries(eh_host_feat_mem_monitor PRIVATE eh_host_autoinit)

target_sources(eh_host_feat_mem_monitor PRIVATE
    "src/eh_host_mem_monitor.c"
    "src/eh_host_feat_mem_monitor_v2_req.c"
    "src/eh_host_feat_mem_monitor_v2_resp_evt.c"
)
target_link_libraries(eh_host_feat_mem_monitor PRIVATE
    eh_host_feat_rpc_ext_v2
    eh_host_feat_rpc
    eh_proto_v2
)
