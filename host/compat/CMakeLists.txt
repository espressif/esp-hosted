# SPDX-License-Identifier: Apache-2.0
# Upstream MCU back-compat surface; esp_hosted_* sub-headers are
# static inline redirects, 5 orchestration APIs are real symbols here.

# eh_host_compat_types: INTERFACE publishing the esp_hosted_*.h headers.
if(NOT TARGET eh_host_compat_types)
    add_library(eh_host_compat_types INTERFACE)
    target_include_directories(eh_host_compat_types
        INTERFACE "${CMAKE_CURRENT_LIST_DIR}/include")
endif()

add_library(esp_hosted_compat STATIC
    src/esp_hosted_compat.c
)

target_link_libraries(esp_hosted_compat
    PUBLIC
        eh_host_core
        eh_host_compat_types
        eh_host_feat_rpc_ext_v2
    PRIVATE
        eh_host_config
)

# Feature deps — link only when the feature target exists.
foreach(_feat bt system cp_ext_coex gpio_exp ota heartbeat peer_data
              mem_monitor nw_split power_save openthread
              wifi wifi_ext_dpp wifi_ext_ent wifi_ext_itwt)
    if(TARGET eh_host_feat_${_feat})
        target_link_libraries(esp_hosted_compat PUBLIC eh_host_feat_${_feat})
    endif()
endforeach()

