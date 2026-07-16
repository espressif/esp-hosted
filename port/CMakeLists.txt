# SPDX-License-Identifier: Apache-2.0
# Port API surface — header-only INTERFACE libs. Sources live in port/<impl>/.

# eh_host_port_api_headers: headers only, cycle-free.
# Safe to link from port/idf_components/* (which port_api bundles on non-IDF).
add_library(eh_host_port_api_headers INTERFACE)
target_include_directories(eh_host_port_api_headers
    INTERFACE "${CMAKE_CURRENT_LIST_DIR}/os/include"
)

# Non-IDF: deliver -include sdkconfig.h via eh_host_config (IDF wires it itself)
if(TARGET eh_host_config)
    target_link_libraries(eh_host_port_api_headers INTERFACE eh_host_config)
endif()

# eh_host_port_api: full bundle (headers + IDF-shape components).
# DO NOT link from port/idf_components/* — use eh_host_port_api_headers to avoid cycle.
add_library(eh_host_port_api INTERFACE)
target_link_libraries(eh_host_port_api INTERFACE eh_host_port_api_headers)

if(ESP_PLATFORM)
    target_link_libraries(eh_host_port_api INTERFACE
        idf::esp_common
        idf::esp_timer
        idf::log
        idf::esp_event
        idf::esp_wifi
        idf::esp_netif
    )
else()
    target_link_libraries(eh_host_port_api INTERFACE
        utils
        log
        esp_event
        esp_system
        esp_netif
        esp_wifi_remote
        esp_timer
    )
endif()
