# SPDX-License-Identifier: Apache-2.0
if(CONFIG_EH_USE_LEGACY_API)
    add_executable(wifi_apsta_host_c_app legacy/main.c)
    target_link_libraries(wifi_apsta_host_c_app PRIVATE esp_hosted)
else()
    add_executable(wifi_apsta_host_c_app main.c)
    target_link_libraries(wifi_apsta_host_c_app PRIVATE esp_hosted)
endif()
