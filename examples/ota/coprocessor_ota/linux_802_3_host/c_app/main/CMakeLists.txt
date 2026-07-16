# SPDX-License-Identifier: Apache-2.0
if(CONFIG_EH_USE_LEGACY_API)
    add_executable(ota_upgrade_c_app legacy/main.c)
else()
    add_executable(ota_upgrade_c_app main.c)
endif()
target_link_libraries(ota_upgrade_c_app PRIVATE esp_hosted)
# main.c streams the OTA image over HTTP via libcurl on Linux (vs
# esp_http_client on IDF).  Pull libcurl from the system if present.
find_package(CURL QUIET)
if(CURL_FOUND)
    target_link_libraries(ota_upgrade_c_app PRIVATE CURL::libcurl)
endif()
