cmake_minimum_required(VERSION 3.16)

if(NOT CMAKE_SCRIPT_MODE_FILE AND CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR
        "Do not run cmake/idf.py from the repo root.\n"
        "Build from examples/<feature>/<scenario>/{cp,mcu_host,esp_host}.")
endif()

set(EH_PATH "${CMAKE_CURRENT_LIST_DIR}")

if(NOT ESP_PLATFORM)
    set(_eh_cfg "${CMAKE_BINARY_DIR}/config")
    if(NOT EXISTS "${_eh_cfg}/sdkconfig.cmake")
        message(FATAL_ERROR
            "${_eh_cfg}/sdkconfig.cmake is missing.\n"
            "hosted_project.cmake runs first-pass kconfgen at "
            "`project()` time before this file is processed; if you "
            "are reaching this file from another driver, that driver "
            "must emit sdkconfig.cmake at ${_eh_cfg}/ first.\n"
            "No fallback to other build dirs is performed by design.")
    endif()
    include("${_eh_cfg}/sdkconfig.cmake")
endif()

# Public `esp_hosted` surface (IDF component + Linux INTERFACE wrap).
if(ESP_PLATFORM)
    file(GLOB _eh_feat_incs LIST_DIRECTORIES true RELATIVE "${EH_PATH}"
        "${EH_PATH}/host/features/eh_host_feat_*/include")
    # BT adapter header (examples/common_components); exposed on the esp_hosted
    # surface so an example's main sees esp_hosted_bt_stack.h via REQUIRES esp_hosted.
    list(APPEND _eh_feat_incs "examples/common_components/esp_hosted_bt/include")

    # ldgen places the feature-descriptor section into a declared region (no
    # "undeclared region" linker warning). Host and CP each map their own section.
    set(_eh_ldfragments "")
    if(CONFIG_ESP_HOSTED_HOST)
        list(APPEND _eh_ldfragments "host/eh_host_feat_descs.lf")
    elseif(CONFIG_ESP_HOSTED_CP)
        list(APPEND _eh_ldfragments "coprocessor/eh_cp_feat_descs.lf")
    endif()

    # Pull in IDF's bt component only when a built-in BT host stack is enabled.
    set(_eh_bt_port_reqs "")
    if(CONFIG_BT_NIMBLE_ENABLED OR CONFIG_BT_BLUEDROID_ENABLED)
        list(APPEND _eh_bt_port_reqs bt)
    endif()

    idf_component_register(NAME esp_hosted
        INCLUDE_DIRS host/compat/include ${_eh_feat_incs}
        REQUIRES esp_event esp_timer log esp_common esp_wifi
        PRIV_REQUIRES driver esp_driver_gpio esp_driver_spi esp_driver_sdmmc esp_driver_uart
                      esp_pm nvs_flash
                      protocomm  # eh_cp_feat_rpc when CP framework is built
                      ${_eh_bt_port_reqs}
        LDFRAGMENTS ${_eh_ldfragments})

    # eh_host  — HOST-side aggregator; every host/ subdir self-attaches.
    # eh_cp    — CP-side aggregator; every coprocessor/ subdir self-attaches.
    if(CONFIG_ESP_HOSTED_HOST)
        if(NOT TARGET eh_host)
            add_library(eh_host INTERFACE)
        endif()
        target_link_libraries(${COMPONENT_LIB} INTERFACE eh_host)
    endif()
    if(CONFIG_ESP_HOSTED_CP)
        if(NOT TARGET eh_cp)
            add_library(eh_cp INTERFACE)
        endif()
        target_link_libraries(${COMPONENT_LIB} INTERFACE eh_cp)
    endif()

    if(CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY)
        idf_build_set_property(LINK_OPTIONS "-Wl,--wrap=esp_wifi_connect" APPEND)
    endif()
else()
    if(CONFIG_ESP_HOSTED_HOST AND NOT TARGET eh_host)
        add_library(eh_host INTERFACE)
    endif()
    if(CONFIG_ESP_HOSTED_CP AND NOT TARGET eh_cp)
        add_library(eh_cp INTERFACE)
    endif()

    if(NOT TARGET esp_hosted)
        add_library(esp_hosted INTERFACE)
        if(TARGET eh_host)
            target_link_libraries(esp_hosted INTERFACE eh_host)
        endif()
        if(TARGET eh_cp)
            target_link_libraries(esp_hosted INTERFACE eh_cp)
        endif()
    endif()

endif()

if(CONFIG_ESP_HOSTED)
    add_subdirectory("${EH_PATH}/common"       ${CMAKE_BINARY_DIR}/common)
endif()

if(CONFIG_ESP_HOSTED_HOST)
    add_subdirectory("${EH_PATH}/host/eh_host_config"
                     ${CMAKE_BINARY_DIR}/eh_host_config)
endif()
if(CONFIG_ESP_HOSTED_CP)
    add_subdirectory("${EH_PATH}/coprocessor"  ${CMAKE_BINARY_DIR}/coprocessor)
endif()
if(CONFIG_ESP_HOSTED_HOST)
    add_subdirectory("${EH_PATH}/host"         ${CMAKE_BINARY_DIR}/host)
endif()
# BT stack adapter (examples/common_components) — a gated internal lib of the
# esp_hosted component; wired here (parent→child) so an example only enables the
# BT feature and picks its stack via the IDF BT Kconfig. feat_bt (added by host/
# above) is available for the down-link.
if(CONFIG_ESP_HOSTED_HOST_FEAT_BT)
    add_subdirectory("${EH_PATH}/examples/common_components/esp_hosted_bt"
                     ${CMAKE_BINARY_DIR}/esp_hosted_bt)
endif()
add_subdirectory("${EH_PATH}/port"             ${CMAKE_BINARY_DIR}/port)

if(CONFIG_ESP_HOSTED_HOST)
    if(CONFIG_ESP_HOSTED_HOST_PORT_TYPE_ESP_IDF)
        add_subdirectory("${EH_PATH}/port/os/idf"
                         ${CMAKE_BINARY_DIR}/port_os_idf)
        target_link_libraries(eh_host INTERFACE eh_host_port_idf)
    elseif(CONFIG_ESP_HOSTED_HOST_PORT_TYPE_POSIX)
        add_subdirectory("${EH_PATH}/port/os/posix"
                         ${CMAKE_BINARY_DIR}/port_os_posix)
        target_link_libraries(eh_host INTERFACE eh_host_port_posix)
    endif()

    if(NOT ESP_PLATFORM AND TARGET esp_event)
        target_link_libraries(eh_host INTERFACE esp_event)
    endif()
endif()

if(ESP_PLATFORM)
    include("${EH_PATH}/tools/cmake/eh_ci_pedantic.cmake")
endif()
