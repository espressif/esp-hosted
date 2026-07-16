# SPDX-License-Identifier: Apache-2.0
# Wrapper around the esp_wifi_remote submodule (./esp_wifi_remote/).
# Non-IDF: compile submodule sources + wifi_default.c + wifi_event_base.c.
# IDF: early-return (IDF supplies esp_wifi_remote natively).

set(_eh_wifi_remote_sub
    "${CMAKE_CURRENT_LIST_DIR}/esp_wifi_remote/components/esp_wifi_remote")
set(_v "${_eh_wifi_remote_sub}/idf_v6.1")
set(_inj "${_v}/include/injected")

if(ESP_PLATFORM)
    return()
endif()

set(_srcs
    "${_eh_wifi_remote_sub}/esp_wifi_remote.c"
    "${_v}/esp_wifi_with_remote.c"
    "${_v}/esp_wifi_remote_weak.c"
    "${_v}/esp_eap_client_remote_weak.c"
    "src/wifi_default.c"
    "src/wifi_event_base.c"
)

# net.c bridges Wi-Fi → lwip; skip on Linux (kernel owns netdev). Needed on stm32/zephyr.
if(NOT CONFIG_ESP_HOSTED_HOST_PORT_TYPE_POSIX)
    list(APPEND _srcs "${_eh_wifi_remote_sub}/esp_wifi_remote_net.c")
endif()

add_library(esp_wifi_remote STATIC ${_srcs})

# BEFORE: vendored + injected IDF-wifi headers + our esp_eap_client.h stub.
# Wins over compat-tree shims.
target_include_directories(esp_wifi_remote BEFORE PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/include"
    "${_inj}"
    "${_v}/include"
    "${_eh_wifi_remote_sub}/include")

# eh_host_config (not eh_host_port_api) avoids CMake cycle —
# port_api INTERFACE-bundles esp_wifi_remote itself on non-IDF.
target_link_libraries(esp_wifi_remote
    PUBLIC eh_host_config utils log esp_system esp_event esp_netif)

# net.c needs FreeRTOS.h via esp_private/wifi.h + assert.h
if("${_srcs}" MATCHES "esp_wifi_remote_net\\.c")
    if(TARGET freertos_shim)
        target_link_libraries(esp_wifi_remote PRIVATE freertos_shim)
    endif()
    target_compile_options(esp_wifi_remote PRIVATE "SHELL:-include assert.h")
endif()

# Force retention of wifi_event_base.c (WIFI_EVENT referenced only by address)
target_link_options(esp_wifi_remote INTERFACE
    "-Wl,--undefined=WIFI_EVENT")

if(TARGET eh_host)
    target_link_libraries(eh_host INTERFACE esp_wifi_remote)
endif()

# Promote wrapper Kconfig out of "Component config" submenu — its
# "Wi-Fi Remote" sits alongside Hosted's top-level menus.
set_property(TARGET esp_wifi_remote PROPERTY
    HOSTED_KCONFIG_TOPLEVEL "${CMAKE_CURRENT_LIST_DIR}/Kconfig.ext")

# back-compat alias
if(NOT TARGET esp_wifi_remote_port)
    add_library(esp_wifi_remote_port ALIAS esp_wifi_remote)
endif()
