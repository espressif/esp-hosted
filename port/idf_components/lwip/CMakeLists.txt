# eh_host_extra/lwip — lwIP port glue (NOT a full lwIP vendored copy)
# for non-IDF hosts (per S4 §3b matrix).
# SPDX-License-Identifier: Apache-2.0
#
# Per-port enablement:
#   - esp_idf port: real IDF supplies its esp-lwip variant natively
#     (early return).
#   - linux_user port: Linux kernel TCP/IP supersedes; no lwIP
#     (early return).
#   - stm32 / zephyr ports: port glue here; lwIP CORE comes from a
#     separate source (system liblwip, or an external git submodule).
#
# Vendoring policy: VERBATIM copy of upstream
# `host/extra_components/lwip/`.  See README.md for the periodic-sync
# checklist.  The upstream CMakeLists.txt references `lwip/src/...`
# paths that live outside this dir (the lwIP core git submodule);
# the source list below restricts itself to files actually vendored
# here (port glue + apps).  Adaptation is a follow-up phase; do NOT
# edit vendored .c/.h here.

# 1. esp_idf port: real IDF builds esp-lwip as a normal IDF component.
if(ESP_PLATFORM)
    return()
endif()

# 2. Bare-metal MCU hosts only.
if(NOT (CONFIG_ESP_HOSTED_HOST_PORT_TYPE_STM32 OR CONFIG_ESP_HOSTED_HOST_PORT_TYPE_ZEPHYR))
    return()
endif()

# 3. STM32 / Zephyr build: assemble a STATIC library from the vendored
#    port glue + apps.  Mirrors upstream CMakeLists.txt's source list,
#    minus the lwIP-CORE files (those come from an out-of-tree submodule).
if(CONFIG_ESP_HOSTED_HOST_PORT_TYPE_ZEPHYR)
    set(_eh_lwip_target_dir "linux")  # Zephyr POSIX subset
else()
    set(_eh_lwip_target_dir "esp32xx") # STM32 reuses the esp32xx port glue
endif()

set(_eh_lwip_srcs
    "apps/esp_md5/esp_md5.c"
    "apps/sntp/sntp.c"
    "port/hooks/tcp_isn_default.c"
    "port/hooks/lwip_default_hooks.c"
    "port/debug/lwip_debug.c"
    "port/sockets_ext.c"
    "port/freertos/sys_arch.c"
)

if(CONFIG_LWIP_ICMP)
    list(APPEND _eh_lwip_srcs
        "apps/ping/esp_ping.c"
        "apps/ping/ping.c"
        "apps/ping/ping_sock.c"
    )
endif()

if(CONFIG_LWIP_DHCPS)
    list(APPEND _eh_lwip_srcs "apps/dhcpserver/dhcpserver.c")
endif()

if(CONFIG_LWIP_DHCP_RESTORE_LAST_IP)
    list(APPEND _eh_lwip_srcs "port/esp32xx/netif/dhcp_state.c")
endif()

if(CONFIG_VFS_SUPPORT_IO)
    list(APPEND _eh_lwip_srcs "port/${_eh_lwip_target_dir}/vfs_lwip.c")
elseif("${_eh_lwip_target_dir}" STREQUAL "esp32xx")
    list(APPEND _eh_lwip_srcs "port/esp32xx/no_vfs_syscalls.c")
endif()

add_library(lwip STATIC ${_eh_lwip_srcs})

target_include_directories(lwip
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/include"
        "${CMAKE_CURRENT_LIST_DIR}/include/apps"
        "${CMAKE_CURRENT_LIST_DIR}/apps/esp_md5"
        "${CMAKE_CURRENT_LIST_DIR}/include/apps/sntp"
        "${CMAKE_CURRENT_LIST_DIR}/port/include"
        "${CMAKE_CURRENT_LIST_DIR}/port/freertos/include"
        "${CMAKE_CURRENT_LIST_DIR}/port/${_eh_lwip_target_dir}/include"
        "${CMAKE_CURRENT_LIST_DIR}/port/${_eh_lwip_target_dir}/include/arch"
        "${CMAKE_CURRENT_LIST_DIR}/port/${_eh_lwip_target_dir}/include/sys"
)

target_compile_options(lwip PRIVATE -Wno-address -Wno-format)
target_compile_definitions(lwip PRIVATE ESP_LWIP_COMPONENT_BUILD)

# lwip's own .c files include "sdkconfig.h" — declare the dependency
# explicitly via target_link_libraries so the include path reaches
# its compilation.
target_link_libraries(lwip PUBLIC eh_host_config)
