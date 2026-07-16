# SPDX-License-Identifier: Apache-2.0
# esp_netif slot. IDF: natively. Linux: kmod-state observer over netlink.

if(ESP_PLATFORM)
    return()
endif()

add_library(esp_netif STATIC
    linux/esp_netif_linux.c
    linux/esp_netif_dhcp_client_linux.c
    linux/esp_netif_dhcp_server_linux.c
    esp_netif_event_base.c
    esp_netif_defaults.c)
target_include_directories(esp_netif
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
target_link_libraries(esp_netif PUBLIC eh_host_port_api_headers esp_event pthread)
# Force retention: IP_EVENT (address-only refs) + DHCP TUs (constructor-only refs)
target_link_options(esp_netif INTERFACE
    "-Wl,--undefined=IP_EVENT"
    "-Wl,--undefined=_esp_netif_dhcpc_linux_linked"
    "-Wl,--undefined=_esp_netif_dhcps_linux_linked")
