# SPDX-License-Identifier: Apache-2.0
# ESP-IDF port-type STATIC. Sources gated by ESP_PLATFORM.

if(NOT CONFIG_ESP_HOSTED_HOST_PORT_TYPE_ESP_IDF)
    return()
endif()

set(EH_HOST_PORT_IDF_SRCS
    "src/eh_host_port_dma.c"
    "src/eh_host_port_sync.c"
    "src/eh_host_port_queue.c"
    "src/eh_host_port_task.c"
    "src/eh_host_port_gpio.c"
    "src/eh_host_port_power.c"
    "src/eh_host_port_transport_defaults.c"
    "src/eh_host_port_transport_config.c"
)

if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO)
    list(APPEND EH_HOST_PORT_IDF_SRCS "src/eh_host_port_sdio.c")
endif()

if(CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD_READY)
    list(APPEND EH_HOST_PORT_IDF_SRCS "src/eh_host_port_openthread.c")
endif()

add_library(eh_host_port_idf STATIC ${EH_HOST_PORT_IDF_SRCS})

target_include_directories(eh_host_port_idf
    PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/../include"
    PRIVATE "src"
)

target_link_libraries(eh_host_port_idf
    PUBLIC
        eh_host_port_api
        eh_mempool
    PRIVATE
        eh_common
        eh_frame
        idf::freertos
        idf::esp_timer
        idf::log
        idf::esp_event
        idf::esp_system
        idf::esp_hw_support
        idf::driver
        idf::esp_driver_gpio
        idf::esp_netif
)

# idf::bt optional — only present when CONFIG_BT_ENABLED is set
if(TARGET idf::bt)
    target_link_libraries(eh_host_port_idf PRIVATE idf::bt)
endif()

if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO)
    target_link_libraries(eh_host_port_idf PRIVATE
        idf::sdmmc
        idf::esp_driver_sdmmc)
endif()

# UART -> hal/uart_types.h lived 'hal' for idf < 6.x, later comes through esp_driver_uart
if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART)
    target_link_libraries(eh_host_port_idf PRIVATE idf::esp_driver_uart)
    if(TARGET idf::esp_hal_uart)
        target_link_libraries(eh_host_port_idf PRIVATE idf::esp_hal_uart)
    endif()
endif()

if(CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD_READY AND TARGET idf::esp_driver_uart)
    target_link_libraries(eh_host_port_idf PRIVATE idf::esp_driver_uart)
endif()
