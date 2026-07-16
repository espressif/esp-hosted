# eh_host_mcu_transport — MCU bus transport runtime
# SPDX-License-Identifier: Apache-2.0
# Exactly one of BUS_{SPI,SDIO,SPI_HD,UART} is selected by Kconfig.

if(NOT CONFIG_ESP_HOSTED_HOST_TRANSPORT_MCU_READY)
    return()
endif()

set(EH_TRANSPORT_SRCS
    "src/eh_host_mcu_transport_init.c"
    "src/eh_host_mcu_transport_channels.c"
    "src/eh_host_mcu_wifi_remote_glue.c"
    "src/eh_host_raw_tp_stats.c"
    "src/eh_host_mcu_transport_state.c"
    "src/eh_host_mcu_transport_init_event.c"
    "src/eh_host_mcu_transport_send_caps.c"
    "src/eh_host_mcu_hci.c"
)

if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_READY)
    list(APPEND EH_TRANSPORT_SRCS "src/eh_host_bus_spi.c")
endif()
if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO_READY)
    list(APPEND EH_TRANSPORT_SRCS "src/eh_host_bus_sdio.c")
endif()
if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD_READY)
    list(APPEND EH_TRANSPORT_SRCS "src/eh_host_bus_spi_hd.c")
endif()
if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART_READY)
    list(APPEND EH_TRANSPORT_SRCS "src/eh_host_bus_uart.c")
endif()

add_library(eh_host_mcu_transport STATIC ${EH_TRANSPORT_SRCS})

target_include_directories(eh_host_mcu_transport
    PUBLIC "include"
    PRIVATE "src"
)

target_link_libraries(eh_host_mcu_transport
    PUBLIC
        eh_common
        eh_frame
        eh_host_port_api
        eh_mempool
        eh_host_config
        eh_tlv
    PRIVATE
        # mcu_transport's RX dispatch calls auto-init walker after caps_tx.
        eh_host_autoinit
)

# IDF component-requirements walker can't evaluate EH_HOST_* gates, so explicitly
# link per-bus peripheral drivers; missing one fails at CMake configure time.
if(ESP_PLATFORM)
    target_link_libraries(eh_host_mcu_transport PRIVATE idf::esp_timer)
    if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_READY)
        target_link_libraries(eh_host_mcu_transport PRIVATE
            idf::esp_driver_spi idf::esp_driver_gpio)
    endif()
    if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD_READY)
        target_link_libraries(eh_host_mcu_transport PRIVATE
            idf::esp_driver_spi idf::esp_driver_gpio)
    endif()
    if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO_READY)
        target_link_libraries(eh_host_mcu_transport PRIVATE
            idf::esp_driver_sdmmc idf::sdmmc idf::esp_driver_gpio)
    endif()
    if(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART_READY)
        target_link_libraries(eh_host_mcu_transport PRIVATE
            idf::esp_driver_uart idf::esp_driver_gpio)
    endif()
endif()
