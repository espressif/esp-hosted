# eh_cp_transport — CP physical bus drivers (SPI/SDIO/UART/SPI-HD)

# Legacy shim headers (eh_tlv.h, eh_log.h, etc.)
set(common_shared_inc "${CMAKE_CURRENT_LIST_DIR}/../../../modules/common/eh_common/include")
set(cp_core_inc "${CMAKE_CURRENT_LIST_DIR}/../eh_cp_core/include")

# ── Transport sources (one selected per Kconfig) ──────────────────────────
set(CP_TRANSPORT_SRCS src/eh_cp_transport_gpio_pin_guard.c)
if(CONFIG_EH_TRANSPORT_CP_SDIO)
    list(APPEND CP_TRANSPORT_SRCS src/eh_cp_transport_sdio.c)
elseif(CONFIG_EH_TRANSPORT_CP_SPI)
    list(APPEND CP_TRANSPORT_SRCS src/eh_cp_transport_spi.c)
elseif(CONFIG_EH_TRANSPORT_CP_SPI_HD)
    list(APPEND CP_TRANSPORT_SRCS src/eh_cp_transport_spi_hd.c)
else()
    list(APPEND CP_TRANSPORT_SRCS src/eh_cp_transport_uart.c)
endif()

# ── IDF driver dependencies (IDF 5.3+ split drivers) ──────────────────────
set(public_requires)
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    list(APPEND public_requires
        idf::esp_driver_sdio
        idf::esp_driver_spi
        idf::esp_driver_uart
        idf::esp_driver_gpio)
else()
    list(APPEND public_requires idf::driver)
endif()

add_library(eh_cp_transport STATIC ${CP_TRANSPORT_SRCS})

target_include_directories(eh_cp_transport
    PUBLIC
        "include"
        "include/common"
        "${common_shared_inc}"
        "${cp_core_inc}"
    PRIVATE
        "private_include"
)

target_link_libraries(eh_cp_transport
    PUBLIC
        idf::esp_timer
        idf::esp_common
        eh_common
        eh_frame
        eh_mempool
        eh_tlv
        ${public_requires}
)
