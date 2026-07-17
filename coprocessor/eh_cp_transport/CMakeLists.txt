
set(common_shared_inc "${CMAKE_CURRENT_LIST_DIR}/../../common/eh_common/include")
set(cp_core_inc "${CMAKE_CURRENT_LIST_DIR}/../eh_cp_core/include")

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

# CP SDIO build constraint
# ------------------------
# - SDIO SW_AGGR requires the 4092 B send cap to be lifted.
# - idf `master` already includes the fix; older versions need `eh.py patch-idf`.
# - Build-time check ensures we do not fail at runtime, by using incorrect SDIO config
if(CONFIG_EH_TRANSPORT_CP_SDIO AND CONFIG_EH_TRANSPORT_CP_SDIO_MODE_SW_AGGR)
    add_custom_target(eh_sdio_cap_check ALL
        COMMAND ${CMAKE_COMMAND}
                -D "IDF_PATH=${IDF_PATH}"
                -D "EH_PY=${EH_PATH}/tools/eh.py"
                -P "${CMAKE_CURRENT_LIST_DIR}/cmake/check_sdio_cap.cmake"
        VERBATIM)
    add_dependencies(eh_cp_transport eh_sdio_cap_check)
endif()

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

if(TARGET eh_cp)
    target_link_libraries(eh_cp INTERFACE eh_cp_transport)
endif()
