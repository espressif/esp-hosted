# ESP Hosted Transport Host Component

# Common directory reference (same pattern as coprocessor transport)
set(common_dir "../../../common")

# Transport coprocessor common headers (shared between host and coprocessor)
set(transport_cp_common "${CMAKE_CURRENT_LIST_DIR}/../../../coprocessor/components/eh_cp_transport/include/common")

# Host platform detection and source selection
set(HOST_TRANSPORT_SRCS)
set(HOST_TRANSPORT_INCLUDES)
set(HOST_TRANSPORT_PRIV_INCLUDES "private_include")
set(HOST_TRANSPORT_REQUIRES)

# Core agnostic transport layer (always included)
set(CORE_TRANSPORT_SRCS
    "src/eh_host_transport.c"
)
list(APPEND HOST_TRANSPORT_SRCS ${CORE_TRANSPORT_SRCS})

# MCU platform sources (ESP-IDF host) - CMake component system
if(CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU)
    message(STATUS "ESP Hosted Transport Host: Building for MCU platform")

    set(MCU_TRANSPORT_SRCS
        "src/mcu/transport_drv.c"
        "src/mcu/eh_host_transport_mcu_impl.c"
        "src/adapters/mcu_adapter.c"
    )

    # Conditionally include only the selected transport driver (FIX-008)
    if(CONFIG_ESP_HOSTED_HOST_MCU_TRANSPORT_SPI)
        list(APPEND MCU_TRANSPORT_SRCS "src/mcu/spi/spi_drv.c")
    elseif(CONFIG_ESP_HOSTED_HOST_MCU_TRANSPORT_SDIO)
        list(APPEND MCU_TRANSPORT_SRCS "src/mcu/sdio/sdio_drv.c")
    elseif(CONFIG_ESP_HOSTED_HOST_MCU_TRANSPORT_SPI_HD)
        list(APPEND MCU_TRANSPORT_SRCS "src/mcu/spi_hd/spi_hd_drv.c")
    elseif(CONFIG_ESP_HOSTED_HOST_MCU_TRANSPORT_UART)
        list(APPEND MCU_TRANSPORT_SRCS "src/mcu/uart/uart_drv.c")
    else()
        # Default: SPI (matches Kconfig default)
        list(APPEND MCU_TRANSPORT_SRCS "src/mcu/spi/spi_drv.c")
    endif()

    list(APPEND HOST_TRANSPORT_SRCS ${MCU_TRANSPORT_SRCS})
    list(APPEND HOST_TRANSPORT_INCLUDES "src/mcu")
    list(APPEND HOST_TRANSPORT_REQUIRES "esp_common" "driver" "freertos")
endif()

# Linux kernel sources handled via Makefile (standard kernel build)
# Note: Linux kernel modules MUST use Makefiles - CMake is not supported
# The Makefile in eh_fg/host/linux/kmod/ references this component's headers

# Port header directory (IDF build: CONFIG_XXX → H_XXX bridge)
# port/host/idf/include/port_eh_host_config.h is included by many
# host transport source files via #include "port_eh_host_config.h"
set(port_host_idf_include "${CMAKE_CURRENT_LIST_DIR}/../../../../../port/host/idf/include")

# Canonical common headers (components/common/eh_common/include/)
set(common_shared_inc "${CMAKE_CURRENT_LIST_DIR}/../../../../../components/common/eh_common/include")

idf_component_register(
    SRCS ${HOST_TRANSPORT_SRCS}
    INCLUDE_DIRS
        "include"
        "${common_dir}"                 # Main common headers (eh_header.h, etc.)
        "${transport_cp_common}"        # Shared transport headers from CP component
        "${port_host_idf_include}"      # H_XXX config abstraction (port/host/idf/include/)
        "${common_shared_inc}"          # Shared TLV + caps common headers
        ${HOST_TRANSPORT_INCLUDES}
    PRIV_INCLUDE_DIRS ${HOST_TRANSPORT_PRIV_INCLUDES}
    REQUIRES ${HOST_TRANSPORT_REQUIRES}
)

# Future: When eh_transport becomes separate repo:
# set(common_dir "common")  # Direct common folder in transport repo