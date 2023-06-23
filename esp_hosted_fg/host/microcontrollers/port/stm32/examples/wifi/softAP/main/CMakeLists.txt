set(FG_ROOT_DIR "../../../../../../../..")
set(MCU_DIR "${FG_ROOT_DIR}/host/microcontrollers")

set(COMPONENT_SRCS "softap_example_main.c" "${MCU_DIR}/drivers/transport/transport_drv.c"  "${MCU_DIR}/drivers/network/netdev_api.c" "${MCU_DIR}/drivers/network/netdev_stub.c" "${MCU_DIR}/drivers/serial/serial_ll_if.c" "${MCU_DIR}/utils/common.c" "${MCU_DIR}/utils/util.c" "${MCU_DIR}/utils/stats.c" "../../../../os_wrapper.c" "${MCU_DIR}/drivers/serial/serial_drv.c" "${MCU_DIR}/drivers/netif/wifi_default.c" "${MCU_DIR}/drivers/netif/wifi_netif.c")

set(COMPONENT_ADD_INCLUDEDIRS "." "${MCU_DIR}/drivers/transport" "${MCU_DIR}/drivers/transport/spi" "${MCU_DIR}/drivers/transport/sdio" "${MCU_DIR}/drivers/netif" "${MCU_DIR}/drivers/network" "${MCU_DIR}/drivers/serial" "${MCU_DIR}/utils" "${FG_ROOT_DIR}/common/include" "../../../.." )

#components
# rpc lib files
set(RPC_LIB_DIR "${FG_ROOT_DIR}/host/common/rpc_lib")
set(RPC_SRC_DIR "${FG_ROOT_DIR}/host/common/rpc_lib/src")
list(APPEND COMPONENT_SRCS  "${RPC_SRC_DIR}/rpc_core.c" "${RPC_SRC_DIR}/rpc_req.c" "${RPC_SRC_DIR}/rpc_rsp.c" "${RPC_SRC_DIR}/rpc_evt.c" "${RPC_SRC_DIR}/rpc_api.c" "${FG_ROOT_DIR}/host/common/virtual_serial_if/src/serial_if.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${RPC_SRC_DIR}/include" "${RPC_LIB_DIR}/include" "${FG_ROOT_DIR}/host/common/virtual_serial_if/include")

list(APPEND COMPONENT_SRCS  "${FG_ROOT_DIR}/host/common/rpc_wrapper/rpc_wrapper.c" "${FG_ROOT_DIR}/host/common/rpc_wrapper/wifi/wifi_drv.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS  "${FG_ROOT_DIR}/host/common/rpc_wrapper" "${FG_ROOT_DIR}/host/common/rpc_wrapper/wifi")
# esp_queue
list(APPEND COMPONENT_SRCS "${FG_ROOT_DIR}/host/common/esp_queue/src/esp_queue.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${FG_ROOT_DIR}/host/common/esp_queue/include")

# helpers - mempool
list(APPEND COMPONENT_SRCS "${MCU_DIR}/helpers/mempool/mempool.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${MCU_DIR}/helpers/mempool" )

# protobuf files
list(APPEND COMPONENT_SRCS "${FG_ROOT_DIR}/common/protobuf-c/protobuf-c/protobuf-c.c" "${FG_ROOT_DIR}/common/esp_hosted_config.pb-c.c" )
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${FG_ROOT_DIR}/common/protobuf-c" )

# transport files
if(CONFIG_ESP_SDIO_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS "${MCU_DIR}/drivers/transport/sdio/sdio_ll.c" "${MCU_DIR}/drivers/transport/sdio/sdio_api.c" "${MCU_DIR}/drivers/transport/sdio/sdio_host.c" "${MCU_DIR}/drivers/transport/sdio/sdio_drv.c")
else(CONFIG_ESP_SPI_HOST_INTERFACE)
	list(APPEND COMPONENT_SRCS "${MCU_DIR}/drivers/transport/spi/spi_drv.c")
endif()

idf_component_register(SRCS ${COMPONENT_SRCS}
                    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS})
