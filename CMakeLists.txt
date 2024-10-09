if(CONFIG_ESP_HOSTED_ENABLED)
	message(STATUS "Using Hosted Wi-Fi")
	set(FG_root_dir ".")
	set(host_dir "${FG_root_dir}/host")

	set(COMPONENT_ADD_INCLUDEDIRS "${host_dir}" "${host_dir}/port" "${host_dir}/api")
	set(COMPONENT_SRCS "${host_dir}/esp_hosted_host_init.c" "${host_dir}/api/esp_wifi_weak.c" "${host_dir}/api/esp_hosted_api.c" "${host_dir}/drivers/transport/transport_drv.c" "${host_dir}/drivers/serial/serial_ll_if.c" "${host_dir}/utils/common.c" "${host_dir}/utils/util.c" "${host_dir}/utils/stats.c" "${host_dir}/port/os_wrapper.c" "${host_dir}/drivers/serial/serial_drv.c")

	list(APPEND COMPONENT_ADD_INCLUDEDIRS "." "${host_dir}/api" "${host_dir}/drivers/transport" "${host_dir}/drivers/transport/spi" "${host_dir}/drivers/transport/sdio" "${host_dir}/drivers/serial" "${host_dir}/utils")

	# rpc files - wrap -> slaveif -> core
	set(rpc_dir "${host_dir}/drivers/rpc")
	set(rpc_core_dir "${rpc_dir}/core")
	set(rpc_slaveif_dir "${rpc_dir}/slaveif")
	set(rpc_wrap_dir "${rpc_dir}/wrap")
	list(APPEND COMPONENT_SRCS  "${rpc_core_dir}/rpc_core.c" "${rpc_core_dir}/rpc_req.c" "${rpc_core_dir}/rpc_rsp.c" "${rpc_core_dir}/rpc_evt.c"
		"${rpc_slaveif_dir}/rpc_slave_if.c"
		"${rpc_wrap_dir}/rpc_wrap.c")
	list(APPEND COMPONENT_ADD_INCLUDEDIRS "${rpc_core_dir}" "${rpc_slaveif_dir}" "${rpc_wrap_dir}")

	# virtual serial
	set(virt_serial_dir "${host_dir}/drivers/virtual_serial_if")
	list(APPEND COMPONENT_SRCS  "${virt_serial_dir}/serial_if.c")
	list(APPEND COMPONENT_ADD_INCLUDEDIRS "${virt_serial_dir}")

	# mempool
	list(APPEND COMPONENT_SRCS "${host_dir}/drivers/mempool/mempool.c")
	list(APPEND COMPONENT_ADD_INCLUDEDIRS "${host_dir}/drivers/mempool" )

	# slave and host common files
	set(common_dir "${FG_root_dir}/common")
	list(APPEND COMPONENT_SRCS "${common_dir}/protobuf-c/protobuf-c/protobuf-c.c" "${common_dir}/proto/esp_hosted_rpc.pb-c.c" )
	list(APPEND COMPONENT_ADD_INCLUDEDIRS "${common_dir}/include" "${common_dir}/protobuf-c" "${common_dir}/proto" )

	# bt (NimBLE)
	### TODO config for HCI over UART
	list(APPEND COMPONENT_ADD_INCLUDEDIRS "${host_dir}/drivers/bt")
	if(CONFIG_ESP_HCI_VHCI)
		list(APPEND COMPONENT_SRCS "${host_dir}/drivers/bt/vhci_drv.c")
	else()
		list(APPEND COMPONENT_SRCS "${host_dir}/drivers/bt/hci_stub_drv.c")
	endif()

	# transport files
	if(CONFIG_ESP_SDIO_HOST_INTERFACE)
		list(APPEND COMPONENT_SRCS "${host_dir}/drivers/transport/sdio/sdio_drv.c" "${host_dir}/port/sdio_wrapper.c")
	elseif(CONFIG_ESP_SPI_HD_HOST_INTERFACE)
		list(APPEND COMPONENT_SRCS "${host_dir}/drivers/transport/spi_hd/spi_hd_drv.c" "${host_dir}/port/spi_hd_wrapper.c")
	elseif(CONFIG_ESP_SPI_HOST_INTERFACE)
		list(APPEND COMPONENT_SRCS "${host_dir}/drivers/transport/spi/spi_drv.c" "${host_dir}/port/spi_wrapper.c")
	elseif(CONFIG_ESP_UART_HOST_INTERFACE)
		list(APPEND COMPONENT_SRCS "${host_dir}/drivers/transport/uart/uart_drv.c" "${host_dir}/port/uart_wrapper.c")
	endif()
endif()

idf_component_register(SRCS ${COMPONENT_SRCS}
	REQUIRES soc esp_event esp_netif esp_timer driver esp_wifi bt esp_http_client
	EXCLUDE_SRCS ${EXCLUDE_COMPONENT_SRCS}
	INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS})

idf_component_set_property(${COMPONENT_NAME} WHOLE_ARCHIVE TRUE)

if(CONFIG_ESP_SDIO_HOST_INTERFACE)
	idf_component_optional_requires(PRIVATE sdmmc)
endif()

# Required if using ESP-IDF without commit 6b6065de509b5de39e4655fd425bf96f43b365f7:
# fix(driver_spi): fix p4 cache auto writeback during spi(dma) rx
# if(CONFIG_IDF_TARGET_ESP32P4 AND (CONFIG_ESP_SPI_HOST_INTERFACE OR CONFIG_ESP_SPI_HD_HOST_INTERFACE))
# 	# used to workaround SPI transfer issue
# 	idf_component_optional_requires(PRIVATE esp_mm)
# endif()
