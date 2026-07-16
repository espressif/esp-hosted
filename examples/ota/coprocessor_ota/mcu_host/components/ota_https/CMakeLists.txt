if(CONFIG_OTA_METHOD_HTTPS)
	set(SRC_LIST "ota_https.c" "ota_https_wifi.c")
endif()

idf_component_register(SRCS ${SRC_LIST}
	INCLUDE_DIRS "."
	REQUIRES esp_http_client app_update esp_wifi esp_hosted nvs_flash esp_https_ota mbedtls esp-tls bootloader_support esp_app_format)

# Conditional certificate embedding based on Kconfig
if(CONFIG_OTA_METHOD_HTTPS)
if(CONFIG_OTA_USE_SELF_SIGNED_CERT)
    if(EXISTS "${COMPONENT_DIR}/certs/server_cert.pem")
        target_add_binary_data(
            ${COMPONENT_LIB}
            "certs/server_cert.pem"
            TEXT
        )
        message(STATUS "✓ HTTPS OTA: Embedding self-signed certificate for testing")
    else()
        message(FATAL_ERROR
            "HTTPS OTA: Self-signed cert enabled but certs/server_cert.pem not found! \n"
            "1. Run ./create_self_signed_certs.sh in components/ota_https/test_server directory to generate certificates.\n"
            "2. Run ./create_https_server.py in components/ota_https/test_server directory to start the HTTPS server.")
    endif()
else()
    message(STATUS "✓ HTTPS OTA: Using CA certificate bundle (Production mode)")
endif()
endif()
