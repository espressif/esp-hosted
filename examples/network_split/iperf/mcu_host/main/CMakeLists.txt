if(CONFIG_ESP_HOSTED_CP_EXAMPLE_MQTT_CLIENT)
    list(APPEND additional_srcs "extra/example_mqtt_client.c")
    list(APPEND additional_includes "extra")
endif()

if(CONFIG_ESP_HOSTED_CP_EXAMPLE_HTTP_CLIENT)
    list(APPEND additional_srcs "extra/example_http_client.c")
    list(APPEND additional_includes "extra")
endif()

if(CONFIG_EXAMPLE_NW_SPLIT_ECHO_SERVER)
    list(APPEND additional_srcs "eh_echo_server.c")
endif()

idf_component_register(SRCS "iperf_example_main.c" ${additional_srcs}
                    INCLUDE_DIRS "." ${additional_includes}
                    REQUIRES esp_hosted
						esp_hosted_examples_common
						lwip
						mqtt
						nvs_flash
						esp_http_client
					PRIV_REQUIRES iperf)
