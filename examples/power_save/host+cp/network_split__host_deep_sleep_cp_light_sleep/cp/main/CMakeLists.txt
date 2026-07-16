idf_component_register(SRCS "main.c"
                            "host_ps_integration.c"
                            "cp_light_sleep_controller.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hosted
                    PRIV_REQUIRES nvs_flash esp_wifi esp_pm)
