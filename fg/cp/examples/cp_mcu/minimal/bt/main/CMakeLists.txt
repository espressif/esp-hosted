idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
					   REQUIRES esp_hosted
					   PRIV_REQUIRES nvs_flash
)
