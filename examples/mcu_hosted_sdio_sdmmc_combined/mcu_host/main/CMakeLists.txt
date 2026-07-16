set(srcs "sd_card_example_main.c"
    "sd_card_functions.c"
    "esp_hosted_wifi.c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES fatfs esp_wifi
                       WHOLE_ARCHIVE)

if(NOT CONFIG_SOC_SDMMC_HOST_SUPPORTED)
    fail_at_build_time(sdmmc ""
                             "Only ESP32, ESP32-S3, ESP32-P4 targets are supported."
                             "Please refer README.md for more details")
endif()
