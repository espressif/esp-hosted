cmake_minimum_required(VERSION 3.5)

#set(EXTRA_COMPONENT_DIRS src)
#?set(MAIN_SRCS ../../../app/app_main.c)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(COMPONENTS main esp_netif esp_partition nvs_flash esp_event esp_timer driver freertos)
#set(COMPONENTS main esp_netif esp_partition nvs_flash esp_event esp_timer driver freertos esp_gdbstub) #with gdbstub
project(esp_hosted_esp32_host)
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
#target_link_options(${mdns} INTERFACE -fsanitize=address -fsanitize=undefined)
