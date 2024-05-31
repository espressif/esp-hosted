# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VERSION_MAJOR_1 "0")
set(PROJECT_VERSION_MAJOR_2 "0")
set(PROJECT_VERSION_MINOR "6")
message(*************************************************************************************)
message("                    Building ESP-Hosted-MCU FW :: ${PROJECT_VERSION_MAJOR_1}.${PROJECT_VERSION_MAJOR_2}.${PROJECT_VERSION_MINOR} ")
message(*************************************************************************************)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(APPEND EXCLUDE_COMPONENTS lwip esp_netif)
#set(COMPONENTS main nvs_flash protocomm esp_rom)
set(COMPONENTS esp_timer esptool_py bootloader main nvs_flash esp_rom esp_wifi protocomm )
#set(COMPONENTS main nvs_flash protocomm esp_gdbstub) # incase gdbsub needed
project(network_adapter)
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
