# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VERSION_MAJOR_1 "1")
set(PROJECT_VERSION_MAJOR_2 "0")
set(PROJECT_VERSION_MINOR "2")
message(*************************************************************************************)
message("                    Building ESP-Hosted Firmware version :: ${PROJECT_VERSION_MAJOR_1}.${PROJECT_VERSION_MAJOR_2}.${PROJECT_VERSION_MINOR} ")
message(*************************************************************************************)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXCLUDE_COMPONENTS wpa_supplicant)
project(network_adapter)
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
