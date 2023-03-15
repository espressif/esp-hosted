cmake_minimum_required(VERSION 3.5)

#set(EXTRA_COMPONENT_DIRS src)
#?set(MAIN_SRCS ../../../app/app_main.c)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_host)
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
