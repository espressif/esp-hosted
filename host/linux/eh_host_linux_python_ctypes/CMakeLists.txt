# SPDX-License-Identifier: Apache-2.0
# Standalone libeh_host.so build.

cmake_minimum_required(VERSION 3.19)

if(ESP_PLATFORM)
    return()
endif()

# PIC must precede project() so it propagates to the static closure.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)

project(eh_host_so C)

include($ENV{EH_PATH}/tools/cmake/eh_host_so.cmake)
