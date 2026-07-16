# SPDX-License-Identifier: Apache-2.0
# Shares the exerciser command corpus (../../common/eh_api_cmd.c) with the
# mcu_host role. The posix esp_console port makes esp_console_* build unchanged
# here; this in-place linux build references common/ directly.
set(_common ${CMAKE_CURRENT_LIST_DIR}/../../../common)
add_executable(system_api_exerciser_c_app
    main.c
    ${_common}/eh_api_cmd.c)
target_include_directories(system_api_exerciser_c_app PRIVATE ${_common})
target_link_libraries(system_api_exerciser_c_app PRIVATE esp_hosted)
