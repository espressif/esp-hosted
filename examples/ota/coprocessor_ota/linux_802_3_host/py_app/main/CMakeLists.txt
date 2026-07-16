# SPDX-License-Identifier: Apache-2.0
include($ENV{EH_PATH}/tools/cmake/eh_host_so.cmake)
add_custom_target(${PROJECT_NAME} ALL
    DEPENDS eh_host_so
    SOURCES main.py)
