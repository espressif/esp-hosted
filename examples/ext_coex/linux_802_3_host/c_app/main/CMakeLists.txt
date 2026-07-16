# SPDX-License-Identifier: Apache-2.0
if(CONFIG_EH_USE_LEGACY_API)
    add_executable(ext_coex_pta_c_app legacy/main.c)
else()
    add_executable(ext_coex_pta_c_app main.c)
endif()
target_link_libraries(ext_coex_pta_c_app PRIVATE esp_hosted )
