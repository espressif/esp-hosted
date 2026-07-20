# SPDX-License-Identifier: Apache-2.0
if(CONFIG_EH_USE_LEGACY_API)
    add_executable(system_hosted_events_c_app legacy/main.c)
else()
    add_executable(system_hosted_events_c_app main.c)
endif()
target_link_libraries(system_hosted_events_c_app PRIVATE esp_hosted esp_hosted_examples_common)
