# SPDX-License-Identifier: Apache-2.0
if(CONFIG_EH_USE_LEGACY_API)
    add_executable(peer_data_transfer_echo_c_app legacy/main.c)
else()
    add_executable(peer_data_transfer_echo_c_app main.c)
endif()
target_link_libraries(peer_data_transfer_echo_c_app PRIVATE esp_hosted )
