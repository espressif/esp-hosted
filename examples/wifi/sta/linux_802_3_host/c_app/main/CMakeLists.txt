# SPDX-License-Identifier: Apache-2.0
add_executable(wifi_sta_host_c_app main.c)
target_link_libraries(wifi_sta_host_c_app PRIVATE esp_hosted esp_hosted_examples_common)
