# eh_host_extra/esp_event — pthread-backed esp_event runtime (linux_user)
# SPDX-License-Identifier: Apache-2.0
#
# Provides a Tier-1 subset of the upstream IDF esp_event API on top of
# POSIX pthreads, so apps written against esp_event_handler_register /
# esp_event_post run unmodified on the Linux host.
#
# Scope (see src/ headers for exact semantics):
#   - esp_event_loop_create_default / _delete_default
#   - esp_event_handler_register / _unregister (legacy form)
#   - esp_event_handler_instance_register / _unregister (forward to
#     legacy form; instance handle = handler fn pointer)
#   - esp_event_post / _post_to(NULL,…)
#
# Out of scope (return ESP_ERR_NOT_SUPPORTED):
#   - non-default loops (any non-NULL loop handle in *_with variants),
#     manual loop_run
#
# This target is selected only when the host port is linux_user; on the
# esp_idf port real IDF supplies esp_event natively. Main thread wires
# this into the linux_user host-side build (see the top-level CMakeLists);
# example linux_host build.sh is not modified by this target.

if(ESP_PLATFORM)
    # IDF supplies esp_event natively as `idf::esp_event`; this port-side
    # shadow is for non-FreeRTOS hosts only.
    return()
endif()

add_library(esp_event STATIC
    "src/esp_event_loop.c"
    "src/esp_event_handler.c"
)

target_include_directories(esp_event
    PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/include"
    PRIVATE "src"
)

target_compile_options(esp_event
    PRIVATE -Wall -Wextra -D_POSIX_C_SOURCE=200809L
)

target_link_libraries(esp_event
    PUBLIC utils log esp_system eh_host_port_api_headers
    PRIVATE pthread
)
