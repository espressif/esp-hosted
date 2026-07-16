# SPDX-License-Identifier: Apache-2.0
# Linux userspace port-type STATIC.

if(CONFIG_ESP_HOSTED_HOST_PORT_TYPE_POSIX OR
   NOT DEFINED CONFIG_ESP_HOSTED_HOST_PORT_TYPE_POSIX)

    set(EH_HOST_PORT_POSIX_SRCS
        "src/eh_host_port_dma.c"
        "src/eh_host_port_sync.c"
        "src/eh_host_port_event_group.c"
        "src/eh_host_port_queue.c"
        "src/eh_host_port_task.c"
        "src/eh_host_port_gpio.c"
        "src/eh_host_port_power.c"
    )

    add_library(eh_host_port_posix STATIC ${EH_HOST_PORT_POSIX_SRCS})

    target_include_directories(eh_host_port_posix
        PUBLIC  "${CMAKE_CURRENT_LIST_DIR}/../include"
        PRIVATE "src"
    )

    target_compile_options(eh_host_port_posix
        PRIVATE "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/posix_features.h"
    )

    target_link_libraries(eh_host_port_posix
        PUBLIC  eh_host_port_api eh_mempool esp_event
        PRIVATE pthread
    )
endif()
