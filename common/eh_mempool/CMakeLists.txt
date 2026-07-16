# eh_mempool — mynewt os_mempool primitive.

set(EH_MEMPOOL_SRCS src/eh_mempool_ll.c)

# Legacy CP-side wrapper is FreeRTOS-dependent — only build on ESP.
if(ESP_PLATFORM OR CMAKE_SYSTEM_NAME STREQUAL "Generic")
    list(APPEND EH_MEMPOOL_SRCS src/eh_mempool.c)
endif()

add_library(eh_mempool STATIC ${EH_MEMPOOL_SRCS})
target_include_directories(eh_mempool
    PUBLIC "include"
)

if(ESP_PLATFORM)
    # Legacy wrapper includes FreeRTOS headers + esp_log.
    target_link_libraries(eh_mempool
        PRIVATE idf::freertos idf::log
    )
endif()

if(CONFIG_ESP_HOSTED_HOST)
    target_link_libraries(eh_mempool
        PRIVATE eh_host_port_api
    )
endif()
