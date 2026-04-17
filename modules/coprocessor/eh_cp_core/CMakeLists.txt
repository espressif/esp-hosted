# eh_cp_core — Two-registry engine, transport glue

set(common_shared_inc
    "${CMAKE_CURRENT_LIST_DIR}/../../../modules/common/eh_common/include")

set(CORE_SRCS
    "src/eh_cp_core.c"
    "src/eh_cp_registries.c"
)
set(PROTOCOL_SRCS
    "src/eh_cp_event_base.c"
)
set(UTIL_SRCS
    "src/eh_cp_utils.c"
    "src/eh_cp_transport_test.c"
)
add_library(eh_cp_core STATIC
    ${CORE_SRCS}
    ${PROTOCOL_SRCS}
    ${UTIL_SRCS}
)

target_include_directories(eh_cp_core
    PUBLIC
        "include"
        "${common_shared_inc}"
)

set(cp_core_requires
    idf::esp_event
    idf::driver
    idf::esp_timer
    idf::esp_common
    eh_cp_transport
    eh_common
    eh_frame
)

if(CONFIG_ESP_WIFI_ENABLED)
    list(APPEND cp_core_requires idf::esp_wifi)
endif()

target_link_libraries(eh_cp_core
    PUBLIC
        ${cp_core_requires}
)
