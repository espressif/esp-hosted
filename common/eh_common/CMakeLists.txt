add_library(eh_common STATIC
    src/eh_common_stub.c
    src/eh_check.c
)

target_include_directories(eh_common
    PUBLIC
        "include"
)

target_link_libraries(eh_common PUBLIC eh_tlv)

if(NOT ESP_PLATFORM)
    target_link_libraries(eh_common PUBLIC log)
endif()
