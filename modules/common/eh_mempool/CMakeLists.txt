add_library(eh_mempool STATIC
    src/eh_mempool.c
    src/eh_mempool_ll.c
)

target_include_directories(eh_mempool
    PUBLIC
        "include"
)

target_link_libraries(eh_mempool
    PUBLIC
        idf::esp_common
)
