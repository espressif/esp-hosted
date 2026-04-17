add_library(eh_common STATIC
    src/eh_common_stub.c
)

target_include_directories(eh_common
    PUBLIC
        "include"
)
