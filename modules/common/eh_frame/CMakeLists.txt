add_library(eh_frame STATIC
    src/eh_frame.c
)

target_include_directories(eh_frame
    PUBLIC
        "include"
)

target_link_libraries(eh_frame
    PUBLIC
        eh_common
)
