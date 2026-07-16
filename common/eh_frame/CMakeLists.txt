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

# esp_err.h transitive: idf::esp_common on IDF, port utils otherwise.
if(ESP_PLATFORM)
    target_link_libraries(eh_frame PUBLIC idf::esp_common)
else()
    target_link_libraries(eh_frame PUBLIC utils)
endif()
