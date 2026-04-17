cmake_minimum_required(VERSION 3.16)
project(eh_host_rpc C)

# Cross-compilation support
if(CROSS_COMPILE)
    set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc")
endif()

# Directory paths
get_filename_component(DIR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

# Add dependencies if not already built
if(NOT TARGET ehrcp_linux_protobuf)
    add_subdirectory("${DIR_ROOT}/host/components/eh_proto_linux_v1" ehrcp_protobuf_build)
endif()

if(NOT TARGET esp_queue)
    add_subdirectory("${DIR_ROOT}/host/components/esp_queue" esp_queue_build)
endif()

# Create static library
add_library(eh_host_rpc_static STATIC
    src/ctrl_core.c
    src/ctrl_api.c
    src/serial_if.c
    port/linux/src/platform_wrapper.c
)

target_include_directories(eh_host_rpc_static PUBLIC
    "${DIR_ROOT}/common/include"
    include
    src/include
    port/linux/include
)

target_link_libraries(eh_host_rpc_static PUBLIC
    ehrcp_linux_protobuf
    esp_queue
)

# Create shared library
add_library(eh_host_rpc_shared SHARED
    src/ctrl_core.c
    src/ctrl_api.c
    src/serial_if.c
    port/linux/src/platform_wrapper.c
)

target_include_directories(eh_host_rpc_shared PUBLIC
    "${DIR_ROOT}/common/include"
    include
    src/include
    port/linux/include
)

target_link_libraries(eh_host_rpc_shared
    ehrcp_linux_protobuf
    esp_queue
    pthread
    rt
)

set_target_properties(eh_host_rpc_shared eh_host_rpc_static PROPERTIES
    OUTPUT_NAME "eh_host_rpc"
    POSITION_INDEPENDENT_CODE ON
)
