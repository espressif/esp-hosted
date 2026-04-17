cmake_minimum_required(VERSION 3.16)
project(eh_host C CXX)

# Cross-compilation support
if(CROSS_COMPILE)
    set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc")
    set(CMAKE_CXX_COMPILER "${CROSS_COMPILE}g++")
endif()

# Add components
add_subdirectory(components/esp_queue)
add_subdirectory(components/eh_proto_linux_v1)
add_subdirectory(components/control_lib)

# Add applications
add_subdirectory(linux/host_control/c_support)
add_subdirectory(linux/host_control/python_support)

# Convenience target to build everything
add_custom_target(all_esp_hosted
    DEPENDS eh_host_rpc_static eh_host_rpc_shared
    COMMENT "Building all ESP-Hosted components"
)
