cmake_minimum_required(VERSION 3.16)
project(eh_host_python_support C)

# Cross-compilation
if(CROSS_COMPILE)
    set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc")
endif()

# Find control library (built by parent or separately)
get_filename_component(DIR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(DIR_CTRL_LIB "${DIR_ROOT}/host/components/eh_host_rpc_lib")

# Add control library dependency if building standalone
if(NOT TARGET eh_host_rpc_shared)
    add_subdirectory("${DIR_CTRL_LIB}" control_lib_build)
endif()
