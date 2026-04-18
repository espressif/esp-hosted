cmake_minimum_required(VERSION 3.16)
project(eh_host_c_support C CXX)

# Cross-compilation
if(CROSS_COMPILE)
    set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc")
    set(CMAKE_CXX_COMPILER "${CROSS_COMPILE}g++")
endif()

# Find control library (built by parent or separately)
get_filename_component(DIR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)

# Add control library dependency if building standalone
if(NOT TARGET eh_host_rpc_static)
    add_subdirectory("${DIR_ROOT}/host/components/eh_host_rpc_lib" control_lib_build)
endif()

# Common sources
add_library(common OBJECT test_utils.c nw_helper_func.c app_custom_rpc.c app_peer_data_transfer.c eh_host_ext_peer_data_transfer.c)
target_link_libraries(common PRIVATE eh_host_rpc_static)
target_include_directories(common
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/common
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build executables
foreach(target IN ITEMS test stress hosted_daemon)
    add_executable(${target} ${target}.c)
    target_link_libraries(${target} common eh_host_rpc_static pthread rt)
    set_target_properties(${target} PROPERTIES OUTPUT_NAME "${target}.out")
endforeach()

# hosted_shell (only if replxx available)
find_library(REPLXX_LIB replxx)
if(REPLXX_LIB)
    add_executable(hosted_shell hosted_shell.c)
    target_link_libraries(hosted_shell common eh_host_rpc_static ${REPLXX_LIB} stdc++ pthread rt)
    set_target_properties(hosted_shell PROPERTIES OUTPUT_NAME "hosted_shell.out")
endif()
