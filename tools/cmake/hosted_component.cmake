# SPDX-License-Identifier: Apache-2.0
#
# hosted_component.cmake — Linux-side analog of IDF's idf_component_register.
#
# Shared example components (examples/common_components/*) are written once in
# IDF shape — `idf_component_register(SRCS ... INCLUDE_DIRS ... REQUIRES ...)`
# — so the SAME source tree builds under ESP-IDF (mcu_host) and under the
# plain-CMake Linux host build (linux_802_3_host/c_app). On IDF the command
# comes from the framework; here we provide an equivalent that turns the call
# into an ordinary CMake library, so no component needs a Linux-specific
# CMakeLists (matches the "Linux feels IDF-homely, no per-example band-aids"
# rule).
#
# The component's name is its directory basename (IDF convention); that is the
# target other code links by name (e.g. `target_link_libraries(app
# esp_hosted_examples_common)`).
#
# REQUIRES / PRIV_REQUIRES are IDF component names. On Linux only some exist as
# real targets (esp_event, esp_netif, nvs_flash, esp_hosted, ...). We link the
# ones that resolve and skip the rest — their headers arrive transitively via
# esp_hosted (e.g. esp_wifi headers come through esp_wifi_remote). Unknown
# names are not an error: IDF's component set is a superset of the Linux port.

macro(idf_component_register)
    set(_options WHOLE_ARCHIVE)
    set(_multi SRCS SRC_DIRS EXCLUDE_SRCS INCLUDE_DIRS PRIV_INCLUDE_DIRS
        REQUIRES PRIV_REQUIRES REQUIRED_IDF_TARGETS
        EMBED_FILES EMBED_TXTFILES LDFRAGMENTS)
    cmake_parse_arguments(_ehc "${_options}" "" "${_multi}" ${ARGN})

    get_filename_component(_ehc_name "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

    # Collect sources: explicit SRCS plus *.c/*.cpp globbed from SRC_DIRS.
    set(_ehc_srcs ${_ehc_SRCS})
    foreach(_dir IN LISTS _ehc_SRC_DIRS)
        file(GLOB _globbed
            "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/*.cpp")
        list(APPEND _ehc_srcs ${_globbed})
    endforeach()
    if(_ehc_EXCLUDE_SRCS)
        list(REMOVE_ITEM _ehc_srcs ${_ehc_EXCLUDE_SRCS})
    endif()

    if(_ehc_srcs)
        add_library(${_ehc_name} STATIC ${_ehc_srcs})
        set(_ehc_pub PUBLIC)
        set(_ehc_priv PRIVATE)
    else()
        # Header-only component (INCLUDE_DIRS but no sources).
        add_library(${_ehc_name} INTERFACE)
        set(_ehc_pub INTERFACE)
        set(_ehc_priv INTERFACE)
    endif()

    foreach(_inc IN LISTS _ehc_INCLUDE_DIRS)
        target_include_directories(${_ehc_name} ${_ehc_pub}
            "${CMAKE_CURRENT_SOURCE_DIR}/${_inc}")
    endforeach()
    foreach(_inc IN LISTS _ehc_PRIV_INCLUDE_DIRS)
        target_include_directories(${_ehc_name} ${_ehc_priv}
            "${CMAKE_CURRENT_SOURCE_DIR}/${_inc}")
    endforeach()

    # Link REQUIRES that resolve to real targets; skip the rest.
    foreach(_req IN LISTS _ehc_REQUIRES)
        if(TARGET ${_req})
            target_link_libraries(${_ehc_name} ${_ehc_pub} ${_req})
        endif()
    endforeach()
    foreach(_req IN LISTS _ehc_PRIV_REQUIRES)
        if(TARGET ${_req})
            target_link_libraries(${_ehc_name} ${_ehc_priv} ${_req})
        endif()
    endforeach()
endmacro()
