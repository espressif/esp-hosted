# SPDX-License-Identifier: Apache-2.0
#
# Defines the eh_host_so SHARED library (libeh_host.so on disk) by
# whole-archiving the eh_host INTERFACE aggregator's STATIC closure.
# Shared by the standalone ctypes project and every Linux py-app
# example.
#
# Caller must already have:
#   - set(CMAKE_POSITION_INDEPENDENT_CODE ON) BEFORE project()
#   - include(hosted_project.cmake)
#   - project(<name> C)
# so the `eh_host` INTERFACE target exists and PIC has propagated to
# every host-stack STATIC archive.

if(NOT TARGET eh_host)
    message(FATAL_ERROR
        "eh_host_so.cmake: include hosted_project.cmake + project(...) first; "
        "eh_host INTERFACE target not found")
endif()

# Walk the eh_host INTERFACE closure and collect every STATIC archive
# explicitly.  $<LINK_LIBRARY:WHOLE_ARCHIVE,…> doesn't propagate
# transitively through INTERFACE deps on CMake 3.28; enumerating
# ourselves makes the bracket air-tight.
function(_eh_collect_static_closure root_target out_var)
    set(_visited "")
    set(_pending "${root_target}")
    while(_pending)
        list(POP_FRONT _pending _cur)
        if(_cur IN_LIST _visited OR NOT TARGET "${_cur}")
            continue()
        endif()
        list(APPEND _visited "${_cur}")
        foreach(_prop LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
            get_target_property(_deps "${_cur}" "${_prop}")
            if(_deps)
                foreach(_d IN LISTS _deps)
                    string(GENEX_STRIP "${_d}" _d_s)
                    if(_d_s MATCHES "^[A-Za-z_][A-Za-z0-9_:-]*$"
                            AND TARGET "${_d_s}"
                            AND NOT _d_s IN_LIST _visited)
                        list(APPEND _pending "${_d_s}")
                    endif()
                endforeach()
            endif()
        endforeach()
    endwhile()
    set(_static "")
    foreach(_t IN LISTS _visited)
        get_target_property(_type "${_t}" TYPE)
        if(_type STREQUAL "STATIC_LIBRARY")
            list(APPEND _static "${_t}")
        endif()
    endforeach()
    set("${out_var}" "${_static}" PARENT_SCOPE)
endfunction()

_eh_collect_static_closure(eh_host _eh_static_closure)

set(_eh_ctypes_dir "${EH_PATH}/host/linux/eh_host_linux_python_ctypes")

add_library(eh_host_so SHARED "${_eh_ctypes_dir}/eh_host_dummy.c")

target_link_libraries(eh_host_so PRIVATE
    "-Wl,--whole-archive"
    ${_eh_static_closure}
    "-Wl,--no-whole-archive"
    eh_host
    pthread rt)

target_link_options(eh_host_so PRIVATE
    "-Wl,--version-script=${_eh_ctypes_dir}/eh_host.lds"
    "-Wl,-soname,libeh_host.so")

set_target_properties(eh_host_so PROPERTIES
    OUTPUT_NAME               eh_host
    POSITION_INDEPENDENT_CODE ON
    C_STANDARD                11
    C_STANDARD_REQUIRED       ON
    LIBRARY_OUTPUT_DIRECTORY  "${CMAKE_BINARY_DIR}/eh_libeh_host")
