# SPDX-License-Identifier: Apache-2.0
#
# hosted_project.cmake — IDF analog of $IDF_PATH/tools/cmake/project.cmake.
#
# A Linux-side example top-level CMakeLists.txt is:
#
#     cmake_minimum_required(VERSION 3.19)
#     # (walk up to find this file — repo-root-relative)
#     include(<repo>/tools/cmake/hosted_project.cmake)
#     project(myapp)
#     add_subdirectory(main)
#
# Everything else — host-stack subdir, shared example components,
# Kconfig pipeline, sdkconfig.h plumbing, menuconfig/save-defconfig
# CMake targets — is set up by this file as include-side-effects +
# a cmake_language(DEFER) that runs after the example's targets are
# declared.
#
# No public `hosted_kconfig_apply()` call needed in the example: the
# deferred finalize discovers the executable target via
# BUILDSYSTEM_TARGETS at end-of-dir and applies the pipeline to it.

cmake_minimum_required(VERSION 3.19)

# ── EH_PATH comes from `. ./export.sh` (the IDF $IDF_PATH
# analog).  This file CAN'T be reached without it being set (the
# example's `include($ENV{EH_PATH}/...)` is how we got
# here), so we just confirm + cache so cmake --build's nested
# re-configure (where ENV isn't necessarily preserved) still works.
if(NOT DEFINED ENV{EH_PATH} OR "$ENV{EH_PATH}" STREQUAL "")
    message(FATAL_ERROR
        "EH_PATH not set.  Source the repo's export.sh first:\n"
        "    . <repo>/export.sh\n"
        "(mirrors IDF's $IDF_PATH / export.sh model)")
endif()
set(EH_PATH "$ENV{EH_PATH}"
    CACHE PATH "esp_hosted repo root" FORCE)

# ── Pull in the Kconfig pipeline (defines hosted_kconfig_apply +
# internal helpers; also re-derives EH_PATH as a safety net).
include("${EH_PATH}/tools/cmake/hosted_kconfig.cmake")

# ── Provide idf_component_register so IDF-shape shared components
# (examples/common_components/*) build under the plain-CMake Linux host.
include("${EH_PATH}/tools/cmake/hosted_component.cmake")

# ── Pre-project compile defaults (matches IDF project.cmake's setup
# done before the user's project() call).
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# ── Override project() so the example's call automatically triggers
# the post-project setup (host-stack subdir, shared-component subdir,
# Kconfig-pipeline DEFER).  Same pattern IDF's project.cmake uses to
# hide the component-graph machinery from example authors: the
# example writes plain `project(myapp)` and everything else happens
# under the hood.
macro(project _eh_project_name)
    # Call the real CMake project() command (renamed `_project` after
    # the macro override).  Compiler detection runs here.
    _project("${_eh_project_name}" ${ARGN})

    # First-pass kconfgen — defaults-only (no graph walk yet, no
    # Kconfig.projbuild from EXTRA_COMPONENT_DIRS components).  Emits
    # sdkconfig.cmake + sdkconfig.h to <build>/config/ so the repo
    # root's `include(sdkconfig.cmake)` and the host stack's
    # `if(CONFIG_*)` gates have real CMake variables to evaluate
    # against when add_subdirectory runs below.
    # The second pass (full graph) runs at end-of-configure from
    # __hosted_project_finalize.
    hosted_kconfig_emit_defaults()

    # Pull in the host stack (declares the `esp_hosted` target + all
    # transitive sub-libraries via host/, common/, port/ subtrees).
    # IDF analog of project.cmake bringing in its own framework.
    add_subdirectory("${EH_PATH}"
                     "${CMAKE_BINARY_DIR}/esp_hosted_stack")

    # Process any extra component dirs the example declared with
    # `list(APPEND EXTRA_COMPONENT_DIRS …)` before include().  Same
    # idiom IDF uses; we just do add_subdirectory ourselves rather
    # than going through IDF's component-registry scan.  The binary
    # subdir is derived from the source dir's basename so two
    # entries with the same name fail loudly.
    foreach(_eh_extra IN LISTS EXTRA_COMPONENT_DIRS)
        if(NOT IS_DIRECTORY "${_eh_extra}")
            message(FATAL_ERROR
                "EXTRA_COMPONENT_DIRS entry not a directory: ${_eh_extra}")
        endif()
        get_filename_component(_eh_name "${_eh_extra}" NAME)
        add_subdirectory("${_eh_extra}"
                         "${CMAKE_BINARY_DIR}/${_eh_name}")
    endforeach()
endmacro()

# ── Defer the Kconfig pipeline apply to end-of-dir, so the example's
# add_subdirectory(main) has had a chance to declare its executable.
# At apply time we discover the executable target by walking
# BUILDSYSTEM_TARGETS of the source dir; convention: exactly one
# add_executable per example (matches IDF's `project()` model).
function(__hosted_project_finalize)
    # Find the "primary" target this project builds.  Prefer an
    # executable (the IDF example shape), else accept a SHARED /
    # STATIC / MODULE library (the libeh_host.so / standalone-lib
    # shape).  Walks top dir first, then subdirs.
    set(_primary "")
    set(_dirs "${CMAKE_SOURCE_DIR}")
    get_property(_subs DIRECTORY "${CMAKE_SOURCE_DIR}"
                 PROPERTY SUBDIRECTORIES)
    list(APPEND _dirs ${_subs})

    # First pass: executable.
    foreach(_dir IN LISTS _dirs)
        get_property(_tgts DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS)
        foreach(_t IN LISTS _tgts)
            get_target_property(_type "${_t}" TYPE)
            if(_type STREQUAL "EXECUTABLE")
                if(_primary)
                    message(FATAL_ERROR
                        "hosted_project: multiple executable targets "
                        "(${_primary}, ${_t}).  Expected exactly one.")
                endif()
                set(_primary "${_t}")
            endif()
        endforeach()
    endforeach()

    # Second pass: library (only when no executable).
    if(NOT _primary)
        foreach(_dir IN LISTS _dirs)
            get_property(_tgts DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS)
            foreach(_t IN LISTS _tgts)
                get_target_property(_type "${_t}" TYPE)
                if(_type MATCHES "_LIBRARY$" AND NOT _type STREQUAL "INTERFACE_LIBRARY")
                    if(_primary)
                        # Multiple libs: take the SHARED one if present
                        # (libeh_host.so case), else the first STATIC.
                        get_target_property(_prim_type "${_primary}" TYPE)
                        if(_type STREQUAL "SHARED_LIBRARY"
                                AND NOT _prim_type STREQUAL "SHARED_LIBRARY")
                            set(_primary "${_t}")
                        endif()
                    else()
                        set(_primary "${_t}")
                    endif()
                endif()
            endforeach()
        endforeach()
    endif()

    if(NOT _primary)
        message(FATAL_ERROR
            "hosted_project: no executable or library target found in "
            "${CMAKE_SOURCE_DIR}.  Add `add_executable(...)` or "
            "`add_library(...)` somewhere in the project.")
    endif()
    hosted_kconfig_apply("${_primary}")
endfunction()

cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}"
    CALL __hosted_project_finalize)
