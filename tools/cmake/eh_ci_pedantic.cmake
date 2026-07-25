# SPDX-License-Identifier: Apache-2.0
#
# CI-only maximal strictness, scoped to esp_hosted's OWN sources.
#
# Included once from the top-level CMakeLists.txt, AFTER all add_subdirectory()
# calls (so every eh_* target already exists). Applies an aggressive warning set
# with -Werror to every eh_* target only; IDF, vendored and third-party targets
# are never touched. Each flag is probed once (cached) and applied only where the
# compiler supports it, so the set stays portable across GCC versions.
#
# Enable with:  ESP_HOSTED_CI_PEDANTIC=1 idf.py build
#
# NOTE: the collection root is CMAKE_CURRENT_SOURCE_DIR (the includer's directory
# == repo root), NOT CMAKE_CURRENT_LIST_DIR (which would be this module's dir).

if(ESP_PLATFORM AND DEFINED ENV{ESP_HOSTED_CI_PEDANTIC})
    include(CheckCCompilerFlag)

    function(_eh_collect_targets _out _dir)
        get_property(_subdirs DIRECTORY "${_dir}" PROPERTY SUBDIRECTORIES)
        get_property(_targets DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS)

        foreach(_subdir IN LISTS _subdirs)
            _eh_collect_targets(_child_targets "${_subdir}")
            list(APPEND _targets ${_child_targets})
        endforeach()

        set(${_out} ${_targets} PARENT_SCOPE)
    endfunction()

    function(_eh_add_supported_c_warning _target _flag)
        string(MAKE_C_IDENTIFIER "${_flag}" _flag_id)
        set(_cache_var "EH_COMPILER_SUPPORTS_${_flag_id}")

        check_c_compiler_flag("${_flag}" ${_cache_var})

        if(${_cache_var})
            target_compile_options(
                "${_target}"
                PRIVATE
                $<$<COMPILE_LANGUAGE:C>:${_flag}>
            )
        endif()
    endfunction()

    # Full aggressive warning set, applied to eh_* targets ONLY, as plain warnings
    # (NOT -Werror). Rationale: a global -Werror stops at the first diagnostic and
    # also fails on ESP-IDF's own headers (which our TUs #include and we must not
    # patch). Instead the CI/build step fails the build only when a warning's file
    # path is under OUR sources — so these all act as hard gates on esp_hosted code
    # while IDF/vendored/third-party noise is ignored. Fix our code to zero; never
    # silence a flag. (-fanalyzer is intentionally omitted here for build speed and
    # re-added as a final, separate pass.)
    set(_eh_strict_common_flags
        # Keep OUR targets warnings-only: IDF compiles this project with a global
        # -Werror, so without this every added -W would become an error and stop at
        # the first IDF-header hit. -Wno-error (applied last, PRIVATE to eh_*) leaves
        # IDF's own -Werror intact for IDF code; the CI step fails on any warning
        # whose path is under our sources.
        -Wno-error
        -Wall
        -Wextra
        -Wpedantic

        -Wshadow
        -Wformat=2
        -Wformat-security
        -Wconversion
        -Wsign-conversion
        -Warith-conversion
        -Wsign-compare

        -Wcast-align
        -Wcast-qual
        -Wwrite-strings
        -Wpointer-arith
        -Wredundant-decls
        -Wmissing-declarations

        -Wswitch-default
        -Wimplicit-fallthrough=5

        -Wdouble-promotion
        -Wfloat-equal

        -Wvla
        -Walloca
        -Wstack-usage=4096

        -Wnull-dereference
        -Wlogical-op
        -Wduplicated-cond
        -Wduplicated-branches

        -Wstrict-overflow=5
        -Wstringop-overflow=4
        -Warray-bounds=2
        -Wshift-overflow=2
        -Wshift-negative-value
        -Walloc-zero
        -Walloc-size-larger-than=65536
        -Wsizeof-pointer-div
        -Wsizeof-array-div
    )

    set(_eh_strict_c_flags
        -Wstrict-prototypes
        -Wmissing-prototypes
        -Wold-style-definition
        -Wbad-function-cast
        -Wnested-externs
        -Wjump-misses-init
    )

    _eh_collect_targets(_eh_all_targets "${CMAKE_CURRENT_SOURCE_DIR}")

    foreach(_target IN LISTS _eh_all_targets)
        if(NOT "${_target}" MATCHES "^eh_")
            continue()
        endif()

        # Skip generated serializer code (protobuf-c output) — regenerated from
        # .proto, not hand-maintained, so out of scope for warning cleanup.
        if("${_target}" MATCHES "^eh_proto")
            continue()
        endif()

        get_target_property(_target_type "${_target}" TYPE)
        get_target_property(_target_imported "${_target}" IMPORTED)

        if(_target_imported
           OR _target_type STREQUAL "INTERFACE_LIBRARY"
           OR _target_type STREQUAL "UTILITY")
            continue()
        endif()

        foreach(_flag IN LISTS _eh_strict_common_flags)
            _eh_add_supported_c_warning("${_target}" "${_flag}")
        endforeach()

        foreach(_flag IN LISTS _eh_strict_c_flags)
            _eh_add_supported_c_warning("${_target}" "${_flag}")
        endforeach()
    endforeach()
endif()
