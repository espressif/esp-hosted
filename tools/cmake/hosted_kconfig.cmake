# SPDX-License-Identifier: Apache-2.0
#
# hosted_kconfig.cmake — Linux-side Kconfig pipeline driver.
#
# Mirrors IDF's tools/cmake/kconfig.cmake behaviour:
#
#   1. Walk the CMake build graph from the project's main executable
#      through transitive LINK_LIBRARIES; for each resolved target
#      check fixed-name files <SOURCE_DIR>/Kconfig and
#      <SOURCE_DIR>/Kconfig.projbuild.
#   2. Export the IDF env contract:
#        COMPONENT_KCONFIGS
#        COMPONENT_KCONFIGS_PROJBUILD
#        COMPONENT_KCONFIGS_SOURCE_FILE
#        COMPONENT_KCONFIGS_PROJBUILD_SOURCE_FILE
#   3. Invoke prepare_kconfig_files.py to materialise kconfigs.in +
#      kconfigs_projbuild.in.
#   4. Invoke `python -m kconfgen` to produce sdkconfig +
#      sdkconfig.h + sdkconfig.cmake + sdkconfig.json + kconfig_menus.json,
#      all under <build>/config/.
#   5. Register CMake custom targets `menuconfig` and `save-defconfig`.
#
# Public surface (one entry-point call from the project's top-level
# CMakeLists.txt, AFTER project() has run and the build graph is set up):
#
#   include($ENV{EH_PATH}/tools/cmake/hosted_kconfig.cmake)
#   ...
#   add_executable(myapp ...)
#   target_link_libraries(myapp PRIVATE esp_hosted ...)
#   hosted_kconfig_apply(myapp)
#
# `hosted_kconfig_apply(<exe_target>)` is the single hook — it does
# the graph walk, runs kconfgen, attaches -include sdkconfig.h and
# the config dir to every target in the resolved graph.  It is NOT
# a component registration API; component membership is decided
# entirely by what `target_link_libraries` resolves to.
#
# Per-target overrides (optional, no global variables):
#   set_target_properties(my_target PROPERTIES
#       HOSTED_KCONFIG          "/abs/path/to/Kconfig"
#       HOSTED_KCONFIG_PROJBUILD "/abs/path/to/Kconfig.projbuild")
# Used when a target wants to declare paths that don't follow the
# default <SOURCE_DIR>/Kconfig{.projbuild} convention.

cmake_minimum_required(VERSION 3.19)  # cmake_language(DEFER) + GENEX_STRIP

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# EH_PATH is set by hosted_project.cmake before this file
# is included (via `. ./export.sh` → $ENV{EH_PATH}).  If
# something else is including us directly without that, fail loud
# rather than silently guessing.
if(NOT DEFINED EH_PATH OR NOT EXISTS "${EH_PATH}/Kconfig")
    message(FATAL_ERROR
        "hosted_kconfig.cmake: EH_PATH not set or invalid.\n"
        "Source the repo's export.sh first: `. <repo>/export.sh`.")
endif()

# Search order for the kconfig backend:
#   1. ${EH_HOSTED_KCONFIG_BACKEND} explicitly set by the caller / env.
#   2. ${EH_PATH}/tools/vendor/esp-idf-kconfig (canonical, tracked).
if(NOT DEFINED EH_HOSTED_KCONFIG_BACKEND)
    if(DEFINED ENV{EH_HOSTED_KCONFIG_BACKEND})
        set(EH_HOSTED_KCONFIG_BACKEND "$ENV{EH_HOSTED_KCONFIG_BACKEND}")
    endif()
endif()
if(NOT EH_HOSTED_KCONFIG_BACKEND)
    set(_candidates
        "$ENV{EH_PATH}/tools/vendor/esp-idf-kconfig"
        "$ENV{EH_PATH}/tools/vendor")
    foreach(_c IN LISTS _candidates)
        if(EXISTS "${_c}/kconfgen" OR EXISTS "${_c}/kconfgen/__init__.py")
            set(EH_HOSTED_KCONFIG_BACKEND "${_c}")
            break()
        endif()
    endforeach()
endif()
if(NOT EH_HOSTED_KCONFIG_BACKEND OR NOT EXISTS "${EH_HOSTED_KCONFIG_BACKEND}")
    message(FATAL_ERROR
        "hosted_kconfig: cannot locate esp-idf-kconfig backend.\n"
        "Looked at $ENV{EH_PATH}/tools/vendor/esp-idf-kconfig.\n"
        "Set EH_HOSTED_KCONFIG_BACKEND in the env or as a CMake variable.")
endif()

# The backend's report module imports `rich`, which is not in a stock distro
# Python. On hosts without it, our tiny plain-text shim stands in — but ONLY
# when real rich is missing, so machines that have it keep the styled output
# (a PYTHONPATH entry would otherwise shadow site-packages). Computed once
# here; appended to every kconfgen PYTHONPATH below. Empty string = no-op.
set(EH_RICH_FALLBACK_PP "" CACHE INTERNAL "rich shim PYTHONPATH suffix" FORCE)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import rich"
    RESULT_VARIABLE _eh_have_rich OUTPUT_QUIET ERROR_QUIET)
if(NOT _eh_have_rich EQUAL 0
        AND EXISTS "$ENV{EH_PATH}/tools/vendor/rich_fallback/rich/__init__.py")
    set(EH_RICH_FALLBACK_PP ":$ENV{EH_PATH}/tools/vendor/rich_fallback"
        CACHE INTERNAL "rich shim PYTHONPATH suffix" FORCE)
endif()

# prepare_kconfig_files.py lives in IDF's tools/kconfig_new/, not in
# the esp-idf-kconfig package — esp-idf-kconfig is the kconfiglib +
# kconfgen + menuconfig backend, the env-contract helper is part of
# IDF's CMake glue.  We vendor it byte-for-byte alongside this file
# (verbatim from $IDF_PATH/tools/kconfig_new/prepare_kconfig_files.py)
# so the pipeline runs without needing IDF on PATH.
set(EH_HOSTED_PREPARE_KCONFIG_FILES
    "${CMAKE_CURRENT_LIST_DIR}/prepare_kconfig_files.py")
if(NOT EXISTS "${EH_HOSTED_PREPARE_KCONFIG_FILES}")
    message(FATAL_ERROR
        "hosted_kconfig: prepare_kconfig_files.py missing at "
        "${EH_HOSTED_PREPARE_KCONFIG_FILES}.  Refresh from "
        "\$IDF_PATH/tools/kconfig_new/prepare_kconfig_files.py.")
endif()

# Root Kconfig — sources the IDF env-contract files.
if(NOT EH_HOSTED_ROOT_KCONFIG)
    set(EH_HOSTED_ROOT_KCONFIG "$ENV{EH_PATH}/Kconfig")
endif()
if(NOT EXISTS "${EH_HOSTED_ROOT_KCONFIG}")
    message(FATAL_ERROR
        "hosted_kconfig: root Kconfig not found at ${EH_HOSTED_ROOT_KCONFIG}.\n"
        "Set EH_PATH in the env so it points at the repo root.")
endif()

# Defaults are example-owned only:
#   - <project>/sdkconfig.defaults
#   - <project>/sdkconfig.defaults.<target>

# ── Internal: graph walk ──────────────────────────────────────────────
#
# Walks transitive LINK_LIBRARIES of `target`, accumulating resolved
# CMake target names.  Visits each target exactly once.  Skips raw
# library names (e.g. `pthread`) and generator expressions.
function(__hosted_kconfig_walk_target_deps target out_list)
    set(_visited "")
    set(_pending "${target}")
    while(_pending)
        list(POP_FRONT _pending _cur)
        if(_cur IN_LIST _visited)
            continue()
        endif()
        if(NOT TARGET "${_cur}")
            continue()
        endif()
        list(APPEND _visited "${_cur}")
        # Pull both PRIVATE/PUBLIC link deps and INTERFACE deps —
        # INTERFACE-only libraries (the common shape for header-only
        # ports + shims here) advertise their deps via the INTERFACE
        # property.
        foreach(_prop LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
            get_target_property(_deps "${_cur}" "${_prop}")
            if(_deps)
                foreach(_d IN LISTS _deps)
                    # Strip $<…> generator expressions; only keep
                    # plain target names that resolve to real targets.
                    string(GENEX_STRIP "${_d}" _d_stripped)
                    if(_d_stripped MATCHES "^[A-Za-z_][A-Za-z0-9_:-]*$"
                            AND TARGET "${_d_stripped}"
                            AND NOT _d_stripped IN_LIST _visited)
                        list(APPEND _pending "${_d_stripped}")
                    endif()
                endforeach()
            endif()
        endforeach()
    endwhile()
    set("${out_list}" "${_visited}" PARENT_SCOPE)
endfunction()


# ── Internal: per-target deterministic Kconfig lookup ─────────────────
#
# For each target in `targets`, look at:
#   - <SOURCE_DIR>/Kconfig
#   - <SOURCE_DIR>/Kconfig.projbuild
# plus optional per-target HOSTED_KCONFIG / HOSTED_KCONFIG_PROJBUILD
# properties (for targets whose Kconfig files don't sit alongside
# SOURCE_DIR).  No globbing, no recursion.
function(__hosted_kconfig_collect targets out_kconfigs out_projbuilds out_toplevel out_port_shims)
    set(_kconfigs "")
    set(_projbuilds "")
    set(_toplevel "")
    set(_port_shims "")
    foreach(_t IN LISTS targets)
        get_target_property(_src "${_t}" SOURCE_DIR)
        if(_src)
            set(_k "${_src}/Kconfig")
            set(_kext "${_src}/Kconfig.ext")
            set(_p "${_src}/Kconfig.projbuild")
            # Skip the root Kconfig — it's already being parsed as the
            # top-level entry point.  Sourcing it again from the
            # env-driven .in file would recurse infinitely.
            if(EXISTS "${_k}" AND NOT "${_k}" STREQUAL "${EH_HOSTED_ROOT_KCONFIG}")
                list(APPEND _kconfigs "${_k}")
            endif()
            if(EXISTS "${_kext}"
               AND NOT "${_kext}" STREQUAL "${EH_HOSTED_ROOT_KCONFIG}"
               AND NOT "${_kext}" STREQUAL "$ENV{EH_PATH}/port/Kconfig.ext")
                list(APPEND _port_shims "${_kext}")
            endif()
            if(EXISTS "${_p}")
                list(APPEND _projbuilds "${_p}")
            endif()
        endif()
        get_target_property(_k_explicit "${_t}" HOSTED_KCONFIG)
        if(_k_explicit AND NOT _k_explicit STREQUAL "_k_explicit-NOTFOUND"
                       AND EXISTS "${_k_explicit}")
            list(APPEND _kconfigs "${_k_explicit}")
        endif()
        get_target_property(_p_explicit "${_t}" HOSTED_KCONFIG_PROJBUILD)
        if(_p_explicit AND NOT _p_explicit STREQUAL "_p_explicit-NOTFOUND"
                       AND EXISTS "${_p_explicit}")
            list(APPEND _projbuilds "${_p_explicit}")
        endif()
        # Target may flag its Kconfig as top-level (menu lives outside
        # the "Component config" wrapper).  Used for vendored upstream
        # components like esp_wifi_remote whose Kconfig declares its
        # own `menu "Wi-Fi Remote"` — that menu should sit alongside
        # ESP-Hosted's "Configure host" / "Configure coprocessor", not
        # nested two levels deep under "Component config".
        get_target_property(_k_top "${_t}" HOSTED_KCONFIG_TOPLEVEL)
        if(_k_top AND NOT _k_top STREQUAL "_k_top-NOTFOUND"
                  AND EXISTS "${_k_top}")
            list(APPEND _toplevel "${_k_top}")
            list(REMOVE_ITEM _kconfigs "${_k_top}")
            list(REMOVE_ITEM _port_shims "${_k_top}")
        endif()
    endforeach()
    if(_kconfigs)
        list(REMOVE_DUPLICATES _kconfigs)
        list(SORT _kconfigs)
    endif()
    if(_toplevel)
        list(REMOVE_DUPLICATES _toplevel)
        list(SORT _toplevel)
    endif()
    if(_projbuilds)
        list(REMOVE_DUPLICATES _projbuilds)
        list(SORT _projbuilds)
    endif()
    if(_port_shims)
        list(REMOVE_DUPLICATES _port_shims)
        list(SORT _port_shims)
    endif()
    set("${out_kconfigs}" "${_kconfigs}" PARENT_SCOPE)
    set("${out_projbuilds}" "${_projbuilds}" PARENT_SCOPE)
    set("${out_toplevel}" "${_toplevel}" PARENT_SCOPE)
    set("${out_port_shims}" "${_port_shims}" PARENT_SCOPE)
endfunction()


# ── Internal: defaults precedence (per-project → per-project.target) ─
function(__hosted_kconfig_resolve_defaults project_dir hosted_target out_list)
    set(_list "")
    set(_proj "${project_dir}/sdkconfig.defaults")
    if(EXISTS "${_proj}")
        list(APPEND _list "${_proj}")
    endif()
    set(_proj_t "${project_dir}/sdkconfig.defaults.${hosted_target}")
    if(EXISTS "${_proj_t}")
        list(APPEND _list "${_proj_t}")
    endif()
    # Ordered overlays via EH_SDKCONFIG_OVERLAY (';'-separated files), applied
    # LAST so they override the example's committed defaults — the hosted analog
    # of IDF's SDKCONFIG_DEFAULTS layering (bench/suite/case). The test infra
    # assembles the layers and owns regenerating sdkconfig when they change; a
    # manual dev build just leaves the var unset (no-op).
    if(DEFINED ENV{EH_SDKCONFIG_OVERLAY})
        set(_eh_ovls "$ENV{EH_SDKCONFIG_OVERLAY}")
        foreach(_ovl IN LISTS _eh_ovls)
            if(EXISTS "${_ovl}")
                list(APPEND _list "${_ovl}")
            endif()
        endforeach()
    endif()
    set("${out_list}" "${_list}" PARENT_SCOPE)
endfunction()


# ── Internal: write the IDF-shape config.env JSON ─────────────────────
function(__hosted_kconfig_write_config_env env_path
                                            kconfigs projbuilds
                                            kconfigs_in projbuilds_in
                                            hosted_target)
    # Semicolons in CMake lists are the same separator IDF uses with
    # --list-separator=semicolon, so we pass them through verbatim.
    file(WRITE "${env_path}" "{\n")
    file(APPEND "${env_path}" "  \"COMPONENT_KCONFIGS\": \"${kconfigs}\",\n")
    file(APPEND "${env_path}" "  \"COMPONENT_KCONFIGS_PROJBUILD\": \"${projbuilds}\",\n")
    file(APPEND "${env_path}" "  \"COMPONENT_KCONFIGS_SOURCE_FILE\": \"${kconfigs_in}\",\n")
    file(APPEND "${env_path}" "  \"COMPONENT_KCONFIGS_PROJBUILD_SOURCE_FILE\": \"${projbuilds_in}\",\n")
    file(APPEND "${env_path}" "  \"IDF_TARGET\": \"${hosted_target}\",\n")
    # ESP_IDF_VERSION drives version-pinned `orsource` directives in
    # vendored Kconfig files (notably esp_wifi_remote's
    # `orsource "./Kconfig.idf_v$ESP_IDF_VERSION.in"`).  We pin to 6.1
    # because that's the IDF version our vendored snapshots align with
    # (port/esp_idf_port/esp_wifi_remote/idf_v6.1/, etc.).
    file(APPEND "${env_path}" "  \"ESP_IDF_VERSION\": \"6.1\"\n")
    file(APPEND "${env_path}" "}\n")
endfunction()


# ── Internal: register menuconfig + save-defconfig + reconfigure targets ─
function(__hosted_kconfig_register_targets config_env defaults_args sdkconfig synth_kconfig hosted_target)
    if(TARGET menuconfig)
        return()
    endif()
    set(_pythonpath "${EH_HOSTED_KCONFIG_BACKEND}")
    # Active backend (esp-idf-kconfig submodule) provides kconfgen but
    # its menuconfig has a hard Textual dep.  Prefer the legacy curses
    # menuconfig vendored at tools/vendor for the interactive TUI only —
    # kconfgen stays on the submodule.
    set(_menuconfig_pythonpath "${_pythonpath}")
    set(_legacy_menuconfig_root "$ENV{EH_PATH}/tools/vendor")
    if(EXISTS "${_legacy_menuconfig_root}/menuconfig/__main__.py"
            AND EXISTS "${_legacy_menuconfig_root}/kconfiglib/__init__.py")
        set(_menuconfig_pythonpath
            "${_legacy_menuconfig_root}:${_menuconfig_pythonpath}")
    endif()
    if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
        set(_pythonpath "${_pythonpath}:$ENV{PYTHONPATH}")
        set(_menuconfig_pythonpath "${_menuconfig_pythonpath}:$ENV{PYTHONPATH}")
    endif()
    # rich fallback (computed once at file scope; empty when real rich present).
    set(_pythonpath "${_pythonpath}${EH_RICH_FALLBACK_PP}")
    set(_menuconfig_pythonpath "${_menuconfig_pythonpath}${EH_RICH_FALLBACK_PP}")

    set(_kconfgen_base
        ${CMAKE_COMMAND} -E env "PYTHONPATH=${_pythonpath}"
        ${Python3_EXECUTABLE} -m kconfgen
        --list-separator=semicolon
        --kconfig "${synth_kconfig}"
        --config "${sdkconfig}"
        ${defaults_args}
        --env-file "${config_env}")

    set(_prepare_cmd
        ${CMAKE_COMMAND} -E env "PYTHONPATH=${_pythonpath}"
        ${Python3_EXECUTABLE} "${EH_HOSTED_PREPARE_KCONFIG_FILES}"
        --list-separator=semicolon
        --env-file "${config_env}")

    # menuconfig + save-defconfig need the same env vars as the
    # build-time kconfgen invocation: the synth Kconfig has
    # `osource "$COMPONENT_KCONFIGS_PROJBUILD_SOURCE_FILE"` and
    # `osource "$COMPONENT_KCONFIGS_SOURCE_FILE"` which silently skip
    # when the env vars are unset.  Without these, menuconfig shows
    # only "Component config" — Example Configuration / projbuild
    # menus disappear because kconfgen reads `$VAR` as empty string,
    # and `osource ""` is a no-op.  IDF passes the same env vars.
    set(_kconfgen_env
        "COMPONENT_KCONFIGS_PROJBUILD_SOURCE_FILE=${CMAKE_BINARY_DIR}/config/kconfigs_projbuild.in"
        "COMPONENT_KCONFIGS_SOURCE_FILE=${CMAKE_BINARY_DIR}/config/kconfigs.in"
        "EH_PORT_SHIM_KCONFIGS_DIR=${CMAKE_BINARY_DIR}/config"
        "IDF_TARGET=${hosted_target}"
        "ESP_IDF_VERSION=6.1")

    add_custom_target(menuconfig
        COMMAND ${_prepare_cmd}
        COMMAND ${CMAKE_COMMAND} -E env
                "PYTHONPATH=${_menuconfig_pythonpath}"
                "KCONFIG_CONFIG=${sdkconfig}"
                ${_kconfgen_env}
                ${Python3_EXECUTABLE} -m menuconfig "${synth_kconfig}"
        COMMAND ${CMAKE_COMMAND} -E env
                "PYTHONPATH=${_pythonpath}"
                ${_kconfgen_env}
                ${Python3_EXECUTABLE} -m kconfgen
                --list-separator=semicolon
                --kconfig "${synth_kconfig}"
                --config "${sdkconfig}"
                ${defaults_args}
                --env-file "${config_env}"
                --output config "${sdkconfig}"
                --output header "${CMAKE_BINARY_DIR}/config/sdkconfig.h"
                --output cmake "${CMAKE_BINARY_DIR}/config/sdkconfig.cmake"
                --output json "${CMAKE_BINARY_DIR}/config/sdkconfig.json"
                --output json_menus "${CMAKE_BINARY_DIR}/config/kconfig_menus.json"
        USES_TERMINAL
        VERBATIM)

    add_custom_target(save-defconfig
        COMMAND ${_prepare_cmd}
        COMMAND ${CMAKE_COMMAND} -E env
                "PYTHONPATH=${_pythonpath}"
                ${_kconfgen_env}
                ${Python3_EXECUTABLE} -m kconfgen
                --list-separator=semicolon
                --kconfig "${synth_kconfig}"
                --config "${sdkconfig}"
                ${defaults_args}
                --env-file "${config_env}"
                --dont-write-deprecated
                --output savedefconfig "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.defaults"
        USES_TERMINAL
        VERBATIM)
endfunction()


# ── Internal: read the persisted hosted target (or default linux). ──
function(__hosted_kconfig_resolve_target out_var)
    set(_target_file "${CMAKE_SOURCE_DIR}/.eh.target")
    if(EXISTS "${_target_file}")
        file(READ "${_target_file}" _t)
        string(STRIP "${_t}" _t)
        if(_t STREQUAL "posix")
            set(_t "linux")
        endif()
    else()
        set(_t "linux")
    endif()
    set("${out_var}" "${_t}" PARENT_SCOPE)
endfunction()


# ── Internal: write the synthesised top-level Kconfig wrapper. ───────
# Keeps the repo-root Kconfig clean of `$COMPONENT_KCONFIGS_*_SOURCE_FILE`
# synth wraps the real root Kconfig and adds the IDF-style env-driven
# source hooks at the positions IDF uses (projbuild near top,
# components inside "Component config" menu).
function(__hosted_kconfig_write_synth synth_kconfig toplevel_kconfigs)
    # Layout mirrors IDF's idf.py:
    #
    #   mainmenu
    #     Example Configuration         (main/Kconfig.projbuild)
    #     Component config
    #       ESP-Hosted                  (root Kconfig)
    #       Wi-Fi Remote                (vendored, top-level marked)
    #       ESP NETIF Adapter           (per-component Kconfig)
    #       …
    #
    # Inside "Component config", top-level vendored Kconfigs come
    # BEFORE the root Kconfig so kconfgen sees SLAVE_IDF_TARGET et al
    # before our `depends on SLAVE_IDF_TARGET_xxx` references evaluate
    # (forward-resolution timing — see L29 in
    # `.meta2/knowledge/lessons.md` if added).
    file(WRITE "${synth_kconfig}"
        "# hosted_kconfig.cmake generated — do not edit by hand.\n"
        "mainmenu \"ESP-Hosted (Linux)\"\n"
        "\n"
        "    # Per-example Kconfig.projbuild surface — sits at the top\n"
        "    # of menuconfig, outside \"Component config\".\n"
        "    osource \"\$COMPONENT_KCONFIGS_PROJBUILD_SOURCE_FILE\"\n"
        "\n"
        "    menu \"Component config\"\n")
    foreach(_top IN LISTS toplevel_kconfigs)
        file(APPEND "${synth_kconfig}"
            "        orsource \"${_top}\"\n")
    endforeach()
    file(APPEND "${synth_kconfig}"
        "        orsource \"${EH_HOSTED_ROOT_KCONFIG}\"\n"
        "        osource \"\$COMPONENT_KCONFIGS_SOURCE_FILE\"\n"
        "    endmenu\n")
endfunction()


# ── Internal: one kconfgen run.
#
# Both passes share this — first pass at project()-macro time emits
# defaults-only sdkconfig (no graph yet), second pass at DEFER
# finalize re-emits with the full graph's Kconfig + Kconfig.projbuild
# surface picked up.  Identical output locations both times; second
# run overwrites first.  CMAKE_CONFIGURE_DEPENDS picks up the change
# and re-runs configure once if the second pass produced different
# CONFIG_* values.
function(__hosted_kconfig_run_one kconfigs projbuilds defaults hosted_target write_header toplevel_kconfigs)
    set(_config_dir "${CMAKE_BINARY_DIR}/config")
    set(_kconfigs_in "${_config_dir}/kconfigs.in")
    set(_projbuilds_in "${_config_dir}/kconfigs_projbuild.in")
    set(_config_env "${_config_dir}/config.env")
    set(_sdkconfig "${CMAKE_SOURCE_DIR}/sdkconfig")
    set(_sdkconfig_h "${_config_dir}/sdkconfig.h")
    set(_sdkconfig_cmake "${_config_dir}/sdkconfig.cmake")
    set(_sdkconfig_json "${_config_dir}/sdkconfig.json")
    set(_sdkconfig_json_menus "${_config_dir}/kconfig_menus.json")
    set(_synth_kconfig "${_config_dir}/Kconfig")
    file(MAKE_DIRECTORY "${_config_dir}")

    __hosted_kconfig_write_synth("${_synth_kconfig}" "${toplevel_kconfigs}")
    __hosted_kconfig_write_config_env("${_config_env}"
                                      "${kconfigs}" "${projbuilds}"
                                      "${_kconfigs_in}" "${_projbuilds_in}"
                                      "${hosted_target}")

    set(_defaults_args "")
    foreach(_d IN LISTS defaults)
        list(APPEND _defaults_args --defaults "${_d}")
    endforeach()

    set(_pythonpath "${EH_HOSTED_KCONFIG_BACKEND}")
    if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
        set(_pythonpath "${_pythonpath}:$ENV{PYTHONPATH}")
    endif()
    set(_pythonpath "${_pythonpath}${EH_RICH_FALLBACK_PP}")  # rich shim if needed
    set(_env_wrap ${CMAKE_COMMAND} -E env "PYTHONPATH=${_pythonpath}")

    execute_process(
        COMMAND ${_env_wrap}
                ${Python3_EXECUTABLE} "${EH_HOSTED_PREPARE_KCONFIG_FILES}"
                --list-separator=semicolon
                --env-file "${_config_env}"
        RESULT_VARIABLE _prep_rc
        OUTPUT_VARIABLE _prep_out
        ERROR_VARIABLE  _prep_err)
    if(_prep_rc)
        message(FATAL_ERROR
            "hosted_kconfig: prepare_kconfig_files.py failed (rc=${_prep_rc})\n"
            "stdout:\n${_prep_out}\n"
            "stderr:\n${_prep_err}")
    endif()

    # Pass 1 (defaults-only) skips the C header entirely — only
    # sdkconfig.cmake matters for the CMake walk.  Pass 2 writes the
    # header to a `.new` file and copy-if-differents it onto the
    # consumer-visible sdkconfig.h, so the header's mtime only ticks
    # forward when CONFIG_* values actually change.  Without that
    # trick, every `hosted build` re-emits sdkconfig.h (fresh mtime,
    # identical content) and forces a full rebuild of every TU that
    # `-include`s it.
    set(_header_args "")
    if(write_header)
        set(_header_args --output header "${_sdkconfig_h}.new")
    endif()

    # Only the final pass writes sdkconfig — pass 1 has a minimal synth
    # (no per-component projbuilds) and would filter out symbols it
    # doesn't know about, clobbering user menuconfig edits.
    set(_config_args "")
    if(write_header)
        set(_config_args --output config "${_sdkconfig}")
    endif()

    execute_process(
        COMMAND ${_env_wrap}
                ${Python3_EXECUTABLE} -m kconfgen
                --list-separator=semicolon
                --kconfig "${_synth_kconfig}"
                --config "${_sdkconfig}"
                ${_defaults_args}
                --env-file "${_config_env}"
                ${_config_args}
                ${_header_args}
                --output cmake "${_sdkconfig_cmake}"
                --output json "${_sdkconfig_json}"
                --output json_menus "${_sdkconfig_json_menus}"
        RESULT_VARIABLE _kc_rc
        OUTPUT_VARIABLE _kc_out
        ERROR_VARIABLE  _kc_err)
    if(_kc_rc)
        message(FATAL_ERROR
            "hosted_kconfig: kconfgen failed (rc=${_kc_rc})\n"
            "stdout:\n${_kc_out}\n"
            "stderr:\n${_kc_err}")
    endif()

    if(write_header)
        configure_file("${_sdkconfig_h}.new" "${_sdkconfig_h}" COPYONLY)
        file(REMOVE "${_sdkconfig_h}.new")
    endif()

    # Reconfigure whenever sdkconfig or any defaults file changes.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${_sdkconfig}" ${defaults})
endfunction()


# ── Public entry point — first pass.
#
# Run BEFORE add_subdirectory(${EH_PATH}) by hosted_project.cmake.
# Emits sdkconfig.cmake + sdkconfig.h with defaults-only (empty
# COMPONENT_KCONFIGS / COMPONENT_KCONFIGS_PROJBUILD), so the repo
# root's `include(sdkconfig.cmake)` succeeds and the host stack's
# `if(CONFIG_*)` gates evaluate against real CMake variables.
function(hosted_kconfig_emit_defaults)
    __hosted_kconfig_resolve_target(_hosted_target)
    __hosted_kconfig_resolve_defaults("${CMAKE_SOURCE_DIR}" "${_hosted_target}"
                                      _defaults)
    __hosted_kconfig_run_one("" "" "${_defaults}" "${_hosted_target}" FALSE
                             "")
endfunction()


# ── Public entry point — second pass.
#
# Run at end-of-configure (DEFER finalize) once all add_subdirectory's
# have declared targets.  Walks the executable/library target's
# transitive LINK_LIBRARIES, collects Kconfig + Kconfig.projbuild from
# each resolved target's SOURCE_DIR, re-emits sdkconfig with the
# full graph's surface.
#
# `-include sdkconfig.h` and the build/config include dir are NOT
# attached per-target here — that propagation is owned by the
# `eh_sdkconfig` INTERFACE target in repo-root CMakeLists.txt, which
# `esp_hosted` PUBLIC-links so every consumer picks them up
# automatically through normal CMake target deps.
function(hosted_kconfig_apply app_target)
    if(NOT TARGET ${app_target})
        message(FATAL_ERROR
            "hosted_kconfig_apply: '${app_target}' is not a target. "
            "Call hosted_kconfig_apply(<name>) AFTER add_executable + "
            "target_link_libraries.")
    endif()

    __hosted_kconfig_resolve_target(_hosted_target)
    __hosted_kconfig_walk_target_deps(${app_target} _graph)
    __hosted_kconfig_collect("${_graph}" _kconfigs _projbuilds _toplevel _port_shims)
    __hosted_kconfig_resolve_defaults("${CMAKE_SOURCE_DIR}" "${_hosted_target}"
                                      _defaults)

    set(_port_shims_dir "${CMAKE_BINARY_DIR}/config")
    set(_port_shims_in "${_port_shims_dir}/port_shims.in")
    file(WRITE "${_port_shims_in}" "")
    foreach(_s IN LISTS _port_shims)
        file(APPEND "${_port_shims_in}" "orsource \"${_s}\"\n")
    endforeach()
    set(ENV{EH_PORT_SHIM_KCONFIGS_DIR} "${_port_shims_dir}")

    __hosted_kconfig_run_one("${_kconfigs}" "${_projbuilds}"
                             "${_defaults}" "${_hosted_target}" TRUE
                             "${_toplevel}")

    # Re-include sdkconfig.cmake so any updated CONFIG_* values are
    # visible in CMake.  (First-pass already included it before the
    # host stack add_subdirectory; this re-include reflects the
    # second-pass output.)
    include("${CMAKE_BINARY_DIR}/config/sdkconfig.cmake")

    # Build the --defaults arg list for the menuconfig / save-defconfig
    # custom targets.
    set(_defaults_args "")
    foreach(_d IN LISTS _defaults)
        list(APPEND _defaults_args --defaults "${_d}")
    endforeach()

    # Register menuconfig + save-defconfig CMake targets (idempotent).
    __hosted_kconfig_register_targets(
        "${CMAKE_BINARY_DIR}/config/config.env"
        "${_defaults_args}"
        "${CMAKE_SOURCE_DIR}/sdkconfig"
        "${CMAKE_BINARY_DIR}/config/Kconfig"
        "${_hosted_target}")
endfunction()
