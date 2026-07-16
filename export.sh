# SPDX-License-Identifier: Apache-2.0
# esp_hosted environment for bash / zsh.
#
# Source this from the repo root:
#     . ./export.sh
#
# After that, BOTH build entry points work from anywhere:
#   - idf.py        for ESP-IDF projects under cp/, mcu_host/, ...
#                   (sourced from $IDF_PATH/export.sh; CP work needs this)
#                   linux_802_3_host/c_app/
#
# Idempotent — re-sourcing does not duplicate PATH entries.

# ── Resolve repo root (this file's dir). ─────────────────────────
if [ -n "${BASH_SOURCE[0]:-}" ]; then
    _eh_self="${BASH_SOURCE[0]}"
elif [ -n "${(%):-%x}" ]; then
    _eh_self="${(%):-%x}"
else
    _eh_self="$0"
fi
_eh_repo="$(cd "$(dirname "${_eh_self}")" && pwd)"
unset _eh_self

# ── EH_PATH — analog of IDF's $IDF_PATH.  Every Linux
# example's CMakeLists.txt does
#     include($ENV{EH_PATH}/tools/cmake/hosted_project.cmake)
export EH_PATH="${_eh_repo}"

# ── ESP-IDF env (idf.py + IDF tools). ────────────────────────────
# IDF is required for CP and IDF-host (mcu/) builds; not for pure Linux-host
# builds. eh.py is the single source of truth for WHICH IDF to use (eh.conf
# override -> .deps -> $IDF_PATH). Take its answer verbatim — no hardcoded
# paths — and do so even when IDF_PATH is already set, so `. ./export.sh` after
# an install activates the IDF the repo resolved, not an unrelated one left in
# the environment. If eh.py can't resolve one, leave IDF_PATH as-is.
_eh_idf="$(python3 "${_eh_repo}/tools/eh.py" get-idf-path --raw 2>/dev/null || true)"
if [ -n "${_eh_idf}" ] && [ -f "${_eh_idf}/export.sh" ]; then
    IDF_PATH="${_eh_idf}"
fi
unset _eh_idf

if [ -n "${IDF_PATH:-}" ] && [ -f "${IDF_PATH}/export.sh" ]; then
    # IDF's export.sh chatters; quiet it for the common-case sourcing.
    . "${IDF_PATH}/export.sh" >/dev/null
    echo "esp_hosted env: ESP-IDF (${IDF_PATH}) sourced — idf.py on PATH"
else
    echo "esp_hosted env: ESP-IDF not found (set IDF_PATH if you need idf.py;"
    echo "                Linux-only host builds work without it)"
fi

case ":${PATH}:" in
    *":${_eh_repo}/tools:"*)
        ;;
    *)
        # APPEND (not prepend): tools/ holds eh.py (unique, still found), but it
        # also contains a `cmake/` subdir. Prepending would make `cmake` resolve
        # to that directory and shadow the real cmake binary ("Permission
        # denied"). Appending lets the system cmake win while eh.py still works.
        export PATH="${PATH}:${_eh_repo}/tools"
        ;;
esac

# ── PYTHONPATH for build tooling only.  This is the kconfgen /
# menuconfig backend invoked by hosted_kconfig.cmake — same role
# IDF's export.sh plays when it adds $IDF_PATH/tools/* entries to
# PYTHONPATH for idf.py's own machinery.
#
# Application-side packages (e.g. the `eh_host` Python ctypes
# binding) are NOT declared here — that would be export.sh knowing
# about specific component paths, which violates the agnostic-
# build-tooling rule.  Users of those packages either pip-install
# them or set PYTHONPATH themselves; the build system stays out.
case ":${PYTHONPATH:-}:" in
    *":${_eh_repo}/tools/vendor/esp-idf-kconfig:"*) ;;
    *) export PYTHONPATH="${_eh_repo}/tools/vendor/esp-idf-kconfig${PYTHONPATH:+:${PYTHONPATH}}" ;;
esac

echo "esp_hosted env: eh.py on PATH (${_eh_repo}/tools)"
echo "esp_hosted env: EH_PATH=${EH_PATH}"
unset _eh_repo
