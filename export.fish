# SPDX-License-Identifier: Apache-2.0
# esp_hosted environment for fish.
#
# Source this from the repo root:
#     . ./export.fish
#
# After that, BOTH build entry points work from anywhere:
#   - idf.py        for ESP-IDF projects under cp/, mcu_host/, ...
#                   (sourced from $IDF_PATH/export.fish; CP work needs this)
#                   linux_802_3_host/c_app/
#
# Idempotent — re-sourcing does not duplicate PATH entries.

# ── Resolve repo root (this file's dir). ─────────────────────────
set -l _eh_self (status --current-filename)
set -l _eh_repo (path resolve (dirname $_eh_self))

# ── ESP-IDF env (idf.py + IDF tools). ────────────────────────────
# IDF is required for CP and IDF-host (mcu/) builds; not for pure Linux-host
# builds. eh.py is the single source of truth for WHICH IDF to use (eh.conf
# override -> .deps -> $IDF_PATH). Take its answer verbatim — no hardcoded
# paths — even when IDF_PATH is already set, so `. ./export.fish` after an
# install activates the IDF the repo resolved, not an unrelated one in the env.
set -l _eh_idf (python3 $_eh_repo/tools/eh.py get-idf-path --raw 2>/dev/null)
if test -n "$_eh_idf"; and test -f "$_eh_idf/export.fish"
    set -gx IDF_PATH $_eh_idf
end

if set -q IDF_PATH; and test -n "$IDF_PATH"; and test -f $IDF_PATH/export.fish
    # IDF's export.fish chatters; quiet it for the common-case sourcing.
    source $IDF_PATH/export.fish >/dev/null
    echo "esp_hosted env: ESP-IDF ($IDF_PATH) sourced — idf.py on PATH"
else
    echo "esp_hosted env: ESP-IDF not found (set IDF_PATH if you need idf.py;"
    echo "                Linux-only host builds work without it)"
end

if not contains -- $_eh_repo/tools $PATH
    # APPEND (not prepend): tools/ has eh.py (unique) but also a cmake/ subdir.
    # Prepending makes `cmake` resolve to that dir and shadow the real cmake
    # binary. Appending keeps system cmake working while eh.py stays on PATH.
    set -gx PATH $PATH $_eh_repo/tools
end

echo "esp_hosted env: eh.py on PATH ($_eh_repo/tools)"
