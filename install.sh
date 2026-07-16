#!/usr/bin/env bash
# esp_hosted dependency installer (bash/zsh) — thin wrapper over `eh.py install`.
# Fetches + builds ESP-IDF and esp-emu into .deps/ (interactive; -y to skip
# the prompt; --uninstall to remove .deps/; --help for options).
# After it finishes: `. ./export.sh`.
set -euo pipefail
_here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
exec python3 "${_here}/tools/eh.py" install "$@"
