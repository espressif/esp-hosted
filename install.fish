#!/usr/bin/env fish
# esp_hosted dependency installer (fish) — thin wrapper over `eh.py install`.
# Fetches + builds ESP-IDF and esp-emu into .deps/ (interactive; -y to skip
# the prompt; --uninstall to remove .deps/; --help for options).
# After it finishes: `. ./export.fish`.
set -l _here (cd (dirname (status --current-filename)); and pwd)
exec python3 "$_here/tools/eh.py" install $argv
