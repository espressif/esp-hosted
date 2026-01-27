#!/usr/bin/env bash
set -e

# Absolute path to this script's folder
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Enforce superuser
if [ "$EUID" -ne 0 ]; then
    echo "❌ This script must be run as superuser (sudo)"
    echo "Usage: sudo $0"
    exit 1
fi

# Check that venv exists
VENV_PY="$SCRIPT_DIR/hosted_venv/bin/python"
if [ ! -f "$VENV_PY" ]; then
    echo "❌ Virtual environment not found. Please run setup_python.sh first."
    exit 1
fi

# Run test.py inside the venv
"$VENV_PY" "$SCRIPT_DIR/test.py"

