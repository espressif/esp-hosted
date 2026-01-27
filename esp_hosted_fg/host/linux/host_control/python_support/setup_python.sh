#!/usr/bin/env bash
set -e

# Install system Python and venv support
sudo apt update
sudo apt install -y python3 python3-venv python3-pip

# Create virtual environment if it doesn't exist
if [ ! -d "hosted_venv" ]; then
    python3 -m venv hosted_venv
fi

# Activate venv
source hosted_venv/bin/activate

# Upgrade pip inside venv
pip install --upgrade pip

# Install Python dependencies
pip install -r requirements.txt

echo
echo "✅Python environment ready"
echo "Activate it with:"
echo "  source hosted_venv/bin/activate"

