#!/usr/bin/env bash


# Install python3
sudo apt install python3

# Python virtual environment steps are explained in documentation at
# https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/docs/common/python_demo.md#demo-app-in-python
# Current setup assumes python venv is not needed.
sudo python3 -m pip install prompt_toolkit fire argparse docstring_parser requests
