"""
ESP-Hosted Hardware Test Infrastructure — conftest.py

Root conftest for all hw test subdirectories.
Sets up sys.path so test files can import from infra/.

Convention:
  dut[0] = Host (ESP32-P4)
  dut[1] = CP/Slave (ESP32-C6)
"""

import os
import sys

# Add tests/ to sys.path so all test files can do "from infra.xxx import ..."
_tests_dir = os.path.join(os.path.dirname(__file__), '..')
if _tests_dir not in sys.path:
    sys.path.insert(0, _tests_dir)
